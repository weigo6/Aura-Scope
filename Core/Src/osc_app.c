#include "osc_app.h"
#include "adc.h"
#include "tim.h"
#include "st7735.h"
#include "signal_gen.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
// 全局变量定义
uint32_t osc_adc_buffer[OSC_ADC_NUM];
volatile uint8_t osc_adc_cplt_flag = 0;
volatile uint8_t osc_adc_half_cplt_flag = 0; // 新增半传输标志

static OscState_t osc_state = OSC_RUN;
static OscMode_t osc_mode = OSC_MODE_CH1;
static uint8_t attenuation_idx = 0; // 0: x1, 1: x10, 2: x50, 3: x100
static const uint8_t attenuation_values[] = {1, 10, 50, 100};
static uint8_t cdc_enabled = 0;

// 底部信息显示模式 (0: Vpp, 1: Freq)
static uint8_t osc_info_mode = 0;

// 绘图条带缓冲区 (1列 x (OSC_WAVE_HEIGHT-2)高，避开上下边框)
static uint16_t wave_strip_buffer[OSC_WAVE_HEIGHT - 2];

// 频率测量相关变量
static volatile uint32_t tim3_overflow_count = 0;
static volatile uint32_t tim2_overflow_count = 0;
static volatile uint32_t last_capture_ch1 = 0;
static volatile uint32_t last_overflow_ch1 = 0; // Snapshot of overflow at capture
static volatile uint32_t last_capture_ch2 = 0;
static volatile uint32_t last_overflow_ch2 = 0; // Snapshot of overflow at capture
static volatile uint32_t freq_ch1 = 0;
static volatile uint32_t freq_ch2 = 0;
static volatile uint8_t  first_capture_ch1 = 1;
static volatile uint8_t  first_capture_ch2 = 1;

// Timebase Config
typedef struct {
    uint16_t arr;
    uint8_t step;
    const char* name;
} TimebaseConfig_t;

// 72MHz Clock
// 500kSps: 143
// 200kSps: 359
// 100kSps: 719
// 50kSps:  1439
// 20kSps:  3599
// 10kSps:  7199
static TimebaseConfig_t timebase_configs[] = {
    {143, 1, "500k"},
    {359, 1, "200k"},
    {719, 1, "100k"},
    {1439, 1, "50k"},
    {3599, 1, "20k"},
    {7199, 1, "10k"},
    {7199, 2, "5k"},  // 10kSps / 2
    {7199, 5, "2k"},  // 10kSps / 5
    {7199, 10, "1k"}  // 10kSps / 10
};
#define TIMEBASE_NUM (sizeof(timebase_configs)/sizeof(TimebaseConfig_t))
static int8_t timebase_idx = 2; // Default 100k

static uint16_t newWaveCH1[OSC_WAVE_DRAW_WIDTH];
static uint16_t newWaveCH2[OSC_WAVE_DRAW_WIDTH];

static float vpp_ch1 = 0.0f;
static float vpp_ch2 = 0.0f;

// 外部变量 (由main.c维护)
extern TIM_HandleTypeDef htim1;

// 触发阈值 (ADC值)
#define TRIG_THRESHOLD 2048
#define TRIG_HYSTERESIS 40  // 滞回区间，约 3.3V/4096 * 40 = 0.03V

static void UpdateTimebaseHardware(void) {
    uint16_t arr = timebase_configs[timebase_idx].arr;
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
}

void OSC_Init(void) {
    // 启动信号发生器
    SignalGen_Init();
    // Apply default timebase
    UpdateTimebaseHardware();

    // Sync LED1 (USB) state with cdc_enabled
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, cdc_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void OSC_Start(void) {
    // 启动前先进行校准，确保测量精度
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    // 启动ADC DMA (双重模式)
    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, osc_adc_buffer, OSC_ADC_NUM) != HAL_OK) {
        Error_Handler();
    }
    // 启动定时器触发
    HAL_TIM_Base_Start(&htim1);
}

// ADC转换完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        osc_adc_cplt_flag = 1;
    }
}

// ADC半传输回调
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        osc_adc_half_cplt_flag = 1;
    }
}

// ADC值转Y坐标 (根据公式 Vin = 50 * [5 - 2 * V_adc])
// ADC=0 -> Vin=250V (High) -> Top
// ADC=4095 -> Vin=-80V (Low) -> Bottom
// 形状和方向与 x1 模式一致，只是数值比例不同
static uint16_t ADC2Y(uint16_t adc_val) {
    // 限制波形在 [OSC_WAVE_TOP_Y + 1, OSC_WAVE_BOTTOM_Y - 1] 之间，防止破坏上下边框
    uint16_t draw_height = OSC_WAVE_HEIGHT - 2; 
    uint16_t y = (adc_val * draw_height) / 4096;
    return OSC_WAVE_TOP_Y + 1 + y;
}

static void OSC_CDC_SendBlock(uint32_t* buffer, uint32_t len) {
    if (!cdc_enabled) return;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
    uint16_t data_bytes = (uint16_t)(len * 4);
    uint8_t header[5];
    uint8_t* data_ptr = (uint8_t*)buffer;
    uint32_t sent = 0;
    uint32_t retry;

    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = (uint8_t)timebase_idx;
    header[3] = (uint8_t)(data_bytes & 0xFF);
    header[4] = (uint8_t)((data_bytes >> 8) & 0xFF);

    retry = 0;
    while (CDC_Transmit_FS(header, sizeof(header)) == USBD_BUSY) {
        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
        if (++retry > 1000) return;
    }

    while (sent < data_bytes) {
        uint16_t chunk = (uint16_t)((data_bytes - sent) > APP_TX_DATA_SIZE ? APP_TX_DATA_SIZE : (data_bytes - sent));
        retry = 0;
        while (CDC_Transmit_FS(data_ptr + sent, chunk) == USBD_BUSY) {
            if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
            if (++retry > 1000) return;
        }
        sent += chunk;
    }
}

void OSC_Process(void) {
    uint8_t process_active_block = 0; // 0: None, 1: Half(First 1000), 2: Full(Second 1000)

    // 1. 响应 DMA 标志并分块处理
    if (osc_state == OSC_RUN) {
        if (osc_adc_half_cplt_flag) {
            process_active_block = 1;
            osc_adc_half_cplt_flag = 0;
        } 
        else if (osc_adc_cplt_flag) {
            process_active_block = 2;
            osc_adc_cplt_flag = 0;
        }
    } else {
        osc_adc_half_cplt_flag = 0;
        osc_adc_cplt_flag = 0;
    }

    // 2. 核心处理逻辑 (优先级 1：合并扫描 - 快照与 Vpp 同时计算)
    if (process_active_block != 0) {
        uint8_t step = timebase_configs[timebase_idx].step;
        uint32_t block_start_idx = (process_active_block == 1) ? 0 : (OSC_ADC_NUM / 2);
        uint32_t block_len = OSC_ADC_NUM / 2;
        
        uint32_t min1 = 0xFFF, max1 = 0, min2 = 0xFFF, max2 = 0;
        uint16_t trigger_pos = block_start_idx; 
        uint8_t trigger_found = 0;
        
        uint32_t required_points = OSC_WAVE_WIDTH * step;
        int32_t search_limit = (int32_t)block_len - (int32_t)required_points - 10;

        for(int i = 0; i < block_len; i++) {
            uint32_t idx = block_start_idx + i;
            uint32_t raw = osc_adc_buffer[idx];
            uint16_t val1 = raw & 0xFFFF;
            uint16_t val2 = (raw >> 16) & 0xFFFF;
            
            // --- 任务 1：Vpp 极值查找 ---
            if (val1 < min1) min1 = val1;
            if (val1 > max1) max1 = val1;
            if (val2 < min2) min2 = val2;
            if (val2 > max2) max2 = val2;

            // --- 任务 2：触发查找 ---
            if (!trigger_found && i > 0 && i < search_limit) {
                uint16_t val_prev = osc_adc_buffer[idx-1] & 0xFFFF;
                if (val_prev > (TRIG_THRESHOLD + TRIG_HYSTERESIS) && val1 <= TRIG_THRESHOLD) {
                    trigger_pos = idx;
                    trigger_found = 1;
                }
            }
        }
        
        // 最终转换 Vpp
        float factor = 2.0f * attenuation_values[attenuation_idx];
        float conv = 3.3f / 4095.0f * factor;
        vpp_ch1 = (max1 - min1) * conv;
        vpp_ch2 = (max2 - min2) * conv;

        // 3. 准备波形快照
        for (int i = 0; i < OSC_WAVE_DRAW_WIDTH; i++) {
            uint32_t idx = trigger_pos + i * step;
            if (idx >= OSC_ADC_NUM) idx = OSC_ADC_NUM - 1;
            
            uint32_t raw = osc_adc_buffer[idx];
            newWaveCH1[i] = ADC2Y(raw & 0xFFFF);
            newWaveCH2[i] = ADC2Y((raw >> 16) & 0xFFFF);
        }

        // 4. USB 数据回传 (流式)
        if (cdc_enabled) {
            OSC_CDC_SendBlock(&osc_adc_buffer[block_start_idx], block_len);
        }
    }
}

void OSC_DrawWaveform(void) {
    if (osc_state == OSC_RUN) {
        // 使用“垂直条带刷新”方案配合 DMA
        // 每一列建立一个小的缓冲区，计算好像素后一次性发给 DMA
        // 垂直范围限制在 [TOP+1, BOTTOM-1]，避开上下边框线
        const uint16_t strip_top = OSC_WAVE_TOP_Y + 1;
        const uint16_t strip_bottom = OSC_WAVE_BOTTOM_Y - 1;
        const uint16_t strip_height = strip_bottom - strip_top;

        for (int x = 0; x < OSC_WAVE_DRAW_WIDTH; x++) {
            // 1. 严格限制 x 坐标，避开左右边框 (边框在 x=0 和 x=159)
            uint16_t screen_x = x + 1;
            if (screen_x >= ST7735_WIDTH - 1) break;

            // 2. 等待上一次 DMA 传输完成
            while (ST7735_IsBusy());

            // 3. 准备这一列的条带数据 (预填背景网格)
            for (int h = 0; h < strip_height; h++) {
                uint16_t screen_y = h + strip_top;
                uint16_t color = 0x0000; // 默认黑色

                // --- 背景网格逻辑 (同步 ui.c) ---
                // 1. 中心十字轴 (实线 0x3186 -> 大端 86 31)
                if (screen_x == ST7735_WIDTH / 2 || screen_y == OSC_WAVE_TOP_Y + OSC_WAVE_HEIGHT / 2) {
                    color = 0x8631;
                }
                // 2. 垂直虚线网格 (每 20px, 4px周期)
                else if (screen_x % 20 == 0 && screen_y % 4 == 0) {
                    color = 0x0421; // 0x2104 -> 大端 04 21
                }
                // 3. 水平虚线网格 (每 18px, 4px周期)
                else if ((screen_y - OSC_WAVE_TOP_Y) % 18 == 0 && screen_x % 4 == 0) {
                    color = 0x0421;
                }

                wave_strip_buffer[h] = color;
            }

            // 3. 在条带中描绘波形点 (将 Y 坐标映射到条带内的相对位置)
            // 描绘 CH1 (绿色 0x07E0 -> 大端 E0 07)
            if (osc_mode == OSC_MODE_CH1 || osc_mode == OSC_MODE_DUAL) {
                int y1 = newWaveCH1[x] - strip_top;
                if (y1 >= 0 && y1 < strip_height) {
                    wave_strip_buffer[y1] = 0xE007; // 绿色 (大端)
                }
                // 连线处理
                if (x > 0) {
                    int y_prev = newWaveCH1[x-1] - strip_top;
                    int y_min = (y1 < y_prev) ? y1 : y_prev;
                    int y_max = (y1 > y_prev) ? y1 : y_prev;
                    for (int y = y_min; y <= y_max; y++) {
                        if (y >= 0 && y < strip_height) wave_strip_buffer[y] = 0xE007;
                    }
                }
            }

            // 描绘 CH2 (黄色 0xFFE0 -> 大端 E0 FF)
            if (osc_mode == OSC_MODE_CH2 || osc_mode == OSC_MODE_DUAL) {
                int y2 = newWaveCH2[x] - strip_top;
                if (y2 >= 0 && y2 < strip_height) {
                    wave_strip_buffer[y2] = 0xE0FF; // 黄色 (大端)
                }
                // 连线处理
                if (x > 0) {
                    int y_prev = newWaveCH2[x-1] - strip_top;
                    int y_min = (y2 < y_prev) ? y2 : y_prev;
                    int y_max = (y2 > y_prev) ? y2 : y_prev;
                    for (int y = y_min; y <= y_max; y++) {
                        if (y >= 0 && y < strip_height) wave_strip_buffer[y] = 0xE0FF;
                    }
                }
            }

            // 4. 发送条带到屏幕对应列 (仅刷新内部区域)
            ST7735_SetWindow(screen_x, strip_top, screen_x, strip_bottom - 1);
            ST7735_WriteDataBufferDMA(wave_strip_buffer, strip_height);
        }
    }
}

void OSC_ChangeTimebase(int8_t direction) {
    if (direction > 0) {
        if (timebase_idx < TIMEBASE_NUM - 1) timebase_idx++; // Slower (Zoom Out)
    } else {
        if (timebase_idx > 0) timebase_idx--; // Faster (Zoom In)
    }
    UpdateTimebaseHardware();
}

void OSC_TogglePause(void) {
    osc_state = (osc_state == OSC_RUN) ? OSC_PAUSE : OSC_RUN;
}

void OSC_ToggleAttenuation(void) {
    attenuation_idx = (attenuation_idx + 1) % 4;
}

void OSC_CycleChannelMode(void) {
    osc_mode = (OscMode_t)((osc_mode + 1) % OSC_MODE_COUNT);
}

void OSC_ToggleInfoMode(void) {
    osc_info_mode = !osc_info_mode;
}

void OSC_ToggleCDC(void) {
    cdc_enabled = !cdc_enabled;
    // Update LED1 (USB)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, cdc_enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t OSC_GetCDCState(void) {
    return cdc_enabled;
}

uint8_t OSC_GetStep(void) {
    return timebase_configs[timebase_idx].step;
}

const char* OSC_GetTimebaseName(void) {
    return timebase_configs[timebase_idx].name;
}

OscState_t OSC_GetState(void) {
    return osc_state;
}

OscMode_t OSC_GetChannelMode(void) {
    return osc_mode;
}

uint8_t OSC_GetAttenuation(void) {
    return attenuation_values[attenuation_idx];
}

uint8_t OSC_GetInfoMode(void) {
    return osc_info_mode;
}

float OSC_GetVpp(uint8_t channel) {
    if (channel == 0) return vpp_ch1;
    return vpp_ch2;
}

uint32_t OSC_GetFrequency(uint8_t channel) {
    if (channel == 0) return freq_ch1;
    return freq_ch2;
}


// 中断回调函数
void OSC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        tim3_overflow_count++;
    }
    else if (htim->Instance == TIM2) {
        tim2_overflow_count++;
    }
}

void OSC_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // CH1 Frequency (PA6)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            uint32_t CCR = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t overflow = tim3_overflow_count;
            
            // 处理溢出中断竞争：如果捕获时Update标志已置位但中断未处理 (CCR很小)
            if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) && (CCR < 0x8000)) {
                overflow++;
            }

            if (first_capture_ch1) {
                last_capture_ch1 = CCR;
                last_overflow_ch1 = overflow;
                first_capture_ch1 = 0;
            } else {
                // 计算间隔 Tick 数
                // diff_overflow = overflow - last_overflow_ch1 (自动处理 wrapping)
                uint32_t diff_overflow = overflow - last_overflow_ch1;
                uint64_t ticks = (uint64_t)diff_overflow * 65536 + CCR - last_capture_ch1;
                
                // Clock = 1MHz
                if (ticks > 0) {
                    freq_ch1 = 1000000 / ticks;
                }
                
                last_capture_ch1 = CCR;
                last_overflow_ch1 = overflow;
            }
        }
    }
    else if (htim->Instance == TIM2) {
        // CH2 Frequency (PA1) - Shared with Signal Gen (PWM)
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            uint32_t CCR = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            uint32_t overflow = tim2_overflow_count;
            
            uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);
            uint32_t PSC = htim->Instance->PSC;
            uint32_t clock_freq = 72000000 / (PSC + 1);
            uint32_t period = ARR + 1;

            if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) && (CCR < (period / 2))) {
                overflow++;
            }
            
            if (first_capture_ch2) {
                last_capture_ch2 = CCR;
                last_overflow_ch2 = overflow;
                first_capture_ch2 = 0;
            } else {
                // 计算间隔 Tick 数
                uint32_t diff_overflow = overflow - last_overflow_ch2;
                uint64_t ticks = (uint64_t)diff_overflow * period + CCR - last_capture_ch2;
                
                if (ticks > 0) {
                    freq_ch2 = clock_freq / ticks;
                }
                
                last_capture_ch2 = CCR;
                last_overflow_ch2 = overflow;
            }
        }
    }
}
