#include "osc_app.h"
#include "adc.h"
#include "tim.h"
#include "st7735.h"
#include "signal_gen.h"
#include <stdio.h>
#include <string.h>

// 全局变量定义
uint32_t osc_adc_buffer[OSC_ADC_NUM];
volatile uint8_t osc_adc_cplt_flag = 0;

static OscState_t osc_state = OSC_RUN;
static OscMode_t osc_mode = OSC_MODE_CH1;
static uint8_t attenuation_x50 = 0; // 0: x1, 1: x50

// 底部信息显示模式 (0: Vpp, 1: Freq)
static uint8_t osc_info_mode = 0;

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

static uint16_t oldWaveCH1[OSC_WAVE_DRAW_WIDTH];
static uint16_t newWaveCH1[OSC_WAVE_DRAW_WIDTH];
static uint16_t oldWaveCH2[OSC_WAVE_DRAW_WIDTH];
static uint16_t newWaveCH2[OSC_WAVE_DRAW_WIDTH];

static float vpp_ch1 = 0.0f;
static float vpp_ch2 = 0.0f;

// 外部变量 (由main.c维护)
extern TIM_HandleTypeDef htim1;

// 触发阈值 (ADC值)
#define TRIG_THRESHOLD 2048

static void UpdateTimebaseHardware(void) {
    uint16_t arr = timebase_configs[timebase_idx].arr;
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
}

void OSC_Init(void) {
    // 启动信号发生器
    SignalGen_Init();
    // Apply default timebase
    UpdateTimebaseHardware();
}

void OSC_Start(void) {
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

void OSC_Process(void) {
    if (osc_adc_cplt_flag) {
        if (osc_state == OSC_RUN) {
            uint8_t step = timebase_configs[timebase_idx].step;
            
            // 1. 计算 Vpp
            uint32_t min1 = 0xFFF, max1 = 0;
            uint32_t min2 = 0xFFF, max2 = 0;
            
            for(int i = 0; i < OSC_ADC_NUM; i++) {
                uint16_t val1 = osc_adc_buffer[i] & 0xFFFF;
                uint16_t val2 = (osc_adc_buffer[i] >> 16) & 0xFFFF;
                
                if (val1 < min1) min1 = val1;
                if (val1 > max1) max1 = val1;
                if (val2 < min2) min2 = val2;
                if (val2 > max2) max2 = val2;
            }
            
            // 计算 Vpp
            // 基础公式: (max - min) * 3.3 / 4095 * 2
            // 如果衰减档位开启，则 * 50
            float factor = 2.0f;
            if (attenuation_x50) factor *= 50.0f;
            
            vpp_ch1 = (max1 - min1) * 3.3f / 4095.0f * factor;
            vpp_ch2 = (max2 - min2) * 3.3f / 4095.0f * factor;

            // 2. 触发查找 (CH1上升沿 -> 对应ADC下降沿)
            uint16_t t = 0;
            uint32_t search_limit = OSC_ADC_NUM - (OSC_WAVE_WIDTH * step) - 10;
            if (search_limit > OSC_ADC_NUM) search_limit = 0;

            for (int i = 0; i < search_limit; i++) {
                uint16_t val1 = osc_adc_buffer[i] & 0xFFFF;
                uint16_t val2 = osc_adc_buffer[i+1] & 0xFFFF;
                if (val1 > TRIG_THRESHOLD && val2 <= TRIG_THRESHOLD) {
                    t = i;
                    break;
                }
            }
            
            // 3. 准备波形数据 (仅采集绘制区域宽度的数据)
            for (int i = 0; i < OSC_WAVE_DRAW_WIDTH; i++) {
                uint32_t idx = t + i * step;
                if (idx >= OSC_ADC_NUM) idx = OSC_ADC_NUM - 1;
                
                uint32_t raw = osc_adc_buffer[idx];
                newWaveCH1[i] = ADC2Y(raw & 0xFFFF);
                newWaveCH2[i] = ADC2Y((raw >> 16) & 0xFFFF);
            }
        }
        
        osc_adc_cplt_flag = 0;
    }
}

void OSC_DrawWaveform(void) {
    if (osc_state == OSC_RUN) {
        // 在 (1, OSC_WAVE_DRAW_WIDTH-1) 范围内绘制波形连接线
        for (int i = 1; i < OSC_WAVE_DRAW_WIDTH; i++) {
            // 1. 始终擦除旧波形
            ST7735_DrawLine(i, oldWaveCH1[i-1], i + 1, oldWaveCH1[i], ST7735_BLACK);
            ST7735_DrawLine(i, oldWaveCH2[i-1], i + 1, oldWaveCH2[i], ST7735_BLACK);
            
            // 2. 绘制新波形 (仅绘制当前启用的通道)
            if (osc_mode == OSC_MODE_CH1 || osc_mode == OSC_MODE_DUAL) {
                ST7735_DrawLine(i, newWaveCH1[i-1], i + 1, newWaveCH1[i], ST7735_GREEN);
            }
            if (osc_mode == OSC_MODE_CH2 || osc_mode == OSC_MODE_DUAL) {
                ST7735_DrawLine(i, newWaveCH2[i-1], i + 1, newWaveCH2[i], ST7735_YELLOW);
            }
            
            // 更新旧数据
            oldWaveCH1[i-1] = newWaveCH1[i-1];
            oldWaveCH2[i-1] = newWaveCH2[i-1];
        }
        oldWaveCH1[OSC_WAVE_DRAW_WIDTH-1] = newWaveCH1[OSC_WAVE_DRAW_WIDTH-1];
        oldWaveCH2[OSC_WAVE_DRAW_WIDTH-1] = newWaveCH2[OSC_WAVE_DRAW_WIDTH-1];
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
    attenuation_x50 = !attenuation_x50;
}

void OSC_CycleChannelMode(void) {
    osc_mode = (OscMode_t)((osc_mode + 1) % OSC_MODE_COUNT);
}

void OSC_ToggleInfoMode(void) {
    osc_info_mode = !osc_info_mode;
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
    return attenuation_x50 ? 50 : 1;
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
