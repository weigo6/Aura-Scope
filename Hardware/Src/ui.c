/**
  ******************************************************************************
  * @file    ui.c
  * @brief   UI 管理与界面绘制实现文件
  ******************************************************************************
  */

#include "ui.h"
#include "signal_gen.h"
#include "osc_app.h"
#include <stdio.h>

static UI_State_t current_ui_state = UI_STATE_WELCOME;
static UI_State_t last_ui_state = UI_STATE_COUNT;
static uint8_t ui_full_redraw = 0;

// 缓存上一次显示的数值，用于减少频闪
static int last_v1_int = -1, last_v1_frac = -1;
static int last_v2_int = -1, last_v2_frac = -1;
static uint32_t last_freq1 = 0xFFFFFFFF;
static uint32_t last_freq2 = 0xFFFFFFFF;
static uint8_t last_info_mode = 0xFF;
static uint8_t last_cdc_state = 0xFF;
static OscMode_t last_osc_mode_bottom = OSC_MODE_COUNT;

typedef enum {
    PARAM_FREQUENCY = 0,
    PARAM_DUTY_CYCLE,
    PARAM_COUNT
} SigGen_Param_t;

static SigGen_Param_t sig_gen_selected_param = PARAM_FREQUENCY;

// For Oscilloscope anti-flicker
#define ADC_BUFFER_SIZE 1000


/**
  * @brief UI 初始化
  */
void UI_Init(void)
{
    current_ui_state = UI_STATE_WELCOME;
    last_ui_state = UI_STATE_COUNT; // Ensure first refresh triggers full redraw
    UI_Refresh();
}

/**
  * @brief 切换到下一个界面 (自动刷新)
  */
void UI_NextState(void)
{
    UI_StepState(1);
    UI_Refresh();
}

/**
  * @brief 仅切换状态不刷新 (用于快速连续切换)
  * @param steps 前进的步数
  */
void UI_StepState(int steps)
{
    current_ui_state = (UI_State_t)((current_ui_state + steps) % UI_STATE_COUNT);
}

/**
  * @brief 刷新当前 UI 界面
  */
void UI_Refresh(void)
{
    if (current_ui_state != last_ui_state) {
        ui_full_redraw = 1;
        ST7735_FillScreen(ST7735_BLACK);
    } else {
        ui_full_redraw = 0;
    }

    switch (current_ui_state) {
        case UI_STATE_WELCOME:
            UI_DrawWelcome();
            break;
        case UI_STATE_OSCILLOSCOPE:
            UI_DrawOscilloscope();
            break;
        case UI_STATE_SIGNAL_GEN:
            UI_DrawSignalGen();
            break;
        default:
            break;
    }
    
    last_ui_state = current_ui_state;
}

/**
  * @brief 绘制欢迎界面
  */
void UI_DrawWelcome(void)
{
    if (!ui_full_redraw) return; // Static screen, no update needed

    // 绘制深色极客风格背景
    for (uint16_t i = 0; i < ST7735_WIDTH; i += 20) {
        ST7735_DrawLine(i, 0, i, ST7735_HEIGHT - 1, 0x0841); // 深灰色垂直线
    }
    for (uint16_t i = 0; i < ST7735_HEIGHT; i += 20) {
        ST7735_DrawLine(0, i, ST7735_WIDTH - 1, i, 0x0841); // 深灰色水平线
    }

    // 绘制中心发光效果
    ST7735_DrawRect(38, 38, 122, 92, ST7735_BLUE);
    ST7735_DrawRect(40, 40, 120, 90, ST7735_CYAN);
    
    // 显示 "AuraScope"
    ST7735_DrawString(44, 50, "AuraScope", ST7735_WHITE, ST7735_BLACK);
    ST7735_DrawString(55, 70, "V1.0.0", ST7735_GREEN, ST7735_BLACK);

    // 提示信息 (居中)
    ST7735_DrawString(8, 105, "Press KEY to Start", ST7735_YELLOW, ST7735_BLACK);
}

/**
  * @brief 绘制波形背景网格
  */
void UI_DrawGrid(void)
{
    // 绘制垂直网格线 (每 20 像素一根)
    for (uint16_t x = 20; x < ST7735_WIDTH - 1; x += 20) {
        for (uint16_t y = OSC_WAVE_TOP_Y + 1; y < OSC_WAVE_BOTTOM_Y; y += 4) {
            ST7735_DrawPixel(x, y, 0x2104); // 使用深灰色虚线 (RGB565: 4,4,4)
        }
    }

    // 绘制水平网格线 (每 18 像素一根，正好 5 个区间对应 90 像素高度)
    for (uint16_t y = OSC_WAVE_TOP_Y + 18; y < OSC_WAVE_BOTTOM_Y; y += 18) {
        for (uint16_t x = 1; x < ST7735_WIDTH - 1; x += 4) {
            ST7735_DrawPixel(x, y, 0x2104); // 使用深灰色虚线
        }
    }

    // 绘制中心十字轴 (实线)
    ST7735_DrawLine(ST7735_WIDTH / 2, OSC_WAVE_TOP_Y + 1, ST7735_WIDTH / 2, OSC_WAVE_BOTTOM_Y - 1, 0x3186);
    ST7735_DrawLine(1, OSC_WAVE_TOP_Y + OSC_WAVE_HEIGHT / 2, ST7735_WIDTH - 2, OSC_WAVE_TOP_Y + OSC_WAVE_HEIGHT / 2, 0x3186);
}

/**
  * @brief 绘制示波器界面
  */
void UI_DrawOscilloscope(void)
{
    // 绘制坐标网格 (仅在完全重绘时绘制)
    if (ui_full_redraw) {
        // 绘制波形外框 (0, 20) 到 (159, 110)
        ST7735_DrawRect(0, OSC_WAVE_TOP_Y, ST7735_WIDTH - 1, OSC_WAVE_BOTTOM_Y, ST7735_GRAY);
        // 绘制内部网格
        UI_DrawGrid();
    }
    
    // 调用 OSC_App 进行波形绘制
    OSC_DrawWaveform();

    // 重要：在绘制顶部/底部文字前，确保波形 DMA 传输已完成
    // 防止 SetWindow 命令与文字绘制冲突
    while (ST7735_IsBusy());

    // 更新状态显示 (顶部状态栏 Y=2)
    char buf[32];
    OscMode_t osc_mode = OSC_GetChannelMode();
    
    // 1. 运行状态 (左)
    if (OSC_GetState() == OSC_PAUSE) {
        ST7735_DrawString(0, 2, "STOP", ST7735_RED, ST7735_BLACK);
    } else {
        ST7735_DrawString(0, 2, "RUN ", ST7735_GREEN, ST7735_BLACK);
    }
    
    // 2. 时基 (中)
    snprintf(buf, sizeof(buf), "%s  ", OSC_GetTimebaseName()); 
    ST7735_DrawString(64, 2, buf, ST7735_YELLOW, ST7735_BLACK);

    // 3. 衰减档位 (右)
    if (OSC_GetAttenuation() == 50) {
        ST7735_DrawString(100, 2, "x50", ST7735_RED, ST7735_BLACK);
    } else {
        ST7735_DrawString(100, 2, "x1 ", ST7735_GREEN, ST7735_BLACK);
    }

    // 4. USB CDC 状态 (最右侧)
    uint8_t cdc_state = OSC_GetCDCState();
    if (cdc_state != last_cdc_state || ui_full_redraw) {
        if (cdc_state) {
            ST7735_DrawString(130, 2, "USB", ST7735_CYAN, ST7735_BLACK);
        } else {
            ST7735_DrawString(130, 2, "---", ST7735_GRAY, ST7735_BLACK);
        }
        last_cdc_state = cdc_state;
    }

    // 更新底部信息 (底部栏 Y=112)
    float vpp1 = OSC_GetVpp(0);
    float vpp2 = OSC_GetVpp(1);
    uint32_t freq1 = OSC_GetFrequency(0);
    uint32_t freq2 = OSC_GetFrequency(1);
    uint8_t info_mode = OSC_GetInfoMode();

    int v1_int = (int)vpp1, v1_frac = (int)((vpp1 - v1_int) * 100);
    int v2_int = (int)vpp2, v2_frac = (int)((vpp2 - v2_int) * 100);

    // 检查是否需要刷新
    if (osc_mode != last_osc_mode_bottom ||
        info_mode != last_info_mode ||
        v1_int != last_v1_int || v1_frac != last_v1_frac ||
        v2_int != last_v2_int || v2_frac != last_v2_frac ||
        freq1 != last_freq1 || freq2 != last_freq2)
    {
        char str_ch1[18], str_ch2[18];
        
        if (info_mode == 0) { // Vpp Mode
            snprintf(str_ch1, sizeof(str_ch1), "C1:%d.%02dV", v1_int, v1_frac);
            snprintf(str_ch2, sizeof(str_ch2), "C2:%d.%02dV", v2_int, v2_frac);
        } else { // Freq Mode
            // Format frequency: <1k: xxxHz, >=1k: x.xkHz (避免浮点printf)
            if (freq1 < 1000) snprintf(str_ch1, sizeof(str_ch1), "CH1:%luHz", freq1);
            else snprintf(str_ch1, sizeof(str_ch1), "CH1:%lu.%1lukHz", freq1/1000, (freq1%1000)/100);
            
            if (freq2 < 1000) snprintf(str_ch2, sizeof(str_ch2), "CH2:%luHz", freq2);
            else snprintf(str_ch2, sizeof(str_ch2), "CH2:%lu.%1lukHz", freq2/1000, (freq2%1000)/100);
        }

        // 构造最终显示字符串并填充空格
        char pad1[18], pad2[18];
        snprintf(pad1, sizeof(pad1), "%-11s", str_ch1);
        snprintf(pad2, sizeof(pad2), "%-11s", str_ch2);
        
        if (osc_mode == OSC_MODE_CH1) {
            ST7735_DrawString(0, 112, pad1, ST7735_GREEN, ST7735_BLACK);
            // 清除右侧可能存在的旧 CH2 (使用空格覆盖)
            ST7735_DrawString(85, 112, "           ", ST7735_WHITE, ST7735_BLACK);
        }
        else if (osc_mode == OSC_MODE_CH2) {
            // 清除左侧可能存在的旧 CH1 (使用空格覆盖)
            ST7735_DrawString(0, 112, "           ", ST7735_WHITE, ST7735_BLACK);
            ST7735_DrawString(85, 112, pad2, ST7735_CYAN, ST7735_BLACK);
        }
        else {
            ST7735_DrawString(0, 112, pad1, ST7735_GREEN, ST7735_BLACK);
            ST7735_DrawString(85, 112, pad2, ST7735_CYAN, ST7735_BLACK);
        }

        // 更新缓存
        last_v1_int = v1_int; last_v1_frac = v1_frac;
        last_v2_int = v2_int; last_v2_frac = v2_frac;
        last_freq1 = freq1; last_freq2 = freq2;
        last_info_mode = info_mode;
        last_osc_mode_bottom = osc_mode;
    }
}

/**
  * @brief 绘制信号发生器界面
  */
void UI_DrawSignalGen(void)
{
    char buf[32];
    
    // 静态元素 (仅重绘时绘制)
    if (ui_full_redraw) {
        // 标题
        ST7735_FillRect(0, 0, ST7735_WIDTH - 1, 18, ST7735_MAGENTA);
        ST7735_DrawString(16, 2, "SIGNAL GENERATOR", ST7735_WHITE, ST7735_MAGENTA);
        
        // 方波提示
        ST7735_DrawString(10, 30, "Wave: Square", ST7735_CYAN, ST7735_BLACK);
    }
    
    // 动态元素：选中框与数值
    
    // 1. 频率
    uint16_t color = (sig_gen_selected_param == PARAM_FREQUENCY) ? ST7735_YELLOW : ST7735_WHITE;
    ST7735_DrawRect(5, 55, 154, 80, color);
    snprintf(buf, sizeof(buf), "Freq: %lu Hz  ", (unsigned long)g_SignalGen.frequency);
    ST7735_DrawString(10, 60, buf, ST7735_CYAN, ST7735_BLACK);
    
    // 2. 占空比
    color = (sig_gen_selected_param == PARAM_DUTY_CYCLE) ? ST7735_YELLOW : ST7735_WHITE;
    ST7735_DrawRect(5, 85, 154, 110, color);
    snprintf(buf, sizeof(buf), "Duty: %u %%  ", g_SignalGen.duty_cycle);
    ST7735_DrawString(10, 90, buf, ST7735_CYAN, ST7735_BLACK);
}

/**
  * @brief 处理按键事件
  */
void UI_HandleKey(uint8_t key_id)
{
    // KEY1: 切换界面 (全局)
    if (key_id == 1) {
        UI_NextState();
        return;
    }

    if (current_ui_state == UI_STATE_OSCILLOSCOPE) {
        if (key_id == 2) {
            // KEY2: 切换衰减档位 (x1 / x50)
            OSC_ToggleAttenuation();
        } else if (key_id == 3) {
            // KEY3: 切换通道显示 (CH1 -> CH2 -> Dual)
            OSC_CycleChannelMode();
            ui_full_redraw = 1; // 强制刷新以更新布局
            UI_Refresh();
        } else if (key_id == 4) {
            // 在暂停状态下，点击 KEY4 切换 USB 传输
            if (OSC_GetState() == OSC_PAUSE) {
                OSC_ToggleCDC();
            } else {
                // 运行状态下，KEY4 切换底部信息显示 (Vpp / Freq)
                OSC_ToggleInfoMode();
            }
        } else if (key_id == 5) { // EC11 Button
            // EC11 Button: Pause
            OSC_TogglePause();
        }
    }
    else if (current_ui_state == UI_STATE_SIGNAL_GEN) {
        switch (key_id) {
            case 2: // KEY2: 切换选中参数
                sig_gen_selected_param = (SigGen_Param_t)((sig_gen_selected_param + 1) % PARAM_COUNT);
                break;
                
            case 3: // KEY3: 减少值
                if (sig_gen_selected_param == PARAM_FREQUENCY) {
                    uint32_t step = (g_SignalGen.frequency >= 1000) ? 100 : ((g_SignalGen.frequency >= 100) ? 10 : 1);
                    if (g_SignalGen.frequency > step) SignalGen_SetFrequency(g_SignalGen.frequency - step);
                } else if (sig_gen_selected_param == PARAM_DUTY_CYCLE) {
                    if (g_SignalGen.duty_cycle > 1) SignalGen_SetDutyCycle(g_SignalGen.duty_cycle - 1);
                }
                break;
                
            case 4: // KEY4: 增加值
                if (sig_gen_selected_param == PARAM_FREQUENCY) {
                    uint32_t step = (g_SignalGen.frequency >= 1000) ? 100 : ((g_SignalGen.frequency >= 100) ? 10 : 1);
                    SignalGen_SetFrequency(g_SignalGen.frequency + step);
                } else if (sig_gen_selected_param == PARAM_DUTY_CYCLE) {
                    if (g_SignalGen.duty_cycle < 99) SignalGen_SetDutyCycle(g_SignalGen.duty_cycle + 1);
                }
                break;
        }
        UI_Refresh();
    }
}

/**
  * @brief 处理旋钮事件
  */
void UI_HandleEncoder(int8_t delta)
{
    // 信号发生器界面
    if (current_ui_state == UI_STATE_SIGNAL_GEN) {
        if (delta > 0) {
            UI_HandleKey(4);
        } else {
            UI_HandleKey(3);
        }
    }
    // 示波器界面：调节时基 (Hardware Sampling Rate)
    else if (current_ui_state == UI_STATE_OSCILLOSCOPE) {
        if (delta > 0) {
            OSC_ChangeTimebase(1); // Zoom Out (Slower Sampling)
        } else {
            OSC_ChangeTimebase(-1); // Zoom In (Faster Sampling)
        }
    }
}
