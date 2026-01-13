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

typedef enum {
    PARAM_FREQUENCY = 0,
    PARAM_DUTY_CYCLE,
    PARAM_COUNT
} SigGen_Param_t;

static SigGen_Param_t sig_gen_selected_param = PARAM_FREQUENCY;

// For Oscilloscope anti-flicker
#define ADC_BUFFER_SIZE 1000

static void FormatFreq(float freq, char *buf, size_t size) {
    if (freq < 1000) {
        snprintf(buf, size, "%3.0fHz", freq); // 999Hz
    } else if (freq < 100000) {
        snprintf(buf, size, "%2.1fkHz", freq / 1000.0f); // 99.9kHz
    } else {
        snprintf(buf, size, "%3.0fkHz", freq / 1000.0f); // 999kHz
    }
}

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
  * @brief 绘制示波器界面
  */
void UI_DrawOscilloscope(void)
{
    // 绘制坐标网格 (仅在完全重绘时绘制)
    if (ui_full_redraw) {
        // 波形区域高度调整为 100 (20-120)
        ST7735_DrawRect(0, OSC_WAVE_TOP_Y, ST7735_WIDTH-1, OSC_WAVE_BOTTOM_Y, ST7735_GRAY);
        // 中心线
        ST7735_DrawLine(0, OSC_WAVE_TOP_Y + OSC_WAVE_HEIGHT/2, ST7735_WIDTH-1, OSC_WAVE_TOP_Y + OSC_WAVE_HEIGHT/2, ST7735_GRAY);
    }
    
    // 调用 OSC_App 进行波形绘制
    OSC_DrawWaveform();
    
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
        ST7735_DrawString(136, 2, "x50", ST7735_RED, ST7735_BLACK);
    } else {
        ST7735_DrawString(136, 2, "x1 ", ST7735_GREEN, ST7735_BLACK);
    }

    // 更新底部信息 (底部栏 Y=112)
    float vpp1 = OSC_GetVpp(0);
    float freq1 = OSC_GetFreq(0);
    float vpp2 = OSC_GetVpp(1);
    float freq2 = OSC_GetFreq(1);
    
    char freq_str[10];
    
    // 移除 FillRect 以避免闪烁，改用空格补齐清除旧字符

    if (osc_mode == OSC_MODE_CH1) {
        // CH1 独占: "CH1: 3.30V 1.0kHz    " (补齐到20字符)
        int v_int = (int)vpp1;
        int v_frac = (int)((vpp1 - v_int) * 100);
        FormatFreq(freq1, freq_str, sizeof(freq_str));
        // %-20s 在右侧补空格 (需配合临时buffer)
        char tmp[24];
        snprintf(tmp, sizeof(tmp), "CH1: %d.%02dV %s", v_int, v_frac, freq_str);
        snprintf(buf, sizeof(buf), "%-20s", tmp); 
        ST7735_DrawString(0, 112, buf, ST7735_GREEN, ST7735_BLACK);
    }
    else if (osc_mode == OSC_MODE_CH2) {
        int v_int = (int)vpp2;
        int v_frac = (int)((vpp2 - v_int) * 100);
        FormatFreq(freq2, freq_str, sizeof(freq_str));
        char tmp[24];
        snprintf(tmp, sizeof(tmp), "CH2: %d.%02dV %s", v_int, v_frac, freq_str);
        snprintf(buf, sizeof(buf), "%-20s", tmp);
        ST7735_DrawString(0, 112, buf, ST7735_CYAN, ST7735_BLACK);
    }
    else {
        // Dual Mode
        // CH1 Left (10 chars width)
        int v_int1 = (int)vpp1;
        int v_frac1 = (int)((vpp1 - v_int1) * 10);
        FormatFreq(freq1, freq_str, sizeof(freq_str));
        char tmp[16];
        snprintf(tmp, sizeof(tmp), "%d.%dV %s", v_int1, v_frac1, freq_str);
        snprintf(buf, sizeof(buf), "%-10s", tmp); // Pad to 10 chars
        ST7735_DrawString(0, 112, buf, ST7735_GREEN, ST7735_BLACK);
        
        // CH2 Right (10 chars width)
        int v_int2 = (int)vpp2;
        int v_frac2 = (int)((vpp2 - v_int2) * 10);
        FormatFreq(freq2, freq_str, sizeof(freq_str));
        snprintf(tmp, sizeof(tmp), "%d.%dV %s", v_int2, v_frac2, freq_str);
        snprintf(buf, sizeof(buf), "%-10s", tmp);
        ST7735_DrawString(80, 112, buf, ST7735_CYAN, ST7735_BLACK);
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
            // KEY4: Unused currently (or extra function)
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
