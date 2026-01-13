#ifndef __OSC_APP_H
#define __OSC_APP_H

#include "main.h"

// 示波器参数定义
#define OSC_ADC_NUM 2000        // ADC采集深度
#define OSC_WAVE_WIDTH 160      // 屏幕总宽度
#define OSC_WAVE_DRAW_WIDTH 158 // 实际波形绘制宽度 (左右预留边框)
#define OSC_WAVE_TOP_Y 20       // 波形显示区域顶部Y坐标
#define OSC_WAVE_HEIGHT 90      // 调整高度以避开底部文字 (20+90=110, 底部文字在112)
#define OSC_WAVE_BOTTOM_Y (OSC_WAVE_TOP_Y + OSC_WAVE_HEIGHT)

// 运行状态
typedef enum {
    OSC_RUN = 0,
    OSC_PAUSE = 1
} OscState_t;

// 通道显示模式
typedef enum {
    OSC_MODE_CH1 = 0,
    OSC_MODE_CH2,
    OSC_MODE_DUAL,
    OSC_MODE_COUNT
} OscMode_t;

// 全局变量声明
extern uint32_t osc_adc_buffer[OSC_ADC_NUM];
extern volatile uint8_t osc_adc_cplt_flag;

// 核心功能函数
void OSC_Init(void);
void OSC_Start(void);
void OSC_Process(void);         // 处理数据（触发、缩放、计算）
void OSC_DrawWaveform(void);    // 绘制波形（需在UI刷新中调用）

// 控制函数
void OSC_ChangeTimebase(int8_t direction);
void OSC_TogglePause(void);
void OSC_ToggleAttenuation(void); // 切换衰减档位 (x1 / x50)
void OSC_CycleChannelMode(void);  // 切换通道显示 (CH1 -> CH2 -> Dual)

// 获取状态
uint8_t OSC_GetStep(void);
const char* OSC_GetTimebaseName(void);
OscState_t OSC_GetState(void);
OscMode_t OSC_GetChannelMode(void);
uint8_t OSC_GetAttenuation(void); // 返回 1 或 50

// 数据获取函数 (供UI显示)
float OSC_GetVpp(uint8_t channel);
uint32_t OSC_GetFrequency(uint8_t channel);

// 切换底部信息显示模式 (Vpp / Freq)
void OSC_ToggleInfoMode(void);
// 获取当前底部信息显示模式 (0: Vpp, 1: Freq)
uint8_t OSC_GetInfoMode(void);

// Input Capture Callbacks
void OSC_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* __OSC_APP_H */
