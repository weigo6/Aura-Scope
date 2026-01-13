#ifndef __SIGNAL_GEN_H
#define __SIGNAL_GEN_H

#include "main.h"

// 波形类型枚举
typedef enum {
    WAVE_SQUARE = 0,
    WAVE_TYPE_COUNT
} WaveType_t;

// 信号发生器状态结构体
typedef struct {
    WaveType_t type;
    uint32_t frequency;     // Hz
    uint8_t duty_cycle;     // 1-99% (仅方波有效)
    uint8_t running;        // 是否正在输出
} SignalGen_t;

extern SignalGen_t g_SignalGen;

// 函数原型
void SignalGen_Init(void);
void SignalGen_SetType(WaveType_t type);
void SignalGen_SetFrequency(uint32_t freq);
void SignalGen_SetDutyCycle(uint8_t duty);
void SignalGen_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* __SIGNAL_GEN_H */
