#include "signal_gen.h"
#include "tim.h"

SignalGen_t g_SignalGen;

static void SignalGen_ConfigHardware(void);

void SignalGen_Init(void)
{
    // 初始化默认参数
    g_SignalGen.type = WAVE_SQUARE;
    g_SignalGen.frequency = 1000;
    g_SignalGen.duty_cycle = 50;
    g_SignalGen.running = 0;

    SignalGen_ConfigHardware();
    
    // Sync LED2 (Signal Gen) state
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, g_SignalGen.running ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SignalGen_SetType(WaveType_t type)
{
    // 仅支持方波，忽略其他类型设置
    if (type == WAVE_SQUARE) {
        g_SignalGen.type = WAVE_SQUARE;
        // SignalGen_ConfigHardware(); // 不需要重新配置，因只支持方波
    }
}

void SignalGen_SetFrequency(uint32_t freq)
{
    if (freq < 1) freq = 1;
    if (freq > 20000) freq = 20000; // 限制最高频率
    
    g_SignalGen.frequency = freq;
    SignalGen_ConfigHardware();
}

void SignalGen_SetDutyCycle(uint8_t duty)
{
    if (duty < 1) duty = 1;
    if (duty > 99) duty = 99;
    
    g_SignalGen.duty_cycle = duty;
    // 总是更新，因为仅支持方波
    SignalGen_ConfigHardware();
}

static void SignalGen_ConfigHardware(void)
{
    if (!g_SignalGen.running) {
        return;
    }

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop_IT(&htim2);

    uint32_t psc = 71;
    if (g_SignalGen.frequency == 0) g_SignalGen.frequency = 1;
    
    uint32_t arr = (1000000 / g_SignalGen.frequency) - 1;
    
    if (g_SignalGen.frequency > 10000) {
        psc = 0;
        arr = (72000000 / g_SignalGen.frequency) - 1;
    }

    __HAL_TIM_SET_PRESCALER(&htim2, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    
    uint32_t pulse = (arr + 1) * g_SignalGen.duty_cycle / 100;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
}

void SignalGen_SetRunning(uint8_t running)
{
    if (running) {
        if (!g_SignalGen.running) {
            g_SignalGen.running = 1;
            SignalGen_ConfigHardware();
        }
    } else {
        if (g_SignalGen.running) {
            g_SignalGen.running = 0;
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
            HAL_TIM_Base_Stop_IT(&htim2);
        }
    }
    // Update LED2 (Signal Gen)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, g_SignalGen.running ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SignalGen_ToggleRunning(void)
{
    SignalGen_SetRunning(!g_SignalGen.running);
}

void SignalGen_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}
