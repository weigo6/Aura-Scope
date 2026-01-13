/**
  ******************************************************************************
  * @file    ui.h
  * @brief   UI 管理与界面绘制头文件
  ******************************************************************************
  */

#ifndef __UI_H__
#define __UI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "st7735.h"

/**
  * @brief UI 界面枚举
  */
typedef enum {
    UI_STATE_WELCOME = 0,
    UI_STATE_OSCILLOSCOPE,
    UI_STATE_SIGNAL_GEN,
    UI_STATE_COUNT
} UI_State_t;

/**
  * @brief UI 初始化
  */
void UI_Init(void);

/**
  * @brief 切换到下一个界面
  */
void UI_NextState(void);

/**
  * @brief 仅切换状态不刷新
  * @param steps 前进的步数
  */
void UI_StepState(int steps);

/**
  * @brief 刷新当前 UI 界面
  */
void UI_Refresh(void);

/**
  * @brief 绘制欢迎界面
  */
void UI_DrawWelcome(void);

/**
  * @brief 绘制示波器界面
  */
void UI_DrawOscilloscope(void);

/**
  * @brief 绘制信号发生器界面
  */
void UI_DrawSignalGen(void);

/**
  * @brief 处理按键事件
  * @param key_id 按键ID (1:KEY1, 2:KEY2, 3:KEY3, 4:KEY4)
  */
void UI_HandleKey(uint8_t key_id);

/**
  * @brief 处理旋钮事件
  * @param delta 变化量 (+1/-1)
  */
void UI_HandleEncoder(int8_t delta);

#ifdef __cplusplus
}
#endif

#endif /* __UI_H__ */
