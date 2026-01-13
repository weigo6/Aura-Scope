/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7735.h
  * @brief   ST7735 1.8寸 TFT 显示屏驱动头文件
  *          支持 128x160 分辨率，硬件 SPI 通信
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 AuraScope Project.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ST7735_H__
#define __ST7735_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* ST7735 显示屏尺寸定义（横屏） */
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 128

/* 颜色定义 (RGB565 格式) */
#define ST7735_BLACK     0x0000
#define ST7735_WHITE     0xFFFF
#define ST7735_RED       0xF800
#define ST7735_GREEN     0x07E0
#define ST7735_BLUE      0x001F
#define ST7735_CYAN      0x07FF
#define ST7735_MAGENTA   0xF81F
#define ST7735_YELLOW    0xFFE0
#define ST7735_ORANGE    0xFC00
#define ST7735_PURPLE    0x8010
#define ST7735_GRAY      0x8410

/* GPIO 引脚定义 */
#define ST7735_RES_PIN   GPIO_PIN_5
#define ST7735_RES_PORT  GPIOB
#define ST7735_DC_PIN    GPIO_PIN_6
#define ST7735_DC_PORT   GPIOB
#define ST7735_CS_PIN    GPIO_PIN_7
#define ST7735_CS_PORT   GPIOB
#define ST7735_BLK_PIN   GPIO_PIN_8
#define ST7735_BLK_PORT  GPIOB

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* 控制引脚宏定义 */
#define ST7735_RES_LOW()   HAL_GPIO_WritePin(ST7735_RES_PORT, ST7735_RES_PIN, GPIO_PIN_RESET)
#define ST7735_RES_HIGH()  HAL_GPIO_WritePin(ST7735_RES_PORT, ST7735_RES_PIN, GPIO_PIN_SET)
#define ST7735_DC_LOW()    HAL_GPIO_WritePin(ST7735_DC_PORT, ST7735_DC_PIN, GPIO_PIN_RESET)
#define ST7735_DC_HIGH()   HAL_GPIO_WritePin(ST7735_DC_PORT, ST7735_DC_PIN, GPIO_PIN_SET)
#define ST7735_CS_LOW()    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_RESET)
#define ST7735_CS_HIGH()   HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_SET)
#define ST7735_BLK_ON()    HAL_GPIO_WritePin(ST7735_BLK_PORT, ST7735_BLK_PIN, GPIO_PIN_SET)
#define ST7735_BLK_OFF()   HAL_GPIO_WritePin(ST7735_BLK_PORT, ST7735_BLK_PIN, GPIO_PIN_RESET)

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  ST7735 初始化函数
  * @retval None
  */
void ST7735_Init(void);

/**
  * @brief  设置显示窗口
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @retval None
  */
void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
  * @brief  填充整个屏幕
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_FillScreen(uint16_t color);

/**
  * @brief  绘制单个像素点
  * @param  x: X 坐标 (0-127)
  * @param  y: Y 坐标 (0-159)
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/**
  * @brief  绘制直线
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
  * @brief  绘制矩形
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_DrawRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
  * @brief  填充矩形
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
  * @brief  显示 AuraScope 启动界面
  * @retval None
  */
void ST7735_ShowSplashScreen(void);

/**
  * @brief  在指定位置显示一个字符
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  ch: 字符
  * @param  color: 颜色
  * @param  bgcolor: 背景色
  * @retval None
  */
void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bgcolor);

/**
  * @brief  在指定位置显示字符串
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  str: 字符串指针
  * @param  color: 颜色
  * @param  bgcolor: 背景色
  * @retval None
  */
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bgcolor);

/**
  * @brief  发送数据缓冲区到 ST7735
  * @param  data: 数据缓冲区指针
  * @param  len: 数据长度（16位数据个数）
  * @retval None
  */
void ST7735_WriteDataBuffer(uint16_t *data, uint32_t len);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __ST7735_H__ */
