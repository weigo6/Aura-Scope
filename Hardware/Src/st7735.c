/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    st7735.c
  * @brief   ST7735 1.8寸 TFT 显示屏驱动实现
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

/* Includes ------------------------------------------------------------------*/
#include "st7735.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include "font.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* ST7735 命令定义 */
#define ST7735_NOP         0x00
#define ST7735_SWRESET     0x01
#define ST7735_RDDID       0x04
#define ST7735_RDDST       0x09
#define ST7735_SLPIN       0x10
#define ST7735_SLPOUT      0x11
#define ST7735_PTLON       0x12
#define ST7735_NORON       0x13
#define ST7735_INVOFF      0x20
#define ST7735_INVON       0x21
#define ST7735_DISPOFF     0x28
#define ST7735_DISPON      0x29
#define ST7735_CASET       0x2A
#define ST7735_RASET       0x2B
#define ST7735_RAMWR       0x2C
#define ST7735_RAMRD       0x2E
#define ST7735_PTLAR       0x30
#define ST7735_COLMOD      0x3A
#define ST7735_MADCTL      0x36
#define ST7735_FRMCTR1     0xB1
#define ST7735_FRMCTR2     0xB2
#define ST7735_FRMCTR3     0xB3
#define ST7735_INVCTR      0xB4
#define ST7735_DISSET5     0xB6
#define ST7735_PWCTR1      0xC0
#define ST7735_PWCTR2      0xC1
#define ST7735_PWCTR3      0xC2
#define ST7735_PWCTR4      0xC3
#define ST7735_PWCTR5      0xC4
#define ST7735_VMCTR1      0xC5
#define ST7735_RDID1       0xDA
#define ST7735_RDID2       0xDB
#define ST7735_RDID3       0xDC
#define ST7735_RDID4       0xDD
#define ST7735_PWCTR6      0xFC
#define ST7735_GMCTRP1     0xE0
#define ST7735_GMCTRN1     0xE1

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* SPI 句柄在 spi.h 中声明 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void ST7735_WriteCommand(uint8_t cmd);
static void ST7735_WriteData(uint8_t data);
static void ST7735_WriteData16(uint16_t data);
void ST7735_WriteDataBuffer(uint16_t *data, uint32_t len);
void ST7735_WriteDataBufferDMA(uint16_t *data, uint32_t len);
bool ST7735_IsBusy(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  发送命令到 ST7735
  * @param  cmd: 命令字节
  * @retval None
  */
static void ST7735_WriteCommand(uint8_t cmd)
{
    ST7735_DC_LOW();  // 命令模式
    ST7735_CS_LOW();  // 片选使能
    
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    
    ST7735_CS_HIGH(); // 片选禁用
}

/**
  * @brief  发送数据到 ST7735
  * @param  data: 数据字节
  * @retval None
  */
static void ST7735_WriteData(uint8_t data)
{
    ST7735_DC_HIGH(); // 数据模式
    ST7735_CS_LOW();  // 片选使能
    
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    
    ST7735_CS_HIGH(); // 片选禁用
}

/**
  * @brief  发送 16 位数据到 ST7735
  * @param  data: 16 位数据 (RGB565)
  * @retval None
  */
static void ST7735_WriteData16(uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF;  // 高字节
    buffer[1] = data & 0xFF;         // 低字节
    
    ST7735_DC_HIGH(); // 数据模式
    ST7735_CS_LOW();  // 片选使能
    
    HAL_SPI_Transmit(&hspi1, buffer, 2, HAL_MAX_DELAY);
    
    ST7735_CS_HIGH(); // 片选禁用
}

/**
  * @brief  发送数据缓冲区到 ST7735
  * @param  data: 数据缓冲区指针
  * @param  len: 数据长度（16位数据个数）
  * @retval None
  */
void ST7735_WriteDataBuffer(uint16_t *data, uint32_t len)
{
    ST7735_DC_HIGH(); // 数据模式
    ST7735_CS_LOW();  // 片选使能
    
    // 将 16 位数据转换为字节流
    uint8_t *buffer = (uint8_t *)data;
    uint32_t byteLen = len * 2;
    
    HAL_SPI_Transmit(&hspi1, buffer, byteLen, HAL_MAX_DELAY);
    
    ST7735_CS_HIGH(); // 片选禁用
}

/**
  * @brief  使用 DMA 发送数据缓冲区到 ST7735 (非阻塞)
  * @param  data: 数据缓冲区指针 (必须保持有效直到传输完成)
  * @param  len: 数据长度（16位数据个数）
  * @retval None
  */
void ST7735_WriteDataBufferDMA(uint16_t *data, uint32_t len)
{
    ST7735_DC_HIGH(); // 数据模式
    ST7735_CS_LOW();  // 片选使能
    
    // 注意：STM32 是小端，ST7735 是大端。
    // 如果传入的数据已经是大端字节序，则直接发送。
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, len * 2);
}

/**
  * @brief  检查 SPI 是否忙碌
  * @retval bool: true 为忙碌
  */
bool ST7735_IsBusy(void)
{
    return (hspi1.State == HAL_SPI_STATE_BUSY_TX);
}

// 在 SPI 传输完成中断中拉高 CS
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        ST7735_CS_HIGH(); // 传输完成，拉高片选
    }
}

/**
  * @brief  批量填充颜色（优化版）
  * @param  color: RGB565 颜色值
  * @param  count: 像素数量
  */
static void ST7735_FillColor(uint16_t color, uint32_t count)
{
    uint8_t colorData[2] = {(uint8_t)(color >> 8), (uint8_t)color};
    uint8_t buffer[256]; // 128 像素缓冲区
    
    for (int i = 0; i < 128; i++) {
        buffer[i*2] = colorData[0];
        buffer[i*2+1] = colorData[1];
    }
    
    ST7735_DC_HIGH();
    ST7735_CS_LOW();
    
    while (count >= 128) {
        HAL_SPI_Transmit(&hspi1, buffer, 256, HAL_MAX_DELAY);
        count -= 128;
    }
    if (count > 0) {
        HAL_SPI_Transmit(&hspi1, buffer, count * 2, HAL_MAX_DELAY);
    }
    
    ST7735_CS_HIGH();
}

/* USER CODE END 0 */

/**
  * @brief  ST7735 初始化函数
  * @retval None
  */
void ST7735_Init(void)
{
    // 硬件复位
    ST7735_RES_LOW();
    HAL_Delay(10);
    ST7735_RES_HIGH();
    HAL_Delay(10);
    
    // 软件复位
    ST7735_WriteCommand(ST7735_SWRESET);
    HAL_Delay(120);
    
    // 退出睡眠模式
    ST7735_WriteCommand(ST7735_SLPOUT);
    HAL_Delay(120);
    
    // 帧率控制 - 正常模式
    ST7735_WriteCommand(ST7735_FRMCTR1);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    
    ST7735_WriteCommand(ST7735_FRMCTR2);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    
    // 帧率控制 - 空闲模式
    ST7735_WriteCommand(ST7735_FRMCTR3);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    
    // 显示反转控制
    ST7735_WriteCommand(ST7735_INVCTR);
    ST7735_WriteData(0x07);
    
    // 电源控制
    ST7735_WriteCommand(ST7735_PWCTR1);
    ST7735_WriteData(0xA2);
    ST7735_WriteData(0x02);
    ST7735_WriteData(0x84);
    
    ST7735_WriteCommand(ST7735_PWCTR2);
    ST7735_WriteData(0xC5);
    
    ST7735_WriteCommand(ST7735_PWCTR3);
    ST7735_WriteData(0x0A);
    ST7735_WriteData(0x00);
    
    ST7735_WriteCommand(ST7735_PWCTR4);
    ST7735_WriteData(0x8A);
    ST7735_WriteData(0x2A);
    
    ST7735_WriteCommand(ST7735_PWCTR5);
    ST7735_WriteData(0x8A);
    ST7735_WriteData(0xEE);
    
    // VCOM 控制
    ST7735_WriteCommand(ST7735_VMCTR1);
    ST7735_WriteData(0x0E);
    
    // 关闭反转
    ST7735_WriteCommand(ST7735_INVOFF);
    
    // 内存访问控制
    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteData(0x60);  // 翻转180度横屏: 0x60 (MY=0, MX=1, MV=1, ML=0, RGB=0)
    
    // 像素格式：16 位 RGB565
    ST7735_WriteCommand(ST7735_COLMOD);
    ST7735_WriteData(0x05);
    
    // Gamma 校正
    ST7735_WriteCommand(ST7735_GMCTRP1);
    ST7735_WriteData(0x02);
    ST7735_WriteData(0x1c);
    ST7735_WriteData(0x07);
    ST7735_WriteData(0x12);
    ST7735_WriteData(0x37);
    ST7735_WriteData(0x32);
    ST7735_WriteData(0x29);
    ST7735_WriteData(0x2d);
    ST7735_WriteData(0x29);
    ST7735_WriteData(0x25);
    ST7735_WriteData(0x2B);
    ST7735_WriteData(0x39);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x03);
    ST7735_WriteData(0x10);
    
    ST7735_WriteCommand(ST7735_GMCTRN1);
    ST7735_WriteData(0x03);
    ST7735_WriteData(0x1d);
    ST7735_WriteData(0x07);
    ST7735_WriteData(0x06);
    ST7735_WriteData(0x2E);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x29);
    ST7735_WriteData(0x2D);
    ST7735_WriteData(0x2E);
    ST7735_WriteData(0x2E);
    ST7735_WriteData(0x37);
    ST7735_WriteData(0x3F);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x02);
    ST7735_WriteData(0x10);
    
    // 正常显示模式
    ST7735_WriteCommand(ST7735_NORON);
    HAL_Delay(10);
    
    // 显示开启
    ST7735_WriteCommand(ST7735_DISPON);
    HAL_Delay(10);
    
    // 开启背光
    ST7735_BLK_ON();
}

/**
  * @brief  设置显示窗口
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @retval None
  */
void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // 列地址设置
    ST7735_WriteCommand(ST7735_CASET);
    ST7735_WriteData16(x0 + 0); // 尝试 0 偏移以覆盖边缘
    ST7735_WriteData16(x1 + 0);
    
    // 行地址设置
    ST7735_WriteCommand(ST7735_RASET);
    ST7735_WriteData16(y0 + 0); // 尝试 0 偏移以覆盖边缘
    ST7735_WriteData16(y1 + 0);
    
    // 写入 RAM
    ST7735_WriteCommand(ST7735_RAMWR);
}

/**
  * @brief  填充整个屏幕
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1, color);
}

/**
  * @brief  绘制单个像素点
  * @param  x: X 坐标 (0-127)
  * @param  y: Y 坐标 (0-159)
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) {
        return;  // 超出范围
    }
    
    ST7735_SetWindow(x, y, x, y);
    ST7735_WriteData16(color);
}

/**
  * @brief  绘制直线（使用 Bresenham 算法）
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int16_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int16_t dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;
    uint16_t x = x0;
    uint16_t y = y0;
    
    while (1) {
        ST7735_DrawPixel(x, y, color);
        
        if (x == x1 && y == y1) {
            break;
        }
        
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

/**
  * @brief  绘制矩形边框
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_DrawRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    // 上边
    ST7735_DrawLine(x0, y0, x1, y0, color);
    // 下边
    ST7735_DrawLine(x0, y1, x1, y1, color);
    // 左边
    ST7735_DrawLine(x0, y0, x0, y1, color);
    // 右边
    ST7735_DrawLine(x1, y0, x1, y1, color);
}

/**
  * @brief  填充矩形
  * @param  x0: 起始 X 坐标
  * @param  y0: 起始 Y 坐标
  * @param  x1: 结束 X 坐标
  * @param  y1: 结束 Y 坐标
  * @param  color: RGB565 颜色值
  * @retval None
  */
void ST7735_FillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (x0 > x1) { uint16_t t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { uint16_t t = y0; y0 = y1; y1 = t; }
    if (x1 >= ST7735_WIDTH) x1 = ST7735_WIDTH - 1;
    if (y1 >= ST7735_HEIGHT) y1 = ST7735_HEIGHT - 1;
    
    ST7735_SetWindow(x0, y0, x1, y1);
    ST7735_FillColor(color, (uint32_t)(x1 - x0 + 1) * (y1 - y0 + 1));
}

/**
  * @brief  显示 AuraScope 启动界面
  * @retval None
  */
void ST7735_ShowSplashScreen(void)
{
    // 清屏为黑色
    ST7735_FillScreen(ST7735_BLACK);
    HAL_Delay(100);
    
    // 绘制渐变背景（从深蓝到黑色）
    for (uint16_t y = 0; y < ST7735_HEIGHT; y++) {
        uint16_t blue = (y * 31) / ST7735_HEIGHT;  // 0-31 的蓝色分量
        uint16_t color = blue;  // RGB565: 蓝色
        ST7735_DrawLine(0, y, ST7735_WIDTH - 1, y, color);
    }
    
    HAL_Delay(200);
    
    // 绘制 "AuraScope" 标题（使用线条绘制，简化版本）
    // 绘制 A (在屏幕中央偏上)
    uint16_t startX = 20;
    uint16_t startY = 30;
    ST7735_DrawLine(startX, startY + 20, startX + 5, startY, ST7735_CYAN);
    ST7735_DrawLine(startX + 5, startY, startX + 10, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX + 2, startY + 10, startX + 8, startY + 10, ST7735_CYAN);
    
    // 绘制 u
    startX += 15;
    ST7735_DrawLine(startX, startY, startX, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 20, startX + 8, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX + 8, startY + 20, startX + 8, startY + 10, ST7735_CYAN);
    
    // 绘制 r
    startX += 12;
    ST7735_DrawLine(startX, startY, startX, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 10, startX + 5, startY + 10, ST7735_CYAN);
    ST7735_DrawLine(startX + 5, startY + 10, startX + 5, startY + 20, ST7735_CYAN);
    
    // 绘制 a
    startX += 10;
    ST7735_DrawLine(startX + 2, startY + 10, startX + 5, startY, ST7735_CYAN);
    ST7735_DrawLine(startX + 5, startY, startX + 8, startY + 10, ST7735_CYAN);
    ST7735_DrawLine(startX + 3, startY + 5, startX + 7, startY + 5, ST7735_CYAN);
    ST7735_DrawLine(startX + 2, startY + 10, startX + 8, startY + 20, ST7735_CYAN);
    
    // 绘制 S
    startX += 12;
    ST7735_DrawLine(startX, startY, startX + 8, startY, ST7735_CYAN);
    ST7735_DrawLine(startX, startY, startX, startY + 10, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 10, startX + 8, startY + 10, ST7735_CYAN);
    ST7735_DrawLine(startX + 8, startY + 10, startX + 8, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 20, startX + 8, startY + 20, ST7735_CYAN);
    
    // 绘制 c
    startX += 12;
    ST7735_DrawLine(startX + 2, startY + 5, startX + 2, startY + 15, ST7735_CYAN);
    ST7735_DrawLine(startX + 2, startY + 5, startX + 6, startY + 5, ST7735_CYAN);
    ST7735_DrawLine(startX + 2, startY + 15, startX + 6, startY + 15, ST7735_CYAN);
    
    // 绘制 o
    startX += 10;
    ST7735_DrawLine(startX + 2, startY + 5, startX + 2, startY + 15, ST7735_CYAN);
    ST7735_DrawLine(startX + 2, startY + 5, startX + 6, startY + 5, ST7735_CYAN);
    ST7735_DrawLine(startX + 6, startY + 5, startX + 6, startY + 15, ST7735_CYAN);
    ST7735_DrawLine(startX + 2, startY + 15, startX + 6, startY + 15, ST7735_CYAN);
    
    // 绘制 p
    startX += 10;
    ST7735_DrawLine(startX, startY, startX, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX, startY, startX + 6, startY, ST7735_CYAN);
    ST7735_DrawLine(startX + 6, startY, startX + 6, startY + 10, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 10, startX + 6, startY + 10, ST7735_CYAN);
    
    // 绘制 e
    startX += 10;
    ST7735_DrawLine(startX, startY, startX, startY + 20, ST7735_CYAN);
    ST7735_DrawLine(startX, startY, startX + 6, startY, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 10, startX + 6, startY + 10, ST7735_CYAN);
    ST7735_DrawLine(startX, startY + 20, startX + 6, startY + 20, ST7735_CYAN);
    
    HAL_Delay(300);
    
    // 绘制装饰性波形图案（正弦波样式）
    for (uint16_t x = 10; x < 118; x++) {
        // 简化的正弦波：使用三角波近似
        int16_t y = 100 + (int16_t)(8 * ((x % 32 < 16) ? (x % 32) : (32 - x % 32)) - 64) / 8;
        if (y >= 0 && y < ST7735_HEIGHT) {
            ST7735_DrawPixel(x, y, ST7735_YELLOW);
        }
    }
    
    HAL_Delay(200);
    
    // 绘制版本信息 "v1.0"（简化版本）
    startX = 50;
    startY = 130;
    // 绘制 "v"
    ST7735_DrawLine(startX, startY, startX + 3, startY + 10, ST7735_WHITE);
    ST7735_DrawLine(startX + 3, startY + 10, startX + 6, startY, ST7735_WHITE);
    
    // 绘制 "1"
    startX += 10;
    ST7735_DrawLine(startX + 3, startY, startX + 3, startY + 10, ST7735_WHITE);
    ST7735_DrawLine(startX, startY + 2, startX + 3, startY, ST7735_WHITE);
    
    // 绘制 "."
    startX += 8;
    ST7735_DrawPixel(startX, startY + 8, ST7735_WHITE);
    
    // 绘制 "0"
    startX += 4;
    ST7735_DrawLine(startX, startY, startX + 4, startY, ST7735_WHITE);
    ST7735_DrawLine(startX, startY, startX, startY + 10, ST7735_WHITE);
    ST7735_DrawLine(startX + 4, startY, startX + 4, startY + 10, ST7735_WHITE);
    ST7735_DrawLine(startX, startY + 10, startX + 4, startY + 10, ST7735_WHITE);
    
    HAL_Delay(500);
}

/**
  * @brief  在指定位置显示一个字符
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  ch: 字符
  * @param  color: 颜色
  * @param  bgcolor: 背景色
  * @retval None
  */
void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bgcolor)
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;

    ST7735_SetWindow(x, y, x + 7, y + 15);

    for (uint8_t i = 0; i < 16; i++) {
        uint8_t line = font8x16[(uint8_t)ch][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (0x80 >> j)) {
                ST7735_WriteData16(color);
            } else {
                ST7735_WriteData16(bgcolor);
            }
        }
    }
}

/**
  * @brief  在指定位置显示字符串
  * @param  x: X 坐标
  * @param  y: Y 坐标
  * @param  str: 字符串指针
  * @param  color: 颜色
  * @param  bgcolor: 背景色
  * @retval None
  */
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bgcolor)
{
    while (*str) {
        if (x + 8 > ST7735_WIDTH) {
            x = 0;
            y += 16;
        }
        if (y + 16 > ST7735_HEIGHT) break;

        ST7735_DrawChar(x, y, *str, color, bgcolor);
        x += 8;
        str++;
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
