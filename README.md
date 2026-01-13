# AuraScope (STM32F103C8T6 示波器)

AuraScope 是一个基于 STM32F103C8T6 微控制器的简易数字示波器项目。它利用 STM32 内部 ADC 进行信号采集，并通过 1.8 英寸 TFT 屏幕显示波形。

## 1. 核心硬件参数

- **MCU**: STM32F103C8T6 (64KB Flash, 20KB SRAM)
- **内核**: ARM Cortex-M3 (72MHz)
- **显示屏**: 1.8" TFT LCD (ST7735 驱动, 128x160 分辨率)
- **开发库**: STM32Cube HAL 库

## 2. 硬件引脚分配 (Pinout)

### 显示屏接口 (ST7735)
| 信号 | 引脚 | 描述 |
| --- | --- | --- |
| SCL (SCK) | PA5 | SPI 时钟 |
| SDA (MOSI) | PA7 | SPI 数据 |
| RES | PB5 | 复位 |
| DC | PB6 | 数据/命令选择 |
| CS | PB7 | 片选 |
| BLK | PB8 | 背光控制 |

### 信号采集 (ADC)
- **CH1 输入**: PA3 (ADC1)
- **CH2 输入**: PA4 (ADC2)
- **采样模式**: 双 ADC 同步规则采样 (Dual Regular Simultaneous Mode)
- **测试信号输出**: PA2 (PWM)

### 频率检测 (频率计)
通过外部滞回比较器将模拟信号转换为方波，利用定时器捕获周期。
- **CH1 频率捕获**: PA6 (TIM3_CH1)
- **CH2 频率捕获**: PA1 (TIM2_CH2)

### 用户交互
- **按键**:
  - KEY1: PB13
  - KEY2: PB14
  - KEY3: PB15
  - KEY4: PA8
- **EC11 旋转编码器**:
  - A相: PB4
  - B相: PB3
  - 按键: PB9
- **LED 指示灯**:
  - LED1: PC14
  - LED2: PC15

### 通讯接口
- **USB**: PA11/PA12 (USB CDC 虚拟串口)
- **UART1**: PA9 (TX), PA10 (RX)
- **UART3**: PB10 (TX), PB11 (RX)

## 3. 开发与构建

本项目使用 **STM32CubeMX** 生成初始化代码，并结合 **CMake** 进行构建。

### 目录结构
- `Core/`: 核心业务逻辑与 STM32 初始化代码
- `Drivers/`: STM32 HAL 库与 CMSIS
- `Hardware/`: 板载外设驱动 (如 ST7735, UI 逻辑)
- `cmake/`: CMake 工具链配置

### 构建说明
本项目依赖 STM32 VS Code Extension 生成的 `CMakePresets.json`。
建议使用 VS Code 配合 STM32 扩展进行开发与调试。

## 4. 软件特性
- **DMA 传输**: ADC 采样与部分外设通信使用 DMA 以降低 CPU 负载。
- **定点运算**: 核心波形处理优先使用定点数，优化性能。
- **硬件 SPI**: 屏幕驱动使用硬件 SPI 接口，提升刷新率。

## 5. 许可证
[License Information]
