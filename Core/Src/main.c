/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "st7735.h"
#include "ui.h"
#include "signal_gen.h"
#include "osc_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t key_pressed = 0;
volatile int8_t encoder_delta = 0;
volatile uint32_t last_button_press_time = 0;

float osc_vpp_ch1 = 0.0f;
float osc_vpp_ch2 = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  MX_TIM1_Init();

  // 初始化 ST7735 显示屏
  ST7735_Init();
  HAL_Delay(100);
  
  // 显示启动界面 (可选)
  // ST7735_ShowSplashScreen();
  // HAL_Delay(1000);
  
  // 初始化 UI
  UI_Init();

  // 初始化示波器核心
  OSC_Init();
  
  // 上电后等待硬件信号稳定，防止误触发
  HAL_Delay(200);

  // Start Peripherals
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  
  // Start Frequency Measurement (Input Capture)
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // CH1 Freq
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); // CH2 Freq
  // Note: TIM2 Base IT is started by SignalGen_Init -> SignalGen_ConfigHardware
  // TIM3 Base IT needs to be started for overflow counting
  HAL_TIM_Base_Start_IT(&htim3);

  // Start Oscilloscope ADC
  OSC_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Process Oscilloscope Data (Trigger, Scaling, Vpp)
    OSC_Process();
    UI_Refresh();

    // 如果没有外部交互且波形刚处理完，可以适当减小延迟以提升帧率
    // 但为了保证按键响应，这里保留一个极小的延迟
    if (key_pressed) {
        UI_HandleKey(key_pressed);
        key_pressed = 0;
    }
    if (encoder_delta != 0) {
        UI_HandleEncoder(encoder_delta);
        encoder_delta = 0;
        UI_Refresh(); // Encoder changes params, need refresh
    }
    
    HAL_Delay(5); // 提升帧率限制到 ~200fps (实际受限于 SPI/USB 阻塞)
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Forward declaration
void OSC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void OSC_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();
    
    // 按键消抖与处理
    if (GPIO_Pin == GPIO_PIN_13 || GPIO_Pin == GPIO_PIN_14 || 
        GPIO_Pin == GPIO_PIN_15 || GPIO_Pin == GPIO_PIN_8 || GPIO_Pin == GPIO_PIN_9) {
            
        if (current_time - last_button_press_time > 200) { // 200ms debounce
            last_button_press_time = current_time;
            if (GPIO_Pin == GPIO_PIN_13) key_pressed = 1;      // KEY1
            else if (GPIO_Pin == GPIO_PIN_14) key_pressed = 2; // KEY2
            else if (GPIO_Pin == GPIO_PIN_15) key_pressed = 3; // KEY3
            else if (GPIO_Pin == GPIO_PIN_8) key_pressed = 4;  // KEY4
            else if (GPIO_Pin == GPIO_PIN_9) key_pressed = 5;  // EC11 Button
        }
    }
    
    // 旋转编码器处理 (PB4: A, PB3: B)
    // 假设 GPIO_MODE_IT_FALLING
    else if (GPIO_Pin == GPIO_PIN_4) { // A Falling
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
            encoder_delta = 1; // CW
        } else {
            encoder_delta = -1; // CCW
        }
    }
    else if (GPIO_Pin == GPIO_PIN_3) { // B Falling
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) {
            encoder_delta = -1; // CCW
        } else {
            encoder_delta = 1; // CW
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        SignalGen_TIM_PeriodElapsedCallback(htim);
        OSC_TIM_PeriodElapsedCallback(htim);
    }
    else if (htim->Instance == TIM3) {
        OSC_TIM_PeriodElapsedCallback(htim);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    OSC_TIM_IC_CaptureCallback(htim);
}

// HAL_ADC_ConvCpltCallback moved to osc_app.c

/* USER CODE BEGIN 4_Init */
/* USER CODE END 4_Init */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
