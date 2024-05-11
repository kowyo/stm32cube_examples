/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_2.8_lcd.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KP 2
#define TI 50
#define TD 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t hall_a = 0;
uint8_t hall_b = 0;
uint8_t hall_c = 0;
uint16_t duty_cycle = 0;
uint8_t key_value = 0;
uint8_t isInterruptEnabled = 0;
uint8_t isBreakEnabled = 0;
uint8_t isTimerStarted = 0;
uint8_t HallStatus = 0;

uint32_t speed_counter = 0;
uint32_t speed_timer = 0;

uint16_t output = 0;

double actual_vel = 0;
double target_vel = 0;
double voltage = 0;
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
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  STM32_LCD_Init();
  LCD_Clear(BackColor);
  LCD_SetTextColor(Blue);
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 50);
    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc3), HAL_ADC_STATE_REG_EOC))
    {
      // duty_cycle = HAL_ADC_GetValue(&hadc3) * 8399 / 4095;
      target_vel = HAL_ADC_GetValue(&hadc3) * 4600 / 4095;
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 50);
    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
      voltage = HAL_ADC_GetValue(&hadc1) * 3.3 / 4095;
      voltage = voltage * (7.2 + 560 + 560) / 7.2;
    }

    LCD_DisplayStringLine(Line0, "Target Speed: ");
    LCD_Draw_NUM(0, 140, target_vel);
    LCD_DisplayStringLine(Line1, "Key Value: ");
    LCD_Draw_NUM(25, 140, key_value);
    LCD_DisplayStringLine(Line2, "Break: ");
    LCD_Draw_NUM(50, 140, isBreakEnabled);
    LCD_DisplayStringLine(Line3, "Hall Status: ");
    LCD_Draw_NUM(75, 140, HallStatus);
    LCD_DisplayStringLine(Line4, "Speed: ");
    LCD_Draw_NUM(100, 140, actual_vel);
    LCD_DisplayStringLine(Line5, "Voltage: ");
    LCD_Draw_NUM(125, 140, voltage);
    LCD_DisplayStringLine(Line6, "isTimerStarted: ");
    LCD_Draw_NUM(150, 140, isTimerStarted);
    LCD_DisplayStringLine(Line7, "PID Output: ");
    LCD_Draw_NUM(175, 140, output);
    
    // if (HAL_GPIO_ReadPin(KEY_SEL_GPIO_Port, KEY_SEL_Pin) == GPIO_PIN_RESET)
    // {
    //   HAL_Delay(50);
    //   if (HAL_GPIO_ReadPin(KEY_SEL_GPIO_Port, KEY_SEL_Pin) == GPIO_PIN_RESET)
    //   {
    //     key_value = !key_value;
    //   }
    // }
    // if (key_value == 1)
    // {
    //   Set_Speed(duty_cycle);
    // }
    // else
    // {
    //   HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    //   HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    //   HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    // }
    
    output = pid_controller(target_vel, actual_vel, KP, TI, TD);
    Set_Speed(output);
    printf("%d,%d\n", (int)actual_vel, (int)target_vel);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    speed_counter++;
    isInterruptEnabled = 1;
    // Read the status of the hall sensors
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
    {
      hall_a = 1;
    }
    else
    {
      hall_a = 0;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET)
    {
      hall_b = 1;
    }
    else
    {
      hall_b = 0;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
    {
      hall_c = 1;
    }
    else
    {
      hall_c = 0;
    }

    // Phase change based on the hall sensor status
    if (hall_a == 1 && hall_b == 0 && hall_c == 1)
    {
      HallStatus = 1;
      // Timer 8
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // TIM8_CH1_ON;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);  // TIM8_CH2_OFF;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);  // TIM8_CH3_OFF;
      // GPIO
      HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_RESET); // UL_OFF;
      HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_SET);   // VL_ON;
      HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_RESET); // WL_OFF;
    }
    if (hall_a == 1 && hall_b == 0 && hall_c == 0)
    {
      HallStatus = 2;
      // Timer 8
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // TIM8_CH1_ON;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);  // TIM8_CH2_OFF;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);  // TIM8_CH3_OFF;
      // GPIO
      HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_RESET); // UL_OFF;
      HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_RESET); // VL_OFF;
      HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_SET);   // WL_ON;
    }
    if (hall_a == 1 && hall_b == 1 && hall_c == 0)
    {
      HallStatus = 3;
      // Timer 8
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);  // TIM8_CH1_OFF;
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // TIM8_CH2_ON;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);  // TIM8_CH3_OFF;
      // GPIO
      HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_RESET); // UL_OFF;
      HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_RESET); // VL_OFF;
      HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_SET);   // WL_ON;
    }
    if (hall_a == 0 && hall_b == 1 && hall_c == 0)
    {
      HallStatus = 4;
      // Timer 8
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);  // TIM8_CH1_OFF;
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // TIM8_CH2_ON;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);  // TIM8_CH3_OFF;
      // GPIO
      HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_SET);   // UL_ON;
      HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_RESET); // VL_OFF;
      HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_RESET); // WL_OFF;
    }
    if (hall_a == 0 && hall_b == 1 && hall_c == 1)
    {
      HallStatus = 5;
      // Timer 8
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);  // TIM8_CH1_OFF;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);  // TIM8_CH2_OFF;
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // TIM8_CH3_ON;
      // GPIO
      HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_SET);   // UL_ON;
      HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_RESET); // VL_OFF;
      HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_RESET); // WL_OFF;
    }
    if (hall_a == 0 && hall_b == 0 && hall_c == 1)
    {
      HallStatus = 6;
      // Timer 8
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);  // TIM8_CH1_OFF;
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);  // TIM8_CH2_OFF;
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // TIM8_CH3_ON;
      // GPIO
      HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_RESET); // UL_OFF;
      HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_SET);   // VL_ON;
      HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_RESET); // WL_OFF;
    }
  }
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8)
  {
    isBreakEnabled = 1;
    // TIM8_CH1_OFF;
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    // TIM8_CH2_OFF;
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    // TIM8_CH3_OFF;
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    // UL_OFF;
    HAL_GPIO_WritePin(UL_GPIO_Port, UL_Pin, GPIO_PIN_RESET);
    // VL_OFF;
    HAL_GPIO_WritePin(VL_GPIO_Port, VL_Pin, GPIO_PIN_RESET);
    // WL_OFF;
    HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, GPIO_PIN_RESET);

    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    isTimerStarted = 1;
    speed_timer++;
    if (speed_timer >= 50)
    {
      actual_vel = speed_counter * 60.0 * 60.0 / 360.0 / 4.0 / (50 * 0.001);
      speed_counter = 0;
      speed_timer = 0;
    }
  }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if (huart->Instance == USART1)
//   {
//     HAL_UART_Receive_IT(&huart1, &key_value, 1);
//   }
// }

void Set_Speed(uint16_t dutycycle)
{
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, dutycycle);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, dutycycle);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, dutycycle);
}
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

#ifdef  USE_FULL_ASSERT
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
