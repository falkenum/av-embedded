
#include "main.h"

static GPIO_InitTypeDef  GPIO_InitStruct;
static TIM_HandleTypeDef TIM_HandleStruct;
static TIM_OC_InitTypeDef TIM_OC_InitStruct;

void SystemClock_Config(void);
static void Error_Handler(void);

void TIM2_IRQHandler() {
  HAL_TIM_IRQHandler(&TIM_HandleStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void assert_ok(HAL_StatusTypeDef status) {
  if (status != HAL_OK)
    Error_Handler();
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  __HAL_RCC_TIM2_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  TIM_HandleStruct.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_HandleStruct.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_HandleStruct.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  TIM_HandleStruct.Init.Period = 100;
  TIM_HandleStruct.Init.Prescaler = 4500;
  TIM_HandleStruct.Instance = TIM2;

  TIM_OC_InitStruct.OCFastMode = TIM_OCFAST_ENABLE;
  TIM_OC_InitStruct.OCIdleState = TIM_OCIDLESTATE_RESET;
  TIM_OC_InitStruct.OCMode = TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.Pulse = 50;
  TIM_OC_InitStruct.OCPolarity = TIM_OCPOLARITY_HIGH;

  assert_ok(HAL_TIM_PWM_Init(&TIM_HandleStruct));
  assert_ok(HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_1));
  assert_ok(HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_1));

  int delta = 1;
  while (1)
  {
    if (TIM2->CCR1 == 100)
      delta = -1;
    else if (TIM2->CCR1 == 0)
      delta = 1;

    TIM2->CCR1 = TIM2->CCR1 + delta;
    HAL_Delay(10);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows: 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
   /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif