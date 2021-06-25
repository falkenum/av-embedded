
#include "main.h"
#define WINDOW_SIZE 512
#define FFT_SIZE 1024
#define ADC_FIFO_CHUNK_SIZE 256
#define ADC_FIFO_NUM_CHUNKS (WINDOW_SIZE/ADC_FIFO_CHUNK_SIZE)
#define ADC_FIFO_NUM_SAMPLES (ADC_FIFO_NUM_CHUNKS*ADC_FIFO_CHUNK_SIZE)
#define MAX_SAMPLE_MAG_DB 50.0
#define MIN_SAMPLE_MAG_DB (-10.0)
#define MAX_HBEF_DIFF_DB 30.0
#define MIN_HBEF_DIFF_DB 10.0
#define SAMPLING_RATE 45000
#define TIM_CLK_FREQ 90000000
#define FFT_BIN_BANDWIDTH ((float32_t) SAMPLING_RATE / FFT_SIZE)
#define MIN_FREQ 80
#define MAX_FREQ 8000
#define NUM_LEDS 2
#define MAX_BIN ((size_t) (MAX_FREQ / FFT_BIN_BANDWIDTH))
#define MIN_BIN ((size_t) (MIN_FREQ / FFT_BIN_BANDWIDTH))

#define I2C_ADDR 0x7E
#define I2C_TIMEOUT 100

// ADC_HandleTypeDef hadc1;
// DMA_HandleTypeDef hdma_dac1, hdma_dac2;
TIM_HandleTypeDef htim2, htim3;
// UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

// typedef uint16_t adc_sample_t;
// typedef union {
//   adc_sample_t samples[ADC_FIFO_NUM_SAMPLES];
//   adc_sample_t chunks[ADC_FIFO_NUM_CHUNKS][ADC_FIFO_CHUNK_SIZE];
// } adc_data_t;

// typedef struct {
//   adc_data_t data;
//   size_t head_chunk;
// } adc_fifo_t;

// volatile adc_fifo_t adc_fifo;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_DMA_Init(void);
// static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
// static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

// static void ADC_FIFO_Init(void) {
//   adc_fifo.head_chunk = 0;
// }

// void DMA2_Stream0_IRQHandler(void) {
//   HAL_DMA_IRQHandler(&hdma_adc1);
// }

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//   if ((hadc->DMA_Handle->Instance->CR & DMA_SxCR_CT) == RESET) {
//     // if buf 1 is complete, update its address
//     hadc->DMA_Handle->Instance->M1AR = (uint32_t) adc_fifo.data.chunks[adc_fifo.head_chunk];
//   } else {
//     // else buf 0 is complete, update its address
//     hadc->DMA_Handle->Instance->M0AR = (uint32_t) adc_fifo.data.chunks[adc_fifo.head_chunk];
//   }

//   adc_fifo.head_chunk = (adc_fifo.head_chunk + 1) % ADC_FIFO_NUM_CHUNKS;

// }

static void i2c_write(uint8_t addr, uint8_t datum) {
  if (HAL_OK != HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &datum, 1, I2C_TIMEOUT)) {
    Error_Handler();
  };
}

int main(void) {
  // fft_status = PROCESSING;
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  // MX_DMA_Init();
  // MX_ADC1_Init();
  // MX_TIM2_Init();
  // MX_TIM3_Init();
  // MX_USART2_UART_Init();
  MX_I2C1_Init();

  // __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);

  // ADC_FIFO_Init();

  float32_t fft_in[FFT_SIZE];
  float32_t fft_out[FFT_SIZE];
  // float32_t sample_mag[MAX_BIN-MIN_BIN];
  float32_t sample_mag_db[MAX_BIN-MIN_BIN];

  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // HAL_ADC_Start_DoubleBuffer_DMA(&hadc1, (uint32_t*) adc_fifo.data.chunks[0], (uint32_t*) adc_fifo.data.chunks[0], ADC_FIFO_CHUNK_SIZE);

  arm_rfft_fast_instance_f32 fft_struct;
  arm_rfft_fast_init_f32(&fft_struct, FFT_SIZE);
  volatile uint32_t* led_pulse_ptrs[3] = {&htim3.Instance->CCR1, &htim3.Instance->CCR2, &htim3.Instance->CCR3};

  // resetting the led driver
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
  HAL_Delay(100);

  uint8_t mode1 = 0;
  if (HAL_OK != HAL_I2C_Mem_Read(&hi2c1, 0x7E, 0x00, I2C_MEMADD_SIZE_8BIT, &mode1, 1, 100) || 0x09 != mode1) {
    Error_Handler();
  };  

  uint8_t pwm = 0x08;

  // pwmall
  i2c_write(0x3f, pwm);
  // irefall
  i2c_write(0x40, 0x10);

  while (1)
  {
    i2c_write(0x3f, pwm++);
    HAL_Delay(2);

    // static size_t last_head_chunk = 0;

    // // wait for fifo to update
    // while (last_head_chunk == adc_fifo.head_chunk);
    // last_head_chunk = adc_fifo.head_chunk;

    // // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    // float32_t a0 = 25.0/46.0;
    // for (uint32_t i = 0; i < ADC_FIFO_NUM_SAMPLES; i += 1) {
    //   size_t sample_idx = (adc_fifo.head_chunk*ADC_FIFO_CHUNK_SIZE + i) % ADC_FIFO_NUM_SAMPLES;

    //   fft_in[i] = ((float32_t) adc_fifo.data.samples[sample_idx] - 2048) / 4096;

    //   // applying window
    //   fft_in[i] = (a0 - (1-a0)*arm_cos_f32(2*PI*i/ADC_FIFO_NUM_SAMPLES)) * fft_in[i];
    // }

    // // zero padding
    // for (uint32_t i = ADC_FIFO_NUM_SAMPLES; i < FFT_SIZE; i += 1) {
    //   fft_in[i] = 0.0;
    // }

    // arm_rfft_fast_f32(&fft_struct, fft_in, fft_out, 0);

    // // suppress DC component
    // for (size_t i = 0; (i/2)*FFT_BIN_BANDWIDTH < MIN_FREQ; i+=2) {
    //   fft_out[i] = 0.0;
    //   fft_out[i+1] = 0.0;
    // }

    // size_t dominant_bin_for_frame[NUM_LEDS];
    // float32_t highband_energy_factor[NUM_LEDS];
    // static float32_t last_hbef[NUM_LEDS] = {0};

    // for (size_t i = 0; i < NUM_LEDS; i++) {
    //   dominant_bin_for_frame[i] = MIN_BIN;
    //   highband_energy_factor[i] = 0.0;
    //   // last_hbef[i] = 0.0;
    // }

    // for (uint32_t i = MIN_BIN; i < MAX_BIN; i++) {
    //   float32_t sample_raw[2] = {fft_out[i*2], fft_out[i*2+1]};
    //   float32_t sample_mag_sq;
    //   arm_power_f32(sample_raw, 2, &sample_mag_sq);

    //   sample_mag_db[i] = 10*log10f((float) sample_mag_sq + __FLT_EPSILON__);

    //   size_t led_num = NUM_LEDS * (i - MIN_BIN) / (float) (MAX_BIN - MIN_BIN);

    //   if (sample_mag_db[i] > sample_mag_db[dominant_bin_for_frame[led_num]])
    //     dominant_bin_for_frame[led_num] = i;

    //   highband_energy_factor[led_num] += sample_mag_sq * i;
    // }


    // for (size_t led_num = 0; led_num < NUM_LEDS; led_num++) {
    //   float32_t hbef_diff = highband_energy_factor[led_num] - last_hbef[led_num];
    //   float32_t hbef_diff_rect = hbef_diff > 0 ? hbef_diff : 0;
    //   float32_t hbef_diff_db = 10*log10f((float) hbef_diff_rect + __FLT_EPSILON__);
    //   last_hbef[led_num] = highband_energy_factor[led_num];

    //   float32_t curr_sample_mag_db = sample_mag_db[dominant_bin_for_frame[led_num]];

    //   float32_t hbef_pwm_ratio;
    //   if (hbef_diff_db > MAX_HBEF_DIFF_DB)
    //     hbef_pwm_ratio = 1.0;
    //   else if (hbef_diff_db < MIN_HBEF_DIFF_DB)
    //     hbef_pwm_ratio = 0.0;
    //   else
    //     hbef_pwm_ratio = hbef_diff_db / MAX_HBEF_DIFF_DB;

    //   float32_t sample_pwm_ratio;
    //   if (curr_sample_mag_db > MAX_SAMPLE_MAG_DB)
    //     sample_pwm_ratio = 1.0;
    //   else if (curr_sample_mag_db < MIN_SAMPLE_MAG_DB)
    //     sample_pwm_ratio = 0.0;
    //   else
    //     // need to do this because min db is negative
    //     sample_pwm_ratio = (curr_sample_mag_db - MIN_SAMPLE_MAG_DB) / (MAX_SAMPLE_MAG_DB-MIN_SAMPLE_MAG_DB);

    //   float32_t max_pwm_ratio = hbef_pwm_ratio > sample_pwm_ratio ? hbef_pwm_ratio : sample_pwm_ratio;
    //   *(led_pulse_ptrs[led_num]) = (uint32_t) ((1 - max_pwm_ratio) * htim3.Instance->ARR);
    // }

    // // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    // // HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, sizeof(float32_t)*FFT_SIZE/2, 0xFFFFFFFF);
  }
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
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_ADC1_Init(void)
// {
//   __HAL_RCC_ADC1_CLK_ENABLE();

//   ADC_ChannelConfTypeDef sConfig = {0};

//   hadc1.Instance = ADC1;
//   hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
//   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//   hadc1.Init.ScanConvMode = DISABLE;
//   hadc1.Init.ContinuousConvMode = DISABLE;
//   hadc1.Init.DiscontinuousConvMode = DISABLE;
//   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//   hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
//   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//   hadc1.Init.NbrOfConversion = 1;
//   hadc1.Init.DMAContinuousRequests = ENABLE;
//   hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//   if (HAL_ADC_Init(&hadc1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//   */
//   sConfig.Channel = ADC_CHANNEL_0;
//   sConfig.Rank = 1;
//   sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM_CLK_FREQ / SAMPLING_RATE;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim2.Init.Period / 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

static void MX_TIM3_Init(void)
{

  __HAL_RCC_TIM3_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  // sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  // sConfigOC.Pulse = 1800;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }


}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_USART2_UART_Init(void)
// {

//     __HAL_RCC_USART2_CLK_ENABLE();
//     /**USART2 GPIO Configuration
//     PA2     ------> USART2_TX
//     PA3     ------> USART2_RX
//     */
//   /* USER CODE BEGIN USART2_Init 0 */

//   /* USER CODE END USART2_Init 0 */

//   /* USER CODE BEGIN USART2_Init 1 */

//   /* USER CODE END USART2_Init 1 */
//   huart2.Instance = USART2;
//   huart2.Init.BaudRate = 115200;
//   huart2.Init.WordLength = UART_WORDLENGTH_8B;
//   huart2.Init.StopBits = UART_STOPBITS_1;
//   huart2.Init.Parity = UART_PARITY_NONE;
//   huart2.Init.Mode = UART_MODE_TX_RX;
//   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//   if (HAL_UART_Init(&huart2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN USART2_Init 2 */

//   /* USER CODE END USART2_Init 2 */

// }

/**
  * Enable DMA controller clock
  */
// static void MX_DMA_Init(void)
// {

//   /* DMA controller clock enable */
//   __HAL_RCC_DMA2_CLK_ENABLE();

//   /* DMA interrupt init */
//   /* DMA2_Stream0_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

//     /* ADC1 DMA Init */
//     /* ADC1 Init */
//     hdma_adc1.Instance = DMA2_Stream0;
//     hdma_adc1.Init.Channel = DMA_CHANNEL_0;
//     hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//     hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//     hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//     hdma_adc1.Init.Mode = DMA_CIRCULAR;
//     hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
//     hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
//     {
//       Error_Handler();
//     }
// }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  // __HAL_RCC_GPIOC_CLK_ENABLE();
  // __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**ADC1 GPIO Configuration
  PA0-WKUP     ------> ADC1_IN0
  */
  // GPIO_InitStruct.Pin = GPIO_PIN_0;
  // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);

  // GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

static void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
