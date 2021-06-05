// #include "main.h"
// #define FFT_SIZE 1024

// TIM_HandleTypeDef htim2 = {0};
// UART_HandleTypeDef huart2 = {0};
// ADC_HandleTypeDef hadc1 = {0};
// DMA_HandleTypeDef hdma2 = {0};

// uint32_t adc_values[FFT_SIZE] = {0};
// arm_rfft_fast_instance_f32 fft_struct;

// void SystemClock_Config(void);
// static void GPIO_Init(void);
// static void TIM2_Init(void);
// static void ADC1_Init(void);
// static void UART_Init(void);
// static void DMA_Init(void);
// // static void ADC_Start_DMA_MultiBuffer(ADC_HandleTypeDef* hadc);
// // static void process(uint8_t buff_idx);

// void DMA2_Stream0_IRQHandler() {
//   HAL_DMA_IRQHandler(&hdma2);
// }

// // void XferCpltCallback(DMA_HandleTypeDef* hdma) {
// //   process(0);
// // }

// // void XferM1CpltCallback(DMA_HandleTypeDef* hdma) {
// //   process(1);
// // }

// // void XferErrorCallback(DMA_HandleTypeDef* hdma) {
// //   Error_Handler();
// // }

// int main(void)
// {
//   HAL_Init();
//   SystemClock_Config();
  
//   __HAL_LINKDMA(&hadc1, DMA_Handle, hdma2);

//   GPIO_Init();
//   TIM2_Init();
//   DMA_Init();
//   ADC1_Init();
//   UART_Init();
  

//   // float32_t uart_buf[FFT_SIZE];
//   // float32_t sample_buf[FFT_SIZE];
//   // float32_t fft_out[FFT_SIZE];

//   // arm_rfft_fast_init_f32(&fft_struct, FFT_SIZE);
//   HAL_ADC_Start_DMA(&hadc1, adc_values, FFT_SIZE);
//   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//   while (1)
//   {

//     // HAL_UART_Receive(&huart2, (uint8_t*) uart_buf, sizeof(float32_t)*FFT_SIZE, 0xFFFFFFFF);
//     // for (uint32_t i = FFT_SIZE/2; i < FFT_SIZE; i++) {
//     //   uart_buf[i] = 0;
//     // }

//     // uint32_t start = HAL_GetTick();
//     // for (uint32_t i = 0; i < FFT_SIZE; i += 1) {
//     //   sample_buf[i] = uart_buf[i];

//     //   // applying window
//     //   float32_t a0 = 25/46;
//     //   sample_buf[i] = (a0 - (1-a0)*arm_cos_f32(2*PI*i/FFT_SIZE)) * sample_buf[i];
//     // }

//     // arm_rfft_fast_f32(&fft_struct, sample_buf, fft_out, 0);

//     // for (uint32_t i = 0; i < FFT_SIZE/2; i++) {
//     //   float32_t sample_raw[2] = {fft_out[i*2], fft_out[i*2+1]};
//     //   float32_t sample_temp, sample_mag_db;
//     //   arm_power_f32(sample_raw, 2, &sample_temp);
//     //   sample_mag_db = 10*log10f((float) sample_temp);
//     //   uart_buf[i] = sample_mag_db;

//     // }
//     // uint32_t elapsed = HAL_GetTick() - start;
//     // HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, sizeof(float32_t)*FFT_SIZE/2, 0xFFFFFFFF);
//     HAL_Delay(0);
//   }
// }

// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
//   RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//   RCC_OscInitStruct.PLL.PLLM = 8;
//   RCC_OscInitStruct.PLL.PLLN = 180;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//   RCC_OscInitStruct.PLL.PLLQ = 2;
//   RCC_OscInitStruct.PLL.PLLR = 2;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Activate the Over-Drive mode
//   */
//   if (HAL_PWREx_EnableOverDrive() != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// static void TIM2_Init(void)
// {
//   __HAL_RCC_TIM2_CLK_ENABLE();

//   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//   TIM_MasterConfigTypeDef sMasterConfig = {0};
//   TIM_OC_InitTypeDef sConfigOC = {0};

//   htim2.Instance = TIM2;
//   htim2.Init.Prescaler = 100;
//   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim2.Init.Period = 10;
//   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//   if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//   if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sConfigOC.OCMode = TIM_OCMODE_PWM1;
//   sConfigOC.Pulse = 5;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
// }

// static void GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   __HAL_RCC_GPIOA_CLK_ENABLE();

//   /*Configure GPIO pin : LD2_Pin */
//   GPIO_InitStruct.Pin = LD2_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//   HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

//   // ADC input pin
//   GPIO_InitStruct.Pin = GPIO_PIN_0;
//   GPIO_InitStruct.Mode = MODE_ANALOG;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   // UART pins
//   GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//   GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
// }

// void Error_Handler(void)
// {
//   __disable_irq();
//   while (1)
//   {
//   }
// }

// static void ADC1_Init(void) {
//   __HAL_RCC_ADC1_CLK_ENABLE();
//   ADC_ChannelConfTypeDef sConfig = {0};

//   hadc1.Instance = ADC1;
//   hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
//   hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//   hadc1.Init.ScanConvMode = DISABLE;
//   hadc1.Init.ContinuousConvMode = ENABLE;
//   hadc1.Init.DiscontinuousConvMode = DISABLE;
//   hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//   hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
//   hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//   hadc1.Init.NbrOfConversion = 1;
//   hadc1.Init.DMAContinuousRequests = ENABLE;
//   hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

//   if (HAL_ADC_Init(&hadc1) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   sConfig.Channel = ADC_CHANNEL_0;
//   sConfig.Rank = 1;
//   sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }


// static void UART_Init(void)
// {
//   __HAL_RCC_USART2_CLK_ENABLE();

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

// }

// static void DMA_Init(void) {
//   /* DMA controller clock enable */
//   __HAL_RCC_DMA2_CLK_ENABLE();
//   hdma2.Instance = DMA2_Stream0;
//   hdma2.Init.Direction = DMA_PERIPH_TO_MEMORY;
//   hdma2.Init.MemInc = DMA_MINC_ENABLE;
//   hdma2.Init.Mode = DMA_CIRCULAR;
//   hdma2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//   hdma2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//   hdma2.Init.PeriphInc = DMA_PINC_DISABLE;
  
//   if (HAL_DMA_Init(&hdma2) != HAL_OK) {
//     Error_Handler();
//   }

//   HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
//   // HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

//   // HAL_DMA_RegisterCallback(&hdma2, HAL_DMA_XFER_CPLT_CB_ID, XferCpltCallback);
//   // HAL_DMA_RegisterCallback(&hdma2, HAL_DMA_XFER_M1CPLT_CB_ID, XferM1CpltCallback);
//   // HAL_DMA_RegisterCallback(&hdma2, HAL_DMA_XFER_ERROR_CB_ID, XferErrorCallback);
// }

// // static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma)   
// // {
// //   /* Retrieve ADC handle corresponding to current DMA handle */
// //   ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  
// //   /* Update state machine on conversion status if not in error state */
// //   if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
// //   {
// //     /* Update ADC state machine */
// //     SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);
    
// //     /* Determine whether any further conversion upcoming on group regular   */
// //     /* by external trigger, continuous mode or scan sequence on going.      */
// //     /* Note: On STM32F4, there is no independent flag of end of sequence.   */
// //     /*       The test of scan sequence on going is done either with scan    */
// //     /*       sequence disabled or with end of conversion flag set to        */
// //     /*       of end of sequence.                                            */
// //     if(ADC_IS_SOFTWARE_START_REGULAR(hadc)                   &&
// //        (hadc->Init.ContinuousConvMode == DISABLE)            &&
// //        (HAL_IS_BIT_CLR(hadc->Instance->SQR1, ADC_SQR1_L) || 
// //         HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_EOCS)  )   )
// //     {
// //       /* Disable ADC end of single conversion interrupt on group regular */
// //       /* Note: Overrun interrupt was enabled with EOC interrupt in          */
// //       /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
// //       /* by overrun IRQ process below.                                      */
// //       __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);
      
// //       /* Set ADC state */
// //       CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);   
      
// //       if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
// //       {
// //         SET_BIT(hadc->State, HAL_ADC_STATE_READY);
// //       }
// //     }
    
// //     /* Conversion complete callback */
// // #if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
// //     hadc->ConvCpltCallback(hadc);
// // #else
// //     HAL_ADC_ConvCpltCallback(hadc);
// // #endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
// //   }
// //   else /* DMA and-or internal error occurred */
// //   {
// //     if ((hadc->State & HAL_ADC_STATE_ERROR_INTERNAL) != 0UL)
// //     {
// //       /* Call HAL ADC Error Callback function */
// // #if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
// //       hadc->ErrorCallback(hadc);
// // #else
// //       HAL_ADC_ErrorCallback(hadc);
// // #endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
// //     }
// // 	else
// // 	{
// //       /* Call DMA error callback */
// //       hadc->DMA_Handle->XferErrorCallback(hdma);
// //     }
// //   }
// // }

// // /**
// //   * @brief  DMA half transfer complete callback. 
// //   * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
// //   *                the configuration information for the specified DMA module.
// //   * @retval None
// //   */
// // static void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma)   
// // {
// //   ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
// //    /* Half conversion callback */
// // #if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
// //   hadc->ConvHalfCpltCallback(hadc);
// // #else
// //   HAL_ADC_ConvHalfCpltCallback(hadc);
// // #endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
// // }

// // /**
// //   * @brief  DMA error callback 
// //   * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
// //   *                the configuration information for the specified DMA module.
// //   * @retval None
// //   */
// // static void ADC_DMAError(DMA_HandleTypeDef *hdma)   
// // {
// //   ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
// //   hadc->State= HAL_ADC_STATE_ERROR_DMA;
// //   /* Set ADC error code to DMA error */
// //   hadc->ErrorCode |= HAL_ADC_ERROR_DMA;
// //    /* Error callback */
// // #if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
// //   hadc->ErrorCallback(hadc);
// // #else
// //   HAL_ADC_ErrorCallback(hadc);
// // #endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
// // }

// // void ADC_Start_DMA_MultiBuffer(ADC_HandleTypeDef* hadc)
// // {
// //   __IO uint32_t counter = 0U;
// //   ADC_Common_TypeDef *tmpADC_Common;
  
// //   /* Check the parameters */
// //   assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
// //   assert_param(IS_ADC_EXT_TRIG_EDGE(hadc->Init.ExternalTrigConvEdge)); 
  
// //   /* Process locked */
// //   __HAL_LOCK(hadc);
  
// //   /* Enable the ADC peripheral */
// //   /* Check if ADC peripheral is disabled in order to enable it and wait during 
// //   Tstab time the ADC's stabilization */
// //   if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
// //   {  
// //     /* Enable the Peripheral */
// //     __HAL_ADC_ENABLE(hadc);
    
// //     /* Delay for ADC stabilization time */
// //     /* Compute number of CPU cycles to wait for */
// //     counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
// //     while(counter != 0U)
// //     {
// //       counter--;
// //     }
// //   }
  
// //   /* Check ADC DMA Mode                                                     */
// //   /* - disable the DMA Mode if it is already enabled                        */
// //   if((hadc->Instance->CR2 & ADC_CR2_DMA) == ADC_CR2_DMA)
// //   {
// //     CLEAR_BIT(hadc->Instance->CR2, ADC_CR2_DMA);
// //   }
  
// //   /* Start conversion if ADC is effectively enabled */
// //   if(HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON))
// //   {
// //     /* Set ADC state                                                          */
// //     /* - Clear state bitfield related to regular group conversion results     */
// //     /* - Set state bitfield related to regular group operation                */
// //     ADC_STATE_CLR_SET(hadc->State,
// //                       HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
// //                       HAL_ADC_STATE_REG_BUSY);
    
// //     /* If conversions on group regular are also triggering group injected,    */
// //     /* update ADC state.                                                      */
// //     if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
// //     {
// //       ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);  
// //     }
    
// //     /* State machine update: Check if an injected conversion is ongoing */
// //     if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
// //     {
// //       /* Reset ADC error code fields related to conversions on group regular */
// //       CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));         
// //     }
// //     else
// //     {
// //       /* Reset ADC all error code fields */
// //       ADC_CLEAR_ERRORCODE(hadc);
// //     }

// //     /* Process unlocked */
// //     /* Unlock before starting ADC conversions: in case of potential           */
// //     /* interruption, to let the process to ADC IRQ Handler.                   */
// //     __HAL_UNLOCK(hadc);   

// //     /* Pointer to the common control register to which is belonging hadc    */
// //     /* (Depending on STM32F4 product, there may be up to 3 ADCs and 1 common */
// //     /* control register)                                                    */
// //     tmpADC_Common = ADC_COMMON_REGISTER(hadc);

// //     /* Set the DMA transfer complete callback */
// //     hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

// //     /* Set the DMA half transfer complete callback */
// //     hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;
    
// //     /* Set the DMA error callback */
// //     hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;

    
// //     /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
// //     /* start (in case of SW start):                                           */
    
// //     /* Clear regular group conversion flag and overrun flag */
// //     /* (To ensure of no unknown state from potential previous ADC operations) */
// //     __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);

// //     /* Enable ADC overrun interrupt */
// //     __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);
    
// //     /* Enable ADC DMA mode */
// //     hadc->Instance->CR2 |= ADC_CR2_DMA;
    
// //     /* Start the DMA channel */
// //     HAL_DMAEx_MultiBufferStart(hadc->DMA_Handle, (uint32_t) &hadc->Instance->DR, (uint32_t) &adc_values[0], (uint32_t) &adc_values[1], FFT_SIZE);
    
// //     /* Check if Multimode enabled */
// //     if(HAL_IS_BIT_CLR(tmpADC_Common->CCR, ADC_CCR_MULTI))
// //     {
// //         /* if no external trigger present enable software conversion of regular channels */
// //         if((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET) 
// //         {
// //           /* Enable the selected ADC software conversion for regular group */
// //           hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
// //         }
// //     }
// //     else
// //     {
// //       /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
// //       if((hadc->Instance == ADC1) && ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
// //       {
// //         /* Enable the selected ADC software conversion for regular group */
// //           hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
// //       }
// //     }
// //   }
// //   else
// //   {
// //     /* Update ADC state machine to error */
// //     SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

// //     /* Set ADC error code to ADC IP internal error */
// //     SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
// //   }
// // }

// // static void process(uint8_t buff_idx) {
// //   uint32_t max = 0;
// //   uint32_t min = 4095;

// //   for (int i = 0; i < FFT_SIZE; i++) {
// //     uint32_t value = adc_values[buff_idx][i];

// //     if (value > max)
// //       max = value;
// //     if (value < min)
// //       min = value;

// //   }
// //   // HAL_Delay((min + max) / UINT32_MAX);
// //   // for (uint32_t i = 0; i < FFT_SIZE; i += 1) {
// //   //   sample_buf[i] = uart_buf[i];

// //   //   // applying window
// //   //   float32_t a0 = 25/46;
// //   //   sample_buf[i] = (a0 - (1-a0)*arm_cos_f32(2*PI*i/FFT_SIZE)) * sample_buf[i];
// //   // }

// //   // arm_rfft_fast_f32(&fft_struct, sample_buf, fft_out, 0);

// //   // for (uint32_t i = 0; i < FFT_SIZE/2; i++) {
// //   //   float32_t sample_raw[2] = {fft_out[i*2], fft_out[i*2+1]};
// //   //   float32_t sample_temp, sample_mag_db;
// //   //   arm_power_f32(sample_raw, 2, &sample_temp);
// //   //   sample_mag_db = 10*log10f((float) sample_temp);
// //   //   uart_buf[i] = sample_mag_db;

// //   // }
// //   // uint32_t elapsed = HAL_GetTick() - start;
// //   // HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, sizeof(float32_t)*FFT_SIZE/2, 0xFFFFFFFF);
// // }

#include "main.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  static uint32_t adc_data[1024] = {0};
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, adc_data, 1024);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    HAL_Delay(1);
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
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
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
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
