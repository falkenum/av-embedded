
#include "main.h"
#define FFT_SIZE 1024

TIM_HandleTypeDef htim2 = {0};
UART_HandleTypeDef huart2 = {0};
ADC_HandleTypeDef hadc1 = {0};

void SystemClock_Config(void);
static void GPIO_Init(void);
static void TIM2_Init(void);
static void ADC1_Init(void);
static void UART_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  GPIO_Init();
  // TIM2_Init();
  // ADC1_Init();
  UART_Init();

  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  char rx_buf[100] = {0};
  char tx_buf[100] = {0};
  float32_t sample_buf[FFT_SIZE];
  float32_t fft_out[FFT_SIZE];

  arm_rfft_fast_instance_f32 fft_struct;
  arm_rfft_fast_init_f32(&fft_struct, FFT_SIZE);

  while (1)
  {
    // HAL_ADC_Start(&hadc1);
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, HAL_ADC_GetValue(&hadc1));

    for (uint32_t i = 0; i < FFT_SIZE; i++) {
      HAL_UART_Receive(&huart2, (uint8_t*) rx_buf, 3, 0xFFFFFFFF);
      uint16_t sample_as_uint = ((uint16_t)(rx_buf[0]) << 8) | rx_buf[1];
      sample_buf[i] = (sample_as_uint - INT16_MAX) / (float32_t) INT16_MAX;
    }

    arm_rfft_fast_f32(&fft_struct, sample_buf, fft_out, 0);

    for (uint32_t i = 0; i < FFT_SIZE/2; i++) {
      float32_t sample_raw[2] = {fft_out[i*2], fft_out[i*2+1]};
      float32_t sample_temp, sample_mag, sample_mag_db;
      arm_power_f32(sample_raw, 2, &sample_temp);
      static const float32_t eps = 0.000001;
      sample_mag_db = 10*log10f((float)(sample_temp + eps));
      *((float32_t*) tx_buf) = sample_mag_db;

      tx_buf[4] = ',';
      HAL_UART_Transmit(&huart2, (uint8_t*) tx_buf, 5, 0xFFFFFFFF);
    }
    HAL_Delay(1);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

static void TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1<<8;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 80;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  // ADC input pin
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // UART pins
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

static void ADC1_Init(void) {
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION8b;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  HAL_ADC_Init(&hadc1);
}

static void UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

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

}