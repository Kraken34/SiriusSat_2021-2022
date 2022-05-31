/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define ADC_BUFFER_SIZE 1024   // halfWord (uint16_t)
#define STAMP_BUFFER_SIZE 1024 // halfWord (uint16_t)

// Cyrcle adc buffer
volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

volatile uint32_t detectionCount = 0;
uint32_t detectionCountBuf = 0;
uint8_t reqI2C = 0;

int main() {
	// System clock and HAL init
	HAL_Init();
	SystemClock_Config();
	// peripheral init
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();

	HAL_Delay(2000);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer, ADC_BUFFER_SIZE);

	if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK ) { Error_Handler(); }

	while (true) {
		// blink
		GPIOC->BSRR = 1<<13;
		HAL_Delay(1000);
		GPIOC->BSRR = 1<<13 <<16;
		HAL_Delay(1000);
	}
}

extern "C" {
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	// buffer is half full
	for (uint16_t i = 0; i < ADC_BUFFER_SIZE/2; ++i) {
		if (adcBuffer[i] > 600) {
			// while next value is lower than previos
			while (adcBuffer[i] > adcBuffer[i+1] && i < ADC_BUFFER_SIZE/2) { ++i; }
			++detectionCount;
		}
	}
}}
extern "C" {
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// buffer is full
	for (uint16_t i = ADC_BUFFER_SIZE/2; i < ADC_BUFFER_SIZE; ++i) {
		if (adcBuffer[i] > 600) {
			// while next value is lower than previos
			while (adcBuffer[i] > adcBuffer[i+1] && i+2 < ADC_BUFFER_SIZE) { ++i; }
			++detectionCount;
		}
	}
}}

extern "C" {
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)  {}}

extern "C" {
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {}}

// If we receive own address
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (TransferDirection) { // Transmit from master
		HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, &reqI2C, 1, I2C_NEXT_FRAME);
	}
	else { // Receive from master
		detectionCountBuf = detectionCount;
		detectionCount = 0;
		HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t*)&detectionCountBuf, 4, I2C_NEXT_FRAME);
	}
}

// Init functions
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}
static void MX_ADC1_Init() {
  ADC_ChannelConfTypeDef sConfig = {0};
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}
static void MX_I2C1_Init() {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 172;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}
static void MX_DMA_Init() {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 5);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}
static void MX_GPIO_Init() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// If erorr
void Error_Handler() {
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (true) {
	  // blink very fast if error
	  GPIOC->BSRR = 1<<13;
	  HAL_Delay(100);
	  GPIOC->BSRR = 1<<13 <<16;
	  HAL_Delay(100);
  }
}
