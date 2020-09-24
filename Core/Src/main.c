/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "ADXL355.h"
#include "CPR_math.h"
#include "st7735.h"


/* Private macro -------------------------------------------------------------*/
#define LENGTH_SAMPLES 1024
#define CHAR_BUFF_SIZE 4

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

ADXL355_HandleTypeDef ADXL355_t;

float32_t aInput_f32[LENGTH_SAMPLES];
float32_t aOutput[LENGTH_SAMPLES / 2];

uint16_t fftSize = 512;
uint8_t ifftFlag = 0;
uint8_t doBitReverse = 1;
float32_t A[NUMBER_HARMONICS];
float32_t Sk[NUMBER_HARMONICS];
uint32_t fcc;
uint16_t bufferCounter;

float32_t pressure_inMM = 0;
uint8_t readFlag = 0;
uint8_t sleepFlag = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
static void SPI1_Init(void);
static void TIM1_Init(void);
static void TIM3_Init(void);
static void SPI2_Init(void);
static void Sleep_Mode(void);
static void Display_Pressure(SPI_HandleTypeDef *hspi, float32_t pressure,
		uint8_t digitsAfterDot);
static void _float_to_char(float32_t x, char *p, uint8_t size,
		uint8_t digitsAfterDot);
static void Sensor_Init(void);


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_SPI2_Init();


	HAL_TIM_Base_Start_IT(&htim1); /* Start TIM1 in interrupt mode */
	HAL_TIM_Base_Start_IT(&htim3); /* Start TIM1 in interrupt mode */

	ST7735_Init(&hspi2); /* Initialize display */
	Sensor_Init(); /* Initialize accelerometer sensor */
	HAL_Delay(1000); /* 1 second delay for initialization */

	/* Infinite loop */
	while (1) {

		/* If readFlag is set read data from accelerometer */
		if (readFlag) {

			ADXL355_ReadData(&hspi1); /* Read acceleration data from sensor */

			aInput_f32[bufferCounter * 2] = (float32_t) i32SensorZ / ADXL355_RANGE_2G_SCALE * 9.81f; /* Convert raw acceleration data into m/s^2  and write into the buffer*/
			aInput_f32[(bufferCounter * 2) + 1] = 0; /* Imaginary part */

			bufferCounter++; /* Increment buffer counter to write data in input array */

			readFlag = 0; /* Reset sensor read flag */

		}

		/* If buffer counter reachs calculate pressure and display it */
		else if (bufferCounter == fftSize) {
			pressure_inMM = CPR_CompressionDepth(aInput_f32, fftSize, ifftFlag, doBitReverse); /* Calculate pressure value from acceleration data */

			Display_Pressure(&hspi2, pressure_inMM, 2); /* Display obtained pressure value */

			bufferCounter = 0; /* Zero input buffer counter */

			Sleep_Mode(); /* Go into sleep mode */

		}

		/* After every reading session go into sleep mode */
		else if (sleepFlag) {
			Sleep_Mode();
			sleepFlag = 0;
		}

	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void SPI1_Init(void) {


	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void SPI2_Init(void) {

	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}


}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };


	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 9999;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 4199;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}


}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void TIM3_Init(void) {


	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };


	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 124;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 3499;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}


}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
 * @brief Initialize ADXL355 sensor unit
 *
 * @return none
 *
 **/
void Sensor_Init(void) {
	ADXL355_t.ADXL355_Range = ADXL355_RANGE_2G;
	ADXL355_t.ADXL355_LowPass = ADXL355_ODR_1000;
	ADXL355_t.ADXL355_HighPass = 0x00;

	ADXL355_Init(&hspi1, &ADXL355_t);
}

/**
 * @brief Sleep mode for ADXL355 and CPU
 *
 * @return none
 **/
void Sleep_Mode(void) {

	ADXL355_Standby(&hspi1); /* Enter sensor standby mode */
	HAL_SuspendTick(); /* Suspend SysTick timer to prevent interrupts that can awake CPU */
	HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI); /*Enter CPU sleep mode */
	HAL_ResumeTick(); /* Resume SysTick timer */
}

/**
 * @brief Display compression depth by converting float value to string
 *
 * @param SPI handle structure
 * @param Pressure value in millimeters
 * @param How many digits after the dot for float pressure value
 *
 * @return none
 *
 **/
void Display_Pressure(SPI_HandleTypeDef *hspi, float32_t pressure,
		uint8_t digitsAfterDot) {
	char displayBuffer[CHAR_BUFF_SIZE]; /* String to write on display */
	ST7735_WriteString(hspi, 20, 64, "Pressure:", Font_11x18, ST7735_RED,
	ST7735_WHITE);
	_float_to_char(pressure, displayBuffer, CHAR_BUFF_SIZE, digitsAfterDot); /* Convert pressure value into string */
	ST7735_WriteString(hspi, 20, 84, displayBuffer, Font_11x18, ST7735_RED,
	ST7735_WHITE); /* Display pressure value */

}

/**
 * @brief Covert float value to char arrays
 *
 * @param Input float value
 * @param[out] Output char array
 * @param Size of output char array
 * @param How many digits after the dot for float pressure value
 *
 * @return none
 **/
void _float_to_char(float32_t x, char *p, uint8_t size, uint8_t digitsAfterDot) {
	p = p + size; /* go to end of buffer */
	uint32_t power = 10; /* Indicator of how many digits will be put after dot */
	uint32_t fractions; /* Variable to store the fractional part */
	uint32_t integer; /* Variable to store the integer part */
	uint8_t i;

	/* Take power of 10 for how many digit is wanted for write after decimal seperator */
	for (i = 0; i < (digitsAfterDot - 1); i++) {
		power *= 10;
	}
	/* Negative numbers */
	if (x < 0) {
		integer = (int32_t) (-1 * x); /* Take integer part */
		fractions = (int32_t) (x * -1 * power) % (int32_t) power; /* Calculate fractional part */
	}
	/* Positive numbers */
	else {
		integer = (int32_t) x; /* Take integer part */
		fractions = (int32_t) (x * power) % (int32_t) power; /* Calculate fractional part */

	}

	/* Write fractional part of the float */
	for (i = 0; i < digitsAfterDot; i++) {
		*--p = (fractions % 10) + '0';
		fractions /= 10;
	}

	*--p = '.'; /* Put the decimal point */

	/* Write integer part of the float */
	while (integer > 0) {
		*--p = (integer % 10) + '0';
		integer /= 10;
	}
	/* Put dash for negative numbers */
	if (x < 0) {
		*--p = '-';
	}

}
/**
 * @brief Handles TIM1 update interrupt and TIM10 global interrupt
 *
 * @return none
 */
void TIM1_UP_TIM10_IRQHandler(void) {
	if (__HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_UPDATE) != RESET) /* Check interrupt flag */

	{
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE); /* Clear interrupt flag */

		sleepFlag = 1; /* Set sleep flag to go into sleep mode */

		TIM3->CR1 ^= TIM_CR1_CEN; /* Toggle timer which handles sensor reading */

	}
}
/**
 * @brief Handles TIM3 global interrupt
 *
 * @return none
 */
void TIM3_IRQHandler(void) {
	if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) /* Check interrupt flag */

	{
		__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE); /* Clear interrupt flag */

		readFlag = 1; /* Set sensor read flag */

	}
}

/**
 * @brief  This function is executed in case of error occurrence
 *
 * @return None
 */
void Error_Handler(void) {
	/* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
