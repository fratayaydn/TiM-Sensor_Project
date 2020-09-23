/*!
 *****************************************************************************
 * @file:    ADXL355.c
 * @brief:   ADXL355 accelerometer IC
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
 *****************************************************************************/

/***************************** Include Files **********************************/
#include "ADXL355.h"

/****************************** Global Data ***********************************/

int32_t volatile i32SensorX;
int32_t volatile i32SensorY;
int32_t volatile i32SensorZ;
int32_t volatile i32SensorT;
uint32_t volatile ui32SensorX;
uint32_t volatile ui32SensorY;
uint32_t volatile ui32SensorZ;
//uint32_t volatile ui32SensorT;

/************************* Static functions *****************************/

/**
 * @brief Select ADXL355 by resetting CHIP SELECT Pin
 *
 * @return none
 **/
static void ADXL355_Select() {
	HAL_GPIO_WritePin(ADXL355_CS_PORT, ADXL355_CS_PIN, GPIO_PIN_RESET);
}
/**
 * @brief Deselect ADXL355 by setting CHIP SELECT Pin
 *
 * @return none
 **/
static void ADXL355_Unselect() {
	HAL_GPIO_WritePin(ADXL355_CS_PORT, ADXL355_CS_PIN, GPIO_PIN_SET);
}

/************************* Global scope functions *****************************/

/**
 * @brief Initialize with parameters and Start ADXL355
 *
 * @param SPI handle Structure
 * @param ADXL355 handle Structure
 *
 * @return none
 **/

void ADXL355_Init(SPI_HandleTypeDef *hspi, ADXL355_HandleTypeDef *ADXL355_t) {
	ADXL355_Range(hspi, ADXL355_t->ADXL355_Range); /*Set G range of ADXL355*/
	ADXL355_Filter(hspi, ADXL355_t->ADXL355_HighPass, ADXL355_t->ADXL355_LowPass); /*Set filters of ADXL355*/
	ADXL355_Startup(hspi); /*Turn on measurement mode of ADXL355*/
}

/**
 * @brief Write data to a register in requested address
 * @param SPI handle Structure
 * @param Register address
 * @param Data to write
 *
 * @return none
 **/
void ADXL355_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t ui8address,
		uint8_t ui8Data) {
	uint8_t data[2];
	data[0] = ((ui8address << 1) | ADXL355_WRITE); /* Combine write register address and Write command */
	data[1] = ui8Data;

	ADXL355_Select(); /* Select accelerometer */

	HAL_SPI_Transmit(hspi, data, 2, 1);

	ADXL355_Unselect(); /* Deselect accelerometer */

}

/**
 * @brief Read data from a register in requested address
 *
 * @param SPI handle Structure
 * @param Register address
 * @param Number of register to read (SPI_READ_ONE_REG,SPI_READ_TWO_REG or SPI_READ_THREE_REG )
 *
 * @return Data in the register
 **/
uint32_t ADXL355_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t ui8address,
		enRegsNum enRegs) {

	uint8_t ui24Result[3];
	uint32_t ui32Result;
	uint8_t ui8writeAddress;

	ui8writeAddress = ((ui8address << 1) | ADXL355_READ); /* Combine read register address and READ command */

	ADXL355_Select(); /* Select accelerometer */

	HAL_SPI_Transmit(hspi, &ui8writeAddress, 1, 1); /* Send register address */

	if (enRegs == SPI_READ_ONE_REG) {

		HAL_SPI_Receive(hspi, ui24Result, 1, 1);
		ui32Result = ui24Result[0];

	}
	if (enRegs == SPI_READ_TWO_REG) { /* Only used for Temp & X,Y,Z offset and threshold registers*/

		HAL_SPI_Receive(hspi, ui24Result, 2, 1);
		/* Combine 2Bit register into one uint32 */
		ui32Result = ((ui24Result[0] << 8) | ui24Result[1]);

	} else { /* Only used for X,Y,Z axis data registers*/

		HAL_SPI_Receive(hspi, ui24Result, 3, 1);
		/* Combine 3Bit register into one uint32 */
		ui32Result = ((ui24Result[0] << 16) | (ui24Result[1] << 8) | ui24Result[2]);

	}

	ADXL355_Unselect(); /* Deselect accelerometer */

	return ui32Result;
}

/**
 * @brief Turns ADXL355 measurement mode.
 *
 * @param SPI handle Structure
 *
 * @return none
 *
 **/
void ADXL355_Startup(SPI_HandleTypeDef *hspi) {
	uint8_t ui8temp;

	ui8temp = (uint8_t) ADXL355_ReadRegister(hspi, ADXL355_POWER_CTL, SPI_READ_ONE_REG); /*Read POWER_CTL register, before modifying it */

	ui8temp &= ~(0x01); /* Set measurement bit in POWER_CTL register */

	ADXL355_WriteRegister(hspi, ADXL355_POWER_CTL, ui8temp); /* Write the new value to POWER_CTL register */
}

/**
 * @brief Puts ADXL355 into standby mode.
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ADXL355_Standby(SPI_HandleTypeDef *hspi) {
	uint8_t ui8temp;

	ui8temp = (uint8_t) ADXL355_ReadRegister(hspi, ADXL355_POWER_CTL, SPI_READ_ONE_REG); /*Read POWER_CTL register, before modifying it */

	ui8temp |= 0x01; /* Clear measurement bit in POWER_CTL register */

	ADXL355_WriteRegister(hspi, ADXL355_POWER_CTL, ui8temp); /* Write the new value to POWER_CTL register */

}

/**
 * @brief Reads the accelerometer data.
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ADXL355_ReadData(SPI_HandleTypeDef *hspi) {

	/* Receive raw acceleration datas from accelerometer */
	ui32SensorX = ADXL355_ReadRegister(hspi, ADXL355_XDATA3, SPI_READ_THREE_REG);
	ui32SensorY = ADXL355_ReadRegister(hspi, ADXL355_YDATA3, SPI_READ_THREE_REG);
	ui32SensorZ = ADXL355_ReadRegister(hspi, ADXL355_ZDATA3, SPI_READ_THREE_REG);
	//ui32SensorT = ADXL355_ReadRegister(&hspi, TEMP2, SPI_READ_TWO_REG);

	/* Receive signed integer raw datas */
	i32SensorX = ADXL355_AccDataConversion(ui32SensorX);
	i32SensorY = ADXL355_AccDataConversion(ui32SensorY);
	i32SensorZ = ADXL355_AccDataConversion(ui32SensorZ);

}

/**
 * @brief Changes high pass and low pass filter
 *
 * @param SPI handle Structure
 * @param High pass filter value
 * @param Low pass filter value(ie: ADXL355_ODR_1000, ADXL355_ODR_2000, ADXL355_ODR_4000);
 *
 * @return none
 *
 **/
void ADXL355_Filter(SPI_HandleTypeDef *hspi, uint8_t hpf, uint8_t lpf) {
	uint8_t filter = 0;

	filter = (hpf << 4) | lpf; /* Combine high pass and low pass filter values to send */
	ADXL355_Standby(hspi);
	ADXL355_WriteRegister(hspi, ADXL355_FILTER, filter);/* Set filter values within FILTER register */
	ADXL355_Startup(hspi);

}

/**
 *  @brief Changes ranges to 2g, 4g or 8g.
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ADXL355_Range(SPI_HandleTypeDef *hspi, uint8_t range) {

	ADXL355_WriteRegister(hspi, ADXL355_RANGE, range); /* Set sensor range within RANGE register */

}

/**
 * @brief Convert the two's complement data in X,Y,Z registers to signed integers
 *
 * @param Raw data from register
 *
 * @return int32_t converted signed integer data
 **/
int32_t ADXL355_AccDataConversion(uint32_t ui32SensorData) {
	int32_t volatile i32Conversion = 0;

	ui32SensorData = (ui32SensorData >> 4);
	ui32SensorData = (ui32SensorData & 0x000FFFFF);

	if ((ui32SensorData & 0x00080000) == 0x00080000) {

		i32Conversion = (ui32SensorData | 0xFFF00000);

	} else {
		i32Conversion = ui32SensorData;
	}

	return i32Conversion;
}
