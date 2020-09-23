/*!
 *****************************************************************************
 * @file:    ADXL355.h
 * @brief:   ADXL355 accelerometer IC
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
 *
 *****************************************************************************/


#ifndef ADXL355_H_
#define ADXL355_H_

#include "stm32f4xx_hal.h"



/*******************************************************************************
 **************************** Internal types ************************************
 ********************************************************************************/

/* Read data mode */

typedef enum {
	SPI_READ_ONE_REG = 1, /* Read one ACC register */
	SPI_READ_TWO_REG, /* Read two ACC registers */
	SPI_READ_THREE_REG, /* Read X,Y,Z ACC registers */

} enRegsNum;

typedef struct {
	uint8_t ADXL355_Range;
	uint16_t ADXL355_LowPass;
	uint16_t ADXL355_HighPass;
}ADXL355_HandleTypeDef;

/********************************* Definitions ********************************/

/* ADXL355 registers addresses */
#define ADXL355_DEVID_AD                 0x00
#define ADXL355_DEVID_MST                0x01
#define ADXL355_PARTID                   0x02
#define ADXL355_REVID                    0x03
#define ADXL355_STATUS                   0x04
#define ADXL355_FIFO_ENTRIES             0x05
#define ADXL355_TEMP2                    0x06
#define ADXL355_TEMP1                    0x07
#define ADXL355_XDATA3                   0x08
#define ADXL355_XDATA2                   0x09
#define ADXL355_XDATA1                   0x0A
#define ADXL355_YDATA3                   0x0B
#define ADXL355_YDATA2                   0x0C
#define ADXL355_YDATA1                   0x0D
#define ADXL355_ZDATA3                   0x0E
#define ADXL355_ZDATA2                   0x0F
#define ADXL355_ZDATA1                   0x10
#define ADXL355_FIFO_DATA                0x11
#define ADXL355_OFFSET_X_H               0x1E
#define ADXL355_OFFSET_X_L               0x1F
#define ADXL355_OFFSET_Y_H               0x20
#define ADXL355_OFFSET_Y_L               0x21
#define ADXL355_OFFSET_Z_H               0x22
#define ADXL355_OFFSET_Z_L               0x23
#define ADXL355_ACT_EN                   0x24
#define ADXL355_ACT_THRESH_H             0x25
#define ADXL355_ACT_THRESH_L             0x26
#define ADXL355_ACT_COUNT                0x27
#define ADXL355_FILTER                   0x28
#define ADXL355_FIFO_SAMPLES             0x29
#define ADXL355_INT_MAP                  0x2A
#define ADXL355_SYNC                     0x2B
#define ADXL355_RANGE                    0x2C
#define ADXL355_POWER_CTL                0x2D
#define ADXL355_SELF_TEST                0x2E
#define ADXL355_RESET                    0x2F

/**************************** Configuration parameters **********************/

/* Temperature parameters */
#define ADXL355_TEMP_BIAS       (float)1852.0      /* Accelerometer temperature bias(in ADC codes) at 25 Deg C */
#define ADXL355_TEMP_SLOPE      (float)-9.05       /* Accelerometer temperature change from datasheet (LSB/degC) */

/* Accelerometer parameters */

/* ODR values */
#define ADXL355_ODR_4000      ((uint16_t)0x00)
#define ADXL355_ODR_2000      ((uint16_t)0x01)
#define ADXL355_ODR_1000      ((uint16_t)0x02)
#define ADXL355_ODR_500       ((uint16_t)0x03)
#define ADXL355_ODR_250       ((uint16_t)0x04)
#define ADXL355_ODR_125       ((uint16_t)0x05)
#define ADXL355_ODR_62_5      ((uint16_t)0x06)
#define ADXL355_ODR_31_25     ((uint16_t)0x07)
#define ADXL355_ODR_15_625    ((uint16_t)0x08)
#define ADXL355_ODR_7_813     ((uint16_t)0x09)
#define ADXL355_ODR_3_906     ((uint16_t)0x0A)

/* Range values */
#define ADXL355_RANGE_2G      ((uint16_t)0x81)
#define ADXL355_RANGE_4G      ((uint16_t)0x82)
#define ADXL355_RANGE_8G      ((uint16_t)0x83)

/*Scales to corresponding range */
#define ADXL355_RANGE_2G_SCALE      256000.0f
#define ADXL355_RANGE_4G_SCALE      128000.0f
#define ADXL355_RANGE_8G_SCALE      64000.0f

/* Accelerometer write command */
#define ADXL355_WRITE         0x0

/* Accelerometer read command */
#define ADXL355_READ          0x1
/****************************** Global Data ***********************************/

extern volatile int32_t i32SensorX;
extern volatile int32_t i32SensorY;
extern volatile int32_t i32SensorZ;

extern volatile uint32_t ui32SensorX;
extern volatile uint32_t ui32SensorY;
extern volatile uint32_t ui32SensorZ;
//extern volatile uint32_t ui32SensorT;

/****************************** PINs ***********************************/

/* Chip Select PIN */
#define ADXL355_CS_PORT		GPIOA
#define ADXL355_CS_PIN		GPIO_PIN_4
/* DATA READY PIN */
#define ADXL355_DRDY_Port	GPIOI
#define ADXL355_DRDY_PIN	GPIO_PIN_2

/*************************** Functions prototypes *****************************/

void ADXL355_Init(SPI_HandleTypeDef *hspi, ADXL355_HandleTypeDef *ADXL355_t);
void ADXL355_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t ui8address, uint8_t ui8Data);
uint32_t ADXL355_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t ui8address, enRegsNum enRegs);

uint8_t ADXL355_ReadSingleRegisters(SPI_HandleTypeDef *hspi, uint8_t ui8address);
uint32_t ADXL355_ReadMultipleRegisters(SPI_HandleTypeDef *hspi, uint8_t ui8address, uint8_t count);

void ADXL355_Startup(SPI_HandleTypeDef *hspi);
void ADXL355_Standby(SPI_HandleTypeDef *hspi);
void ADXL355_ReadData(SPI_HandleTypeDef *hspi);
void ADXL355_Filter(SPI_HandleTypeDef *hspi, uint8_t hpf, uint8_t lpf);
void ADXL355_Range(SPI_HandleTypeDef *hspi, uint8_t range);
int32_t ADXL355_AccDataConversion(uint32_t ui32SensorData);


#endif /* ADXL355_H_ */
