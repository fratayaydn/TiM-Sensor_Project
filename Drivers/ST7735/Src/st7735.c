#include "st7735.h"

#define DELAY 0x80

static const uint8_t init_cmds1[] = { // Init for 7735R, part 1 (red or green tab)
15,                       // 15 commands in list:
		ST7735_SWRESET, DELAY,  //  1: Software reset, 0 args, w/delay
		150,                    //     150 ms delay
		ST7735_SLPOUT, DELAY,  //  2: Out of sleep mode, 0 args, w/delay
		255,                    //     500 ms delay
		ST7735_FRMCTR1, 3,  //  3: Frame rate ctrl - normal mode, 3 args:
		0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		ST7735_FRMCTR2, 3,  //  4: Frame rate control - idle mode, 3 args:
		0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		ST7735_FRMCTR3, 6,  //  5: Frame rate ctrl - partial mode, 6 args:
		0x01, 0x2C, 0x2D,       //     Dot inversion mode
		0x01, 0x2C, 0x2D,       //     Line inversion mode
		ST7735_INVCTR, 1,  //  6: Display inversion ctrl, 1 arg, no delay:
		0x07,                   //     No inversion
		ST7735_PWCTR1, 3,  //  7: Power control, 3 args, no delay:
		0xA2, 0x02,                   //     -4.6V
		0x84,                   //     AUTO mode
		ST7735_PWCTR2, 1,  //  8: Power control, 1 arg, no delay:
		0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
		ST7735_PWCTR3, 2,  //  9: Power control, 2 args, no delay:
		0x0A,                   //     Opamp current small
		0x00,                   //     Boost frequency
		ST7735_PWCTR4, 2,  // 10: Power control, 2 args, no delay:
		0x8A,                   //     BCLK/2, Opamp current small & Medium low
		0x2A,
		ST7735_PWCTR5, 2,  // 11: Power control, 2 args, no delay:
		0x8A, 0xEE,
		ST7735_VMCTR1, 1,  // 12: Power control, 1 arg, no delay:
		0x0E,
		ST7735_INVOFF, 0,  // 13: Don't invert display, no args, no delay
		ST7735_MADCTL, 1,  // 14: Memory access control (directions), 1 arg:
		ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
		ST7735_COLMOD, 1,  // 15: set color mode, 1 arg, no delay:
		0x05 },                 //     16-bit color

		init_cmds2[] = {            // Init for 7735R, part 2
		2,                        //  2 commands in list:
				ST7735_CASET, 4,  //  1: Column addr set, 4 args, no delay:
				0x00, 0x00,             //     XSTART = 0
				0x00, 0x7F,             //     XEND = 127
				ST7735_RASET, 4,  //  2: Row addr set, 4 args, no delay:
				0x00, 0x00,             //     XSTART = 0
				0x00, 0x7F },           //     XEND = 127

		init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
				4,                        //  4 commands in list:
				ST7735_GMCTRP1,
				16, //  1: Magical unicorn dust, 16 args, no delay:
				0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25,
				0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
				ST7735_GMCTRN1,
				16, //  2: Sparkles and rainbows, 16 args, no delay:
				0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E,
				0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
				ST7735_NORON, DELAY, //  3: Normal display on, no args, w/delay
				10,                     //     10 ms delay
				ST7735_DISPON, DELAY, //  4: Main screen turn on, no args w/delay
				100 };                  //     100 ms delay

/**
 * @brief Select ST7735 by resetting CHIP SELECT Pin
 *
 * @return none
 **/
static void ST7735_Select() {
	HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Deselect ST7735 by setting CHIP SELECT Pin
 *
 * @return none
 **/
static void ST7735_Unselect() {
	HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Reset ST7735 by setting RESET Pin
 *
 * @return none
 **/
static void ST7735_Reset() {
	HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
}

/**
 * @brief Send command to ST7735
 *
 * @param SPI handle Structure
 * @param Command value
 *
 * @return none
 **/
static void ST7735_WriteCommand(SPI_HandleTypeDef *hspi, uint8_t cmd) {
	HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

/**
 * @brief Write data to the register of ST7735
 *
 * @param SPI handle Structure
 * @param Buffer to send
 * @param Size of buffer
 *
 * @return none
 **/
void ST7735_WriteData(SPI_HandleTypeDef *hspi, uint8_t *buff, size_t buff_size) {
	HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(hspi, buff, buff_size, HAL_MAX_DELAY);
}

/**
 * @brief Send command array
 *
 * @param SPI handle Structure
 * @param Command array
 *
 * @return none
 **/
static void ST7735_ExecuteCommandList(SPI_HandleTypeDef *hspi,
		const uint8_t *addr) {
	uint8_t numCommands, numArgs;
	uint16_t ms;

	numCommands = *addr++;
	while (numCommands--) {
		uint8_t cmd = *addr++;
		ST7735_WriteCommand(hspi, cmd);

		numArgs = *addr++;
		/* If high bit set, delay follows args */
		ms = numArgs & DELAY;
		numArgs &= ~DELAY;
		if (numArgs) {
			ST7735_WriteData(hspi, (uint8_t*) addr, numArgs);
			addr += numArgs;
		}

		if (ms) {
			ms = *addr++;
			if (ms == 255)
				ms = 500;
			HAL_Delay(ms);
		}
	}
}

/**
 * @brief Select where to write in display
 *
 * @param SPI handle Structure
 * @param x start coordinate of display
 * @param y start coordinate of display
 * @param x ending coordinate of display
 * @param y ending coordinate of display
 *
 * @return none
 **/
static void ST7735_SetAddressWindow(SPI_HandleTypeDef *hspi, uint8_t x0,
		uint8_t y0, uint8_t x1, uint8_t y1) {
	/* Set column address */
	ST7735_WriteCommand(hspi, ST7735_CASET);
	uint8_t data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
	ST7735_WriteData(hspi, data, sizeof(data));

	/* Set row address */
	ST7735_WriteCommand(hspi, ST7735_RASET);
	data[1] = y0 + ST7735_YSTART;
	data[3] = y1 + ST7735_YSTART;
	ST7735_WriteData(hspi, data, sizeof(data));

	/* Write to RAM */
	ST7735_WriteCommand(hspi, ST7735_RAMWR);
}

/**
 * @brief Initialize by sending initial commands to the ST7735
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ST7735_Init(SPI_HandleTypeDef *hspi) {
	ST7735_Unselect();
	ST7735_Select();
	ST7735_Reset();
	ST7735_ExecuteCommandList(hspi, init_cmds1);
	ST7735_ExecuteCommandList(hspi, init_cmds2);
	ST7735_ExecuteCommandList(hspi, init_cmds3);
	ST7735_Unselect();
}

/**
 * @brief Write string to the display
 *
 * @param SPI handle Structure
 * @param x coordinate of display
 * @param y coordinate of display
 * @param char to write to display
 * @param Font structure
 * @param Color of string to write
 * @param Background color of display
 *
 * @return none
 **/
static void ST7735_WriteChar(SPI_HandleTypeDef *hspi, uint16_t x, uint16_t y,
		char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
	uint32_t i, b, j;

	ST7735_SetAddressWindow(hspi, x, y, x + font.width - 1, y + font.height - 1); /* Determine where to write character */

	/* Write character with color and fill other pixels with background color */
	for (i = 0; i < font.height; i++) {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++) {
			if ((b << j) & 0x8000) {
				uint8_t data[] = { color >> 8, color & 0xFF };
				ST7735_WriteData(hspi, data, sizeof(data));
			} else {
				uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
				ST7735_WriteData(hspi, data, sizeof(data));
			}
		}
	}
}

/**
 * @brief Write string to the display
 *
 * @param SPI handle Structure
 * @param x coordinate of display
 * @param y coordinate of display
 * @param Base address of str to write to display
 * @param Font structure
 * @param Color of string to write
 * @param Background color of display
 *
 * @return none
 **/
void ST7735_WriteString(SPI_HandleTypeDef *hspi, uint16_t x, uint16_t y,
		const char *str, FontDef font, uint16_t color, uint16_t bgcolor) {
	ST7735_Select();

	/* Check if string will fit in requested area, otherwise start from beginning */
	while (*str) {
		if (x + font.width >= ST7735_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7735_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				/* skip spaces in the beginning of the new line */
				str++;
				continue;
			}
		}

		ST7735_WriteChar(hspi, x, y, *str, font, color, bgcolor); /* Write character */
		/* Go to next pixel */
		x += font.width;
		str++;
	}

	ST7735_Unselect();
}

