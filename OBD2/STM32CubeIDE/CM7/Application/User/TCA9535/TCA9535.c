/*
 * TCA9535.c
 *
 *  Created on: Oct 8, 2025
 *      Author: TC-Desenvolvimento
 */

#include "TCA9535.h"
void delay_us(uint32_t us);


void TCA9535_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t devAddr,
		uint8_t reg, uint8_t data) {
	HAL_I2C_Mem_Write(hi2c, devAddr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1,
			500);
}

void TCA9535_Init(I2C_HandleTypeDef *hi2c, uint8_t config_port0,
		uint8_t config_port1, uint8_t output_port0, uint8_t output_port1) {

	// Configure I/O direction
	TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_OUTPUT_0, config_port0);

	TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_OUTPUT_1, config_port1);

	TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_CONFIG_0, output_port0);

	TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_CONFIG_1, output_port1);

}

void TCA9535_WritePin(I2C_HandleTypeDef *hi2c, uint8_t port, uint8_t pin,
		GPIO_PinState PinState) {
	if (port > 1 || pin > 7)
		return;  // invalid input, just return

	uint8_t reg = (port == 0) ? REG_OUTPUT_0 : REG_OUTPUT_1;
	uint8_t port_val;

	if (HAL_I2C_Mem_Read(hi2c, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &port_val,
			1, 500) != HAL_OK) {
		I2C_RECOVERY(&hi2c4);
		return;  // fail silently
	}

	if (PinState == GPIO_PIN_SET) {
		port_val |= (1 << pin);
	} else {
		port_val &= ~(1 << pin);
	}

	HAL_I2C_Mem_Write(hi2c, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &port_val, 1,
			500);
}

uint8_t TCA9535_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t port, uint8_t pin) {
	if (port > 1 || pin > 7) {
		return 0xFF;
	}
	uint8_t data = 0;
	if (HAL_I2C_Mem_Read(hi2c, I2C_ADDR, 2 + port, I2C_MEMADD_SIZE_8BIT, &data,
			1, 500) != HAL_OK) {
		I2C_RECOVERY(&hi2c4);
		return 0xFF;
	}

	return (data & (1 << pin)) ? 1 : 0;

}

void I2C_RECOVERY(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_DeInit(hi2c);

	I2C_GPIO_ManualMode();
	for (int i = 0; i < 9; i++) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		delay_us(25);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		delay_us(25);

		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET)
			break;
	}

	I2C_GPIO_AFMode();
	MX_I2C4_Init();

}

void I2C_GPIO_ManualMode(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Enable GPIO clock if not already */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* --- Configure SCL as Open-Drain Output --- */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;          // important
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* --- Configure SDA as Input (Hi-Z) --- */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;          // important
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Ensure SCL released HIGH */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

void I2C_GPIO_AFMode(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void delay_us(uint32_t us)
{
    uint32_t count = (SystemCoreClock / 1000000) * us / 5;
    while(count--) __NOP();
}
