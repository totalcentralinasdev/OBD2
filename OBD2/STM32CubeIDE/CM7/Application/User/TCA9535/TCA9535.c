/*
 * TCA9535.c
 *
 *  Created on: Oct 8, 2025
 *      Author: TC-Desenvolvimento
 */


#include "TCA9535.h"



void TCA9535_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t devAddr,uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(hi2c, devAddr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}


void TCA9535_Init(I2C_HandleTypeDef *hi2c,uint8_t config_port0, uint8_t config_port1,uint8_t output_port0, uint8_t output_port1) {


    // Configure I/O direction
	TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_OUTPUT_0, config_port0);


    TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_OUTPUT_1, config_port1);



    TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_CONFIG_0, output_port0);


    TCA9535_WriteRegister(hi2c, I2C_ADDR, REG_CONFIG_1, output_port1);

}



void TCA9535_WritePin(I2C_HandleTypeDef *hi2c, uint8_t port, uint8_t pin, GPIO_PinState PinState)
{
    if (port > 1 || pin > 7) return;  // invalid input, just return

    uint8_t reg = (port == 0) ? REG_OUTPUT_0 : REG_OUTPUT_1;
    uint8_t port_val;


    if (HAL_I2C_Mem_Read(hi2c, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &port_val, 1, HAL_MAX_DELAY) != HAL_OK) {
        return;  // fail silently
    }

    if (PinState == GPIO_PIN_SET) {
        port_val |= (1 << pin);
    } else {
        port_val &= ~(1 << pin);
    }

    HAL_I2C_Mem_Write(hi2c, I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &port_val, 1, HAL_MAX_DELAY);
}


uint8_t TCA9535_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t port, uint8_t pin){
	if (port > 1 || pin > 7) {
		return 0xFF;
	}
	uint8_t data = 0;
	if (HAL_I2C_Mem_Read(hi2c, I2C_ADDR, 2+port, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
		return 0xFF;
	}


	return (data & (1 << pin)) ? 1 : 0;

}
