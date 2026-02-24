/*
 * TCA9535.h
 *
 *  Created on: Oct 8, 2025
 *      Author: TC-Desenvolvimento
 */

#ifndef APPLICATION_USER_TCA9535_TCA9535_H_
#define APPLICATION_USER_TCA9535_TCA9535_H_

#include "stm32h7xx.h"
#include "i2c.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
} TCA9535_HandleTypeDef;



#define TCA9535_PORT0 0
#define TCA9535_PORT1 1

#define I2C_ADDR 		(0x20 << 1)

#define REG_INPUT_0		0x00
#define REG_INPUT_1		0x01
#define REG_OUTPUT_0 	0x02
#define REG_OUTPUT_1 	0x03
#define REG_TOOGLE_0 	0x04
#define REG_TOOGLE_1 	0x05
#define REG_CONFIG_0 	0x06
#define REG_CONFIG_1    0x07


void TCA9535_WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t devAddr,uint8_t reg, uint8_t data);


void TCA9535_Init(I2C_HandleTypeDef *hi2c, uint8_t config_port0, uint8_t config_port1,uint8_t output_port0, uint8_t output_port1);


void TCA9535_WritePin(I2C_HandleTypeDef *hi2c, uint8_t port, uint8_t pin, GPIO_PinState PinState);

uint8_t TCA9535_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t port, uint8_t pin);

void I2C_GPIO_AFMode(void);

void I2C_GPIO_ManualMode(void);

void I2C_RECOVERY(I2C_HandleTypeDef *hi2c);


#endif /* APPLICATION_USER_TCA9535_TCA9535_H_ */
