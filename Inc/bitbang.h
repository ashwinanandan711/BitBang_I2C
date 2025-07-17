/*
 * bitbang.h
 *
 *  Created on: Jul 16, 2025
 *      Author: Anandan K
 */

#ifndef BITBANG_H_
#define BITBANG_H_

#define SUCCESS 1
#define FAILED  0

#include "main.h"

typedef struct 
{
    GPIO_Handle_t *SDA;
    GPIO_Handle_t *SCL;

}BitBang_ptr_t;

void I2C_delay();
GPIO_Handle_t* I2C_GPIObitbang_InitSDA();
GPIO_Handle_t* I2C_GPIObitbang_InitSCL();
void GPIO_Mode_Output(GPIO_Handle_t *pHandle);
void GPIO_Mode_Input(GPIO_Handle_t *pHandle);
void SCL_High(BitBang_ptr_t *ptr);
void SCL_Low(BitBang_ptr_t *ptr);
void SDA_High(BitBang_ptr_t *ptr);
void SDA_Low(BitBang_ptr_t *ptr);
void I2C_Start(BitBang_ptr_t *ptr);
void I2C_Stop(BitBang_ptr_t *ptr);
int ack_Read(BitBang_ptr_t *ptr);
int ack_Write(BitBang_ptr_t *ptr);
uint8_t I2C_Read_bytes(BitBang_ptr_t *ptr, uint8_t addr, uint8_t *arr, uint8_t no_of_bytes);
uint8_t I2C_Write_bytes(BitBang_ptr_t *ptr, uint8_t addr, uint8_t *arr);


#endif /* BITBANG_H_ */
