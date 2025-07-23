/*
 * bitbang.c
 *
 *  Create  d on: Jul 16, 2025
 *      Author: Anandan K
 */

#include "bitbang.h"
#include <stdint.>

void I2C_delay()
{
    for (uint16_t i = 0; i < 300; i++);
}
/***************************GPIO initialization***************************/
GPIO_Handle_t* I2C_GPIObitbang_InitSDA()
{
    GPIO_Handle_t SDA;
    SDA.pGPIOx = GPIOA;
    SDA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    SDA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    SDA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SDA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GPIO_Init(&SDA);
    GPIO_PClkControl(SDA.pGPIOx, ENABLE);
    return &SDA;
}

GPIO_Handle_t* I2C_GPIObitbang_InitSCL()
{
    GPIO_Handle_t SCL;
    SCL.pGPIOx = GPIOA;
    SCL.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    SCL.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    SCL.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SCL.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    SCL.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_Init(&SCL);
    GPIO_PClkControl(SCL.pGPIOx, ENABLE);
    return &SCL;
}

/***************************SDA Direction control***************************/

void GPIO_Mode_Output(GPIO_Handle_t *pHandle)
{
    pHandle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
}

void GPIO_Mode_Input(GPIO_Handle_t *pHandle)
{
    pHandle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
}

/***************************GPIO output state control***************************/

void SCL_High(BitBang_ptr_t *ptr)
{
    GPIO_WriteToOutputPin(ptr->SCL->pGPIOx, ptr->SCL->GPIO_PinConfig.GPIO_PinNumber,1);
}

void SCL_Low(BitBang_ptr_t *ptr)
{
    GPIO_WriteToOutputPin(ptr->SCL->pGPIOx, ptr->SCL->GPIO_PinConfig.GPIO_PinNumber,0);
}

void SDA_High(BitBang_ptr_t *ptr)
{
    GPIO_WriteToOutputPin(ptr->SDA->pGPIOx, ptr->SDA->GPIO_PinConfig.GPIO_PinNumber,1);
}

void SDA_Low(BitBang_ptr_t *ptr)
{
    GPIO_WriteToOutputPin(ptr->SDA->pGPIOx, ptr->SDA->GPIO_PinConfig.GPIO_PinNumber,0);
}

/***************************Start and Stop conditions***************************/

void I2C_Start(BitBang_ptr_t *ptr)
{
    GPIO_Mode_Output(ptr->SDA);
    SDA_High(ptr);
    I2C_delay();
    SCL_High(ptr);
    I2C_delay();
    SDA_Low(ptr);
    I2C_delay();
    SCL_Low(ptr);
}

void I2C_Stop(BitBang_ptr_t *ptr)
{
    GPIO_Mode_Output(ptr->SDA);
    SDA_Low(ptr);
    I2C_delay();
    SCL_High(ptr);
    I2C_delay();
    SDA_High(ptr);
}

/***************************ack Handling***************************/

uint8_t ack_Read(BitBang_ptr_t *ptr)
{
    GPIO_Mode_Input(ptr->SDA);
    I2C_delay();
    SCL_High(ptr);
    I2C_delay();
    uint8_t ack1 = GPIO_ReadFromInputPin(ptr->SDA->pGPIOx, ptr->SDA->GPIO_PinConfig.GPIO_PinNumber);
    SCL_Low(ptr);
    I2C_delay();
    return ack1;
}

void ack_Write(BitBang_ptr_t *ptr, uint8_t ack)
{
    if (ack)
    {
        SDA_Low(ptr);
        SCL_High(ptr);
        I2C_delay();
        SCL_Low(ptr);
    }
    else 
    {
        SDA_High(ptr);
        SCL_High(ptr);
        I2C_delay();
        SCL_Low(ptr);
    }
}

/***************************Basic read & write***************************/

uint8_t SDA_Write_byte(BitBang_ptr_t *ptr, uint8_t data)
{
    GPIO_Mode_Output(ptr->SDA);
    uint8_t i;
    for (i=0; i<8; i++)
    {
        (data & 0x80) ? SDA_High(ptr) : SDA_Low(ptr);
        data <<= 1;
        SCL_High(ptr);
        I2C_delay();
        SCL_Low(ptr);
        I2C_delay();
    }
    i = ack_Read(ptr);
    return i;
}

uint8_t SDA_Read_byte(BitBang_ptr_t *ptr, uint8_t ack);
{
    GPIO_Mode_Input(ptr->SDA);
    uint8_t data = 0;
    for (uint8_t i = 0; i <8; i++)
    {
        data <<= 1;
        SCL_HIGH(ptr);
        I2C_delay();
        data |= (0x01);
        SCL_LOW(ptr);
        I2C_delay();
    }
    GPIO_Mode_Output(ptr->SDA);    
    ack_Write(ptr, ack);
    
    return data;
}

/***************************Address write***************************/

uint8_t SDA_Write_addr(BitBang_ptr_t *ptr, uint8_t addr, uint8_t rw)
{
    uint8_t data = ((addr << 1) | rw);
    GPIO_Mode_Output(ptr->SDA);
    uint8_t i;
    for (i=0; i<8; i++)
    {
        (data & 0x80) ? SDA_High(ptr) : SDA_Low(ptr);
        data <<= 1;
        SCL_High(ptr);
        I2C_delay();
        SCL_Low(ptr);
        I2C_delay();
    }
    i = ack_Read(ptr);
    return i;
}

/***************************Read and Write Byte arrays***************************/

uint8_t I2C_Read_bytes(BitBang_ptr_t *ptr, uint8_t addr, uint8_t *arr, uint8_t no_of_bytes)
{
    uint8_t ack;
    I2C_Start(ptr);
    ack = SDA_Write_addr(ptr, addr, 1);
 
    if (!ack)
    {
        for (uint8_t i = 0; i < no_of_bytes; i++)
        {
            arr[i] = SDA_Read_byte(ptr, i < no_of_bytes-1);
        }

        I2C_Stop(ptr);
        return SUCCESS;
    }

    else
    {
        return FAILED;
    }

}

uint8_t I2C_Write_bytes(BitBang_ptr_t *ptr, uint8_t addr, uint8_t *arr)
{
    uint8_t ack;
    I2C_Start(ptr);
    ack = SDA_Write_addr(ptr, addr, 0);
 
    if (!ack)
    {
        for (uint8_t i = 0; i < no_of_bytes; i++)
        {
            ack = SDA_Write_byte(ptr, arr[i]);
            if (!ack)
            {
                continue;
            }
            else 
            {
                return FAILED;
            }
        }

        I2C_Stop(ptr);
        return SUCCESS;
    }

    else
    {
        return FAILED;
    }

}

