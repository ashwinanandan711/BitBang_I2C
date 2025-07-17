#include "main.h"

BitBang_ptr_t i2c;
uint8_t I2C_transmit_flag; 
uint8_t read_buffer[10];
uint8_t write_buffer[10];

int main(void)
{
    GPIO_Handle_t *SDAptr = I2C_GPIObitbang_InitSDA();
    GPIO_Handle_t *SCLptr = I2C_GPIObitbang_InitSCL();

    i2c.SCL = SCLptr;
    i2c.SDA = SDAptr;

    BitBang_ptr_t *I2Cptr = &i2c;

    uint8_t address = 0x11;

    I2C_transmit_flag = I2C_Read_bytes(I2Cptr, address, read_buffer, 10);
    if (I2C_transmit_flag == FAILED)
    {
        I2C_Stop(I2Cptr);
    }

    write_buffer = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};

    I2C_transmit_flag = I2C_Write_bytes(I2Cptr, address, write_buffer);
    if (I2C_transmit_flag == FAILED)
    {
        I2C_Stop(I2Cptr);
    }

}
