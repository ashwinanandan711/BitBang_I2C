/*
 * stm32f446xx.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Anandan K
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
/*******************************Base address initialization********************************/
#include<stdint.h> /*Without this library,the shorthand notation of the data types
                     such as int32_t won't work*/

#define __vo volatile // Shorthand notation of the volatile keyword

#define FLASH_BASEADDR              0x80000000U     //Flash base address
#define SRAM1_BASEADDR              0x20000000U     //SRAM1 base address
#define SRAM2_BASEADDR              0x20001C00U     //SRAM2 base address
#define ROM_BASEADDR                0x1FFF0000U     //ROM base address
#define SRAM                        SRAM1_BASEADDR  // SRAM1 is treated as main RAM memory

#define PERIPH_BASEADDR             0x40000000U     //Base address of the peripheral buses
#define APB1_BASEADDR               PERIPH_BASEADDR //Base address of APB1 bus
#define APB2_BASEADDR               0x40010000U     //Base address of APB2 bus
#define AHB1_BASEADDR               0x40020000U     //Base address of AHB1 bus
#define AHB2_BASEADDR               0x50000000U     //Base address of AHB2 bus

/* Base addresses of relevant peripherals hanging on AHB1 bus*/

#define GPIOA_BASEADDR              AHB1_BASEADDR + 0x0000U
#define GPIOB_BASEADDR              AHB1_BASEADDR + 0x0400U
#define GPIOC_BASEADDR              AHB1_BASEADDR + 0x0800U
#define GPIOD_BASEADDR              AHB1_BASEADDR + 0x0C00U
#define GPIOE_BASEADDR              AHB1_BASEADDR + 0x1000U
#define GPIOF_BASEADDR              AHB1_BASEADDR + 0x1400U
#define GPIOG_BASEADDR              AHB1_BASEADDR + 0x1800U
#define GPIOH_BASEADDR              AHB1_BASEADDR + 0x1C00U
#define RCC_BASEADDR                AHB1_BASEADDR + 0x3800U

/* Base addresses of relevant peripherals hanging on APB2 bus */

#define USART1_BASEADDR             APB2_BASEADDR + 0x1000U
#define USART6_BASEADDR             APB2_BASEADDR + 0x1400U
#define SPI1_BASEADDR               APB2_BASEADDR + 0x3000U
#define SPI4_BASEADDR               APB2_BASEADDR + 0x3400U
#define SYSCFG_BASEADDR             APB2_BASEADDR + 0x3800U
#define EXTI_BASEADDR               APB2_BASEADDR + 0x3C00U

/* Base addresses of relevant peripherals hanging on APB1 bus */

#define SPI2_BASEADDR               APB1_BASEADDR + 0x3800U
#define SPI3_BASEADDR               APB1_BASEADDR + 0x3C00U
#define USART2_BASEADDR             APB1_BASEADDR + 0x4400U
#define USART3_BASEADDR             APB1_BASEADDR + 0x4800U
#define UART4_BASEADDR              APB1_BASEADDR + 0x4C00U
#define UART5_BASEADDR              APB1_BASEADDR + 0x5000U
#define I2C1_BASEADDR               APB1_BASEADDR + 0x5400U
#define I2C2_BASEADDR               APB1_BASEADDR + 0x5800U
#define I2C3_BASEADDR               APB1_BASEADDR + 0x5C00U

/*
 * Macros are named in capital letters simply to differentiate them from the regular code.
 * Prefixes can be added in front of macro names to show which layer they belong to.
 * For example: DRV_GPIOA_BASEADDR or HAL_GPIOA_BASEADDR
 * */

/**************************Peripheral register definition structures**************************/

/* Registers of the peripherals of an MCU are specific to that MCU only
 * Refer to the device's reference manual for all details regarding the same*/

typedef struct
{
	__vo uint32_t MODER;   /*GPIO port mode register.                           Offset = 0x00*/
	__vo uint32_t OTYPER;  /*GPIO port output type register.                    Offset = 0x04*/
	__vo uint32_t OSPEEDR; /*GPIO port output speed register.                   Offset = 0x08*/
	__vo uint32_t PUPDR;   /*GPIO port pull-up/pull-down register.              Offset = 0x0C*/
	__vo uint32_t IDR;     /*GPIO port input data register.                     Offset = 0x10*/
	__vo uint32_t ODR;     /*GPIO port output data register.                    Offset = 0x14*/
	__vo uint32_t BSRR;    /*GPIO port bit set/reset register.                  Offset = 0x18*/
	__vo uint32_t LCKR;    /*GPIO port configuration lock register.             Offset = 0x1C*/
	__vo uint32_t AFR[2];  /*GPIO Alternate functionality registers. AFR[0] = low, AFR[1] = high*/
	 	 	 	 	 	 	                 /*Offset of AFRL = 0x20 and Offset of AFRH = 0x24  */
}GPIO_RegDef_t;

/*Above structure contains the GPIO registers*/

typedef struct
{
	__vo uint32_t RCC_CR;        /*RCC Clock control register.                         Offset = 0x00*/
	__vo uint32_t RCC_PLLCFGR;   /*RCC PLL configuration register.                     Offset = 0x04*/
	__vo uint32_t RCC_CFGR;      /*RCC Clock configuration register.                   Offset = 0x08*/
	__vo uint32_t RCC_CIR;       /*RCC Clock interrupt register.                       Offset = 0x0C*/
	__vo uint32_t RCC_AHB1RSTR;  /*RCC AHB1 peripheral reset register.                 Offset = 0x10*/
	__vo uint32_t RCC_AHB2RSTR;  /*RCC AHB2 peripheral reset register.                 Offset = 0x14*/
	__vo uint32_t RCC_AHB3RSTR;  /*RCC AHB3 peripheral reset register.                 Offset = 0x18*/
	uint32_t RESERVED0;          /*RESERVED - 0x1C                                                  */
	__vo uint32_t RCC_APB1RSTR;  /*RCC APB1 peripheral reset register.                 Offset = 0x20*/
	__vo uint32_t RCC_APB2RSTR;  /*RCC APB2 peripheral reset register.                 Offset = 0x24*/
	uint32_t RESERVED1[2];       /*RESERVED - 0x28 & 0x2C                                           */
	__vo uint32_t RCC_AHB1ENR;   /*RCC AHB1 peripheral clock enable register.          Offset = 0x30*/
	__vo uint32_t RCC_AHB2ENR;   /*RCC AHB2 peripheral clock enable register.          Offset = 0x34*/
	__vo uint32_t RCC_AHB3ENR;   /*RCC AHB3 peripheral clock enable register.          Offset = 0x38*/
	uint32_t RESERVED2;          /*RESERVED - 0x3C                                                  */
	__vo uint32_t RCC_APB1ENR;   /*RCC APB1 peripheral clock enable register.          Offset = 0x40*/
	__vo uint32_t RCC_APB2ENR;   /*RCC APB2 peripheral clock enable register.          Offset = 0x44*/
	uint32_t RESERVED3[2];       /*RESERVED - 0x48 & 0x4C                                           */
	__vo uint32_t RCC_AHB1LPENR; /*RCC AHB1 peripheral clock enable in low power mode. Offset = 0x50*/
	__vo uint32_t RCC_AHB2LPENR; /*RCC AHB2 peripheral clock enable in low power mode. Offset = 0x54*/
	__vo uint32_t RCC_AHB3LPENR; /*RCC AHB1 peripheral clock enable in low power mode. Offset = 0x58*/
	uint32_t RESERVED4;          /*RESERVED - 0x5C                                                  */
	__vo uint32_t RCC_APB1LPENR; /*RCC APB1 peripheral clock enable in low power mode. Offset = 0x60*/
	__vo uint32_t RCC_APB2LPENR; /*RCC APB2 peripheral clock enable in low power mode. Offset = 0x64*/
	uint32_t RESERVED5[2];       /*RESERVED - 0x68 & 0x6C                                           */
	__vo uint32_t RCC_BDCR;      /*RCC Backup domain control register.                 Offset = 0x70*/
	__vo uint32_t RCC_CSR;       /*RCC Clock control and status register.              Offset = 0x74*/
	uint32_t RESERVED6[2];       /*RESERVED - 0x78 & 0x7C                                           */
	__vo uint32_t RCC_SSCGR;     /*RCC Spread spectrum clock generation register.      Offset = 0x80*/
	__vo uint32_t RCC_PLLI2SCFGR;/*RCC PLLI2S configuration register.                  Offset = 0x84*/
	__vo uint32_t RCC_PLLSAICFGR;/*RCC PLL configuration register.                     Offset = 0x88*/
	__vo uint32_t RCC_DCKCFGR;   /*RCC dedicated clock configuration register.         Offset = 0x8C*/
	__vo uint32_t CKGATENR;      /*RCC clocks gated enable register.                   Offset = 0x90*/
	__vo uint32_t DCKCFGR2;      /*RCC dedicated clocks configuration register 2.      Offset = 0x94*/

}RCC_RegDef_t;
/*Above structure defines the RCC peripheral registers*/

typedef struct
{
	__vo uint32_t CR1;           /**/
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;

/*Peripheral definitions (Peripheral base addresses type casted to xx_RegDef_t)  */

#define GPIOA                       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                       ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                       ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                       ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                       ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                       ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC                         ((RCC_RegDef_t*)RCC_BASEADDR)

#define I2C1                        ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                        ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                        ((I2C_RegDef_t*)I2C3_BASEADDR)


/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN() (RCC->RCC_AHB1ENR |= ( 1 << 7 ))

/*
 * Clock enable macros for USARTx & UARTx peripherals
 */

#define USART1_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 4 ))
#define USART2_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  (RCC->RCC_APB1ENR  |= (1 << 19))
#define UART5_PCLK_EN()  (RCC->RCC_APB1ENR  |= (1 << 20))
#define USART6_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 5 ))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()   (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()   (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()   (RCC->RCC_APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()   (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()   (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()   (RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()   (RCC->RCC_APB2ENR |= (1 << 13))

/*
 * Clock enable macros for syscfg peripherals
 */

#define SYSCFG_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI() (RCC->RCC_AHB1ENR &= ~( 1 << 7 ))

/*
 * Clock disable macros for USARTx & UARTx peripherals
 */

#define USART1_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 4 ))
#define USART2_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 5 ))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()   (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()   (RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()   (RCC->RCC_APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()   (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()   (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()   (RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()   (RCC->RCC_APB2ENR &= ~(1 << 13))

/*
 * Clock disable macros for syscfg peripherals
 */

#define SYSCFG_PCLK_DI() (RCC->RCC_APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<0); RCC->RCC_AHB1RSTR &= ~(1<<0);}while(0)
#define GPIOB_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<1); RCC->RCC_AHB1RSTR &= ~(1<<1);}while(0)
#define GPIOC_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<2); RCC->RCC_AHB1RSTR &= ~(1<<2);}while(0)
#define GPIOD_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<3); RCC->RCC_AHB1RSTR &= ~(1<<3);}while(0)
#define GPIOE_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<4); RCC->RCC_AHB1RSTR &= ~(1<<4);}while(0)
#define GPIOF_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<5); RCC->RCC_AHB1RSTR &= ~(1<<5);}while(0)
#define GPIOG_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<6); RCC->RCC_AHB1RSTR &= ~(1<<6);}while(0)
#define GPIOH_REG_RESET() do {RCC->RCC_AHB1RSTR |= (1<<7); RCC->RCC_AHB1RSTR &= ~(1<<7);}while(0)

/*
 * Macros to reset I2Cx peripherals
 */

#define I2C1_REG_RESET() do {RCC->RCC_APB1RSTR |= (1<<21); RCC->RCC_APB1RSTR &= ~(1<<21);}while(0)
#define I2C2_REG_RESET() do {RCC->RCC_APB1RSTR |= (1<<22); RCC->RCC_APB1RSTR &= ~(1<<22);}while(0)
#define I2C3_REG_RESET() do {RCC->RCC_APB1RSTR |= (1<<23); RCC->RCC_APB1RSTR &= ~(1<<23);}while(0)


/*Generic macros*/

#define ENABLE                 1
#define DISABLE                0
#define SET                    ENABLE
#define RESET                  DISABLE
#define GPIO_PIN_SET           SET
#define GPIO_PIN_RESET         RESET

/*********************************************************************************
 *                      Bit position definitions of I2C registers
 *********************************************************************************/

/* I2C CR1 Register */

#define I2C_CR1_PE             0
#define I2C_CR1_NOSTRECH       7
#define I2C_CR1_START          8
#define I2C_CR1_STOP           9
#define I2C_CR1_ACK            10
#define I2C_CR1_SWRST          15

/* I2C CR2 Register*/

#define I2C_CR2_FREQ           0
#define I2C_CR2_ITERREN        8
#define I2C_CR2_ITEVTEN        9
#define I2C_CR2_ITBUFEN        10

/* I2C SR1 Register*/

#define I2C_SR1_SB             0  //Start bit (Master mode)
#define I2C_SR1_ADDR           1
#define I2C_SR1_BTF            2
#define I2C_SR1_ADD10          3
#define I2C_SR1_STOPF          4
#define I2C_SR1_RxNE           6
#define I2C_SR1_TxE            7
#define I2C_SR1_BERR           8
#define I2C_SR1_ARLO           9
#define I2C_SR1_AF             10
#define I2C_SR1_OVR            11
#define I2C_SR1_TIMEOUT        14

/* 	I2C SR2 Register*/

#define I2C_SR2_MSL            0
#define I2C_SR2_BUSY           1
#define I2C_SR2_TRA            2
#define I2C_SR2_GENCALL        4
#define I2C_SR2_DUALF          7

/* I2C CCR Register*/

#define I2C_CCR_CCR            0
#define I2C_CCR_DUTY           14
#define I2C_CCR_FS             15


#include "stm32f446xx_gpio_driver.h"


#endif /* INC_STM32F446XX_H_ */
