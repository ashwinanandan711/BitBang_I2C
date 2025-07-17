/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Anandan K
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
 /*
  * GPIO Pin configuration structure
  */

typedef struct
{
	uint8_t GPIO_PinNumber;       /*Possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;         /*Possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;        /*Possible values from @GPIO_PIN_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;  /*Possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;       /*Possible values from @GPIO_PIN_OP_TYPES*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handle structure of GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN           0   /*GPIO input mode*/
#define GPIO_MODE_OUT          1   /*GPIO output mode*/
#define GPIO_MODE_ALTFN        2   /*GPIO alternate functionality mode*/
#define GPIO_MODE_ANALOG       3   /*GPIO analog mode*/

// The above are directly taken from the modes of the MODER register

#define GPIO_MODE_IT_FT        4   /*GPIO input falling edge mode*/
#define GPIO_MODE_IT_RT        5   /*GPIO input rising edge mode*/
#define GPIO_MODE_IT_RFT       6   /*GPIO input rising edge - falling edge trigger mode*/

//The above macros are for interrupt handling while the GPIO is in input mode.

/*
 * @GPIO_PIN_OP_TYPES
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP        0   /*Push-Pull output mode*/
#define GPIO_OP_TYPE_OD        1   /*Open Drain output mode*/

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds (Refer to data sheet for frequency values of the relevant speed mode)
 */

#define GPIO_SPEED_LOW         0
#define GPIO_SPEED_MEDIUM      1
#define GPIO_SPEED_HIGH        2
#define GPIO_SPEED_VERYHIGH    3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up / pull down configuration
 */

#define GPIO_NO_PUPD           0
#define GPIO_PU                1
#define GPIO_PD                2

/*
 * @GPIO_PIN_NUMBERS
 * GPIO possible pin numbers
 */

#define GPIO_PIN_NO_0          0
#define GPIO_PIN_NO_1          1
#define GPIO_PIN_NO_2          2
#define GPIO_PIN_NO_3          3
#define GPIO_PIN_NO_4          4
#define GPIO_PIN_NO_5          5
#define GPIO_PIN_NO_6          6
#define GPIO_PIN_NO_7          7
#define GPIO_PIN_NO_8          8
#define GPIO_PIN_NO_9          9
#define GPIO_PIN_NO_10         10
#define GPIO_PIN_NO_11         11
#define GPIO_PIN_NO_12         12
#define GPIO_PIN_NO_13         13
#define GPIO_PIN_NO_14         14
#define GPIO_PIN_NO_15         15



/*************************************************************************************************************
 *                                    APIs supported by this driver
 *                    Information regarding the APIs can be found in the source files
 ************************************************************************************************************/

/*GPIO Init and De-init*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*GPIO Peripheral clock control*/

void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*GPIO read and write*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*GPIO IRQ configuration and ISR Handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
