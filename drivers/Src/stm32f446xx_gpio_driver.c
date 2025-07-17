/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Apr 26, 2025
 *      Author: Anandan K
 */

#include "stm32f446xx_gpio_driver.h"

/******************************GPIO Init and De-init functions*********************************/

/******************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - This function initializes a given GPIO port
 *
 * @param[in]       - A pointer to the GPIO Handle structure
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	/* List of parameters to configure
	 * 1. Mode of GPIO pin
	 * 2. Speed
	 * 3. Pullup / Pulldown settings
	 * 4. Output type configuration
	 * 5. Alternate functionality configuration (if required)
	 */

	uint32_t temp = 0; // temporary register
	// Just so you remember, the handle struct contains all necessary inputs
	// You just have to find those and place them in the appt registers

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// 0,1,2,3 are non interrupt modes that's why the condition is written the way it is
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		//1. Mode has been configured successfully.
	}
	else
	{

	}
	temp = 0;

	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	// 2. Speed has been configured successfully

	temp = 0;

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	// 3. Pull up/ Pull down configuration has been configured successfully

	temp = 0;

	pGPIOHandle->pGPIOx->MODER &= ~(0x01 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	// 4. Output type has been configured successfully

	temp = 0;

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 = 0, temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0x0F << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2);
	}
	// 5. Alternate functionality has been configured successfully.

}

/******************************************************
 * @fn              - GPIO_DeInit
 *
 * @brief           - This function de-initializes a given GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}

/************************************GPIO Peripheral clock control***************************************/

/******************************************************
 * @fn              - GPIO_PClkControl
 *
 * @brief           - This function enables or disables the peripheral clock for a given GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 * @param[in]       - Enable or Disable macro
 *
 * @return          - none
 *
 * @note            - none
 *
 * */

void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/**********************************GPIO read and write functions*****************************************/

/******************************************************
 * @fn              - GPIO_ReadFromInputPin
 *
 * @brief           - This function is used to read input from a single pin of a GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 * @param[in]       - Pin number from which input has to be read
 *
 * @return          - boolean / uint8_t
 *
 * @note            - none
 *	
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)(( pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}

/******************************************************
 * @fn              - GPIO_ReadFromInputPort
 *
 * @brief           - This function is used to read input from the whole GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 *
 * @return          - 16 bit value (since each port has 16 pins)
 *
 * @note            - none
 *
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/******************************************************
 * @fn              - GPIO_WriteToOutputPin
 *
 * @brief           - This function is used to write onto a single pin of a GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 * @param[in]       - Pin Number at which the output has to be written
 * @param[in]       - Value that has to be written at the given pin (SET/RESET)
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/******************************************************
 * @fn              - GPIO_WriteToOutputPort
 *
 * @brief           - This function is used to write onto an entire GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 * @param[in]       - 16 bit value (since a port has 16 pins)
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/******************************************************
 * @fn              - GPIO_ToggleOutputPin
 *
 * @brief           - This function toggles the present state of a particular pin of a given GPIO port
 *
 * @param[in]       - The base address of the GPIO peripheral
 * @param[in]       - Enable or Disable macro
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
	// or can be written as pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);

}

/***************************GPIO IRQ configuration and ISR Handling functions**************************/

/******************************************************
 * @fn              - GPIO_IRQConfig
 *
 * @brief           - This function is used to set the interrupt configurations
 *
 * @param[in]       - The base address of the GPIO peripheral
 * @param[in]       - Enable or Disable macro
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/******************************************************
 * @fn              - GPIO_IRQHandling
 *
 * @brief           - This function toggles the present state of a particular pin of a given GPIO port
 *
 * @param[in]       - The pin number at which interrupt has occurred
 *
 * @return          - none
 *
 * @note            - none
 *
 * */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
