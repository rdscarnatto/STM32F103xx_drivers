/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: 18 ene. 2023
 *      Author: ARRSC
 */

#include "stm32f103xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}//TODO Disable
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	uint8_t tpin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint8_t tcfg = pGPIOHandle->GPIO_PinConfig.GPIO_PinCfg;
	uint8_t offset = 0x00;

	if(tpin > 7)
	{
		tpin -=8;
		offset = 0x01;
	}

	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (tpin * 4)) | (tcfg << (tpin * 4+2)));

	if(offset == 0x00)
	{
		pGPIOHandle->pGPIOx->CRL &= ~(0xF << (tpin*4)); // Reset Target Pin
		pGPIOHandle->pGPIOx->CRL |= temp; // Set up the direction and option pin
	}else
	{
		pGPIOHandle->pGPIOx->CRH &= ~(0xF << (tpin*4));
		pGPIOHandle->pGPIOx->CRH |= temp;
	}

}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//TODO Read from pin
	return 0;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	//TODO Read from port
	return 0;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
