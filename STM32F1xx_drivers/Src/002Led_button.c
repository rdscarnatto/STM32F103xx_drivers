/*
 * 001LedToggle.c
 *
 *  Created on: 19 ene. 2023
 *      Author: R. Scarnatto
 */
#include "stm32f103xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, GpioBtn;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHZ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_CNF_OUT_PP;

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_CNF_IN_PUPD;

	GPIO_Init(&GpioBtn);



	GPIO_WriteToOutputPin(GPIOC, 13, GPIO_PIN_SET); // Led OFF in Blue Pill Board

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,0) == 0)
		{
			GPIO_ToggleOutputPin(GPIOC, 13);
			delay();
		}
	}
}



