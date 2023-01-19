/*
 * 001LedToggle.c
 *
 *  Created on: 19 ene. 2023
 *      Author: Daniel
 */
#include "stm32f103xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed = {0};

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHZ;
	GpioLed.GPIO_PinConfig.GPIO_PinCfg = GPIO_CNF_OUT_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	GPIO_WriteToOutputPin(GPIOC, 13, GPIO_PIN_SET); // Led OFF in Blue Pill Board

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, 13);
		delay();
	}
	return 0;
}
