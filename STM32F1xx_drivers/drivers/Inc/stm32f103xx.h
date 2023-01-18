/*
 * stm32f103xx.h
 *
 *  Created on: Jan 17, 2023
 *      Author: Daniel
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*
* Base address of FALSH and SRAM memories
*/

#define FLASH_BASEADDR		0x80000000U
#define SRAM_BASEADDR		0x20000000U
#define ROM_BSEADDR			0x1FFF0000U
#define SRAM				SRAM_BASEADDR

/*
* Base address of AHB, APB1 and APB2
*/

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHBPERIPH_BASE		0x40018000U

/*
* Base address of peripheral which are hanging on AHB bus
*/

#define SDIO_BASEADDR		(AHBPERIPH_BASE + 0x0000)
#define DMA1_BASEADDR		(AHBPERIPH_BASE + 0x8000)
#define DMA2_BASEADDR		(DMA1_BASEADDR + 0x0400)
#define RCC_BASEADDR		(DMA1_BASEADDR + 0x1000)
#define	CRC_BASEADDR		(DMA1_BASEADDR + 0x3000)
#define ETH_BASEADDR		(DMA1_BASEADDR + 0x8000)

/*
* Base address of peripheral which are hanging on APB1 bus
*/

#define TIM2_BASEADDR		(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR		(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR		(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR		(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR		(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR		(APB1PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR		(APB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR		(APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR		(APB1PERIPH_BASE + 0x2000)

#define RTC_BASEADDR		(APB1PERIPH_BASE + 0x2800)

#define WWDG_BASEADDR		(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR		(APB1PERIPH_BASE + 0x3000)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)

#define USB_BASEADDR		(APB1PERIPH_BASE + 0x5C00)

#define bxCAN1_BASEADDR		(APB1PERIPH_BASE + 0x6400)
#define bxCAN2_BASEADDR		(APB1PERIPH_BASE + 0x6800)

#define BKP_BASEADDR		(APB1PERIPH_BASE + 0x6C00)
#define PWR_BASEADDR		(APB1PERIPH_BASE + 0x7000)
#define DAC_BASEADDR		(APB1PERIPH_BASE + 0x7400)

/*
* Base address of peripheral which are hanging on APB2 bus
* TODO: Complete for all other peripherals
*/

#define AFIO_BASEADDR 		(APB2PERIPH_BASE + 0x0000)
#define EXTI_BASEADDR 		(APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASEADDR 		(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR 		(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR 		(APB2PERIPH_BASE + 0x1000)
#define ADC1_BASEADDR 		(APB2PERIPH_BASE + 0x2400)
#define ADC2_BASEADDR 		(APB2PERIPH_BASE + 0x2800)
#define TIM1_BASEADDR 		(APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR 		(APB2PERIPH_BASE + 0x3000)
#define TIM8_BASEADDR 		(APB2PERIPH_BASE + 0x3400)
#define USART1_BASEADDR 	(APB2PERIPH_BASE + 0x3800)
#define SPI1_BASEADDR 		(APB2PERIPH_BASE + 0x3000)

/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t CRL;           /*!< GPIO port configuration register low,         	Address offset: 0x00      */
	__vo uint32_t CRH;           /*!< GPIO port configuration register high,     	Address offset: 0x04      */
	__vo uint32_t IDR;			 /*!< GPIO port input data register,     			Address offset: 0x08      */
	__vo uint32_t ODR;			 /*!< GPIO port output data register,     			Address offset: 0x0C      */
	__vo uint32_t BSRR;			 /*!< GPIO Port bit set/reset register,     			Address offset: 0x10      */
	__vo uint32_t BRR;			 /*!< GPIO Port bit reset register,     				Address offset: 0x14      */
	__vo uint32_t LCKR;			 /*!< GPIO Port configuration lock register,     	Address offset: 0x18      */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
}RCC_RegDef_t;


/*
* peripheral definitions (Peripheral base address typecast to xxx_RegDef_t
*/

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

/*
* Clock Enable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_EN()   ( RCC->APB2ENR |= (1 << 2) )
#define GPIOB_PCLK_EN()   ( RCC->APB2ENR |= (1 << 3) )
#define GPIOC_PCLK_EN()   ( RCC->APB2ENR |= (1 << 4) )
#define AFIO_PCLK_EN()    ( RCC->APB2ENR |= (1 << 0) )

/*
* Clock Enable Macros for ADCx peripherals
*/

#define ADC1_PCLK_EN()    ( RCC->APB2ENR |= (1 << 9) )
#define ADC2_PCLK_EN()    ( RCC->APB2ENR |= (1 << 10) )

/*
* Clock Enable Macros for SPIx peripherals
*/
#define SPI1_PCLK_EN()    ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()    ( RCC->APB1ENR |= (1 << 15) )

/*
* Clock Enable Macros for I2Cx peripherals
*/
#define I2C1_PCLK_EN()    ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 22) )
/*
* Clock Enable Macros for USARTx peripherals
*/
#define USART1_PCLK_EN()    ( RCC->APB2ENR |= (1 << 14) )
#define USART2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()    ( RCC->APB1ENR |= (1 << 18) )
/*
* Clock Enable Macros for TIMx peripherals
*/
#define TIM1_PCLK_EN()    ( RCC->APB2ENR |= (1 << 11) )
#define TIM2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 0) )
#define TIM3_PCLK_EN()    ( RCC->APB1ENR |= (1 << 1) )
#define TIM4_PCLK_EN()    ( RCC->APB1ENR |= (1 << 2) )
#define TIM5_PCLK_EN()    ( RCC->APB1ENR |= (1 << 3) )
#define TIM6_PCLK_EN()    ( RCC->APB1ENR |= (1 << 4) )
#define TIM7_PCLK_EN()    ( RCC->APB1ENR |= (1 << 5) )

/*
* Clock Enable Macros for CANx peripherals
*/
#define CAN1_PCLK_EN()    ( RCC->APB1ENR |= (1 << 25) )
#define CAN2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 26) )
/*
* Clock Enable Macros for other peripherals
* TODO: Watch dog, BKP, PWR,
* */

/*
* Clock Disable Macros for GPIOx peripherals
* TODO: Complete all disable macros
*/
#define GPIOA_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 2) )
#define GPIOB_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 3) )
#define GPIOC_PCLK_DI()   ( RCC->APB2ENR &= ~(1 << 4) )
#define AFIO_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 0) )










#endif /* INC_STM32F103XX_H_ */
