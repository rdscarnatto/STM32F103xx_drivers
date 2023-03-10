/*
 * stm32f103xx.h
 *
 *  Created on: Jan 17, 2023
 *      Author: Daniel
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_
#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4
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
//#define SPI1_BASEADDR 		(APB2PERIPH_BASE + 0x3000)

/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t CRL;           /*!< GPIO port configuration register low,         Address offset: 0x00      */
	__vo uint32_t CRH;           /*!< GPIO port configuration register high,     	Address offset: 0x04      */
	__vo uint32_t IDR;			 /*!< GPIO port input data register,     			Address offset: 0x08      */
	__vo uint32_t ODR;			 /*!< GPIO port output data register,     			Address offset: 0x0C      */
	__vo uint32_t BSRR;			 /*!< GPIO Port bit set/reset register,     		Address offset: 0x10      */
	__vo uint32_t BRR;			 /*!< GPIO Port bit reset register,     			Address offset: 0x14      */
	__vo uint32_t LCKR;			 /*!< GPIO Port configuration lock register,     	Address offset: 0x18      */
}GPIO_RegDef_t;

/*
 * RCC registers definitions
*/
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
 * EXTI registers definition
*/
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/*
 * AFIO registers definition
*/
typedef struct
{
	__vo uint32_t EVCR;				/*!< AFIO Event control register						Address offset: 0x00 */
	__vo uint32_t MAPR;				/*!< AFIO Remap and debug I/O configuration register	Address offset: 0x04 */
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED0;
	__vo uint32_t MAPR2;
}AFIO_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;

/*
* peripheral definitions (Peripheral base address typecast to xxx_RegDef_t
*/

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO		((AFIO_RegDef_t*)AFIO_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  		((I2C_RegDef_t*)I2C2_BASEADDR)


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

/*
* Macros to reset GPIOx peripherals
*
*/
#define GPIOA_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOB_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));}while(0)
#define GPIOC_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));}while(0)



/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_i2c_driver.h"
#include "stm32f103xx_rcc_driver.h"

#endif /* INC_STM32F103XX_H_ */
