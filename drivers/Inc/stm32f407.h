/*
	Made by Ruslan Abdulin
				with love.

	-------------------------------
	|--- STM32F407 device file ---|
	-------------------------------

	DRV - Driver related
	DRVF - Driver related function
	DRVV - Driver related value
	p - Pointer
*/

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_
#include <stdint.h>


/*
 * General use macros ---------------------------------------------------
 */
#define __vo volatile
#define __weak __attribute__((weak))
#define SET 1
#define RESET 0
#define ENABLE SET
#define DISABLE RESET
#define HIGH SET
#define LOW RESET
#define NVIC_NUM_PRIOR_BITS 4


/*
 * Pointers -------------------------------------------------------------
 */

/* Basic pointers of NVIC registers */
#define DRV_NVIC_ISER	((__vo uint32_t*) 0xE000E100)	// set-enable; ends at 0xE000E11C
#define DRV_NVIC_ICER	((__vo uint32_t*) 0XE000E180)	// clear-enable; ends at 0xE000E19C
#define DRV_NVIC_IPR	((__vo uint32_t*) 0xE000E400)	// priority; ends at 0xE000E4EF


/*
 * Base Addresses -------------------------------------------------------
 */

/* Base addresses of memory */
#define DRV_FLASH_BASEADDR 0x08000000U
#define DRV_SRAM1_BASEADDR 0x20000000U
#define DRV_SRAM2_BASEADDR 0x2001C000U
#define DRV_ROM_BASEADDR   0x1FFF0000U


/* Base addresses of peripheral buses */
#define DRV_APB1_BASEADDR 0x40000000U
#define DRV_APB2_BASEADDR 0x40010000U
#define DRV_AHB1_BASEADDR 0x40020000U
#define DRV_AHB2_BASEADDR 0x50000000U
#define DRV_RCC_BASEADDR  0x40023800U


/* Base addresses of APB1 peripherals */
#define DRV_SPI2_BASEADDR   (DRV_APB1_BASEADDR + 0x3800U)
#define DRV_SPI3_BASEADDR   (DRV_APB1_BASEADDR + 0x3C00U)
#define DRV_USART2_BASEADDR (DRV_APB1_BASEADDR + 0x4800U)
#define DRV_USART3_BASEADDR (DRV_APB1_BASEADDR + 0x3C00U)
#define DRV_UART4_BASEADDR  (DRV_APB1_BASEADDR + 0x4C00U)
#define DRV_UART5_BASEADDR  (DRV_APB1_BASEADDR + 0x5000U)
#define DRV_UART7_BASEADDR  (DRV_APB1_BASEADDR + 0x7800U)
#define DRV_UART8_BASEADDR  (DRV_APB1_BASEADDR + 0x7C00U)
#define DRV_I2C1_BASEADDR   (DRV_APB1_BASEADDR + 0x5400U)
#define DRV_I2C2_BASEADDR   (DRV_APB1_BASEADDR + 0x5800U)
#define DRV_I2C3_BASEADDR   (DRV_APB1_BASEADDR + 0x5C00U)


/* Base addresses of APB2 peripherals */
#define DRV_USART1_BASEADDR   (DRV_APB2_BASEADDR + 0x1000U)
#define DRV_USART6_BASEADDR   (DRV_APB2_BASEADDR + 0x1400U)
#define DRV_SPI1_BASEADDR     (DRV_APB2_BASEADDR + 0x3000U)
#define DRV_SPI4_BASEADDR     (DRV_APB2_BASEADDR + 0x3400U)
#define DRV_EXTI_BASEADDR     (DRV_APB2_BASEADDR + 0x3C00U)
#define DRV_SYSCFG_BASEADDR   (DRV_APB2_BASEADDR + 0x3800U)


/* Base addresses of AHB1 peripherals */
#define DRV_GPIOA_BASEADDR (DRV_AHB1_BASEADDR)
#define DRV_GPIOB_BASEADDR (DRV_AHB1_BASEADDR + 0x0400U)
#define DRV_GPIOC_BASEADDR (DRV_AHB1_BASEADDR + 0x0800U)
#define DRV_GPIOD_BASEADDR (DRV_AHB1_BASEADDR + 0x0C00U)
#define DRV_GPIOE_BASEADDR (DRV_AHB1_BASEADDR + 0x1000U)
#define DRV_GPIOF_BASEADDR (DRV_AHB1_BASEADDR + 0x1400U)
#define DRV_GPIOG_BASEADDR (DRV_AHB1_BASEADDR + 0x1800U)
#define DRV_GPIOH_BASEADDR (DRV_AHB1_BASEADDR + 0x1C00U)
#define DRV_GPIOI_BASEADDR (DRV_AHB1_BASEADDR + 0x2000U)
#define DRV_GPIOJ_BASEADDR (DRV_AHB1_BASEADDR + 0x2400U)
#define DRV_GPIOK_BASEADDR (DRV_AHB1_BASEADDR + 0x2800U)


/*
 * IRQ numbers ---------------------------------------------------------
 */
#define DRV_IRQ_NUM_EXTI0		6
#define DRV_IRQ_NUM_EXTI1		7
#define DRV_IRQ_NUM_EXTI2		8
#define DRV_IRQ_NUM_EXTI3		9
#define DRV_IRQ_NUM_EXTI4		10
#define DRV_IRQ_NUM_EXTI9_5		23
#define DRV_IRQ_NUM_EXTI15_10	40

#define DRV_IRQ_NUM_SPI1	35
#define DRV_IRQ_NUM_SPI2	36
#define DRV_IRQ_NUM_SPI3	51


/*
 * Bit-position macros for CR1 register of SPI -------------------------
 */
#define DRV_BITPOS_SPI_CR1_BIDIMODE		15
#define DRV_BITPOS_SPI_CR1_BIDIOE		14
#define DRV_BITPOS_SPI_CR1_CRCEN		13
#define DRV_BITPOS_SPI_CR1_CRCNEXT		12
#define DRV_BITPOS_SPI_CR1_DFF			11
#define DRV_BITPOS_SPI_CR1_RXONLY		10
#define DRV_BITPOS_SPI_CR1_SSM			9
#define DRV_BITPOS_SPI_CR1_SSI			8
#define DRV_BITPOS_SPI_CR1_LSBFIRST		7
#define DRV_BITPOS_SPI_CR1_SPE			6
#define DRV_BITPOS_SPI_CR1_BR			3
#define DRV_BITPOS_SPI_CR1_MSTR			2
#define DRV_BITPOS_SPI_CR1_CPOL			1
#define DRV_BITPOS_SPI_CR1_CPHA			0


/*
 * Bit-position macros for CR2 register of SPI -------------------------
 */
#define DRV_BITPOS_SPI_CR2_TXEIE			7
#define DRV_BITPOS_SPI_CR2_RXNEIE			6
#define DRV_BITPOS_SPI_CR2_ERRIE			5
#define DRV_BITPOS_SPI_CR2_FRF				4
#define DRV_BITPOS_SPI_CR2_SSOE				2
#define DRV_BITPOS_SPI_CR2_TXDMAEN			1
#define DRV_BITPOS_SPI_CR2_RXDMAEN			0


/*
 * Bit-position macros for SR register of SPI -------------------------
 */
#define DRV_BITPOS_SPI_SR_FRE			8
#define DRV_BITPOS_SPI_SR_BSY			7
#define DRV_BITPOS_SPI_SR_OVR			6
#define DRV_BITPOS_SPI_SR_MODF			5
#define DRV_BITPOS_SPI_SR_CRCERR		4
#define DRV_BITPOS_SPI_SR_UDR			3
#define DRV_BITPOS_SPI_SR_CHSIDE		2
#define DRV_BITPOS_SPI_SR_TXE			1
#define DRV_BITPOS_SPI_SR_RXNE			0


/*
 * Registers structures ------------------------------------------------
 */

/* GPIO registers structure */
typedef struct{
	__vo uint32_t MODER; //   mode type
	__vo uint32_t OTYPER; //  output type
	__vo uint32_t OSPEEDR; // output speed
	__vo uint32_t PUPDR; //   internal pull up pull down resistors
	__vo uint32_t IDR; //     input data
	__vo uint32_t ODR; //     output data
	__vo uint32_t BSRR; //    port bit set reset
	__vo uint32_t LCKR; //    port configuration lock
	__vo uint32_t AFR[2]; //    alternate function low and high portions
} GPIO_Def_t;



/* RCC registers structure */
typedef struct{
	__vo uint32_t CR; //         clock control
	__vo uint32_t PLLCFGR; //    PLL clock output configuration
	__vo uint32_t CFGR; //       clock configuration
	__vo uint32_t CIR; //        clock interrupt
	__vo uint32_t AHB1RSTR; //   AHB1 peripheral reset
	__vo uint32_t AHB2RSTR; //   AHB1 peripheral reset
	__vo uint32_t AHB3RSTR; //   AHB3 peripheral reset
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR; //   APB1 peripheral reset
	__vo uint32_t APB2RSTR; //   APB2 peripheral reset
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	__vo uint32_t AHB1ENR; //    AHB1 enable
	__vo uint32_t AHB2ENR; //    AHB2 enable
	__vo uint32_t AHB3ENR; //    AHB3 enable
	uint32_t RESERVED3;
	__vo uint32_t APB1ENR; //    APB1 enable
	__vo uint32_t APB2ENR; //    APB2 enable
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR; //  AHB1 enable in low power mode
	__vo uint32_t AHB2LPENR; //  AHB2 enable in low power mode
	__vo uint32_t AHB3LPENR; //  AHB3 enable in low power mode
	uint32_t RESERVED6;
	__vo uint32_t APB1LPENR; //  APB1 enable in low power mode
	__vo uint32_t APB2LPENR; //  APB2 enable in low power mode
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	__vo uint32_t BDCR; //       backup domain control
	__vo uint32_t CSR; //        clock control and status
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	__vo uint32_t SSCGR; //      spread spectrum clock generation
	__vo uint32_t PLLI2SCFGR; // PLL I2C clock output configuration

} RCC_Def_t;

#define DRV_RCC ((RCC_Def_t*) DRV_RCC_BASEADDR)


/* EXTI registers structure */
typedef struct{
	__vo uint32_t IMR; //	Interrupt Mask Register
	__vo uint32_t EMR; //	Event Mask Register
	__vo uint32_t RTSR; //	Rising Trigger Selection Register
	__vo uint32_t FTSR; //	Falling Trigger Selection Register
	__vo uint32_t SWIER; //	Software Interrupt Event Register
	__vo uint32_t PR; //	Pending Register

} EXTI_Def_t;

#define EXTI ((EXTI_Def_t*) DRV_EXTI_BASEADDR)


/* SYSCFG registers structure */
typedef struct{
	__vo uint32_t MEMRMP; //		Memory Remap Register
	__vo uint32_t PMC; //			Peripheral Mode Configuration Register
	__vo uint32_t EXTICRx[4]; //	External Interrupt Configuration Registers
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	__vo uint32_t CMPCR; //			Compensation Cell Control Register
} SYSCFG_Def_t;

#define SYSCFG ((SYSCFG_Def_t*) DRV_SYSCFG_BASEADDR)


typedef struct {
	uint32_t CR1;		// Control configurations 1
	uint32_t CR2;		// Control configurations 2
	uint32_t SR;		// Status
	uint32_t DR;		// Data
	uint32_t CRCPR;		// CRC polynomial
	uint32_t RXCRCR;	// RX (Receiving) buffer CRC
	uint32_t TXCRCR;	// TX (Sending) buffer CRC
	uint32_t I2SCFGR;	// I2S protocol configurations
	uint32_t I2SPR;		// I2S prescaler
} SPI_Def_t;

#define SPI1 ((SPI_Def_t*)  DRV_SPI1_BASEADDR)
#define SPI2 ((SPI_Def_t*)  DRV_SPI2_BASEADDR)
#define SPI3 ((SPI_Def_t*)  DRV_SPI3_BASEADDR)
#define SPI4 ((SPI_Def_t*)  DRV_SPI4_BASEADDR)


/*
 * Enable macros ----------------------------------------------------
 */

/* Clock enable macros for GPIO */
#define DRVF_GPIOx_PCLK_EN(bitNumToSet) ( DRV_RCC->AHB1ENR |= 0b1 << bitNumToSet )
#define DRVF_GPIOA_PCLK_EN ( DRV_RCC->AHB1ENR |= 0b1 )
#define DRVF_GPIOB_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 1) )
#define DRVF_GPIOC_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 2) )
#define DRVF_GPIOD_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 3) )
#define DRVF_GPIOE_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 4) )
#define DRVF_GPIOF_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 5) )
#define DRVF_GPIOG_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 6) )
#define DRVF_GPIOH_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 7) )
#define DRVF_GPIOI_PCLK_EN ( DRV_RCC->AHB1ENR |= (0b1 << 8) )


/* Clock enable macros for I2C */
#define DRVF_I2C1_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 21) )
#define DRVF_I2C2_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 22) )
#define DRVF_I2C3_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 23) )


/* Clock enable macros for SPI */
#define DRVF_SPI1_CLK_EN ( DRV_RCC->APB2ENR |= (0b1 << 12) )
#define DRVF_SPI2_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 14) )
#define DRVF_SPI3_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 15) )
#define DRVF_SPI4_CLK_EN ( DRV_RCC->APB2ENR |= (0b1 << 13) )


/* Clock enable macros for USART */
#define DRVF_USART1_CLK_EN ( DRV_RCC->APB2ENR |= (0b1 << 4) )
#define DRVF_USART2_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 17) )
#define DRVF_USART3_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 18) )
#define DRVF_UART4_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 19) )
#define DRVF_UART5_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 20) )
#define DRVF_USART6_CLK_EN ( DRV_RCC->APB2ENR |= (0b1 << 5) )


/* Clock enable macros for SYSCFG */
#define DRVF_SYSCFG_CLK_EN ( DRV_RCC->APB2ENR |= (0b1 << 14) )


/* Clock disable macros for GPIO */
#define DRVF_GPIOx_PCLK_DI(bitNumToSet) ( DRV_RCC->AHB1ENR &= ~0b1 << bitNumToSet )
#define DRVF_GPIOA_PCLK_DI ( DRV_RCC->AHB1ENR &= ~0b1 )
#define DRVF_GPIOB_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 1) )
#define DRVF_GPIOC_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 2) )
#define DRVF_GPIOD_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 3) )
#define DRVF_GPIOE_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 4) )
#define DRVF_GPIOF_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 5) )
#define DRVF_GPIOG_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 6) )
#define DRVF_GPIOH_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 7) )
#define DRVF_GPIOI_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0b1 << 8) )


/* Clock disable macros for I2C */
#define DRVF_I2C1_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 21) )
#define DRVF_I2C2_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 22) )
#define DRVF_I2C3_CLK_EN ( DRV_RCC->APB1ENR |= (0b1 << 23) )


/* Clock disable macros for SPI */
#define DRVF_SPI1_CLK_DI ( DRV_RCC->APB2ENR &= ~(0b1 << 12) )
#define DRVF_SPI2_CLK_DI ( DRV_RCC->APB1ENR &= ~(0b1 << 14) )
#define DRVF_SPI3_CLK_DI ( DRV_RCC->APB1ENR &= ~(0b1 << 15) )
#define DRVF_SPI4_CLK_DI ( DRV_RCC->APB2ENR &= ~(0b1 << 13) )


/* Clock disable macros for USART */
#define DRVF_USART1_CLK_DI ( DRV_RCC->APB2ENR &= ~(0b1 << 4) )
#define DRVF_USART2_CLK_DI ( DRV_RCC->APB1ENR &= ~(0b1 << 17) )
#define DRVF_USART3_CLK_DI ( DRV_RCC->APB1ENR &= ~(0b1 << 18) )
#define DRVF_UART4_CLK_DI ( DRV_RCC->APB1ENR &= ~(0b1 << 19) )
#define DRVF_UART5_CLK_DI ( DRV_RCC->APB1ENR &= ~(0b1 << 20) )
#define DRVF_USART6_CLK_DI ( DRV_RCC->APB2ENR &= ~(0b1 << 5) )


/* Clock disable macros for SYSCFG */
#define DRVF_SYSCFG_CLK_DI ( DRV_RCC->APB2ENR &= ~(0b1 << 14) )

#endif /* INC_STM32F407_H_ */

