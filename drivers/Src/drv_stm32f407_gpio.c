#include "drv_stm32f407_gpio.h"

/*
 * Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOxHandle){

	// Enable the corresponding peripheral clock
	GPIO_ClkControl(p_GPIOxHandle->p_GPIOx, ENABLE);

	uint8_t pinNumber = p_GPIOxHandle->GPIOx_PinConfig.pinNumber;

	// 1. Mode
	p_GPIOxHandle->p_GPIOx->MODER &= ~(0b11 << (pinNumber*2));
	if(p_GPIOxHandle->GPIOx_PinConfig.pinMode < 4 ){
		// non-interrupt mode
		p_GPIOxHandle->p_GPIOx->MODER |= p_GPIOxHandle->GPIOx_PinConfig.pinMode << (pinNumber*2);

	} else {
		// interrupt mode

		// 1.1. Set the pin to input mode
		p_GPIOxHandle->p_GPIOx->MODER |= DRV_GPIO_MODE_IN << (pinNumber*2);

		// 1.2. Set the trigger configuration
		if( p_GPIOxHandle->GPIOx_PinConfig.pinMode == DRV_GPIO_MODE_IRQ_FT ){
			// falling edge trigger
			EXTI->RTSR &= ~(0b1 << pinNumber); // clear RTS
			EXTI->FTSR |= 0b1 << pinNumber; // set FTS

		} else if( p_GPIOxHandle->GPIOx_PinConfig.pinMode == DRV_GPIO_MODE_IRQ_RT ){
			// rising edge trigger
			EXTI->FTSR &= ~(0b1 << pinNumber); // clear FTS
			EXTI->RTSR |= 0b1 << pinNumber; // set RTS

		} else if( p_GPIOxHandle->GPIOx_PinConfig.pinMode == DRV_GPIO_MODE_IRQ_RFT ) {
			// both triggers
			EXTI->FTSR |= 0b1 << pinNumber; // set FTS
			EXTI->RTSR |= 0b1 << pinNumber; // set RTS
		}

		// 1.3. Set the GPIO port selection in SYSCFG_EXTICR
		DRVF_SYSCFG_CLK_EN; // Enabling SYSCFG peripheral clock
		uint8_t portNumber = ((uint32_t)p_GPIOxHandle->p_GPIOx - (uint32_t)DRV_GPIOA_BASEADDR) / (uint16_t)0x0400;
		SYSCFG->EXTICRx[pinNumber/4] &= ~(0b1111 << (pinNumber%4*4));
		SYSCFG->EXTICRx[pinNumber/4] |= portNumber << (pinNumber%4*4);

		// 1.4. Enable EXTI interrupt delivery via IMR
		EXTI->IMR |= 0b1 << pinNumber;
	}


	// 2. & 3. as long as the pin is in the Output mode
	if(p_GPIOxHandle->GPIOx_PinConfig.pinMode == DRV_GPIO_MODE_OUT){
		// 2. Output Speed
		p_GPIOxHandle->p_GPIOx->OSPEEDR &= ~(0b11 << (pinNumber * 2));
		p_GPIOxHandle->p_GPIOx->OSPEEDR |= p_GPIOxHandle->GPIOx_PinConfig.pinOutSpeed << (pinNumber * 2);

		// 3. Output type
		p_GPIOxHandle->p_GPIOx->OTYPER &= ~(0b1 << pinNumber);
		p_GPIOxHandle->p_GPIOx->OTYPER |= p_GPIOxHandle->GPIOx_PinConfig.pinOutType << (pinNumber);
	}


	// 4. Pull up Pull down resistor mode
	p_GPIOxHandle->p_GPIOx->PUPDR &= ~(0b11 << (pinNumber*2));
	p_GPIOxHandle->p_GPIOx->PUPDR |= p_GPIOxHandle->GPIOx_PinConfig.pinPuPdControl << (pinNumber*2);



	// 5. Alternate functionality
	if(p_GPIOxHandle->GPIOx_PinConfig.pinMode == DRV_GPIO_MODE_ALTFN){
		// AFR[pinNum / 8] used to figure whether to change low or high portions of ALTF
		// (pinNum % 8) * 4) used to find corresponding bit position for the pin
		p_GPIOxHandle->p_GPIOx->AFR[pinNumber / 8] &= ~(0b1111 << ((pinNumber % 8) * 4));
		p_GPIOxHandle->p_GPIOx->AFR[pinNumber / 8] |= p_GPIOxHandle->GPIOx_PinConfig.pinAltFModeNum << ((pinNumber % 8) * 4);
	}
}


void GPIO_DeInit(GPIO_Def_t *p_GPIOx){
	uint8_t bitPosition = ((uint32_t)p_GPIOx - (uint32_t)DRV_GPIOA_BASEADDR) / (uint16_t)0x0400;

	DRV_RCC->AHB1RSTR |= 0b1 << bitPosition;
	DRV_RCC->AHB1RSTR &= ~(0b1 << bitPosition);
}


/*
 * Peripheral clock control
 */
void GPIO_ClkControl(GPIO_Def_t *p_GPIOx, uint8_t ControlType){
	uint8_t bitPosition = ((uint32_t)p_GPIOx - (uint32_t)DRV_GPIOA_BASEADDR) / (uint16_t)0x0400;

	if( ControlType == ENABLE ){
		DRVF_GPIOx_PCLK_EN(bitPosition);
	} else {
		DRVF_GPIOx_PCLK_DI(bitPosition);
	}
}


/*
 * Read and Write
 */
uint8_t GPIO_ReadInput_Pin(GPIO_Def_t *p_GPIOx, uint8_t pinNumber){
	return (uint8_t)((p_GPIOx->IDR >> pinNumber) & 0b1);
}

uint16_t GPIO_ReadInput_Port(GPIO_Def_t *p_GPIOx){
	return (uint16_t) p_GPIOx->IDR;
}

void GPIO_WriteOutput_Pin(GPIO_Def_t *p_GPIOx, uint8_t pinNumber, uint8_t data){

	if(data == SET){
		p_GPIOx->ODR |= (uint32_t)(0b1 << pinNumber);
	} else {
		p_GPIOx->ODR &= ~(uint32_t)(0b1 << pinNumber);
	}

}

void GPIO_WriteOutput_Port(GPIO_Def_t *p_GPIOx, uint16_t data){
	p_GPIOx->ODR = (uint32_t)data;
}

void GPIO_ToggleOutput_Pin(GPIO_Def_t *p_GPIOx, uint8_t pinNumber){
	p_GPIOx->ODR ^= (0b1 << pinNumber);
}


/*
 * IRQ configuration and ISR handling
 */
uint8_t GPIO_getIrqNum(uint8_t pinNumber){
	switch(pinNumber){
		case 0:
			return DRV_IRQ_NUM_EXTI0;
		case 1:
			return DRV_IRQ_NUM_EXTI1;
		case 2:
			return DRV_IRQ_NUM_EXTI2;
		case 3:
			return DRV_IRQ_NUM_EXTI3;
		case 4:
			return DRV_IRQ_NUM_EXTI4;
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			return DRV_IRQ_NUM_EXTI9_5;
		default:
			return DRV_IRQ_NUM_EXTI15_10;
	}
}

void GPIO_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType){

	// Enable or Disable functionality
	uint8_t registerNumber = IrqNumber / 32; // corresponding number(0-7) for ISER or ICER
	IrqNumber = IrqNumber % 32; // corresponding bit position

	if(ControlType == ENABLE){
		*( DRV_NVIC_ISER+registerNumber ) |= 0b1 << IrqNumber;
	} else{
		*( DRV_NVIC_ICER+registerNumber ) |= 0b1 << IrqNumber;
	}

}

void GPIO_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority){

	//Priority functionality
	uint32_t registerNumber = IrqNumber / 4; // corresponding number(0-59) for IPR
	uint8_t validBitsPosition = 8 - NVIC_NUM_PRIOR_BITS;

	// IrqNumber = (# of a field) * # of bits per field = 8 + shift to the beginning of valid bits = 4
	IrqNumber = (IrqNumber % 4) * 8 + validBitsPosition; // corresponding bit position


	*( DRV_NVIC_IPR+(registerNumber) ) &= ~(0b1111 << IrqNumber); // clear the bits
	*( DRV_NVIC_IPR+(registerNumber) ) |= (uint32_t)IrqPriority << IrqNumber; // set the bits

}

void GPIO_IrqHandling(uint8_t pinNumber){

	if( EXTI->PR & (0b1 << pinNumber ) ){
		EXTI->PR |= 0b1 << pinNumber; // clear the bit
	}

}



