#include "drv_stm32f407_spi.h"

/*
 * Functions-helpers. Created to be used by software only. Non-API.
 */

static void SPI_TXE_IRQ_Handler( SPI_Handle_t *p_SPI_Handle_t ){

	// Check Data format and load the appropriate protion to the SPI->DR register
	if (p_SPI_Handle_t->DFF) {
		//16 bits DFF
		p_SPI_Handle_t->p_SPI_struct->DR = *((uint16_t*) p_SPI_Handle_t->p_TxBuffer);
		p_SPI_Handle_t->TxLen -= 2;
		(uint16_t*) p_SPI_Handle_t->p_TxBuffer++;
	} else {
		//8 bits DFF
		p_SPI_Handle_t->p_SPI_struct->DR = *(p_SPI_Handle_t->p_TxBuffer);
		p_SPI_Handle_t->TxLen -= 1;
		p_SPI_Handle_t->p_TxBuffer++;
	}

	// Check if there is no more data to send
	if( p_SPI_Handle_t->TxLen < 1 ){
		// Turn off the interrupt. Change SPI state
		p_SPI_Handle_t->p_SPI_struct->CR2 &= ~(0b1 << DRV_BITPOS_SPI_CR2_TXEIE);
		p_SPI_Handle_t->State = DRV_STATE_SPI_READY;

		SPI_ApplicationEventCallback( p_SPI_Handle_t, DRV_EVENT_SPI_TX_CMPLT );
	}

}

static void SPI_RXNE_IRQ_Handler( SPI_Handle_t *p_SPI_Handle_t ){

	// Check Data format and load the appropriate protion to the SPI->DR register
	if ( p_SPI_Handle_t->DFF ) {
		//16 bits DFF
		*((uint16_t*) p_SPI_Handle_t->p_RxBuffer) = p_SPI_Handle_t->p_SPI_struct->DR;
		p_SPI_Handle_t->RxLen -= 2;
		(uint16_t*) p_SPI_Handle_t->p_RxBuffer++;
	} else {
		//8 bits DFF
		*(p_SPI_Handle_t->p_RxBuffer) = p_SPI_Handle_t->p_SPI_struct->DR;
		p_SPI_Handle_t->RxLen -= 1;
		p_SPI_Handle_t->p_RxBuffer++;
	}

	// Check if there is no more data to send
	if (p_SPI_Handle_t->RxLen < 1) {
		// Turn off the interrupt. Change SPI state
		p_SPI_Handle_t->p_SPI_struct->CR2 &= ~(0b1 << DRV_BITPOS_SPI_CR2_RXNEIE);
		p_SPI_Handle_t->State = DRV_STATE_SPI_READY;

		SPI_ApplicationEventCallback(p_SPI_Handle_t, DRV_EVENT_SPI_RX_CMPLT);
	}

}

static void SPI_OVR_IRQ_Handler( SPI_Handle_t *p_SPI_Handle_t ){

	// Clear the OVR flag
	if( p_SPI_Handle_t->State != DRV_STATE_SPI_BUSY_TX ){
		uint8_t dummy = p_SPI_Handle_t->p_SPI_struct->DR;
		dummy = p_SPI_Handle_t->p_SPI_struct->SR;
		(void)dummy;
	}

	SPI_ApplicationEventCallback(p_SPI_Handle_t, DRV_EVENT_SPI_OVR_ERR);

}

// ----------------------------------------------------------------------------


/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *p_SPI_Handle_t){

	// Enable the corresponding peripheral clock
	SPI_ClkControl(p_SPI_Handle_t->p_SPI_struct, ENABLE);

	// Disable the SPI peripheral
	//SPI_PeripheralControl(p_SPI_Handle_t->p_SPI_struct, DISABLE);

	uint32_t newCR1 = 0b0;

	// Set device's role
	newCR1 |= (p_SPI_Handle_t->SPI_Config.DeviceMode << DRV_BITPOS_SPI_CR1_MSTR);

	// Set SPI protocol mode( FD doesn't require any actions )
	if( p_SPI_Handle_t->SPI_Config.BusConfig == DRV_SPI_BUS_CONFIG_HD ){
		// Half Duplex
		newCR1 |= (0b1 << DRV_BITPOS_SPI_CR1_BIDIMODE);	// BIDIMODE set
	} else if( p_SPI_Handle_t->SPI_Config.BusConfig == DRV_SPI_BUS_CONFIG_RXONLY ){
		// Simplex
		newCR1 |= (0b1 << DRV_BITPOS_SPI_CR1_RXONLY);	// RXONLY set while BIDIMODE is cleared
	}

	// Set Baud Rate
	newCR1 |= (p_SPI_Handle_t->SPI_Config.SclkSpeed << DRV_BITPOS_SPI_CR1_BR);

	// Set Data Frame Format (Shift reg. = 8 or 16 bits)
	newCR1 |= (p_SPI_Handle_t->SPI_Config.DFF << DRV_BITPOS_SPI_CR1_DFF);

	// Set CPOL
	newCR1 |= (p_SPI_Handle_t->SPI_Config.CPOL << DRV_BITPOS_SPI_CR1_CPOL);

	// Set CPHA
	newCR1 |= (p_SPI_Handle_t->SPI_Config.CPHA << DRV_BITPOS_SPI_CR1_CPHA);

	// Set SSM
	newCR1 |= (p_SPI_Handle_t->SPI_Config.SSM << DRV_BITPOS_SPI_CR1_SSM);


	// Apply the configurations
	p_SPI_Handle_t->p_SPI_struct->CR1 = newCR1;

	// Enable the SPI peripheral
	//SPI_PeripheralControl(p_SPI_Handle_t->p_SPI_struct, ENABLE);
}


// !!! TO BE IMPLEMENTED: CR1_BIDIOE for half duplex and CR1_SSI for determing Slave status
void SPI_DeInit(SPI_Def_t *p_SPI_struct){

}

/*
 * Peripheral clock control
 */
void SPI_ClkControl(SPI_Def_t *p_SPI_struct, uint8_t ControlType){
	if(ControlType == 1){

		if(p_SPI_struct == SPI1){
			DRVF_SPI1_CLK_EN;
		} else if(p_SPI_struct == SPI2){
			DRVF_SPI2_CLK_EN;
		} else if(p_SPI_struct == SPI3){
			DRVF_SPI3_CLK_EN;
		} else if(p_SPI_struct == SPI4){
			DRVF_SPI4_CLK_EN;
		}

	} else {

		if(p_SPI_struct == SPI1){
			DRVF_SPI1_CLK_DI;
		} else if (p_SPI_struct == SPI2) {
			DRVF_SPI2_CLK_DI;
		} else if (p_SPI_struct == SPI3) {
			DRVF_SPI3_CLK_DI;
		} else if(p_SPI_struct == SPI4){
			DRVF_SPI4_CLK_DI;
		}

	}
}


/*
 * Get bit-position status
 */
uint8_t SPI_SR_Status( SPI_Def_t *p_SPI_struct, uint8_t bitPosition){
	if((p_SPI_struct->SR >> bitPosition) & 0b1){
		return HIGH;
	}

	return LOW;
}

/*
 * Data Sending and Receiving
 */
void SPI_SendData( SPI_Def_t *p_SPI_struct, uint8_t *p_TxBuffer, uint32_t length ){

	while (length > 0) {
		// Wait until TX buffer becomes empty
		while (!SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_SR_TXE));

		if (SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_CR1_DFF)) {
			//16 bits DFF
			p_SPI_struct->DR = *((uint16_t*) p_TxBuffer);
			length -= 2;
			(uint16_t*) p_TxBuffer++;
		} else {
			//8 bits DFF
			p_SPI_struct->DR = *(p_TxBuffer);
			length--;
			p_TxBuffer++;
		}
	}

}

void SPI_ReceiveData( SPI_Def_t *p_SPI_struct, uint8_t *p_RxBuffer, uint32_t length ){

	while( length > 0){

		// Wait until Rx buffer becomes non-empty
		while( !SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_SR_RXNE) );

		if (SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_CR1_DFF)) {
			//16 bits DFF
			*((uint16_t*)p_RxBuffer) = p_SPI_struct->DR;
			length -= 2;
			(uint16_t*) p_RxBuffer++;
		} else {
			//8 bits DFF
			*(p_RxBuffer) = p_SPI_struct->DR;
			length--;
			p_RxBuffer++;
		}

	}

}

uint8_t SPI_SendDataIRQ( SPI_Handle_t *p_SPI_Handle_t, uint8_t *p_TxBuffer, uint32_t length ){

	if( p_SPI_Handle_t->State == DRV_STATE_SPI_BUSY_TX){
		return p_SPI_Handle_t->State;
	}

	// Save Tx buffer adress and data length information
	p_SPI_Handle_t->p_TxBuffer = p_TxBuffer; // data to be sent
	p_SPI_Handle_t->TxLen = length; // length of that data
	p_SPI_Handle_t->DFF = SPI_SR_Status(p_SPI_Handle_t->p_SPI_struct, DRV_BITPOS_SPI_CR1_DFF); // length of each portion

	// Change SPI state
	p_SPI_Handle_t->State = DRV_STATE_SPI_BUSY_TX;

	// Enable the TXEIE control bit to generate interrupt whenever TXE flag is set in SR
	p_SPI_Handle_t->p_SPI_struct->CR2 |= 0b1 << DRV_BITPOS_SPI_CR2_TXEIE;

	// ! Data transmission is handled by the ISR
	return p_SPI_Handle_t->State;
}

uint8_t SPI_ReceiveDataIRQ( SPI_Handle_t *p_SPI_Handle_t, uint8_t *p_RxBuffer, uint32_t length ){

	if (p_SPI_Handle_t->State == DRV_STATE_SPI_BUSY_RX) {
		return p_SPI_Handle_t->State;
	}

	// Save Rx buffer adress and data length information
	p_SPI_Handle_t->p_RxBuffer = p_RxBuffer; // variable where the received data will be saved
	p_SPI_Handle_t->RxLen = length; // length of the data to be received
	p_SPI_Handle_t->DFF = SPI_SR_Status(p_SPI_Handle_t->p_SPI_struct, DRV_BITPOS_SPI_CR1_DFF); // length of each portion

	// Change SPI state
	p_SPI_Handle_t->State = DRV_STATE_SPI_BUSY_RX;

	// Enable the RXNEIE control bit to generate interrupt whenever RXNE flag is set in SR
	p_SPI_Handle_t->p_SPI_struct->CR2 |= 0b1 << DRV_BITPOS_SPI_CR2_RXNEIE;

	// ! Data transmission is handled by the ISR
	return p_SPI_Handle_t->State;

}

/*
 * Enable or Disable SPI peripheral
 */
void SPI_PeripheralControl( SPI_Def_t *p_SPI_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_SPI_struct->CR1 |= (0b1 << DRV_BITPOS_SPI_CR1_SPE);
	} else {
		p_SPI_struct->CR1 &= ~(0b1 << DRV_BITPOS_SPI_CR1_SPE);
	}
}

/*
 * SSI(value that overwrites NSS pin's value) control
 */
void SPI_SSIControl( SPI_Def_t *p_SPI_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_SPI_struct->CR1 |= (0b1 << DRV_BITPOS_SPI_CR1_SSI);
	} else {
		p_SPI_struct->CR1 &= ~(0b1 << DRV_BITPOS_SPI_CR1_SSI);
	}
}

/*
 * SSOE(enables or disables NSS output) control
 */
void SPI_SSOEControl( SPI_Def_t *p_SPI_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_SPI_struct->CR2 |= (0b1 << DRV_BITPOS_SPI_CR2_SSOE);
	} else {
		p_SPI_struct->CR2 &= ~(0b1 << DRV_BITPOS_SPI_CR2_SSOE);
	}
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType){
	// Enable or Disable functionality
	uint8_t registerNumber = IrqNumber / 32; // corresponding number(0-7) for ISER or ICER
	IrqNumber = IrqNumber % 32; // corresponding bit position

	if (ControlType == ENABLE) {
		*( DRV_NVIC_ISER + registerNumber) |= 0b1 << IrqNumber;
	} else {
		*( DRV_NVIC_ICER + registerNumber) |= 0b1 << IrqNumber;
	}
}

void SPI_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority){
	//Priority functionality
	uint32_t registerNumber = IrqNumber / 4; // corresponding number(0-59) for IPR
	uint8_t validBitsPosition = 8 - NVIC_NUM_PRIOR_BITS;

	// IrqNumber = (# of a field) * # of bits per field = 8 + shift to the beginning of valid bits = 4
	IrqNumber = (IrqNumber % 4) * 8 + validBitsPosition; // corresponding bit position

	*( DRV_NVIC_IPR + (registerNumber)) &= ~(0b1111 << IrqNumber); // clear the bits
	*( DRV_NVIC_IPR + (registerNumber)) |= (uint32_t) IrqPriority << IrqNumber; // set the bits
}

void SPI_IrqHandling(SPI_Handle_t* p_SPI_Handle_t){
	// Check what caused the interrupt
	uint8_t conditionTXE = p_SPI_Handle_t->p_SPI_struct->SR & (0b1 << DRV_BITPOS_SPI_SR_TXE);
	uint8_t conditionRXNE = p_SPI_Handle_t->p_SPI_struct->SR & (0b1 << DRV_BITPOS_SPI_SR_RXNE);
	uint8_t conditionOVR = p_SPI_Handle_t->p_SPI_struct->SR & (0b1 << DRV_BITPOS_SPI_SR_OVR);

	if( conditionTXE && p_SPI_Handle_t->State == DRV_STATE_SPI_BUSY_TX){
		// TXE = 1 => Tx is ready to be written to
		SPI_TXE_IRQ_Handler( p_SPI_Handle_t);
	}

	if( conditionRXNE && p_SPI_Handle_t->State == DRV_STATE_SPI_BUSY_RX ){
		// RXNE = 1 => Rx is ready to be read
		SPI_RXNE_IRQ_Handler( p_SPI_Handle_t);
	}

	if( conditionOVR && p_SPI_Handle_t->p_SPI_struct->CR2 & (0b1 << DRV_BITPOS_SPI_CR2_ERRIE) ){
		// RXNE = 1 => Rx is ready to be read
		SPI_OVR_IRQ_Handler( p_SPI_Handle_t);
	}
}

/*
 * Event callback(a function meant to be used after some other function is used. In this case, after SPI_IrqHandling finished its execution)
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t* p_SPI_Handle_t, uint8_t event){
	// To be overwritten by the user. This is strictly an API function.
}


