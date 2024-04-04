#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407.h"
#include "drv_stm32f407_gpio.h"
#include "drv_stm32f407_spi.h"

/*
 * !!!  TO DO:  !!!
 * IMPLEMENT TIMERS
 */

/*
 * SPI pins:
 * PA0 - User button
 * PA4 - SPI1_NSS
 * PA5 - SPI1_SCK
 * PA6 - SPI1_MISO
 * PA7 - SPI1_MOSI
 * Alternate functionality mode # for SPI = 5
 */


/*
 * Macros for commands and ACK
 */
#define COMMAND_PRINT_HI		0x50
#define COMMAND_PRINT_BYE		0x51
#define ACK						0xF5

/*
 * Global variables
 */
SPI_Handle_t SPI;
GPIO_Handle_t GPIO;
uint8_t data_to_send = COMMAND_PRINT_BYE;
uint8_t data_to_receive = 0x00;

// Poorly created delay - To be substituted with a timer
void debounce(){
	for(uint32_t i = 0; i < 600000; i++){}
}

void SPI_ApplicationEventCallback(SPI_Handle_t* p_SPI_Handle_t, uint8_t event){
	switch( event ){
		case DRV_EVENT_SPI_RX_CMPLT:
			printf("Reception is complete");
		break;

		case DRV_EVENT_SPI_TX_CMPLT:
			printf("Transfer is complete");
		break;

		case DRV_EVENT_SPI_OVR_ERR:
			printf("Overrun error");
		break;

		default:
			printf("Unrecognized event");
		break;
	}
}

void SPI_doCycle( SPI_Handle_t* SPI, uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t tx_length, uint8_t rx_length ){
	SPI_SendDataIRQ(SPI, tx_buffer, tx_length);
	while (SPI->State != DRV_STATE_SPI_READY);
	SPI_ReceiveDataIRQ(SPI, rx_buffer, rx_length);
	while (SPI->State != DRV_STATE_SPI_READY);
}


int main(){

	// GPIO declarations and basic initializations
	GPIO.p_GPIOx = (GPIO_Def_t*) DRV_GPIOA_BASEADDR;
	GPIO.GPIOx_PinConfig.pinMode = DRV_GPIO_MODE_ALTFN;
	GPIO.GPIOx_PinConfig.pinAltFModeNum = 5;

	// SPI pins PA4 to PA7 initialization
	GPIO.GPIOx_PinConfig.pinNumber = 4;
	GPIO_Init(&GPIO);
	GPIO.GPIOx_PinConfig.pinNumber = 5;
	GPIO_Init(&GPIO);
	GPIO.GPIOx_PinConfig.pinNumber = 6;
	GPIO_Init(&GPIO);
	GPIO.GPIOx_PinConfig.pinNumber = 7;
	GPIO_Init(&GPIO);

	// LED pinA3 initialization
	GPIO.GPIOx_PinConfig.pinNumber = 3;
	GPIO.GPIOx_PinConfig.pinMode = DRV_GPIO_MODE_OUT;
	GPIO.GPIOx_PinConfig.pinOutType = DRV_GPIO_OUTTYPE_PP;
	GPIO.GPIOx_PinConfig.pinPuPdControl = DRV_GPIO_PUPDR_NONE;
	GPIO_Init(&GPIO);

	// User Btn
	GPIO.GPIOx_PinConfig.pinNumber = 0;
	GPIO.GPIOx_PinConfig.pinMode = DRV_GPIO_MODE_IRQ_RT;
	GPIO.GPIOx_PinConfig.pinPuPdControl = DRV_GPIO_PUPDR_DOWN;
	GPIO_Init(&GPIO);
	uint8_t IRQnum = GPIO_getIrqNum(GPIO.GPIOx_PinConfig.pinNumber);
	GPIO_IrqPriorityConfig(IRQnum, 13);
	GPIO_IrqInterruptConfig(IRQnum, ENABLE);

	/* ------------------------------------------------------------------- */

	// SPI handler initialization
	SPI.p_SPI_struct = (SPI_Def_t*) DRV_SPI1_BASEADDR;

	// SPI1 initialization
	SPI.SPI_Config.DeviceMode = DRV_SPI_DEVICE_MODE_MASTER;
	SPI.SPI_Config.BusConfig = DRV_SPI_BUS_CONFIG_FD;
	SPI.SPI_Config.SclkSpeed = DRV_SPI_SCLK_SPEED_DIV8;
	SPI.SPI_Config.DFF = DRV_SPI_DFF_8BITS;
	SPI.SPI_Config.CPHA = DRV_SPI_CPHA_FIRST;
	SPI.SPI_Config.CPOL = DRV_SPI_CPOL_LOW;
	SPI.SPI_Config.SSM = DRV_SPI_SSM_DISABLE;
	SPI.State = DRV_STATE_SPI_READY;

	SPI_Init(&SPI);
	SPI_SSOEControl(SPI.p_SPI_struct, ENABLE);
	GPIO_IrqPriorityConfig(DRV_IRQ_NUM_SPI1, 12);
	SPI_IrqInterruptConfig(DRV_IRQ_NUM_SPI1, ENABLE);


	while(1){}

	return 0;
}

void EXTI0_IRQHandler(){
	GPIO_IrqHandling(GPIO.GPIOx_PinConfig.pinNumber);

	// Enable SPI
	SPI_PeripheralControl(SPI.p_SPI_struct, ENABLE);

	// Send command & dummy read
	SPI_doCycle(&SPI, &data_to_send, &data_to_receive, 0b1, 0b1);

	// Send dummy & valid read
	SPI_doCycle(&SPI, &data_to_send, &data_to_receive, 0b1, 0b1);

	// If received data is valid, toggle the LED
	if (data_to_receive == ACK) {
		GPIO_ToggleOutput_Pin(GPIO.p_GPIOx, 3);
		debounce();
		GPIO_ToggleOutput_Pin(GPIO.p_GPIOx, 3);
	}

	// Disable the peripheral
	while (SPI_SR_Status(SPI.p_SPI_struct, DRV_BITPOS_SPI_SR_BSY));
	SPI_PeripheralControl(SPI.p_SPI_struct, DISABLE);

	debounce();
}


void SPI1_IRQHandler(){
	SPI_IrqHandling(&SPI);
}


/*
 * Questions:
 * 1 - What is a length in send data function. Why do we have a while loop with it?
 * 2 - Where do I plug in data to send it?
 */
