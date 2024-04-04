#ifndef INC_DRV_STM32F407_GPIO_H_
#define INC_DRV_STM32F407_GPIO_H_


#include "stm32f407.h" // target header file


typedef struct{
	uint8_t pinNumber;		// Pin Number of the Port
	uint8_t pinMode;		// (Possible values @GPIO_MODE)
	uint8_t pinOutType;		// Output type (Possible values @GPIO_OUT_TYPE)
	uint8_t pinOutSpeed;		// (Possible values @GPIO_OUT_SPEED)
	uint8_t pinPuPdControl;	// Pull up and Pull down resistors control (Possible values @GPIO_PULL_UPDOWN)
	uint8_t pinAltFModeNum;	// Alternating function mode

} GPIO_PinConfig_t;


typedef struct {
	GPIO_Def_t *p_GPIOx; // Pointer to a certain address where the structure matches setup with the target device
	GPIO_PinConfig_t GPIOx_PinConfig;

} GPIO_Handle_t;


/*
 * GPIO pin modes macros ( @GPIO_MODE )
 */
#define DRV_GPIO_MODE_IN 		0
#define DRV_GPIO_MODE_OUT 		1
#define DRV_GPIO_MODE_ALTFN 	2
#define DRV_GPIO_MODE_ANALOG 	3
#define DRV_GPIO_MODE_IRQ_FT 	4 	// Interrupt for Falling edge
#define DRV_GPIO_MODE_IRQ_RT 	5 	// Interrupt for Rising edge
#define DRV_GPIO_MODE_IRQ_RFT 	6	// Interrupt for Rising & Falling edges


/*
 * GPIO output types macros ( @GPIO_OUT_TYPE )
 */
#define DRV_GPIO_OUTTYPE_PP 	0		// Push Pull mode
#define DRV_GPIO_OUTTYPE_OD 	1		// Open Drain mode


/*
 * GPIO output speed level macros ( @GPIO_OUT_SPEED )
 */
#define DRV_GPIO_OUTSPEED_LOW		0	// Low speed
#define DRV_GPIO_OUTSPEED_MED		1	// Medium speed
#define DRV_GPIO_OUTSPEED_HIGH		2	// High speed
#define DRV_GPIO_OUTSPEED_VHIGH		3	// Very high speed


/*
 * GPIO internal pull-up pull-down resistors macros ( @GPIO_PULL_UPDOWN )
 */
#define DRV_GPIO_PUPDR_NONE		0	// None
#define DRV_GPIO_PUPDR_UP		1	// Pull up resistor enabled
#define DRV_GPIO_PUPDR_DOWN		2	// Pull down resistor enabled



/* ---------------------- APIs supported ---------------------- */

/*
 * Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOxHandle);
void GPIO_DeInit(GPIO_Def_t *p_GPIOx);

/*
 * Peripheral clock control
 */
void GPIO_ClkControl(GPIO_Def_t *p_GPIOx, uint8_t ControlType);

/*
 * Read and Write
 */
uint8_t GPIO_ReadInput_Pin(GPIO_Def_t *p_GPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadInput_Port(GPIO_Def_t *p_GPIOx);
void GPIO_WriteOutput_Pin(GPIO_Def_t *p_GPIOx, uint8_t pinNumber, uint8_t data);
void GPIO_WriteOutput_Port(GPIO_Def_t *p_GPIOx, uint16_t data);
void GPIO_ToggleOutput_Pin(GPIO_Def_t *p_GPIOx, uint8_t pinNumber);

/*
 * IRQ configuration and ISR handling
 */
uint8_t GPIO_getIrqNum(uint8_t pinNumber);
void GPIO_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType);
void GPIO_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority);
void GPIO_IrqHandling(uint8_t pinNumber);


#endif /* INC_DRV_STM32F407_GPIO_H_ */
