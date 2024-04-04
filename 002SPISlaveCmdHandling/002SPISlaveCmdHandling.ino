/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *

 */

#include <SPI.h>

const byte led = 9;           // Slave LED digital I/O pin.

boolean ledState = HIGH;      // LED state flag.

uint8_t dataBuff[255];

uint8_t board_id[10] = "ARDUINOUNO";

#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_PRINT_HI         0x50
#define COMMAND_PRINT_BYE        0x51

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  //make SPI as slave
  
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}


//sends one byte of data 
void SPI_SlaveTransmit(uint8_t data)
{
  /* Start transmission */
  SPDR = data;
  
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}
  

// The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
  
  // Initialize slave LED pin.
  pinMode(led, OUTPUT);
  
  digitalWrite(led,LOW);
  
  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}


byte checkData(byte command)
{
  switch(command){
    case 0x50:
    case 0x51:
      return ACK;
      break;
    default:
      return NACK;
      break;
  }
  
}

// The loop function runs continuously after setup().
void loop() 
{
  byte data,command,dummy,ackornack=NACK;
  
  // 1. fist make sure that ss is low . so lets wait until ss is low 
  while(digitalRead(SS) );
  
  // 2. Receive command
  SPI_SlaveTransmit(0x11);
  command = SPI_SlaveReceive();

  //ackornack = checkData(command);
  // 3. Do the command, determine ack or nack
  if(command == COMMAND_PRINT_HI)
  {
    ackornack = ACK;
    Serial.println("HI");
  }else if ( command == COMMAND_PRINT_BYE)
  {
    ackornack = ACK;
    Serial.println("BYE");
  } else {
    ackornack = NACK;
  }

  // 4. Send ack or nack back to the master to conduct a handshake
  SPI_SlaveTransmit(ackornack);
  dummy = SPI_SlaveReceive(); //dummy byte
 
  Serial.println("\n");
}
