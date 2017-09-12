#include <Arduino.h>
#include <SPI.h> // Includes SPI libaray (port 10, 11, 12, 13)
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

/* 

SPI: 1) Data from master to slave (Mosi) 
        2) Data from slave to master (Miso)
        3) Clock (synchronises the data,eg. if the clock rises the slave will save the data )
        4) Slave select determines which slave is adressed

        So,
        pin 10 = slave select
        pin 11 = miso
        pin 12 = mosi
        pin 13 = clock

Not SPI
        pin 9  = enables chip or not to allow radio transmission
        pin 7  = determines ping/pong
        pin A0 = Throttle test
        
*/

#define PAYLOAD_SIZE 8
#define TIMERCOUNTERVALUE 0x83
// #define DEBUG__

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Set input pins
const int pinThrottle = A0;
const int pinRoll = A1;
const int pinPitch = A2;
const int pinYaw = A3;
const int pinSwitch1 = 7;
const int pinSwitch2 = 8;

bool success;

uint16_t payload[PAYLOAD_SIZE];
uint8_t acknowledgePayload[4];

const uint64_t address =  0x00F0F0F0F0; 

void setup(void)
{
#ifdef DEBUG__
  Serial.begin(2000000);
#endif

  radio.begin();
  radio.setRetries(15,15); // 15 times 15 micro seconds retries
  radio.setChannel(101);    // Radio channel expects channel code.
  radio.openWritingaddress(address);
  radio.setPALevel(RF_PWR_HIGH);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();

  // enable radio
  // TODO: make separate function for enable radio
  radio.startListening();

  // Put in transmit mode
  // TODO: rename to transmit mode
  radio.stopListening();

  // set up interrupt so that readings are done with 500Hz
  noInterrupts();
  TCCR2B = 0;
  TCCR2B |= 1 << WGM12;
  TCCR2B |= 1 << CS12;
  TCNT2 = TIMERCOUNTERVALUE;
  TIMSK2 |= 1 << OCIE2A;
  interrupts();
}

ISR(TIMER2_COMPA_vect){
  TCNT2 = TIMERCOUNTERVALUE;
  
  payload[0] = analogRead(pinThrottle);
  payload[1] = analogRead(pinRoll);
  payload[2] = analogRead(pinPitch);
  payload[3] = analogRead(pinYaw);
  payload[4] = digitalRead(pinSwitch1)<<1 | (digitalRead(pinSwitch2) &0x01);

  success = radio.write( &payload, PAYLOAD_SIZE );

  if (radio.isAckPayloadAvailable()){
    success = radio.read(&acknowledgePayload,4);
#ifdef DEBUG__
    Serial.println(acknowledgePayload[1] <<8 | acknowledgePayload[0]);
#endif
  } else {
    
  }
}

void loop(void)
{
}