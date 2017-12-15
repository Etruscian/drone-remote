#include <Arduino.h>
#include <SPI.h> // Includes SPI libaray (port 10, 11, 12, 13)
#include "nRF24L01.h"
#include "RF24.h"

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
        pin A0 = Throttle
        
*/

#define PAYLOAD_SIZE 8
#define TIMERCOUNTERVALUE_500HZ 0x83
// #define DEBUG__

RF24 radio(9,10);

// Set input pins
const int pinThrottle = A3;
const int pinRoll = A0;
const int pinPitch = A1;
const int pinYaw = A2;
const int pinSwitch1 = 8;
const int pinSwitch2 = 7;

const uint16_t rollOffset = 471;
const uint16_t pitchOffset = 527;
const uint16_t yawOffset = 704;

bool success;

uint16_t payload[PAYLOAD_SIZE];
uint8_t acknowledgePayload[4];

void startRadio(void){
  radio.begin();
  radio.setRetries(15,15); // 15 times 15 micro seconds retries

  uint64_t address =  0x00F0F0F0F0;

  radio.openWritingPipe(address); // Open pipe 0 with the specified address
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
}

void setup(void)
{
#ifdef DEBUG__
  Serial.begin(9600);
#endif

  // Configure the switches
  pinMode(pinSwitch1, INPUT_PULLUP);
  pinMode(pinSwitch2, INPUT_PULLUP);

  // Configure the transceiver
  startRadio();

  // set up interrupt so that readings are done with 500Hz
  noInterrupts();
  TCCR2B = 0;
  TCCR2B |= 1 << WGM12;
  TCCR2B |= 1 << CS12;
  TCNT2 = TIMERCOUNTERVALUE_500HZ;
  TIMSK2 |= 1 << OCIE2A;
#ifndef DEBUG__
  interrupts();
#endif
}

ISR(TIMER2_COMPA_vect){
  TCNT2 = TIMERCOUNTERVALUE_500HZ;
  
  payload[0] = (uint16_t)analogRead(pinThrottle);
  payload[1] = (uint16_t)analogRead(pinRoll) - rollOffset;
  payload[2] = (uint16_t)analogRead(pinPitch) - pitchOffset;
  payload[3] = (uint16_t)analogRead(pinYaw) - yawOffset;
  payload[4] = (uint16_t)(!digitalRead(pinSwitch2) &0x01)<<1 | (!digitalRead(pinSwitch2) &0x01); // inverted due to pullup resistor

  success = radio.write(&payload, PAYLOAD_SIZE);

  if (radio.isAckPayloadAvailable()){
    success = radio.read(&acknowledgePayload,4);
    // Serial.println(success);
    // digitalWrite(pinSwitch1,LOW);
  } else {
    
  }
}

void loop(void)
{
#ifdef DEBUG__
  Serial.print(analogRead(pinThrottle));
  Serial.print("\t");
  // Serial.print((int16_t)(analogRead(pinRoll) - rollOffset));
  // Serial.print("\t");
  // Serial.print((int16_t)(analogRead(pinYaw) - yawOffset));
  // Serial.print("\t");
  Serial.println((int16_t)(analogRead(pinPitch) - pitchOffset));//acknowledgePayload[1] <<8 | acknowledgePayload[0]);

  payload[0] = (uint16_t)analogRead(pinThrottle);
  payload[1] = (uint16_t)analogRead(pinRoll) - rollOffset;
  payload[2] = (uint16_t)analogRead(pinPitch) - pitchOffset;
  payload[3] = (uint16_t)analogRead(pinYaw) - yawOffset;
  payload[4] = (uint16_t)(!digitalRead(pinSwitch2) &0x01)<<1 | (!digitalRead(pinSwitch2) &0x01); // inverted due to pullup resistor

  success = radio.write(&payload, PAYLOAD_SIZE);
  if (radio.isAckPayloadAvailable()){
    success = radio.read(&acknowledgePayload,4);
    Serial.println(success);
    // digitalWrite(pinSwitch1,LOW);
  } else {
    
  }
#endif
}