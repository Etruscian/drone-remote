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
#define LEDPIN 13

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Set input pin
const int pin_throttle = A0;    // set A0 as input
const int pin_roll = A1;
const int pin_pitch = A2;
const int pin_yaw = A3;
const int pin_arm = 8;
const int pin_acro = 7;
bool ok;
uint16_t payload[PAYLOAD_SIZE];

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x00F0F0F0F0LL, 0x00F0F0F0F0LL }; 
//LL in the end means long long type, also declared by uint64_t (dus overbodig).

void setup(void)
{
 Serial.begin(9600);    //set usb speed
//  printf_begin();         //run printf file to be able to print
//  printf("\n\rRF24/examples/pingpair/\n\r");

  //
  // Setup and configure rf radio
  //

  radio.begin();        // initiliaze radio file "RF24.h".
  Serial.println(radio.get_status(),HEX);
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15); // 15 times 15 micro seconds retries

  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(PAYLOAD_SIZE);  // number of bytes (total bits = bytes *8 bits)
  radio.setChannel(101);    // Radio channel expects channel code.
  //
  // Open pipes to other nodes for communication
  //

  radio.openWritingPipe(pipes[0]);      // use first adress in pipes
  radio.openReadingPipe(1,pipes[1]);    // use second adress in pipes

  radio.setPALevel(RF_PWR_HIGH);
  
  //
  // Start listening
  //
  radio.startListening();                 // enable radio

  // First, stop listening so we can talk.
  radio.stopListening();

  pinMode(pin_acro, INPUT);
  pinMode(pin_arm, INPUT);

  // set up interrupt so that readings are done with 500Hz
  noInterrupts();
  TCCR2B = 0;
  TCCR2B |= 1 << WGM12;
  TCCR2B |= 1 << CS12;
  TCNT2 = 0x83;
  TIMSK2 |= 1 << OCIE2A;
  interrupts();
}  // end setup

ISR(TIMER2_COMPA_vect){
  
      // Read input
  TCNT2 = 0x83;
  payload[0] = analogRead(pin_throttle);
  payload[1] = analogRead(pin_roll);
  payload[2] = analogRead(pin_pitch);
  payload[3] = analogRead(pin_yaw);
  payload[4] = (digitalRead(pin_acro)&0x01)<<1 | (digitalRead(pin_arm)&0x01);

  ok = radio.write( &payload, PAYLOAD_SIZE ); // return value named "ok" if acknowledge is received within timeout.
  
  digitalWrite(LEDPIN, 0);
  if (!ok)
    digitalWrite(LEDPIN,1);
}

void loop(void)
{
}