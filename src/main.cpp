#include <Arduino.h>
#include <SPI.h> // Includes SPI libaray (port 10, 11, 12, 13)
#include "nRF24L01.h"
#include "RF24.h"
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>

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
#define DEBUG__

RF24 radio(9, 10);
SSD1306AsciiAvrI2c oled;

volatile uint8_t packetReceived = 0;
volatile uint8_t signalStrength = 0;
uint8_t signalStrengthArray[256] = {0};
uint8_t pos = 0;
uint16_t sum = 0;
uint8_t txBatteryLevel = 0;
uint8_t oldtxBatteryLevel = 0;
uint8_t rxBatteryLevel = 0;
uint8_t oldrxBatteryLevel = 0;
uint8_t oldsignalStrength = 0;

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

void startRadio(void)
{
  radio.begin();
  radio.setRetries(1, 5); // 5 times 5 micro seconds retries

  uint64_t address = 0x007FFFFFFF;
  radio.setChannel(101);
  radio.openWritingPipe(address); // Open pipe 0 with the specified address
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
}

void startScreen(void)
{
  oled.begin(&Adafruit128x32, 0x3C);
  oled.setFont(Stang5x7); //System5x7
  oled.clear();
  oled.setCursor(60, 0);
  oled.print("TX:");
  oled.setCursor(60, 1);
  oled.print("RX:");
  oled.setCursor(48, 3);
  oled.print("TXRX:");
}

uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}

void setup(void)
{
#ifdef DEBUG__
  Serial.begin(9600);
#endif
  // Serial.begin(9600);
  // Configure the switches
  pinMode(pinSwitch1, INPUT_PULLUP);
  pinMode(pinSwitch2, INPUT_PULLUP);

  // Start screen
  // startScreen();

  // Configure the transceiver
  startRadio();

  // set up interrupt so that readings are done with 500Hz
#ifndef DEBUG__
  // noInterrupts();
  // TCCR2B = 0;
  // TCCR2B |= 1 << WGM12;
  // TCCR2B |= 1 << CS12;
  // TCNT2 = TIMERCOUNTERVALUE_500HZ;
  // TIMSK2 |= 1 << OCIE2A;

  // interrupts();
#endif
}

ISR(TIMER2_COMPA_vect)
{
  TCNT2 = TIMERCOUNTERVALUE_500HZ;

  payload[0] = (uint16_t)analogRead(pinThrottle);
  payload[1] = (uint16_t)analogRead(pinRoll) - rollOffset;
  payload[2] = (uint16_t)analogRead(pinPitch) - pitchOffset;
  payload[3] = (uint16_t)analogRead(pinYaw) - yawOffset;
  payload[4] = (uint16_t)(!digitalRead(pinSwitch2) & 0x01) << 1 | (!digitalRead(pinSwitch2) & 0x01); // inverted due to pullup resistor

  success = radio.write(&payload, PAYLOAD_SIZE);

  if (radio.isAckPayloadAvailable())
  {
    success = radio.read(&acknowledgePayload, 4);
    rxBatteryLevel = acknowledgePayload[1];
    packetReceived = 100;
  }
  else
  {
    packetReceived = 0;
  }

  signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), packetReceived);
  pos++;
}

void loop(void)
{
  // if (!(oldtxBatteryLevel == txBatteryLevel / 16.66))
  // {
  //   oldtxBatteryLevel = txBatteryLevel;
  //   oled.clear(80, 128, 0, 0);
  //   for (int i = 0; i < (txBatteryLevel / 16.66); i++)
  //     oled.print("|");
  // }

  // if (!(oldrxBatteryLevel == rxBatteryLevel / 16.66))
  // {
  //   oldrxBatteryLevel = rxBatteryLevel;
  //   oled.clear(80, 128, 1, 1);
  //   for (int i = 0; i < (rxBatteryLevel / 16.66); i++)
  //     oled.print("|");
  // }

  // if (!(oldsignalStrength == signalStrength / 16.66))
  // {
  //   oldsignalStrength = signalStrength;
  //   oled.clear(80, 128, 3, 3);
  //   for (int i = 0; i < (signalStrength / 16.66); i++)
  //     oled.print("|");
  // }

  // delay(100);

#ifdef DEBUG__
  payload[0] = (uint16_t)analogRead(pinThrottle);
  payload[1] = (uint16_t)analogRead(pinRoll) - rollOffset;
  payload[2] = (uint16_t)analogRead(pinPitch) - pitchOffset;
  payload[3] = (uint16_t)analogRead(pinYaw) - yawOffset;
  payload[4] = (uint16_t)(!digitalRead(pinSwitch2) & 0x01) << 1 | (!digitalRead(pinSwitch2) & 0x01); // inverted due to pullup resistor

  success = radio.write(&payload, PAYLOAD_SIZE);
  if (radio.isAckPayloadAvailable())
  {
    success = radio.read(&acknowledgePayload, 4);
    packetReceived = 100;
    // digitalWrite(pinSwitch1,LOW);
  }
  else
  {
    packetReceived = 0;
  }
  signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), packetReceived);
  pos++;

  Serial.println(signalStrength);
  // Serial.print("\t");
  // Serial.print(analogRead(pinThrottle));
  // Serial.print("\t");
  // // Serial.print((int16_t)(analogRead(pinRoll) - rollOffset));
  // // Serial.print("\t");
  // // Serial.print((int16_t)(analogRead(pinYaw) - yawOffset));
  // // Serial.print("\t");
  // Serial.println((int16_t)(analogRead(pinPitch) - pitchOffset)); //acknowledgePayload[1] <<8 | acknowledgePayload[0]);

#endif
}