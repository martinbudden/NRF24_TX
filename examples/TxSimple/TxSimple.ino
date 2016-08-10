/*
 * Simple NRF24L01 transmitter implementation.
 * 
 * Lines commented: // !! USER_SETUP
 * should be checked and changed if necessary for your hardware setup and preferences.
 *
 * This code is in the public domain and may be freely used.
 */

#include <NRF24_TX.h>
#include <nrf24_tx_inav.h>
#include <nrf24_tx_syma.h>


// NRF24L01 pins
static const int CE_PIN = 9;
static const int CSN_PIN = 10;

#include <printf.h>
#include <RF24.h>
RF24 radio(CE_PIN, CSN_PIN);

// !! USER_SETUP - set the pin values according to your hardware
// analog pins
static const int AILERON_PIN = 0;
static const int ELEVATOR_PIN = 0;
static const int THROTTLE_PIN = 0;
static const int RUDDER_PIN = 0;
// digital pins
static const int HEADLESS_PIN = 2;


uint8_t txAddr[INAV_TX::TX_ADDR_LEN];

// !! USER_SETUP - uncomment one of the USE_ #defines below to specify the protocol to use
#define USE_INAV
//#define USE_SYMA

#if defined(USE_INAV)
INAV_TX nrf24(CE_PIN, CSN_PIN);
INAV_TX *inavTx = &nrf24;
NRF24_TX *tx = &nrf24;
static const int protocol = NRF24_TX::INAV;
static const int rcChannelCount = INAV_TX::RC_CHANNEL_COUNT;
uint8_t * const ackPayload = (uint8_t*)inavTx->ackPayloadPtr();
#elif defined(USE_SYMA)
SYMA_TX nrf24(CE_PIN, CSN_PIN);
NRF24_TX *tx = &nrf24;
static const int protocol = NRF24_TX::SYMA_X;
static const int rcChannelCount = SYMA_TX::RC_CHANNEL_COUNT;
#endif

uint16_t rcChannels[rcChannelCount];

uint32_t createTrulyRandomSeed(void);

void setup(void)
{
//  radio.begin();

// !! USER_SETUP - choose random or hardcoded transmit address
#ifdef RANDOM_TX_ADDR
  const uint32_t seed = createTrulyRandomSeed();
  randomSeed(seed);
  for (int ii = 0; ii < INAV_TX::TX_ADDR_LEN; ++ii) {
    txAddr[ii] = random(0,255);
  }
#else
// for INAV
// 0x718293a4 = 1904382884
// on receiver, set nrf24rx_id = 1904382884
  txAddr[0] = 0xa4; //164
  txAddr[1] = 0x93; //147
  txAddr[2] = 0x82; //130
  txAddr[3] = 0x71; //113
  txAddr[4] = 0xD2;
  tx->begin(protocol, txAddr);
// for H8_3D, AUX7=4,121,123,0; 0x04797B00->8091908
// set nrf24rx_id = 8091908
#endif

  Serial.begin(57600);
  Serial.println("");
  Serial.print("Transmitter Starting, protocol: ");
  Serial.println(tx->getProtocolString());
  Serial.println("");
  delay(100);
  printf_begin();
  Serial.println("");
//  radio.printDetails();
  Serial.println("");
}

uint16_t *readRcChannels(void)
{
  memset(rcChannels, 0, sizeof(rcChannels));
  // convert analog pin value to range [0, 1000]
  // !! USER_SETUP - set these up as required, depending on max and min values returned on your hardware
  //rcChannels[NRF24_TX::RC_CHANNEL_AILERON] = map(analogRead(AILERON_PIN), 0, 732, 0, 1000);
  //rcChannels[NRF24_TX::RC_CHANNEL_ELEVATOR] = map(analogRead(ELEVATOR_PIN),  1, 1020, 0, 1000);
  //rcChannels[NRF24_TX::RC_CHANNEL_THROTTLE] = map(analogRead(THROTTLE_PIN), 12, 1021, 0, 1000);
  //rcChannels[NRF24_TX::RC_CHANNEL_RUDDER] = map(analogRead(RUDDER_PIN), 34, 1020, 0, 1000);

  rcChannels[NRF24_TX::RC_CHANNEL_AILERON] = analogRead(AILERON_PIN);
  rcChannels[NRF24_TX::RC_CHANNEL_ELEVATOR] = analogRead(ELEVATOR_PIN);
  rcChannels[NRF24_TX::RC_CHANNEL_THROTTLE] = analogRead(THROTTLE_PIN);
  rcChannels[NRF24_TX::RC_CHANNEL_RUDDER] = analogRead(RUDDER_PIN);

  // convert digital pin value to 0 or 1000
  //rcChannels[NRF24_TX::RC_CHANNEL_HEADLESS] = digitalRead(HEADLESS_PIN) ? 1000 : 0;
  // !! USER_SETUP - add in additional channels, if required

#if defined(USE_INAV)
  // for debugging - set up channels so that ackPayload can be seen by receiver
  rcChannels[NRF24_TX::RC_CHANNEL11] = ackPayload[0];
  rcChannels[NRF24_TX::RC_CHANNEL12] = ackPayload[1];
  rcChannels[NRF24_TX::RC_CHANNEL13] = ackPayload[2];
  rcChannels[NRF24_TX::RC_CHANNEL14] = ackPayload[3];
  rcChannels[NRF24_TX::RC_CHANNEL15] = ackPayload[4];
  rcChannels[NRF24_TX::RC_CHANNEL16] = ackPayload[5];
#endif
  return rcChannels;
}

void printRcData(void)
{
  Serial.print("RFCH=");
  Serial.print(tx->getChannel());
  Serial.print("  ch1=");
  Serial.print(rcChannels[0]+1000);
//  Serial.print("  ch2=");
//  Serial.println(rcChannels[1]+1000);
#if defined(USE_SYMA)
  Serial.print("  ps=");
  Serial.print(nrf24.convertFromPwmSigned(rcChannels[0]));
  Serial.print("  pu=");
  Serial.print(nrf24.convertFromPwmUnsigned(rcChannels[0]));
  Serial.println("");
#endif
}

void printTelemetryData(const uint8_t *data, int len)
{
  if (len > 0) {
    Serial.print(" seq=");
    Serial.print(data[0]);
    Serial.print(" frm=");
    Serial.print(data[1]);
    Serial.print(" len=");
    Serial.print(len);
    switch (data[1]) {
    case 'A':
      // attitude frame
      Serial.print(" attitude=");
      Serial.print(data[2]);
      Serial.print(",");
      Serial.print(data[4]);
      Serial.print(",");
      Serial.print(data[6]);
      break;
    case 'S':
      // state frame
      Serial.print(" state=");
      Serial.print(data[7] & 0x03);
      Serial.print(",");
      Serial.print(data[7] >> 2);
      break;
    default:
      break;
    }
  } else {
    Serial.print("  no telem");
  }
}

void loop(void)
{
  static uint32_t lastTransmitTimeMs = 0;

  tx->setRcChannels(readRcChannels());
  printRcData();
  if (millis() >= lastTransmitTimeMs + tx->transmitPeriodMs()) {
    lastTransmitTimeMs = millis();
#if defined(USE_INAV)
    // set up dummy ack payload which should be overwritten by ack received in transmitPacket()
    ackPayload[0] = 19;
    ackPayload[1] = 'A';
    ackPayload[2] = 100;
    ackPayload[3] = 0;
    ackPayload[4] = 150;
    ackPayload[5] = 99;
    ackPayload[6] = 200;
    ackPayload[7] = 0;
    const int ackPayloadSize = inavTx->transmitPacketAndWaitForAck();
    printTelemetryData(ackPayload, ackPayloadSize);
    Serial.print(" lost=");
    Serial.print(inavTx->getLostPacketCount());
    Serial.print(" retr=");
    Serial.print(inavTx->getAutoRetryCount());
    Serial.println("");
#else
    tx->transmitPacket();
#endif
  }
}
