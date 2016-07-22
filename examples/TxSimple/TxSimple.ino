#include <NRF24_TX.h>
#include <nrf24_tx_inav.h>
#include <nrf24_tx_syma.h>


// NRF24L01 pins
static const int CE_PIN = 9;
static const int CSN_PIN = 10;

#include <printf.h>
#include <RF24.h>
RF24 radio(CE_PIN, CSN_PIN);

// analog pins
static const int AILERON_PIN = 0;
static const int ELEVATOR_PIN = 1;
static const int THROTTLE_PIN = 2;
static const int RUDDER_PIN = 3;
// digital pins
static const int HEADLESS_PIN = 2;


uint8_t txAddr[INAV_TX::TX_ADDR_LEN];
// uncomment one of the USE_ #defines below to specify the protocol to use
#define USE_INAV
//#define USE_SYMA

#if defined(USE_INAV)
INAV_TX nrf24(CE_PIN, CSN_PIN);
NRF24_TX *tx = &nrf24;
static const int protocol = NRF24_TX::INAV;
static const int rcChannelCount = INAV_TX::RC_CHANNEL_COUNT;
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
  const uint32_t seed = createTrulyRandomSeed();
  randomSeed(seed);
  for (int ii = 0; ii < INAV_TX::TX_ADDR_LEN; ++ii) {
    txAddr[ii] = random(0,255);
  }

  tx->begin(protocol, txAddr);

  Serial.begin(57600);
  Serial.println("");
  Serial.print("Transmitter Starting, protocol: ");
  Serial.println(tx->getProtocol());
  Serial.println("");
  delay(100);
  printf_begin();
  radio.begin();
  Serial.println("");
  radio.printDetails();
  Serial.println("");
}

uint16_t *readRcChannels(void)
{
  memset(rcChannels, 0, sizeof(rcChannels));
  // convert analog pin value to range [0, 1000]
//  rcChannels[NRF24_TX::RC_CHANNEL_AILERON] = map(analogRead(AILERON_PIN), 0, 732, 0, 1000);
  rcChannels[NRF24_TX::RC_CHANNEL_AILERON] = analogRead(AILERON_PIN);
  //rcChannels[NRF24_TX::RC_CHANNEL_ELEVATOR] = map(analogRead(ELEVATOR_PIN),  1, 1020, 0, 1000);
  //rcChannels[NRF24_TX::RC_CHANNEL_THROTTLE] = map(analogRead(THROTTLE_PIN), 12, 1021, 0, 1000);
  //rcChannels[NRF24_TX::RC_CHANNEL_RUDDER] = map(analogRead(RUDDER_PIN), 34, 1020, 0, 1000);

  // convert digital pin value to 0 or 1000
  //rcChannels[NRF24_TX::RC_CHANNEL_HEADLESS] = digitalRead(HEADLESS_PIN) ? 1000 : 0;*/
  return rcChannels;
}

void printRcData(void)
{
  Serial.print("ch1=");
  Serial.print(rcChannels[0]+1000);
//  Serial.print("  ch2=");
//  Serial.println(rcChannels[1]+1000);
#if defined(USE_SYMA)
  Serial.print("  ps=");
  Serial.print(nrf24.convertFromPwmSigned(rcChannels[0]));
  Serial.print("  pu=");
  Serial.println(nrf24.convertFromPwmUnsigned(rcChannels[0]));
#else
  Serial.println("");
#endif
}

void loop(void)
{
  static uint32_t lastTransmitTimeMs = 0;

  tx->setRcChannels(readRcChannels());
  printRcData();
  if (millis() >= lastTransmitTimeMs + tx->transmitPeriodMs()) {
    lastTransmitTimeMs = millis();
    tx->transmitPacket();
  }
}
