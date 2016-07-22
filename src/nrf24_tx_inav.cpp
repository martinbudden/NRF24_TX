/*
 * This file is part of the Arduino NRF24_RX library.
 *
 * Written by Martin Budden
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License, <http://www.gnu.org/licenses/>, for
 * more details.
 *
 * All the above text and this condition must be included in any redistribution.
 */

// This file borrows heavily from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arduino.h>

#include <NRF24.h>
#include "NRF24_TX.h"
#include "nrf24_tx_inav.h"

const uint8_t INAV_TX::txAddrBind[INAV_TX::TX_ADDR_LEN] = {0x4b,0x5c,0x6d,0x7e,0x8f};
uint8_t INAV_TX::txAddr[INAV_TX::TX_ADDR_LEN];

INAV_TX::~INAV_TX() {}

INAV_TX::INAV_TX(NRF24L01 *_nrf24)
    : NRF24_TX(_nrf24)
{
    rfChannels = rfChannelArray;
    rcChannels = rcChannelArray;
}

INAV_TX::INAV_TX(uint8_t _ce_pin, uint8_t _csn_pin)
    : NRF24_TX(_ce_pin, _csn_pin)
{
    rfChannels = rfChannelArray;
    rcChannels = rcChannelArray;
}

void INAV_TX::begin(int _protocol, const uint8_t *_txAddr)
{
    protocol = protocol;
    if (_txAddr) {
        memcpy(txAddr, txAddr, sizeof(txAddr));
    }

    protocolState = STATE_BIND;
    transmitPeriodUs = TRANSMIT_PERIOD_US;
    rfChannelCount = RF_CHANNEL_COUNT;
    payloadSize = PAYLOAD_SIZE;

     // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC, 250Kbps RF data rate
    NRF24_TX::initialize(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO), NRF24L01_06_RF_SETUP_RF_DR_250Kbps);
    nrf24->setRfPower(NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);

    nrf24->setChannel(RF_BIND_CHANNEL);
    nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddrBind, TX_ADDR_LEN);

    nrf24->writeReg(NRF24L01_11_RX_PW_P0, payloadSize);

    nrf24->setTxMode(); // enter transmit mode
}

// The hopping channels are determined by the txId
void INAV_TX::setHoppingChannels(void)
{
    uint32_t addr = txAddr[0];
    addr = addr & 0x1f;
    const uint32_t inc = (addr << 24) | (addr << 16) | (addr << 8) | addr;
    uint32_t * const prfChannels = (uint32_t *)rfChannels;
    *prfChannels = 0x10314259 + inc;
}

void INAV_TX::setRcChannels(uint16_t *_rcChannels)
{
    memcpy(rcChannelArray, _rcChannels, sizeof(rcChannelArray));
}

void INAV_TX::buildBindPacket(void)
{
    memset(payload, 0, PAYLOAD_SIZE);
    payload[0] = 0xae; // 10101110
    payload[1] = 0xc9; // 11001001
    payload[2] = txAddr[0];
    payload[3] = txAddr[1];
    payload[4] = txAddr[2];
    payload[5] = txAddr[3];
    payload[6] = txAddr[4];
}

void INAV_TX::buildDataPacket(void)
{
    payload[0] = 0;
    payload[1] = 0;
    // AETR channels have 10 bit resolution
    payload[2] = rcChannels[RC_CHANNEL_AILERON] >> 2;
    payload[3] = rcChannels[RC_CHANNEL_ELEVATOR] >> 2;
    payload[4] = rcChannels[RC_CHANNEL_THROTTLE] >> 2;
    payload[5] = rcChannels[RC_CHANNEL_RUDDER] >> 2;
    // pack the AETR low bits
    payload[6] = (rcChannels[RC_CHANNEL_AILERON] & 0x03) 
        | ((rcChannels[RC_CHANNEL_ELEVATOR] & 0x03) << 2) 
        | ((rcChannels[RC_CHANNEL_THROTTLE] & 0x03) << 4) 
        | ((rcChannels[RC_CHANNEL_RUDDER] & 0x03) << 6);
    uint8_t rate = RATE_LOW;
    if (rcChannels[RC_CHANNEL_RATE]) {
        rate = rcChannels[RC_CHANNEL_RATE] == 1000 ? RATE_HIGH : RATE_MID;
    }
    payload[7] = rate;
    uint8_t flags = 0;
    if (rcChannels[RC_CHANNEL_FLIP]) {
        flags |= FLAG_FLIP;
    }
    if (rcChannels[RC_CHANNEL_PICTURE]) {
        flags |= FLAG_PICTURE;
    }
    if (rcChannels[RC_CHANNEL_VIDEO]) {
        flags |= FLAG_VIDEO;
    }
    if (rcChannels[RC_CHANNEL_HEADLESS]) {
        flags |= FLAG_HEADLESS;
    }
    if (rcChannels[RC_CHANNEL_RTH]) {
        flags |= FLAG_RTH;
    }
    payload[8] = flags;
    // channels 11-14 have 10 bit resolution
    payload[9] = rcChannels[RC_CHANNEL11] >> 2;
    payload[10] = rcChannels[RC_CHANNEL12] >> 2;
    payload[11] = rcChannels[RC_CHANNEL13] >> 2;
    payload[12] = rcChannels[RC_CHANNEL14] >> 2;
    // pack the AETR low bits
    payload[13] = (rcChannels[RC_CHANNEL11] & 0x03) 
        | ((rcChannels[RC_CHANNEL12] & 0x03) << 2) 
        | ((rcChannels[RC_CHANNEL13] & 0x03) << 4) 
        | ((rcChannels[RC_CHANNEL14] & 0x03) << 6);

    // channels 15 and 16 have 8 bit resolution
    payload[14] = rcChannels[RC_CHANNEL15] >> 2;
    payload[15] = rcChannels[RC_CHANNEL16] >> 2;
}

void INAV_TX::setBound(void)
{
    protocolState = STATE_DATA;
    setHoppingChannels();
    rfChannelIndex = 0; // so first hop sets channel to channel zero
    nrf24->setChannel(rfChannels[0]);
    nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddr, TX_ADDR_LEN);
}

void INAV_TX::transmitPacket(void)
{
    ++packetCount;
    if (protocolState == STATE_BIND) {
        if (packetCount > BIND_PACKETS_TO_SEND) {
            setBound();
            buildDataPacket();
        } else {
            // no channel hopping in bind phase
            buildBindPacket();
        }
    } else {
        hopToNextChannel();
        buildDataPacket();
    }
    nrf24->writePayload(payload, payloadSize);
    nrf24->setTxMode();// enter transmit mode to send the packet
}

