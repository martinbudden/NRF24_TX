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
#include "nrf24_tx_syma.h"

#define FLAG_PICTURE    0x40
#define FLAG_VIDEO      0x80
#define FLAG_FLIP       0x40
#define FLAG_HEADLESS   0x80

#define FLAG_FLIP_X5C   0x01
#define FLAG_PICTURE_X5C 0x08
#define FLAG_VIDEO_X5C  0x10
#define FLAG_RATE_X5C   0x04

uint8_t SYMA_TX::txAddr[SYMA_TX::TX_ADDR_LEN]; // change to this address in data phase
const uint8_t SYMA_TX::txAddrBind[SYMA_TX::TX_ADDR_LEN] = {0xab,0xac,0xad,0xae,0xaf};
const uint8_t SYMA_TX::txAddrX5C[SYMA_TX::TX_ADDR_LEN] = {0x6d,0x6a,0x73,0x73,0x73};   // X5C uses same address for bind and data

// set rfChannels to SymaX bind channels
uint8_t SYMA_TX::rfChannelArray[SYMA_X_RF_CHANNEL_COUNT]  = {0x4b, 0x30, 0x40, 0x20};
const uint8_t SYMA_TX::rfChannelArrayX5C[SYMA_X5C_RF_CHANNEL_COUNT] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24, 0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};

SYMA_TX::~SYMA_TX() {}

SYMA_TX::SYMA_TX(NRF24L01 *_nrf24)
    : NRF24_TX(_nrf24)
{
    rcChannels = rcChannelArray;
}

SYMA_TX::SYMA_TX(uint8_t _ce_pin, uint8_t _csn_pin)
    : NRF24_TX(_ce_pin, _csn_pin)
{
    rcChannels = rcChannelArray;
}

void SYMA_TX::begin(int _protocol, const uint8_t *_txAddr)
{
    protocol = _protocol;
    memcpy(txAddr, _txAddr ? _txAddr : txAddrBind, sizeof(txAddr));

    transmitPeriodUs = TRANSMIT_PERIOD_US;
    if (protocol == SYMA_X) {
         // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC, 250Kbps RF data rate
        nrf24->initializeNoAutoAcknowledgement(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO), NRF24L01_06_RF_SETUP_RF_DR_250Kbps);
        payloadSize =PAYLOAD_SIZE_SYMA_X;
        nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddrBind, TX_ADDR_LEN);
        protocolState = STATE_BIND;
        rfChannelCount = SYMA_X_RF_CHANNEL_COUNT;
        rfChannels = rfChannelArray;
    } else {
         // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC, 1Mbps RF data rate
        nrf24->initializeNoAutoAcknowledgement(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO), NRF24L01_06_RF_SETUP_RF_DR_1Mbps);
        payloadSize = PAYLOAD_SIZE_SYMA_X5C;
        nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddrX5C, TX_ADDR_LEN);
        // just go straight into data mode, since the SYMA_X5C protocol does not actually require binding
        protocolState = STATE_DATA;
        rfChannelCount = SYMA_X5C_RF_CHANNEL_COUNT;
        rfChannels = const_cast<uint8_t*>(rfChannelArrayX5C);
    }
    nrf24->setRfPower(NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    rfChannelIndex = rfChannelCount - 1; // so first hop sets channel to channel zero
    nrf24->setChannel(rfChannels[rfChannelIndex]);
    nrf24->writeReg(NRF24L01_11_RX_PW_P0, payloadSize);
    nrf24->setTxMode(); // enter transmit mode
}

void SYMA_TX::hopToNextChannel(void)
{
    // hop channel every second packet
    if ((packetCount & 0x01) == 0) {
        NRF24_TX::hopToNextChannel();
    }
}

// The SymaX hopping channels are determined by the low bits of txAddress
void SYMA_TX::setHoppingChannels()
{
    uint32_t addr = txAddr[0];
    addr = addr & 0x1f;
    if (addr == 0x06) {
        addr = 0x07;
    }
    const uint32_t inc = (addr << 24) | (addr << 16) | (addr << 8) | addr;
    uint32_t * const prfChannels = (uint32_t *)rfChannels;
    if (addr == 0x16) {
        *prfChannels = 0x28481131;
    } else if (addr == 0x1e) {
        *prfChannels = 0x38184121;
    } else if (addr < 0x10) {
        *prfChannels = 0x3A2A1A0A + inc;
    } else if (addr < 0x18) {
        *prfChannels = 0x1231FA1A + inc;
    } else {
        *prfChannels = 0x19FA2202 + inc;
    }
}

void SYMA_TX::setRcChannels(uint16_t *_rcChannels)
{
    memcpy(rcChannelArray, _rcChannels, sizeof(rcChannelArray));
}

uint8_t SYMA_TX::calcChecksum(void)
{
    uint8_t ret = payload[0];

    if (protocol == SYMA_X) {
        for (int ii = 1; ii < payloadSize - 1; ++ii) {
            ret ^= payload[ii];
        }
    } else {
        for (int ii = 1; ii < payloadSize - 1; ++ii) {
            ret += payload[ii];
        }
        ret += 0x55;
    }
    return ret;
}

void SYMA_TX::buildBindPacket(void)
{
    memset(payload, 0, payloadSize);
    if (protocol == SYMA_X) {
        payload[0] = txAddr[4];
        payload[1] = txAddr[3];
        payload[2] = txAddr[2];
        payload[3] = txAddr[1];
        payload[4] = txAddr[0];
        payload[5] = 0xaa;
        payload[6] = 0xaa;
        payload[7] = 0xaa;
        payload[9] = calcChecksum();
    } else {
        payload[7] = 0xae;
        payload[8] = 0xa9;
        payload[14] = 0xc0;
        payload[15] = 0x17; // checksum
    }
}

uint8_t SYMA_TX::convertFromPwmUnsigned(uint32_t pwm)
{
    pwm += PWM_RANGE_MIN;
    return (uint8_t)(((pwm - PWM_RANGE_MIN) * 255) / PWM_RANGE);
}

// Channel values are sign + magnitude 8bit values
uint8_t SYMA_TX::convertFromPwmSigned(uint32_t pwm)
{
    int32_t ret;

    if (pwm > 1000) {
        pwm = 1000;
    }
    pwm += PWM_RANGE_MIN;
    if (pwm < PWM_RANGE_MIDDLE) {
        ret = 0x80 | (((PWM_RANGE_MIDDLE - pwm) * 127) / (PWM_RANGE / 2));
    } else {
        ret = ((pwm - PWM_RANGE_MIDDLE) * 127) / (PWM_RANGE / 2);
    }
    return (uint8_t)ret;
}

void SYMA_TX::buildDataPacket(void)
{
    memset(payload, 0, payloadSize);
    payload[0] = convertFromPwmUnsigned(rcChannels[RC_CHANNEL_THROTTLE]);
    payload[3] = convertFromPwmSigned(rcChannels[RC_CHANNEL_AILERON]);
    if (protocol == SYMA_X) {
        payload[1] = convertFromPwmSigned(rcChannels[RC_CHANNEL_ELEVATOR]);
        payload[2] = convertFromPwmSigned(rcChannels[RC_CHANNEL_RUDDER]);
        if (rcChannels[RC_CHANNEL_FLIP]) {
            payload[6] |= FLAG_FLIP;
        }
        if (rcChannels[RC_CHANNEL_PICTURE]) {
            payload[4] |= FLAG_PICTURE;
        }
        if (rcChannels[RC_CHANNEL_VIDEO]) {
            payload[4] |= FLAG_VIDEO;
        }
        if (rcChannels[RC_CHANNEL_HEADLESS]) {
            payload[14] |= FLAG_HEADLESS;
        }
        payload[9] = calcChecksum();
    } else {
        payload[1] = convertFromPwmSigned(rcChannels[RC_CHANNEL_ELEVATOR]) ^ 0x80;
        payload[2] = convertFromPwmSigned(rcChannels[RC_CHANNEL_RUDDER]);
        uint8_t flags = 0;
        if (rcChannels[RC_CHANNEL_FLIP]) {
            flags |= FLAG_FLIP_X5C;
        }
        if (rcChannels[RC_CHANNEL_PICTURE]) {
            flags |= FLAG_PICTURE_X5C;
        }
        if (rcChannels[RC_CHANNEL_VIDEO]) {
            flags |= FLAG_VIDEO_X5C;
        }
        payload[14] = flags;
        payload[15] = calcChecksum();
    }
}

void SYMA_TX::setBound(void)
{
    protocolState = STATE_DATA;
    if (protocol == SYMA_X) {
        nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddr, TX_ADDR_LEN);
        setHoppingChannels();
        rfChannelIndex = 0;
        nrf24->setChannel(rfChannels[0]);
    }
}

void SYMA_TX::transmitPacket(void)
{
    ++packetCount;
    if (protocolState == STATE_BIND) {
        if (packetCount > BIND_PACKETS_TO_SEND) {
            setBound();
            buildDataPacket();
        } else {
            hopToNextChannel();
            buildBindPacket();
        }
    } else {
        hopToNextChannel();
        buildDataPacket();
    }
    nrf24->writePayload(payload, payloadSize);
    nrf24->setTxMode();// enter transmit mode to send the packet
}

