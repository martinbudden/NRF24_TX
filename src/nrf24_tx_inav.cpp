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

void INAV_TX::begin(uint8_t _protocol, const uint8_t *_txAddr)
{
    protocol = _protocol;
    memcpy(txAddr, _txAddr ? _txAddr : txAddrBind, sizeof(txAddr));

    protocolState = STATE_BIND;
    packetCount = 0;
    transmitPeriodUs = TRANSMIT_PERIOD_US;
    rfChannelCount = 1; // for bind state
    rfChannels[0] = RF_BIND_CHANNEL;
    rfChannelHoppingCount = RF_CHANNEL_HOPPING_COUNT_DEFAULT;
    payloadSize = PAYLOAD_SIZE;

     // sets PWR_UP, EN_CRC, CRCO - 2 byte CRC, 250Kbps RF data rate
    nrf24->initialize(BV(NRF24L01_00_CONFIG_EN_CRC) | BV( NRF24L01_00_CONFIG_CRCO), NRF24L01_06_RF_SETUP_RF_DR_250Kbps);

    nrf24->writeReg(NRF24L01_01_EN_AA, BV(NRF24L01_01_EN_AA_ENAA_P0) | BV(NRF24L01_01_EN_AA_ENAA_P0)); // auto acknowledgment on P0, P1
    nrf24->writeReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0) | BV(NRF24L01_02_EN_RXADDR_ERX_P1));
    nrf24->writeReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    nrf24->writeReg(NRF24L01_04_SETUP_RETR, NRF24L01_04_SETUP_RETR_ARD_2500us | NRF24L01_04_SETUP_RETR_ARC_1); // one retry after 2500us
    nrf24->setRfPower(NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);

    nrf24->setChannel(RF_BIND_CHANNEL);
    nrf24->writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, txAddrBind, TX_ADDR_LEN); // RX_ADDR_P0 must equal TX_ADDR for auto ACK
    nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddrBind, TX_ADDR_LEN);
    nrf24->writeReg(NRF24L01_11_RX_PW_P0, payloadSize);

    nrf24->writeReg(NRF24L01_1C_DYNPD, BV(NRF24L01_1C_DYNPD_P0) | BV(NRF24L01_1C_DYNPD_P1)); // dynamic payload length on pipes P0 & P1
    nrf24->writeReg(NRF24L01_1D_FEATURE, BV(NRF24L01_1D_FEATURE_EN_DPL) | BV(NRF24L01_1D_FEATURE_EN_ACK_PAY));

    nrf24->setTxMode(); // enter transmit mode
}

// The hopping channels are determined by the txId
void INAV_TX::setHoppingChannels(void)
{
    if (rfChannelHoppingCount == 0) {
         // just stay on bind channel, useful for debugging
        rfChannelCount = 1;
        rfChannels[0] = RF_BIND_CHANNEL;
        return;
    }
    rfChannelCount = rfChannelHoppingCount;
    uint8_t ch = 0x10 + (txAddr[0] & 0x07);
    for (int ii = 0; ii < RF_CHANNEL_COUNT_MAX; ++ii) {
        rfChannels[ii] = ch;
        ch += 0x0c;
    }
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
    payload[7] = rfChannelHoppingCount;
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
    nrf24->writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, txAddr, TX_ADDR_LEN); // RX_ADDR_P0 must equal TX_ADDR for auto ACK
    nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddr, TX_ADDR_LEN);
    setHoppingChannels();
    rfChannelIndex = 0;
    nrf24->setChannel(rfChannels[0]);
}

void INAV_TX::transmitPacket(void)
{
    ++packetCount;
    if (protocolState == STATE_BIND) {
        if (packetCount > BIND_PACKETS_TO_SEND) {
            setBound();
            buildDataPacket();
        } else {
            hopToNextChannel(); // resets PLOS_CNT, but does not change channel in bind phase
            buildBindPacket();
        }
    } else {
        hopToNextChannel(); // resets PLOS_CNT
        buildDataPacket();
    }
    nrf24->writePayload(payload, payloadSize);
    nrf24->setTxMode();// enter transmit mode to send the packet, resets ARC_CNT
}

int INAV_TX::transmitPacketAndWaitForAck(void)
{
    transmitPacket(); // asynchronous call
    ackPayloadSize = 0;
    while (true) {
        const uint8_t observeTx = nrf24->readReg(NRF24L01_08_OBSERVE_TX);
        lostPacketCount = (observeTx & NRF24L01_08_OBSERVE_TX_PLOS_CNT_MASK) >> 4;
        autoRetryCount = observeTx & NRF24L01_08_OBSERVE_TX_ARC_CNT_MASK;

        //const uint8_t status = nrf24->readReg(NRF24L01_07_STATUS);
        const uint8_t status = nrf24->readStatus();
        if (status & BV(NRF24L01_07_STATUS_MAX_RT)) {
            nrf24->clearAllInterrupts(); // clear MAX_RT interrupt to allow further transmission
            nrf24->flushTx(); // payload wasn't successfully transmitted, so remove it from TX FIFO
            break;
        }
        if (status & BV(NRF24L01_07_STATUS_TX_DS)) { // NRF24L01_07_STATUS_TX_DS asserted when ack payload received
            // ack payload recieved
            ackPayloadSize = nrf24->readDynamicPayloadIfAvailable(ackPayload);
            break;
        }
    }
    return ackPayloadSize;
}

