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

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "NRF24_TX.h"

class INAV_TX : public NRF24_TX {
public:
    enum {RC_CHANNEL_COUNT = 16};
    enum {TX_ADDR_LEN = 5};
private:
    uint8_t ackPayload[NRF24L01_MAX_PAYLOAD_SIZE];
    uint8_t ackPayloadSize;
    uint8_t lostPacketCount;
    uint8_t autoRetryCount;

    uint16_t rcChannelArray[RC_CHANNEL_COUNT];
    // radio channels for frequency hopping
    enum {RF_BIND_CHANNEL = 0x4c}; // channel 76
    enum {RF_CHANNEL_COUNT_MAX = 8};
    uint8_t rfChannelArray[RF_CHANNEL_COUNT_MAX];
    enum {RF_CHANNEL_HOPPING_COUNT_DEFAULT = 4};
    uint8_t rfChannelHoppingCount;
    enum {STATE_BIND = 0, STATE_ACK = 1, STATE_DATA = 2};

    static const uint8_t txAddrBind[TX_ADDR_LEN];
    static uint8_t txAddr[TX_ADDR_LEN];

    enum {PAYLOAD_SIZE = 16};
    enum {TRANSMIT_PERIOD_US = 4000}; // 4ms, 250 Hz
    enum {BIND_PACKETS_TO_SEND = 375}; // 1.5 seconds of bind packets
    enum {RATE_LOW = 0, RATE_MID = 1, RATE_HIGH = 2};
    enum {
        FLAG_FLIP     = 0x01,
        FLAG_PICTURE  = 0x02,
        FLAG_VIDEO    = 0x04,
        FLAG_RTH      = 0x08,
        FLAG_HEADLESS = 0x10,
    };
private:
    void setBound(void);
protected:
    virtual void setHoppingChannels(void);
    virtual void buildDataPacket(void);
    virtual void buildBindPacket(void);
public:
    virtual void setRcChannels(uint16_t *rcChannels);
    virtual void transmitPacket(void);
    int transmitPacketAndWaitForAck(void);
public:
    virtual ~INAV_TX();
    INAV_TX(NRF24L01 *nrf24);
    INAV_TX(uint8_t _ce_pin, uint8_t _csn_pin);
    virtual void begin(uint8_t protocol, const uint8_t *nrf24_id = 0);

    const uint8_t *ackPayloadPtr(void) const {return ackPayload;}
    int getAckPayloadSize(void) const {return ackPayloadSize;}
    uint8_t getLostPacketCount(void) const {return lostPacketCount;}
    uint8_t getAutoRetryCount(void) const {return autoRetryCount;}
    void setRfChannelHoppingCount(uint8_t _rfChannelHoppingCount) {rfChannelHoppingCount = _rfChannelHoppingCount;}
};

