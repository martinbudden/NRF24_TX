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

class SYMA_TX : public NRF24_TX {
public:
    enum {RC_CHANNEL_COUNT = 16};
private:
    uint16_t rcChannelArray[RC_CHANNEL_COUNT];
    // radio channels for frequency hopping
    enum {RF_BIND_CHANNEL = 0x4c};
    enum {SYMA_X_RF_CHANNEL_COUNT = 4, SYMA_X5C_RF_CHANNEL_COUNT = 15};
    static uint8_t rfChannelArray[SYMA_X_RF_CHANNEL_COUNT];
    static const uint8_t rfChannelArrayX5C[SYMA_X5C_RF_CHANNEL_COUNT];
    enum {STATE_BIND = 0, STATE_DATA = 1};

    enum {TX_ADDR_LEN = 5};
    static uint8_t txAddr[TX_ADDR_LEN]; // transmitter address, sent in bind packet
    static const uint8_t txAddrBind[TX_ADDR_LEN];
    static const uint8_t txAddrX5C[TX_ADDR_LEN];

    enum {PAYLOAD_SIZE_SYMA_X = 10, PAYLOAD_SIZE_SYMA_X5C = 16};
    enum {TRANSMIT_PERIOD_US = 4000}; // 4ms, 250 Hz
    enum {BIND_PACKETS_TO_SEND = 345}; // 5 seconds of bind packets
    uint16_t bindPacketCount;
    uint32_t packetCount;
private:
    uint8_t calcChecksum(void);
    void setBound(void);
public:
    uint8_t convertFromPwmUnsigned(uint32_t pwm);
    uint8_t convertFromPwmSigned(uint32_t pwm);
protected:
    virtual void hopToNextChannel(void);
    virtual void setHoppingChannels(void);
    virtual void buildDataPacket(void);
    virtual void buildBindPacket(void);
public:
    virtual void setRcChannels(uint16_t *rcChannels);
    virtual void transmitPacket(void);
public:
    virtual ~SYMA_TX();
    SYMA_TX(NRF24L01 *nrf24);
    SYMA_TX(uint8_t _ce_pin, uint8_t _csn_pin);
    virtual void begin(int protocol, const uint8_t *nrf24_id = 0);
};

