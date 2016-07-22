/*
 * This file is part of the Arduino NRF24_TX library.
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
#include <NRF24.h>

#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE 1500
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)

class NRF24_TX {
public:
    typedef enum {
        V202_250K = 0,
        V202_1M,
        SYMA_X,
        SYMA_X5C,
        CX10,
        CX10A,
        H8_3D_H20,
        INAV,
        PROTOCOL_COUNT
    } protocol_e;
    typedef enum {
        RC_CHANNEL1 = 0,   // Aileron
        RC_CHANNEL2,       // Elevator
        RC_CHANNEL3,       // Throttle
        RC_CHANNEL4,       // Rudder
        RC_CHANNEL5,       // Rate/Mode
        RC_CHANNEL6,       // Flip
        RC_CHANNEL7,       // Still Camera
        RC_CHANNEL8,       // Video Camera
        RC_CHANNEL9,       // Headless
        RC_CHANNEL10,      // RTH
        RC_CHANNEL11,      // X Calibration (Q282)
        RC_CHANNEL12,      // Y Calibration (Q282)
        RC_CHANNEL13,
        RC_CHANNEL14,
        RC_CHANNEL15,
        RC_CHANNEL16,
        RC_CHANNEL_COUNT,
        // AETR RC channels
        RC_CHANNEL_AILERON = RC_CHANNEL1,
        RC_CHANNEL_ELEVATOR = RC_CHANNEL2,
        RC_CHANNEL_THROTTLE = RC_CHANNEL3,
        RC_CHANNEL_RUDDER = RC_CHANNEL4,
        // RC channels as used by deviation
        RC_CHANNEL_RATE = RC_CHANNEL5,
        RC_CHANNEL_FLIP = RC_CHANNEL6,
        RC_CHANNEL_PICTURE = RC_CHANNEL7,
        RC_CHANNEL_VIDEO = RC_CHANNEL8,
        RC_CHANNEL_HEADLESS = RC_CHANNEL9,
        RC_CHANNEL_RTH = RC_CHANNEL10,
    } rc_channel_e;
protected:
    NRF24L01 *nrf24;
    uint8_t protocol;
    uint8_t protocolState;
    uint8_t rfChannelCount;
    uint8_t rfChannelIndex;
    uint8_t *rfChannels;
    uint16_t *rcChannels;
    uint32_t transmitPeriodUs;
    uint16_t packetCount;
    uint8_t payload[NRF24L01_MAX_PAYLOAD_SIZE];
    uint16_t payloadCrc;
    uint8_t payloadSize;
protected:
    void initialize(uint8_t baseConfig, uint8_t rfDataRate);
    virtual void hopToNextChannel(void);
    virtual void setHoppingChannels(void) = 0;
    virtual void buildBindPacket(void) = 0;
    virtual void buildDataPacket(void) = 0;
public:
    virtual void begin(int protocol, const uint8_t *txAddr) = 0;
    virtual void setRcChannels(uint16_t *rcChannels) = 0;
    virtual void transmitPacket(void) = 0;
public:
    virtual ~NRF24_TX();
    NRF24_TX(NRF24L01 *_nrf24);
    NRF24_TX(uint8_t ce_pin, uint8_t csn_pin);
    uint32_t transmitPeriodMs(void) const {return transmitPeriodUs / 1000;}
    // debugging and instrumentation functions
    const uint8_t *payloadPtr(void) const {return payload;}
    int getPayloadSize(void) const {return payloadSize;}
    uint16_t getPayloadCrc(void) const {return payloadCrc;}
    int getChannel(void) const {return nrf24->getChannel();}
    uint8_t getProtocol(void)const {return protocol;}
};

