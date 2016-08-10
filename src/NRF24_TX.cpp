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

#include <stdbool.h>
#include <stdint.h>

#include <arduino.h>

#include "NRF24_TX.h"

const char *NRF24_TX::protocolString[NRF24_TX::PROTOCOL_COUNT] = {
    "iNav",
    "V202 250Kbps",
    "V202 1Mbps",
    "Syma X",
    "Syma X5C",
    "CX10",
    "CX10A",
    "H8_3D",
};

NRF24_TX::~NRF24_TX() {}

NRF24_TX::NRF24_TX(NRF24L01* _nrf24)
    : nrf24(_nrf24) {}

NRF24_TX::NRF24_TX(uint8_t ce_pin, uint8_t csn_pin)
{
    static NRF24L01 nrf24L01(ce_pin, csn_pin);
    nrf24 = &nrf24L01;
}

void NRF24_TX::hopToNextChannel(void)
{
    ++rfChannelIndex;
    if (rfChannelIndex >= rfChannelCount) {
        rfChannelIndex = 0;
    }
    nrf24->setChannel(rfChannels[rfChannelIndex]);
}

