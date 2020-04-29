/*
 * CCDMonitor.ino (https://github.com/laszlodaniel/CCDLibrary)
 * Copyright (C) 2020, László Dániel
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Example: CCD-bus messages are displayed in the Arduino serial monitor.
 */

#include <CCDLibrary.h>

uint8_t lastMessage[16];
uint8_t lastMessageLength = 0;

void setup()
{
    Serial.begin(250000);
    CCD.begin(CLOCK_ON, IDLE_BITS_14);
    // CLOCK_ON: enables 1 MHz clock signal on D11/PB5 pin for the CDP68HC68S1 CCD-bus transceiver IC. The ChryslerCCDSCIScanner hardware requires a clock signal.
    // CLOCK_OFF: disables 1 MHz clock signal on D11/PB5 pin. The CCDBusTransceiver development board doesn't require a clock signal.
    // IDLE_BITS_XX: sets the number of consecutive bits sensed as CCD-bus idle condition.
    // IDLE_BITS_10: default idle bits according to the CDP68HC68S1 datasheet. It should be increased if messages are not coming through properly.
    // IDLE_BITS_15: maximum masked value, use plain integers above this value.
}

void loop()
{
    if (CCD.available())
    {
        lastMessageLength = CCD.read(lastMessage);
        
        for (uint8_t i = 0; i < lastMessageLength; i++)
        {
            if (lastMessage[i] < 16) Serial.print("0");
            Serial.print(lastMessage[i], HEX);
            Serial.print(" ");
        }
        
        Serial.println();
    }
}