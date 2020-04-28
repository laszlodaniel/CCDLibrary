/*
 * CCDFilter.ino (https://github.com/laszlodaniel/CCDLibrary)
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
 * Example: CCD-bus messages are displayed in the Arduino serial monitor 
 * that start with either the byte B2 or F2 (diagnostic request/response message). 
 * Any number and kind of bytes can be filtered.
 */

#include <CCDLibrary.h>

//#define ChryslerCCDSCIScanner // 1 MHz clock signal is necessary for the CDP68HC68S1 CCD-bus transceiver IC to work

uint8_t lastMessage[16];
uint8_t lastMessageLength = 0;
uint8_t messageIDbyte = 0;

void setup()
{
    #if defined(ChryslerCCDSCIScanner)
        TCCR1A = 0;                        // clear register
        TCCR1B = 0;                        // clear register
        TCNT1  = 0;                        // clear counter
        DDRB   |= (1<<DDB5);               // set OC1A/PB5 as output
        TCCR1A |= (1<<COM1A0);             // toggle OC1A on compare match
        OCR1A  = 7;                        // top value for counter, toggle after counting to 8 (0->7) = 2 MHz interrupt ( = 16 MHz clock frequency / 8)
        TCCR1B |= (1<<WGM12) | (1<<CS10);  // CTC mode, prescaler clock/1 (no prescaler)
    #endif
    
    Serial.begin(250000);
    CCD.begin();
}

void loop()
{
    if (CCD.available())
    {
        lastMessageLength = CCD.read(lastMessage);
        messageIDbyte = lastMessage[0];
        
        // Diagnostic request/response message filter.
        if ((messageIDbyte == 0xB2) || (messageIDbyte == 0xF2))
        {
            for (uint8_t i = 0; i < lastMessageLength; i++)
            {
                if (lastMessage[i] < 16) Serial.print("0");
                Serial.print(lastMessage[i], HEX);
                Serial.print(" ");
            }
            
            Serial.println();
        }
    }
}