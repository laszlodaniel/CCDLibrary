/*
 * CCDRequest.ino (https://github.com/laszlodaniel/CCDLibrary)
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
 * Example: a diagnostic request message (B2) is sent on the CCD-bus. 
 * A diagnostic response message (F2) is expected from the target module. 
 * CCD-bus messages are displayed in the Arduino serial monitor 
 * and are filtered by these two ID bytes. 
 * With "writeInterval" the request speed can be changed.
 *
 * Diagnostic request/response message format:
 *
 * B2: DRB request message ID byte
 * 20: target module on the CCD-bus (20 = Body Control Module, BCM)
 * 22: command: read RAM/ROM/EEPROM value
 * 00: RAM/ROM/EEPROM address (16-bit) high byte
 * 00: RAM/ROM/EEPROM address (16-bit) low byte
 * F4: checksum (B2+20+22+00+00 & FF)
 *
 * F2: DRB response message ID byte
 * 20: responding module on the CCD-bus (Body Control Module, BCM)
 * 22: responding to this command: read RAM/ROM/EEPROM
 * 15: RAM/ROM/EEPROM value at the previously given 16-bit address (0000)
 * EA: RAM/ROM/EEPROM value at the next 16-bit address (0001)
 * 33: checksum (F2+20+22+15+EA & FF)
 */

#include <CCDLibrary.h>

//#define ChryslerCCDSCIScanner // 1 MHz clock signal is necessary for the CDP68HC68S1 CCD-bus transceiver IC to work

uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 500; // ms
uint8_t BCMROMValueRequest[6] = { 0xB2, 0x20, 0x22, 0x00, 0x00, 0xF4 };
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
    currentMillis = millis();
    
    if (currentMillis - lastMillis >= writeInterval)
    {
        lastMillis = currentMillis;
        uint8_t result = CCD.write(BCMROMValueRequest, 6);
        if (result > 0)
        {
            switch (result)
            {
                case 1:
                {
                    Serial.println("Error: data collision");
                    break;
                }
                case 2:
                {
                    Serial.println("Error: timeout");
                    break;
                }
                default:
                {
                    Serial.println("Error: unknown");
                    break;
                }
            }
        }
    }

    if (CCD.available())
    {
        lastMessageLength = CCD.read(lastMessage);
        messageIDbyte = lastMessage[0];
        
        // Diagnostic request/response message filter
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
