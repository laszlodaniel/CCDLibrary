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
 * Example: two custom CCD-bus messages are repeatedly written 
 * on the CCD-bus while other messages (including these) are displayed 
 * in the Arduino serial monitor. 
 * Next message is sent when the previous one is echoed back 
 * from the CCD-bus. 
 * Multiple messages can be sent one after another by adding 
 * new byte arrays and changing the "msgCount" value accordingly.
 */

#include <CCDLibrary.h>

uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 200; // ms
uint8_t msgCount = 2;
uint8_t engineSpeed[4] = { 0xE4, 0x00, 0x00, 0xE4 };
uint8_t vehicleSpeed[4] = { 0x24, 0x00, 0x00, 0x24 };
uint8_t currentMessageTX[16];
uint8_t currentMessageTXLength = 0;
uint8_t lastMessage[16];
uint8_t lastMessageLength = 0;
uint8_t counter = 0;
uint8_t matchCount = 0;
bool next = true;

void setup()
{
    Serial.begin(250000);
    CCD.begin();
}

void loop()
{
    currentMillis = millis();
    
    if (currentMillis - lastMillis >= writeInterval)
    {
        lastMillis = currentMillis;
        
        if (next)
        {
            next = false;
            
            switch (counter)
            {
                case 0:
                {
                    for (uint8_t i = 0; i < 4; i++) currentMessageTX[i] = engineSpeed[i];
                    currentMessageTXLength = 4;
                    CCD.write(currentMessageTX, currentMessageTXLength);
                    break;
                }
                case 1:
                {
                    for (uint8_t i = 0; i < 4; i++) currentMessageTX[i] = vehicleSpeed[i];
                    currentMessageTXLength = 4;
                    CCD.write(currentMessageTX, currentMessageTXLength);
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            counter++;
            if (counter > (msgCount - 1)) counter = 0;
        }
    }

    if (CCD.available())
    {
        lastMessageLength = CCD.read(lastMessage);
        
        for (uint8_t i = 0; i < lastMessageLength; i++)
        {
            if (lastMessage[i] == currentMessageTX[i]) matchCount++;
            if (lastMessage[i] < 16) Serial.print("0");
            Serial.print(lastMessage[i], HEX);
            Serial.print(" ");
        }

        if (matchCount == currentMessageTXLength)
        {
            matchCount = 0;
            next = true;
        }
        Serial.println();
    }
}
