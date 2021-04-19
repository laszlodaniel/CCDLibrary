/*
 * CCDFilter.ino (https://github.com/laszlodaniel/CCDLibrary)
 * Copyright (C) 2021, Daniel Laszlo
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

#define TBEN 4 // CCDPCIBusTransceiver has programmable CCD-bus termination and bias (TBEN pin instead of jumpers)

uint8_t lastMessage[16];
uint8_t lastMessageLength = 0;
uint8_t messageIDbyte = 0;

void setup()
{
    Serial.begin(250000);
    pinMode(TBEN, OUTPUT);
    digitalWrite(TBEN, LOW); // LOW: enable, HIGH: disable CCD-bus termination and bias
    CCD.begin(); // CDP68HC68S1
    //CCD.begin(CCD_DEFAULT_SPEED, CUSTOM_TRANSCEIVER, IDLE_BITS_10, ENABLE_RX_CHECKSUM, ENABLE_TX_CHECKSUM);
}

void loop()
{
    if (CCD.available()) // if there's a new unread message in the buffer
    {
        lastMessageLength = CCD.read(lastMessage); // read message in the lastMessage array and save its length in the lastMessageLength variable

        if (lastMessageLength > 0) // valid message length is always greater than 0
        {
            messageIDbyte = lastMessage[0]; // save first byte of the message in a separate variable

            if ((messageIDbyte == 0xB2) || (messageIDbyte == 0xF2)) // diagnostic request/response message filter
            {
                for (uint8_t i = 0; i < lastMessageLength; i++)
                {
                    if (lastMessage[i] < 16) Serial.print("0"); // print leading zero
                    Serial.print(lastMessage[i], HEX); // print message byte in hexadecimal format on the serial monitor
                    Serial.print(" "); // insert whitespace between bytes
                }

                Serial.println(); // add new line
            }
        }
    }
}