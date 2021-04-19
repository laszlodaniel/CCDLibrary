/*
 * CCDWrite.ino (https://github.com/laszlodaniel/CCDLibrary)
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
 * Example: two custom CCD-bus messages are repeatedly written 
 * on the CCD-bus while other messages (including these) are displayed 
 * in the Arduino serial monitor. 
 * Next message is sent when the previous one is echoed back from the CCD-bus. 
 * Multiple messages can be sent one after another by adding 
 * new byte arrays and changing the "msgCount" value accordingly. 
 * The CCD.write() function automatically updates the source array with 
 * the correct checksum.
 */

#include <CCDLibrary.h>

#define TBEN 4 // CCDPCIBusTransceiver has programmable CCD-bus termination and bias (TBEN pin instead of jumpers)

uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 200; // ms
uint32_t messageSentMillis = 0;
uint16_t messageTimeout = 1000; // ms
uint8_t msgCount = 2;
uint8_t engineSpeed[4] = { 0xE4, 0x00, 0x00, 0x00 };
uint8_t vehicleSpeed[4] = { 0x24, 0x00, 0x00, 0x00 };
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
    pinMode(TBEN, OUTPUT);
    digitalWrite(TBEN, LOW); // LOW: enable, HIGH: disable CCD-bus termination and bias
    CCD.begin(); // CDP68HC68S1
    //CCD.begin(CCD_DEFAULT_SPEED, CUSTOM_TRANSCEIVER, IDLE_BITS_10, ENABLE_RX_CHECKSUM, ENABLE_TX_CHECKSUM);
}

void loop()
{
    currentMillis = millis(); // check current time

    if ((currentMillis - lastMillis) >= writeInterval) // check if writeInterval time has elapsed
    {
        lastMillis = currentMillis; // save current time
        if ((currentMillis - messageSentMillis) >= messageTimeout) next = true; // if previous message is lost somewhere then continue with the next one

        if (next) // don't send the next message until echo of the current one is heard on the CCD-bus
        {
            switch (counter) // fill currentMessageTX array with the current message
            {
                case 0:
                {
                    for (uint8_t i = 0; i < 4; i++) currentMessageTX[i] = engineSpeed[i];
                    currentMessageTXLength = 4;
                    break;
                }
                case 1:
                {
                    for (uint8_t i = 0; i < 4; i++) currentMessageTX[i] = vehicleSpeed[i];
                    currentMessageTXLength = 4;
                    break;
                }
                default:
                {
                    break;
                }
            }

            CCD.write(currentMessageTX, currentMessageTXLength); // send message on the CCD-bus
            messageSentMillis = currentMillis; // save time for timeout
            counter++; // send another message next time
            if (counter > (msgCount - 1)) counter = 0; // after the last message get back to the first one
            next = false; // wait for echo
        }
    }
    
    if (CCD.available()) // if there's a new unread message in the buffer
    {
        lastMessageLength = CCD.read(lastMessage); // read message in the lastMessage array and save its length in the lastMessageLength variable

        if (lastMessageLength > 0) // valid message length is always greater than 0
        {
            for (uint8_t i = 0; i < lastMessageLength; i++)
            {
                if (lastMessage[i] == currentMessageTX[i]) matchCount++; // number of bytes matching with the bytes in the currentMessageTX array
                if (lastMessage[i] < 16) Serial.print("0"); // print leading zero
                Serial.print(lastMessage[i], HEX); // print message byte in hexadecimal format on the serial monitor
                Serial.print(" "); // insert whitespace between bytes
            }

            if (matchCount == currentMessageTXLength) // sent message is the same as the received message
            {
                matchCount = 0; // reset value
                next = true; // echo accepted, send next message
            }

            Serial.println(); // add new line
        }
    }
}
