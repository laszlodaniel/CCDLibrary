/*
 * CCDWrite.ino (https://github.com/laszlodaniel/CCDLibrary)
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
 * Next message is sent when the previous one is echoed back from the CCD-bus. 
 * Multiple messages can be sent one after another by adding 
 * new byte arrays and changing the "msgCount" value accordingly.
 *
 * Wiring (CCDBusTransceiver): 
 * Connect RX/TX pins to the Arduino Mega's / ATmega2560's TX1/RX1 (UART1-channel) pins, respectively. 
 * Use the Arduino's +5V and GND pins to supply power to the development board. 
 * Connect CCD+ and CCD- pins to the vehicle's diagnostic connector (OBD1 or OBD2). 
 * Make sure to connect the additional GND pin to the diagnostic connector's ground pin. 
 * Connect T_EN jumper if standalone operation is needed (without a compatible vehicle). 
 * Disconnect T_EN jumper if CCD-bus is acting strange.
 */

#include <CCDLibrary.h>

uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 200; // ms
uint32_t messageSentMillis = 0;
uint16_t messageTimeout = 1000; // ms
uint8_t msgCount = 2;
uint8_t engineSpeed[4] = { 0xE4, 0x00, 0x00, 0x00 }; // after checksum calculation this reads E4 00 00 E4
uint8_t vehicleSpeed[4] = { 0x24, 0x00, 0x00, 0x00 }; // after checksum calculation this reads 24 00 00 24
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
    //CCD.begin(CLOCK_1MHZ_OFF, IDLE_BITS_14, DISABLE_RX_CHECKSUM, DISABLE_TX_CHECKSUM);
}

void loop()
{
    currentMillis = millis(); // check current time
    
    if (currentMillis - lastMillis >= writeInterval) // check if writeInterval time has elapsed
    {
        lastMillis = currentMillis; // save current time
        if ((currentMillis - messageSentMillis) > messageTimeout) next = true; // if previous message is lost somewhere then continue with the next one
        
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
            
            CCD.write(currentMessageTX, currentMessageTXLength);
            messageSentMillis = currentMillis;
            counter++; // another message next time
            if (counter > (msgCount - 1)) counter = 0; // after last message get back to the first one
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
                if (lastMessage[i] == currentMessageTX[i]) matchCount++;
                if (lastMessage[i] < 16) Serial.print("0"); // print leading zero
                Serial.print(lastMessage[i], HEX); // print message byte in hexadecimal format on the serial monitor
                Serial.print(" "); // insert whitespace between bytes
            }
    
            if (matchCount >= (currentMessageTXLength - 1)) // don't count the last checksum byte
            {
                matchCount = 0; // reset value
                next = true; // echo accepted, send next message
            }
            Serial.println(); // add new line
        }
        else // ignore invalid messages
        {
            next = true;
        }
    }
}
