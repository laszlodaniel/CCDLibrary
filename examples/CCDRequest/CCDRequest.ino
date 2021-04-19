/*
 * CCDRequest.ino (https://github.com/laszlodaniel/CCDLibrary)
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
 * Example: a diagnostic request message (B2) is sent on the CCD-bus. 
 * A diagnostic response message (F2) is expected from the target module. 
 * CCD-bus messages are displayed in the Arduino serial monitor 
 * and are filtered by these two ID bytes. 
 * With "writeInterval" the request speed can be changed.
 * The CCD.write() function automatically updates the source array with 
 * the correct checksum.
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

#define TBEN 4 // CCDPCIBusTransceiver has programmable CCD-bus termination and bias (TBEN pin instead of jumpers)

uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 500; // ms
uint8_t BCMROMValueRequest[6] = { 0xB2, 0x20, 0x22, 0x00, 0x00, 0x00 }; // checksum intentionally set as zero
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
    currentMillis = millis(); // check current time

    if ((currentMillis - lastMillis) >= writeInterval) // check if writeInterval time has elapsed
    {
        lastMillis = currentMillis; // save current time

        uint8_t result = CCD.write(BCMROMValueRequest, 6); // write 6 bytes from the BCMROMValueRequest array on the CCD-bus

        if (result > 0) // check if error occured during message transmission (0 = OK)
        {
            switch (result)
            {
                case 1:
                {
                    Serial.println("Error: zero message length");
                    break;
                }
                case 2:
                {
                    Serial.println("Error: timeout");
                    break;
                }
                case 3:
                {
                    Serial.println("Error: data collision");
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