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
 * that start with either the ID-byte CE or EE (odometer or tripmeter value). 
 * Any number and kind of bytes can be filtered.
 */

#include <CCDLibrary.h>

#define TBEN 4 // CCDPCIBusTransceiver has programmable CCD-bus termination and bias (TBEN pin instead of jumpers)

uint8_t messageFilter[] = { 0xCE, 0xEE }; // ID-bytes to filter

void CCDMessageReceived(uint8_t* message, uint8_t messageLength)
{
    for (uint8_t i = 0; i < messageLength; i++)
    {
        if (message[i] < 16) Serial.print("0"); // print leading zero
        Serial.print(message[i], HEX); // print message byte in hexadecimal format on the serial monitor
        Serial.print(" "); // insert whitespace between bytes
    }

    Serial.println(); // add new line
}

void CCDHandleError(CCD_Operations op, CCD_Errors err)
{
    if (err == CCD_OK) return;

    String s = op == CCD_Read ? "READ " : "WRITE ";

    switch (err)
    {
        case CCD_ERR_BUS_IS_BUSY:
        {
            Serial.println(s + "CCD_ERR_BUS_IS_BUSY");
            break;
        }
        case CCD_ERR_BUS_ERROR:
        {
            Serial.println(s + "CCD_ERR_BUS_ERROR");
            break;
        }
        case CCD_ERR_ARBITRATION_LOST:
        {
            Serial.println(s + "CCD_ERR_ARBITRATION_LOST");
            break;
        }
        case CCD_ERR_CHECKSUM:
        {
            Serial.println(s + "CCD_ERR_CHECKSUM");
            break;
        }
        default: // unknown error
        {
            Serial.println(s + "ERR: " + String(err, HEX));
            break;
        }
    }
}

void setup()
{
    Serial.begin(250000);

    pinMode(TBEN, OUTPUT);
    digitalWrite(TBEN, LOW); // LOW: enable, HIGH: disable CCD-bus termination and bias

    CCD.onMessageReceived(CCDMessageReceived); // subscribe to the message received event and call this function when a CCD-bus message is received
    CCD.onError(CCDHandleError); // subscribe to the error event and call this function when an error occurs
    //CCD.begin(); // CDP68HC68S1
    CCD.begin(CCD_DEFAULT_SPEED, CUSTOM_TRANSCEIVER, IDLE_BITS_10, ENABLE_RX_CHECKSUM, ENABLE_TX_CHECKSUM);

    CCD.ignoreAll(); // don't let any message through
    CCD.listen(messageFilter); // display selected messages only

    //CCD.listenAll(); // disable message filtering
    //CCD.ignore(messageFilter); // ignore selected messages
}

void loop()
{
    // Empty.
}
