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

uint8_t lastMessage[16];
uint8_t lastMessageLength = 0;

void setup()
{
    Serial.begin(250000);
    CCD.begin(); // CDP68HC68S1
    //CCD.begin(CCD_DEFAULT_SPEED, NO_INTERRUPTS, IDLE_BITS_13, ENABLE_RX_CHECKSUM, ENABLE_TX_CHECKSUM);
}

void loop()
{
    if (CCD.available()) // if there's a new unread message in the buffer
    {
        lastMessageLength = CCD.read(lastMessage); // read message in the lastMessage array and save its length in the lastMessageLength variable
        
        if (lastMessageLength > 0) // valid message length is always greater than 0
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