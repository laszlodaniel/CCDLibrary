/*
 * CCDLibrary (https://github.com/laszlodaniel/CCDLibrary)
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
 */

#ifndef CCDLibrary_H
#define CCDLibrary_H

#include <Arduino.h>

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__) // Arduino Mega / ATmega2560, UART1 channel
    #define RX_DDR  DDRD
    #define TX_DDR  DDRD
    #define RX_PORT PORTD
    #define TX_PORT PORTD
    #define RX_P    PD2 // RX1 - D19
    #define TX_P    PD3 // TX1 - D18
    #define RX_PIN  PIND
    #define TX_PIN  PIND
#else
    #error "Arduino Mega / ATmega2560 microcontroller is required!"
#endif

#define CCDUBRR               127  // prescaler for 7812.5 baud speed, UBRR = (F_CPU / (16 * BAUDRATE)) - 1
#define UART_FRAME_ERROR      0x10 // framing error by UART
#define UART_OVERRUN_ERROR    0x08 // overrun condition by UART
#define UART_BUFFER_OVERFLOW  0x04 // receive buffer overflow
#define UART_NO_DATA          0x02 // receive buffer is empty
#define CLOCK_ON              1    // 1 MHz clock signal for the CDP68HC68S1 chip
#define CLOCK_OFF             0
#define IDLE_BITS_10          10   // to calculate CCD-bus idle condition
#define IDLE_BITS_11          11
#define IDLE_BITS_12          12
#define IDLE_BITS_13          13
#define IDLE_BITS_14          14
#define IDLE_BITS_15          15

// Set (1), clear (0) and invert (1->0; 0->1) bit in a register or variable easily.
#define sbi(reg, bit) reg |=  (1 << bit)
#define cbi(reg, bit) reg &= ~(1 << bit)
#define ibi(reg, bit) reg ^=  (1 << bit)

class CCDLibrary
{
    public:
        CCDLibrary();
        void begin(bool clockGenerator = 0, uint8_t busIdleBits = 14);
        bool available();
        uint8_t read(uint8_t *target);
        uint8_t write(uint8_t *buffer, uint8_t bufferLength);
        void handle_TIMER3_COMPA_vect();
        void handle_USART1_RX_vect();
        void handle_USART1_UDRE_vect();
    
    private:
        volatile uint8_t _message[16];
        volatile uint8_t _messageLength;
        volatile uint8_t _serialRxBuffer[16];
        volatile uint8_t _serialRxBufferPos;
        volatile uint8_t _serialTxBuffer[16];
        volatile uint8_t _serialTxBufferPos;
        volatile uint8_t _serialTxLength;
        volatile uint8_t _lastSerialError;
        volatile bool _busIdle;
        uint8_t _busIdleBits;
        uint8_t _calculatedOCRAValue;
        bool _lastMessageRead;
        void uartInit(uint16_t ubrr);
        void busIdleTimerInit();
        void busIdleTimerStart();
        void busIdleTimerStop();
};

extern CCDLibrary CCD;

#endif
