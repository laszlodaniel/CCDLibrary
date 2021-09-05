/*
 * CCDLibrary (https://github.com/laszlodaniel/CCDLibrary)
 * Copyright (C) 2020-2021, Daniel Laszlo
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

#define CCD_DEFAULT_SPEED     7812.5 // default CCD-bus baudrate
#define UART_FRAME_ERROR      0x10 // framing error by UART
#define UART_OVERRUN_ERROR    0x08 // overrun condition by UART
#define UART_BUFFER_OVERFLOW  0x04 // receive buffer overflow
#define UART_NO_DATA          0x02 // receive buffer is empty
#define IDLE_BITS_05          5
#define IDLE_BITS_06          6
#define IDLE_BITS_07          7
#define IDLE_BITS_08          8
#define IDLE_BITS_09          9
#define IDLE_BITS_10          10   // CCD-bus idle condition
#define IDLE_BITS_11          11
#define IDLE_BITS_12          12
#define IDLE_BITS_13          13
#define IDLE_BITS_14          14
#define IDLE_BITS_15          15
#define ENABLE_RX_CHECKSUM    1    // verify received message checksum
#define DISABLE_RX_CHECKSUM   0
#define ENABLE_TX_CHECKSUM    1    // calculate outgoing message checksum
#define DISABLE_TX_CHECKSUM   0
#define CDP68HC68S1           1    // CDP68HC68S1 has two dedicated pins to signal CCD-bus condition
#define IDLE_PIN              2    // Arduino Mega: INT4 pin (CCD-bus idle interrupt)
#define CTRL_PIN              3    // Arduino Mega: INT5 pin (CCD-bus active byte (control) interrupt)
#define CUSTOM_TRANSCEIVER    0

// Set (1), clear (0) and invert (1->0; 0->1) bit in a register or variable easily.
#define sbi(reg, bit) reg |=  (1 << bit)
#define cbi(reg, bit) reg &= ~(1 << bit)
#define ibi(reg, bit) reg ^=  (1 << bit)

enum CCD_Operations
{
    CCD_Read,
    CCD_Write
};

enum CCD_Errors
{
    CCD_OK                        = 0x00,
    CCD_ERR_BUS_IS_BUSY           = 0x81,
    CCD_ERR_BUS_ERROR             = 0x82,
    CCD_ERR_ARBITRATION_LOST      = 0x87,
    CCD_ERR_CHECKSUM              = 0x90
};

typedef void (*onCCDMessageReceivedHandler)(uint8_t* message, uint8_t messageLength);
typedef void (*onCCDErrorHandler)(CCD_Operations op, CCD_Errors err);

class CCDLibrary
{
    public:
        CCDLibrary();
        ~CCDLibrary();
        void begin(float baudrate = 7812.5, bool dedicatedTransceiver = 1, uint8_t busIdleBits = 10, bool verifyRxChecksum = 1, bool calculateTxChecksum = 1);
        uint8_t write(uint8_t *buffer, uint8_t bufferLength);
        void listenAll();
        void listen(uint8_t* ids);
        void ignoreAll();
        void ignore(uint8_t* ids);
        void receiveByte();
        void transmitByte();
        void transmitDelayHandler();
        void timer1Handler();
        void busIdleInterruptHandler();
        void activeByteInterruptHandler();
        void onMessageReceived(onCCDMessageReceivedHandler msgHandler);
        void onError(onCCDErrorHandler errHandler);

    private:
        uint8_t _message[16];
        uint8_t _messageLength;
        volatile uint8_t _serialRxBuffer[16];
        volatile uint8_t _serialRxBufferPos;
        volatile uint8_t _serialTxBuffer[16];
        volatile uint8_t _serialTxBufferPos;
        volatile uint8_t _serialTxLength;
        volatile uint8_t _lastSerialError;
        volatile bool _busIdle;
        volatile bool _transmitAllowed;
        float _baudrate;
        bool _dedicatedTransceiver;
        uint8_t _busIdleBits;
        bool _verifyRxChecksum;
        bool _calculateTxChecksum;
        uint16_t _calculatedOCR1AValue;
        uint16_t _calculatedOCR3AValue;
        uint8_t _ignoreList[32];
        volatile onCCDMessageReceivedHandler _msgHandler;
        volatile onCCDErrorHandler _errHandler;
        void serialInit();
        void transmitDelayTimerInit();
        void transmitDelayTimerStart();
        void transmitDelayTimerStop();
        void clockGeneratorInit();
        void busIdleTimerInit();
        void busIdleTimerStart();
        void busIdleTimerStop();
        void processMessage();
        uint8_t* getBit(uint8_t _id, uint8_t* _pBit);
        void handleMessagesInternal(uint8_t* _message, uint8_t _messageLength);
        void handleErrorsInternal(CCD_Operations _op, CCD_Errors _err);
};

extern CCDLibrary CCD;

#endif