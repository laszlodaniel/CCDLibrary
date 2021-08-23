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

#include <CCDLibrary.h>
#include <util/atomic.h>
#include <util/delay.h>

CCDLibrary CCD;

CCDLibrary::CCDLibrary()
{
    // Empty class constructor.
}

CCDLibrary::~CCDLibrary()
{
    // Empty class destructor.
}

static void isrIdle()
{
    CCD.busIdleInterruptHandler();
}

static void isrActiveByte()
{
    CCD.activeByteInterruptHandler();
}

void CCDLibrary::begin(float baudrate, bool dedicatedTransceiver, uint8_t busIdleBits, bool verifyRxChecksum, bool calculateTxChecksum)
{
    // baudrate:
    //   CCD_DEFAULT_SPEED: one and only speed for CCD-bus is 7812.5 baud
    // dedicatedTransceiver:
    //   CDP68HC68S1: enables 1 MHz clock signal on D11/PB5 pin for the CDP68HC68S1 CCD-bus transceiver IC. 
    //                Bus-idle and arbitration detection is handled by this IC and signaled as external interrupts:
    //                  IDLE_PIN - Arduino pin connected to CDP68HC68S1's IDLE-pin.
    //                  CTRL_PIN - Arduino pin connected to CDP68HC68S1's CTRL-pin.
    //   CUSTOM_TRANSCEIVER: disables 1 MHz clock signal on D11/PB5 pin. The library handles bus-idle and arbitration detection based on timing and bit-manipulation.
    // busIdleBits:
    //   IDLE_BITS_XX: sets the number of consecutive 1-bits sensed as CCD-bus idle condition.
    //   IDLE_BITS_10: default idle bits is 10 according to the CDP68HC68S1 datasheet. It should be changed if messages are not coming through properly.
    // verifyRxChecksum:
    //   ENABLE_RX_CHECKSUM: verifies received messages against their last checksum byte and ignores them if broken.
    //   DISABLE_RX_CHECKSUM: accepts received messages without verification.
    // calculateTxChecksum:
    //   ENABLE_TX_CHECKSUM: calculates the checksum of outgoing messages and overwrites the last message byte with it.
    //   DISABLE_TX_CHECKSUM: sends messages as they are, no checksum calculation is perfomed.

    _baudrate = baudrate;
    _dedicatedTransceiver = dedicatedTransceiver;
    _busIdleBits = busIdleBits;
    _verifyRxChecksum = verifyRxChecksum;
    _calculateTxChecksum = calculateTxChecksum;
    _messageLength = 0;
    _busIdle = true;
    _transmitAllowed = true;

    serialInit();
    transmitDelayTimerInit();
    listenAll();

    if (_dedicatedTransceiver)
    {
        // CDP68HC68S1 transceiver chip.
        // Setup external interrupt for bus-idle detection on the IDLE pin.
        // Detach active byte interrupt.
        pinMode(IDLE_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(IDLE_PIN), isrIdle, CHANGE);
        detachInterrupt(digitalPinToInterrupt(CTRL_PIN));

        // Enable 1 MHz clock generator on Timer 1 and disable bus-idle timer at the same time.
        clockGeneratorInit();
    }
    else
    {
        // Custom transceiver circuits.
        // Setup external interrupt for active byte detection on the "CTRL" pin. RX1 pin is spliced and routed here.
        // Detach bus-idle interrupt. The active byte interrupt takes over the role of detecting bus-idle condition.
        pinMode(CTRL_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);
        detachInterrupt(digitalPinToInterrupt(IDLE_PIN));

        // Enable bus-idle timer on Timer 1 and disable 1 MHz clock generator at the same time.
        busIdleTimerInit();
    }
}

void CCDLibrary::serialInit()
{
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        _serialRxBufferPos = 0;
        _serialTxBufferPos = 0;
        _serialTxLength = 0;
        _lastSerialError = 0;

        // Set baud rate.
        uint16_t ubrr = (uint16_t)(((float)F_CPU / (16.0 * _baudrate)) - 1.0);
        UBRR1H = (ubrr >> 8) & 0x0F;
        UBRR1L = ubrr & 0xFF;

        // Enable UART receiver and transmitter and receive complete interrupt.
        UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

        // Set frame format: asynchronous, 8 data bit, no parity, 1 stop bit.
        UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);
    }
}

ISR(USART1_RX_vect)
{
    CCD.receiveByte();
}

void CCDLibrary::receiveByte()
{
    // Read UART status register and UART data register.
    uint8_t usr  = UCSR1A;
    uint8_t data = UDR1;

    // Get error bits from status register.
    uint8_t lastRxError = (usr & ((1 << FE1) | (1 << DOR1)));

    // Save byte in serial receive buffer.
    if (_serialRxBufferPos < 16)
    {
        _serialRxBuffer[_serialRxBufferPos] = data;
        _serialRxBufferPos++;
    }
    else
    {
        lastRxError |= UART_BUFFER_OVERFLOW; // error: buffer overflow
    }

    // Save last serial error.
    _lastSerialError = lastRxError;

    // Re-enable active byte interrupt.
    if (!_dedicatedTransceiver) attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);
}

ISR(USART1_UDRE_vect)
{
    CCD.transmitByte();
}

void CCDLibrary::transmitByte()
{
    if (_serialTxBufferPos < _serialTxLength)
    {
        UDR1 = _serialTxBuffer[_serialTxBufferPos]; // write next byte
        _serialTxBufferPos++; // increment Tx buffer position
    }
    else
    {
        UCSR1B &= ~(1 << UDRIE1); // Tx buffer empty, disable UDRE interrupt
        _serialTxBufferPos = 0; // reset Tx buffer position
        _serialTxLength = 0; // reset length
    }
}

void CCDLibrary::transmitDelayTimerInit()
{
    // Count 256 microseconds (3906.25 Hz) after CCD-bus goes idle.
    // After this interval elapses ID-byte transmission may occur.
    // Calculate top value to count.
    _calculatedOCR3AValue = (uint16_t)(((float)F_CPU / (3906.25 * 1024.0)) - 1.0);

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        TCCR3A = 0; // clear register
        TCCR3B = 0; // clear register
        TCNT3 = 0; // clear counter
        OCR3A = _calculatedOCR3AValue; // top value to count
        TCCR3B |= (1 << WGM32); // CTC mode, prescaler = 1024, stop timer
        TIMSK3 |= (1 << OCIE3A); // Output Compare Match A Interrupt Enable
    }
}

void CCDLibrary::transmitDelayTimerStart()
{
    TCNT3 = 0; // clear counter
    TCCR3B |= (1 << CS32) | (1 << CS30); // prescaler = 1024, start timer
}

void CCDLibrary::transmitDelayTimerStop()
{
    TCCR3B &= ~(1 << CS32) & ~(1 << CS30); // clear prescaler to stop timer
    TCNT3 = 0; // clear counter
}

ISR(TIMER3_COMPA_vect)
{
    CCD.transmitDelayHandler();
}

void CCDLibrary::transmitDelayHandler()
{
    transmitDelayTimerStop(); // stop transmit delay timer
    _transmitAllowed = true; // set flag
}

void CCDLibrary::clockGeneratorInit()
{
    // Enable 1 MHz clock signal for the CDP68HC68S1 transceiver.
    // OCR1A = (F_CPU / (2 * CLOCK * PRESCALER)) - 1
    // Clock frequency is multiplied by 2 because the raw signal needs to
    // oscillate two times for a single clock period. So in reality the timer
    // toggles the output pin at 2 MHz which results in a 1 MHz clock signal.
    //    F_CPU = 16000000 Hz for Arduino Mega
    //    CLOCK = 1000000 Hz
    //    PRESCALER = 1
    _calculatedOCR1AValue = (uint16_t)(((float)F_CPU / (2.0 * 1000000.0 * 1.0)) - 1.0);

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        TCCR1A = 0; // clear register
        TCCR1B = 0; // clear register
        TCNT1 = 0;  // clear counter
        DDRB |= (1 << DDB5); // set OC1A/PB5 as output
        TCCR1A |= (1 << COM1A0); // toggle OC1A on compare match
        OCR1A = _calculatedOCR1AValue; // top value for counter (16 MHz: 7; 8 MHz: 3)
        TIMSK1 &= ~(1 << OCIE1A); // Disable Output Compare Match A Interrupt
        TCCR1B |= (1 << WGM12) | (1 << CS10); // CTC mode, prescaler = 1, start timer
    }
}

void CCDLibrary::busIdleTimerInit()
{
    // Calculate top value to count beginning from the UART frame start bit until bus-idle condition.
    // OCR1A = ((F_CPU * (1 / BAUDRATE) * BIT_DELAY) / PRESCALER) - 1
    //    F_CPU = 16000000 Hz for Arduino Mega
    //    BAUDRATE = 7812.5 bits per second
    //    BIT_DELAY = 10 UART frame bits + _busIdleBits for idle detector
    //    PRESCALER = 1024
    _calculatedOCR1AValue = (uint16_t)((((float)F_CPU * (1.0 / _baudrate) * (10.0 + _busIdleBits)) / 1024.0) - 1.0);

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        TCCR1A = 0; // clear register
        TCCR1B = 0; // clear register
        TCNT1 = 0; // clear counter
        DDRB &= ~(1 << DDB5); // set OC1A/PB5 as input
        OCR1A = _calculatedOCR1AValue; // top value to count
        TCCR1B |= (1 << WGM12); // CTC mode, prescaler = 1024, stop timer
        TIMSK1 |= (1 << OCIE1A); // Enable Output Compare Match A Interrupt
    }
}

void CCDLibrary::busIdleTimerStart()
{
    TCNT1 = 0; // clear counter
    TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler = 1024, start timer
}

void CCDLibrary::busIdleTimerStop()
{
    TCCR1B &= ~(1 << CS12) & ~(1 << CS10); // clear prescaler to stop timer
    TCNT1 = 0; // clear counter
}

void CCDLibrary::busIdleInterruptHandler()
{
    if (PINE & (1 << PE4)) // IDLE pin transitioned from low to high = CCD-bus busy
    {
        _busIdle = false; // clear flag
        _transmitAllowed = false; // clear flag, interrupt controlled message transmission is not affected by this flag
    }
    else // IDLE pin transitioned from high to low = CCD-bus idle
    {
        transmitDelayTimerStart(); // start counting 256 microseconds for the next message transmission opportunity
        _busIdle = true; // set flag
        processMessage(); // process received message, if any
    }
}

void CCDLibrary::activeByteInterruptHandler()
{
    busIdleTimerStart(); // start bus-idle timer right after UART RX1 pin goes low (start bit)
    detachInterrupt(digitalPinToInterrupt(CTRL_PIN)); // disable interrupt until next byte's start bit
    _busIdle = false; // clear flag
    _transmitAllowed = false; // clear flag, interrupt controlled message transmission is not affected by this flag
}

ISR(TIMER1_COMPA_vect)
{
    CCD.timer1Handler();
}

void CCDLibrary::timer1Handler()
{
    busIdleTimerStop(); // stop bus idle timer
    _busIdle = true; // set flag
    transmitDelayTimerStart(); // start counting 256 microseconds for the next message transmission opportunity
    attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);
    processMessage(); // process received message, if any
}

uint8_t CCDLibrary::write(uint8_t* buffer, uint8_t bufferLength)
{
    // Return values:
    //   0: ok
    //   1: zero buffer length
    //   2: timeout
    //   3: data collision

    if (bufferLength == 0) return 1;

    for (uint8_t i = 0; i < bufferLength; i++) _serialTxBuffer[i] = buffer[i]; // copy message bytes to the transmit buffer

    if (_calculateTxChecksum && (bufferLength > 1)) // calculate message checksum if needed, minimum message length is 2 bytes
    {
        uint8_t checksum = 0;
        uint8_t checksumLocation = bufferLength - 1;
        for (uint8_t i = 0; i < checksumLocation ; i++) checksum += buffer[i];
        _serialTxBuffer[checksumLocation] = checksum; // overwrite last byte in the message with the correct checksum value
        buffer[checksumLocation] = checksum; // overwrite checksum in the source array too
    }

    _serialTxBufferPos = 0; // reset buffer position
    _serialTxLength = bufferLength; // save message length

    bool timeout = false;
    uint32_t timeoutStart = millis();

    // Warning: no bus-arbitration detector algorithm is used!
    // Wait for bus-idle to begin message transmission.
    while (!_busIdle && !timeout)
    {
        if ((uint32_t)(millis() - timeoutStart) >= 1000) timeout = true;
    }

    if (timeout) return 2;

    // Insert 256 microseconds delay here. Since loading the UART TX buffer takes some time let's wait a bit less.
    _delay_us(64.0);

    // Enable UDRE interrupt to begin message transmission.
    UCSR1B |= (1 << UDRIE1);

    return 0;

/*     if (!_dedicatedTransceiver) // CDP68HC68S1 handles arbitration detection internally
    {
        bool timeout = false;
        uint32_t timeoutStart = millis();

        // Wait for bus-idle, the 256 microseconds delay is handled by the transceiver chip.
        while (!_busIdle && !timeout)
        {
            if ((uint32_t)(millis() - timeoutStart) >= 1000) timeout = true;
        }

        if (timeout) return 2;

        // Enable UDRE interrupt to begin message transmission. That's it.
        // The CDP68HC68S1 chip takes care of everything.
        UCSR1B |= (1 << UDRIE1);

        return 0;
    }
    else // custom CCD-bus transceiver circuit needs special handling
    {
        // Disable UART1.
        UCSR1B &= ~(1 << RXCIE1) & ~(1 << RXEN1) & ~(1 << TXEN1) & ~(1 << UDRIE1);

        // Setup UART1 pins manually.
        RX_DDR &= ~(1 << RX_P); // RX1 is input
        TX_DDR |= (1 << TX_P);  // TX1 is output
        RX_PORT |= (1 << RX_P); // RX1 internal pullup resistor enabled
        TX_PORT |= (1 << TX_P); // TX1 internal pullup resistor enabled

        // Prepare variables.
        uint8_t IDbyteTX = _serialTxBuffer[0];
        uint8_t IDbyteRX = 0;
        bool currentRxBit = false;
        bool currentTxBit = false;
        bool error = false;
        bool timeout = false;
        uint32_t timeoutStart = millis();

        // Wait for bus-idle, then wait some more until 256 microseconds elapses after that to begin message transmission.
        while (!_transmitAllowed && !timeout)
        {
            if ((uint32_t)(millis() - timeoutStart) >= 1000) timeout = true;
        }

        if (timeout) return 2;

        _transmitAllowed = false; // clear flag

        if (!(RX_PIN & (1 << RX_P))) error = true; // check if start bit has appeared, if so: bus arbitration lost
        if (!error) cbi(TX_PORT, TX_P); // send start bit (0) immediately

        // Start bit-banging RX1/TX1 pins.
        // Arbitration detection is done by checking if written bit is the same as the received bit.
        for (uint8_t i = 0; i < 10; i++) // send 10 bits
        {
            if (!error) // ongoing arbitration
            {
                if (i == 0) currentTxBit = 0; // start bit
                else if (i == 9) currentTxBit = 1; // stop bit
                else currentTxBit = IDbyteTX & (1 << (i - 1)); // 8 data bits

                if (currentTxBit) sbi(TX_PORT, TX_P);
                else cbi(TX_PORT, TX_P);
            }
            else // arbitration lost
            {
                sbi(TX_PORT, TX_P); // keep TX-pin in non-destructive state
            }

            _delay_us(64.0); // wait 0.5 bit time

            currentRxBit = (RX_PIN & (1 << RX_P)); // read RX pin state
            if (currentRxBit != currentTxBit) error = true;// error: bit mismatch, bus arbitration lost

            if ((i != 0) && (i != 9)) // skip start and stop bits
            {
                if (currentRxBit) sbi(IDbyteRX, (i - 1)); // save bit
            }

            _delay_us(64.0); // wait 0.5 bit time
        }

        // Save received ID byte.
        _serialRxBuffer[0] = IDbyteRX;
        _serialRxBufferPos = 1;

        // Re-enable UART receiver and transmitter and receive complete interrupt.
        UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

        if (IDbyteRX == IDbyteTX) // CCD-bus arbitration won
        {
            // Send second byte next.
            _serialTxBufferPos = 1;

            // Re-enable active byte interrupt.
            attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);

            // Enable UDRE interrupt to continue automatic message transmission.
            UCSR1B |= (1 << UDRIE1);

            return 0;
        }
        else  // CCD-bus arbitration lost
        {
            // Reset transmit buffer.
            _serialTxBufferPos = 0;
            _serialTxLength = 0;

            // Re-enable active byte interrupt.
            attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);

            return 3;
        }
    } */
}

void CCDLibrary::listenAll()
{
    memset(_ignoreList, 0, sizeof(_ignoreList));
}

void CCDLibrary::listen(uint8_t* ids)
{
    ignoreAll();

    while (*ids)
    {
        _ignoreList[*ids] = 0; // clear ignore flag
        ids++;
    }
}

void CCDLibrary::ignoreAll()
{
    memset(_ignoreList, 1, sizeof(_ignoreList));
}

void CCDLibrary::ignore(uint8_t* ids)
{
    listenAll();

    while (*ids)
    {
        _ignoreList[*ids] = 1; // set ignore flag
        ids++;
    }
}

void CCDLibrary::processMessage()
{
    if (_serialRxBufferPos > 0)
    {
        if (_ignoreList[_serialRxBuffer[0]] == 0)
        {
            if (_verifyRxChecksum && (_serialRxBufferPos > 1)) // verify checksum
            {
                uint8_t checksum = 0;
                uint8_t checksumLocation = _serialRxBufferPos - 1;
                for (uint8_t i = 0; i < checksumLocation ; i++) checksum += _serialRxBuffer[i];

                if (checksum == _serialRxBuffer[checksumLocation])
                {
                    for (uint8_t i = 0; i < _serialRxBufferPos; i++) _message[i] = _serialRxBuffer[i];

                    _messageLength = _serialRxBufferPos;
                    _serialRxBufferPos = 0;
                    handleMessagesInternal(_message, _messageLength);
                }
                else
                {
                    _serialRxBufferPos = 0;
                    handleErrorsInternal(CCD_Read, CCD_ERR_CHECKSUM);
                }
            }
            else // ignore checksum
            {
                for (uint8_t i = 0; i < _serialRxBufferPos; i++) _message[i] = _serialRxBuffer[i];

                _messageLength = _serialRxBufferPos;
                _serialRxBufferPos = 0;
                handleMessagesInternal(_message, _messageLength);
            }
        }
        else
        {
            _serialRxBufferPos = 0;
        }
    }
}

void CCDLibrary::handleMessagesInternal(uint8_t* _message, uint8_t _messageLength)
{
    onCCDMessageReceivedHandler msgHandler = _msgHandler;

    if (msgHandler)
    {
        msgHandler(_message, _messageLength); // raise event
    }
}

void CCDLibrary::onMessageReceived(onCCDMessageReceivedHandler msgHandler)
{
    _msgHandler = msgHandler;
}

void CCDLibrary::handleErrorsInternal(CCD_Operations _op, CCD_Errors _err)
{
    if (_err != CCD_OK)
    {
        onCCDErrorHandler errHandler = _errHandler;

        if (errHandler)
        {
            errHandler(_op, _err); // raise event
        }
    }
}

void CCDLibrary::onError(onCCDErrorHandler errHandler)
{
    _errHandler = errHandler;
}
