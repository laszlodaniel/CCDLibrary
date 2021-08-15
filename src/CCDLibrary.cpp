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
    // Empty.
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
    _lastMessageRead = true;
    _busIdle = true;

    serialInit(_baudrate);

    if (_dedicatedTransceiver)
    {
        // Enable 1 MHz clock generator on Timer 1.
        clockGeneratorInit();
        
        // Setup external interrupts for bus-idle and active byte detection.
        pinMode(IDLE_PIN, INPUT_PULLUP);
        pinMode(CTRL_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(IDLE_PIN), isrIdle, FALLING);
        attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);
    }
    else
    {
        // Enable bus-idle timer on Timer 1 and disable 1 MHz clock generator at the same time.
        busIdleTimerInit();

        // Setup external interrupt for active byte detection.
        // Detach bus-idle interrupt and let the active byte interrupt do its task.
        pinMode(CTRL_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);
        detachInterrupt(digitalPinToInterrupt(IDLE_PIN));

        // Start bus-idle timer.
        busIdleTimerStart();
    }
}

void CCDLibrary::serialInit(float baudrate)
{
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        _serialRxBufferPos = 0;
        _serialTxBufferPos = 0;
        _serialTxLength = 0;
        _lastSerialError = 0;

        // Set baud rate.
        uint16_t ubrr = (uint16_t)(((float)F_CPU / (16.0 * baudrate)) - 1.0);
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
    CCD.handle_USART1_RX_vect();
}

void CCDLibrary::handle_USART1_RX_vect()
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
    CCD.handle_USART1_UDRE_vect();
}

void CCDLibrary::handle_USART1_UDRE_vect()
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

    noInterrupts();
    TCCR1A = 0; // clear register
    TCCR1B = 0; // clear register
    TCNT1 = 0;  // clear counter
    DDRB |= (1 << DDB5); // set OC1A/PB5 as output
    TCCR1A |= (1 << COM1A0); // toggle OC1A on compare match
    OCR1A = _calculatedOCR1AValue; // top value for counter (16 MHz: 7; 8 MHz: 3)
    TCCR1B |= (1 << WGM12) | (1 << CS10); // CTC mode, prescaler = 1, start timer
    interrupts();
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

    noInterrupts();
    TCCR1A = 0; // clear register
    TCCR1B = 0; // clear register
    TCNT1 = 0; // clear counter
    DDRB &= ~(1 << DDB5); // set OC1A/PB5 as input
    OCR1A = _calculatedOCR1AValue; // top value to count
    TCCR1B |= (1 << WGM12); // CTC mode, prescaler = 1024, stop timer
    TIMSK1 |= (1 << OCIE1A); // Output Compare Match A Interrupt Enable
    interrupts();
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
    _busIdle = true; // set flag
    processMessage(); // process received message, if any
}

void CCDLibrary::activeByteInterruptHandler()
{
    _busIdle = false; // clear flag

    if (!_dedicatedTransceiver)
    {
        busIdleTimerStart(); // start bus-idle timer
        detachInterrupt(digitalPinToInterrupt(CTRL_PIN)); // disable interrupt until next byte's start bit
    }
}

ISR(TIMER1_COMPA_vect)
{
    CCD.handle_TIMER1_COMPA_vect();
}

void CCDLibrary::handle_TIMER1_COMPA_vect()
{
    if (!_dedicatedTransceiver)
    {
        _busIdle = true; // set flag
        busIdleTimerStop(); // stop bus idle timer
        attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);
        processMessage(); // process received message, if any
    }
}

bool CCDLibrary::available()
{
    return !_lastMessageRead;
}

uint8_t CCDLibrary::read(uint8_t *target)
{
    // Copy last message to target buffer.
    for (uint8_t i = 0; i < _messageLength; i++) target[i] = _message[i];

    _lastMessageRead = true; // set flag
    return _messageLength;
}

uint8_t CCDLibrary::write(uint8_t *buffer, uint8_t bufferLength)
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
    uint32_t timeout_start = millis();

    while (!_busIdle && !timeout) // wait for bus idle condition or timeout (1 second)
    {
        if ((millis() - timeout_start) > 1000) timeout = true;
    }

    if (timeout) return 2;

    if (_dedicatedTransceiver) // CDP68HC68S1 handles arbitration detection internally
    {
        // Enable UDRE interrupt to begin message transmission. That's it.
        // The CDP68HC68S1 chip takes care of everything.
        noInterrupts();
        UCSR1B |= (1 << UDRIE1);
        interrupts();

        return 0;
    }
    else
    {
        // Disable UART.
        noInterrupts();
        UCSR1B &= ~(1 << RXCIE1) & ~(1 << RXEN1) & ~(1 << TXEN1) & ~(1 << UDRIE1);
        interrupts();

        // Clear bus-idle flag
        _busIdle = false;

        // Setup UART pins manually.
        RX_DDR &= ~(1 << RX_P); // RX is input
        TX_DDR |= (1 << TX_P); // TX is output
        RX_PORT |= (1 << RX_P); // RX internal pullup resistor enabled
        TX_PORT |= (1 << TX_P); // TX idling at logic high

        // Prepare variables.
        uint8_t IDbyteTX = _serialTxBuffer[0];
        uint8_t IDbyteRX = 0;
        bool currentRxBit = false;
        bool currentTxBit = false;
        bool error = false;

        // First we have to wait 2 bit time (256 microseconds) for synchronisation.
        _delay_us(251.0); // approximately 5 microseconds elapses until we end up here so don't count that

        // Check if start bit has appeared.
        currentRxBit = (RX_PIN & (1 << RX_P));
        if (!currentRxBit) error = true; // it's supposed to be logic high, another module is ahead of us, bus arbitration lost

        // Start bit-banging RX/TX pins. 
        // Arbitration detection is done by checking if written bit is the same as the received bit.
        for (uint8_t i = 0; i < 10; i++) // send 10 bits
        {
            if (!error)
            {
                if (i == 0) currentTxBit = 0; // 1 start bit
                else if (i == 9) currentTxBit = 1; // 1 stop bit
                else currentTxBit = IDbyteTX & (1 << (i - 1)); // 8 data bits

                if (currentTxBit) sbi(TX_PORT, TX_P);
                else cbi(TX_PORT, TX_P);
            }
            else
            {
                sbi(TX_PORT, TX_P); // keep TX-pin in non-destructive state
            }

            _delay_us(64.0); // wait 0.5 bit time
            currentRxBit = (RX_PIN & (1 << RX_P)); // read RX pin state

            if ((i != 0) && (i != 9)) // skip start and stop bits
            {
                if (currentRxBit) sbi(IDbyteRX, (i - 1)); // save bit
            }

            if (currentRxBit != currentTxBit) error = true;// error: bit mismatch, bus arbitration lost
            _delay_us(64.0); // wait another 0.5 bit time to finish this bit
        }

        // Save ID byte
        _serialRxBuffer[0] = IDbyteRX;
        _serialRxBufferPos = 1;

        // Re-enable UART receiver and transmitter and receive complete interrupt.
        noInterrupts();
        UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
        interrupts();

        if (IDbyteRX == IDbyteTX) // CCD-bus arbitration won
        {
            // Send second byte next.
            _serialTxBufferPos = 1;

            // Re-enable active byte interrupt.
            attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);

            // Enable UDRE interrupt to continue automatic message transmission.
            noInterrupts();
            UCSR1B |= (1 << UDRIE1);
            interrupts();

            return 0;
        }
        else
        {
            // Reset transmit buffer.
            _serialTxBufferPos = 0;
            _serialTxLength = 0;

            // Re-enable active byte interrupt.
            attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrActiveByte, FALLING);

            return 3;
        }
    }
}

void CCDLibrary::processMessage()
{
    // Check if there is something in the buffer.
    if (_serialRxBufferPos > 0)
    {
        if (_verifyRxChecksum && (_serialRxBufferPos > 1)) // verify checksum
        {
            uint8_t checksum = 0;
            uint8_t checksumLocation = _serialRxBufferPos - 1;
            for (uint8_t i = 0; i < checksumLocation ; i++) checksum += _serialRxBuffer[i];

            if (checksum == _serialRxBuffer[checksumLocation])
            {
                // Copy bytes from serial receive buffer to message buffer.
                for (uint8_t i = 0; i < _serialRxBufferPos; i++) _message[i] = _serialRxBuffer[i];

                _messageLength = _serialRxBufferPos;
                _serialRxBufferPos = 0;
            }
            else
            {
                _messageLength = 0; // let invalid messages have zero length
                _serialRxBufferPos = 0; // ignore this message and reset buffer
            }
        }
        else // checksum calculation is not applicable
        {
            // Copy bytes from serial receive buffer to message buffer.
            for (uint8_t i = 0; i < _serialRxBufferPos; i++) _message[i] = _serialRxBuffer[i];

            _messageLength = _serialRxBufferPos;
            _serialRxBufferPos = 0;
        }

        _lastMessageRead = false; // clear flag
    }
}
