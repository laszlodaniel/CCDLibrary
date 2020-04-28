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

#include <CCDLibrary.h>
#include <util/atomic.h>
#include <util/delay.h>

CCDLibrary CCD;

CCDLibrary::CCDLibrary()
{
    // Empty.
}

void CCDLibrary::begin(uint8_t busIdleBits)
{
    _busIdle = false;
    _busIdleBits = busIdleBits; // should be 10 according to CDP68HC68S1 datasheet...
    _lastMessageRead = true;
    _messageLength = 0;
    uartInit(CCDUBRR);
    busIdleTimerInit();
    busIdleTimerStart();
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
    uint8_t ret = 0; // 0: ok, 1: data collision, 2: timeout, 
    
    // Copy message to transmit buffer.
    for (uint8_t i = 0; i < bufferLength; i++) _serialTxBuffer[i] = buffer[i];
    
    _serialTxLength = bufferLength; // save message length

    bool timeout = false;
    uint32_t timeout_start = millis();
    while (!_busIdle && !timeout) // wait for bus idle condition or timeout (1 second)
    {
        if (millis() - timeout_start > 1000) timeout = true;
    }
    if (timeout) return 2;

    // First transmitted byte (ID-byte) is special, it is used to decide bus arbitration when multiple 
    // modules are trying to send a message at the same time. 
    // We need to inspect every bit of the first received byte in real time and block further bit transmission 
    // if data collision occurs. This can happen if another module is sending an ID-byte value lower 
    // than ours thus overwriting our 1 bits with its 0 bits. 
    // A module sending the lowest ID-byte will win the bus arbitration procedure, and others must block further 
    // data transmission. 
    // The hardware UART does not support bit-level manipulation of the data register 
    // so they are turned off temporarily.
    UCSR1B &= ~(1 << RXCIE1) & ~(1 << RXEN1) & ~(1 << TXEN1) & ~(1 << UDRIE1);

    // Setup RX/TX pins manually
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

    // Start big-banging RX/TX pins.
    // Write start bit (0 bit).
    cbi(TX_PORT, TX_P);
    _delay_us(64.0); // wait 0.5 bit time (at 7812.5 baud it's 64 microseconds)
    currentRxBit = (RX_PIN & (1 << RX_P)); // read RX pin state (logic high or low)
    if (currentRxBit) error = true; // it's supposed to be logic low
    _delay_us(64.0); // wait another 0.5 bit time to finish start bit signaling

    // Write 8 data bits beginning with the leftmost bit
	for (uint8_t i = 0; i < 8; i++)
    {
        if (!error) // if bus arbitration is lost, don't write anything, just read bits
        {
            currentTxBit = IDbyteTX & (1 << i);
            if (currentTxBit) sbi(TX_PORT, TX_P);
            else cbi(TX_PORT, TX_P);
        }
        _delay_us(64.0); // wait 0.5 bit time
        currentRxBit = (RX_PIN & (1 << RX_P)); // read RX pin state
        if (currentRxBit) sbi(IDbyteRX, i); // save bit
        if (currentRxBit != currentTxBit) error = true;// error: bit mismatch, bus arbitration lost
        _delay_us(64.0); // wait another 0.5 bit time to finish this bit
    }
    
    // Write stop bit (1 bit)
    sbi(TX_PORT, TX_P); // write stop bit at TX pin (1 bit)
    _delay_us(64.0); // wait 0.5 bit time (at 7812.5 baud it's 64 microseconds)
    currentRxBit = (RX_PIN & (1 << RX_P)); // read RX pin state (logic high or low)
    if (!currentRxBit) error = true; // error: start bit is measured logic low, it's supposed to be logic high
    _delay_us(64.0); // wait another 0.5 bit time to finish stop bit signaling

    _serialRxBuffer[0] = IDbyteRX;
    _serialRxBufferPos = 1;

    if (!error && (IDbyteRX == IDbyteTX))
    {
        // If every bit in the ID-byte is echoed back correctly then we won bus arbitration
        // and we can continue to send the rest of the message bytes.
        // Save the ID-byte manually in the serial receive buffer.
        _serialRxBuffer[0] = IDbyteRX;
        _serialRxBufferPos = 1;
        
        // Continue automatic transmission at the second byte.
        _serialTxBufferPos = 1;
        
        // Re-enable UART receiver and transmitter and receive complete interrupt.
        UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
        
        // Enable UDRE interrupt to continue message transmission.
        UCSR1B |= (1 << UDRIE1);

        ret = 0;
    }
    else
    {
        // Bus arbitration is lost somewhere along the way, this is an unexpected ID-byte. 
        // Save it and continue receiving message bytes.
        _serialRxBuffer[0] = IDbyteRX;
        _serialRxBufferPos = 1;

        // Reset transmit buffer.
        _serialTxBufferPos = 0;
        
        // Re-enable UART receiver and transmitter and receive complete interrupt.
        UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

        ret = 1;
    }

    busIdleTimerStart(); // start bus idle timer
    return ret;
}

ISR(TIMER3_COMPA_vect)
{
    CCD.handle_TIMER3_COMPA_vect();
}

void CCDLibrary::handle_TIMER3_COMPA_vect()
{
    // Stop bus idle timer.
    busIdleTimerStop();

    // Set flag.
    _busIdle = true;
    
    // Check if there is something in the buffer.
    if (_serialRxBufferPos > 0)
    {
        _messageLength = _serialRxBufferPos;
        _serialRxBufferPos = 0;
        
        // Copy bytes from serial receive buffer to message buffer.
        for (uint8_t i = 0; i < _messageLength; i++) _message[i] = _serialRxBuffer[i];

        _lastMessageRead = false; // clear flag
    }
}

ISR(USART1_RX_vect)
{
    CCD.handle_USART1_RX_vect();
}

void CCDLibrary::handle_USART1_RX_vect()
{
    // Stop bus idle timer.
    busIdleTimerStop();

    // Clear bus idle flag.
    _busIdle = false; // TODO: clear flag after first bit received, not after a whole byte is received

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

    // Start bus idle timer.
    busIdleTimerStart();
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
        _serialTxBufferPos = 0; // reset Tx buffer
        _serialTxLength = 0; // reset length
    }
}

void CCDLibrary::uartInit(uint16_t ubrr)
{
    // Reset buffer.
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        _serialRxBufferPos = 0;
        _serialTxBufferPos = 0;
        _serialTxLength = 0;
        _lastSerialError = 0;
    }

    // Set baud rate.
    UBRR1H = (ubrr >> 8) & 0x0F;
    UBRR1L = ubrr & 0xFF;
    
    // Enable UART receiver and transmitter and receive complete interrupt.
    UCSR1B |= (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
    
    // Set frame format: asynchronous, 8 data bit, no parity, 1 stop bit.
    UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);
}

void CCDLibrary::busIdleTimerInit()
{
    // Calculate top value to count.
    // OCR3A = ((F_CPU * (1 / BAUDRATE) * BIT_DELAY) / PRESCALER) - 1
    //    F_CPU = 16000000 Hz
    //    BAUDRATE = 7812.5 bits per second
    //    PRESCALER = 1024
    // OCR3A (10 bits delay) = ((16000000 * (1 / 7812.5) * 10) / 1024) - 1 = 19
    // OCR3A (11 bits delay) = ((16000000 * (1 / 7812.5) * 10) / 1024) - 1 = 21
    // OCR3A (12 bits delay) = ((16000000 * (1 / 7812.5) * 10) / 1024) - 1 = 23
    // OCR3A (13 bits delay) = ((16000000 * (1 / 7812.5) * 13) / 1024) - 1 = 25
    // OCR3A (14 bits delay) = ((16000000 * (1 / 7812.5) * 10) / 1024) - 1 = 27
    _calculatedOCRAValue = (uint8_t)((((float)F_CPU * (1.0 / 7812.5) * (float)_busIdleBits) / 1024.0) - 1.0);
    
    // Setup Timer 3 to do idle timing measurements.
    noInterrupts();
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;
    OCR3A = 0;
    TIMSK3 |= (1 << OCIE3A); // output Compare Match A Interrupt Enable
    interrupts();
}

void CCDLibrary::busIdleTimerStart()
{
    TCCR3B |= (1 << WGM32); // CTC
    TCCR3B |= (1 << CS32) | (1 << CS30); // prescaler: 1024
    TCNT3 = 0;
    OCR3A = _calculatedOCRAValue;
}

void CCDLibrary::busIdleTimerStop()
{
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;
    OCR3A = 0;
}