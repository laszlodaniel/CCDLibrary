# CCDLibrary
Arduino library for interfacing Chrysler's legacy CCD-bus using the [CCDBusTransceiver](https://github.com/laszlodaniel/CCDBusTransceiver) or [CCDPCIBusTransceiver](https://github.com/laszlodaniel/CCDPCIBusTransceiver) development board.

This library is compatible with the CDP68HC68S1 CCD-bus transceiver IC and the [ChryslerCCDSCIScanner](https://github.com/laszlodaniel/ChryslerCCDSCIScanner) hardware. The TBEN pin definition must be changed from 4 to 13 in order for the CCD/SCI GUI to work properly.

Arduino Mega / ATmega2560 microcontroller is required!

Pinout is fixed for the time being:
- UART1-channel (RXD1/TXD1) for data transfer,
- D3 (INT5) for bus-idle detection.

| PCB      | Arduino Mega | OBD2 connector  |
|----------|--------------|-----------------|
| J1 GND   | GND          |                 |
| J1 TX    | RXD1 + D3    |                 |
| J1 RX    | TXD1         |                 |
| J1 +5V   | +5V          |                 |
