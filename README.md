# CCDLibrary
Arduino library for interfacing Chrysler's legacy CCD-bus using the [CCDBusTransceiver](https://github.com/laszlodaniel/CCDBusTransceiver) or [CCDPCIBusTransceiver](https://github.com/laszlodaniel/CCDPCIBusTransceiver) development board.

This library is compatible with the CDP68HC68S1 CCD-bus transceiver IC and the [ChryslerCCDSCIScanner](https://github.com/laszlodaniel/ChryslerCCDSCIScanner) hardware. The TBEN pin definition must be changed from 4 to 13 in order for the CCD/SCI GUI to work properly.

Arduino Mega / ATmega2560 microcontroller is required!

Example using the CDP68HC68S1 CCD-bus transceiver IC:

![CDP68HC68S1_schematic_01.png](https://chryslerccdsci.files.wordpress.com/2020/05/cdp68hc68s1_schematic_01.png)

![IMG_20200510_152453_02.jpg](https://chryslerccdsci.files.wordpress.com/2020/05/img_20200510_152453_02.jpg)

![CCDLibrary_CCDWrite_example_02.png](https://chryslerccdsci.files.wordpress.com/2020/05/ccdlibrary_ccdwrite_example_02.png)

Example using the CCDBusTransceiver development board:

![IMG_20200630_140622_02.jpg](https://chryslerccdsci.files.wordpress.com/2020/06/img_20200630_140622_02.jpg)

![IMG_20200701_155010_02.jpg](https://chryslerccdsci.files.wordpress.com/2020/07/img_20200701_155010_02.jpg)