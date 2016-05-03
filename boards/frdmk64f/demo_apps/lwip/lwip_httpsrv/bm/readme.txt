Overview
========

The lwip_httpsrv demo application demonstrates an HTTPServer on the lwIP TCP/IP stack with bare metal SDK or FreeRTOS.
The user uses an Internet browser to send a request for connection. The board acts as an HTTP server and sends a Web
page back to the PC.

Supported Hardware

These Freescale Freedom development platforms and Tower System modules are supported by the KSDK lwip_httpsrv demo.

- FRDM-K64F
- FRDM-K66F
- TWR-K60D100M
- TWR-K64F120M
- TWR-K65F180M
- TWR-KV58F220M

Note, The RMII mode is used for default setting to initialize the ENET interface between MAC and the external PHY.
You can change it to MII mode as you wish. Please make sure the MII Mode setting in the MAC is synchronized to the
setting in TWR-SERIAL board for the external PHY.

Toolchain supported
===================
- IAR embedded Workbench 7.50.1
- Keil MDK 5.17
- GCC ARM Embedded 2015-4.9-q3
- Kinetis Development Studio IDE 3.0.0
- Atollic TrueSTUDIO 5.4.0

Hardware requirements
=====================
- Mini/micro USB cable
- RJ45 Network cable
- FRDM-K64F board
- Personal Computer

Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect a USB cable between the PC host and the OpenSDA USB port on the target board.
2.  Open a serial terminal on PC for OpenSDA serial device with these settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Insert the Ethernet Cable into the target board's RJ45 port and connect it to your PC network adapter.
4.  Configure the host PC IP address to 192.168.1.100.
5.  Open a web browser.
6.  Download the program to the target board.
7.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
1.  On the browser address bar, type 192.168.1.102(ip address of the board).
    The browser should respond as shown below
    lwIP - A Lightweight TCP/IP stack
    The web page you are watching was served by a simple web server running on top of the lightweight TCP/IP stack lwIP.
    lwIP is an open source implementation of the TCP/IP protocol suite that was originally written by Adam Dunkels of
    the Swedish Institute of Computer Science but now is being actively developed by a team of developers distributed
    world-wide.Since it's release, lwIP has spurred a lot of interest and has been ported to several platforms and
    operating systems. lwIP can be used either with or without an underlying OS.
    The focus of the lwIP TCP/IP implementation is to reduce the RAM usage while still having a full scale TCP. This
    makes lwIP suitable for use in embedded systems with tens of kilobytes of free RAM and room for around 40 kilobytes
    of code ROM.
    More information about lwIP can be found at the lwIP homepage at http://savannah.nongnu.org/projects/lwip/ or at
    the lwIP wiki at http://lwip.wikia.com/.
    Led control
    Button status
2.  Click the led control, the led control page should be displayed
    LED Control
    LED_TOGGLE
    Click LED_TOGGLE button, the led red will be turned on and off(or with a short delay).
3.  Click the button status, the button status page will be displayed
    Polling Example
    BUTTON STATUS:
    UN-PRESSED
    Press the SW3 on the board, buttons status should be updated, corresponding to the status of SW3(or with a short
    delay).

Customization options
=====================


