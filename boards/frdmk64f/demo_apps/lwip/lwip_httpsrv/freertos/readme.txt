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

Note that the RMII mode is used as the default setting to initialize the ENET interface between MAC and the external PHY.
However, it can be changed to the MII mode as well. Ensure the MII Mode setting in the MAC is synchronized to the
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
When the demo runs, the log would be seen on the OpenSDA terminal like:
TCP/IP initializing...
TCP/IP initialized.
On the browser address bar, type 192.168.1.102(IP address of the board).
The browser should respond as below
Welcome to our lwIP HTTP server!
This is a small test page, served by httpserver-netconn.

Customization options
=====================


