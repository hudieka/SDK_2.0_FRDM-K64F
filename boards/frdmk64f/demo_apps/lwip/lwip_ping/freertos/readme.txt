Overview
========

The lwip_ping demo application demonstrates a Ping Demo on the lwIP TCP/IP stack which using the ICMP protocol. The
application periodically sends the ICMP echo request to a PC and processes the PC reply. Type the "ping $board_address"
in the PC command window to send an ICMP echo request to the board. The lwIP stack sends the ICMP echo reply back to the
PC.

Supported Hardware

These Freescale Freedom development platforms and Tower System modules are supported by KSDK lwip_ping demo.

- FRDM-K64F
- FRDM-K66F
- TWR-K60D100M
- TWR-K64F120M
- TWR-K65F180M
- TWR-KV58F220M

Note, The RMII mode is used for default setting to initialize the ENET interface between MAC and the external PHY. If you want,
it can be changed to MII mode as well. Ensure the MII Mode setting in the MAC is synchronized to the setting
in TWR-SERIAL board for the external PHY.

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
ping: send
192.168.1.100
ping: recv
192.168.1.100
 3 ms
ping: send
192.168.1.100
ping: recv
192.168.1.100
 3 ms

Customization options
=====================


