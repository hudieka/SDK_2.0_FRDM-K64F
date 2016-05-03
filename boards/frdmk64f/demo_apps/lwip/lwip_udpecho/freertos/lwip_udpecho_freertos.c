/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include "lwip/opt.h"

#if LWIP_NETCONN
/* Standard C Included Files */
#include <stdio.h>
/* lwip Included Files */
#include "lwip/udp.h"
#include "lwip/debug.h"
#include "netif/etharp.h"
#include "lwip/init.h"
#include "lwip/tcpip.h"
/* SDK Included Files */
#include "ethernetif.h"
#include "board.h"

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_ENET ENET

/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 1
#define configIP_ADDR3 102

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Default gateway address configuration */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 1
#define configGW_ADDR3 100

#define configPHY_ADDRESS 1

#ifndef UDPECHO_STACKSIZE
#define UDPECHO_STACKSIZE 3000
#endif

#ifndef UDPECHO_PRIORITY
#define UDPECHO_PRIORITY 3
#endif

#ifndef UDPECHO_DBG
#define UDPECHO_DBG LWIP_DBG_ON
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

struct netif fsl_netif0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void udpecho_thread(void *arg)
{
    static struct netconn *conn;
    static struct netbuf *buf;
    char buffer[100];
    err_t err;

    LWIP_UNUSED_ARG(arg);
    netif_set_up(&fsl_netif0);
    conn = netconn_new(NETCONN_UDP);
    LWIP_ASSERT("con != NULL", conn != NULL);
    netconn_bind(conn, NULL, 7);
    while (1)
    {
        err = netconn_recv(conn, &buf);
        if (err == ERR_OK)
        {
            if (netbuf_copy(buf, buffer, buf->p->tot_len) != buf->p->tot_len)
            {
                LWIP_DEBUGF(UDPECHO_DBG, ("netbuf_copy failed\r\n"));
            }
            else
            {
                buffer[buf->p->tot_len] = '\0';
                err = netconn_send(conn, buf);
                if (err != ERR_OK)
                {
                    LWIP_DEBUGF(UDPECHO_DBG, ("netconn_send failed: %d\r\n", (int)err));
                }
                else
                {
                    LWIP_DEBUGF(UDPECHO_DBG, ("got %s\r\n", buffer));
                }
            }
            netbuf_delete(buf);
        }
    }
}

void udp_echo_init(void)
{
    sys_thread_new("udpecho_thread", udpecho_thread, NULL, UDPECHO_STACKSIZE, UDPECHO_PRIORITY);
    vTaskStartScheduler();
}

/*!
 * @brief Main function
 */
int main(void)
{
    ip_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;

    MPU_Type *base = MPU;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Disable MPU. */
    base->CESR &= ~MPU_CESR_VLD_MASK;

    LWIP_DEBUGF(UDPECHO_DBG, ("TCP/IP initializing...\r\n"));
    tcpip_init(NULL, NULL);
    LWIP_DEBUGF(UDPECHO_DBG, ("TCP/IP initialized.\r\n"));

    IP4_ADDR(&fsl_netif0_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&fsl_netif0_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&fsl_netif0_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);
    netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&fsl_netif0);

    LWIP_PLATFORM_DIAG(("\r\n************************************************"));
    LWIP_PLATFORM_DIAG((" UDP Echo example"));
    LWIP_PLATFORM_DIAG(("************************************************"));
    LWIP_PLATFORM_DIAG((" IPv4 Address     : %u.%u.%u.%u", ((u8_t *)&fsl_netif0_ipaddr)[0],
                        ((u8_t *)&fsl_netif0_ipaddr)[1], ((u8_t *)&fsl_netif0_ipaddr)[2],
                        ((u8_t *)&fsl_netif0_ipaddr)[3]));
    LWIP_PLATFORM_DIAG((" IPv4 Subnet mask : %u.%u.%u.%u", ((u8_t *)&fsl_netif0_netmask)[0],
                        ((u8_t *)&fsl_netif0_netmask)[1], ((u8_t *)&fsl_netif0_netmask)[2],
                        ((u8_t *)&fsl_netif0_netmask)[3]));
    LWIP_PLATFORM_DIAG((" IPv4 Gateway     : %u.%u.%u.%u", ((u8_t *)&fsl_netif0_gw)[0], ((u8_t *)&fsl_netif0_gw)[1],
                        ((u8_t *)&fsl_netif0_gw)[2], ((u8_t *)&fsl_netif0_gw)[3]));
    LWIP_PLATFORM_DIAG(("************************************************"));

    udp_echo_init();
    /* should not reach this statement */
    for (;;)
        ;
}
#endif
