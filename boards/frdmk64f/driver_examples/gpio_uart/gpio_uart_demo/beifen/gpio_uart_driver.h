#ifndef _GPIO_UART_DRIVER_H_
#define _GPIO_UART_DRIVER_H_

#include "fsl_common.h"
#include "fsl_pit.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*Board GPIO_UART mapping yangliang add*/
#define BOARD_GPIO_UART_GPIO_TX         GPIOB
#define BOARD_GPIO_UART_GPIO_TX_PIN     17U
#define BOARD_GPIO_UART_GPIO_RX         GPIOB
#define BOARD_GPIO_UART_GPIO_RX_PIN     16U
#define BOARD_GPIO_UART_PORT_RX         PORTB

enum 
{
   UART_IDLE = 0,
   UART_START,
   UART_TRANSFERRING,
   UART_PARITY,
   UART_STOP
};


typedef struct _gpio_uart_config
{
    GPIO_Type *tx_gpio_base;    //
    GPIO_Type *rx_gpio_base; 
    PORT_Type *rx_port_base;
    uint32_t tx_pin_num; 
    uint32_t rx_pin_num;
    
    uint8_t databit;   //����λ���ȣ�ͨ����8
    uint8_t stop;      //ֹͣλ����
    uint8_t parity;    //У��λ��0��ʾ��У�飬1��ʾ��У�飬2��ʾżУ��  
    pit_chnl_t pitch;    //ռ�ö�ʱ��ͨ���ţ�����ܳ���оƬPIT�ڲ�ͨ������
    uint32_t baudrate;
 
} gpio_uart_config_t;


void gpio_uart_init(const gpio_uart_config_t *config);
void gpio_uart_send(uint8_t *databuf, uint32_t num);

#endif