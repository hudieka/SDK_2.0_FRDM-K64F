#include "gpio_uart_driver.h"


//Can be porting if you need
#define BOARD_GPIO_UART_RXPIN_HANDLER  PORTB_IRQHandler
#define BOARD_GPIO_UART_TIMER_HANDLER  PIT0_IRQHandler
#define GPIO_UART_RX_IRQ               PORTB_IRQn
#define PIT_IRQ_ID                     PIT0_IRQn
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
//Porting end

#define GPIO_UART_PARITY_MODE_NONE  0
#define GPIO_UART_PARITY_MODE_ODD   1
#define GPIO_UART_PARITY_MODE_EVEN  2

#define UART_DISABLE -1


//参数
gpio_uart_config_t g_gpio_uart_config;


//接收相关变量
static int8_t recv_state= UART_DISABLE;
static uint8_t bit_recv = 0;
static uint8_t bit_recv_index = 0;
static uint8_t data_recv = 0;
static uint8_t timer_cnt = 0;
static int8_t  parity_r = 0;
extern volatile bool g_GpioUartGet;
extern uint8_t g_ucRxdata;

//发送相关变量
static int8_t  send_state = UART_DISABLE;
static uint8_t  bit_send = 0;
static uint8_t  bit_send_index = 0;
static uint8_t  *send_data = NULL;
static int8_t   parity_t = 0;
static uint32_t send_data_cnt;


static void gpio_uart_tx_clear();
static void gpio_uart_rx_clear();


void BOARD_GPIO_UART_RXPIN_HANDLER(void)
{
  //clear int
  GPIO_ClearPinsInterruptFlags(g_gpio_uart_config.rx_gpio_base, 1U << g_gpio_uart_config.rx_pin_num);
  //disable int
  PORT_SetPinInterruptConfig(g_gpio_uart_config.rx_port_base, g_gpio_uart_config.rx_pin_num, kPORT_InterruptOrDMADisabled); 
  //start recv
  recv_state = UART_IDLE;
}

void BOARD_GPIO_UART_TIMER_HANDLER(void)
{
  PIT_ClearStatusFlags(PIT, g_gpio_uart_config.pitch , PIT_TFLG_TIF_MASK);
  
  //2倍波特率采样，保证中间点采样
  timer_cnt ++;
  //奇数点采样接收，偶数点发送
  if( timer_cnt % 2 ==1)
  {
    //Recv state machine
    switch(recv_state)
    {
    case UART_TRANSFERRING:    
      //Get one bit by checking the RX GPIO input level
      bit_recv = GPIO_ReadPinInput(g_gpio_uart_config.rx_gpio_base, g_gpio_uart_config.rx_pin_num);
      data_recv |= (bit_recv << bit_recv_index);
      //parity_r ^= bit_recv;
      //yangliang change
      parity_r = (bit_recv + parity_r)&0x01;
      bit_recv_index ++;
      
      if(bit_recv_index == 8)
      {
        if(g_gpio_uart_config.parity == GPIO_UART_PARITY_MODE_NONE)
        {
            recv_state = UART_STOP;
        }
        else
        {
            recv_state = UART_PARITY;
        }

      }
      break;
      
    case UART_PARITY:
      bit_recv   = GPIO_ReadPinInput(g_gpio_uart_config.rx_gpio_base, g_gpio_uart_config.rx_pin_num);
      //parity_r  ^= bit_recv;
      //yangliang change
      parity_r = (bit_recv + parity_r)&0x01;
      
      if(parity_r == 1)
      {
          parity_r = 0;
      }
      
      if(data_recv != 0x03)
      {
          parity_r = 0;
      }
      
      recv_state = UART_STOP;
      break;
      
    case UART_STOP:
      
      //timer_cnt = 0 ;
      
      // discard the character if parity check fail.
      if (g_gpio_uart_config.parity == GPIO_UART_PARITY_MODE_ODD)
      {
        if(parity_r == 1)
        {
            data_recv = 0;
        }
      }
      else if(g_gpio_uart_config.parity == GPIO_UART_PARITY_MODE_EVEN)
      {
        if(parity_r == 0)
        {
            data_recv = 0;
        }
      }
      g_ucRxdata = data_recv;
      g_GpioUartGet = true;
      
      gpio_uart_rx_clear();
      /* Enable RXD gpio interrupt */
      GPIO_ClearPinsInterruptFlags(g_gpio_uart_config.rx_gpio_base, 1U << g_gpio_uart_config.rx_pin_num);
      PORT_SetPinInterruptConfig(g_gpio_uart_config.rx_port_base, g_gpio_uart_config.rx_pin_num, kPORT_InterruptFallingEdge);
      break;
      
    case UART_IDLE:
      /* Reset recieved data */
      data_recv      = 0;
      bit_recv_index = 0;
      parity_r       = 0;
      recv_state = UART_TRANSFERRING;
      break;
      
    default:
      break;
    }        
  }
  //发送
  else
  {
    switch(send_state)
    {
    case UART_START:
      //Put GPIO to logic 0 to indicate the start
      GPIO_ClearPinsOutput(g_gpio_uart_config.tx_gpio_base, 1U << g_gpio_uart_config.tx_pin_num);
      bit_send_index = 0;
      parity_t   = 0;
      send_state = UART_TRANSFERRING;
      break;
      
    case UART_TRANSFERRING:
      bit_send = (*send_data >> bit_send_index) & 0x1;
      //Put GPIO to the logic level according to the bit value. 0 for low, and 1 for high;
      if(bit_send)
        GPIO_SetPinsOutput(g_gpio_uart_config.tx_gpio_base, 1U << g_gpio_uart_config.tx_pin_num);
      else
        GPIO_ClearPinsOutput(g_gpio_uart_config.tx_gpio_base, 1U << g_gpio_uart_config.tx_pin_num);
      
      parity_t ^= bit_send;
      bit_send_index++;
      if( bit_send_index == 8)
      {
        if(g_gpio_uart_config.parity == GPIO_UART_PARITY_MODE_NONE)
        {
          send_state = UART_STOP;
        }
        else
        {
          send_state = UART_PARITY;
        }
      }
      break;
      
    case UART_PARITY:
      if(g_gpio_uart_config.parity == GPIO_UART_PARITY_MODE_ODD)
        bit_send = (~parity_t)&1; 
      else
        bit_send = parity_t; // even
      
      if(bit_send)
        GPIO_SetPinsOutput(g_gpio_uart_config.tx_gpio_base, 1U << g_gpio_uart_config.tx_pin_num);
      else
        GPIO_ClearPinsOutput(g_gpio_uart_config.tx_gpio_base, 1U << g_gpio_uart_config.tx_pin_num);
      send_state = UART_STOP;
      break;
      
    case UART_STOP:
      //Put GPIO to logic 1 to stop the transferring
      GPIO_SetPinsOutput(g_gpio_uart_config.tx_gpio_base, 1U << g_gpio_uart_config.tx_pin_num);
      send_data_cnt--;
      
      if(send_data_cnt)
      {    /* Continue to send the next data */
        send_state = UART_START ;
        send_data++;
      }
      else
      {   /* Finish data sending */
        send_state = UART_DISABLE;
      }
      break;
      
    case UART_IDLE:
      send_state = UART_START;
      break;
      
    default:
      break;
    }  
    
  }
  
  
  
  
  
}


void gpio_uart_init(const gpio_uart_config_t *config)
{
  /* Structure of initialize PIT */
  pit_config_t pitConfig;
  
  g_gpio_uart_config = *config;
  
  /* Define the init structure for the output GPIO_UART_TX pin */
  gpio_pin_config_t gpio_uart_tx_config = {
    kGPIO_DigitalOutput, 1,
  };
  
  /* Define the init structure for the output GPIO_UART_TX pin */
  gpio_pin_config_t gpio_uart_rx_config = {
    kGPIO_DigitalInput, 0,
  };
  
  /*Init UART_GPIO*/
  GPIO_PinInit(config->tx_gpio_base, config->tx_pin_num, &gpio_uart_tx_config);                               //TX
  GPIO_PinInit(config->rx_gpio_base, config->rx_pin_num, &gpio_uart_rx_config);                               //RX
  PORT_SetPinInterruptConfig(config->rx_port_base, config->rx_pin_num, kPORT_InterruptFallingEdge);           //下降沿做起始位判断
  EnableIRQ(GPIO_UART_RX_IRQ);
  
  /*
  * pitConfig.enableRunInDebug = false;
  */
  
  PIT_GetDefaultConfig(&pitConfig);
  
  /* Init pit module */
  PIT_Init(PIT, &pitConfig);
  
  /* Set timer period for channel pitch_sel */
  PIT_SetTimerPeriod(PIT, config->pitch, USEC_TO_COUNT((1000000U/config->baudrate)/2, PIT_SOURCE_CLOCK));
  
  /* Enable timer interrupts for channel pitch_sel */
  PIT_EnableInterrupts(PIT, config->pitch, kPIT_TimerInterruptEnable);
  
  /* Enable at the NVIC */
  EnableIRQ(PIT_IRQ_ID);
  
  PIT_StartTimer(PIT, config->pitch);
  
}

static void gpio_uart_tx_clear()
{
  send_state = UART_DISABLE;
  bit_send = 0;
  bit_send_index = 0;
  send_data = NULL;
  parity_t = 0;
}

static void gpio_uart_rx_clear()
{
  recv_state= UART_DISABLE;
  bit_recv = 0;
  bit_recv_index = 0;
  data_recv = 0;
  timer_cnt = 0;
  parity_r = 0;
}

void gpio_uart_send(uint8_t *databuf, uint32_t num)
{
  gpio_uart_tx_clear();
  
  send_data = databuf;
  send_data_cnt = num;
  //start to send
  send_state = UART_IDLE;
  
}
