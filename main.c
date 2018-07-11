#include <stm32f10x_conf.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// USART Receiver buffer
#define RX_BUFFER_SIZE 350
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_wr_index=0,rx_rd_index=0;
volatile uint16_t rx_counter=0;
volatile uint8_t rx_buffer_overflow=0;

// USART Transfer buffer
#define TX_BUFFER_SIZE 350
volatile uint8_t   tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t  tx_wr_index=0, //End index
                   tx_rd_index=0; //start index

int led_state=0;

uint16_t get_char(void);

void init_sequence(){
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    //GPIO_PinLockConfig(GPIO_Mode_AF_USART2 GPIOA, GPIO_PinSource2);
    //GPIO_PinLockConfig(GPIO_Mode_AF_USART2 GPIOA, GPIO_PinSource3);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // PD5 -> TX UART.
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &gpio);

    //PD6  -> RX UART.
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef USART;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART.USART_BaudRate = 9600;// Baud speed
    USART.USART_WordLength = USART_WordLength_8b; //8 bit of data
    USART.USART_StopBits = USART_StopBits_1; //one stop bit
    USART.USART_Parity = USART_Parity_No;
    USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       // allow  Rx & Tx
    USART_Init(USART1, &USART);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    NVIC_EnableIRQ(USART1_IRQn);
}






//1 delay tick = 4 mcu ticks
void delay_loop(uint32_t DelayTicks) {
	__asm__ __volatile__ (
			"1: \n"
			"sub %[DelayTicks], %[DelayTicks], #1\n" //1tick
			"cmp %[DelayTicks], #0 \n" // 1tick
			"bne 1b \n" //1 or 2 ticks
			: [DelayTicks] "+r"(DelayTicks)
	);
}

//send char
void Usart_sendchar(uint16_t x){
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1, x);
}

//Send any string by Chars
void Usart_string(char* x){
	int i;
	for (i=0; i<=strlen(x);i++){
		Usart_sendchar(x[i]);
	}
}

//Converting int to string and send
void Usart_int(unsigned int x){
	char str[20];
	itoa(x, str, 10);
	Usart_string(str);
}


void USART1_IRQHandler(void) {
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		      	if ((USART1->SR & (USART_FLAG_NE|USART_FLAG_FE|USART_FLAG_PE|USART_FLAG_ORE)) == 0) {
						rx_buffer[rx_wr_index]=(uint8_t)(USART_ReceiveData(USART1)& 0xFF);
						rx_wr_index=(rx_wr_index+1)%RX_BUFFER_SIZE;
					}
					else USART_ReceiveData(USART1);
		  }
	}

uint16_t get_char(void)
{
	uint16_t data;
	while (rx_rd_index==rx_wr_index);
	data = rx_buffer[rx_rd_index];
	rx_rd_index = (rx_rd_index+1)%RX_BUFFER_SIZE;
	Usart_sendchar(data);
	return data;
}

	//main
int main(void) {
	init_sequence();
        Usart_string("on to on, off to off \n\r");
        //Usart_string("Count: ");

       uint16_t count=0;
	while(1) {
		if (get_char()=='o') {
			if (get_char()=='n') led_state=0;
			else {
				if  (get_char()=='f')   led_state=1;
			}
		}
			//else{Usart_string("\n\rError\n\r");}
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_state ? Bit_SET : Bit_RESET);
	}
}
