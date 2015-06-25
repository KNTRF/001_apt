/** charset=UTF-8 **/#include "stbm_uart.hpp"// コンストラクタSTBM_UART::STBM_UART(uint8_t port, uint32_t baudrate):	_port(port),	_baudrate(baudrate){	_port = port;		switch(_port){		case 2:			_usart_tx_pin = GPIO_Pin_2;			_usart_rx_pin = GPIO_Pin_3;						break;			}}void STBM_UART::init(){	GPIO_InitTypeDef GPIO_InitStructure;		switch(_port){		case 2:			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);			break;	}		/* Configure USART Tx as alternate function push-pull */	GPIO_InitStructure.GPIO_Pin = _usart_tx_pin;	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	GPIO_Init(GPIOA, &GPIO_InitStructure);		/* Configure USART Rx as input floating */	GPIO_InitStructure.GPIO_Pin = _usart_rx_pin;	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	GPIO_Init(GPIOA, &GPIO_InitStructure);		/* USART configuration ------------------------------------------------------*/	USART_InitTypeDef USART_InitStructure;	USART_InitStructure.USART_BaudRate = _baudrate;	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	USART_InitStructure.USART_StopBits = USART_StopBits_1;	USART_InitStructure.USART_Parity = USART_Parity_No ;	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	USART_Init(USART2, &USART_InitStructure);		/* Enable the USART */	USART_Cmd(USART2, ENABLE);}void STBM_UART::send_data(int8_t data){	USART_SendData(USART2, data);}