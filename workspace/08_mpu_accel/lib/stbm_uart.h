/** charset=UTF-8 **/#ifndef __STBM_UART_H__#define __STBM_UART_H__#include "stm32f10x.h"#include "stm32f10x_conf.h"void xbee_init();//void xbee_send_data(int8_t data);void xbee_send_string(const char String[]);void xbee_dma_configuration(uint32_t Memory_Address, uint16_t Buffer_Size);void DMA1_Channel7_IRQHandler(void);void xbee_nvic_configuration(void);#endif /* __STBM_UART_H__ */