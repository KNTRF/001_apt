/** charset=UTF-8 **/#ifndef __STBM_UART_H__#define __STBM_UART_H__#include "stm32f10x.h"#include "stm32f10x_conf.h"void xbee_init();void xbee_send_data(int8_t data);#endif /* __STBM_UART_H__ */