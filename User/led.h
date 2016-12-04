#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"
#define TX_LED 0
#define RX_LED 1

void LED_GPIO_Config(void);
void LED_STATE(u8 sel,u8 on);
#endif /* __LED_H */
