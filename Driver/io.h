#ifndef __IO_H
#define	__IO_H

#include "stm32f0xx.h"
#include <stdint.h>

typedef enum
{
	LED_ALL = 0,
	LED1 = 1,
	LED2 = 2
}eBSP_LED; //板上的LED 序号定义

//蜂咛器 PA15, 输出高电平响
#define BEEP_OFF() (GPIOA->BRR  = GPIO_Pin_15)
#define BEEP_ON()  (GPIOA->BSRR = GPIO_Pin_15)

extern volatile uint32_t ulSysTick;
extern volatile uint8_t  g_uchShortSysTick;

void Port_Init(void);
void vBeepDelay(unsigned int unDelay);
void OSTimeDly(uint32_t ulTick);

void vLed_Init(void);
void LED_control(eBSP_LED LEDx, FlagStatus flag);
void LED_toggle(eBSP_LED LEDx);

#endif /* __IO_H */

