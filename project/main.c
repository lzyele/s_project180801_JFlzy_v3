#include "stm32f0xx.h"
#include "devcfg.h"
#include "nrf24l01.h"
#include "io.h"
#include "rf125k.h"
#include "rc522.h"
#include "timer.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

#define countof(a) (sizeof(a) / sizeof(*(a))) //计算数组内的成员个数
uint8_t Tx_Buffer[] = "Start!\n";

void System_Init(void)
{
		Port_Init();
		TIM3_Config();
		vLed_Init();
	
		UART_TTL_Init();
		RS485_Init();
		RC522_Init();
}

int main(void)
{ 
		SystemInit();
		System_Init();
		
		UART_TTL_SendString(Tx_Buffer, countof(Tx_Buffer)-1);
		//printf("-------------------\r\n");//lzy_a	
		//RS485_SendString(Tx_Buffer, countof(Tx_Buffer)-1);//lzy_m
			
		nNrfInit(NRF_PRIM_RX, NRF_RF_CH_1, NRF_TRAN_SPEED_1Mbps, NRF_TRAN_POWER_0dbm);
	
		//nDevCfgSetDefaultPara();
		nDevCfgLoadCfgParaFromFlash();

		vSig125kInit();
		
		//vBeepDelay(50);//lzym
	
		while(1)
		{
			//LED_toggle(LED_ALL);//lzya
			FD_Card();
//			printf("Card type:0x%x 0x%x\t",card_inf.type[1], card_inf.type[0]);
//			printf("Card NUM:0x%x 0x%x 0x%x 0x%x",card_inf.serialno[0], card_inf.serialno[1],card_inf.serialno[2],card_inf.serialno[3]);
//			printf("\r\n");
			OSTimeDly(300);
		}
}
