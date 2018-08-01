#include "io.h"
#include "devcfg.h"
#include "nrf24l01.h"
#include "rf125k.h"
#include "uart.h"
#include <string.h>

volatile uint32_t ulSysTick = 0xFFFFFF00;

void OSTimeDly(uint32_t ulTick);
/***********************************************************************************
* @name   void LED_Init(void) : PB6->LED1, PC6->LED2， 低电平点亮
  * @brief  配置LED3引脚.
  * @param  None
  * @retval None
  */
void vLed_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
	//LED2
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_Level_3;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	//LED1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 ;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_6);	//开始时候灭LED1
	GPIO_SetBits(GPIOC, GPIO_Pin_14);	//开始时候灭LED1
}

/**
  * @name   void LED_control(void)
  * @brief  控制LED灯的亮灭.
  * @param  
			eBSP_LED LEDx   ; LED的枚举号， 其他号全部控制
			FlagStatus flag ：0灭灯，1亮灯
  * @retval None
  */
void LED_control(eBSP_LED LEDx, FlagStatus flag)
{
	switch(LEDx)
	{
		case 0x01:	//操作LED1
			(flag == 0) ?  GPIO_SetBits(GPIOB, GPIO_Pin_6) :  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
			break;

		case 0x02:	//操作LED2
			(flag == 0) ?  GPIO_SetBits(GPIOC, GPIO_Pin_14) :  GPIO_ResetBits(GPIOC, GPIO_Pin_14);
			break;
				
		default:	//其他值全部操作；
			//(flag == 0) ?  GPIO_SetBits(GPIOA, GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_10) :  GPIO_ResetBits(GPIOA, GPIO_Pin_12 | GPIO_Pin_11|GPIO_Pin_10);
			break;
	}
}

/**
  * @name   void LED_toggle(void)
  * @brief  控制LED反转.
  * @param  
			eBSP_LED LEDx   ; LED的枚举号， 其他号全部控制
  * @retval None
  */
void LED_toggle(eBSP_LED LEDx)
{
	switch(LEDx)
	{
		//GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12)));

		case 0x01:	//操作LED1
			GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6)));
			break;

		case 0x02:	//操作LED2
			GPIO_WriteBit(GPIOC, GPIO_Pin_14, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14)));
			break;
		
		default:	//其他值全部操作；
			GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6)));
			GPIO_WriteBit(GPIOC, GPIO_Pin_14, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14)));
			break;
	}

}
/**********************************************************************
**函数原型:   void Port_Init(void)
**入口参数:   无
**返 回 值:   无
**功    能:   外部端口初始化
**说    明:   
************************************************************************/
void Port_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;   
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

/**********************************************************************
**函数原型:   void vBeepDelay(unsigned int unDelay)
**入口参数:   延时时间
**返 回 值:   无
**功    能:   蜂鸣器响
**说    明:   
************************************************************************/
void vBeepDelay(unsigned int unDelay)
{
    BEEP_ON();
    OSTimeDly(unDelay);
    BEEP_OFF();
}

/**********************************************************************
**函数原型:   uint32_t ulGetCurTick(void) 
**入口参数:   无
**返 回 值:   无
**功    能:   获取当前时钟节拍
**说    明:   
************************************************************************/
static uint32_t ulGetCurTick(void)
{
    __disable_irq();
		uint32_t ulTick = ulSysTick;
    __enable_irq();
	
		return ulTick;
}

/**********************************************************************
**函数原型:   void CheckOverflowCurTick(uint32_t ulTick) 
**入口参数:   时钟节拍
**返 回 值:   无
**功    能:   复位时钟节拍
**说    明:   
************************************************************************/
static void CheckOverflowCurTick(uint32_t ulTick)
{
    if(ulTick >= 0xFFFFFF00)
		{
		    ulSysTick = 0;		
		}   
}

/**********************************************************************
**函数原型:   void OSTimeDly(uint32_t ulTick)
**入口参数:   延时时间
**返 回 值:   无
**功    能:   系统延时
**说    明:   
************************************************************************/
void OSTimeDly(uint32_t ulTick)
{
    uint64_t ulBeginTick;
    uint64_t ulCurTick = 0;

    ulBeginTick = ulGetCurTick();
    ulCurTick   = ulBeginTick;

    while(1)
    {
        if(nrf_rx_inf.rx_irq_flag)
		{
			nrf_rx_inf.rx_irq_flag = false;
			
			NRF_Rx_Dat(&nrf_rx_inf);
			
			UART_TTL_SendString((void*)&nrf_rx_inf.rx_data[0], nrf_rx_inf.rx_len);
			memset((void*)&nrf_rx_inf.rx_data[0], 0, nrf_rx_inf.rx_len);
			nrf_rx_inf.rx_len = 0;
		}
				
		if(CFG_DEV_TYPE_TRI_FREQ == uchDevCfgGetDevType())
		{
				vSig125kSndTask();
		}
				
		ulCurTick = ulGetCurTick();
        if(ulCurTick-ulBeginTick >= ulTick)
        {
            CheckOverflowCurTick(ulCurTick);
			break;
        }
    }
}




