#ifndef __RF125K_H
#define	__RF125K_H

#include "stm32f0xx.h"

#define  ANT_X_SWITCH_PIN   GPIO_Pin_1       
#define  ANT_X_SWITCH_OFF() GPIOB->BSRR = ANT_X_SWITCH_PIN;//�ߵ�ƽ�ر�
#define  ANT_X_SWITCH_ON()  GPIOB->BRR  = ANT_X_SWITCH_PIN;


#define  ANT_Y_SWITCH_PIN   GPIO_Pin_0
#define  ANT_Y_SWITCH_OFF() GPIOB->BSRR = ANT_Y_SWITCH_PIN;//�ߵ�ƽ�ر�
#define  ANT_Y_SWITCH_ON()  GPIOB->BRR =  ANT_Y_SWITCH_PIN;

typedef __packed struct
{
    uint8_t uchType;          /*��Ϣ����*/
    uint8_t uchLen;           /*��Ϣ����*/
    uint8_t uchRemoteAddr[3]; /*Զ�˶��������յ�ַ*/
    uint8_t uchSubAddr;       /*�������ӵ�ַ*/
    uint8_t uchFreq;          /*2.4GHz���� ������־*/
    uint8_t uchLrc;           /*lrc ����*/
	
}AIR_MSG_FRAME, *pST_MSG_FRAME;

extern volatile uint8_t 	g_uchPwm1OptFlg; //PWM1����:0�ر�1��2��ͣ
extern volatile uint8_t 	g_uchPwm2OptFlg; //PWM2����:0�ر�1��2��ͣ
extern uint8_t  g_uchIsTriggerLowSignal;

void vPwmIrq(void);
void vSig125kInit(void);
void vSig125kSndFrameTest(uint8_t uchIndex);
void vSig125kSndTask(void);

#endif /* __RF125K_H */
