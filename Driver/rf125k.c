#include "rf125k.h"
#include "devCfg.h"
#include "timer.h"
#include "uart.h"
#include <string.h>
#include <stdbool.h>

static uint32_t gBitDuration; /*bit ����ʱ��*/
static uint32_t	gCarrierBurstDuration; 
static uint16_t gNrPreambleSymbols;
static uint8_t 	gManchesterPatternLength;
static uint8_t 	gCorrelatorDoublePattern;
static uint32_t gPattern;

static void vPwmsendCarrierAnt1 (uint32_t unCycles);
static void vPwmsendCarrierAnt2 (uint32_t unCycles);
static void vPwmPauseCarrierAnt1(uint32_t unCycles);
static void vPwmPauseCarrierAnt2(uint32_t unCycles);

/**********************************************************************
**����ԭ��:   void vXYMOS_PIN_Init(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   XYMOS�����ų�ʼ��
**˵    ��:   
************************************************************************/
static void vXYMOS_PIN_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;   
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
		GPIO_Init(GPIOB,&GPIO_InitStructure);

		GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
}

/**********************************************************************
**����ԭ��:   void vSig125kSetDefaultValues(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   125KHz�����ź�Ĭ�ϲ�������
**˵    ��:   
************************************************************************/
static void vSig125kSetDefaultValues(void)
{
    gBitDuration 		  			 = 48 /*23*3*/; /*default value in AS3933 (12 in register 7[4:0] 46*/ //384us
    gCarrierBurstDuration 	 = 125*2;			  /*duration of the carrier burst (250 == 2 ms carrier burst  125==1ms)*/ 
    gNrPreambleSymbols       = 6; 					/*Number of Preamble Symbols*/
    gManchesterPatternLength = 16;	  			/*16 bit*/
    gCorrelatorDoublePattern = 0; 					/*0 = single pattern, 1 = double pattern  0100 1001 1111 1111*/

#if 1
		gPattern = 0x5AA55555;
#else
		gPattern = 0x55555555;
#endif 
}

/**********************************************************************
**����ԭ��:   void vSig125kInit(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   125KHz�����źų�ʼ��
**˵    ��:   
************************************************************************/
void vSig125kInit(void)
{    	
		uint16_t carrier_pwm_duty = 0;
	
		vXYMOS_PIN_Init();
		
		carrier_pwm_duty = (uint16_t)uchDevCfgGetTriggerDuty(); //��ʼֵΪ50
		//printf(" carrier_pwm_duty = %d , \r\n", carrier_pwm_duty);
		
		TIM1_Config(carrier_pwm_duty);
		vSig125kSetDefaultValues();
}

/**********************************************************************
**����ԭ��:   void sendCarrier(uint8_t uchIndex, uint32_t unCycles)
**��ڲ���:   ����ͨ��;����ʱ��
**�� �� ֵ:   ��
**��    ��:   ����PWM�ز�
**˵    ��:   
************************************************************************/
static void sendCarrier(uint8_t uchIndex, uint32_t unCycles)
{
    if(1 == uchIndex)
    {
        vPwmsendCarrierAnt1(unCycles);
    }
    else
    {
        vPwmsendCarrierAnt2(unCycles);
    }
}

/**********************************************************************
**����ԭ��:   void pauseCarrier(uint8_t uchIndex, uint32_t unCycles)
**��ڲ���:   ����ͨ��;��ͣʱ��
**�� �� ֵ:   ��
**��    ��:   ��ͣPWM�ز�
**˵    ��:   
************************************************************************/
static void pauseCarrier(uint8_t uchIndex, uint32_t unCycles)
{
    if(1 == uchIndex)
    {
        vPwmPauseCarrierAnt1(unCycles);
    }
    else
    {
				vPwmPauseCarrierAnt2(unCycles);
    }
}

/**********************************************************************
**����ԭ��:   static void sendPattern(uint8_t uchIndex)
**��ڲ���:   ����ͨ��
**�� �� ֵ:   ��
**��    ��:   125KHz�������ݸ�ʽ
**˵    ��:   ��32ҳ ���͸�ʽ������ݣ�����˫��Լ�������ݣ�16-bit pattern��32-bit pattern
************************************************************************/
static void sendPattern(uint8_t uchIndex)
{
		uint32_t cIndex  = 0, index = 0, pattern = 0, bitDuartionDouble = 0;
		uint8_t  nrPreambleSymbols = 0, repeatPattern = 0;

		bitDuartionDouble = gBitDuration << 1;
		nrPreambleSymbols = gNrPreambleSymbols;
		pattern = gPattern;
		repeatPattern = gCorrelatorDoublePattern;

		if(gManchesterPatternLength == 16)
		{
				cIndex = 0x8000;
				pattern >>= 16;
		}
		else
		{
				cIndex = 0x80000000;
		}
		index = cIndex;

		sendCarrier(uchIndex,  gCarrierBurstDuration); //carrier burst
		pauseCarrier(uchIndex, gBitDuration);          //0, separation bit

		while(nrPreambleSymbols--)
		{
				sendCarrier(uchIndex,  gBitDuration);   	 //1, preamble
				pauseCarrier(uchIndex, gBitDuration);   	 //0, preamble
		}

		// pattern 9669 (HEX) = 1001 0110 0110 1001
		// loop optimized for "real" manchester coding (only two times the same bit value at once)
		do
		{
				while(index)
				{
						if(pattern & index)
						{	
								if(pattern & (index >> 1))
								{
										sendCarrier(uchIndex, bitDuartionDouble);
										index >>= 2;
								}
								else
								{
										sendCarrier(uchIndex, gBitDuration);
										index >>= 1;
								}
						}
						else
						{
								if(pattern & (index >> 1))
								{
										pauseCarrier(uchIndex, gBitDuration);
										index >>= 1;
								}
								else
								{
										pauseCarrier(uchIndex, bitDuartionDouble);
										index >>= 2;
								}
						}
				}
				
				index = cIndex;
				
		}while(repeatPattern--); //repeat the pattern
}

/**********************************************************************
**����ԭ��:   char chSig125ksendDataString(uint8_t uchIndex, uint8_t *puchSndData, uint8_t uchDataLen)
**��ڲ���:   ����ͨ��;��������;���ݳ���
**�� �� ֵ:   �����Ƿ񳬳�
**��    ��:  	125KHz�������� 
**˵    ��:   
************************************************************************/
char chSig125ksendDataString(uint8_t uchIndex, uint8_t *puchSndData, uint8_t uchDataLen)
{
    uint16_t usMancherCode 		= 0;
    uint8_t  uchMacherCodeData[30] = {0x00, 0x00, 0x00, 0x00};
    uint8_t  uchBitTotal  		= 0;
    uint8_t  uchCurByteIndex  = 0;
    uint8_t  uchCurBitIndex   = 0;
    uint8_t  uchNextByteIndex = 0;
    uint8_t  uchNextBitIndex  = 0;
    
    uint8_t  uchCurSendBit 		= 0;
    uint8_t  i = 0;
    uint8_t  j = 0;

    if(uchDataLen > 15)
    {
        return (char) -1;
    }

    /*����������ת��Ϊ Manchester ����*/
    for(i=0; i<uchDataLen; i++)
    {   
        for(j=0; j<8; j++) //���ѭ������һ��Byte
        {
            if((puchSndData[i] & (0x80>>j))) //1->10
            {
                usMancherCode <<= 1;
                usMancherCode |=  1;
                usMancherCode <<= 1;
            }
            else //0->01
            {
                usMancherCode <<= 1;
                usMancherCode <<= 1;
                usMancherCode  |= 1;
            }
        }
		
        uchMacherCodeData[i<<1] = (uint8_t) ((usMancherCode & 0xff00) >> 8);
        uchMacherCodeData[(i<<1)+ 1] = (uint8_t) (usMancherCode & 0x00ff);
    }   
    
    uchBitTotal = (uchDataLen<<4); //*16, һ��Byte ��16��Macher λ
    
    
    sendPattern(uchIndex);
    
    while(uchCurSendBit<uchBitTotal) //�����������ݵ�Macherλ
    {
        uchCurByteIndex = (uchCurSendBit >>3); //�൱��/8��Macherλ���ֽ�
        uchCurBitIndex  = (uchCurSendBit &0x07);
        
        uchCurSendBit++;
        uchNextByteIndex = (uchCurSendBit >>3);
        uchNextBitIndex  = (uchCurSendBit &0x07);
        
        if((uchMacherCodeData[uchCurByteIndex] & (0x80>> uchCurBitIndex)))
        {
            if((uchMacherCodeData[uchNextByteIndex] & (0x80 >> uchNextBitIndex)))//11 ��Carrier Burst ��AS3933��39ҳ
            {
              	sendCarrier(uchIndex, (gBitDuration <<1)); //lzy?
                uchCurSendBit++;
            }
            else//10 symbol Ϊ1��3933��29ҳ 
            {
                sendCarrier(uchIndex, gBitDuration);
            }
        }
        else
        {
            if((uchMacherCodeData[uchNextByteIndex] & (0x80>> uchNextBitIndex)))//01
            {
                pauseCarrier(uchIndex, gBitDuration);
            }
            else//00
            {
                pauseCarrier(uchIndex, (gBitDuration <<1));
                uchCurSendBit++;
            }
        }
    }
  
    return 0;
}

AIR_MSG_FRAME g_stAirFrameMsg;
/**********************************************************************
**����ԭ��:   void vSig125kSndFrameTest(unsigned char uchIndex)
**��ڲ���:   ����ͨ��
**�� �� ֵ:   ��
**��    ��:   125KHz�����źŷ���
**˵    ��:   
************************************************************************/
/*
void vSig125kSndFrameTest(uint8_t uchIndex)
{
    uint8_t uchLrc = 0;
    uint8_t i = 0;
    uint8_t *puchTmp = NULL;
    
    g_stAirFrameMsg.uchType = 1;
    g_stAirFrameMsg.uchLen = sizeof(AIR_MSG_FRAME);
    g_stAirFrameMsg.uchRemoteAddr[0] = 0x32;
    g_stAirFrameMsg.uchRemoteAddr[1] = 0x03;
    g_stAirFrameMsg.uchRemoteAddr[2] = 0x00;

    g_stAirFrameMsg.uchSubAddr = 0x00;

    g_stAirFrameMsg.uchFreq = 0x00;

    puchTmp = (uint8_t *)&g_stAirFrameMsg;

    for(i=0; i<(sizeof(AIR_MSG_FRAME) - 1); i++)
    {
        uchLrc ^= puchTmp[i];  //���
    }
    
    g_stAirFrameMsg.uchLrc = uchLrc;


    chSig125ksendDataString(uchIndex, (uint8_t *) &g_stAirFrameMsg, sizeof(AIR_MSG_FRAME));
}
*/

uint8_t 	g_uchIsTriggerLowSignal;
uint32_t 	g_ulNrfEnableTick = 0;
/**********************************************************************
**����ԭ��:   void vSig125kSndTask(void)
**��ڲ���:   ��	
**�� �� ֵ:   ��
**��    ��:   125KHz�����źŷ���
**˵    ��:   
************************************************************************/
void vSig125kSndTask(void)
{
    uint8_t uchRecvAddr[5] = {0x00};
    uint8_t uchLenRecvAddr = 0;
    uint8_t i 						 = 0;
    uint8_t uchLrc 				 = 0;
    uint8_t *puchTmp 			 = NULL;
    static  uint8_t s_uchIndexAttena = 0;
    
    if(g_uchIsTriggerLowSignal == true) //lzyn �����־λ����time3���жϺ��������޸ĵ�
    {   
        memset((uint8_t *)&g_stAirFrameMsg, 0, sizeof(AIR_MSG_FRAME));
        
        g_stAirFrameMsg.uchType = uchDevCfgGetTriggerMode();
        g_stAirFrameMsg.uchLen  = sizeof(AIR_MSG_FRAME);

        nDevCfgGetRecvAddr(&uchLenRecvAddr, uchRecvAddr);
        memcpy(g_stAirFrameMsg.uchRemoteAddr, uchRecvAddr, 3);//�̶�����Ϊ3
        
        g_stAirFrameMsg.uchSubAddr = uchDevCfgGetTriggerSubAddr();
        g_stAirFrameMsg.uchFreq = ((NRF_RF_CH_1 - nDevCfgGetRfFreq())/3) & 0x07;
        puchTmp = (unsigned char *)&g_stAirFrameMsg;
        for(i=0; i<(sizeof(AIR_MSG_FRAME) - 1); i++)
        {
            uchLrc ^= puchTmp[i];
        }
        g_stAirFrameMsg.uchLrc = uchLrc;

        s_uchIndexAttena++;
        s_uchIndexAttena &= 0x01;
				
		//unTick = ulGetCurTick();
        //printf("Sig send unTick = %lu\r\n", unTick);
        chSig125ksendDataString(s_uchIndexAttena, (unsigned char *) &g_stAirFrameMsg, sizeof(AIR_MSG_FRAME));
		
		//__disable_irq();
        g_uchIsTriggerLowSignal = false;
		//__enable_irq();  
    }
}

volatile uint8_t 	g_uchPwm1OptFlg 	 = 0; //PWM1����:0�ر�1��2��ͣ
volatile uint32_t g_unPwm1CyclesTick = 0; //PWM1����
volatile uint8_t  g_uchPwm2OptFlg    = 0; //PWM2����:0�ر�1��2��ͣ
volatile uint32_t g_unPwm2CyclesTick = 0; //PWM2����
/**********************************************************************
**����ԭ��:   void vPwmIrq(void)
**��ڲ���:   ��	
**�� �� ֵ:   ��
**��    ��:   PWM�жϴ���
**˵    ��:   
************************************************************************/
void vPwmIrq(void)
{
		__disable_irq();
		
		if(0 != g_uchPwm1OptFlg)
		{
				if(0 == g_unPwm1CyclesTick)
				{
						g_uchPwm1OptFlg = 0;
						ANT_X_SWITCH_OFF();
				}
				else
				{
						g_unPwm1CyclesTick--;
						if(1 == g_uchPwm1OptFlg)
						{
								ANT_X_SWITCH_ON();
						}
				}
		}

		if(0 != g_uchPwm2OptFlg)
		{
				if(0 == g_unPwm2CyclesTick)
				{
						g_uchPwm2OptFlg = 0;
						ANT_Y_SWITCH_OFF();
				}
				else
				{
						g_unPwm2CyclesTick--;
						if(1 == g_uchPwm2OptFlg)
						{
								ANT_Y_SWITCH_ON();
						}
				}
		}
			
		__enable_irq();
}

/**********************************************************************
**����ԭ��:   void vPwmsendCarrierAnt1(uint32_t unCycles)
**��ڲ���:   ����ʱ��
**�� �� ֵ:   ��
**��    ��:   Ant1����PWM����
**˵    ��:   
************************************************************************/
static void vPwmsendCarrierAnt1(uint32_t unCycles)
{
#if 1
    while((g_uchPwm1OptFlg != 0) || (g_uchPwm2OptFlg != 0)) //��־λ��Ϊ0��˵�����ڷ���
    {
        //printf("Flg = %d %d %d %d\r\n", g_uchPwm1OptFlg, g_uchPwm2OptFlg, g_unPwm1CyclesTick, g_unPwm2CyclesTick);
        continue;
    }
#else
    while((0 != g_unPwm1CyclesTick) || (0 != g_unPwm2CyclesTick))
    {
        continue;
    }
#endif
  
		__disable_irq();
    g_uchPwm1OptFlg = 1; //���ҪPWM����
    g_unPwm1CyclesTick = unCycles;
		__enable_irq();
}

/**********************************************************************
**����ԭ��:   void vPwmsendCarrierAnt2(uint32_t unCycles)
**��ڲ���:   ����ʱ��
**�� �� ֵ:   ��
**��    ��:   Ant2����PWM����
**˵    ��:   
************************************************************************/
static void vPwmsendCarrierAnt2(uint32_t unCycles)
{
#if 1
    while((g_uchPwm1OptFlg != 0) || (g_uchPwm2OptFlg != 0))
    {
        //printf("Flg = %d %d %d %d\r\n", g_uchPwm1OptFlg, g_uchPwm2OptFlg, g_unPwm1CyclesTick, g_unPwm2CyclesTick);
        continue;
    }
#else
    while((0 != g_unPwm1CyclesTick) || (0 != g_unPwm2CyclesTick) )
    {
        continue;
    }    
#endif    

		__disable_irq();
    g_uchPwm2OptFlg = 1;
    g_unPwm2CyclesTick = unCycles;
		__enable_irq();
}

/**********************************************************************
**����ԭ��:   void vPwmPauseCarrierAnt1(uint32_t unCycles)
**��ڲ���:   ��ͣʱ��
**�� �� ֵ:   ��
**��    ��:   Ant1��ͣPWM����
**˵    ��:   
************************************************************************/
static void vPwmPauseCarrierAnt1(uint32_t unCycles)
{
#if 1
		while((g_uchPwm1OptFlg != 0) || (g_uchPwm2OptFlg != 0))
		{
				//printf("Flg = %d %d %d %d\r\n", g_uchPwm1OptFlg, g_uchPwm2OptFlg, g_unPwm1CyclesTick, g_unPwm2CyclesTick);
				continue;
		}
#else
		while((0 != g_unPwm1CyclesTick) || (0 != g_unPwm2CyclesTick))
		{
				continue;
		}
#endif
    
		__disable_irq();
    g_uchPwm1OptFlg = 2; 
    if(unCycles > 2)
    {
        unCycles = unCycles - 2;
    }
    g_unPwm1CyclesTick = unCycles;
		__enable_irq();
}

/**********************************************************************
**����ԭ��:   void vPwmPauseCarrierAnt2(uint32_t unCycles)
**��ڲ���:   ��ͣʱ��
**�� �� ֵ:   ��
**��    ��:   Ant2��ͣPWM����
**˵    ��:   
************************************************************************/
static void vPwmPauseCarrierAnt2(uint32_t unCycles)
{
#if 1
		while((g_uchPwm1OptFlg != 0) || (g_uchPwm2OptFlg != 0))
		{
				//printf("Flg = %d %d %d %d\r\n", g_uchPwm1OptFlg, g_uchPwm2OptFlg, g_unPwm1CyclesTick, g_unPwm2CyclesTick);
				continue;
		}
#else
		while((0 != g_unPwm1CyclesTick) || (0 != g_unPwm2CyclesTick))
		{
				continue;
		}
#endif
			
		__disable_irq();
    g_uchPwm2OptFlg = 2;
    if(unCycles > 2)
    {
        unCycles = unCycles - 2;
    }
    g_unPwm2CyclesTick = unCycles;
		__enable_irq();
}
