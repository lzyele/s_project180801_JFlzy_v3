#include "devCfg.h"
#include "flash.h"
#include "nrf24l01.h"
#include "tool.h"
#include "uart.h"
#include <string.h>

#define DEV_CFG_PRINT_ENT_FUNCTION()  //printf("ENT %s()\r\n",  __FUNCTION__)
#define DEV_CFG_PRINT_EXIT_FUNCTION() //printf("EXIT %s()\r\n", __FUNCTION__)

#define FLG_FLASH_PARA_DATA           0
#define FLG_FLASH_PARA_BACK_UP_DATA   1

ST_DEV_CFG_PARA g_stDevCfgPara;

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSaveParaToFlash(void)
**��ڲ���:   ��
**�� �� ֵ:   �����Ƿ�ɹ�
**��    ��:   �������
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSaveParaToFlash(void)
{
    uint8_t *puchTmp = NULL;
    int nLenTmp 		 = 0;
    uint16_t usCrc   = 0;
    
    /*ÿ�θ��¼�¼����¼����*/
    g_stDevCfgPara.uchUpdataTimes++;

    usCrc = 0;
    nLenTmp = sizeof(ST_DEV_CFG_PARA)-2;
    puchTmp =  (uint8_t *) &g_stDevCfgPara;
    puchTmp+= 2;	/*������LRC�ֶ�*/

  	usCrc = crc16(usCrc, puchTmp, nLenTmp);
  	g_stDevCfgPara.usCrc = usCrc;

  	nSaveCfgParaToFlash((uint8_t *) &g_stDevCfgPara, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_DATA);
  	nSaveCfgParaToFlash((uint8_t *) &g_stDevCfgPara, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_BACK_UP_DATA);

		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetDefaultPara(void)
**��ڲ���:   ��
**�� �� ֵ:   �����Ƿ�ɹ�
**��    ��:   ����ȱʡ����
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetDefaultPara(void)
{    

    uint8_t puchAddr[3] = {0x21, 0x22, 0x23}; //uint8_t puchAddr[3] = {0x44, 0x55, 0x66};

    nDevCfgSetTranPower(false, false, NRF_TRAN_POWER_0dbm);  //���书��,NRF_TRAN_POWER_0dbm:0dbm
    nDevCfgSetTranSpeed(false, false, NRF_TRAN_SPEED_1Mbps); //��������,NRF_TRAN_SPEED_1Mbps:1Mbps
    nDevCfgSetRfFreq   (false, false, NRF_RF_CH_1);					 //����Ƶ��,NRF_RF_CH_1:2.497GHz
    
	nDevCfgSetWgMode   (false, WG_MODE_FORBIT_OUTPUT);			 //WG�ӿڶ���,WG_MODE_FORBIT_OUTPUT:��ֹwg���
		
    nDevCfgSetUartBaud (false, UART_BAUD_115200);						 //���ڲ�����,UART_BAUD_115200:115200
    nDevCfgSetUartOutputIdMode(false, OUTPUT_ID_MODE_RS232_OUTPUT_ID); //�����������ID����ʽ,OUTPUT_ID_MODE_RS232_OUTPUT_ID:ʹ��RS232���
		
    nDevCfgEnableBeepFlg(false, false); 											  //������,�������
    nDevCfgSetReadCardMode(false, READ_CARD_MODE_STATIC_FIXED); //����ģʽ,READ_CARD_MODE_STATIC_FIXED:�̶�����ģʽ
		
    nDevCfgSetRepSndIdInterval(false, REP_SND_ID_INTERVAL_1s); 	//����ID���,REP_SND_ID_INTERVAL_1s:1s
		
    nDevCfgSetDevAddr(false, 0); //�豸��ַ:0
    //nDevCfgSetAckPayload(false, uchLenPayload, puchPayload);

    nDevCfgSetLossValue(false, false, 0); //˥��ֵ:0
		
    nDevCfgGenAckPack(false); //�Զ��ظ�

	nDevCfgSetDevType(false, CFG_DEV_TYPE_TRI_FREQ); /*����������*///CFG_DEV_TYPE_TRI_FREQ:��Ƶ��
		
    nDevCfgSetRecvAddr(false, 3, puchAddr); //2.4GHz���յ�ַ

	nDevCfgSetTriggerMode         (false, TYPE_SND_CUR_POSITION);//TYPE_SND_CUR_POSITION:������Ƭ���͵�ǰλ����Ϣ
    nDevCfgSetTriggerLowFreq      (false, 0);  //��������ƵƵ��:0
    nDevCfgSetTriggerInterval     (false, 16); //�����������:16
    nDevCfgSetTriggerDistance     (false, 1);	 //��������:1
    nDevCfgSetTriggerSubAddr      (false, 0);  //������ַ:0
    nDevCfgSetTriggerActiveFlg    (false, 1);  //��������:1
    nDevCfgSetTriggerDuty         (false, 50); //ռ�ձ�:50
    nDevCfgSetTriggerInOutInterval(false, 2);	 //�������ʱ��:2
    
	nDevCfgSaveParaToFlash();
	return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetDevType(bool bIsSaveToFlash, CFG_DEV_TYPE type)
**��ڲ���:   �Ƿ񱣴�FLASH����;�豸����
**�� �� ֵ:   �����Ƿ�ɹ�
**��    ��:   �����豸����
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetDevType(bool bIsSaveToFlash, CFG_DEV_TYPE type)
{
    g_stDevCfgPara.uchDevMode = (uint8_t) type;

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
		
		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetDevType(void)
**��ڲ���:   ��
**�� �� ֵ:   �豸����
**��    ��:   ��ȡ�豸����
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgGetDevType(void)
{
    return g_stDevCfgPara.uchDevMode;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgGenAckPack(bool bIsSaveToFlash)
**��ڲ���:   �Ƿ񱣴�FLASH����
**�� �� ֵ:   �����Ƿ�ɹ�
**��    ��:   ���ݷ��ض�ʱ�����ûظ���ACK���ݰ�
**˵    ��:   
************************************************************************/
/*���ݷ��ض�ʱ�����ûظ���ACK���ݰ�*/
/*���÷��ض�ʱ��	6	0110xxxx	1	bRepeatSndInterval*/
/*
typedef enum
{
    REP_SND_ID_INTERVAL_5ms = 0,
    REP_SND_ID_INTERVAL_500ms = 1,
    REP_SND_ID_INTERVAL_1s    = 2,
    REP_SND_ID_INTERVAL_3s    = 3,
    REP_SND_ID_INTERVAL_5s    = 4,
    REP_SND_ID_INTERVAL_10s   = 5,
    REP_SND_ID_INTERVAL_15s   = 6,
    REP_SND_ID_INTERVAL_20s   = 7,
    REP_SND_ID_INTERVAL_60S   = 8,
    REP_SND_ID_INTERVAL_180S  = 9,
    REP_SND_ID_INTERVAL_300S  = 10        
}CFG_REP_SND_ID_INTERVAL;
*/
#define CARD_CMD_REP_SND_INTERVAL     0x6
DEV_CFG_STATUS_RET nDevCfgGenAckPack(bool bIsSaveToFlash)
{
		CFG_REP_SND_ID_INTERVAL interval;
		uint8_t uchCmd[2] = {0x00, 0x00};

		interval = nDevCfgGetRepSndIdInterval();

		uchCmd[0] = 0x00;
		uchCmd[1] = (CARD_CMD_REP_SND_INTERVAL<<4)| interval;
    
		nDevCfgSetAckPayload(bIsSaveToFlash, 2, uchCmd);  //mod by tang_shsh 2014-3-4
		
		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   void nDevCfgParaPrintf(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   ��ӡ������Ϣ
**˵    ��:   
************************************************************************/
void nDevCfgParaPrintf(void)
{
    NRF_TRAN_POWER 						tranPower; /*���书��*/
    NRF_TRAN_SPEED 				  	tranSpeed; /*��������*/
    NRF_RF_CHANNEL 						rfChn;
    CFG_WG_MODE 	 				  	cfgWgMode;
    CFG_UART_OUTPUT_ID_MODE 	cfgUartIdMode;
    CFG_UART_BAUD 				  	cfgUartBaud;
    CFG_REP_SND_ID_INTERVAL 	cfgRepSndIdInterval;
    uint16_t usAddr 					= 0;
    uint8_t uchAckLen 				= 0;
    uint8_t uchAckPayload[32] = {0x00, 0x00, 0x00, 0x00};
    int i = 0;
    
    tranPower 					= nDevCfgGetTranPower();
    tranSpeed 					= nDevCfgGetTranSpeed();
    rfChn 							= nDevCfgGetRfFreq();
    cfgWgMode 					= nDevCfgGetWgMode();
    cfgUartBaud 				= nDevCfgGetUartBaud();
    cfgUartIdMode 			= nDevCfgGetUartOutputIdMode();
    cfgRepSndIdInterval = nDevCfgGetRepSndIdInterval();
    usAddr 							= usDevCfgGetDevAddr();
    nDevCfgGetAckPayload(&uchAckLen, uchAckPayload);
    
    printf("tranPower = %d tranSpeed = %d rfChn= %d  cfgWgMode = %d  cfgUartBaud = %d  cfgUartIdMode = %d  cfgRepSndIdInterval = %d  unAddr = %d",
            tranPower, 
            tranSpeed, 
            rfChn, 
            cfgWgMode, 
            cfgUartBaud,
            cfgUartIdMode,
            cfgRepSndIdInterval,
             usAddr);
    
		printf("uchAckPayload(%d) [ ", uchAckLen);
		for(i=0; i<uchAckLen; i++)
		{
				printf("%02X ", uchAckPayload[i]);
		}
		printf("]\r\n");
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgLoadCfgParaFromFlash(void)
**��ڲ���:   ��
**�� �� ֵ:   �����Ƿ�ɹ�
**��    ��:   ��ȡFLASH������Ϣ
**˵    ��:   
************************************************************************/
/*FLASH ��Ч����ȡFLASH�е����ݣ�FLASH������Ч����Ĭ�ϵĲ���д��FLASH ��������*/
DEV_CFG_STATUS_RET nDevCfgLoadCfgParaFromFlash(void)
{
    int nLenTmp = 0;
    uint8_t *puchTmp =  NULL;
    ST_DEV_CFG_PARA stParaBackUp;
    uint8_t uchIsDataInvalid = 0;
    uint8_t uchIsBackUpInvalid = 0;

    volatile uint16_t usCrc = 0;
    
    if(0 != nReadCfgParaFromFlash((uint8_t *) &g_stDevCfgPara, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_DATA))
    {
        //printf("nReadCfgParaFromFlash Read  g_stDevCfgPara fail \r\n");
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    if(0 != nReadCfgParaFromFlash((uint8_t *) &stParaBackUp, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_BACK_UP_DATA))
    {
        //printf("nReadCfgParaFromFlash Read  stParaBackUp fail \r\n");        
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    usCrc = 0;
    nLenTmp = sizeof(ST_DEV_CFG_PARA)-2;
    puchTmp =  (uint8_t *) &g_stDevCfgPara;
    puchTmp+= 2;/*������LRC�ֶ�*/
    
    usCrc = crc16(usCrc, puchTmp, nLenTmp);

    if(usCrc != g_stDevCfgPara.usCrc)
    {
        uchIsDataInvalid = 1; 
    }

    usCrc = 0;
    nLenTmp = sizeof(ST_DEV_CFG_PARA)-2;
    puchTmp =  (uint8_t *) &stParaBackUp;
    puchTmp+= 2;/*������LRC�ֶ�*/

    usCrc = crc16(usCrc, puchTmp, nLenTmp);

    if(usCrc != stParaBackUp.usCrc)
    {
        uchIsBackUpInvalid = 1; 
    }


    if((1 == uchIsDataInvalid) && (1 == uchIsBackUpInvalid))
    {
        //printf("Flash data is Invaild, Load the default para\r\n");
        nDevCfgSetDefaultPara();
        return DEV_CFG_STATUS_SUC;
    }
    else if((1 == uchIsDataInvalid) && (1 != uchIsBackUpInvalid))
    {
        memcpy((uint8_t *) &g_stDevCfgPara, (uint8_t *)&stParaBackUp, sizeof(ST_DEV_CFG_PARA));
        nSaveCfgParaToFlash((uint8_t *) &g_stDevCfgPara, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_DATA);

        //printf("Flash data g_stDevCfgPara is Invaild, Load stParaBackUp para\r\n");

        return DEV_CFG_STATUS_SUC;
    }
    else if((1 != uchIsDataInvalid) && ( 1 == uchIsBackUpInvalid))
    {
        nSaveCfgParaToFlash((uint8_t *) &g_stDevCfgPara, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_BACK_UP_DATA);
        //printf("Flash data stParaBackUp is Invaild, Copy g_stDevCfgPara  para\r\n");
        return DEV_CFG_STATUS_SUC;
    }

    if( 0 != memcmp((uint8_t *) &g_stDevCfgPara, (uint8_t *)&stParaBackUp, sizeof(ST_DEV_CFG_PARA)))
    {
        nSaveCfgParaToFlash((uint8_t *) &g_stDevCfgPara, sizeof(ST_DEV_CFG_PARA), FLG_FLASH_PARA_BACK_UP_DATA);
        //printf("Flash data stParaBackUp is diff with the g_stDevCfgPara, Copy g_stDevCfgPara  para\r\n");
    }
    
    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTranPower(bool bIsSaveToFlash, bool bIsEffectNow, NRF_TRAN_POWER tranPower)
**��ڲ���:   �Ƿ񱣴�FLASH����;�Ƿ���Ч;���书��
**�� �� ֵ:   ���óɹ����
**��    ��:   ���÷��书�ʲ�������FLASH
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTranPower(bool bIsSaveToFlash, bool bIsEffectNow, NRF_TRAN_POWER tranPower)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    //if((NRF_TRAN_POWER_NEGATIVE_18dbm > tranPower) || (NRF_TRAN_POWER_0dbm < tranPower))
		if(NRF_TRAN_POWER_0dbm < tranPower)
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    
    g_stDevCfgPara.unParaRadio = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaRadio, PARA_CFGF_TRAN_POWER, PARA_CFGF_TRAN_POWER_MASK, tranPower);

    if(true == bIsEffectNow)
    {
        vNrfSetTranPower(tranPower);
    }

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
	
		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   NRF_TRAN_POWER nDevCfgGetTranPower(void)
**��ڲ���:   ��
**�� �� ֵ:   ���书��
**��    ��:   ��ȡ��ǰ���书��
**˵    ��:   
************************************************************************/
NRF_TRAN_POWER nDevCfgGetTranPower(void)
{   
    NRF_TRAN_POWER tranPower;

    DEV_CFG_PRINT_ENT_FUNCTION();

    tranPower = (NRF_TRAN_POWER) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaRadio, PARA_CFGF_TRAN_POWER, PARA_CFGF_TRAN_POWER_MASK);
    
    return tranPower;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTranSpeed(bool bIsSaveToFlash, bool bIsEffectNow, NRF_TRAN_SPEED tranSpeed)
**��ڲ���:   �Ƿ񱣴�FLASH����;�Ƿ���Ч;��������
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô������ʲ�������FLASH
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTranSpeed(bool bIsSaveToFlash, bool bIsEffectNow, NRF_TRAN_SPEED tranSpeed)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    //if((NRF_TRAN_SPEED_1Mbps > tranSpeed) || (NRF_TRAN_SPEED_250Kbps < tranSpeed))
		if(NRF_TRAN_SPEED_250Kbps < tranSpeed)
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    g_stDevCfgPara.unParaRadio = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaRadio, PARA_CFGF_TRAN_SPEED, PARA_CFGF_TRAN_SPEED_MASK, tranSpeed);

    if(true == bIsEffectNow)
    {
        vNrfSetTranSpeed(tranSpeed);
    }

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
    
    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   NRF_TRAN_SPEED nDevCfgGetTranSpeed(void)
**��ڲ���:   ��
**�� �� ֵ:   ��������
**��    ��:   ��ȡ��ǰ��������
**˵    ��:   
************************************************************************/
NRF_TRAN_SPEED nDevCfgGetTranSpeed(void)
{
    NRF_TRAN_SPEED tranSpeed;

    DEV_CFG_PRINT_ENT_FUNCTION();
    
    tranSpeed = (NRF_TRAN_SPEED) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaRadio, PARA_CFGF_TRAN_SPEED, PARA_CFGF_TRAN_SPEED_MASK);
    
    return tranSpeed;    
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetRfFreq(bool bIsSaveToFlash, bool bIsEffectNow, NRF_RF_CHANNEL rfChannel)
**��ڲ���:   �Ƿ񱣴�FLASH����;�Ƿ���Ч;����Ƶ��
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô���Ƶ�ʲ�������FLASH
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetRfFreq(bool bIsSaveToFlash, bool bIsEffectNow, NRF_RF_CHANNEL rfChannel)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    if((rfChannel > NRF_RF_CH_1) || rfChannel <NRF_RF_CH_8)
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    
    g_stDevCfgPara.unParaRadio = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaRadio, PARA_CFGF_TRAN_FREQ, PARA_CFGF_TRAN_FREQ_MASK, rfChannel);

    if(true == bIsEffectNow)
    {
        vNrfSetFreq(rfChannel);
    }

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
    
    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   NRF_RF_CHANNEL nDevCfgGetRfFreq(void)
**��ڲ���:   ��
**�� �� ֵ:   ����Ƶ��
**��    ��:   ��ȡ��ǰ����Ƶ��
**˵    ��:   
************************************************************************/
NRF_RF_CHANNEL nDevCfgGetRfFreq(void)
{
    NRF_RF_CHANNEL rfChannel;

    DEV_CFG_PRINT_ENT_FUNCTION();    
    
    rfChannel = (NRF_RF_CHANNEL) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaRadio, PARA_CFGF_TRAN_FREQ, PARA_CFGF_TRAN_FREQ_MASK);
    
		return rfChannel;       
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetDualFreqRecvAddr(void)
**��ڲ���:   ��
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ý��չܵ���ַ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetDualFreqRecvAddr(void)
{
    uint8_t uchAddr[3] = {0x32, 0x03, 0x00};
    uint8_t uchLen 		 = 3;

    nNrfSetRxPipeAddr(NRF_RX_PIPE0, enable, uchAddr, uchLen);    

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetWgMode(bool bIsSaveToFlash, CFG_WG_MODE wgMode)
**��ڲ���:   �Ƿ񱣴�FLASH����;Τ��ģʽ
**�� �� ֵ:   ���óɹ����
**��    ��:   ����Τ��ģʽ
**˵    ��:   
************************************************************************/
/*
[0:2] WEIGAN��ʽ
   000��wg26
   001: wg34
   110:�������
   111: ��ֹΤ�����
[3:4] UART ������
   00��115200
   01: 57600
   10: 38400
   11:9600
[5��6] �Ƿ��ֹ��������������š�
  00�������������
  01����ֹ�����������
*/
DEV_CFG_STATUS_RET nDevCfgSetWgMode(bool bIsSaveToFlash, CFG_WG_MODE wgMode)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    if((wgMode != WG_MODE_WG_26) && (wgMode != WG_MODE_WG_34) && (wgMode != WG_MODE_EXT_SWITCH) && (wgMode != WG_MODE_FORBIT_OUTPUT))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    
    g_stDevCfgPara.unParaInterface = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaInterface, PARA_CFGF_WG_MODE, PARA_CFGF_WG_MODE_MASK, wgMode);

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
    
    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   CFG_WG_MODE nDevCfgGetWgMode(void)
**��ڲ���:   ��
**�� �� ֵ:   Τ��ģʽ
**��    ��:   ��ȡΤ��ģʽ
**˵    ��:   
************************************************************************/
CFG_WG_MODE nDevCfgGetWgMode(void)
{
    CFG_WG_MODE wgMode;

    DEV_CFG_PRINT_ENT_FUNCTION();

    wgMode = (CFG_WG_MODE) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaInterface, PARA_CFGF_WG_MODE, PARA_CFGF_WG_MODE_MASK);

    return wgMode;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetUartBaud(bool bIsSaveToFlash, CFG_UART_BAUD baud)
**��ڲ���:   �Ƿ񱣴�FLASH����;������
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô��ڲ�����
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetUartBaud(bool bIsSaveToFlash, CFG_UART_BAUD baud)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    if((baud != UART_BAUD_115200) && (baud != UART_BAUD_57600) && (baud != UART_BAUD_38400) && (baud != UART_BAUD_9600))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    
    g_stDevCfgPara.unParaInterface = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaInterface, PARA_CFGF_UART_BAUD, PARA_CFGF_UART_BAUD_MASK, baud);

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
 
    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   static CFG_UART_BAUD nDevCfgGetUartBaud(void)
**��ڲ���:   ��
**�� �� ֵ:   ������
**��    ��:   ��ȡ���ڲ�����
**˵    ��:   
************************************************************************/
CFG_UART_BAUD nDevCfgGetUartBaud(void)
{
    CFG_UART_BAUD baud;

    DEV_CFG_PRINT_ENT_FUNCTION();   

    baud = (CFG_UART_BAUD) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaInterface, PARA_CFGF_UART_BAUD, PARA_CFGF_UART_BAUD_MASK);

    return baud;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetUartOutputIdMode(bool bIsSaveToFlash, CFG_UART_OUTPUT_ID_MODE cfgUartMode)
**��ڲ���:   �Ƿ񱣴�FLASH����;����ģʽ
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô���ģʽ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetUartOutputIdMode(bool bIsSaveToFlash, CFG_UART_OUTPUT_ID_MODE cfgUartMode)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    g_stDevCfgPara.unParaInterface = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaInterface, PARA_CFGF_FORBIT_UART_OUT, PARA_CFGF_FORBIT_UART_OUT_MASK, cfgUartMode);

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash(); 
    }

    return DEV_CFG_STATUS_SUC;
}


/**********************************************************************
**����ԭ��:   CFG_UART_BAUD nDevCfgGetUartBaud(void)
**��ڲ���:   ��
**�� �� ֵ:   ���ڹ���ģʽ
**��    ��:   ��ȡ���ڹ���ģʽ
**˵    ��:   
************************************************************************/
CFG_UART_OUTPUT_ID_MODE nDevCfgGetUartOutputIdMode(void)
{
    CFG_UART_OUTPUT_ID_MODE cfgUartMode;
    cfgUartMode = (CFG_UART_OUTPUT_ID_MODE) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaInterface, PARA_CFGF_FORBIT_UART_OUT, PARA_CFGF_FORBIT_UART_OUT_MASK);

    return cfgUartMode;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetRepSndIdInterval(bool bIsSaveToFlash,  CFG_REP_SND_ID_INTERVAL interval)
**��ڲ���:   �Ƿ񱣴�FLASH����;���ض����ʱ��
**�� �� ֵ:   ���óɹ����
**��    ��:   ���÷��ض����ʱ��
**˵    ��:   
************************************************************************/
/*
���ض����	nParaInterval	4	[0:3]
0000:5ms(�����ض�)
0001:500ms
0010:1S
0011��3S
0100��5S
0101��10S
0110�� 15S
0111��20s
1000:  60S
1001: 180S
1010: 300S 
[4:31]������
*/
DEV_CFG_STATUS_RET nDevCfgSetRepSndIdInterval(bool bIsSaveToFlash,  CFG_REP_SND_ID_INTERVAL interval)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    
    g_stDevCfgPara.unParaInterval = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unParaInterval, 
                                                           PARA_CFGF_REP_SND_ID_INTERVAL, 
                                                           PARA_CFGF_REP_SND_ID_INTERVAL_MASK, 
                                                           interval);
    nDevCfgGenAckPack(bIsSaveToFlash);

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   CFG_REP_SND_ID_INTERVAL nDevCfgGetRepSndIdInterval(void)
**��ڲ���:   ��
**�� �� ֵ:   ���ض����ʱ��
**��    ��:   ��ȡ���ض����ʱ��
**˵    ��:   
************************************************************************/
CFG_REP_SND_ID_INTERVAL nDevCfgGetRepSndIdInterval(void)
{
    CFG_REP_SND_ID_INTERVAL repSndIdInterval;

    DEV_CFG_PRINT_ENT_FUNCTION();    

    repSndIdInterval = (CFG_REP_SND_ID_INTERVAL) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unParaInterval, PARA_CFGF_REP_SND_ID_INTERVAL, PARA_CFGF_REP_SND_ID_INTERVAL_MASK);   

    return repSndIdInterval;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetReadCardMode(bool bIsSaveToFlash,  CFG_READ_CARD_MODE cfgReadCardMode)
**��ڲ���:   �Ƿ񱣴�FLASH����;����ģʽ
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ö���ģʽ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetReadCardMode(bool bIsSaveToFlash,  CFG_READ_CARD_MODE cfgReadCardMode)
{
//    DEV_CFG_PRINT_ENT_FUNCTION();
//    
//    if((READ_CARD_MODE_STATIC_FIXED > cfgReadCardMode) || (READ_CARD_EXT_TRIGGER_MODE < cfgReadCardMode))
//    {
//        return DEV_CFG_STATUS_ERR_VALUE;
//    }
//    
//    g_stDevCfgPara.unReadCardMode = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unReadCardMode, 
//                                                          PARA_CFGF_READ_CARD_MODE, 
//                                                          PARA_CFGF_READ_CARD_MODE_MASK, 
//                                                          cfgReadCardMode);

//    if(true == bIsSaveToFlash)
//    {
//        nDevCfgSaveParaToFlash();
//    }

//    /*ʹ������*/
//    switch(cfgReadCardMode)
//    {
//        case READ_CARD_MODE_STATIC_FIXED:
//            NRF_SWITCH_TO_ACTIVE();
//            break;
//        case READ_CARD_SOFTWARE_CTR:
//            if(true == bDevCfgGetEnableListenFlg())
//            {
//                NRF_SWITCH_TO_ACTIVE();              
//            }
//            else
//            {
//                NRF_SWITCH_TO_STANDBY();
//            }
//            break;
//        case READ_CARD_EXT_TRIGGER_MODE:
//            if(true == EXT_INT_TRIGGER_IS_ACTIVE())
//            {
//                NRF_SWITCH_TO_ACTIVE();              
//            }
//            else
//            {
//                NRF_SWITCH_TO_STANDBY();
//            }
//            break;
//        default:
//            return DEV_CFG_STATUS_ERR_VALUE;
//    }
		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   CFG_READ_CARD_MODE nDevCfgGetReadCardMode(void)
**��ڲ���:   ��
**�� �� ֵ:   ����ģʽ
**��    ��:   ��ȡ����ģʽ
**˵    ��:   
************************************************************************/
CFG_READ_CARD_MODE nDevCfgGetReadCardMode(void)
{
    CFG_READ_CARD_MODE cfgReadCardMode;
    
    cfgReadCardMode = (CFG_READ_CARD_MODE) DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unReadCardMode, PARA_CFGF_READ_CARD_MODE, PARA_CFGF_READ_CARD_MODE_MASK);

    return cfgReadCardMode;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgEnableListenFlg(bool bIsSaveToFlash, bool bFlag)
**��ڲ���:   �Ƿ񱣴�FLASH����;���ö���
**�� �� ֵ:   ���óɹ����
**��    ��:   �����Ƿ����ö���
**˵    ��:   
************************************************************************/
/*
[0]�����ü�����Ƭ
  0��������Ƶ�ŵ�����
  1���ݶ������ŵ�����
*/
DEV_CFG_STATUS_RET nDevCfgEnableListenFlg(bool bIsSaveToFlash, bool bFlag)
{
//    CFG_READ_CARD_MODE cfgReadCardMode;
//    DEV_CFG_PRINT_ENT_FUNCTION();
//    
//    g_stDevCfgPara.unReadCardFlg = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unReadCardFlg, PARA_CFGF_ACTIVE_LISTEN_CHN, PARA_CFGF_ACTIVE_LISTEN_CHN_MASK, bFlag);
//    //nDevCfgSaveParaToFlash(); 

//    cfgReadCardMode = nDevCfgGetReadCardMode();

//    if(READ_CARD_SOFTWARE_CTR == cfgReadCardMode)
//    {
//        if(bFlag == true)
//        {
//            NRF_SWITCH_TO_ACTIVE();
//        }
//        else
//        {
//            NRF_SWITCH_TO_STANDBY();
//        }
//    }

//    if(true == bIsSaveToFlash)
//    {
//        nDevCfgSaveParaToFlash(); 
//    }

		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   bool bDevCfgGetEnableListenFlg(void)
**��ڲ���:   ��
**�� �� ֵ:   �Ƿ����ö���
**��    ��:   ��ȡ�Ƿ����ö���
**˵    ��:   
************************************************************************/
bool bDevCfgGetEnableListenFlg(void)
{
    bool bFlag;

    DEV_CFG_PRINT_ENT_FUNCTION();
    bFlag = DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unReadCardFlg, PARA_CFGF_ACTIVE_LISTEN_CHN, PARA_CFGF_ACTIVE_LISTEN_CHN_MASK);

    return bFlag;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgEnableListenFlg(bool bIsSaveToFlash, bool bFlag)
**��ڲ���:   �Ƿ񱣴�FLASH����;���÷�����
**�� �� ֵ:   ���óɹ����
**��    ��:   �����Ƿ����÷�����
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgEnableBeepFlg(bool bIsSaveToFlash, bool bFlag)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    g_stDevCfgPara.unReadCardFlg = DEV_CFG_SET_PARA_VALUE(g_stDevCfgPara.unReadCardFlg, PARA_CFGF_ACTIVE_BEEP, PARA_CFGF_ACTIVE_BEEP_MASK, bFlag);

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   bool bDevCfgGetEnableBeepFlg(void)
**��ڲ���:   ��
**�� �� ֵ:   �Ƿ����÷�����
**��    ��:   ��ȡ�Ƿ����÷�����
**˵    ��:   
************************************************************************/
bool bDevCfgGetEnableBeepFlg(void)
{
    bool bFlag;
    
    bFlag = DEV_CFG_GET_PARA_VALUE(g_stDevCfgPara.unReadCardFlg, PARA_CFGF_ACTIVE_BEEP, PARA_CFGF_ACTIVE_BEEP_MASK);
    
    return bFlag;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetDevAddr(bool bIsSaveToFlash, uint16_t usAddr)
**��ڲ���:   �Ƿ񱣴�FLASH����;�豸��ַ
**�� �� ֵ:   ���óɹ����
**��    ��:   �����豸��ַ
**˵    ��:   
************************************************************************/
/*�豸��ַ*/
DEV_CFG_STATUS_RET nDevCfgSetDevAddr(bool bIsSaveToFlash, uint16_t usAddr)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    g_stDevCfgPara.usDevAddr = usAddr;

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint16_t usDevCfgGetDevAddr(void)
**��ڲ���:   ��
**�� �� ֵ:   �豸��ַ
**��    ��:   ��ȡ�豸��ַ
**˵    ��:   
************************************************************************/
uint16_t usDevCfgGetDevAddr(void)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    return g_stDevCfgPara.usDevAddr;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetLossValue(bool bIsSaveToFlash, bool bIsEffectNow,  uint8_t uchValue)
**��ڲ���:   �Ƿ񱣴�FLASH����;�Ƿ���Ч;˥������ֵ
**�� �� ֵ:   ���óɹ����
**��    ��:   ����˥������ֵ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetLossValue(bool bIsSaveToFlash, bool bIsEffectNow,  uint8_t uchValue)
{
    DEV_CFG_PRINT_ENT_FUNCTION();

    g_stDevCfgPara.uchLossValue = uchValue;

    if(true == bIsEffectNow)
    {
        //nAttennuationSetLossValue(uchValue);
    }
    
    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }
		
    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetLossValue(void)
**��ڲ���:   ��
**�� �� ֵ:   ˥������ֵ
**��    ��:   ��ȡ˥������ֵ
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgGetLossValue(void)
{
    DEV_CFG_PRINT_ENT_FUNCTION();

    return g_stDevCfgPara.uchLossValue;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetLossValue(bool bIsSaveToFlash, bool bIsEffectNow,  uint8_t uchValue)
**��ڲ���:   �Ƿ񱣴�FLASH����;ACK PayLoad
**�� �� ֵ:   ���óɹ����
**��    ��:   ����ACK PayLoad
**˵    ��:   
************************************************************************/
/*
7	ȷ�Ϸ�����������(ACK PAYLOAD��	nAckData	0~32	Ԥ�ȴ洢���ظ��Ŀ�Ƭ�����ݣ����������ÿ�Ƭ������
*/
DEV_CFG_STATUS_RET nDevCfgSetAckPayload(bool bIsSaveToFlash, uint8_t uchLen, uint8_t *puchPayload)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    if(uchLen > 32 || (NULL == puchPayload))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    g_stDevCfgPara.uchLenAck = uchLen;
    memcpy(g_stDevCfgPara.uchAckPayload, puchPayload, uchLen);

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgGetAckPayload(uint8_t *puchLen, uint8_t *puchPayload)
**��ڲ���:   ����;����
**�� �� ֵ:   ���óɹ����
**��    ��:   ��ȡACK PayLoad
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgGetAckPayload(uint8_t *puchLen, uint8_t *puchPayload)
{
    DEV_CFG_PRINT_ENT_FUNCTION();

    *puchLen= g_stDevCfgPara.uchLenAck;
    memcpy(puchPayload, g_stDevCfgPara.uchAckPayload, g_stDevCfgPara.uchLenAck);

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetRecvAddr(bool bIsSaveToFlash, uint8_t uchLen, uint8_t *puchAddr)
**��ڲ���:   �Ƿ񱣴�FLASH����;��ַ����;���յ�ַ
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ý��յ�ַ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetRecvAddr(bool bIsSaveToFlash, uint8_t uchLen, uint8_t *puchAddr)
{
    DEV_CFG_PRINT_ENT_FUNCTION();
    
    if((uchLen > 5) || (NULL == puchAddr) || (uchLen == 0))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    g_stDevCfgPara.uchLenRecvAddr = uchLen;
    memcpy(g_stDevCfgPara.uchRecvAddr, puchAddr, uchLen);

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgGetRecvAddr(uint8_t *puchLen, uint8_t *puchAddr)
**��ڲ���:   ����;����
**�� �� ֵ:   ���óɹ����
**��    ��:   ��ȡ���յ�ַ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgGetRecvAddr(uint8_t *puchLen, uint8_t *puchAddr)
{
    *puchLen= g_stDevCfgPara.uchLenRecvAddr;
    memcpy(puchAddr, g_stDevCfgPara.uchRecvAddr, g_stDevCfgPara.uchLenRecvAddr);

		return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerMode(bool bIsSaveToFlash, LOW_FREQ_TIRGGER_TYPE triggerMode)
**��ڲ���:   �Ƿ񱣴�FLASH����;����ģʽ
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô���ģʽ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerMode(bool bIsSaveToFlash, LOW_FREQ_TIRGGER_TYPE triggerMode)
{
    #if 0
    if((TYPE_SND_CUR_POSITION > triggerMode) || (TYPE_SAMPLE_PONIT_PARKING_MODE < triggerMode))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    #else
    if((TYPE_SND_CUR_POSITION != triggerMode) && 
       (TYPE_SAMPLE_POINT_IN_OUT_MODE   != triggerMode) &&
       (TYPE_SAMPLE_PONIT_PARKING_MODE  != triggerMode) &&
       (TYPE_SAMPLE_POINT_IN_OUT_MODE_2 != triggerMode))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    #endif
    
    g_stDevCfgPara.uchTriggerMode = triggerMode;

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();        
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   LOW_FREQ_TIRGGER_TYPE uchDevCfgGetTriggerMode(void)
**��ڲ���:   ��
**�� �� ֵ:   ����ģʽ
**��    ��:   ��ȡ����ģʽ
**˵    ��:   
************************************************************************/
LOW_FREQ_TIRGGER_TYPE uchDevCfgGetTriggerMode(void)
{
    return (LOW_FREQ_TIRGGER_TYPE) g_stDevCfgPara.uchTriggerMode;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerLowFreq(bool bIsSaveToFlash, uint8_t uchFreq)
**��ڲ���:   �Ƿ񱣴�FLASH����;��������ƵƵ��
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô�������ƵƵ��
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerLowFreq(bool bIsSaveToFlash, uint8_t uchFreq)
{
    if(uchFreq > 3)
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    g_stDevCfgPara.uchTriggerLowFreq = uchFreq;
    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();  
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetTriggerLowFreq(void)
**��ڲ���:   ��
**�� �� ֵ:   ��������ƵƵ��
**��    ��:   ��ȡ��������ƵƵ��
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgGetTriggerLowFreq(void)
{
    return g_stDevCfgPara.uchTriggerLowFreq;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerInterval(bool bIsSaveToFlash, uint8_t uchInterval)
**��ڲ���:   �Ƿ񱣴�FLASH����;��������
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô�������
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerInterval(bool bIsSaveToFlash, uint8_t uchInterval)
{
    g_stDevCfgPara.uchTriggerInterval = uchInterval;

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetTriggerInterval(void)
**��ڲ���:   ��
**�� �� ֵ:   ��������
**��    ��:   ��ȡ��������
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgGetTriggerInterval(void)
{
    return g_stDevCfgPara.uchTriggerInterval;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerDistance(bool bIsSaveToFlash, uint8_t uchDistance)
**��ڲ���:   �Ƿ񱣴�FLASH����;����������
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô���������
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerDistance(bool bIsSaveToFlash, uint8_t uchDistance)
{
    if((0 != uchDistance) && (1 != uchDistance))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    
    g_stDevCfgPara.uchTriggerDistance = uchDistance;

    if(true == bIsSaveToFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetTriggerDistance(void)
**��ڲ���:   ��
**�� �� ֵ:   ����������
**��    ��:   ��ȡ����������
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgGetTriggerDistance(void)
{
    return g_stDevCfgPara.uchTriggerDistance;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerSubAddr(bool bIsSaveFlash, uint8_t uchSubAddr)
**��ڲ���:   �Ƿ񱣴�FLASH����;�������ӵ�ַ
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ô������ӵ�ַ
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerSubAddr(bool bIsSaveFlash, uint8_t uchSubAddr)
{
    g_stDevCfgPara.uchTriggerSubAddr = uchSubAddr;

    if(true == bIsSaveFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetTriggerSubAddr(void)
**��ڲ���:   ��
**�� �� ֵ:   �������ӵ�ַ
**��    ��:   ��ȡ�������ӵ�ַ
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgGetTriggerSubAddr(void)
{
    return g_stDevCfgPara.uchTriggerSubAddr;
}


/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerActiveFlg(bool bIsSaveFlash, uint8_t uchFlg)
**��ڲ���:   �Ƿ񱣴�FLASH����;�Ƿ���������
**�� �� ֵ:   ���óɹ����
**��    ��:   �����Ƿ���������
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerActiveFlg(bool bIsSaveFlash, uint8_t uchFlg)
{
    if((0 != uchFlg) && (1 != uchFlg))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }
    
    g_stDevCfgPara.uchTriggerIsActive = uchFlg;

    if(true == bIsSaveFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgTriggerActiveFlg(void)
**��ڲ���:   ��
**�� �� ֵ:   �Ƿ���������
**��    ��:   ��ȡ�Ƿ���������
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgTriggerActiveFlg(void)
{
    return g_stDevCfgPara.uchTriggerIsActive;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerInOutInterval(bool bIsSaveFlash, uint8_t uchInterval)
**��ڲ���:   �Ƿ񱣴�FLASH����;�������ʱ��
**�� �� ֵ:   ���óɹ����
**��    ��:   ���ý������ʱ��
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerInOutInterval(bool bIsSaveFlash, uint8_t uchInterval)
{
    
    g_stDevCfgPara.uchTriggerIntervalInOut = uchInterval;

    if(true == bIsSaveFlash)
    {
        nDevCfgSaveParaToFlash();        
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgTriggerInOutInterval(void)
**��ڲ���:   ��
**�� �� ֵ:   �������ʱ��
**��    ��:   ��ȡ�������ʱ��
**˵    ��:   
************************************************************************/
uint8_t uchDevCfgTriggerInOutInterval(void)
{
    return g_stDevCfgPara.uchTriggerIntervalInOut;
}

/**********************************************************************
**����ԭ��:   DEV_CFG_STATUS_RET nDevCfgSetTriggerDuty(bool bIsSaveFlash, uint8_t uchDuty)
**��ڲ���:   �Ƿ񱣴�FLASH����;��Ƶ����ռ�ձ�
**�� �� ֵ:   ���óɹ����
**��    ��:   ���õ�Ƶ����ռ�ձ�
**˵    ��:   
************************************************************************/
DEV_CFG_STATUS_RET nDevCfgSetTriggerDuty(bool bIsSaveFlash, uint8_t uchDuty)
{
    if(((uchDuty & 0x0f) == 0) || (((uchDuty & 0xf0)>>4) == 0))
    {
        return DEV_CFG_STATUS_ERR_VALUE;
    }

    g_stDevCfgPara.uchTriggerSigDuty = uchDuty;

    if(true == bIsSaveFlash)
    {
        nDevCfgSaveParaToFlash();
    }

    return DEV_CFG_STATUS_SUC;
}

/**********************************************************************
**����ԭ��:   uint8_t uchDevCfgGetTriggerDuty(void)
**��ڲ���:   ��
**�� �� ֵ:   ��Ƶ����ռ�ձ�
**��    ��:   ��ȡ��Ƶ����ռ�ձ�
**˵    ��:   
************************************************************************/
/*��Ƶ����ռ�ձ�����*/
uint8_t uchDevCfgGetTriggerDuty(void)
{
    return g_stDevCfgPara.uchTriggerSigDuty;
}
