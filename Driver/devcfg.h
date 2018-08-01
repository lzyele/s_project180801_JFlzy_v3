#ifndef  __DEV_CFG_H_
#define  __DEV_CFG_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "nrf24l01.h"


#ifndef false
#define false  0
#endif

#ifndef true
#define true  1
#endif

#ifndef enable
#define enable  1
#endif

#ifndef disable
#define disable  0
#endif

#ifndef bool
typedef uint8_t bool;
#endif


#define FIRE_SOFE_VER "ZCR813-1.1-151127"

/*��ƵƵ�����ü� NRF24L01.H*/
/*WG�ӿڶ���*/
typedef enum
{
    WG_MODE_WG_26         = 0,  //wg26
    WG_MODE_WG_34         = 1,  //wg34
    WG_MODE_EXT_SWITCH    = 6,  //�������
    WG_MODE_FORBIT_OUTPUT = 7   //��ֹwg���
}CFG_WG_MODE;

/*���ڲ�����*/
typedef enum
{
    UART_BAUD_115200 = 0,
    UART_BAUD_57600  = 1,
    UART_BAUD_38400  = 2,
    UART_BAUD_9600   = 3
}CFG_UART_BAUD;

/*�����������ID����ʽ*/
typedef enum
{
    DISABLE_UART_OUTPUT_ID 				 = 0,  //��ֹ���ID
    OUTPUT_ID_MODE_RS232_OUTPUT_ID = 1,  //ʹ��RS232���
    OUTPUT_ID_MODE_RS485_OUTPUT_ID = 2   //ʹ��RS485���
}CFG_UART_OUTPUT_ID_MODE;

//��������/��ͣ
typedef enum
{
    DISABLE_LISTEN_CHN  = 0,/*ֹͣ����*/
    ENABLE_LISTEN_CHN   = 1 /*���ü���*/
}CFG_LISTEN_CHN;

/*��ƵƵ�����ü� NRF24L01.H*/
typedef enum
{
    REP_SND_ID_INTERVAL_125ms = 0,
    REP_SND_ID_INTERVAL_250ms = 1,
    REP_SND_ID_INTERVAL_500ms	= 2,
    REP_SND_ID_INTERVAL_1s    = 3,
    REP_SND_ID_INTERVAL_2s    = 4,
    REP_SND_ID_INTERVAL_3s    = 5,
    REP_SND_ID_INTERVAL_4s    = 6,
    REP_SND_ID_INTERVAL_10s   = 7,  
}CFG_REP_SND_ID_INTERVAL;

/*�Ƿ����ü���*/
typedef enum
{
    DISABLE_LISTEN_CHANNEL = 0,
    ENABLE_LISTEN_CHANNEL  = 1
}CFG_LISTEN_CHANNEL;

//��Ƶ�����豸����λƫ��
#define PARA_CFGF_TRAN_POWER  0
#define PARA_CFGF_TRAN_SPEED  2
#define PARA_CFGF_TRAN_FREQ   4
#define PARA_CFGF_RADIO_REVS  7

#define PARA_CFGF_TRAN_POWER_MASK  (0x3<<PARA_CFGF_TRAN_POWER)
#define PARA_CFGF_TRAN_SPEED_MASK  (0x3<<PARA_CFGF_TRAN_SPEED)
#define PARA_CFGF_TRAN_FREQ_MASK   (0xff<<PARA_CFGF_TRAN_FREQ)

//�ӿڲ���
#define PARA_CFGF_WG_MODE           0
#define PARA_CFGF_UART_BAUD         3
#define PARA_CFGF_FORBIT_UART_OUT   5

#define PARA_CFGF_WG_MODE_MASK         (0x7<<PARA_CFGF_WG_MODE)
#define PARA_CFGF_UART_BAUD_MASK       (0x3<<PARA_CFGF_UART_BAUD)
#define PARA_CFGF_FORBIT_UART_OUT_MASK (0x3<<PARA_CFGF_FORBIT_UART_OUT)

/*
[0]�����ü�����Ƭ
  0��������Ƶ�ŵ�����
  1���ݶ������ŵ�����
*/
#define PARA_CFGF_ACTIVE_LISTEN_CHN       0
#define PARA_CFGF_ACTIVE_BEEP             1

#define PARA_CFGF_ACTIVE_LISTEN_CHN_MASK  (1<<PARA_CFGF_ACTIVE_LISTEN_CHN)
#define PARA_CFGF_ACTIVE_BEEP_MASK          (1<< PARA_CFGF_ACTIVE_BEEP)

/*���ض���� nParaInterval*/
#define PARA_CFGF_REP_SND_ID_INTERVAL          0
#define PARA_CFGF_REP_SND_ID_INTERVAL_MASK     (0xf<<PARA_CFGF_REP_SND_ID_INTERVAL)

/*���üĴ���FIELD ֵ*/
#define DEV_CFG_GET_PARA_VALUE(paraValue, fieldOffset, mask)  ((paraValue & mask)>>fieldOffset)
#define DEV_CFG_SET_PARA_VALUE(paraValue, fieldOffset, mask, fieldValue)     ((paraValue&(~mask)) | (fieldValue<<fieldOffset))

/*����ģʽ*/
/*�̶�����ģʽ*/
/*������ƶ���ģʽ*/
/*�ⲿ��������ģʽ*/
typedef enum
{
    READ_CARD_MODE_STATIC_FIXED  = 1,    
    READ_CARD_SOFTWARE_CTR = 2,
    READ_CARD_EXT_TRIGGER_MODE = 3
}CFG_READ_CARD_MODE;

#define PARA_CFGF_READ_CARD_MODE        0
#define PARA_CFGF_READ_CARD_MODE_MASK   (0xf <<PARA_CFGF_READ_CARD_MODE)

typedef enum
{
    CFG_DEV_TYPE_TRI_FREQ  = 1, /*��Ƶ��*/
    CFG_DEV_TYPE_DUAL_FREQ = 2  /*˫Ƶ��*/
}CFG_DEV_TYPE; /*�豸������*/

typedef enum
{
    TYPE_SND_CUR_POSITION = 0, 				     /*������Ƭ���͵�ǰλ����Ϣ*/
    TYPE_SAMPLE_POINT_IN_OUT_MODE = 1, 		 /*������Ƭ�ɼ���ǰλ����Ϣ*/
    TYPE_SAMPLE_PONIT_PARKING_MODE = 2, 	 /*ͣ����ģʽ*/ 
    TYPE_SAMPLE_POINT_IN_OUT_MODE_2 = 0x81 /*������Ƭ�ɼ���ǰλ����Ϣ, ��ʱ��ͣ�����ɼ�¼*/
}LOW_FREQ_TIRGGER_TYPE;/*����ģʽ*/

/*��������*/
typedef struct
{
    uint16_t usCrc;									 /*У��ֵ*/
    uint8_t uchUpdataTimes;					 /*���´���*/
    uint8_t uchFill[1];
    uint32_t unParaRadio; 					 /*��Ƶ����*/
    uint32_t unParaInterface; 			 /*�ӿڲ���*/
    uint32_t unParaInterval; 				 /*�������*/
    uint32_t unReadCardMode; 				 /*����ģʽ*/
    uint32_t unReadCardFlg; 				 /*�������λ*/
    uint16_t usDevAddr;      				 /*�豸��ַ*/
    uint8_t uchLossValue;    				 /*˥������ֵ*/
    uint8_t uchDevMode;      				 /*�豸����,����������˫Ƶ��������Ƶ��*/
    uint8_t uchLenRecvAddr;  				 /*���յ�ַ����*/
    uint8_t uchRecvAddr[5];  				 /*���յ�ַ*/
    uint8_t uchLenAck;    					 /**/
    uint8_t uchAckPayload[32]; 		 	 /**/
    uint8_t uchTriggerMode; 				 /*������ģʽ*/
    uint8_t uchTriggerLowFreq; 			 /*��������ƵƵ��*/
    uint8_t uchTriggerInterval; 		 /*��������*/
    uint8_t uchTriggerDistance; 		 /*����������*/
    uint8_t uchTriggerSubAddr; 			 /*�������ӵ�ַ*/
    uint8_t uchTriggerIsActive; 		 /*�Ƿ���������*/
    uint8_t uchTriggerIntervalInOut; /*�������ʱ��*/
    uint8_t uchTriggerSigDuty;       /*��Ƶ�ź�ռ�ձ�*/
        
/*
    CFG_INDEX_TRIGGER_MODE                = 80, //������ģʽ
    CFG_INDEX_TRIGGER_LOW_FREQ_INDEX      = 81, //��Ƶ�Ĵ���Ƶ��
    CFG_INDEX_TRIGGER_LOW_FREQ_DISTANCE   = 82, //���Ǿ���
    CFG_INDEX_TRIGGER_SND_INTERVAL        = 84, //�������ڼ��
    CFG_INDEX_TRIGGER_SUB_ADDR            = 86, //�����������ֵ�ַ
    CFG_INDEX_TRIGGER_IS_ACTIVE           = 87  //��������or��������
*/
}ST_DEV_CFG_PARA, *pST_DEV_CFG_PARA;

typedef enum
{
    DEV_CFG_STATUS_SUC = 0,
    DEV_CFG_STATUS_ERR_VALUE = -1
}DEV_CFG_STATUS_RET;

DEV_CFG_STATUS_RET nDevCfgSaveParaToFlash(void); /*�����豸������flash��*/
DEV_CFG_STATUS_RET nDevCfgSetDefaultPara(void);  /*����ȱʡ����*/
DEV_CFG_STATUS_RET nDevCfgLoadCfgParaFromFlash(void); /*��FLASH�ж�ȡ�豸�Ĳ���*/

void nDevCfgParaPrintf(void);

DEV_CFG_STATUS_RET nDevCfgSetDevType(bool bIsSaveToFlash, CFG_DEV_TYPE type); /*�����豸����*/
uint8_t uchDevCfgGetDevType(void);
	
DEV_CFG_STATUS_RET nDevCfgGenAckPack(bool bIsSaveToFlash);

DEV_CFG_STATUS_RET nDevCfgSetTranPower(bool bIsSaveToFlash, bool bIsEffectNow, NRF_TRAN_POWER tranPower); /*����2.4���͹���*/
NRF_TRAN_POWER nDevCfgGetTranPower(void); /*��ȡ���书��*/
DEV_CFG_STATUS_RET nDevCfgSetTranSpeed(bool bIsSaveToFlash, bool bIsEffectNow, NRF_TRAN_SPEED tranSpeed); /*���ô�������*/
NRF_TRAN_SPEED nDevCfgGetTranSpeed(void); /*��ȡ��������*/
DEV_CFG_STATUS_RET nDevCfgSetRfFreq(bool bIsSaveToFlash, bool bIsEffectNow, NRF_RF_CHANNEL rfChannel);		 /*���ô���Ƶ��*/
NRF_RF_CHANNEL nDevCfgGetRfFreq(void); 	 /*��ȡ����Ƶ��*/
DEV_CFG_STATUS_RET nDevCfgSetDualFreqRecvAddr(void); /*����˫Ƶ���Ľ��յ�ַ*/

DEV_CFG_STATUS_RET nDevCfgSetWgMode(bool bIsSaveToFlash, CFG_WG_MODE wgMode);   /*����WGģʽ*/
CFG_WG_MODE nDevCfgGetWgMode(void); /*��ȡWGģʽ*/

DEV_CFG_STATUS_RET nDevCfgSetUartBaud(bool bIsSaveToFlash, CFG_UART_BAUD baud); /*���ô��ڲ�����*/
CFG_UART_BAUD nDevCfgGetUartBaud(void); /*��ȡ���ڲ�����*/

DEV_CFG_STATUS_RET nDevCfgSetUartOutputIdMode(bool bIsSaveToFlash, CFG_UART_OUTPUT_ID_MODE cfgUartMode);
CFG_UART_OUTPUT_ID_MODE nDevCfgGetUartOutputIdMode(void);

DEV_CFG_STATUS_RET nDevCfgSetRepSndIdInterval(bool bIsSaveToFlash, CFG_REP_SND_ID_INTERVAL interval); /*���÷��ض����ʱ��*/
CFG_REP_SND_ID_INTERVAL nDevCfgGetRepSndIdInterval(void); /*��ȡ���ض����ʱ��*/

DEV_CFG_STATUS_RET nDevCfgSetReadCardMode(bool bIsSaveToFlash,  CFG_READ_CARD_MODE cfgReadCardMode);
CFG_READ_CARD_MODE nDevCfgGetReadCardMode(void);

DEV_CFG_STATUS_RET nDevCfgEnableListenFlg(bool bIsSaveToFlash, bool bFlag); /*�����Ƿ����ö���*/

bool bDevCfgGetEnableListenFlg(void); /*��ȡ�Ƿ����ö������*/

DEV_CFG_STATUS_RET nDevCfgEnableBeepFlg(bool bIsSaveToFlash, bool bFlag);
bool bDevCfgGetEnableBeepFlg(void);

DEV_CFG_STATUS_RET nDevCfgSetDevAddr(bool bIsSaveToFlash, uint16_t usAddr); /*�����豸��ַ*/
uint16_t usDevCfgGetDevAddr(void);

DEV_CFG_STATUS_RET nDevCfgSetLossValue(bool bIsSaveToFlash, bool bIsEffectNow,  uint8_t uchValue);
uint8_t uchDevCfgGetLossValue(void);

DEV_CFG_STATUS_RET nDevCfgSetAckPayload(bool bIsSaveToFlash, uint8_t uchLen, uint8_t *puchPayload); /*����ACK PayLoad*/
DEV_CFG_STATUS_RET nDevCfgGetAckPayload(uint8_t *puchLen, uint8_t *puchPayload); /*��ȡACK PayLoad*/

DEV_CFG_STATUS_RET nDevCfgSetRecvAddr(bool bIsSaveToFlash, uint8_t uchLen, uint8_t *puchAddr);
DEV_CFG_STATUS_RET nDevCfgGetRecvAddr(uint8_t *puchLen, uint8_t *puchAddr);

DEV_CFG_STATUS_RET nDevCfgSetTriggerMode(bool bIsSaveToFlash, LOW_FREQ_TIRGGER_TYPE triggerMode);
LOW_FREQ_TIRGGER_TYPE uchDevCfgGetTriggerMode(void);


DEV_CFG_STATUS_RET nDevCfgSetTriggerLowFreq(bool bIsSaveToFlash, uint8_t uchFreq);
uint8_t uchDevCfgGetTriggerLowFreq(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerInterval(bool bIsSaveToFlash, uint8_t uchInterval);
uint8_t uchDevCfgGetTriggerInterval(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerDistance(bool bIsSaveToFlash, uint8_t uchDistance);
uint8_t uchDevCfgGetTriggerDistance(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerSubAddr(bool bIsSaveFlash, uint8_t uchSubAddr);
uint8_t uchDevCfgGetTriggerSubAddr(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerActiveFlg(bool bIsSaveFlash, uint8_t uchFlg);
uint8_t uchDevCfgTriggerActiveFlg(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerDistance(bool bIsSaveToFlash, uint8_t uchDistance);
uint8_t uchDevCfgGetTriggerDistance(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerInOutInterval(bool bIsSaveFlash, uint8_t uchInterval);
uint8_t uchDevCfgTriggerInOutInterval(void);

DEV_CFG_STATUS_RET nDevCfgSetTriggerDuty(bool bIsSaveFlash, uint8_t uchDuty);
uint8_t uchDevCfgGetTriggerDuty(void);

#ifdef __cplusplus
}
#endif

#endif

