#include "rc522.h"
#include "uart.h"
#include <string.h>

CARD_STRUCT card_inf;  //lzy_n ȫ�ֱ�����Ѱ��
//CARD_STRUCT g_stCard_inf; 

#define rc522_spi_mode  0
#define rc522_dubug     1

#if rc522_spi_mode
		
#else
		#define	STU_522_MISO	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)
		
		#define SET_522_MOSI	GPIO_SetBits(GPIOA, GPIO_Pin_7)
		#define CLR_522_MOSI	GPIO_ResetBits(GPIOA, GPIO_Pin_7)
		
		#define SET_522_CK		GPIO_SetBits(GPIOA, GPIO_Pin_5)
		#define CLR_522_CK		GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#endif

		#define SET_522_CS		GPIO_SetBits(GPIOA, GPIO_Pin_4)
		#define CLR_522_CS		GPIO_ResetBits(GPIOA, GPIO_Pin_4)

		#define SET_522_RST		GPIO_SetBits(GPIOA, GPIO_Pin_12)
		#define CLR_522_RST		GPIO_ResetBits(GPIOA, GPIO_Pin_12)

/**********************************************************************
**����ԭ��:   void RC522_Delay(uint16_t cnt)
**��ڲ���:   ��ʱʱ��
**�� �� ֵ:   ��
**��    ��:   RC522��ʱ
**˵    ��:   
************************************************************************/
static void RC522_Delay(uint16_t cnt)  
{
		cnt *=48; //��Ƶ48M
		while(cnt--);
}

#if rc522_spi_mode
/**********************************************************************
**����ԭ��:   void spi_master_init(void)
**��ڲ���:   NULL
**�� �� ֵ:   NULL
**��    ��:   RC522 SPI��ʼ��
**˵    ��:   
************************************************************************/
static void spi_master_init(void)
{		
			GPIO_InitTypeDef GPIO_InitStructure;
			SPI_InitTypeDef  SPI_InitStructure;

			RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

			/* SPI1 �ܽŸ��ù��� */
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
			
			/* SPI SCK, MISO, MOSI pin configuration */
			GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			
			//CSN2->PA4	
			GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			//RST->PA12	
			GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			
			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
			SPI_InitStructure.SPI_Mode      = SPI_Mode_Master;  //����SPI����ģʽ:����Ϊ��SPI
			SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b;  //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
			SPI_InitStructure.SPI_CPOL      = SPI_CPOL_Low;	 	  //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
			SPI_InitStructure.SPI_CPHA      = SPI_CPHA_1Edge;	  //����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
			SPI_InitStructure.SPI_NSS 			= SPI_NSS_Soft;		  //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ32
			SPI_InitStructure.SPI_FirstBit  = SPI_FirstBit_MSB; //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
			SPI_InitStructure.SPI_CRCPolynomial = 7; //CRCֵ����Ķ���ʽ
			SPI_Init(SPI1, &SPI_InitStructure); //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
			
			SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
		 
			SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����			
}
#else
static void spi_master_init(void)
{		
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 |GPIO_Pin_12;   
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;   
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;    	
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
}
#endif

#if rc522_spi_mode
/**********************************************************************
**����ԭ��:   uint8_t SPI1_RW_Byte(uint8_t TxData)
**��ڲ���:   ��������
**�� �� ֵ:   ��ȡ����
**��    ��:   ��дSPI
**˵    ��:   
************************************************************************/
static uint8_t SPI1_RW_Byte(uint8_t TxData)
{				 	
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{

		}
		SPI_SendData8(SPI1, TxData); //ͨ������SPIx����һ������

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{

		}
		return SPI_ReceiveData8(SPI1); //����ͨ��SPIx������յ�����				 	    
}

/**********************************************************************
**����ԭ��:   uint8_t spi_write_reg(uint8_t ucRegAddr, uint8_t ucRegVal)
**��ڲ���:   ucRegAddr��д��ַ��ucRegVal��д����
**�� �� ֵ:   true or false
**��    ��:   д�Ĵ���
**˵    ��:   
************************************************************************/
static uint8_t SPIReadByte(void)
{                                                        
		return (SPI1_RW_Byte(0xFF));                                              
}

/**********************************************************************
**����ԭ��:   static uint8_t spi_read_reg(uint8_t ucRegAddr)
**��ڲ���:   ucRegAddr������ַ
**�� �� ֵ:   �Ĵ���ֵ
**��    ��:   ���Ĵ���
**˵    ��:   
************************************************************************/
static void SPIWriteByte(uint8_t SPIData)
{
		SPI1_RW_Byte(SPIData);	
}
#else
/**********************************************************************
**����ԭ��:   uint8_t SPIReadByte(void)
**��ڲ���:   ��
**�� �� ֵ:   ������
**��    ��:   SPI��  
**˵    ��:   
************************************************************************/
static uint8_t SPIReadByte(void)
{
    uint8_t SPICount = 0;                                      		  
    uint8_t SPIData  = 0;                  
    
		for(; SPICount < 8; SPICount++)                  
    {
        SPIData <<=1;                                            
        CLR_522_CK; 
				RC522_Delay(1);                             								  
					
				if(STU_522_MISO)
        {
            SPIData|=0x01;
        }
        
				SET_522_CK;   
				RC522_Delay(1);                                              	
   }                                                              
   
	 return (SPIData);                                              
}

/**********************************************************************
**����ԭ��:   void SPIWriteByte(uint8_t SPIData)
**��ڲ���:   д����
**�� �� ֵ:   ��
**��    ��:   SPIд
**˵    ��:   
************************************************************************/
static void SPIWriteByte(uint8_t SPIData)
{
    uint8_t SPICount = 0;                                       			
    
		for(; SPICount < 8; SPICount++)
    {
        if (SPIData & 0x80)
        {
            SET_522_MOSI;
        }  
        else
        {
            CLR_522_MOSI;
        }
				RC522_Delay(1);   
        
				CLR_522_CK;
				RC522_Delay(1);   
        
				SET_522_CK;
				RC522_Delay(1);   

				SPIData <<= 1;
    }	
}
#endif

/**********************************************************************
**����ԭ��:   uint8_t ReadRawRC(uint8_t Address)
**��ڲ���:  	Address���Ĵ�����ַ
**�� �� ֵ:   �Ĵ���ֵ
**��    ��:   ��RC522�Ĵ���
**˵    ��:   
************************************************************************/
static uint8_t ReadRawRC(uint8_t  Address)
{
    uint8_t ucAddr   = ((Address <<1) &0x7E) |0x80;
    uint8_t ucResult = 0;
		
		CLR_522_CS;
		SPIWriteByte(ucAddr);
		ucResult = SPIReadByte();
		SET_522_CS;
	
		return ucResult;
}

/**********************************************************************
**����ԭ��:   void WriteRawRC(uint8_t Address, uint8_t value)
**��ڲ���:  	Address���Ĵ�����ַ��value��д���ֵ
**�� �� ֵ:   ��
**��    ��:   дRC522�Ĵ���
**˵    ��:   
************************************************************************/
static void WriteRawRC(uint8_t  Address, uint8_t  value)
{  
    uint8_t ucAddr = (Address <<1) &0x7E;

		CLR_522_CS;
		SPIWriteByte(ucAddr);
		SPIWriteByte(value);
		SET_522_CS;
}

/**********************************************************************
**����ԭ��:   void SetBitMask(uint8_t reg, uint8_t mask) 
**��ڲ���:  	reg���Ĵ�����ַ��mask����λֵ
**�� �� ֵ:   ��
**��    ��:   ��RC522�Ĵ���λ
**˵    ��:   
************************************************************************/
static void SetBitMask(uint8_t reg, uint8_t mask)  
{
    char tmp = ReadRawRC(reg);
 
    WriteRawRC(reg, tmp | mask);
}

/**********************************************************************
**����ԭ��:   void ClearBitMask(uint8_t reg, uint8_t mask) 
**��ڲ���:  	reg���Ĵ�����ַ��mask����λֵ
**�� �� ֵ:   ��
**��    ��:   ��RC522�Ĵ���λ
**˵    ��:   
************************************************************************/
static void ClearBitMask(uint8_t reg, uint8_t mask)  
{
    char tmp = ReadRawRC(reg);
    
    WriteRawRC(reg, tmp & (~mask));
} 

/**********************************************************************
**����ԭ��:   void CalulateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
**��ڲ���:  	pIndata���������ݣ�len�����ݳ��ȣ�pOutData���������
**�� �� ֵ:   ��
**��    ��:   ����CRC16
**˵    ��:   
************************************************************************/ 
static void CalulateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
    uint8_t i=0, n=0;
  
		ClearBitMask(DivIrqReg,  0x04);
    WriteRawRC(CommandReg,   PCD_IDLE);
    SetBitMask(FIFOLevelReg, 0x80);
    
		for(; i<len; i++)
    {   
				WriteRawRC(FIFODataReg, *(pIndata+i));
		}
    WriteRawRC(CommandReg, PCD_CALCCRC);
    
		i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while((i!=0) && !(n&0x04));
		
		pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
}

/**********************************************************************
**����ԭ��:   char PcdComMF522(uint8_t  Command, 
															uint8_t  *pInData, 
															uint8_t  InLenByte,
															uint8_t  *pOutData, 
															uint32_t *pOutLenBit) 
**��ڲ���:  	Command��RC522�����֣�pInData�����͵���Ƭ�����ݣ�InLenByte���������ݵ��ֽڳ��ȣ�pOutData�����տ�Ƭ�ķ������ݣ�pOutLenBit���������ݵĳ���
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   RC522�뿨ͨ��
**˵    ��:   
************************************************************************/
static char PcdComMF522(uint8_t  Command, 
											  uint8_t  *pInData, 
												uint8_t  InLenByte,
											  uint8_t  *pOutData, 
											  uint16_t *pOutLenBit)
{
    char status = MI_ERR;
    uint8_t  irqEn    = 0x00;
    uint8_t  waitFor  = 0x00;
    uint8_t  lastBits = 0x00;
    uint8_t  n = 0;
    uint16_t i = 0;
    
	switch (Command)
    {
				case PCD_AUTHENT:
						irqEn   = 0x12;
						waitFor = 0x10;
						break;
		
				case PCD_TRANSCEIVE:
						irqEn   = 0x77;
						waitFor = 0x30;
						break;
		
				default:
						break;
    }
   
		WriteRawRC(ComIEnReg, 	 irqEn|0x80);
    ClearBitMask(ComIrqReg,  0x80);
    WriteRawRC(CommandReg, 	 PCD_IDLE);
    SetBitMask(FIFOLevelReg, 0x80);
		
    for(; i<InLenByte; i++)
    {   
				WriteRawRC(FIFODataReg, pInData[i]);
		}
		WriteRawRC(CommandReg, Command);
			
    if(Command == PCD_TRANSCEIVE)
    {    
				SetBitMask(BitFramingReg, 0x80);  
		}
		
		i = 6000;
    do 
    {
        n = ReadRawRC(ComIrqReg);
        i--;
    }
    while((i!=0) && !(n&0x01) && !(n&waitFor));
		ClearBitMask(BitFramingReg, 0x80);
		
    if(i!=0)
    {    
				if(!(ReadRawRC(ErrorReg)&0x1B))
        {
						status = MI_OK;
            
						if(n & irqEn & 0x01)
            {   
								status = MI_NOTAGERR;   
						}
            
						if(Command == PCD_TRANSCEIVE)
            {
								n = ReadRawRC(FIFOLevelReg);
              	lastBits = ReadRawRC(ControlReg) & 0x07;
							
								if(lastBits)
                {   
										*pOutLenBit = (n-1)*8 + lastBits;   
								}
                else
                {   
										*pOutLenBit = n*8;   
								}
                
								if (n == 0)
                {   
										n = 1;    
								}
								
								if(n > MAXRLEN)
                {   
										n = MAXRLEN;   
								}
								
								for(i=0; i<n; i++)
                {   
										pOutData[i] = ReadRawRC(FIFODataReg);    
								}
            }
        }
        else
        {   
						status = MI_ERR;   
				}
        
    }
   
    SetBitMask(ControlReg, 0x80); // stop timer now
    WriteRawRC(CommandReg, PCD_IDLE); 
    
		return status;	
}

/**********************************************************************
**����ԭ��:   char PcdRequest(uint8_t req_code, uint8_t *pTagType)
**��ڲ���:   req_code��Ѱ����ʽ��pTagType����Ƭ���ʹ���
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   Ѱ��  
**˵    ��:   
************************************************************************/
static char PcdRequest(uint8_t req_code, uint8_t *pTagType)
{
		char  	 status = MI_ERR;  
		uint16_t unLen  = 0;
		uint8_t  ucComMF522Buf[MAXRLEN] = {0}; 

		ClearBitMask(Status2Reg,  0x08);
		WriteRawRC(BitFramingReg, 0x07);
		SetBitMask(TxControlReg,  0x03);
		
		ucComMF522Buf[0] = req_code;

		status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);

		if ((status == MI_OK) && (unLen == 0x10))
		{    
				*pTagType     = ucComMF522Buf[0];
				*(pTagType+1) = ucComMF522Buf[1];
		}
		else
		{   
				status = MI_ERR;   
		}
   
		return status;	
}

/**********************************************************************
**����ԭ��:   char PcdAnticoll(uint8_t *pSnr)
**��ڲ���:   rpSnr����Ƭ���к�
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ����ײ
**˵    ��:   
************************************************************************/
static char PcdAnticoll(uint8_t *pSnr)
{
    char  	 status = MI_ERR;
    uint8_t  i=0, snr_check=0;
    uint16_t unLen = 0;
    uint8_t  ucComMF522Buf[MAXRLEN] = {0}; 
    

    ClearBitMask(Status2Reg, 0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,    0x80);
 
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

    if(status == MI_OK)
    {
    	 for(; i<4; i++)
       {   
           *(pSnr+i)  = ucComMF522Buf[i];
           snr_check ^= ucComMF522Buf[i];
       }
       if(snr_check != ucComMF522Buf[i])
       {   
				   status = MI_ERR;    
			 }
    }
    
    SetBitMask(CollReg, 0x80);
    return status;
}

/**********************************************************************
**����ԭ��:   char PcdSelect(uint8_t *pSnr)
**��ڲ���:   rpSnr����Ƭ���к�
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ѡ����Ƭ
**˵    ��:   
************************************************************************/
static char PcdSelect(uint8_t *pSnr)
{
    char  	 status = MI_ERR;
    uint8_t  i      = 0;
    uint16_t unLen  = 0;
    uint8_t  ucComMF522Buf[MAXRLEN] = {0}; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for(; i<4; i++)
    {
				ucComMF522Buf[i+2] = *(pSnr+i);
				ucComMF522Buf[6]  ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]);
  
    ClearBitMask(Status2Reg, 0x08);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);
    
    if((status == MI_OK) && (unLen == 0x18))
    {   
				status = MI_OK;  
		}
    else
    {   
				status = MI_ERR;    
		}

    return status;
}

/**********************************************************************
**����ԭ��:   char PcdAuthState(uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pSnr)
**��ڲ���:   auth_mode��������֤ģʽ��addr�����ַ��pKey�����룻pSnr����Ƭ���к�
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ��֤��Ƭ��Կ
**˵    ��:   
************************************************************************/             
static char PcdAuthState(uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pSnr)
{
    char  		status = MI_ERR;
    uint16_t  unLen  = 0;
    uint8_t   i=0, ucComMF522Buf[MAXRLEN] = {0}; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for(i=0; i<6; i++)
    {    
				ucComMF522Buf[i+2] = *(pKey+i);   
		}
    for(i=0; i<4; i++)
    {    
				ucComMF522Buf[i+8] = *(pSnr+i);   
		}
 
    status = PcdComMF522(PCD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &unLen);
    if((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {   
				status = MI_ERR;   
		}
    
    return status;	
}

/**********************************************************************
**����ԭ��:   char PcdRead(uint8_t addr, uint8_t *pData)
**��ڲ���:  	addr�����ַ��pData������������
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ��ȡM1��һ������
**˵    ��:   
************************************************************************/
static char PcdRead(uint8_t addr, uint8_t *pData)
{
    char     status = MI_ERR;
    uint16_t unLen  = 0;
    uint8_t  i=0, ucComMF522Buf[MAXRLEN] = {0}; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
   
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    if((status == MI_OK) && (unLen == 0x90))
    {
        for(i=0; i<16; i++)
        {    
						*(pData+i) = ucComMF522Buf[i];   
				}
    }
    else
    {   
				status = MI_ERR;   
		}
    
    return status;
		
}

/**********************************************************************
**����ԭ��:   char PcdWrite(uint8_t addr, uint8_t *pData)
**��ڲ���:  	addr�����ַ��pData��д�������
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   д���ݵ�M1��һ��
**˵    ��:   
************************************************************************/                 
static char PcdWrite(uint8_t addr, uint8_t *pData)
{
    char     status = MI_ERR;
    uint16_t unLen  = 0;
		uint8_t  ucComMF522Buf[MAXRLEN] = {0}; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   
				status = MI_ERR;   
		}
        
    if(status == MI_OK)
    {
        memcpy(ucComMF522Buf, pData, 16);
        CalulateCRC(ucComMF522Buf, 16, &ucComMF522Buf[16]);

        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf, 18, ucComMF522Buf, &unLen);
				if((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {   
						status = MI_ERR;   
				}
    }
 
    return status;		
}

/**********************************************************************
**����ԭ��:   char PcdHalt(void)
**��ڲ�����  ��
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ���Ƭ��������״̬
**˵    ��:   
************************************************************************/ 
static char PcdHalt(void)
{
    char     status = MI_ERR;
    uint16_t unLen  = 0;
    uint8_t  ucComMF522Buf[MAXRLEN] = {0}; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);
 
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
		
		return status;
}

/**********************************************************************
**����ԭ��:   char PcdReset(void)
**��ڲ���:  	��
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ��λ
**˵    ��:   
************************************************************************/
static char PcdReset(void)
{
	SET_522_RST;
	RC522_Delay(10);
	CLR_522_RST;
	RC522_Delay(10);
	SET_522_RST;
	RC522_Delay(10);

	WriteRawRC(CommandReg, PCD_RESETPHASE);
	WriteRawRC(ModeReg,      0x3D);
	WriteRawRC(TReloadRegL,  30);
	WriteRawRC(TReloadRegH,  0);
	WriteRawRC(TModeReg,     0x8D);
	WriteRawRC(TPrescalerReg,0x3E);
	WriteRawRC(TxAutoReg,    0x40);

	return MI_OK;
}

/**********************************************************************
**����ԭ��:   void PcdAntennaOn(void)
**��ڲ���:  	��
**�� �� ֵ:   ��
**��    ��:   ��������
**˵    ��:   
************************************************************************/
//static void PcdAntennaOn(void)
//{ 
//		uint8_t i = ReadRawRC(TxControlReg);
//		
//    if(!(i & 0x03))
//    {
//        SetBitMask(TxControlReg, 0x03);
//    }
//}

/**********************************************************************
**����ԭ��:   void PcdAntennaOn(void)
**��ڲ���:  	��
**�� �� ֵ:   ��
**��    ��:   �ر�����
**˵    ��:   
************************************************************************/
static void PcdAntennaOff(void)
{
		ClearBitMask(TxControlReg, 0x03);
}

/**********************************************************************
**����ԭ��:   char M500PcdConfigISOType(uint8_t type)
**��ڲ���:  	type��Э������
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   ����RC522�Ĺ�����ʽ
**˵    ��:   
************************************************************************/
static char M500PcdConfigISOType(uint8_t type)
{
   if(type == 'A')                      //ISO14443_A
   { 
			 ClearBitMask(Status2Reg,  0x08);
		   WriteRawRC(ModeReg,       0x3D);	//3F
       WriteRawRC(RxSelReg,      0x86);	//84
		   WriteRawRC(RFCfgReg,      0x7F); //4F
		   WriteRawRC(TReloadRegL,   30);	  //tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
		   WriteRawRC(TReloadRegH,   0);
		   WriteRawRC(TModeReg,      0x8D);
		   WriteRawRC(TPrescalerReg, 0x3E);
   }
   else
	 { 
			return 1; 
   }
   
   return MI_OK; 
}

/**********************************************************************
**����ԭ��:   char RC522_Pdown(void)
**��ڲ���:  	��
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   �ر�RC522
**˵    ��:   
************************************************************************/ 
static char RC522_Pdown(void)
{
    PcdHalt();
		
    CLR_522_RST;

		return MI_OK;
}

/**********************************************************************
**����ԭ��:   void RC522_Init(void)
**��ڲ���:  	��
**�� �� ֵ:   ��
**��    ��: 	��ʼ��RC522  
**˵    ��:   
************************************************************************/ 
void RC522_Init(void)
{
    spi_master_init();
				
		PcdReset();
    PcdAntennaOff();
    M500PcdConfigISOType('A');
		RC522_Delay(10);
}

/**********************************************************************
**����ԭ��:   uint8_t RE_Pcd(uint8_t *MLastSelectedSnr, uint8_t *RevBuffer)
**��ڲ���:  	MLastSelectedSnr�����ؿ����кţ�RevBuffer�����ؿ�����
**�� �� ֵ:   �ɹ�����MI_OK
**��    ��:   Ѱ��
**˵    ��:   
************************************************************************/
static uint8_t RE_Pcd(uint8_t *MLastSelectedSnr, uint8_t *RevBuffer)
{
		char status = MI_OK;	
	
		PcdReset();
		
		status = PcdRequest(PICC_REQALL, RevBuffer);   

		if(status != MI_OK) 
		{
				return status;	
		}
		
		status = PcdAnticoll(MLastSelectedSnr);      
		if(status != MI_OK) 
		{
				return status;
		}
		
		status=PcdSelect(MLastSelectedSnr);         
		if(status != MI_OK) 
		{
				return status;
		}
		
		return status;	
}

/**********************************************************************
**����ԭ��:   uint8_t RE_Sector(uint8_t Block_No, uint8_t *p)
**��ڲ���:  	Block_No�������ţ�p����������
**�� �� ֵ:   �����ɹ���񣬷���0���ɹ�,����1ʧ��
**��    ��:   ����
**˵    ��:   
************************************************************************/
static uint8_t RE_Sector(uint8_t Block_No, uint8_t *p)
{
		char    status = MI_ERR;		
		uint8_t i = 0;
	
		for(; i<4; i++)
		{
				status = PcdRead(Block_No, p);		              
				if(status == MI_OK)
				{
						break;
				}
				if(i >= 3)
				{
						RC522_Pdown();
						return status;;		
				}	

		}
	
		if(status == MI_OK)
		{
				Block_No++;	                                  
				for(i=0; i<4; i++)
				{
						status = PcdRead(Block_No, p+16);		              
						if(status == MI_OK)
						{
								break;
						}
						if(i >= 3)
						{
								RC522_Pdown();
								return status;;		
						}	
				}
		}

		if(status == MI_OK)
		{
				Block_No++;	                                  
				for(i=0; i<4; i++)
				{
						status = PcdRead(Block_No, p+32);		               
						if(status == MI_OK)
						{
								break;
						}
						if(i >= 3)
						{
								RC522_Pdown();
								return status;;		
						}	
				}
		}
	  
		if(status != MI_OK)
		{
				RC522_Pdown();
				return status;
		}
	
		RC522_Pdown();
		return MI_OK;
}

/**********************************************************************
**����ԭ��:  uint8_t SE_Card(void)
**��ڲ���:  ��
**�� �� ֵ:  �����ɹ���񣬷���0���ɹ�,����1ʧ��
**��    ��:  Ѱ��    
**˵    ��:   
************************************************************************/
static uint8_t SE_Card(void)
{
		char status = MI_OK;
	
		//PcdReset(); //lzy_n ���溯���Ѿ���ʼ����
		status = RE_Pcd((void *)&card_inf.serialno[0], (void *)&card_inf.type[0]);
		//RC522_Pdown();
		
		if(status != MI_OK)
		{
				memset((void *)&card_inf.serialno[0], 0, 4);
				memset((void *)&card_inf.type[0], 0, 2);
				return status;
		}
	
#if rc522_dubug	
		UART_TTL_SendString(&card_inf.serialno[0], 4);	
		UART_TTL_SendString(&card_inf.type[0], 2);
#endif
		
		return status;	
}

/**********************************************************************
**����ԭ��:  uint8_t RE_Card(uint8_t Sector, uint8_t *card_key)
**��ڲ���:  Sector��������M1����0-15
**�� �� ֵ:  �����ɹ���񣬷���0���ɹ�,����1ʧ��
**��    ��:  ����  
**˵    ��:   
************************************************************************/
uint8_t RE_Card(uint8_t Sector, uint8_t *card_key)
{	
		uint8_t BlockNo = 0;
		char status = SE_Card();

		if (status != MI_OK)
		{
				return MI_ERR;
		}
	
		BlockNo = Sector<<2;

		status = PcdAuthState(PICC_AUTHENT1A, BlockNo, card_key, (void *)&card_inf.serialno[0]);
		
		if(status == MI_OK)
		{
				status = RE_Sector(BlockNo, (void *)&card_inf.rblockdata[0]);
				
#if rc522_dubug
				UART_TTL_SendString(&card_inf.rblockdata[0], 48);
#endif
		}		

		if (status != MI_OK)
		{
				return MI_ERR;
		}
		
		return status;			
}

/**********************************************************************
**����ԭ��:  uint8_t WR_Card(uint8_t BlockNo, uint8_t *Data, uint8_t *card_key)
**��ڲ���:  BlockNO����ţ�Data����д������
**�� �� ֵ:  �����ɹ���񣬷���0���ɹ�,����1ʧ��
**��    ��:  д��  
**˵    ��:   
************************************************************************/
uint8_t WR_Card(uint8_t BlockNo, uint8_t *Data, uint8_t *card_key)
{
		char status = SE_Card();
		
		if (status != MI_OK) return MI_ERR;

		status = PcdAuthState(PICC_AUTHENT1A, BlockNo, card_key, (void *)&card_inf.serialno[0]);
	
		if(status == MI_OK)
		{
				status = PcdWrite(BlockNo, Data); 
				
				if (status != MI_OK)
				{
						return MI_ERR;
				}
		}
		
		RE_Card(BlockNo/4, card_key);
		
		return status;	
}

/**********************************************************************
**����ԭ��:  uint8_t RE_Card(uint8_t Sector, uint8_t *card_key)
**��ڲ���:  Sector��������M1����0-15
**�� �� ֵ:  �����ɹ���񣬷���0���ɹ�,����1ʧ��
**��    ��:  ����  
**˵    ��:   
************************************************************************/
void FD_Card(void)
{
		SE_Card();
}
