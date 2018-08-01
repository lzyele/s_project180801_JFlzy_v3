#include "nrf24l01.h"
#include "uart.h"
#include <stdbool.h>
#include <string.h>

#ifndef enable
#define enable  1
#endif

#ifndef disable
#define disable 0
#endif

// SPI(nRF24L01) commands ,	NRF��SPI����궨�壬���NRF����ʹ���ĵ�
#define NRF_READ_REG  0x00  // Define read command to register
#define NRF_WRITE_REG 0x20  // Define write command to register
#define R_RX_PL_WID		0x60  // Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define RD_RX_PLOAD 	0x61  // Define RX payload register address
#define WR_TX_PLOAD 	0xA0  // Define TX payload register address
#define FLUSH_TX    	0xE1  // Define flush TX register command
#define FLUSH_RX    	0xE2  // Define flush RX register command
#define REUSE_TX_PL 	0xE3  // Define reuse TX payload register command
#define NOP        	 	0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses) ��NRF24L01 ��ؼĴ�����ַ�ĺ궨��
#define CONFIG      	0x00  // 'Config' register address
#define EN_AA       	0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   	0x02  // 'Enabled RX addresses' register address
#define SETUP_AW    	0x03  // 'Setup address width' register address
#define SETUP_RETR  	0x04  // 'Setup Auto. Retrans' register address
#define RF_CH       	0x05  // 'RF channel' register address
#define RF_SETUP    	0x06  // 'RF setup' register address
#define STATUS      	0x07  // 'Status' register address
#define OBSERVE_TX  	0x08  // 'Observe TX' register address
#define CD          	0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0  	0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1  	0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2  	0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3  	0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4  	0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5  	0x0F  // 'RX address pipe5' register address
#define TX_ADDR     	0x10  // 'TX address' register address
#define RX_PW_P0    	0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1    	0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2    	0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3    	0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4    	0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5    	0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS 	0x17  // 'FIFO Status Register' register address

#define FEATURE     	0x1D  //lzy_add  Feature Register
#define DYNPD       	0x1C  //lzy_add Enable dynamic payload length

#define MAX_RT      	0x10  //�ﵽ����ط������жϱ�־λ
#define TX_DS			0x20  //��������жϱ�־λ	  // 
#define RX_DR			0x40  //���յ������жϱ�־λ

#define NRF_CSN_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define NRF_CSN_LOW()  GPIO_ResetBits(GPIOB, GPIO_Pin_12)		

#define NRF_CE_HIGH()	 GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define NRF_CE_LOW()	 GPIO_ResetBits(GPIOC, GPIO_Pin_7)			   

#define NRF_Read_IRQ() GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)

// SETUP_AW
#define NRF_REGF_AW					 0
#define NRF_REGF_AW_MASK    (0x3 << NRF_REGF_AW)

// RF_CH
#define NRF_REGF_RF_CH			 0
#define NRF_REGF_RF_CH_MASK (0x7f<<NRF_REGF_RF_CH)

// CONFIG  �������õ�һ���Ĵ��� CONFIG
#define NRF_REGF_PRIM_RX			 		   0
#define NRF_REGF_PWR_UP				 		   1
#define NRF_REGF_CRCO				   		   2
#define NRF_REGF_EN_CRC				 		   3
#define NRF_REGF_MASK_MAX_RT	 		   4
#define NRF_REGF_MASK_TX_DS		 		   5
#define NRF_REGF_MASK_RX_DR		 		   6
#define NRF_REGF_PRIM_RX_MASK			  (1<<NRF_REGF_PRIM_RX)
#define NRF_REGF_PWR_UP_MASK			  (1<<NRF_REGF_PWR_UP)
#define NRF_REGF_CRCO_MASK				  (1<<NRF_REGF_CRCO)
#define NRF_REGF_EN_CRC_MASK        (1<<NRF_REGF_EN_CRC)
#define NRF_REGF_MASK_MAX_RT_MASK	  (1<<NRF_REGF_MASK_MAX_RT)
#define NRF_REGF_MASK_TX_DS_MASK	  (1<<NRF_REGF_MASK_TX_DS)
#define NRF_REGF_MASK_RX_DR_MASK    (1<<NRF_REGF_MASK_RX_DR)

// RF_SETUP
#define NRF_REGF_LNA_HCURR				   0
#define NRF_REGF_RF_PWR						   1
#define NRF_REGF_RF_DR						   3
#define NRF_REGF_PLL_LOCK				     4
#define NRF_REGF_RF_DR_LOW           5
#define NRF_REGF_CONT_WAVE           7
#define NRF_REGF_LNA_HCURRMAST_MASK	(1<<NRF_REGF_LNA_HCURR)
#define NRF_REGF_RF_PWR_MASK				(0x03<<NRF_REGF_RF_PWR)
#define NRF_REGF_RF_DR_MASK					(1<<NRF_REGF_RF_DR)
#define NRF_REGF_PLL_LOCK_MASK			(1<<NRF_REGF_PLL_LOCK)
#define NRF_REGF_RF_DR_LOW_MASK     (1<<NRF_REGF_RF_DR_LOW)
#define NRF_REGF_CONT_WAVE_MASK     (1<<NRF_REGF_CONT_WAVE)

// STATUS
#define NRF_REGF_TX_FULL						 0
#define NRF_REGF_RX_P_NO						 1
#define NRF_REGF_MAX_RT							 4
#define NRF_REGF_TX_DS							 5
#define NRF_REGF_RX_DR							 6
#define NRF_REGF_TX_FULL_MASK	      (1<<NRF_REGF_TX_FULL)
#define NRF_REGF_RX_P_NO_MASK				(0x07<<NRF_REGF_RX_P_NO)
#define NRF_REGF_MAX_RT_MASK				(1<<NRF_REGF_MAX_RT)
#define NRF_REGF_TX_DS_MASK		    	(1<<NRF_REGF_TX_DS)
#define NRF_REGF_RX_DR_MASK		    	(1<<NRF_REGF_RX_DR)

// SETUP_RETR
#define NRF_REGF_ARC								 0
#define NRF_REGF_ARD								 4
#define NRF_REGF_ARC_MASK           (0xf<<NRF_REGF_ARC)
#define NRF_REGF_ARD_MASK           (0xf<<NRF_REGF_ARD)

// FEATURE
#define NRF_REGF_EN_DYN_ACK					 0
#define NRF_REGF_EN_ACK_PAY					 1
#define NRF_REGF_EN_DPL							 2
#define NRF_REGF_EN_DYN_ACK_MASK		(1<<NRF_REGF_EN_DYN_ACK)
#define NRF_REGF_EN_ACK_PAY_MASK		(1<<NRF_REGF_EN_ACK_PAY)
#define NRF_REGF_EN_DPL_MASK			  (1<<NRF_REGF_EN_DPL)

/*���üĴ���FIELD ֵ*/
#define NRF_GET_REG_FIELD_VALUE(regValue, fieldOffset, mask) \
															((regValue & mask)>>fieldOffset)
#define NRF_SET_REG_FIELD_VALUE(regValue, fieldOffset, mask, fieldValue) \
														  ((regValue&(~mask)) | (fieldValue<<fieldOffset))

// RF_SETUP
#define NRF_REGF_LNA_HCURR					0
#define NRF_REGF_RF_PWR						1
#define NRF_REGF_RF_DR						3
#define NRF_REGF_PLL_LOCK					4
#define NRF_REGF_RF_DR_LOW    				5
#define NRF_REGF_CONT_WAVE      			7

//static uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x11, 0x12, 0x13};
static uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x21, 0x22, 0x23};

NRF_PIPE_ADDR pstAddr;
volatile NRF_RX_STRCUT nrf_rx_inf;

/**********************************************************************
**����ԭ��:   void EXIT_NRF_Init(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   NRF�ⲿ�ж�
**˵    ��:   
************************************************************************/
static void EXIT_NRF_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct; 
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* config the extiline(PC6) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,    ENABLE);

	/* Configyre P[A|B|C|D|E]0  NIVC  */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	/* EXTI line gpio config(PC6) */	
	//IRQ->PC6�����ó�����
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 ;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);	

	/* EXTI line(PC6) mode config */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
	EXTI_InitStruct.EXTI_Line = EXTI_Line6;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct); 
}

/**********************************************************************
**����ԭ��:   void EXTI4_15_IRQHandler(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   NRF�ⲿ�жϴ�����
**˵    ��:   
************************************************************************/
void EXTI4_15_IRQHandler(void)
{
		if(EXTI_GetITStatus(EXTI_Line6) != RESET)
		{			
				nrf_rx_inf.rx_irq_flag = true;
			
				/* Clear the EXTI line 6 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line6);
		}
}

/**********************************************************************
**����ԭ��:   void NRF_Delay(uint16_t cnt)
**��ڲ���:   ��ʱʱ��
**�� �� ֵ:   ��
**��    ��:   NRF��ʱ
**˵    ��:   
************************************************************************/
//static void NRF_Delay(uint16_t cnt)  
//{
//		while(cnt--);
//}

/**********************************************************************
**����ԭ��:   void SPI2_Config(void)
**��ڲ���:   NULL
**�� �� ֵ:   NULL
**��    ��:   SPI2��ʼ��
**˵    ��:   
************************************************************************/
static void SPI2_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		SPI_InitTypeDef  SPI_InitStructure;

		RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE); //PORTBʱ��ʹ�� 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2ʱ��ʹ�� 

		/* SPI pin mappings */
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_0);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_0);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_0);
	
		/* MOSI2->PB15, MISO2->PB14, SPISCK2->PB13, */
		GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��GPIOB
	
		/*	CSN2->PB12	*/
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 ;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��GPIOB
		GPIO_SetBits(GPIOB,GPIO_Pin_12); 	     //SPIƬѡȡ�� 
	
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
		SPI_InitStructure.SPI_Mode      = SPI_Mode_Master; //����SPI����ģʽ:����Ϊ��SPI
		SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b; //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
		SPI_InitStructure.SPI_CPOL      = SPI_CPOL_Low;		 //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
		SPI_InitStructure.SPI_CPHA      = SPI_CPHA_1Edge;	 //����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		       //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
		SPI_InitStructure.SPI_CRCPolynomial = 7; 					 //CRCֵ����Ķ���ʽ
		SPI_Init(SPI2, &SPI_InitStructure);  							 //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
		SPI_RxFIFOThresholdConfig(SPI2,SPI_RxFIFOThreshold_QF);
 
		SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����		
}

/**********************************************************************
**����ԭ��:   uint8_t SPI2_RW_Byte(uint8_t TxData)
**��ڲ���:   ����������
**�� �� ֵ:   ��������
**��    ��:   SPI2��дһ���ֽ�
**˵    ��:   
************************************************************************/
static uint8_t SPI2_RW_Byte(uint8_t TxData)
{					 	
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{

		}
	
		SPI_SendData8(SPI2, TxData); //ͨ������SPIx����һ������

		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{

		}
		return SPI_ReceiveData8(SPI2); //����ͨ��SPIx������յ�����				 	    
}

/**********************************************************************
**����ԭ��:   void SPI_NRF_Init(void)
**��ڲ���:   NULL
**�� �� ֵ:   NULL
**��    ��:   NRF24L01 SPI��ʼ��
**˵    ��:   
************************************************************************/
void SPI_NRF_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
		//CE->PC7,���ó������ 
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_ResetBits(GPIOC, GPIO_Pin_7);
		
		SPI2_Config(); //��ʼ��SPI
}

/**********************************************************************
**����ԭ��:   uint8_t SPI_NRF_WriteReg(uint8_t reg, uint8_t dat)
**��ڲ���:   �Ĵ�����ַ,д������
**�� �� ֵ:   �Ĵ���״̬
**��    ��:   NRF24L01 д�Ĵ���
**˵    ��:   
************************************************************************/
static uint8_t SPI_NRF_WriteReg(uint8_t reg, uint8_t dat)
{
		uint8_t status;
		
		/*�õ�CSN��ʹ��SPI����*/
    NRF_CSN_LOW();
				
		/*��������Ĵ����� */
		status = SPI2_RW_Byte(reg);
		 
		/*��Ĵ���д������*/
    SPI2_RW_Byte(dat); 
	          
		/*CSN���ߣ����*/	   
  	NRF_CSN_HIGH();	
		
		/*����״̬�Ĵ�����ֵ*/
   	return(status);
}

/**********************************************************************
**����ԭ��:   uint8_t SPI_NRF_ReadReg(uint8_t reg)
**��ڲ���:   �Ĵ�����ַ
**�� �� ֵ:   �Ĵ���״̬
**��    ��:   NRF24L01 ���Ĵ���
**˵    ��:   
************************************************************************/
static uint8_t SPI_NRF_ReadReg(uint8_t reg)
{
		uint8_t reg_val;

		/*�õ�CSN��ʹ��SPI����*/
		NRF_CSN_LOW();
				
  	/*���ͼĴ�����*/
		SPI2_RW_Byte(reg); 

		/*��ȡ�Ĵ�����ֵ */
		reg_val = SPI2_RW_Byte(NOP);
	            
   	/*CSN���ߣ����*/
		NRF_CSN_HIGH();		
   	
		return reg_val;
}	

/**********************************************************************
**����ԭ��:   uint8_t SPI_NRF_ReadBuf(uint8_t reg, uint8_t bytes, uint8_t *pBuf)
**��ڲ���:   �Ĵ�����ַ,��ȡ�ֽ���,��ȡ����
**�� �� ֵ:   �Ĵ���״̬
**��    ��:   NRF24L01 ������
**˵    ��:   
************************************************************************/
static uint8_t SPI_NRF_ReadBuf(uint8_t reg, uint8_t bytes, uint8_t *pBuf)
{
		uint8_t status, byte_cnt;
	
		/*�õ�CSN��ʹ��SPI����*/
		NRF_CSN_LOW();
		
		/*���ͼĴ�����*/		
		status = SPI2_RW_Byte(reg); 

		/*��ȡ����������*/
		for(byte_cnt=0; byte_cnt<bytes; byte_cnt++)
		{
				pBuf[byte_cnt] = SPI2_RW_Byte(NOP); //��NRF24L01��ȡ����	 
		}

		/*CSN���ߣ����*/
		NRF_CSN_HIGH();	
		
		return status; //���ؼĴ���״ֵ̬
}

/**********************************************************************
**����ԭ��:   uint8_t SPI_NRF_WriteBuf(uint8_t reg, uint8_t bytes, uint8_t *pBuf)
**��ڲ���:   �Ĵ�����ַ,��ȡ�ֽ���,д������
**�� �� ֵ:   �Ĵ���״̬
**��    ��:   NRF24L01 д����
**˵    ��:   
************************************************************************/
static uint8_t SPI_NRF_WriteBuf(uint8_t reg, uint8_t bytes, uint8_t *pBuf)
{
		uint8_t status, byte_cnt;
	
   	/*�õ�CSN��ʹ��SPI����*/
		NRF_CSN_LOW();			

		/*���ͼĴ�����*/	
  	status = SPI2_RW_Byte(reg); 
 	
  	/*�򻺳���д������*/
		for(byte_cnt=0; byte_cnt<bytes; byte_cnt++)
		{
				SPI2_RW_Byte(*pBuf++); //д���ݵ������� 	 
	  }
		
		/*CSN���ߣ����*/
		NRF_CSN_HIGH();			
  
  	return status; //����NRF24L01��״̬ 		
}

/**********************************************************************
**����ԭ��:   void vNrfSetFreq(NRF_RF_CHANNEL Freq)
**��ڲ���:   Ƶ��
**�� �� ֵ:   ��
**��    ��:   ����Ƶ��
**˵    ��:   
************************************************************************/
void vNrfSetFreq(NRF_RF_CHANNEL Freq)
{
    Freq &= NRF_REGF_RF_CH_MASK; 
    SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH, Freq); //Ƶ�ʼĴ���
}

/**********************************************************************
**����ԭ��:   void vNrfSetTranPower(NRF_TRAN_POWER power)
**��ڲ���:   ����
**�� �� ֵ:   ��
**��    ��:   ���ù���
**˵    ��:   
************************************************************************/
void vNrfSetTranPower(NRF_TRAN_POWER power)
{
    uint8_t uchReg = 0;
		
    uchReg = SPI_NRF_ReadReg(RF_SETUP);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_PWR, NRF_REGF_RF_PWR_MASK, power); 
    SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP, uchReg);
}

/**********************************************************************
**����ԭ��:   void vNrfSetTranSpeed(NRF_TRAN_SPEED speed)
**��ڲ���:   ����
**�� �� ֵ:   ��
**��    ��:   ��������
**˵    ��:   
************************************************************************/
void vNrfSetTranSpeed(NRF_TRAN_SPEED speed)
{
    uint8_t uchReg = 0;
    
    uchReg = SPI_NRF_ReadReg(RF_SETUP);
    if(speed == NRF_TRAN_SPEED_1Mbps)
    {       
        uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_DR_LOW, NRF_REGF_RF_DR_LOW_MASK, 0); 
        uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_DR, NRF_REGF_RF_DR_MASK, 0); 
    }
    else if(speed == NRF_TRAN_SPEED_2Mbps)
    {
        uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_DR_LOW, NRF_REGF_RF_DR_LOW_MASK, 0); 
        uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_DR, NRF_REGF_RF_DR_MASK, 1); 
    }
    else
    {
        uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_DR_LOW, NRF_REGF_RF_DR_LOW_MASK, 1); 
        uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RF_DR, NRF_REGF_RF_DR_MASK, 0); 
    }
    SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP, uchReg);
}

/**********************************************************************
**����ԭ��:   void vNrfEnableRxPipe(NRF_PIPE_NUM enumPipe, bool uchIsEnable)
**��ڲ���:   �ܵ�;����
**�� �� ֵ:   ��
**��    ��:   �򿪻��߹ر�ĳ���ܵ�
**˵    ��:   
************************************************************************/
static void vNrfEnableRxPipe(NRF_PIPE_NUM enumPipe, bool uchIsEnable)
{
    uint8_t uchReg = 0;
    uint8_t uchMask = 0;
    
    uchReg = SPI_NRF_ReadReg(EN_RXADDR);
    uchMask = (1<<enumPipe);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, enumPipe, uchMask, uchIsEnable);  
    SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR, uchReg);
}

/**********************************************************************
**����ԭ��:   void vNrfEnablePipeAutoAck(NRF_PIPE_NUM enumPipe, bool uchIsEnable)
**��ڲ���:   �ܵ�;����
**�� �� ֵ:   ��
**��    ��:   �򿪻��߹ر�ĳ���ܵ����Զ��ظ�
**˵    ��:   
************************************************************************/
static void vNrfEnablePipeAutoAck(NRF_PIPE_NUM enumPipe, bool uchIsEnable)
{
    uint8_t uchReg = 0;
    uint8_t uchMask = 0;
    
    uchReg = SPI_NRF_ReadReg(EN_AA);
    uchMask = (1<<enumPipe);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, enumPipe, uchMask, uchIsEnable); 
    SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA, uchReg);
}

/**********************************************************************
**����ԭ��:   void nNrfSetOperationMode(NRF_OPT_MODE enumMode)
**��ڲ���:  	����ģʽ
**�� �� ֵ:   ��
**��    ��:   ���ù���ģʽ
**˵    ��:   
************************************************************************/
void nNrfSetOperationMode(NRF_OPT_MODE enumMode)
{
    uint8_t uchReg = 0;
    uchReg = SPI_NRF_ReadReg(CONFIG);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_PRIM_RX, NRF_REGF_PRIM_RX_MASK, enumMode);
    SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, uchReg);
}

/**********************************************************************
**����ԭ��:   void vNrfSetCrcPara(bool bIsEnable, NRF_CRC_TYPE crcType)
**��ڲ���:   ����;CRCģʽ
**�� �� ֵ:   ��
**��    ��:   CRC��������
**˵    ��:   
************************************************************************/
/*
EN_CRC 3 1 R/W Enable CRC. Forced high if one of the bits in the
EN_AA is high
CRCO 2 0 R/W CRC encoding scheme
'0' - 1 byte
'1' �C 2 bytes
*/
static void vNrfSetCrcPara(bool bIsEnable, NRF_CRC_TYPE crcType)
{
    uint8_t uchReg = 0;
    
    uchReg = SPI_NRF_ReadReg(CONFIG);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_EN_CRC, NRF_REGF_EN_CRC_MASK, bIsEnable);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_CRCO, NRF_REGF_CRCO_MASK, crcType);
    SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, uchReg);
}

/**********************************************************************
**����ԭ��:   void vNrfEnableIrq(bool bEnableRxDR, bool bEnableTxDS, bool bEnbaleMaxRT)
**��ڲ���:   �����жϿ���;
**�� �� ֵ:   ��
**��    ��:   �жϲ�������
**˵    ��:   
************************************************************************/
static void vNrfEnableIrq(bool bEnableRxDR, bool bEnableTxDS, bool bEnbaleMaxRT)
{
    uint8_t uchReg = 0;
    
    uchReg = SPI_NRF_ReadReg(CONFIG);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_RX_DR,  NRF_REGF_RX_DR_MASK,  !bEnableRxDR);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_TX_DS,  NRF_REGF_TX_DS_MASK,  !bEnableTxDS);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_MAX_RT, NRF_REGF_MAX_RT_MASK, !bEnbaleMaxRT);    
    SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, uchReg);    
}

/**********************************************************************
**����ԭ��:   void vNrfSetRetransmitPara(NRF_RT_DELAY enumDelay, NRF_RT_CONT uchCount)
**��ڲ���:   ��ʱʱ��;�ط�����
**�� �� ֵ:   ��
**��    ��:   �ط���������
**˵    ��:   
************************************************************************/
static void vNrfSetRetransmitPara(NRF_RT_DELAY enumDelay, NRF_RT_CONT uchCount)
{
    uint8_t uchReg = 0;
    
    uchReg = SPI_NRF_ReadReg(SETUP_RETR);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_ARC, NRF_REGF_ARC_MASK, uchCount); 
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_ARD, NRF_REGF_ARD_MASK, enumDelay);
    SPI_NRF_WriteReg(NRF_WRITE_REG+SETUP_RETR, uchReg);    
}

/**********************************************************************
**����ԭ��:   void  vNrfSetAddrWidth(NRF_ADDR_WIDTHS enumLenAddr)
**��ڲ���:   ��ַλ��
**�� �� ֵ:   ��
**��    ��:   ��ַ��Ȳ�������
**˵    ��:   
************************************************************************/
static void  vNrfSetAddrWidth(NRF_ADDR_WIDTHS enumLenAddr)
{
    uint8_t uchReg = 0;
    uchReg = SPI_NRF_ReadReg(SETUP_AW);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_AW, NRF_REGF_AW_MASK, enumLenAddr); 
    SPI_NRF_WriteReg(NRF_WRITE_REG+SETUP_AW, uchReg);
}

/**********************************************************************
**����ԭ��:   void nNrfSetRxPipeAddr(NRF_PIPE_NUM enumPipe, bool uchIsEnable, uint8_t *puchAddr, uint8_t uchLenAddr)
**��ڲ���:   ���չܵ�;����;��ַ;��ַ����
**�� �� ֵ:   ��
**��    ��:   ���ý��չܵ���ַ
**˵    ��:   
************************************************************************/
void nNrfSetRxPipeAddr(NRF_PIPE_NUM enumPipe, bool uchIsEnable, uint8_t *puchAddr, uint8_t uchLenAddr)
{
    vNrfEnableRxPipe(enumPipe, uchIsEnable);
    
    if(uchLenAddr != 0 && puchAddr != NULL)
    {	
        SPI_NRF_WriteBuf(NRF_WRITE_REG+(RX_ADDR_P0 + enumPipe), uchLenAddr, puchAddr);
    }
}

/**********************************************************************
**����ԭ��:   int nNrfSetTxAddr(uint8_t *puchAddr, uint8_t uchLenAddr)
**��ڲ���:   ���͵�ַ;��ַ����
**�� �� ֵ:   ��
**��    ��:   ���͵�ַ��������
**˵    ��:   
************************************************************************/
static void nNrfSetTxAddr(uint8_t *puchAddr, uint8_t uchLenAddr)
{
    SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR, uchLenAddr, puchAddr);
}

/**********************************************************************
**����ԭ��:   void vNrfEnableDynamicPayloadLen(NRF_PIPE_NUM enumPipe,  uint8_t uchIsEnable)
**��ڲ���:   �ܵ�;����
**�� �� ֵ:   ��
**��    ��:   ��̬���ղ�������
**˵    ��:   
************************************************************************/
/*Enable dynamic payload length*/
static void vNrfEnableDynamicPayloadLen(NRF_PIPE_NUM enumPipe,  uint8_t uchIsEnable)
{
    uint8_t uchReg = 0;
    uint8_t uchMask = 0; 

    uchReg = SPI_NRF_ReadReg(DYNPD);
    uchMask = (1<<enumPipe);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, enumPipe, uchMask, uchIsEnable); 
    SPI_NRF_WriteReg(NRF_WRITE_REG+DYNPD, uchReg);    
}

/**********************************************************************
**����ԭ��:   void vNrfEnableFeature( bool bIsEnDynAck,  bool bIsEnAckPay, bool bIsEnDPL)
**��ڲ���:   
**�� �� ֵ:   
**��    ��:   
**˵    ��:   
************************************************************************/
/*
EN_DPL 2 0 R/W Enables Dynamic Payload Length
EN_ACK_PAYd 1 0 R/W Enables Payload with ACK
EN_DYN_ACK 0 0 R/W Enables the W_TX_PAYLOAD_NOACK command
*/
static void vNrfEnableFeature( bool bIsEnDynAck,  bool bIsEnAckPay, bool bIsEnDPL)
{
    uint8_t uchReg = 0;
    
    uchReg = SPI_NRF_ReadReg(FEATURE);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_EN_DYN_ACK, NRF_REGF_EN_DYN_ACK_MASK, bIsEnDynAck);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_EN_ACK_PAY, NRF_REGF_EN_ACK_PAY_MASK, bIsEnAckPay);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_EN_DPL, NRF_REGF_EN_DPL_MASK, bIsEnDPL);    
    SPI_NRF_WriteReg(NRF_WRITE_REG+FEATURE, uchReg);
}

/**********************************************************************
**����ԭ��:   void nNrfSetPowerMode(NRF_POWER_MODE enumMode)
**��ڲ���:   ����ģʽ
**�� �� ֵ:   ��
**��    ��:   ��Դ����
**˵    ��:   
************************************************************************/
static void nNrfSetPowerMode(NRF_POWER_MODE enumMode)
{
    uint8_t uchReg = 0;
    
    uchReg = SPI_NRF_ReadReg(CONFIG);
    uchReg = NRF_SET_REG_FIELD_VALUE(uchReg, NRF_REGF_PWR_UP, NRF_REGF_PWR_UP_MASK, enumMode);
    SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, uchReg);
}

/**********************************************************************
**����ԭ��:   vNrfPresetEnhanceShortBurst
**��ڲ���:   
**�� �� ֵ:   ��
**��    ��:   ��������
**˵    ��:   
************************************************************************/
static void vNrfPresetEnhanceShortBurst(
								 NRF_OPT_MODE optMode, 
								 NRF_RF_CHANNEL rfCh, 
								 NRF_TRAN_SPEED tranSpeed, 
								 NRF_TRAN_POWER tranPower,
								 NRF_RT_CONT rtCount, 
								 NRF_RT_DELAY rtDelay, 
								 NRF_PIPE_ADDR *pstAddr)
{
    //uint8_t uchRecvAddr[4] = {0x00, 0x00, 0x00, 0x00};
    
	NRF_CE_LOW(); //�������ģʽ
		
    // Disable auto ACK on all pipes, except PIPE0
    vNrfEnablePipeAutoAck(NRF_RX_PIPE0, enable);
    vNrfEnablePipeAutoAck(NRF_RX_PIPE1, disable);
    vNrfEnablePipeAutoAck(NRF_RX_PIPE2, disable);
    vNrfEnablePipeAutoAck(NRF_RX_PIPE3, disable);
    vNrfEnablePipeAutoAck(NRF_RX_PIPE4, disable);
    vNrfEnablePipeAutoAck(NRF_RX_PIPE5, disable);   

    // Disable RX addresses, except PIPE0
    vNrfEnableRxPipe(NRF_RX_PIPE0, enable);
    vNrfEnableRxPipe(NRF_RX_PIPE1, disable);
    vNrfEnableRxPipe(NRF_RX_PIPE2, disable);
    vNrfEnableRxPipe(NRF_RX_PIPE3, disable);
    vNrfEnableRxPipe(NRF_RX_PIPE4, disable);
    vNrfEnableRxPipe(NRF_RX_PIPE5, disable);   

    // CONFIG - CRC enable, 2-Bit CRC, RX/TX mode
    // CONFIG - CRC enable, 2-Bit CRC, RX/TX mode, disable MAX_RT_IRQ + TX_DS/RX_DR
    vNrfSetCrcPara(enable, NRF_CRC_2BYTE);
    nNrfSetOperationMode(optMode);
    if(optMode == NRF_PRIM_TX)
    {
       vNrfEnableIrq(disable, enable, enable);
    }
    else
    {
       vNrfEnableIrq(enable, disable, enable);
    }
    // Enable auto retry and delay
    vNrfSetRetransmitPara(rtDelay, rtCount);    

    // Set address width to 5 bytes
    vNrfSetAddrWidth(NRF_ADDR_WIDTHS_3BYTE);
    nNrfSetRxPipeAddr(NRF_RX_PIPE0, enable, pstAddr->uchPipe0Addr, pstAddr->uchLen);
    
#if 0
    nNrfSetRxPipeAddr(NRF_RX_PIPE1, enable, pstAddr->uchPipe1Addr, pstAddr->uchLen);
    nNrfSetRxPipeAddr(NRF_RX_PIPE2, enable, pstAddr->uchPipe2Addr + pstAddr->uchLen-1, 1);
#endif
	
    nNrfSetTxAddr(pstAddr->uchPipe0Addr, pstAddr->uchLen);
    vNrfEnableDynamicPayloadLen(NRF_RX_PIPE0, enable);
	
#if 0
    vNrfEnableDynamicPayloadLen(NRF_RX_PIPE1, enable);
    vNrfEnableDynamicPayloadLen(NRF_RX_PIPE2, enable);
#endif

    // Set RF-channel
    vNrfSetFreq(rfCh);
     
    // Setup Data-Rate to 1MBit and RF power to 0db
    vNrfSetTranPower(tranPower);
    vNrfSetTranSpeed(tranSpeed);

    //enable feature
    #if 0
    vNrfEnableFeature(enable, enable, enable);
    #else
    vNrfEnableFeature(disable, enable, enable);
    #endif
	
    // Power up radio
    nNrfSetPowerMode(NRF_POWER_UP);

	NRF_CE_HIGH(); //�������״̬
}

/**********************************************************************
**����ԭ��:   void NRF_RX_Mode(NRF_TRAN_SPEED tranSpeed, NRF_TRAN_POWER tranPower)
**��ڲ���:   ��������;���书��
**�� �� ֵ:   ��
**��    ��:   ���ò��������ģʽ
**˵    ��:   
************************************************************************/
/*
static void NRF_RX_Mode(NRF_TRAN_SPEED tranSpeed, NRF_TRAN_POWER tranPower)
{
		NRF_CE_LOW();	
	
		SPI_NRF_WriteReg(NRF_WRITE_REG+SETUP_AW, 0x01);
	
		SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0, RX_ADR_WIDTH, RX_ADDRESS); //дRX�ڵ��ַ

		SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA, 0x01);    //ʹ��ͨ��0���Զ�Ӧ��    

		SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR, 0x01);//ʹ��ͨ��0�Ľ��յ�ַ    

		SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH, CHANAL);  //����RFͨ��Ƶ��    

		SPI_NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH); //ѡ��ͨ��0����Ч���ݿ�� 

		SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP, 0x06); //0x06����TX�������,0db����, 1Mbps,���������濪��
    
		SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 	0x0f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 

		SPI_NRF_WriteReg(NRF_WRITE_REG+7,0x0e);
	
		SPI_NRF_WriteReg(NRF_WRITE_REG+DYNPD,   0x01);  //ʹ�ܶ�̬��� 
		
		SPI_NRF_WriteReg(NRF_WRITE_REG+FEATURE, 0x04);  //ѡ��ͨ��0�Ķ�̬��� 
	
    NRF_CE_HIGH(); //CE���ߣ��������ģʽ
}    
*/

/**********************************************************************
**����ԭ��:   void NRF_TX_Mode(NRF_TRAN_SPEED tranSpeed, NRF_TRAN_POWER tranPower)
**��ڲ���:   ��������;���书��
**�� �� ֵ:   ��
**��    ��:   ���÷���ģʽ
**˵    ��:   
************************************************************************/
/*
static void NRF_TX_Mode(NRF_TRAN_SPEED tranSpeed, NRF_TRAN_POWER tranPower)
{  
		NRF_CE_LOW();		

		SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR, TX_ADR_WIDTH, TX_ADDRESS);    //дTX�ڵ��ַ 
	
		SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0, RX_ADR_WIDTH, RX_ADDRESS); //����RX�ڵ��ַ,��ҪΪ��ʹ��ACK   

		SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA, 0x01);      //ʹ��ͨ��0���Զ�Ӧ��    

		SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR, 0x01);  //ʹ��ͨ��0�Ľ��յ�ַ  

		SPI_NRF_WriteReg(NRF_WRITE_REG+SETUP_RETR, 0x1a); //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��

		SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH, CHANAL);    //����RFƵ��ͨ��ΪCHANAL
	
		//Setup Data-Rate to 1MBit and RF power to 0db

		SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP, 0x0f);   //����TX�������,0db����,2Mbps,���������濪��   
		vNrfSetTranPower(tranPower);
		vNrfSetTranSpeed(tranSpeed);

		SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0e);     //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�

		NRF_CE_HIGH(); //CE���ߣ��������ģʽ
}
*/

/**********************************************************************
**����ԭ��:   uint8_t NRF_Check(void)
**��ڲ���:   ��
**�� �� ֵ:   SUCCESS/ERROR ��������/����ʧ��
**��    ��:   ��Ҫ����NRF��MCU�Ƿ���������
**˵    ��:   
************************************************************************/
static uint8_t NRF_Check(void)
{
		uint8_t test_buf[5] = {0x55, 0x01, 0x02, 0x03, 0xAA};
		uint8_t read_buf[5] = {0};
		uint8_t i = 0; 
	 
		/*д��5���ֽڵĵ�ַ.  */  
		SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,5, test_buf);
	
		/*����д��ĵ�ַ */
		SPI_NRF_ReadBuf(TX_ADDR,5, read_buf); 
	 
		/*�Ƚ�*/               
		for(i=0;i<5;i++)
		{
				if(read_buf[i] != test_buf[i])
						break;
		} 
	       
		if(i==5)
		{
				return SUCCESS; //MCU��NRF�ɹ����� 
		}
		else
		{
				return ERROR;   //MCU��NRF����������
		}
}

/**********************************************************************
**����ԭ��:   uint8_t NRF_Tx_Dat(uint8_t *txbuf)
**��ڲ���:   �洢�˽�Ҫ���͵����ݵ�����
**�� �� ֵ:   ���ͽ�����ɹ�����TXDS,ʧ�ܷ���MAXRT��ERROR
**��    ��:   ������NRF�ķ��ͻ�������д������
**˵    ��:   
************************************************************************/
uint8_t NRF_Tx_Dat(uint8_t *txbuf)
{
		uint8_t state;  

		/*ceΪ�ͣ��������ģʽ1*/
		NRF_CE_LOW();

		/*д���ݵ�TX BUF ��� 32���ֽ�*/						
		SPI_NRF_WriteBuf(WR_TX_PLOAD,TX_PLOAD_WIDTH, txbuf);

    /*CEΪ�ߣ�txbuf�ǿգ��������ݰ� */   
		NRF_CE_HIGH();
	  	
	  /*�ȴ���������ж� */                            
		//while(NRF_Read_IRQ() != 0); 	
	
		/*��ȡ״̬�Ĵ�����ֵ */                              
		state = SPI_NRF_ReadReg(STATUS);

		/*���TX_DS��MAX_RT�жϱ�־*/                  
		SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS, state); 	

		SPI_NRF_WriteReg(FLUSH_TX, NOP); //���TX FIFO�Ĵ��� 

		/*�ж��ж�����*/    
		if(state & MAX_RT) 		 //�ﵽ����ط�����
		{
			 return MAX_RT; 
		}
		else if(state & TX_DS) //�������
		{	
				return TX_DS;
		}
		else
		{
			return ERROR;        //����ԭ����ʧ��
		}
} 

/**********************************************************************
**����ԭ��:   uint8_t NRF_Rx_Dat(volatile NRF_RX_STRCUT *rx_inf)
**��ڲ���:   ���ڽ��ո����ݵ�����
**�� �� ֵ:   ���ս��
**��    ��:   ���ڴ�NRF�Ľ��ջ������ж�������
**˵    ��:   
************************************************************************/
uint8_t NRF_Rx_Dat(volatile NRF_RX_STRCUT *rx_inf)
{
		uint8_t state; 
		
		/*�ȴ������ж�*/
		//while(NRF_Read_IRQ() != 0); 
	
		/*��ȡstatus�Ĵ�����ֵ  */               
		state = SPI_NRF_ReadReg(STATUS);
	 
		/* ����жϱ�־*/      
		SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS, state);
	
		/*�ж��Ƿ���յ�����*/
		if(state & RX_DR) //���յ�����
		{
				rx_inf->rx_len = SPI_NRF_ReadReg(R_RX_PL_WID);
				SPI_NRF_ReadBuf(RD_RX_PLOAD, rx_inf->rx_len, (void*)&rx_inf->rx_data[0]); //��ȡ����
		
				SPI_NRF_WriteReg(FLUSH_RX, NOP); //���RX FIFO�Ĵ���
				return RX_DR; 
		}
		else 
		{
				return ERROR; //û�յ��κ�����
		}
}

/**********************************************************************
**����ԭ��:   void nNrfInit(NRF_OPT_MODE optMode, NRF_RF_CHANNEL rfCh, NRF_TRAN_SPEED tranSpeed, NRF_TRAN_POWER tranPower)
**��ڲ���:   ����ģʽ;ͨ��Ƶ��;��������;���书��
**�� �� ֵ:   ��
**��    ��:   24L01�豸�ĳ�ʼ��
**˵    ��:   
************************************************************************/
void nNrfInit(NRF_OPT_MODE optMode, NRF_RF_CHANNEL rfCh, NRF_TRAN_SPEED tranSpeed, NRF_TRAN_POWER tranPower)
{
		SPI_NRF_Init();	
		EXIT_NRF_Init();
		
		//��֤�ܷ�������д
		while(NRF_Check() != SUCCESS){} //lzy_n ���ȵ���������Ϊֹ
	
		if(optMode == NRF_PRIM_RX)
		{	
				memcpy(pstAddr.uchPipe0Addr, RX_ADDRESS, RX_ADR_WIDTH);
				pstAddr.uchLen = RX_ADR_WIDTH;
				
				vNrfPresetEnhanceShortBurst(optMode, rfCh, tranSpeed, tranPower, NRF_RT_DISABLE, NRF_RT_DELAY_250, &pstAddr);														
				SPI_NRF_WriteReg(FLUSH_RX, NOP); //���RX FIFO�Ĵ���
				SPI_NRF_WriteReg(FLUSH_TX, NOP); //���TX FIFO�Ĵ���
				SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS, 0x70); //���STATUS�Ĵ��� �����Ӧ���ж�λ
		}
}

/**********************************************************************
**����ԭ��:   void nNrf_test(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   2.4GHz����
**˵    ��:   
************************************************************************/
/*
void nNrf_test(void)
{	
		SPI_NRF_Init();	
		EXIT_NRF_Init();
		
		//��֤�ܷ�������д
		while(NRF_Check() != SUCCESS)
		{
		
		}
		
#if 1
		memcpy(pstAddr.uchPipe0Addr, RX_ADDRESS, RX_ADR_WIDTH);
		pstAddr.uchLen = RX_ADR_WIDTH;
		
		vNrfPresetEnhanceShortBurst(NRF_PRIM_RX, 
																NRF_RF_CH_8, 
																NRF_TRAN_SPEED_1Mbps, 
																NRF_TRAN_POWER_0dbm,
																NRF_RT_DISABLE, 
																NRF_RT_DELAY_250, 
																&pstAddr);
#else		
		NRF_RX_Mode(NRF_TRAN_SPEED_1Mbps, NRF_TRAN_POWER_0dbm);		
#endif		
		
		//NRF_Rx_Dat(nrf_rx_inf.rx_data);
}
*/
