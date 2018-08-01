#include "uart.h"
#include <stdarg.h>

/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
/* Private functions ---------------------------------------------------------*/

/**********************************************************************
**����ԭ��:   void USART2_Configuration(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   ���ڳ�ʼ������
**˵    ��:   
************************************************************************/
static void USART2_Configuration(void)
{  
		GPIO_InitTypeDef  GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef  NVIC_InitStructure;
	
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA,   ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
						
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);        
		/*
		*  USART2_TX -> PA2 , USART2_RX -> PA3
		*/                                
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;                 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOA, &GPIO_InitStructure);        

		USART_InitStructure.USART_BaudRate 	 = 115200;						  //���ô��ڲ�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//��������λ
		USART_InitStructure.USART_StopBits 	 = USART_StopBits_1;		//����ֹͣλ
		USART_InitStructure.USART_Parity     = USART_Parity_No;			//����Ч��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //����������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //���ù���ģʽ
		USART_Init(USART2, &USART_InitStructure); //������ṹ��

		USART_Cmd(USART2, ENABLE); //ʹ�ܴ���2
		
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //�����ж�
		NVIC_InitStructure.NVIC_IRQChannelPriority = 2;		//�������ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //ʹ��
		NVIC_Init(&NVIC_InitStructure);		
}			

/**********************************************************************
**����ԭ��:   void USART2_SendByte(uint8_t byte)
**��ڲ���:   �����ֽ�
**�� �� ֵ:   ��
**��    ��:   ����2����һ���ֽ�
**˵    ��:   
************************************************************************/
static void USART2_SendByte(uint8_t byte)
{
		USART_SendData(USART2, byte);
		while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
}

/**********************************************************************
**����ԭ��:   void USART2_SendString(uint8_t *string, uint8_t len)
**��ڲ���:   �������飬���鳤��
**�� �� ֵ:   ��
**��    ��:   ����2����һ������
**˵    ��:   
************************************************************************/
static void USART2_SendString(uint8_t *string, uint8_t len)
{
		for(uint8_t i=0; i<len; i++)
		{
				USART2_SendByte(string[i]);
		}
}

/**********************************************************************
**����ԭ��:  	void USART2_IRQHandler(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   ����2�жϺ���
**˵    ��:   
************************************************************************/
void USART2_IRQHandler(void)
{
		uint8_t temp = 0;
		
		//�����ж�
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
				temp = USART_ReceiveData(USART2); //�����Զ����RXNE
				USART2_SendByte(temp);
		}
}

PUTCHAR_PROTOTYPE 
{
		/* ��Printf���ݷ������� */
		USART_SendData(USART2, (uint8_t)  ch);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		{
		
		}
		return (ch);
}

/**********************************************************************
**����ԭ��:   void UART_TTL_Init(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   TTL���ڳ�ʼ��
**˵    ��:   
************************************************************************/
void UART_TTL_Init(void)
{
		USART2_Configuration();
}

/**********************************************************************
**����ԭ��:   void UART_TTL_SendByte(uint8_t byte)
**��ڲ���:   �����ֽ�
**�� �� ֵ:   ��
**��    ��:   TTL���ڷ���һ���ֽ�
**˵    ��:   
************************************************************************/
void UART_TTL_SendByte(uint8_t byte)
{
		USART2_SendByte(byte);
}

/**********************************************************************
**����ԭ��:   void UART_TTL_SendString(uint8_t *string, uint8_t len)
**��ڲ���:   �������飬���鳤��
**�� �� ֵ:   ��
**��    ��:   TTL���ڷ���һ������
**˵    ��:   
************************************************************************/
void UART_TTL_SendString(uint8_t *string, uint8_t len)
{
		USART2_SendString(string, len);
}

/**********************************************************************
**����ԭ��:   void USART4_Configuration(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   ���ڳ�ʼ������
**˵    ��:   
************************************************************************/
static void USART4_Configuration(void)
{  
		GPIO_InitTypeDef  GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef  NVIC_InitStructure;
	
		RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC,   ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4, ENABLE);
						
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_0);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_0);        
		/*
		*  USART4_TX -> PC10 , USART4_RX -> PC11
		*/                                
     
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;                 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOC, &GPIO_InitStructure); 
		
		USART_InitStructure.USART_BaudRate 	 = 115200;						  //���ô��ڲ�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//��������λ
		USART_InitStructure.USART_StopBits 	 = USART_StopBits_1;		//����ֹͣλ
		USART_InitStructure.USART_Parity     = USART_Parity_No;			//����Ч��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //����������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //���ù���ģʽ
		USART_Init(USART4, &USART_InitStructure); //������ṹ��

		USART_Cmd(USART4, ENABLE); //ʹ�ܴ���4
		
		USART_ITConfig(USART4, USART_IT_RXNE, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = USART3_6_IRQn; //�����ж�
		NVIC_InitStructure.NVIC_IRQChannelPriority = 2;			//�������ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   	//ʹ��
		NVIC_Init(&NVIC_InitStructure);		
}			

/**********************************************************************
**����ԭ��:   void USART4_SendByte(uint8_t byte)
**��ڲ���:   �����ֽ�
**�� �� ֵ:   ��
**��    ��:   ����4����һ���ֽ�
**˵    ��:   
************************************************************************/
static void USART4_SendByte(uint8_t byte)
{
		USART_SendData(USART4, byte);
		while(!USART_GetFlagStatus(USART4, USART_FLAG_TXE));
}

/**********************************************************************
**����ԭ��:   void USART4_SendString(uint8_t *string, uint8_t len)
**��ڲ���:   �������飬���鳤��
**�� �� ֵ:   ��
**��    ��:   ����4����һ������
**˵    ��:   
************************************************************************/
static void USART4_SendString(uint8_t *string, uint8_t len)
{
		for(uint8_t i=0; i<len; i++)
		{
				USART4_SendByte(string[i]);
		}
}

/**********************************************************************
**����ԭ��:  	void USART3_6_IRQHandler(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   ����4�жϺ���
**˵    ��:   
************************************************************************/
void USART3_6_IRQHandler(void)
{
		uint8_t temp = 0;
		
		//�����ж�
		if(USART_GetITStatus(USART4, USART_IT_RXNE) != RESET)
		{
				temp = USART_ReceiveData(USART4); //�����Զ����RXNE
				USART2_SendByte(temp);
		}
}

#define	ENABLE_485_RE	GPIOB->BRR  = GPIO_Pin_5; /* ���� */
#define	ENABLE_485_DE	GPIOB->BSRR = GPIO_Pin_5; /* ���� */

/**********************************************************************
**����ԭ��:   void RS485_Init(void)
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   RS485���ڳ�ʼ��
**˵    ��:   
************************************************************************/
void RS485_Init(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;   
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	
		USART4_Configuration();
}

/**********************************************************************
**����ԭ��:   void RS485_TX_Delay(void) 
**��ڲ���:   ��
**�� �� ֵ:   ��
**��    ��:   RS485������ʱ
**˵    ��:   
************************************************************************/
static void RS485_TX_Delay(void) 
{
		uint16_t cnt = 1024;
		while(cnt--);
}

/**********************************************************************
**����ԭ��:   void RS485_SendString(uint8_t *string, uint8_t len)
**��ڲ���:   �������飬���鳤��
**�� �� ֵ:   ��
**��    ��:   RS485����һ������
**˵    ��:   
************************************************************************/
void RS485_SendString(uint8_t *string, uint8_t len)
{
    ENABLE_485_DE;

		USART4_SendString(string, len);
		RS485_TX_Delay();
		
		ENABLE_485_RE;
}



