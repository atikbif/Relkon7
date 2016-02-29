/*
 * hcanal.c
 *
 *  Created on: Feb 29, 2012
 *      Author: Роман
 */

#include "hcanal.h"
#include "canal.h"
#include "main.h"
#include "stm32f10x_conf.h"

extern unsigned char canal_tx_buf[BUF_SIZE];
extern unsigned char canal_rx_buf[BUF_SIZE];
extern unsigned char canal2_tx_buf[BUF_SIZE];
extern unsigned char canal2_rx_buf[BUF_SIZE];
extern volatile unsigned int canal_rx_cnt;
extern volatile unsigned int canal2_rx_cnt;
extern volatile unsigned char ascii_type,ascii_type2;
extern plc_stat _Sys;

unsigned long get_pc_tmr(void)
{
	return TIM2->CNT;
}

unsigned long get_pr_tmr(void)
{
	return TIM5->CNT;
}

void init_prog_canal(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	TIM_DeInit( TIM5 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Prescaler = 0xFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (unsigned short)0xFFFF;
	TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM5, ENABLE );
	TIM_Cmd( TIM5, ENABLE );

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	switch(_Sys.Can2_Baudrate)
	{
		case 0:USART_InitStructure.USART_BaudRate = 4800;break;
		case 1:USART_InitStructure.USART_BaudRate = 9600;break;
		case 2:USART_InitStructure.USART_BaudRate = 19200;break;
		case 3:USART_InitStructure.USART_BaudRate = 38400;break;
		case 4:USART_InitStructure.USART_BaudRate = 57600;break;
		case 5:USART_InitStructure.USART_BaudRate = 115200;break;
		case 6:USART_InitStructure.USART_BaudRate = 230400;break;
		case 7:USART_InitStructure.USART_BaudRate = 460800;break;
		case 8:USART_InitStructure.USART_BaudRate = 921600;break;
	}
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART4, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART4, USART_IT_RXNE , ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(UART4, ENABLE);

	GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_RESET);
}

void prog_message_bin(void)
{
	unsigned char n10=0,n100=0,val;

	canal2_tx_buf[0]=0x0D;canal2_tx_buf[1]=0x0A;canal2_tx_buf[2]='A';canal2_tx_buf[3]='D';canal2_tx_buf[4]='D';
	canal2_tx_buf[5]='R';canal2_tx_buf[6]='E';canal2_tx_buf[7]='S';canal2_tx_buf[8]='S';canal2_tx_buf[9]='-';
	val=_Sys.Adr;
	while(val>=100){val-=100;n100++;}
	while(val>=10){val-=10;n10++;}
	canal2_tx_buf[10]='0'+n100;
	canal2_tx_buf[11]='0'+n10;
	canal2_tx_buf[12]='0'+val;
	write_canal2(13);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_7));
	canal2_tx_buf[0]=0x0D;canal2_tx_buf[1]=0x0A;canal2_tx_buf[2]='B';canal2_tx_buf[3]='I';canal2_tx_buf[4]='N';
	canal2_tx_buf[5]=' ';canal2_tx_buf[6]=' ';
	write_canal2(7);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_7));
	canal2_tx_buf[0]=0x0D;canal2_tx_buf[1]=0x0A;
	switch(_Sys.Can2_Baudrate)
	{
		case 0:canal2_tx_buf[2]='4';canal2_tx_buf[3]='8';canal2_tx_buf[4]='0';canal2_tx_buf[5]='0';canal2_tx_buf[6]=' ';canal2_tx_buf[7]=' ';break;
		case 1:canal2_tx_buf[2]='9';canal2_tx_buf[3]='6';canal2_tx_buf[4]='0';canal2_tx_buf[5]='0';canal2_tx_buf[6]=' ';canal2_tx_buf[7]=' ';break;
		case 2:canal2_tx_buf[2]='1';canal2_tx_buf[3]='9';canal2_tx_buf[4]='2';canal2_tx_buf[5]='0';canal2_tx_buf[6]='0';canal2_tx_buf[7]=' ';break;
		case 3:canal2_tx_buf[2]='3';canal2_tx_buf[3]='8';canal2_tx_buf[4]='4';canal2_tx_buf[5]='0';canal2_tx_buf[6]='0';canal2_tx_buf[7]=' ';break;
		case 4:canal2_tx_buf[2]='5';canal2_tx_buf[3]='7';canal2_tx_buf[4]='6';canal2_tx_buf[5]='0';canal2_tx_buf[6]='0';canal2_tx_buf[7]=' ';break;
		case 5:canal2_tx_buf[2]='1';canal2_tx_buf[3]='1';canal2_tx_buf[4]='5';canal2_tx_buf[5]='2';canal2_tx_buf[6]='0';canal2_tx_buf[7]='0';break;
	}
	canal2_tx_buf[8]=0x0D;canal2_tx_buf[9]=0x0A;write_canal2(10);
}

void prog_message_ascii(void)
{
	unsigned char n10=0,n100=0,val;

	canal2_tx_buf[0]=0x0D;canal2_tx_buf[1]=0x0A;canal2_tx_buf[2]='A';canal2_tx_buf[3]='D';canal2_tx_buf[4]='D';
	canal2_tx_buf[5]='R';canal2_tx_buf[6]='E';canal2_tx_buf[7]='S';canal2_tx_buf[8]='S';canal2_tx_buf[9]='-';
	val=_Sys.Adr;
	while(val>=100){val-=100;n100++;}
	while(val>=10){val-=10;n10++;}
	canal2_tx_buf[10]='0'+n100;
	canal2_tx_buf[11]='0'+n10;
	canal2_tx_buf[12]='0'+val;
	write_canal2(13);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_7));
	canal2_tx_buf[0]=0x0D;canal2_tx_buf[1]=0x0A;canal2_tx_buf[2]='A';canal2_tx_buf[3]='S';canal2_tx_buf[4]='C';
	canal2_tx_buf[5]='I';canal2_tx_buf[6]='I';
	write_canal2(7);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_7));
	canal2_tx_buf[0]=0x0D;canal2_tx_buf[1]=0x0A;
	switch(_Sys.Can2_Baudrate)
	{
		case 0:canal2_tx_buf[2]='4';canal2_tx_buf[3]='8';canal2_tx_buf[4]='0';canal2_tx_buf[5]='0';canal2_tx_buf[6]=' ';canal2_tx_buf[7]=' ';break;
		case 1:canal2_tx_buf[2]='9';canal2_tx_buf[3]='6';canal2_tx_buf[4]='0';canal2_tx_buf[5]='0';canal2_tx_buf[6]=' ';canal2_tx_buf[7]=' ';break;
		case 2:canal2_tx_buf[2]='1';canal2_tx_buf[3]='9';canal2_tx_buf[4]='2';canal2_tx_buf[5]='0';canal2_tx_buf[6]='0';canal2_tx_buf[7]=' ';break;
		case 3:canal2_tx_buf[2]='3';canal2_tx_buf[3]='8';canal2_tx_buf[4]='4';canal2_tx_buf[5]='0';canal2_tx_buf[6]='0';canal2_tx_buf[7]=' ';break;
		case 4:canal2_tx_buf[2]='5';canal2_tx_buf[3]='7';canal2_tx_buf[4]='6';canal2_tx_buf[5]='0';canal2_tx_buf[6]='0';canal2_tx_buf[7]=' ';break;
		case 5:canal2_tx_buf[2]='1';canal2_tx_buf[3]='1';canal2_tx_buf[4]='5';canal2_tx_buf[5]='2';canal2_tx_buf[6]='0';canal2_tx_buf[7]='0';break;
	}
	canal2_tx_buf[8]=0x0D;canal2_tx_buf[9]=0x0A;write_canal2(10);
}

void init_pc_canal(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	TIM_DeInit( TIM2 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Prescaler = 0xFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ( unsigned short ) 0xFFFF;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM2, ENABLE );
	TIM_Cmd( TIM2, ENABLE );

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	switch(_Sys.Can1_Baudrate)
	{
		case 0:USART_InitStructure.USART_BaudRate = 4800;break;
		case 1:USART_InitStructure.USART_BaudRate = 9600;break;
		case 2:USART_InitStructure.USART_BaudRate = 19200;break;
		case 3:USART_InitStructure.USART_BaudRate = 38400;break;
		case 4:USART_InitStructure.USART_BaudRate = 57600;break;
		case 5:USART_InitStructure.USART_BaudRate = 115200;break;
		case 6:USART_InitStructure.USART_BaudRate = 230400;break;
		case 7:USART_InitStructure.USART_BaudRate = 460800;break;
		case 8:USART_InitStructure.USART_BaudRate = 921600;break;
	}
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE , ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART2, ENABLE);

	GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);
}

void pc_message_bin(void)
{
	unsigned char n10=0,n100=0,val;
	canal_tx_buf[0]=0x0D;canal_tx_buf[1]=0x0A;canal_tx_buf[2]='A';canal_tx_buf[3]='D';canal_tx_buf[4]='D';
	canal_tx_buf[5]='R';canal_tx_buf[6]='E';canal_tx_buf[7]='S';canal_tx_buf[8]='S';canal_tx_buf[9]='-';
	val=_Sys.Adr;
	while(val>=100){val-=100;n100++;}
	while(val>=10){val-=10;n10++;}
	canal_tx_buf[10]='0'+n100;
	canal_tx_buf[11]='0'+n10;
	canal_tx_buf[12]='0'+val;
	write_canal(13);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_0));
	canal_tx_buf[0]=0x0D;canal_tx_buf[1]=0x0A;canal_tx_buf[2]='B';canal_tx_buf[3]='I';canal_tx_buf[4]='N';
	canal_tx_buf[5]=' ';canal_tx_buf[6]=' ';
	write_canal(7);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_0));
	canal_tx_buf[0]=0x0D;canal_tx_buf[1]=0x0A;
	switch(_Sys.Can1_Baudrate)
	{
		case 0:canal_tx_buf[2]='4';canal_tx_buf[3]='8';canal_tx_buf[4]='0';canal_tx_buf[5]='0';canal_tx_buf[6]=' ';canal_tx_buf[7]=' ';break;
		case 1:canal_tx_buf[2]='9';canal_tx_buf[3]='6';canal_tx_buf[4]='0';canal_tx_buf[5]='0';canal_tx_buf[6]=' ';canal_tx_buf[7]=' ';break;
		case 2:canal_tx_buf[2]='1';canal_tx_buf[3]='9';canal_tx_buf[4]='2';canal_tx_buf[5]='0';canal_tx_buf[6]='0';canal_tx_buf[7]=' ';break;
		case 3:canal_tx_buf[2]='3';canal_tx_buf[3]='8';canal_tx_buf[4]='4';canal_tx_buf[5]='0';canal_tx_buf[6]='0';canal_tx_buf[7]=' ';break;
		case 4:canal_tx_buf[2]='5';canal_tx_buf[3]='7';canal_tx_buf[4]='6';canal_tx_buf[5]='0';canal_tx_buf[6]='0';canal_tx_buf[7]=' ';break;
		case 5:canal_tx_buf[2]='1';canal_tx_buf[3]='1';canal_tx_buf[4]='5';canal_tx_buf[5]='2';canal_tx_buf[6]='0';canal_tx_buf[7]='0';break;
	}
	canal_tx_buf[8]=0x0D;canal_tx_buf[9]=0x0A;write_canal(10);
}

void pc_message_ascii(void)
{
	unsigned char n10=0,n100=0,val;
	canal_tx_buf[0]=0x0D;canal_tx_buf[1]=0x0A;canal_tx_buf[2]='A';canal_tx_buf[3]='D';canal_tx_buf[4]='D';
	canal_tx_buf[5]='R';canal_tx_buf[6]='E';canal_tx_buf[7]='S';canal_tx_buf[8]='S';canal_tx_buf[9]='-';
	val=_Sys.Adr;
	while(val>=100){val-=100;n100++;}
	while(val>=10){val-=10;n10++;}
	canal_tx_buf[10]='0'+n100;
	canal_tx_buf[11]='0'+n10;
	canal_tx_buf[12]='0'+val;
	write_canal(13);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_0));
	canal_tx_buf[0]=0x0D;canal_tx_buf[1]=0x0A;canal_tx_buf[2]='A';canal_tx_buf[3]='S';canal_tx_buf[4]='C';
	canal_tx_buf[5]='I';canal_tx_buf[6]='I';
	write_canal(7);
	while(GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_0));
	canal_tx_buf[0]=0x0D;canal_tx_buf[1]=0x0A;
	switch(_Sys.Can1_Baudrate)
	{
		case 0:canal_tx_buf[2]='4';canal_tx_buf[3]='8';canal_tx_buf[4]='0';canal_tx_buf[5]='0';canal_tx_buf[6]=' ';canal_tx_buf[7]=' ';break;
		case 1:canal_tx_buf[2]='9';canal_tx_buf[3]='6';canal_tx_buf[4]='0';canal_tx_buf[5]='0';canal_tx_buf[6]=' ';canal_tx_buf[7]=' ';break;
		case 2:canal_tx_buf[2]='1';canal_tx_buf[3]='9';canal_tx_buf[4]='2';canal_tx_buf[5]='0';canal_tx_buf[6]='0';canal_tx_buf[7]=' ';break;
		case 3:canal_tx_buf[2]='3';canal_tx_buf[3]='8';canal_tx_buf[4]='4';canal_tx_buf[5]='0';canal_tx_buf[6]='0';canal_tx_buf[7]=' ';break;
		case 4:canal_tx_buf[2]='5';canal_tx_buf[3]='7';canal_tx_buf[4]='6';canal_tx_buf[5]='0';canal_tx_buf[6]='0';canal_tx_buf[7]=' ';break;
		case 5:canal_tx_buf[2]='1';canal_tx_buf[3]='1';canal_tx_buf[4]='5';canal_tx_buf[5]='2';canal_tx_buf[6]='0';canal_tx_buf[7]='0';break;
	}
	canal_tx_buf[8]=0x0D;canal_tx_buf[9]=0x0A;write_canal(10);
}

void write_canal(unsigned short count)
{
	DMA_InitTypeDef DMA_InitStructure;
	if(count>BUF_SIZE) return;
	GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)canal_tx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	DMA_Cmd(DMA1_Channel7, ENABLE);
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
   {
     USART_ClearITPendingBit(USART2, USART_IT_TC);
     if(USART_GetFlagStatus(USART2, USART_FLAG_TXE)!=RESET)
     {
        GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);
        USART_ITConfig(USART2, USART_IT_TC, DISABLE);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
     }
   }
   if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
   {
	 canal_rx_buf[canal_rx_cnt]=USART_ReceiveData(USART2);
     if(canal_rx_buf[canal_rx_cnt]=='$') ascii_type=_RELKON;
	 if(canal_rx_buf[canal_rx_cnt]==':') {ascii_type=_MODBUS;}
	 canal_rx_cnt++;
     if(canal_rx_cnt>=BUF_SIZE) canal_rx_cnt=0;
     USART_ClearITPendingBit(USART2, USART_IT_RXNE);
     TIM2->CNT=0;
    }
}

void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC7) != RESET)
	{
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA_ClearITPendingBit(DMA1_IT_GL7);
		USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	}
}

void write_canal2(unsigned short count)
{
	DMA_InitTypeDef DMA_InitStructure;

	if(count>BUF_SIZE) return;
	GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_SET);
	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = UART4_DR_Base;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)canal2_tx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
	DMA_Cmd(DMA2_Channel5, ENABLE);
}

void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4, USART_IT_TC) != RESET)
   {
     USART_ClearITPendingBit(UART4, USART_IT_TC);
     if(USART_GetFlagStatus(UART4, USART_FLAG_TXE)!=RESET)
     {
        GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_RESET);
        USART_ITConfig(UART4, USART_IT_TC, DISABLE);
        USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
     }
   }
   if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
   {
	 canal2_rx_buf[canal2_rx_cnt]=USART_ReceiveData(UART4);
     if(canal2_rx_buf[canal2_rx_cnt]=='$') ascii_type2=_RELKON;
	 if(canal2_rx_buf[canal2_rx_cnt]==':') {ascii_type2=_MODBUS;}
	 canal2_rx_cnt++;
     if(canal2_rx_cnt>=BUF_SIZE) canal2_rx_cnt=0;
     USART_ClearITPendingBit(UART4, USART_IT_RXNE);
     TIM5->CNT=0;
    }
}

void DMA2_Channel5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_IT_TC5) != RESET)
	{
		DMA_Cmd(DMA2_Channel5, DISABLE);
		DMA_ClearITPendingBit(DMA2_IT_GL5);
		USART_ITConfig(UART4, USART_IT_TC, ENABLE);
	}
}
