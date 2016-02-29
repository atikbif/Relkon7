/*
 * hinout.c
 *
 *  Created on: Feb 29, 2012
 *      Author: Роман
 */

#include "hinout.h"
#include "stm32f10x_conf.h"

extern unsigned char tx_mod_buf[MOD_BUF_SIZE];
extern unsigned char rx_mod_buf[MOD_BUF_SIZE];
extern volatile unsigned char Tx_end;
extern volatile unsigned short rx_mod_cnt;
extern volatile unsigned char emu_mode;

extern volatile unsigned char prot_enable;
extern volatile unsigned char EXCHANGE;

extern const unsigned char mod_table[];

static unsigned char get_input(void);
static void set_output(unsigned char dt);

inline void baud_19200(void)
{
    USART_InitTypeDef USART_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    USART_Cmd(USART1, DISABLE);
    TIM_Cmd( TIM4, DISABLE );

    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

	TIM_DeInit( TIM4 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Prescaler = 0x06;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseStructure );

    USART_Cmd(USART1, ENABLE);TIM_Cmd( TIM4, ENABLE );
}

unsigned long get_mmb_tmr(void)
{
	return(TIM4->CNT);
}

void dout_settings(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void din_settings(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void write_dout(unsigned char num, unsigned char val)
{
	set_output(val);
	switch(num)
	{
		case 0:
			GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
			break;
		case 1:
			GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
			break;
		case 2:
			GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
			break;
		case 3:
			GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_RESET);
			break;
		case 4:
			GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);
			break;
		case 5:
			GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_RESET);
			break;
	}
}

unsigned char read_din(unsigned char num)
{
	unsigned char tmp=0;
	switch(num)
	{
		case 0:
			GPIO_WriteBit(GPIOE,GPIO_Pin_15,Bit_RESET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
			tmp = get_input();
			GPIO_WriteBit(GPIOE,GPIO_Pin_15,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
			break;
		case 1:
			GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_RESET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
			tmp=get_input();
			GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
			break;
		case 2:
			GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
			tmp=get_input();
			GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
			break;
		case 3:
			GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
			tmp=get_input();
			GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
			break;
		case 4:
			GPIO_WriteBit(GPIOE,GPIO_Pin_5,Bit_RESET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
			tmp=get_input();
			GPIO_WriteBit(GPIOE,GPIO_Pin_5,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
			break;
		case 5:
			GPIO_WriteBit(GPIOE,GPIO_Pin_6,Bit_RESET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_SET);
			tmp=get_input();
			GPIO_WriteBit(GPIOE,GPIO_Pin_6,Bit_SET);
			GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);
			break;
	}
	return(tmp);
}

void init_mb_canal(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	TIM_DeInit( TIM4 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Prescaler = 0x01;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ( unsigned short ) 0xFFFF;
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM4, ENABLE );
	TIM_Cmd( TIM4, ENABLE );

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	if((mod_table[0])&&(emu_mode!=2)){USART_InitStructure.USART_StopBits = USART_StopBits_2;}
	else{USART_InitStructure.USART_StopBits = USART_StopBits_1;}
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE , ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART1, ENABLE);
	GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_RESET);
}

void dio_conf(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);// CLK OUT1
	GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);// CLK OUT2
	GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);// CLK OUT3
	GPIO_WriteBit(GPIOE,GPIO_Pin_14,Bit_RESET);// CLK OUT4
	GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET);// CLK OUT5
	GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_RESET);// CLK OUT6

	GPIO_WriteBit(GPIOE,GPIO_Pin_15,Bit_SET);// OE IN1
	GPIO_WriteBit(GPIOD,GPIO_Pin_11,Bit_SET);// OE IN2
	GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);// OE IN3
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);// OE IN4
	GPIO_WriteBit(GPIOE,GPIO_Pin_5,Bit_SET);// OE IN5
	GPIO_WriteBit(GPIOE,GPIO_Pin_6,Bit_SET);// OE IN6

	GPIO_WriteBit(GPIOE,GPIO_Pin_13,Bit_RESET);// CLK IN
}

void write_module(unsigned short count)
{
	DMA_InitTypeDef DMA_InitStructure;
	if(count>MOD_BUF_SIZE) return;
	GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_SET);
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)tx_mod_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

static unsigned char get_input(void)
{
    unsigned char tmp=0;
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)==Bit_RESET) tmp|=0x80;
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==Bit_RESET) tmp|=0x40;
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)==Bit_RESET) tmp|=0x20;
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9)==Bit_RESET) tmp|=0x10;
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)==Bit_RESET) tmp|=0x08;
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==Bit_RESET) tmp|=0x04;
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)==Bit_RESET) tmp|=0x02;
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==Bit_RESET) tmp|=0x01;
    return tmp;
}

static void set_output(unsigned char dt)
{
    if(dt&0x80) GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_RESET); else GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_SET);
    if(dt&0x40) GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_RESET); else GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_SET);
    if(dt&0x20) GPIO_WriteBit(GPIOA,GPIO_Pin_10,Bit_RESET); else GPIO_WriteBit(GPIOA,GPIO_Pin_10,Bit_SET);
    if(dt&0x10) GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_RESET); else GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
    if(dt&0x08) GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET); else GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET);
    if(dt&0x04) GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET); else GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_SET);
    if(dt&0x02) GPIO_WriteBit(GPIOC,GPIO_Pin_7,Bit_RESET); else GPIO_WriteBit(GPIOC,GPIO_Pin_7,Bit_SET);
    if(dt&0x01) GPIO_WriteBit(GPIOC,GPIO_Pin_6,Bit_RESET); else GPIO_WriteBit(GPIOC,GPIO_Pin_6,Bit_SET);
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
   {
     USART_ClearITPendingBit(USART1, USART_IT_TC);
     if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=RESET)
     {
        GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_RESET);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        if(((mod_table[0])&&(emu_mode!=2))||(EXCHANGE)) {Tx_end++;TIM4->CNT=0;rx_mod_cnt=0;}
     }
   }
   if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
   {
     rx_mod_buf[rx_mod_cnt++]=USART_ReceiveData(USART1);
     if(rx_mod_cnt>=MOD_BUF_SIZE) rx_mod_cnt=0;
     USART_ClearITPendingBit(USART1, USART_IT_RXNE);
     TIM4->CNT=0;
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_GL4);
        USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    }
}
