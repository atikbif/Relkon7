/*
 * hlcd.c
 *
 *  Created on: Mar 1, 2012
 *      Author: Роман
 */
#include "stm32f10x_conf.h"
#include "hlcd.h"

static volatile unsigned char pr_key;
extern volatile unsigned char _SysKey;
extern unsigned char TxLCDBuf[84];
static volatile unsigned char lcd_cnt=0;

extern volatile unsigned char canalpu_tx_buf[512];
extern volatile unsigned char canalpu_rx_buf[512];
extern volatile unsigned short canalpu_rx_cnt,canalpu_tx_cnt;
extern unsigned char pult_dis;
static unsigned short pu_txmax;
extern volatile unsigned short pu_tmr;

void init_lcd_canal(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	if(pult_dis!=0x31) USART_InitStructure.USART_BaudRate = 38400;
	else USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART5, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART5, USART_IT_RXNE , ENABLE);
	USART_Cmd(UART5, ENABLE);
	GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);
}

void UART5_IRQHandler(void)
{
	unsigned char key;
	if(pult_dis!=0x31)
	{
		if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
		{
			if(USART_GetFlagStatus(UART5,USART_FLAG_NE | USART_FLAG_FE)==RESET)
			{
				key = ~USART_ReceiveData(UART5);
				if(key==0xFF) key=0;
				if(key==pr_key)  _SysKey=key;pr_key=key;
			}else
			{
				USART_ReceiveData(UART5);
				_SysKey=0;pr_key=0;
			}
			USART_ClearITPendingBit(UART5, USART_IT_RXNE);
			USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
		}
		if(USART_GetITStatus(UART5, USART_IT_TC) != RESET)
		{
			USART_ClearITPendingBit(UART5, USART_IT_TC);
			if(lcd_cnt<84) USART_SendData(UART5, TxLCDBuf[lcd_cnt++]);

			if(USART_GetFlagStatus(UART5, USART_FLAG_TXE)!=RESET)
			{
				USART_ClearFlag(UART5, USART_FLAG_TXE);
				GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);
				USART_ITConfig(UART5, USART_IT_TC, DISABLE);
				USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
			}
		}
	}else
	{
		if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
		{
			pu_tmr=0;
			canalpu_rx_buf[canalpu_rx_cnt++]=USART_ReceiveData(UART5);
			if(canalpu_rx_cnt>=512) canalpu_rx_cnt=0;
			USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		}
		if(USART_GetITStatus(UART5, USART_IT_TC) != RESET)
		{
			USART_ClearITPendingBit(UART5, USART_IT_TC);
			if(canalpu_tx_cnt<pu_txmax) USART_SendData(UART5, canalpu_tx_buf[canalpu_tx_cnt++]);
			else
			{
				canalpu_tx_cnt=0;
				USART_ITConfig(UART5, USART_IT_TC, DISABLE);
				USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
			}
		}
	}
}

void write_lcd(unsigned short cnt)
{
	canalpu_tx_cnt=1;pu_txmax = cnt;
	GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_SET);
	USART_ITConfig(UART5, USART_IT_TC, ENABLE);
	if(pult_dis!=0x31){lcd_cnt=1;USART_SendData(UART5, TxLCDBuf[0]);}
	else USART_SendData(UART5, canalpu_tx_buf[0]);
}

void rst_pu_toggle(void)
{
	if(canalpu_tx_cnt==0) GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);
}
