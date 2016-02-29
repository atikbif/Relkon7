/*
 * hfram.c
 *
 *  Created on: Feb 29, 2012
 *      Author: Роман
 */

#include "hfram.h"
#include "stm32f10x_conf.h"

static unsigned char get_inv(unsigned char dt);

void _SPI_init(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

	/* Configure USART3 TX and USART3 CK pins as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	USART_ClockInitStructure.USART_Clock = USART_Clock_Enable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;//High;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;//2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInit(USART3, &USART_ClockInitStructure);
	USART_InitStructure.USART_BaudRate = 2000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	SPI_CS_HIGH();
}

unsigned char SPI_SendByte(unsigned char byte)
{
  USART_ReceiveData(USART3);
  USART_SendData(USART3, byte);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

  while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
  return USART_ReceiveData(USART3);
}

void read_data(unsigned char adr_H,unsigned char adr_L,unsigned char size,unsigned char* ptr)
{
    vu16 cnt=0;
    if((unsigned long)ptr<0x20000000) return;
    SPI_CS_LOW();
    SPI_SendByte(0xC0);
    SPI_SendByte(get_inv(adr_H));
    SPI_SendByte(get_inv(adr_L));
    while(size--)
    {
        ptr[cnt] = SPI_SendByte(0xFF);
        cnt++;
    }
    SPI_CS_HIGH();
}

void write_data(unsigned char adr_H,unsigned char adr_L,unsigned char size,unsigned char* ptr)
{
    u8 cnt=0;
    SPI_CS_LOW();
    SPI_CS_LOW();
    SPI_SendByte(0x40);
    SPI_SendByte(get_inv(adr_H));
    SPI_SendByte(get_inv(adr_L));
    while(size--)
    {
        SPI_SendByte(ptr[cnt++]);
    }
    SPI_CS_HIGH();SPI_CS_HIGH();
}

void write_enable(void)
{
    SPI_CS_LOW();SPI_CS_LOW();
    SPI_SendByte(0x60);
    SPI_CS_HIGH();SPI_CS_HIGH();
}

static unsigned char get_inv(unsigned char dt)
{
    unsigned char tmp=0;
    if(dt&0x01) tmp|=0x80;
    if(dt&0x02) tmp|=0x40;
    if(dt&0x04) tmp|=0x20;
    if(dt&0x08) tmp|=0x10;
    if(dt&0x10) tmp|=0x08;
    if(dt&0x20) tmp|=0x04;
    if(dt&0x40) tmp|=0x02;
    if(dt&0x80) tmp|=0x01;
    return tmp;
}
