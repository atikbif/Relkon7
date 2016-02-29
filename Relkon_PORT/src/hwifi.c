/*
 * hwifi.c
 *
 *  Created on: Jun 21, 2012
 *      Author: Роман
 */


#include "hwifi.h"
#include "cc3000_common.h"

static SPI_InitTypeDef   SPI_InitStructure;
static GPIO_InitTypeDef   GPIO_InitStructure;
static NVIC_InitTypeDef NVIC_InitSPIGPIOStructure;
static EXTI_InitTypeDef EXTI_SPIGPIO_InitStructure;
static NVIC_InitTypeDef NVIC_RxInt_InitStructure;

volatile unsigned char fl_rd=0;

struct
{
	unsigned short tx_max;
	unsigned short tx_cnt;
	unsigned short rx_cnt;
	unsigned char* tx;
	unsigned char* rx;
}spi_int_hdr;

extern tSpiInformation sSpiInformation;
extern portTickType WFxLastExecutionTime;

void wait_spi(void)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );;
}

void cc_gpio_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// SPI INIT

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//256;//64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_FLAG_TXE);
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
	SPI_Cmd(SPI2, ENABLE);

	NVIC_RxInt_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_RxInt_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_RxInt_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_RxInt_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_RxInt_InitStructure);

	// GPIO INIT

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				// CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				// PWR_EN
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				// IRQ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void WriteWlanPin( unsigned char val )
{
  if(val == WLAN_ENABLE) {GPIO_SetBits(GPIOD, GPIO_Pin_3);}
  else {GPIO_ResetBits(GPIOD, GPIO_Pin_3);}
}

long ReadWlanInterruptPin(void)
{
	return GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8);
}

void WlanInterruptEnable(void)
{
	EXTI_DeInit();
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource8);

	EXTI_SPIGPIO_InitStructure.EXTI_Line = EXTI_Line8 ;
	EXTI_SPIGPIO_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_SPIGPIO_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_SPIGPIO_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_SPIGPIO_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearFlag(EXTI_Line8);

	NVIC_InitSPIGPIOStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitSPIGPIOStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitSPIGPIOStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitSPIGPIOStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitSPIGPIOStructure);
}

void WlanInterruptDisable(void)
{
	EXTI_SPIGPIO_InitStructure.EXTI_Line = EXTI_Line8 ;
	EXTI_SPIGPIO_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_SPIGPIO_InitStructure);

	NVIC_InitSPIGPIOStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitSPIGPIOStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitSPIGPIOStructure);
}

void init_cctimer(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM6, ENABLE);
}

void pause_us(unsigned short val)
{
	TIM6->CNT = 0;
	while(TIM6->CNT < val);
}

void SpiSend(unsigned char* tx_ptr,unsigned char* rx_ptr,unsigned short l,portTickType ptr)
{
	spi_int_hdr.tx = tx_ptr;spi_int_hdr.rx = rx_ptr;
	spi_int_hdr.tx_max = l;
	spi_int_hdr.tx_cnt = 0;spi_int_hdr.rx_cnt = 0;
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET) vTaskDelayUntil( &ptr, WF_DELAY );
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
	while(spi_int_hdr.rx_cnt < spi_int_hdr.tx_max) vTaskDelayUntil( &ptr, WF_DELAY );
}

void SPI2_IRQHandler(void)
{
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) != RESET)
	{
		if(spi_int_hdr.tx) SPI_I2S_SendData(SPI2, spi_int_hdr.tx[spi_int_hdr.tx_cnt++]);
		else {spi_int_hdr.tx_cnt++;SPI_I2S_SendData(SPI2, 0x00);}
		if(spi_int_hdr.tx_cnt >= spi_int_hdr.tx_max) SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
	}
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
	{
		if(spi_int_hdr.rx) spi_int_hdr.rx[spi_int_hdr.rx_cnt++] = SPI_I2S_ReceiveData(SPI2);
		else{spi_int_hdr.rx_cnt++;SPI_I2S_ReceiveData(SPI2);}
		if(spi_int_hdr.rx_cnt >= spi_int_hdr.tx_max) SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{

		if(!ReadWlanInterruptPin())
		{
			if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
			{
				sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
			}
			else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
			{
				sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
			}else fl_rd=1;
		}
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}
