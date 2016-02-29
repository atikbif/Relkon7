/*
 * ain.c
 *
 *  Created on: Jan 18, 2012
 *      Author: Роман
 */
#include "hain.h"
#include "stm32f10x_conf.h"

#define TRANSMIT	0
#define RECEIVE		1
#define ClockSpeed              100000

volatile unsigned short adc_data[16];
volatile short _EA[8];

struct
{
	unsigned char direction;
	unsigned char tx[8];
	unsigned char rx[8];
	unsigned char send_amount;
	unsigned char rcv_amount;
	unsigned char cnt;
	unsigned char stat;
	unsigned char addr;
	unsigned short err;
}volatile __attribute__((aligned(4))) i2c;

volatile short ext_adc;
volatile unsigned short i2c_debug=0;

void init_adc()
{
	ADC_InitTypeDef  ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_2 |GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_Cmd(ADC1, DISABLE);
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//Left;
	ADC_InitStructure.ADC_NbrOfChannel = 8;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 8, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 5, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_DeInit(DMA1_Channel1);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(ADC1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&adc_data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//Medium;//DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
	ext_adc_init();
}

unsigned short get_adc(unsigned char num)
{
    return (((unsigned long)adc_data[num]*125)/127);
}

void adc_write_set(unsigned char num)
{
	/*if(num>5) num=0;
	i2c.direction = TRANSMIT;
	i2c.send_amount=3;
	i2c.tx[0] = 0x01;
	switch(num)
	{
		case 0:i2c.addr = 0x92;i2c.tx[1] = 0xC4;break;
		case 1:i2c.addr = 0x92;i2c.tx[1] = 0xD4;break;
		case 2:i2c.addr = 0x92;i2c.tx[1] = 0xE4;break;
		case 3:i2c.addr = 0x92;i2c.tx[1] = 0xF4;break;
		case 4:i2c.addr = 0x90;i2c.tx[1] = 0xD4;break;
		case 5:i2c.addr = 0x90;i2c.tx[1] = 0xC4;break;
	}
	i2c.tx[2] = 0xD3;
	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==Bit_SET))
	{
		I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
		I2C_GenerateSTART(I2C1, ENABLE);
	}
	else
	{
		i2c.err++;
		if(i2c.err>=1000){i2c.err=0;ext_adc_init();}
	}*/
}

void get_ext_adc(void)
{
/*	i2c.direction = RECEIVE;
	i2c.tx[0]=0x00;
	i2c.stat=0;
	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==Bit_SET) && (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==Bit_SET))
	{
		I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF , ENABLE);
		I2C_GenerateSTART(I2C1, ENABLE);
	}
	else
	{
		i2c.err++;
		if(i2c.err>=1000){i2c.err=0;ext_adc_init();}
	}*/
}

void ext_adc_init(void)
{
/*	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef   I2C_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	I2C_DeInit(I2C1);
	//I2C_SoftwareResetCmd(I2C1,ENABLE);
	i2c.err=0;

	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	if((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==Bit_RESET) || (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==Bit_RESET)) return;


	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//GPIO_Mode_AF_PP;//GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

	I2C_Cmd(I2C1, ENABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;
	I2C_Init(I2C1, &I2C_InitStructure);*/
}

void I2C1_EV_IRQHandler(void)
{
	switch(i2c.direction)
	{
		case TRANSMIT:
			switch (I2C_GetLastEvent(I2C1))
			{
				case I2C_EVENT_MASTER_MODE_SELECT:
					I2C_Send7bitAddress(I2C1, i2c.addr, I2C_Direction_Transmitter);
					break;
				case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
					I2C_SendData(I2C1, i2c.tx[0]);
					i2c.cnt=1;
					break;
				case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
					if(i2c.cnt < i2c.send_amount)
					{
						I2C_SendData(I2C1, i2c.tx[i2c.cnt++]);
					}
					else
					{
						I2C_GenerateSTOP(I2C1, ENABLE);
						I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
					}
					break;
			}
			break;
		case RECEIVE:
			switch (I2C_GetLastEvent(I2C1))
			{
				case I2C_EVENT_MASTER_MODE_SELECT:
					if(i2c.stat == 0)
					{
						I2C_Send7bitAddress(I2C1, i2c.addr, I2C_Direction_Transmitter);
					}
					else
					{
						I2C_Send7bitAddress(I2C1, i2c.addr, I2C_Direction_Receiver);
						i2c.stat=0;
					}

					break;
				case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
					I2C_SendData(I2C1, i2c.tx[0]);
					break;
				case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
					i2c.stat=1;
					I2C_GenerateSTART(I2C1, ENABLE);
					i2c.cnt=0;
					break;
				case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:

					break;
				case I2C_EVENT_MASTER_BYTE_RECEIVED:
					i2c.rx[i2c.cnt++] = I2C_ReceiveData(I2C1);
					if(i2c.cnt==1)
					{
						I2C_AcknowledgeConfig(I2C1, DISABLE);
						I2C_GenerateSTOP(I2C1, ENABLE);
					}
					if(i2c.cnt==2)
					{
						I2C_AcknowledgeConfig(I2C1, ENABLE);
						((unsigned char*)&ext_adc)[0]=i2c.rx[1];
						((unsigned char*)&ext_adc)[1]=i2c.rx[0];
						i2c.err=0;i2c_debug++;
						I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
					}
					break;
			}
			break;
	}
}

void I2C1_ER_IRQHandler(void)
{
	/* Check on I2C2 AF flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_AF))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
		I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
	}
}
