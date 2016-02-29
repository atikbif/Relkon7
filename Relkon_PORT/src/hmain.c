/*
 * hmain.c
 *
 *  Created on: Mar 1, 2012
 *      Author: Роман
 */

#include "hmain.h"
#include "hfram.h"
#include "FreeRTOS.h"
#include "stm32f10x_conf.h"

extern volatile unsigned char _Sys_SPI_Buzy;
extern portTickType FLLastExecutionTime;

void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOE, &GPIO_InitStructure );
}

void toggle_led(void)
{
	GPIOE->ODR ^= GPIO_Pin_10;
}

void nvic_init(void)
{
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x2000 );
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

unsigned long get_idle_tmr(void)
{
	return(TIM3->CNT);
}

void update_code(void)
{
	while(_Sys_SPI_Buzy) vTaskDelayUntil( &FLLastExecutionTime, ( portTickType ) 1 / portTICK_RATE_MS );
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	portENABLE_INTERRUPTS();
	write_enable();write_data(0x7F, 0xF5, 10, (unsigned char*)"Relkon 001");
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
}

void reset_cmd(void)
{
	SCB->AIRCR = ((u32)0x05FA0000)| (u32)0x04;
}

