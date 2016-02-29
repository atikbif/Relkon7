/*
 * fram.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "FreeRTOS.h"
#include "fram.h"

volatile unsigned char _Sys_SPI_Buzy=0;
// запись в FRAM
void write_fram(unsigned short adr,unsigned char size,unsigned char* ptr)
{
	volatile portTickType startTick,currentTick;
    if((unsigned long)ptr<0x20000000) return;
    startTick = xTaskGetTickCount();
    while(_Sys_SPI_Buzy){currentTick = xTaskGetTickCount();if(currentTick - startTick>=10) return;};
    portDISABLE_INTERRUPTS();
    _Sys_SPI_Buzy=1;
    portENABLE_INTERRUPTS();
    write_enable();
    write_data(adr>>8,adr&0xFF,size,ptr);
    portDISABLE_INTERRUPTS();
    _Sys_SPI_Buzy=0;
    portENABLE_INTERRUPTS();
}

// чтение из FRAM
void read_fram(unsigned short adr,unsigned char size,unsigned char* ptr)
{
	volatile portTickType startTick,currentTick;
	if((unsigned long)ptr<0x20000000) return;
	startTick = xTaskGetTickCount();
    while(_Sys_SPI_Buzy){currentTick = xTaskGetTickCount();if(currentTick - startTick>=10) return;};
    portDISABLE_INTERRUPTS();
    _Sys_SPI_Buzy=1;
    portENABLE_INTERRUPTS();
    read_data(adr>>8,adr&0xFF,size,ptr);
    portDISABLE_INTERRUPTS();
    _Sys_SPI_Buzy=0;
    portENABLE_INTERRUPTS();
}



