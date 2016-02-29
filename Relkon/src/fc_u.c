/*
 * fc_u.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "fc_u.h"
#include "additional.h"

const unsigned short S4_max=1;

const unsigned char mod_table[]=
{0,0x01,0x02,0x03,0x41,0x42,0x43,0x81,0x82,0x00,0x83,0xA1,0xA2,0x00,0x00};


struct process
{
    unsigned long DELAY;
    unsigned int SIT;
}_Sys4x_p0;



void R100Task( void *pvParameters )
{
    portTickType xLastExecutionTime;

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64); // IWDG counter clock: 40KHz(LSI) / 64  (1.6 ms)
    IWDG_SetReload(150); //Set counter reload value
    IWDG_ReloadCounter();
    IWDG_Enable();
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {
        if ( _Sys4x_p0.DELAY == 0 )
            switch (_Sys4x_p0.SIT)
            {
                case 1:
                    /*11*/_Sys4x_p0.DELAY=0;
                    break;
            }
        IWDG_ReloadCounter();
        r100++;
        vTaskDelayUntil( &xLastExecutionTime, R100_DELAY );
    }
}

void R1000Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil( &xLastExecutionTime, R1000_DELAY );
    }
}

void R1Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {
        if ( _Sys4x_p0.DELAY != 0 )
            _Sys4x_p0.DELAY--;
        r1++;
        vTaskDelayUntil( &xLastExecutionTime, R1_DELAY );
    }
}
void R5Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {

        r5++;
        vTaskDelayUntil( &xLastExecutionTime, R5_DELAY );
    }
}
void R10Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {
        r10++;
        vTaskDelayUntil( &xLastExecutionTime, R10_DELAY );
    }
}
void Relkon_init()
{
    /*6*/_Sys4x_p0.SIT = 1; _Sys4x_p0.DELAY = 0;
}

/***********PultDataCI***************/
const unsigned char str1[][20] = {
    "\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40",
};
const unsigned char str2[][20] = {
    "\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40",
};
const unsigned char str3[][20] = {
    "\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40",
};
const unsigned char str4[][20] = {
    "\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40\40",
};
void print_var(void)
{
    switch(_Sys.S1)
    {
        default: break;
    }
    switch(_Sys.S2)
    {
        default: break;
    }
    switch(_Sys.S3)
    {
        default: break;
    }
    switch(_Sys.S4)
    {
        default: break;
    }
}


