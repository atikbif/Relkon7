/*
 * fc_u.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "fc_u.h"
#include "additional.h"
#include "mmb.h"

const unsigned short S4_max=1;
const unsigned short mb_req_count=2;
const unsigned short modbus_delay = 100;
unsigned short mvar_1,mvar_2,mvar_3,mvar_4,mvar_5;
unsigned short mvar_6,mvar_7,mvar_8;

const mvar mb_req1_vars[] = {
	{0,-1,&mvar_1},
	{2,-1,&mvar_2},
	{4,-1,&mvar_3},
	{6,-1,&mvar_4},
	{8,-1,&mvar_5},
	{10,-1,&mvar_6}
};
const mvar mb_req2_vars[] = {
	{0,-1,&mvar_7},
	{2,-1,&mvar_8}
};

const mvar_reqs mb_mvar_reqs[] = {
	{"\x01\x03\x00\x01\x00\x06\x94\x08", 8, 17, mb_req1_vars, 6, 0},
	{"\x01\x03\x00\x5a\x00\x02\xe4\x18", 8, 9, mb_req2_vars, 2, 0}
};

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


