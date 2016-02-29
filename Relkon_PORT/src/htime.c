#include "htime.h"
#include "hfram.h"
#include "stm32f10x_conf.h"

tm times,wr_times;

unsigned long AsynchPrediv = 0, SynchPrediv = 0;

const unsigned long month_sec[13]={0,2678400,5097600,7776000,10368000,13046400,15638400,18316800,20995200,23587200,26265600,28857600,31536000};

void init_timer(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  	TIM_TimeBaseStructure.TIM_Prescaler = 720;//600;
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  	TIM_Cmd(TIM3, ENABLE);
}

void init_calendar(void)
{
	if(BKP_ReadBackupRegister(BKP_DR1) != 0xA1A1)
	{
		RTC_Config();
		BKP_WriteBackupRegister(BKP_DR1, 0xA1A1);
		RTC_WaitForLastTask();

		RTC_SetCounter(0x00000000);

		RTC_WaitForLastTask();
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		PWR_BackupAccessCmd(ENABLE);

		RTC_WaitForSynchro();
		RTC_WaitForLastTask();
	}
}

void set_time(void)
{
	unsigned long cur_time=0;
	if(wr_times.year>99) wr_times.year=0;
	cur_time+=wr_times.sec;
	cur_time+=wr_times.min*60;
	cur_time+=wr_times.hour*3600;
	cur_time+=month_sec[wr_times.month-1];
	cur_time+=(wr_times.date-1)*3600*24;
	RTC_WaitForLastTask();RTC_SetCounter(cur_time);
	write_enable();
	write_data(0x7F,0x56,1,(unsigned char*)&wr_times.year);
	times.year=wr_times.year;
}

void get_time(void)
{
	unsigned long cur_time;
	unsigned char sys_tmp;
	cur_time=RTC_GetCounter();
	while(cur_time>=31536000)
	{
		cur_time-=31536000;times.year++;
		if(times.year>99) times.year=0;
		RTC_WaitForLastTask();RTC_SetCounter(cur_time);
		write_enable();
		write_data(0x7F,0x56,1,(unsigned char*)&times.year);
	}
	for(sys_tmp=1;sys_tmp<13;sys_tmp++) {if(cur_time<month_sec[sys_tmp]) break;}
	times.month = sys_tmp;
	cur_time-=month_sec[sys_tmp-1];
	sys_tmp=1;
	while(cur_time>=86400){sys_tmp++;cur_time-=86400;}
	times.date=sys_tmp;
	sys_tmp=0;
	while(cur_time>=3600){sys_tmp++;cur_time-=3600;}
	times.hour=sys_tmp;
	times.min = (unsigned short)cur_time/60;
	times.sec = cur_time - times.min*60;
}

void RTC_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_DeInit();
	RCC_LSEConfig(RCC_LSE_ON);
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
	RTC_WaitForLastTask();
	RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	RTC_WaitForLastTask();
}
