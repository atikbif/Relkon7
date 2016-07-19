/*
 * rk.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "rk.h"
#include "main.h"
#include "crc.h"
#include "htime.h"
#include "fram.h"
#include "mmb.h"
#include "FreeRTOS.h"


extern plc_stat _Sys;
extern tm times,wr_times;
extern volatile unsigned char _Sys_SPI_Buzy;
//extern portTickType RFxLastExecutionTime;
extern portTickType PCxLastExecutionTime;
extern portTickType PUxLastExecutionTime;
extern portTickType PRxLastExecutionTime;
extern portTickType MBxLastExecutionTime;
extern portTickType ExLastExecutionTime;
//extern portTickType WFxLastExecutionTime;

extern unsigned char   IN[32];
extern unsigned char   OUT[32];
extern volatile unsigned char _Sys_IN[6];
extern volatile unsigned char _Sys_OUT[6];
extern volatile unsigned short _Sys_ADC[8];
extern volatile unsigned short _Sys_DAC[4];
extern volatile short _EA[8];

extern mmb_ain _ADC;
extern mmb_dac _DAC;

extern unsigned char plc[8],err[8];
extern unsigned char exch_addr[9];

unsigned char TX[64];
unsigned char RX[64];


volatile unsigned char obj_name[20]={'0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

const unsigned char Relkon_Ver[]={"Relkon 6.0 vers  28"};
const unsigned char MCU_Type[]={"STM32F207VCT6 CORTEX"};
static const unsigned char RF_Type[]={"Canal: RADIO CC2500 "};
static const unsigned char PC_Type[]={"Canal: PC/DEVICE    "};
static const unsigned char PU_Type[]={"Canal: PU           "};
static const unsigned char PR_Type[]={"Canal: PROG         "};
static const unsigned char MB_Type[]={"Canal: MATCHBOX     "};
static const unsigned char UDP_Type[]={"Canal: UDP          "};
static const unsigned char WF_Type[]={"Canal: WIFI         "};

static const unsigned char ascii_code[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

static unsigned short get_error(request* req);
static unsigned short bin_to_ascii(unsigned char* ptr,unsigned short lng);
static unsigned short convert_to_bin(unsigned short start_pos,unsigned short end_pos,unsigned char* ptr);
static char check_ascii(unsigned char symb);

static unsigned short get_error(request* req)
{
	unsigned short tmp;
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xFF;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4); else return(bin_to_ascii(req->tx_buf,4));
}

static unsigned short bin_to_ascii(unsigned char* ptr,unsigned short lng)
{
    unsigned short  tmp=lng;
    while(lng){ptr[((lng-1)<<1)+1]=ascii_code[ptr[lng-1]>>4];ptr[((lng-1)<<1)+2]=ascii_code[ptr[lng-1]&0x0F];lng--;}
    ptr[0]='!';ptr[((tmp-1)<<1)+3]=0x0D;
    return ((tmp<<1)+2);
}

static unsigned short convert_to_bin(unsigned short start_pos,unsigned short end_pos,unsigned char* ptr)
{
    unsigned short lng,tmp;
    char code_h,code_l;
    lng=end_pos-start_pos+1;
    if(lng & 0x01) return 0;
    lng=lng>>1;
    for(tmp=0;tmp<lng;tmp++)
    {
        code_h=check_ascii(ptr[start_pos+(tmp<<1)]);
        code_l=check_ascii(ptr[start_pos+(tmp<<1)+1]);
        if((code_h<0)||(code_l<0)) return 0;
        ptr[tmp]=(code_h<<4)|(code_l);
    }
    return lng;
}

static char check_ascii(unsigned char symb)
{
    unsigned char tmp;
    for(tmp=0;tmp<16;tmp++) if(symb==ascii_code[tmp]) return tmp;
    return -1;
}

unsigned short get_obj_name(request* req)	//0xA2
{
	unsigned short tmp;
	for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=obj_name[tmp];
	req->tx_buf[0]=0xFF;
	req->tx_buf[1]=0xA2;
	tmp=GetCRC16(req->tx_buf,22);
	req->tx_buf[22]=tmp>>8;
	req->tx_buf[23]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(24);else return(bin_to_ascii(req->tx_buf,24));
}

unsigned short get_software_ver(request* req)	//0xA0
{
	unsigned short tmp;
	for(tmp=0;tmp<19;tmp++) req->tx_buf[2+tmp]=Relkon_Ver[tmp];
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xA0;
	tmp=GetCRC16(req->tx_buf,21);
	req->tx_buf[21]=tmp>>8;
	req->tx_buf[22]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(23);else return(bin_to_ascii(req->tx_buf,23));
}

unsigned short get_hardware_ver(request* req)	//0xA1
{
	unsigned short tmp;
	for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=MCU_Type[tmp];
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xA1;
	tmp=GetCRC16(req->tx_buf,22);
	req->tx_buf[22]=tmp>>8;
	req->tx_buf[23]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(24);else return(bin_to_ascii(req->tx_buf,24));
}

unsigned short get_can_name(request* req)
{
	unsigned short tmp;
	switch(req->can_name)
	{
//		case CAN_RF:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=RF_Type[tmp];break;
		case CAN_PC:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=PC_Type[tmp];break;
		case CAN_PU:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=PU_Type[tmp];break;
		case CAN_PR:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=PR_Type[tmp];break;
		case CAN_MB:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=MB_Type[tmp];break;
		case CAN_UDP:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=UDP_Type[tmp];break;
		//case CAN_WF:for(tmp=0;tmp<20;tmp++) req->tx_buf[2+tmp]=WF_Type[tmp];break;
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xA2;
	tmp=GetCRC16(req->tx_buf,22);
	req->tx_buf[22]=tmp>>8;
	req->tx_buf[23]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(24);else return(bin_to_ascii(req->tx_buf,24));
}

unsigned short read_mem(request* req)	//0xD0
{
	unsigned short tmp;
	for(tmp=0;tmp<req->cnt;tmp++)  req->tx_buf[1+tmp]=*(&_Sys.Mem.b1[0]+req->addr+tmp);
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short read_time(request* req)	//0xD1
{
	unsigned short tmp;
	if(req->cnt > 128){return(get_error(req));}
	for(tmp=0;tmp < req->cnt;tmp++)
	{
		switch(req->addr + tmp)
		{
			case 0x00:req->tx_buf[1+tmp]=(times.sec%10)|((times.sec/10)<<4);break;
			case 0x01:req->tx_buf[1+tmp]=(times.min%10)|((times.min/10)<<4);break;
			case 0x02:req->tx_buf[1+tmp]=(times.hour%10)|((times.hour/10)<<4);break;
			case 0x03:req->tx_buf[1+tmp]=(times.day%10)|((times.day/10)<<4);break;
			case 0x04:req->tx_buf[1+tmp]=(times.date%10)|((times.date/10)<<4);break;
			case 0x05:req->tx_buf[1+tmp]=(times.month%10)|((times.month/10)<<4);break;
			case 0x06:req->tx_buf[1+tmp]=(times.year%10)|((times.year/10)<<4);break;
			default:req->tx_buf[1+tmp]=0;
		}
	}
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short read_frmem(request* req)	//0xD3
{
	unsigned short tmp;
	unsigned char err_cnt=0;
	if(req->cnt > 128){return(get_error(req));}
	switch(req->can_name)
	{
//		case CAN_RF:while(_Sys_SPI_Buzy) vTaskDelayUntil(&RFxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);break;
		case CAN_PC:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PCxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_PU:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PUxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_PR:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PRxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_MB:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&MBxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_UDP:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		//case CAN_WF:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&WFxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
	}

	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	//portENABLE_INTERRUPTS();
	read_data(req->addr>>8,req->addr&0xFF,req->cnt,&req->tx_buf[1]);
	//portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short read_ram(request* req)	//0xD4
{
	unsigned short tmp;
	unsigned char* XRAM_Ptr = 0;
	if(req->cnt>500){return(get_error(req));}
	for(tmp=0;tmp < req->cnt;tmp++)  req->tx_buf[1+tmp]=XRAM_Ptr[0x20000000+req->addr+tmp];
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short read_flash(request* req)	//0xD5
{
	unsigned short tmp;
	unsigned char* XRAM_Ptr = 0;
	if(req->cnt > 500){return(get_error(req));}
	for(tmp=0;tmp < req->cnt;tmp++)  req->tx_buf[1+tmp]=XRAM_Ptr[0x8000000+req->laddr+tmp];
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short read_preset(request* req)	//0xD6
{
	unsigned short  tmp;
	if(req->cnt > 500){return(get_error(req));}
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<1024) req->tx_buf[1+tmp]=_Sys.FR.b1[req->addr+tmp];else req->tx_buf[1+tmp]=0;
	}
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short write_mem(request* req)	//0xE0
{
	unsigned short tmp;
	for(tmp=0;tmp < req->cnt;tmp++)  *(&_Sys.Mem.b1[0]+req->addr+tmp)=req->rx_buf[4+tmp];
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xE0;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4);else return(bin_to_ascii(req->tx_buf,4));
}

unsigned short write_time(request* req)	//0xE1
{
	unsigned short tmp;
	if(req->cnt>128){return(get_error(req));}
	wr_times.sec=times.sec;wr_times.min=times.min;wr_times.hour=times.hour;
	wr_times.date=times.date;wr_times.month=times.month;wr_times.year=times.year;
	wr_times.day=times.day;
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		switch(req->addr+tmp)
		{
			case 0x00:wr_times.sec=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
			case 0x01:wr_times.min=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
			case 0x02:wr_times.hour=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
			case 0x03:wr_times.day=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
			case 0x04:wr_times.date=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
			case 0x05:wr_times.month=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
			case 0x06:wr_times.year=(req->rx_buf[4+tmp]&0x0F)+((req->rx_buf[4+tmp]>>4)*10);break;
		}
	}
	set_time();

	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xE1;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4);else return(bin_to_ascii(req->tx_buf,4));
}

unsigned short write_frmem(request* req)	//0xE3
{
	unsigned short tmp;
	unsigned char err_cnt=0;
	if(req->cnt > 128){return(get_error(req));}
	switch(req->can_name)
	{
//		case CAN_RF:while(_Sys_SPI_Buzy) vTaskDelayUntil(&RFxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);break;
		case CAN_PC:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PCxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_PU:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PUxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_PR:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PRxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_MB:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&MBxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_UDP:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		//case CAN_WF:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&WFxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
	}
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	//portENABLE_INTERRUPTS();
	write_enable();
	write_data(req->addr >> 8,req->addr & 0xFF,req->cnt,&req->rx_buf[6]);
	//portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xE3;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4);else return(bin_to_ascii(req->tx_buf,4));
}

unsigned short write_ram(request* req)	//0xE4
{
	unsigned short tmp;
	unsigned char* XRAM_Ptr = 0;
	if(req->cnt > 500){return(get_error(req));}
	for(tmp=0;tmp<req->cnt;tmp++)  XRAM_Ptr[0x20000000+req->addr+tmp]=req->rx_buf[6+tmp];
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xE4;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4);else return(bin_to_ascii(req->tx_buf,4));
}

unsigned short write_preset(request* req)	//0xE6
{
	unsigned short  tmp;
	unsigned char err_cnt=0;
	if((req->cnt > 128) ||(req->addr + req->cnt > 1024) || (req->cnt==0)) {return(get_error(req));}
	switch(req->can_name)
	{
//		case CAN_RF:while(_Sys_SPI_Buzy) vTaskDelayUntil(&RFxLastExecutionTime,(portTickType)1/portTICK_RATE_MS );break;
		case CAN_PC:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PCxLastExecutionTime,(portTickType)1/portTICK_RATE_MS );err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_PU:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PUxLastExecutionTime,(portTickType)1/portTICK_RATE_MS );err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_PR:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PRxLastExecutionTime,(portTickType)1/portTICK_RATE_MS );err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_MB:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&MBxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		case CAN_UDP:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		//case CAN_WF:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&WFxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
	}
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	//portENABLE_INTERRUPTS();
	write_enable();
	req->addr+=0x7B00;
	write_data(req->addr>>8,req->addr&0xFF,req->cnt,&req->rx_buf[6]);
	for(tmp=0;tmp<req->cnt;tmp++) _Sys.FR.b1[tmp+(req->addr-0x7B00)]=req->rx_buf[6+tmp];
	//portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xE6;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4);else return(bin_to_ascii(req->tx_buf,4));
}

unsigned short read_io(request* req)	//0xB0
{
	unsigned short  tmp;
	if(req->cnt > 500){return(get_error(req));}
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<6) req->tx_buf[1+tmp]=_Sys_IN[req->addr+tmp];
		else{
			if(req->addr+tmp<12) req->tx_buf[1+tmp]=_Sys_OUT[req->addr+tmp-6];
			else{
				if(req->addr+tmp<28)
				{
					switch((req->addr+tmp-12)%2)
					{
						case 0x00:req->tx_buf[1+tmp]=_Sys_ADC[(req->addr+tmp-12)>>1] & 0xFF;break;
						case 0x01:req->tx_buf[1+tmp]=_Sys_ADC[(req->addr+tmp-12)>>1]>>8;break;
					}
				}else{
					if(req->addr+tmp<36)
					{
						switch((req->addr+tmp-28)%2)
						{
							case 0x00:req->tx_buf[1+tmp]=_Sys_DAC[(req->addr+tmp-28)>>1] & 0xFF;break;
							case 0x01:req->tx_buf[1+tmp]=_Sys_DAC[(req->addr+tmp-28)>>1]>>8;break;
						}
					}
					else{
						if(req->addr+tmp<68) req->tx_buf[1+tmp]=IN[req->addr+tmp-36];
						else{
							if(req->addr+tmp<100) req->tx_buf[1+tmp]=OUT[req->addr+tmp-68];
							else{
								if(req->addr+tmp<356)
								{
									switch((req->addr+tmp-100)%8)
									{
										case 0x00:req->tx_buf[1+tmp]=_ADC.D1[(req->addr+tmp-100)>>3] & 0xFF;break;
										case 0x01:req->tx_buf[1+tmp]=_ADC.D1[(req->addr+tmp-100)>>3]>>8;break;
										case 0x02:req->tx_buf[1+tmp]=_ADC.D2[(req->addr+tmp-100)>>3] & 0xFF;break;
										case 0x03:req->tx_buf[1+tmp]=_ADC.D2[(req->addr+tmp-100)>>3]>>8;break;
										case 0x04:req->tx_buf[1+tmp]=_ADC.D3[(req->addr+tmp-100)>>3] & 0xFF;break;
										case 0x05:req->tx_buf[1+tmp]=_ADC.D3[(req->addr+tmp-100)>>3]>>8;break;
										case 0x06:req->tx_buf[1+tmp]=_ADC.D4[(req->addr+tmp-100)>>3] & 0xFF;break;
										case 0x07:req->tx_buf[1+tmp]=_ADC.D4[(req->addr+tmp-100)>>3]>>8;break;
									}
								}else{
									if(req->addr+tmp<484)
									{
										switch((req->addr+tmp-356)%4)
										{
											case 0x00:req->tx_buf[1+tmp]=_DAC.D1[(req->addr+tmp-356)>>2] & 0xFF;break;
											case 0x01:req->tx_buf[1+tmp]=_DAC.D1[(req->addr+tmp-356)>>2]>>8;break;
											case 0x02:req->tx_buf[1+tmp]=_DAC.D2[(req->addr+tmp-356)>>2] & 0xFF;break;
											case 0x03:req->tx_buf[1+tmp]=_DAC.D2[(req->addr+tmp-356)>>2]>>8;break;
										}
									}else {
										if(req->addr+tmp<500)
										{
											switch((req->addr+tmp-484)%2)
											{
												case 0x00:req->tx_buf[1+tmp]=(_EA[(req->addr+tmp-484)>>1]>0 ? (_EA[(req->addr+tmp-484)>>1]*2) : 0) & 0xFF;break;
												case 0x01:req->tx_buf[1+tmp]=(_EA[(req->addr+tmp-484)>>1]>0 ? (_EA[(req->addr+tmp-484)>>1]*2) : 0)>>8;break;
											}
										}else req->tx_buf[1+tmp]=0x00;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	req->tx_buf[0]=_Sys.Adr;
	tmp=GetCRC16(req->tx_buf,req->cnt+1);
	req->tx_buf[req->cnt+1]=tmp>>8;
	req->tx_buf[req->cnt+2]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(req->cnt+3);else return(bin_to_ascii(req->tx_buf,req->cnt+3));
}

unsigned short write_io(request* req)	//0xB1
{
	unsigned short  tmp;
	if(req->cnt > 500){return(get_error(req));}
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<6) _Sys_IN[req->addr+tmp]=req->rx_buf[6+tmp];
		else{
			if(req->addr+tmp<12) _Sys_OUT[req->addr+tmp-6]=req->rx_buf[6+tmp];
			else{
				if(req->addr+tmp<28)
				{
					switch((req->addr+tmp-12)%2)
					{
						case 0x00:
						_Sys_ADC[(req->addr+tmp-12)>>1]&=0xFF00;_Sys_ADC[(req->addr+tmp-12)>>1]|=req->rx_buf[6+tmp];break;
						case 0x01:
						_Sys_ADC[(req->addr+tmp-12)>>1]&=0x00FF;_Sys_ADC[(req->addr+tmp-12)>>1]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
					}
				}else{
					if(req->addr+tmp<36)
					{
						switch((req->addr+tmp-28)%2)
						{
							case 0x00:
							_Sys_DAC[(req->addr+tmp-28)>>1]&=0xFF00;_Sys_DAC[(req->addr+tmp-28)>>1]|=req->rx_buf[6+tmp];break;
							case 0x01:
							_Sys_DAC[(req->addr+tmp-28)>>1]&=0x00FF;_Sys_DAC[(req->addr+tmp-28)>>1]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
						}
					}
					else{
						if(req->addr+tmp<68) IN[req->addr+tmp-36]=req->rx_buf[6+tmp];
						else{
							if(req->addr+tmp<100) OUT[req->addr+tmp-68]=req->rx_buf[6+tmp];
							else{
								if(req->addr+tmp<356)
								{
									switch((req->addr+tmp-100)%8)
									{
										case 0x00:
										_ADC.D1[(req->addr+tmp-100)>>3]&=0xFF00;_ADC.D1[(req->addr+tmp-100)>>3]|=req->rx_buf[6+tmp];break;
										case 0x01:
										_ADC.D1[(req->addr+tmp-100)>>3]&=0x00FF;_ADC.D1[(req->addr+tmp-100)>>3]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
										case 0x02:
										_ADC.D2[(req->addr+tmp-100)>>3]&=0xFF00;_ADC.D2[(req->addr+tmp-100)>>3]|=req->rx_buf[6+tmp];break;
										case 0x03:
										_ADC.D2[(req->addr+tmp-100)>>3]&=0x00FF;_ADC.D2[(req->addr+tmp-100)>>3]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
										case 0x04:
										_ADC.D3[(req->addr+tmp-100)>>3]&=0xFF00;_ADC.D3[(req->addr+tmp-100)>>3]|=req->rx_buf[6+tmp];break;
										case 0x05:
										_ADC.D3[(req->addr+tmp-100)>>3]&=0x00FF;_ADC.D3[(req->addr+tmp-100)>>3]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
										case 0x06:
										_ADC.D4[(req->addr+tmp-100)>>3]&=0xFF00;_ADC.D4[(req->addr+tmp-100)>>3]|=req->rx_buf[6+tmp];break;
										case 0x07:
										_ADC.D4[(req->addr+tmp-100)>>3]&=0x00FF;_ADC.D4[(req->addr+tmp-100)>>3]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
									}
								}else{
									if(req->addr+tmp<484)
									{
										switch((req->addr+tmp-356)%4)
										{
											case 0x00:
											_DAC.D1[(req->addr+tmp-356)>>2]&=0xFF00;_DAC.D1[(req->addr+tmp-356)>>2]|=req->rx_buf[6+tmp];break;
											case 0x01:
											_DAC.D1[(req->addr+tmp-356)>>2]&=0x00FF;_DAC.D1[(req->addr+tmp-356)>>2]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
											case 0x02:
											_DAC.D2[(req->addr+tmp-356)>>2]&=0xFF00;_DAC.D2[(req->addr+tmp-356)>>2]|=req->rx_buf[6+tmp];break;
											case 0x03:
											_DAC.D2[(req->addr+tmp-356)>>2]&=0x00FF;_DAC.D2[(req->addr+tmp-356)>>2]|=((unsigned short)req->rx_buf[6+tmp])<<8;break;
										}
									}else {
										if(req->addr+tmp<500)
										{
											switch((req->addr+tmp-484)%2)
											{
												case 0x01:_EA[(req->addr+tmp-484)>>1] = ((((unsigned short)req->rx_buf[6+tmp])<<8) | req->rx_buf[5+tmp])>>1;break;
											}
										}else req->tx_buf[1+tmp]=0x00;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0xB1;
	tmp=GetCRC16(req->tx_buf,2);
	req->tx_buf[2]=tmp>>8;
	req->tx_buf[3]=tmp&0xFF;
	if(req->mode==BIN_MODE) return(4);else return(bin_to_ascii(req->tx_buf,4));
}

unsigned short exchange_cmd(request* req)
{
	unsigned short tmp=0;
	unsigned char plcNum=0;
	for(tmp=0;tmp<8;tmp++) {
		if(req->rx_buf[2]==exch_addr[tmp+1]) {plcNum=tmp+1;break;}
	}
	if(plcNum)
	{
		for(tmp=0;tmp<64;tmp++) RX[tmp]=req->rx_buf[3+tmp];
		plc[plcNum-1]=0;err[plcNum-1]=0;
		req->tx_buf[0]=req->rx_buf[2];
		req->tx_buf[1]=0xE5;
		req->tx_buf[2]=_Sys.Adr;
		for(tmp=0;tmp<64;tmp++) req->tx_buf[3+tmp]=TX[tmp];
		tmp=GetCRC16(req->tx_buf,67);
		req->tx_buf[67]=tmp>>8;
		req->tx_buf[68]=tmp&0xFF;
		return(69);
	}
	return(0);
}

unsigned short check_ascii_rk(request* req)
{
	unsigned short tmp=0,ascii_end,ascii_start;
	if(req->rx_buf[req->cnt-1]==0x0D)
	{
		tmp=ascii_end=req->cnt-2;
		while(tmp){if(req->rx_buf[tmp]=='$') break;tmp--;}
		ascii_start = tmp+1;
		tmp=convert_to_bin(ascii_start,ascii_end,req->rx_buf);
		if(tmp) {if((GetCRC16(req->rx_buf,tmp))||((req->rx_buf[0]!=_Sys.Adr)&&(req->rx_buf[0]!=0x00)&&(req->rx_buf[0]!=0xFF))) tmp=0;}
	}
	return tmp;
}

