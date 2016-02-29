/*
 * modbus.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "modbus.h"
#include "main.h"
#include "crc.h"
#include "fram.h"
#include "mmb.h"
#include "FreeRTOS.h"

#define MEM_2(a)           _Sys.Mem.b2[(a)]

extern unsigned char   IN[32];
extern unsigned char   OUT[32];
extern volatile unsigned char _Sys_IN[6];
extern volatile unsigned char _Sys_OUT[6];
extern volatile unsigned short _Sys_ADC[8];
extern volatile unsigned short _Sys_DAC[4];

extern volatile unsigned char _Sys_SPI_Buzy;

extern mmb_ain _ADC;
extern mmb_dac _DAC;

extern plc_stat _Sys;
extern portTickType PCxLastExecutionTime;
//extern portTickType RFxLastExecutionTime;
extern portTickType PRxLastExecutionTime;

static const unsigned char ascii_code[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

static unsigned short get_error(request* req, unsigned char code);
static unsigned short bin_to_ascii(unsigned char* ptr,unsigned short lng);
static unsigned short convert_to_bin(unsigned short start_pos,unsigned short end_pos,unsigned char* ptr);
static char check_ascii(unsigned char symb);

static unsigned short get_error(request* req, unsigned char code)
{
	unsigned short tmp;
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x80 | req->rx_buf[1];
	req->tx_buf[2]=code;
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,3);
		req->tx_buf[3]=tmp>>8;
		req->tx_buf[4]=tmp&0xFF;
		return(5);
	}
	else
	{
		req->tx_buf[3]=getLRC(req->tx_buf,3);
		return(bin_to_ascii(req->tx_buf,4));
	}
}

static unsigned short bin_to_ascii(unsigned char* ptr,unsigned short lng)
{
    unsigned short  tmp=lng;
    while(lng){ptr[((lng-1)<<1)+1]=ascii_code[ptr[lng-1]>>4];ptr[((lng-1)<<1)+2]=ascii_code[ptr[lng-1]&0x0F];lng--;}
    ptr[0]=':';ptr[((tmp-1)<<1)+3]=0x0D;ptr[((tmp-1)<<1)+4]=0x0A;
    return ((tmp<<1)+3);
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

unsigned short read_coils(request* req)
{
	unsigned short tmp;
	unsigned short byte_count;
	if((req->cnt > 176)||(req->cnt == 0)) return(get_error(req,0x03));
	if(req->addr + req->cnt > 176) return(get_error(req,0x02));
	byte_count=req->cnt >> 3;
	if(req->cnt != (byte_count<<3)) byte_count++;
	for(tmp=0;tmp<byte_count;tmp++) req->tx_buf[3+tmp]=0;
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<48)  {if(_Sys_OUT[(req->addr+tmp)>>3]&(1<<((req->addr+tmp)%8))) req->tx_buf[3+(tmp>>3)]|=1<<(tmp%8);}
		else {if(OUT[(req->addr+tmp-48)>>2]&(1<<((req->addr+tmp-48)%4))) req->tx_buf[3+(tmp>>3)]|=1<<(tmp%8);}
	}
	req->tx_buf[1]=0x01;
	req->tx_buf[2]=byte_count;
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,3+byte_count);
		req->tx_buf[3+byte_count]=tmp>>8;
		req->tx_buf[4+byte_count]=tmp&0xFF;
		return(5+byte_count);
	}
	else
	{
		req->tx_buf[3+byte_count]=getLRC(req->tx_buf,3+byte_count);
		return(bin_to_ascii(req->tx_buf,4+byte_count));
	}
}

unsigned short read_dinputs(request* req)
{
	unsigned short tmp,byte_count;
	if((req->cnt>176)||(req->cnt==0)) return(get_error(req,0x03));
	if(req->addr + req->cnt > 176) return(get_error(req,0x02));
	byte_count = req->cnt >> 3;
	if(req->cnt != (byte_count<<3)) byte_count++;
	for(tmp=0;tmp<byte_count;tmp++) req->tx_buf[3+tmp]=0;
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr + tmp < 48)  {if(_Sys_IN[(req->addr+tmp)>>3]&(1<<((req->addr+tmp)%8))) req->tx_buf[3+(tmp>>3)]|=1<<(tmp%8);}
		else {if(IN[(req->addr+tmp-48)>>2]&(1<<((req->addr+tmp-48)%4))) req->tx_buf[3+(tmp>>3)]|=1<<(tmp%8);}
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x02;
	req->tx_buf[2]=byte_count;
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,3+byte_count);
		req->tx_buf[3+byte_count]=tmp>>8;
		req->tx_buf[4+byte_count]=tmp&0xFF;
		return(5+byte_count);
	}
	else
	{
		req->tx_buf[3+byte_count]=getLRC(req->tx_buf,3+byte_count);
		return(bin_to_ascii(req->tx_buf,4+byte_count));
	}
}

unsigned short read_holdregs(request* req)
{
	unsigned short tmp,byte_count;
	unsigned char err_cnt=0;
	if((req->cnt >= 129)||(req->cnt == 0)||((req->cnt == 128)&&(req->addr >= 0x8000))) return(get_error(req,0x03));
	if((req->addr + req->cnt >= 129)&&(req->addr < 0x8000)) return(get_error(req,0x02));
	if(req->addr<0x8000)
	{
		for(tmp=0;tmp<req->cnt;tmp++)
		{req->tx_buf[3+tmp*2]=MEM_2(req->addr+tmp)>>8;req->tx_buf[4+tmp*2]=MEM_2(req->addr+tmp)&0xFF;}
	}
	else
	{
		switch(req->can_name)
		{
			case CAN_PC:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PCxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
			case CAN_PR:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PRxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		}

		portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=1;
		//portENABLE_INTERRUPTS();
		req->addr-=0x8000;
		req->addr *= 2;
		read_data((req->addr)>>8,(req->addr)&0xFF,req->cnt<<1,&req->tx_buf[3]);
		for(tmp=0;tmp<req->cnt;tmp++) {byte_count=req->tx_buf[3+tmp*2];req->tx_buf[3+tmp*2]=req->tx_buf[4+tmp*2];req->tx_buf[4+tmp*2]=byte_count;}
		//portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=0;
		portENABLE_INTERRUPTS();
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x03;
	req->tx_buf[2]=req->cnt*2;
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,3+ req->cnt*2);
		req->tx_buf[3+req->cnt*2]=tmp>>8;
		req->tx_buf[4+req->cnt*2]=tmp&0xFF;
		return(5+req->cnt*2);
	}
	else
	{
		req->tx_buf[3+req->cnt*2]=getLRC(req->tx_buf,3+req->cnt*2);
		return(bin_to_ascii(req->tx_buf,4+req->cnt*2));
	}
}

unsigned short read_inregs(request* req) // adc
{
	unsigned short tmp;
	if((req->cnt>=129)||(req->cnt==0)) return(get_error(req,0x03));
	if(req->addr+req->cnt>=129) return(get_error(req,0x02));
	for(tmp=0;tmp<req->cnt*2;tmp++)
	{
		if(req->addr+tmp<16)
		{
			if(tmp&0x01) req->tx_buf[3+tmp]=_Sys_ADC[req->addr+(tmp>>1)] & 0xFF;else req->tx_buf[3+tmp]=_Sys_ADC[req->addr+(tmp>>1)] >> 8;
		}
		else req->tx_buf[3+tmp]=0;
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x04;
	req->tx_buf[2]=req->cnt*2;
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,3+req->cnt*2);
		req->tx_buf[3+req->cnt*2]=tmp>>8;
		req->tx_buf[4+req->cnt*2]=tmp&0xFF;
		return(5+req->cnt*2);
	}
	else
	{
		req->tx_buf[3+req->cnt*2]=getLRC(req->tx_buf,3+req->cnt*2);
		return(bin_to_ascii(req->tx_buf,4+req->cnt*2));
	}
}

unsigned short write_single_coil(request* req)
{
	unsigned short tmp;
	if((req->cnt)&&(req->cnt!=0xFF00)) return(get_error(req,0x03));
	if(req->addr>=176) return(get_error(req,0x02));
	if(req->addr<48) {if(req->cnt) _Sys_OUT[req->addr>>3]|=1<<(req->addr%8);else _Sys_OUT[req->addr>>3]&=~(1<<(req->addr%8));}
	else {if(req->cnt) OUT[(req->addr-48)>>2]|=1<<((req->addr-48)%4);else _Sys_OUT[(req->addr-48)>>2]&=~(1<<((req->addr-48)%4));}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x05;
	req->tx_buf[2]=req->rx_buf[2];
	req->tx_buf[3]=req->rx_buf[3];
	req->tx_buf[4]=req->rx_buf[4];
	req->tx_buf[5]=req->rx_buf[5];
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,6);
		req->tx_buf[6]=tmp>>8;
		req->tx_buf[7]=tmp&0xFF;
		return(8);
	}
	else
	{
		req->tx_buf[6]=getLRC(req->tx_buf,6);
		return(bin_to_ascii(req->tx_buf,7));
	}
}

unsigned short write_single_reg(request* req)
{
	unsigned short tmp,byte_count;
	unsigned char err_cnt=0;
	if((req->addr >= 128)&&(req->addr < 0x8000)) return(get_error(req,0x02));
	if(req->addr < 0x8000) _Sys.Mem.b2[req->addr]=req->cnt;
	else
	{
		req->addr-=0x8000;
		req->addr*=2;
		byte_count=req->rx_buf[4];req->rx_buf[4]=req->rx_buf[5];req->rx_buf[5]=byte_count;
		switch(req->can_name)
		{
			case CAN_PC:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PCxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
			case CAN_PR:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PRxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		}
		portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=1;
		//portENABLE_INTERRUPTS();
		write_enable();
		write_data((req->addr)>>8,(req->addr)&0xFF,2,&req->rx_buf[4]);
		if((req->addr>=0x7B00)&&(req->addr<0x7EFF)) // write EE in RAM
		{
			_Sys.FR.b1[req->addr - 0x7B00]=req->rx_buf[4];
			_Sys.FR.b1[req->addr - 0x7B00 + 1]=req->rx_buf[5];
		}
		//portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=0;
		portENABLE_INTERRUPTS();
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x06;
	req->tx_buf[2]=req->addr>>8;
	req->tx_buf[3]=req->addr&0xFF;
	req->tx_buf[4]=req->cnt>>8;
	req->tx_buf[5]=req->cnt&0xFF;
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,6);
		req->tx_buf[6]=tmp>>8;
		req->tx_buf[7]=tmp&0xFF;
		return(8);
	}
	else
	{
		req->tx_buf[6]=getLRC(req->tx_buf,6);
		return(bin_to_ascii(req->tx_buf,7));
	}
}

unsigned short write_multi_regs(request* req)
{
	unsigned short tmp,byte_count;
	unsigned char err_cnt=0;
	if((req->cnt >= 129)||(req->cnt == 0)||((req->cnt == 128)&&(req->addr >= 0x8000))) return(get_error(req,0x03));
	if((req->addr+req->cnt>=129)&&(req->addr<0x8000)) return(get_error(req,0x02));
	if(req->addr<0x8000)
	{
		for(tmp=0;tmp<req->cnt;tmp++)
			{MEM_2(req->addr+tmp)=req->rx_buf[8+tmp*2] | ((unsigned short)req->rx_buf[7+tmp*2]<<8);}
	}
	else
	{
		req->addr-=0x8000;req->addr*=2;
		for(tmp=0;tmp<req->cnt;tmp++)
		{
			byte_count=req->rx_buf[7+tmp*2];
			req->rx_buf[7+tmp*2]=req->rx_buf[8+tmp*2];
			req->rx_buf[8+tmp*2]=byte_count;
			if((req->addr>=0x7B00)&&(req->addr<0x7EFF))
			{
				_Sys.FR.b1[req->addr -0x7B00 + tmp*2] = req->rx_buf[7+tmp*2];
				_Sys.FR.b1[req->addr -0x7B00 +tmp*2 + 1] = req->rx_buf[8+tmp*2];
			}
		}
		switch(req->can_name)
		{
			case CAN_PC:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PCxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
			case CAN_PR:while(_Sys_SPI_Buzy) {vTaskDelayUntil(&PRxLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}break;
		}

		portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=1;
		//portENABLE_INTERRUPTS();
		write_enable();
		write_data((req->addr)>>8,(req->addr)&0xFF,req->cnt*2,&req->rx_buf[7]);
		//portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=0;
		portENABLE_INTERRUPTS();
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x10;
	req->tx_buf[2]=req->rx_buf[2];
	req->tx_buf[3]=req->rx_buf[3];
	req->tx_buf[4]=req->rx_buf[4];
	req->tx_buf[5]=req->rx_buf[5];
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,6);
		req->tx_buf[6]=tmp>>8;
		req->tx_buf[7]=tmp&0xFF;
		return(8);
	}
	else
	{
		req->tx_buf[6]=getLRC(req->tx_buf,6);
		return(bin_to_ascii(req->tx_buf,7));
	}
}

unsigned short write_multi_coils(request* req)
{
	unsigned short tmp;
	if((req->cnt>178)||(req->cnt==0)) return(get_error(req,0x03));
	if(req->addr + req->cnt > 178) return(get_error(req,0x02));
	 for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<48)
		{
			if((req->rx_buf[7+(tmp>>3)])&(1<<(tmp%8))) _Sys_OUT[(req->addr+tmp)>>3]|=1<<((tmp+req->addr)%8);else _Sys_OUT[(req->addr+tmp)>>3] &= ~(1<<((tmp+req->addr)%8));
		}
		else
		{
			if((req->rx_buf[7+(tmp>>3)])&(1<<(tmp%8))) OUT[(req->addr+tmp-48)>>2]|=1<<((tmp+req->addr)%4);else OUT[(req->addr+tmp-48)>>2] &= ~(1<<((tmp+req->addr)%4));
		}
	}
	req->tx_buf[0]=_Sys.Adr;
	req->tx_buf[1]=0x0F;
	req->tx_buf[2]=req->rx_buf[2];
	req->tx_buf[3]=req->rx_buf[3];
	req->tx_buf[4]=req->rx_buf[4];
	req->tx_buf[5]=req->rx_buf[5];
	if(req->mode==BIN_MODE)
	{
		tmp=GetCRC16(req->tx_buf,6);
		req->tx_buf[6]=tmp>>8;
		req->tx_buf[7]=tmp&0xFF;
		return(8);
	}
	else
	{
		req->tx_buf[6]=getLRC(req->tx_buf,6);
		return(bin_to_ascii(req->tx_buf,7));
	}
}

unsigned short check_ascii_modbus(request* req)
{
	unsigned short tmp=0,ascii_end,ascii_start;
	if((req->rx_buf[req->cnt-1]==0x0A)&&(req->rx_buf[req->cnt-2]==0x0D))
	{
		tmp=ascii_end=req->cnt-3;
		while(tmp){if(req->rx_buf[tmp]==':') break;tmp--;}
		ascii_start = tmp+1;
		tmp=convert_to_bin(ascii_start,ascii_end,req->rx_buf);
		if(tmp) {if(CheckLRC(req->rx_buf,tmp)==0) tmp=0;}
	}
	return tmp;
}

