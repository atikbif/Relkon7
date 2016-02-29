/*
 * modbustcp.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "modbustcp.h"
#include "main.h"
#include "fram.h"
#include "FreeRTOS.h"

extern unsigned char   IN[32];
extern unsigned char   OUT[32];
extern volatile unsigned char _Sys_IN[6];
extern volatile unsigned char _Sys_OUT[6];
extern volatile unsigned short _Sys_ADC[8];
extern volatile unsigned short _Sys_DAC[4];

extern plc_stat _Sys;
extern portTickType ExLastExecutionTime;

extern volatile unsigned char _Sys_SPI_Buzy;

#define MEM_2(a)           _Sys.Mem.b2[(a)]

static unsigned short get_error(request* req, unsigned char code);

static unsigned short get_error(request* req, unsigned char code)
{
	unsigned short len;
	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id
	len = 2;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)

	req->tx_buf[7]=0x80 | req->rx_buf[1];
	req->tx_buf[8]=code;
	return(7+len);
}

unsigned short tcpread_coils(request* req)
{
	unsigned short tmp,len;
	unsigned short byte_count;

	if((req->cnt > 176)||(req->cnt == 0)) return(get_error(req,0x03));
	if(req->addr + req->cnt > 176) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	byte_count=req->cnt >> 3;
	if(req->cnt != (byte_count<<3)) byte_count++;
	for(tmp=0;tmp<byte_count;tmp++) req->tx_buf[7+2+tmp]=0;
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<48)  {if(_Sys_OUT[(req->addr+tmp)>>3]&(1<<((req->addr+tmp)%8))) req->tx_buf[7+2+(tmp>>3)]|=1<<(tmp%8);}
		else {if(OUT[(req->addr+tmp-48)>>2]&(1<<((req->addr+tmp-48)%4))) req->tx_buf[7+2+(tmp>>3)]|=1<<(tmp%8);}
	}
	len = 2 + byte_count;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)

	req->tx_buf[7]=0x01;
	req->tx_buf[7+1]=byte_count;
	return(7+len);

}

unsigned short tcpread_dinputs(request* req)
{
	unsigned short tmp,len;
	unsigned short byte_count;

	if((req->cnt>176)||(req->cnt==0)) return(get_error(req,0x03));
	if(req->addr + req->cnt > 176) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	byte_count = req->cnt >> 3;
	if(req->cnt != (byte_count<<3)) byte_count++;
	for(tmp=0;tmp<byte_count;tmp++) req->tx_buf[7+2+tmp]=0;
	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr + tmp < 48)  {if(_Sys_IN[(req->addr+tmp)>>3]&(1<<((req->addr+tmp)%8))) req->tx_buf[7+2+(tmp>>3)]|=1<<(tmp%8);}
		else {if(IN[(req->addr+tmp-48)>>2]&(1<<((req->addr+tmp-48)%4))) req->tx_buf[7+2+(tmp>>3)]|=1<<(tmp%8);}
	}
	len = 2 + byte_count;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)

	req->tx_buf[7]=0x02;
	req->tx_buf[7+1]=byte_count;
	return(7+len);
}

unsigned short tcpread_holdregs(request* req)
{
	unsigned short tmp,byte_count,len;
	unsigned char err_cnt=0;
	if((req->cnt >= 129)||(req->cnt == 0)||((req->cnt == 128)&&(req->addr >= 0x8000))) return(get_error(req,0x03));
	if((req->addr + req->cnt >= 129)&&(req->addr < 0x8000)) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	if(req->addr<0x8000)
	{
		for(tmp=0;tmp<req->cnt;tmp++)
		{req->tx_buf[7+2+tmp*2]=MEM_2(req->addr+tmp)>>8;req->tx_buf[7+3+tmp*2]=MEM_2(req->addr+tmp)&0xFF;}
	}
	else
	{
		while(_Sys_SPI_Buzy) {vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}
		portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=1;
		//portENABLE_INTERRUPTS();
		req->addr-=0x8000;
		req->addr *= 2;
		read_data((req->addr)>>8,(req->addr)&0xFF,req->cnt<<1,&req->tx_buf[7+2]);
		for(tmp=0;tmp<req->cnt;tmp++) {byte_count=req->tx_buf[7+2+tmp*2];req->tx_buf[7+2+tmp*2]=req->tx_buf[7+3+tmp*2];req->tx_buf[7+3+tmp*2]=byte_count;}
		//portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=0;
		portENABLE_INTERRUPTS();
	}
	len = 2 + req->cnt*2;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)
	req->tx_buf[7]=0x03;
	req->tx_buf[7+1]=req->cnt*2;
	return(7+len);
}

unsigned short tcpread_inregs(request* req) // adc
{
	unsigned short tmp,len;
	if((req->cnt>=129)||(req->cnt==0)) return(get_error(req,0x03));
	if(req->addr+req->cnt>=129) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	for(tmp=0;tmp<req->cnt*2;tmp++)
	{
		if(req->addr+tmp<16)
		{
			if(tmp&0x01) req->tx_buf[7+2+tmp]=_Sys_ADC[req->addr+(tmp>>1)] & 0xFF;else req->tx_buf[7+2+tmp]=_Sys_ADC[req->addr+(tmp>>1)] >> 8;
		}
		else req->tx_buf[7+2+tmp]=0;
	}
	len = 2 + req->cnt*2;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)
	req->tx_buf[7]=0x04;
	req->tx_buf[7+1]=req->cnt*2;
	return(7+len);
}

unsigned short tcpwrite_single_coil(request* req)
{
	unsigned short len;
	if((req->cnt)&&(req->cnt!=0xFF00)) return(get_error(req,0x03));
	if(req->addr>=176) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	if(req->addr<48) {if(req->cnt) _Sys_OUT[req->addr>>3]|=1<<(req->addr%8);else _Sys_OUT[req->addr>>3]&=~(1<<(req->addr%8));}
	else {if(req->cnt) OUT[(req->addr-48)>>2]|=1<<((req->addr-48)%4);else _Sys_OUT[(req->addr-48)>>2]&=~(1<<((req->addr-48)%4));}
	len = 5;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)
	req->tx_buf[7]=0x05;
	req->tx_buf[7+1]=req->rx_buf[7+1];
	req->tx_buf[7+2]=req->rx_buf[7+2];
	req->tx_buf[7+3]=req->rx_buf[7+3];
	req->tx_buf[7+4]=req->rx_buf[7+4];
	return(7+len);
}

unsigned short tcpwrite_single_reg(request* req)
{
	unsigned short byte_count,len;
	unsigned short tmp_addr;
	unsigned char err_cnt=0;
	if((req->addr >= 128)&&(req->addr < 0x8000)) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	if(req->addr < 0x8000) _Sys.Mem.b2[req->addr]=req->cnt;
	else
	{
		tmp_addr = req->addr - 0x8000;
		tmp_addr *= 2;

		byte_count=req->rx_buf[7+3];req->rx_buf[7+3]=req->rx_buf[7+4];req->rx_buf[7+4]=byte_count;
		//while(_Sys_SPI_Buzy) vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);
		while(_Sys_SPI_Buzy) {vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}
		portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=1;
		//portENABLE_INTERRUPTS();
		write_enable();
		write_data(tmp_addr>>8,tmp_addr&0xFF,2,&req->rx_buf[7+3]);
		if((tmp_addr >= 0x7B00)&&(tmp_addr < 0x7EFF)) // write EE in RAM
		{
			_Sys.FR.b1[tmp_addr - 0x7B00]=req->rx_buf[7+3];
			_Sys.FR.b1[tmp_addr - 0x7B00 + 1]=req->rx_buf[7+4];
		}
		//portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=0;
		portENABLE_INTERRUPTS();
	}
	len = 5;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)
	req->tx_buf[7]=0x06;
	req->tx_buf[7+1]=req->addr>>8;
	req->tx_buf[7+2]=req->addr&0xFF;
	req->tx_buf[7+3]=req->cnt>>8;
	req->tx_buf[7+4]=req->cnt&0xFF;
	return(7+len);
}

unsigned short tcpwrite_multi_regs(request* req)
{
	unsigned short tmp,byte_count,len;
	unsigned short tmp_addr;
	unsigned char err_cnt=0;
	if((req->cnt >= 129)||(req->cnt == 0)||((req->cnt == 128)&&(req->addr >= 0x8000))) return(get_error(req,0x03));
	if((req->addr+req->cnt>=129)&&(req->addr<0x8000)) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	if(req->addr<0x8000)
	{
		for(tmp=0;tmp<req->cnt;tmp++)
			{MEM_2(req->addr+tmp)=req->rx_buf[7+7+tmp*2] | ((unsigned short)req->rx_buf[7+6+tmp*2]<<8);}
	}
	else
	{
		tmp_addr = req->addr - 0x8000;
		tmp_addr *= 2;

		for(tmp=0;tmp<req->cnt;tmp++)
		{
			byte_count=req->rx_buf[7+6+tmp*2];
			req->rx_buf[7+6+tmp*2]=req->rx_buf[7+7+tmp*2];
			req->rx_buf[7+7+tmp*2]=byte_count;
			if((tmp_addr>=0x7B00)&&(tmp_addr<0x7EFF))
			{
				_Sys.FR.b1[tmp_addr -0x7B00 + tmp*2] = req->rx_buf[7+6+tmp*2];
				_Sys.FR.b1[tmp_addr -0x7B00 +tmp*2 + 1] = req->rx_buf[7+7+tmp*2];
			}
		}
		//while(_Sys_SPI_Buzy) vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);
		while(_Sys_SPI_Buzy) {vTaskDelayUntil(&ExLastExecutionTime,(portTickType)1/portTICK_RATE_MS);err_cnt++;if(err_cnt>=10) return 0;}

		portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=1;
		//portENABLE_INTERRUPTS();
		write_enable();
		write_data(tmp_addr>>8,tmp_addr&0xFF,req->cnt*2,&req->rx_buf[7+6]);
		//portDISABLE_INTERRUPTS();
		_Sys_SPI_Buzy=0;
		portENABLE_INTERRUPTS();
	}
	len = 5;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)
	req->tx_buf[7]=0x10;
	req->tx_buf[7+1]=req->rx_buf[7+1];
	req->tx_buf[7+2]=req->rx_buf[7+2];
	req->tx_buf[7+3]=req->rx_buf[7+3];
	req->tx_buf[7+4]=req->rx_buf[7+4];
	return(7+len);
}

unsigned short tcpwrite_multi_coils(request* req)
{
	unsigned short tmp,len;
	if((req->cnt>178)||(req->cnt==0)) return(get_error(req,0x03));
	if(req->addr + req->cnt > 178) return(get_error(req,0x02));

	req->tx_buf[0] = req->rx_buf[0];
	req->tx_buf[1] = req->rx_buf[1];							// transaction identifier
	req->tx_buf[2] = 0;req->tx_buf[3] = 0;						// protocol identifier
	req->tx_buf[6] = req->rx_buf[6];							// unit id

	for(tmp=0;tmp<req->cnt;tmp++)
	{
		if(req->addr+tmp<48)
		{
			if((req->rx_buf[7+6+(tmp>>3)])&(1<<(tmp%8))) _Sys_OUT[(req->addr+tmp)>>3]|=1<<((tmp+req->addr)%8);else _Sys_OUT[(req->addr+tmp)>>3] &= ~(1<<((tmp+req->addr)%8));
		}
		else
		{
			if((req->rx_buf[7+6+(tmp>>3)])&(1<<(tmp%8))) OUT[(req->addr+tmp-48)>>2]|=1<<((tmp+req->addr)%4);else OUT[(req->addr+tmp-48)>>2] &= ~(1<<((tmp+req->addr)%4));
		}
	}
	len = 5;
	req->tx_buf[4]=(len+1)>>8;req->tx_buf[5]=(len+1)&0xFF;		// length (unit_id + data)
	req->tx_buf[7]=0x0F;
	req->tx_buf[7+1]=req->rx_buf[7+1];
	req->tx_buf[7+2]=req->rx_buf[7+2];
	req->tx_buf[7+3]=req->rx_buf[7+3];
	req->tx_buf[7+4]=req->rx_buf[7+4];
	return(7+len);

}

