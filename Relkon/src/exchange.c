/*
 * exchange.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "exchange.h"
#include "hinout.h"
#include "main.h"
#include "crc.h"
#include "rk.h"

extern unsigned char tx_mod_buf[MOD_BUF_SIZE];

unsigned char plc[8]={0},err[8]={0};
extern plc_stat _Sys;

volatile unsigned char EXCHANGE=0,START_P1=0,START_P2=0,START_P3=0,START_P4=0;
volatile unsigned char START_P5=0,START_P6=0,START_P7=0,START_P8=0;

unsigned char TX_1[64],TX_2[64],TX_3[64],TX_4[64],TX_5[64],TX_6[64],TX_7[64],TX_8[64];
unsigned char RX_1[64],RX_2[64],RX_3[64],RX_4[64],RX_5[64],RX_6[64],RX_7[64],RX_8[64];

static unsigned short exch_tmr,exch_step;
static unsigned short temp;

void exchange_work(void)
{
	if(EXCHANGE)
	{
		exch_tmr++;
		if(exch_tmr>=50)
		{
			exch_tmr=0;
			switch(exch_step)
			{
				case 0x00:
				exch_step++;
				if(START_P1)
				{
					tx_mod_buf[0]=0x01;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_1[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC1++;if(((PLC1%4)==0)&&(ERR1<255)) ERR1++;break;
				}
				case 0x01:
				exch_step++;
				if(START_P2)
				{
					tx_mod_buf[0]=0x02;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_2[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC2++;if(((PLC2%4)==0)&&(ERR2<255)) ERR2++;break;
				}
				case 0x02:
				exch_step++;
				if(START_P3)
				{
					tx_mod_buf[0]=0x03;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_3[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC3++;if(((PLC3%4)==0)&&(ERR3<255)) ERR3++;break;
				}
				case 0x03:
				exch_step++;
				if(START_P4)
				{
					tx_mod_buf[0]=0x04;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_4[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC4++;if(((PLC4%4)==0)&&(ERR4<255)) ERR4++;break;
				}
				case 0x04:
				exch_step++;
				if(START_P5)
				{
					tx_mod_buf[0]=0x05;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_5[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC5++;if(((PLC5%4)==0)&&(ERR5<255)) ERR5++;break;
				}
				case 0x05:
				exch_step++;
				if(START_P6)
				{
					tx_mod_buf[0]=0x06;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_6[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC6++;if(((PLC6%4)==0)&&(ERR6<255)) ERR6++;break;
				}
				case 0x06:
				exch_step++;
				if(START_P7)
				{
					tx_mod_buf[0]=0x07;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_7[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC7++;if(((PLC7%4)==0)&&(ERR7<255)) ERR7++;break;
				}
				case 0x07:
				exch_step=0;
				if(START_P8)
				{
					tx_mod_buf[0]=0x08;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_8[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC8++;if(((PLC8%4)==0)&&(ERR8<255)) ERR8++;break;
				}
				exch_step++;
				if(START_P1)
				{
					tx_mod_buf[0]=0x01;tx_mod_buf[1]=0xE5;tx_mod_buf[2]=_Sys.Adr;
					for(temp=0;temp<64;temp++) tx_mod_buf[3+temp]=TX_1[temp];
					temp=GetCRC16(tx_mod_buf,67);tx_mod_buf[67]=temp>>8;tx_mod_buf[68]=temp&0xFF;
					write_module(69);PLC1++;if(((PLC1%4)==0)&&(ERR1<255)) ERR1++;
				}
				break;
			}

		}
	}
}

void exch_answer(request* req)
{
	unsigned char temp;
	switch(req->addr)
	{
		case 1:for(temp=0;temp<64;temp++) RX_1[temp]=req->rx_buf[3+temp];PLC1=0;ERR1=0;break;
		case 2:for(temp=0;temp<64;temp++) RX_2[temp]=req->rx_buf[3+temp];PLC2=0;ERR2=0;break;
		case 3:for(temp=0;temp<64;temp++) RX_3[temp]=req->rx_buf[3+temp];PLC3=0;ERR3=0;break;
		case 4:for(temp=0;temp<64;temp++) RX_4[temp]=req->rx_buf[3+temp];PLC4=0;ERR4=0;break;
		case 5:for(temp=0;temp<64;temp++) RX_5[temp]=req->rx_buf[3+temp];PLC5=0;ERR5=0;break;
		case 6:for(temp=0;temp<64;temp++) RX_6[temp]=req->rx_buf[3+temp];PLC6=0;ERR6=0;break;
		case 7:for(temp=0;temp<64;temp++) RX_7[temp]=req->rx_buf[3+temp];PLC7=0;ERR7=0;break;
		case 8:for(temp=0;temp<64;temp++) RX_8[temp]=req->rx_buf[3+temp];PLC8=0;ERR8=0;break;
	}
}

