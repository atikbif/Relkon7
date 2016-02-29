/*
 * inout.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "inout.h"
#include "hdac.h"
#include "crc.h"
#include "mmb.h"
#include "exchange.h"
#include "modbus.h"
#include "rk.h"
#include "main.h"
#include "FreeRTOS.h"

extern volatile unsigned char EXCHANGE;
extern const unsigned char mod_table[];
volatile unsigned short _Sys_DAC[4];
volatile unsigned short _Sys_ADC[8];
volatile unsigned char _Sys_IN[6];
volatile unsigned char _Sys_OUT[6];
unsigned char main_step=0;
extern volatile unsigned char emu_mode;
unsigned char tx_mod_buf[MOD_BUF_SIZE];
unsigned char rx_mod_buf[MOD_BUF_SIZE];
volatile unsigned char Tx_end=0;
volatile unsigned short rx_mod_cnt=0;
unsigned short tx_mod_cnt=0;
extern unsigned char   IN[32];
extern unsigned char   OUT[32];
extern unsigned char TX[64];
extern unsigned char RX[64];
extern unsigned char plc[8],err[8];
extern plc_stat _Sys;
volatile unsigned char prot_enable=1;
request req_mb;

portTickType MBxLastExecutionTime;

void InOutTask( void *pvParameters )
{

	unsigned short tmp;


	MBxLastExecutionTime = xTaskGetTickCount();
	dio_conf();
	init_mb_canal();

	req_mb.tx_buf = tx_mod_buf; req_mb.rx_buf = rx_mod_buf;req_mb.mode = BIN_MODE;req_mb.can_name = CAN_MB;

	for(;;)
	{
		set_dac(0,_Sys_DAC[0]);
		set_dac(1,_Sys_DAC[1]);
		switch(main_step)
		{
			case 0:
				if(emu_mode==0)
				{
					_Sys_IN[0]=read_din(0);
					_Sys_IN[1]=read_din(1);
					_Sys_IN[2]=read_din(2);
					_Sys_IN[3]=read_din(3);
				}
				if(emu_mode!=2)
				{
					dout_settings();
					write_dout(0,_Sys_OUT[0]);
					write_dout(1,_Sys_OUT[1]);
					write_dout(2,_Sys_OUT[2]);
					write_dout(3,_Sys_OUT[3]);
					din_settings();
				}
				break;
			case 1:
				if(emu_mode==0)
				{
					_Sys_IN[0]=read_din(0);
					_Sys_IN[4]=read_din(4);
					_Sys_IN[5]=read_din(5);
				}

				if(emu_mode!=2)
				{
					dout_settings();
					write_dout(0,_Sys_OUT[0]);
					write_dout(1,_Sys_OUT[1]);
					write_dout(2,_Sys_OUT[2]);
					write_dout(4,_Sys_OUT[4]);
					write_dout(5,_Sys_OUT[5]);
					din_settings();
				}
				break;
		}
		main_step++;
		if(main_step>1) main_step=0;
		if((mod_table[0])&&(emu_mode!=2)) {mmb_work();}
		else
		{
			exchange_work();
			if((rx_mod_cnt)&&(get_mmb_tmr() >= 15000)&&(prot_enable))
			{
				if(((rx_mod_buf[0]==_Sys.Adr)||(rx_mod_buf[0]==0))&&(GetCRC16(rx_mod_buf,rx_mod_cnt)==0))
				switch(rx_mod_buf[1])
				{
					case 0x01:
						req_mb.addr = ((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt = ((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_coils(&req_mb);write_module(tmp);break;
					case 0x02:
						req_mb.addr = ((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt = ((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_dinputs(&req_mb);write_module(tmp);break;
					case 0x03:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_holdregs(&req_mb);write_module(tmp);break;
					case 0x04:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp=read_inregs(&req_mb);write_module(tmp);break;
					case 0x05:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = write_single_coil(&req_mb);write_module(tmp);break;
					case 0x06:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = write_single_reg(&req_mb);write_module(tmp);break;
					case 0x10:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp=write_multi_regs(&req_mb);write_module(tmp);break;
					case 0x0F:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp=write_multi_coils(&req_mb);write_module(tmp);break;
					case 0xA0:tmp=get_software_ver(&req_mb);write_module(tmp);break;
					case 0xA1:tmp=get_hardware_ver(&req_mb);write_module(tmp);break;
					case 0xA2:tmp=get_can_name(&req_mb);write_module(tmp);break;
					case 0xB0:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_io(&req_mb);write_module(tmp);break;
					case 0xB1:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = write_io(&req_mb);write_module(tmp);break;
					case 0xD0:
						req_mb.addr = rx_mod_buf[2];req_mb.cnt = rx_mod_buf[3];
						tmp=read_mem(&req_mb);write_module(tmp);break;
					case 0xD1:
						req_mb.addr=rx_mod_buf[2];req_mb.cnt=rx_mod_buf[3];
						tmp=read_time(&req_mb);write_module(tmp);break;
					case 0xD3:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_frmem(&req_mb);write_module(tmp);break;
					case 0xD4:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_ram(&req_mb);write_module(tmp);break;
					case 0xD5:
						req_mb.laddr=((unsigned long)rx_mod_buf[2]<<24) | ((unsigned long)rx_mod_buf[3]<<16) | ((unsigned long)rx_mod_buf[4]<<8) |rx_mod_buf[5];
						req_mb.cnt=((unsigned int)rx_mod_buf[6]<<8) | rx_mod_buf[7];
						tmp = read_flash(&req_mb);write_module(tmp);break;
					case 0xD6:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp = read_preset(&req_mb);write_module(tmp);break;
					case 0xE0:
						req_mb.addr=rx_mod_buf[2];req_mb.cnt=rx_mod_buf[3];
						tmp = write_mem(&req_mb);write_module(tmp);break;
					case 0xE1:
						req_mb.addr=rx_mod_buf[2];req_mb.cnt=rx_mod_buf[3];
						tmp=write_time(&req_mb);write_module(tmp);break;
					case 0xE3:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp=write_frmem(&req_mb);write_module(tmp);break;
					case 0xE4:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp=write_ram(&req_mb);write_module(tmp);break;
					case 0xE5:
						if(EXCHANGE) {req_mb.addr=rx_mod_buf[2];exch_answer(&req_mb);}
						else{tmp=exchange_cmd(&req_mb);if(tmp) write_module(tmp);}
						break;
					case 0xE6:
						req_mb.addr=((unsigned short)rx_mod_buf[2]<<8) | rx_mod_buf[3];
						req_mb.cnt=((unsigned short)rx_mod_buf[4]<<8) | rx_mod_buf[5];
						tmp=write_preset(&req_mb);write_module(tmp);break;
					case 0xFE:reset_cmd();break;
				}
				rx_mod_cnt=0;
			}
		}
		vTaskDelayUntil( &MBxLastExecutionTime, InOut_DELAY );
	}
}
