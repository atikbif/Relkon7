/*
 * canal.c
 *
 *  Relkon ver 1.0
 *  Author: –оман
 *
 */

#include "FreeRTOS.h"
#include "canal.h"
#include "hcanal.h"
#include "crc.h"
#include "main.h"
#include "modbus.h"
#include "rk.h"
#include "hlcd.h"
#include "inout.h"
#include "string_func.h"

extern volatile unsigned char _Sys_SPI_Buzy;	// флаг зан€тости SPI другим процессом

// массив временных задержек дл€ RS485 на разных скорост€х
static const unsigned char _Sys_Delay[9]={45,20,10,6,4,2,1,1,1};

// приЄмные и передающие буферы
unsigned char canal_tx_buf[BUF_SIZE];
unsigned char canal_rx_buf[BUF_SIZE];
unsigned char canal2_tx_buf[BUF_SIZE];
unsigned char canal2_rx_buf[BUF_SIZE];
volatile unsigned char canalpu_tx_buf[512];
volatile unsigned char canalpu_rx_buf[512];

// счЄтчики прин€тых байт
volatile unsigned int canal_rx_cnt=0;
volatile unsigned int canal2_rx_cnt=0;
volatile unsigned short canalpu_rx_cnt=0,canalpu_tx_cnt=0;

volatile unsigned short pu_tmr=0;

volatile unsigned char ascii_type=_RELKON,ascii_type2=_RELKON;
// флаги разрешени€ стандартных протоколов в каналах
volatile unsigned char can_en=1,can2_en=1;
extern plc_stat _Sys;
unsigned char mstr1=0,mstr2=0;

// буферы дл€ модульного канала
extern unsigned char tx_mod_buf[MOD_BUF_SIZE];
extern unsigned char rx_mod_buf[MOD_BUF_SIZE];
extern volatile unsigned short rx_mod_cnt;
extern volatile unsigned char prot_enable;

// структуры с описанием параметров команды
request req_pc,req_pr,req_pu;

portTickType PCxLastExecutionTime;
portTickType PRxLastExecutionTime;
portTickType PUxLastExecutionTime;

/* ------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------ */

void can_disable(unsigned char can)		// запрет протокола в канале
{
	switch(can)
	{
		case 1:can2_en=0;break;
		case 2:can_en=0;break;
		case 3:prot_enable=0;break;
	}
}

void can_enable(unsigned char can)		// разрешение протокола в канале
{
	switch(can)
	{
		case 1:can2_en=1;break;
		case 2:can_en=1;break;
		case 3:prot_enable=1;break;
	}
}

unsigned char* get_can_rx_ptr(unsigned char num)	// возвращает указатель на приЄмный буфер канала
{
	unsigned char *ptr=0;
	switch(num)
	{
		case 1: ptr = canal2_rx_buf; break;
		case 2: ptr = canal_rx_buf; break;
		case 3: ptr = rx_mod_buf; break;
	}
	return ptr;
}

unsigned char* get_can_tx_ptr(unsigned char num)	// возвращает указатель на передающий буфер канала
{
	unsigned char *ptr=0;
	switch(num)
	{
		case 1: ptr = canal2_tx_buf; break;
		case 2: ptr = canal_tx_buf; break;
		case 3: ptr = tx_mod_buf; break;
	}
	return ptr;
}

void clear_rx_cnt(unsigned char num)	// сбрасывает счЄтчик прин€тых байт
{
	switch(num)
	{
		case 1: canal2_rx_cnt=0; break;
		case 2: canal_rx_cnt=0; break;
		case 3: rx_mod_cnt=0; break;
	}
}

unsigned short get_rx_cnt(unsigned char num)	// возвращает счЄтчик прин€тых байт
{
	switch(num)
	{
		case 1: return canal2_rx_cnt;
		case 2: return canal_rx_cnt;
		case 3: return rx_mod_cnt;
	}
	return 0;
}

// отсылка данных в канал
// ptr - указатель на буфер
// cnt - количество отсылаемых байт
void send(unsigned char can_num,unsigned char* ptr, unsigned short cnt)
{
    unsigned short tmp;
    unsigned char* tx_buf,*rx_buf;
    tx_buf = get_can_tx_ptr(can_num);
    rx_buf = get_can_rx_ptr(can_num);
    if(tx_buf==0) return;
    for(tmp=0;tmp<BUF_SIZE;tmp++) rx_buf[tmp]=0;
    for(tmp=0;tmp<cnt;tmp++) tx_buf[tmp]=ptr[tmp];
    switch(can_num)
    {
    	case 1:canal2_rx_cnt=0;write_canal2(cnt);break;
    	case 2:canal_rx_cnt=0;write_canal(cnt);break;
    	case 3:rx_mod_cnt=0;write_module(cnt);break;
    }
}

// поиск подстроки ptr в приЄмном буфере канала can_num
char search(unsigned char can_num,unsigned char* ptr)
{
	unsigned char* rx_buf;
    rx_buf=get_can_rx_ptr(can_num);
    if(rx_buf==0) return 0;
    if(find(ptr,rx_buf,BUF_SIZE)) return 1;
    return 0;
}

// пультовой канал в режиме коммуникационного
void PultCanTask( void *pvParameters )
{
	unsigned short tmp=0;
	init_lcd_canal();
	PUxLastExecutionTime = xTaskGetTickCount();
	req_pu.tx_buf = (unsigned char*)canalpu_tx_buf; req_pu.rx_buf = (unsigned char*)canalpu_rx_buf;
	req_pu.mode = BIN_MODE;req_pu.can_name = CAN_PU;
	pu_tmr=0;canalpu_rx_cnt=0;
	for( ;; )
	{
		// проверка наличи€ байт в буфере и таймаута тишины
		if((canalpu_rx_cnt)&&(pu_tmr>=3))
		{
			// проверка сетевого адреса контроллер и контрольной суммы
			if(((canalpu_rx_buf[0]==_Sys.Adr)||(canalpu_rx_buf[0]==0x00)||(canalpu_rx_buf[0]==0xFF))&&(GetCRC16((unsigned char*)canalpu_rx_buf,canalpu_rx_cnt)==0))
			{
				// разбор команд
				switch(canalpu_rx_buf[1])
				{
					case 0x01:
						req_pu.addr = ((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt = ((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_coils(&req_pu);write_lcd(tmp);break;
					case 0x02:
						req_pu.addr = ((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt = ((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_dinputs(&req_pu);write_lcd(tmp);break;
					case 0x03:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_holdregs(&req_pu);write_lcd(tmp);break;
					case 0x04:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp=read_inregs(&req_pu);write_lcd(tmp);break;
					case 0x05:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = write_single_coil(&req_pu);write_lcd(tmp);break;
					case 0x06:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = write_single_reg(&req_pu);write_lcd(tmp);break;
					case 0x10:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp=write_multi_regs(&req_pu);write_lcd(tmp);break;
					case 0x0F:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp=write_multi_coils(&req_pu);write_lcd(tmp);break;
					case 0xA0:tmp=get_software_ver(&req_pu);write_lcd(tmp);break;
					case 0xA1:tmp=get_hardware_ver(&req_pu);write_lcd(tmp);break;
					case 0xA2:
						if(canalpu_rx_buf[0]==0xFF) tmp=get_obj_name(&req_pu);
						else tmp=get_can_name(&req_pu);write_lcd(tmp);break;
					case 0xB0:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_io(&req_pu);write_lcd(tmp);break;
					case 0xB1:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = write_io(&req_pu);write_lcd(tmp);break;
					case 0xD0:
						req_pu.addr = canalpu_rx_buf[2];req_pu.cnt = canalpu_rx_buf[3];
						tmp=read_mem(&req_pu);write_lcd(tmp);break;
					case 0xD1:
						req_pu.addr=canalpu_rx_buf[2];req_pu.cnt=canalpu_rx_buf[3];
						tmp=read_time(&req_pu);write_lcd(tmp);break;
					case 0xD3:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_frmem(&req_pu);write_lcd(tmp);break;
					case 0xD4:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_ram(&req_pu);write_lcd(tmp);break;
					case 0xD5:
						req_pu.laddr=((unsigned long)canalpu_rx_buf[2]<<24) | ((unsigned long)canalpu_rx_buf[3]<<16) | ((unsigned long)canalpu_rx_buf[4]<<8) |canalpu_rx_buf[5];
						req_pu.cnt=((unsigned int)canalpu_rx_buf[6]<<8) | canalpu_rx_buf[7];
						tmp = read_flash(&req_pu);write_lcd(tmp);break;
					case 0xD6:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp = read_preset(&req_pu);write_lcd(tmp);break;
					case 0xE0:
						req_pu.addr=canalpu_rx_buf[2];req_pu.cnt=canalpu_rx_buf[3];
						tmp = write_mem(&req_pu);write_lcd(tmp);break;
					case 0xE1:
						req_pu.addr=canalpu_rx_buf[2];req_pu.cnt=canalpu_rx_buf[3];
						tmp=write_time(&req_pu);write_lcd(tmp);break;
					case 0xE3:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp=write_frmem(&req_pu);write_lcd(tmp);break;
					case 0xE4:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp=write_ram(&req_pu);write_lcd(tmp);break;
					case 0xE5:
						tmp=exchange_cmd(&req_pu);if(tmp) write_lcd(tmp);break;
					case 0xE6:
						req_pu.addr=((unsigned short)canalpu_rx_buf[2]<<8) | canalpu_rx_buf[3];
						req_pu.cnt=((unsigned short)canalpu_rx_buf[4]<<8) | canalpu_rx_buf[5];
						tmp=write_preset(&req_pu);write_lcd(tmp);break;
					case 0xFE:reset_cmd();break;
				}
			}
			canalpu_rx_cnt=0;
		}
		vTaskDelayUntil( &PUxLastExecutionTime, CANAL_DELAY );
	}
}

// бинарный протокол в канале "PC"
void BinCanTask( void *pvParameters )
{
	unsigned short tmp;
	init_pc_canal();
	pc_message_bin();	// сообщение приветстви€
	req_pc.tx_buf = canal_tx_buf; req_pc.rx_buf = canal_rx_buf;req_pc.mode = BIN_MODE;req_pc.can_name = CAN_PC;
	PCxLastExecutionTime = xTaskGetTickCount();
	for( ;; )
	{
		// проверка наличи€ байт в буфере, таймаута тишины и флага разрешени€ протокола
		if((canal_rx_cnt)&&(get_pc_tmr()>_Sys_Delay[_Sys.Can1_Baudrate])&&(can_en))
		{
			// проверка сетевого адреса и CRC
			if(((canal_rx_buf[0]==_Sys.Adr)||(canal_rx_buf[0]==0x00)||(canal_rx_buf[0]==0xFF))&&(GetCRC16(canal_rx_buf,canal_rx_cnt)==0))
			{
				// разбор команд
				switch(canal_rx_buf[1])
				{
					case 0x01:
						req_pc.addr = ((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt = ((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_coils(&req_pc);write_canal(tmp);break;
					case 0x02:
						req_pc.addr = ((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt = ((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_dinputs(&req_pc);write_canal(tmp);break;
					case 0x03:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_holdregs(&req_pc);write_canal(tmp);break;
					case 0x04:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp=read_inregs(&req_pc);write_canal(tmp);break;
					case 0x05:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = write_single_coil(&req_pc);write_canal(tmp);break;
					case 0x06:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = write_single_reg(&req_pc);write_canal(tmp);break;
					case 0x10:
						if(mstr1==0x10) break;
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp=write_multi_regs(&req_pc);write_canal(tmp);break;
					case 0x0F:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp=write_multi_coils(&req_pc);write_canal(tmp);break;
					case 0xA0:tmp=get_software_ver(&req_pc);write_canal(tmp);break;
					case 0xA1:tmp=get_hardware_ver(&req_pc);write_canal(tmp);break;
					case 0xA2:
						if(canal_rx_buf[0]==0xFF) tmp=get_obj_name(&req_pc);
						else tmp=get_can_name(&req_pc);write_canal(tmp);break;
					case 0xB0:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_io(&req_pc);write_canal(tmp);break;
					case 0xB1:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = write_io(&req_pc);write_canal(tmp);break;
					case 0xD0:
						req_pc.addr = canal_rx_buf[2];req_pc.cnt = canal_rx_buf[3];
						tmp=read_mem(&req_pc);write_canal(tmp);break;
					case 0xD1:
						req_pc.addr=canal_rx_buf[2];req_pc.cnt=canal_rx_buf[3];
						tmp=read_time(&req_pc);write_canal(tmp);break;
					case 0xD3:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_frmem(&req_pc);write_canal(tmp);break;
					case 0xD4:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_ram(&req_pc);write_canal(tmp);break;
					case 0xD5:
						req_pc.laddr=((unsigned long)canal_rx_buf[2]<<24) | ((unsigned long)canal_rx_buf[3]<<16) | ((unsigned long)canal_rx_buf[4]<<8) |canal_rx_buf[5];
						req_pc.cnt=((unsigned int)canal_rx_buf[6]<<8) | canal_rx_buf[7];
						tmp = read_flash(&req_pc);write_canal(tmp);break;
					case 0xD6:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp = read_preset(&req_pc);write_canal(tmp);break;
					case 0xE0:
						req_pc.addr=canal_rx_buf[2];req_pc.cnt=canal_rx_buf[3];
						tmp = write_mem(&req_pc);write_canal(tmp);break;
					case 0xE1:
						req_pc.addr=canal_rx_buf[2];req_pc.cnt=canal_rx_buf[3];
					  	tmp=write_time(&req_pc);write_canal(tmp);break;
					case 0xE3:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp=write_frmem(&req_pc);write_canal(tmp);break;
					case 0xE4:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp=write_ram(&req_pc);write_canal(tmp);break;
					case 0xE5:
						tmp=exchange_cmd(&req_pc);if(tmp) write_canal(tmp);break;
					case 0xE6:
						req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
						req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
						tmp=write_preset(&req_pc);write_canal(tmp);break;
					case 0xFE:reset_cmd();break;
				}
			}
			mstr1=0;
			canal_rx_cnt=0;
		}
		vTaskDelayUntil( &PCxLastExecutionTime, CANAL_DELAY );
	}
}

// ASCII протокол на канале "PC"
void AsciiCanTask( void *pvParameters )
{
	unsigned short tmp;
	unsigned char ascii_request=0;
	init_pc_canal();
	pc_message_ascii();
	req_pc.tx_buf = canal_tx_buf; req_pc.rx_buf = canal_rx_buf;req_pc.mode = ASCII_MODE;
	req_pc.can_name = CAN_PC;
	PCxLastExecutionTime = xTaskGetTickCount();
	for( ;; )
	{
		if((canal_rx_cnt>=7)&&(can_en))
		{
			if(ascii_type==_RELKON)
			{
				// проверка формата команды и преобразование к бинарному виду
				req_pc.cnt=canal_rx_cnt;tmp=check_ascii_rk(&req_pc);if(tmp) ascii_request=1;
			}
			else
			{
				// проверка формата команды и преобразование к бинарному виду
				req_pc.cnt=canal_rx_cnt;tmp=check_ascii_modbus(&req_pc);if(tmp) ascii_request=1;
			}
		}
		if(ascii_request)
		{
			ascii_request=0;
			// разбор команды
			switch(canal_rx_buf[1])
			{
				case 0x01:
					req_pc.addr = ((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt = ((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_coils(&req_pc);write_canal(tmp);break;
				case 0x02:
					req_pc.addr = ((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt = ((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_dinputs(&req_pc);write_canal(tmp);break;
				case 0x03:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_holdregs(&req_pc);write_canal(tmp);break;
				case 0x04:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp=read_inregs(&req_pc);write_canal(tmp);break;
				case 0x05:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = write_single_coil(&req_pc);write_canal(tmp);break;
				case 0x06:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = write_single_reg(&req_pc);write_canal(tmp);break;
				case 0x10:
					if(mstr1==0x10) break;
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp=write_multi_regs(&req_pc);write_canal(tmp);break;
				case 0x0F:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp=write_multi_coils(&req_pc);write_canal(tmp);break;
				case 0xA0:tmp=get_software_ver(&req_pc);write_canal(tmp);break;
				case 0xA1:tmp=get_hardware_ver(&req_pc);write_canal(tmp);break;
				case 0xA2:
					if(canal_rx_buf[0]==0xFF) tmp=get_obj_name(&req_pc);
					else tmp=get_can_name(&req_pc);write_canal(tmp);break;
				case 0xB0:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_io(&req_pc);write_canal(tmp);break;
				case 0xB1:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = write_io(&req_pc);write_canal(tmp);break;
				case 0xD0:
					req_pc.addr = canal_rx_buf[2];req_pc.cnt = canal_rx_buf[3];
					tmp=read_mem(&req_pc);write_canal(tmp);break;
				case 0xD1:
					req_pc.addr=canal_rx_buf[2];req_pc.cnt=canal_rx_buf[3];
					tmp=read_time(&req_pc);write_canal(tmp);break;
				case 0xD3:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_frmem(&req_pc);write_canal(tmp);break;
				case 0xD4:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_ram(&req_pc);write_canal(tmp);break;
				case 0xD5:
					req_pc.laddr=((unsigned long)canal_rx_buf[2]<<24) | ((unsigned long)canal_rx_buf[3]<<16) | ((unsigned long)canal_rx_buf[4]<<8) |canal_rx_buf[5];
					req_pc.cnt=((unsigned int)canal_rx_buf[6]<<8) | canal_rx_buf[7];
					tmp = read_flash(&req_pc);write_canal(tmp);break;
				case 0xD6:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp = read_preset(&req_pc);write_canal(tmp);break;
				case 0xE0:
					req_pc.addr=canal_rx_buf[2];req_pc.cnt=canal_rx_buf[3];
					tmp = write_mem(&req_pc);write_canal(tmp);break;
				case 0xE1:
					req_pc.addr=canal_rx_buf[2];req_pc.cnt=canal_rx_buf[3];
					tmp=write_time(&req_pc);write_canal(tmp);break;
				case 0xE3:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp=write_frmem(&req_pc);write_canal(tmp);break;
				case 0xE4:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp=write_ram(&req_pc);write_canal(tmp);break;
				case 0xE6:
					req_pc.addr=((unsigned short)canal_rx_buf[2]<<8) | canal_rx_buf[3];
					req_pc.cnt=((unsigned short)canal_rx_buf[4]<<8) | canal_rx_buf[5];
					tmp=write_preset(&req_pc);write_canal(tmp);break;
				case 0xFE:reset_cmd();break;
			}
			mstr1=0;
			canal_rx_cnt=0;
		}
	vTaskDelayUntil( &PCxLastExecutionTime, CANAL_DELAY );
	}
}

// бинарный протокол канала "PROG"
void BinCan2Task( void *pvParameters )
{
    unsigned short tmp;

    init_prog_canal();
    prog_message_bin();
    req_pr.tx_buf = canal2_tx_buf; req_pr.rx_buf = canal2_rx_buf;req_pr.mode = BIN_MODE;req_pr.can_name = CAN_PR;
    PRxLastExecutionTime = xTaskGetTickCount();
    _Sys_SPI_Buzy=0;
    for( ;; )
	{
        if((canal2_rx_cnt)&&(get_pr_tmr()>_Sys_Delay[_Sys.Can2_Baudrate])&&(can2_en))
        {
            if(((canal2_rx_buf[0]==_Sys.Adr)||(canal2_rx_buf[0]==0x00)||(canal2_rx_buf[0]==0xFF))&&(GetCRC16(canal2_rx_buf,canal2_rx_cnt)==0))
            {
                switch(canal2_rx_buf[1])
                {
					case 0x01:
						req_pr.addr = ((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt = ((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_coils(&req_pr);write_canal2(tmp);break;
					case 0x02:
						req_pr.addr = ((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt = ((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_dinputs(&req_pr);write_canal2(tmp);break;
					case 0x03:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_holdregs(&req_pr);write_canal2(tmp);break;
					case 0x04:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=read_inregs(&req_pr);write_canal2(tmp);break;
					case 0x05:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = write_single_coil(&req_pr);write_canal2(tmp);break;
					case 0x06:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = write_single_reg(&req_pr);write_canal2(tmp);break;
					case 0x10:
						if(mstr2==0x10) break;
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_multi_regs(&req_pr);write_canal2(tmp);break;
					case 0x0F:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_multi_coils(&req_pr);write_canal2(tmp);break;
					case 0xA0:tmp=get_software_ver(&req_pr);write_canal2(tmp);break;
					case 0xA1:tmp=get_hardware_ver(&req_pr);write_canal2(tmp);break;
					case 0xA2:
						if(canal2_rx_buf[0]==0xFF) tmp=get_obj_name(&req_pr);
						else tmp=get_can_name(&req_pr);write_canal2(tmp);break;
					case 0xB0:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_io(&req_pr);write_canal2(tmp);break;
					case 0xB1:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = write_io(&req_pr);write_canal2(tmp);break;
					case 0xD0:
						req_pr.addr = canal2_rx_buf[2];req_pr.cnt = canal2_rx_buf[3];
						tmp=read_mem(&req_pr);write_canal2(tmp);break;
					case 0xD1:
						req_pr.addr=canal2_rx_buf[2];req_pr.cnt=canal2_rx_buf[3];
						tmp=read_time(&req_pr);write_canal2(tmp);break;
					case 0xD3:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_frmem(&req_pr);write_canal2(tmp);break;
					case 0xD4:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_ram(&req_pr);write_canal2(tmp);break;
					case 0xD5:
						req_pr.laddr=((unsigned long)canal2_rx_buf[2]<<24) | ((unsigned long)canal2_rx_buf[3]<<16) | ((unsigned long)canal2_rx_buf[4]<<8) |canal2_rx_buf[5];
						req_pr.cnt=((unsigned int)canal2_rx_buf[6]<<8) | canal2_rx_buf[7];
						tmp = read_flash(&req_pr);write_canal2(tmp);break;
					case 0xD6:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_preset(&req_pr);write_canal2(tmp);break;
					case 0xE0:
						req_pr.addr=canal2_rx_buf[2];req_pr.cnt=canal2_rx_buf[3];
						tmp = write_mem(&req_pr);write_canal2(tmp);break;
					case 0xE1:
						req_pr.addr=canal2_rx_buf[2];req_pr.cnt=canal2_rx_buf[3];
						tmp=write_time(&req_pr);write_canal2(tmp);break;
					case 0xE3:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_frmem(&req_pr);write_canal2(tmp);break;
					case 0xE4:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_ram(&req_pr);write_canal2(tmp);break;
					case 0xE5:
						tmp=exchange_cmd(&req_pr);if(tmp) write_canal2(tmp);break;
					case 0xE6:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_preset(&req_pr);write_canal2(tmp);break;
					case 0xFE:reset_cmd();break;
                }

            }
            canal2_rx_cnt=0;
            mstr2=0;
        }
        vTaskDelayUntil( &PRxLastExecutionTime, CANAL_DELAY );
    }
}

// ASCII протокол канала "PROG"
void AsciiCan2Task( void *pvParameters )
{
    unsigned short tmp;
    unsigned char ascii_request=0;

    init_prog_canal();
    prog_message_ascii();

	_Sys_SPI_Buzy=0;
	req_pr.tx_buf = canal2_tx_buf; req_pr.rx_buf = canal2_rx_buf;req_pr.mode = ASCII_MODE;req_pr.can_name = CAN_PR;
	PRxLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{

        if((canal2_rx_cnt>=7)&&(can2_en))
        {
        	if(ascii_type2==_RELKON)
			{
        		// проверка формата команды и преобразование к бинарному виду
				req_pr.cnt=canal2_rx_cnt;tmp=check_ascii_rk(&req_pr);if(tmp) ascii_request=1;
			}
			else
			{
				// проверка формата команды и преобразование к бинарному виду
				req_pr.cnt=canal2_rx_cnt;tmp=check_ascii_modbus(&req_pr);if(tmp) ascii_request=1;
			}
        }
            if(ascii_request)
            {
                ascii_request=0;
                // разбор команды
                switch(canal2_rx_buf[1])
                {
					case 0x01:
						req_pr.addr = ((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt = ((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_coils(&req_pr);write_canal2(tmp);break;
					case 0x02:
						req_pr.addr = ((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt = ((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_dinputs(&req_pr);write_canal2(tmp);break;
					case 0x03:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_holdregs(&req_pr);write_canal2(tmp);break;
					case 0x04:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=read_inregs(&req_pr);write_canal2(tmp);break;
					case 0x05:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = write_single_coil(&req_pr);write_canal2(tmp);break;
					case 0x06:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = write_single_reg(&req_pr);write_canal2(tmp);break;
					case 0x10:
						if(mstr2==0x10) break;
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_multi_regs(&req_pr);write_canal2(tmp);break;
					case 0x0F:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_multi_coils(&req_pr);write_canal2(tmp);break;
					case 0xA0:tmp=get_software_ver(&req_pr);write_canal2(tmp);break;
					case 0xA1:tmp=get_hardware_ver(&req_pr);write_canal2(tmp);break;
					case 0xA2:
						if(canal2_rx_buf[2]==0xFF) tmp=get_obj_name(&req_pr);
						else tmp=get_can_name(&req_pr);write_canal2(tmp);break;
					case 0xB0:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_io(&req_pr);write_canal2(tmp);break;
					case 0xB1:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = write_io(&req_pr);write_canal2(tmp);break;
					case 0xD0:
						req_pr.addr = canal2_rx_buf[2];req_pr.cnt = canal2_rx_buf[3];
						tmp=read_mem(&req_pr);write_canal2(tmp);break;
					case 0xD1:
						req_pr.addr=canal2_rx_buf[2];req_pr.cnt=canal2_rx_buf[3];
						tmp=read_time(&req_pr);write_canal2(tmp);break;
					case 0xD3:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_frmem(&req_pr);write_canal2(tmp);break;
					case 0xD4:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_ram(&req_pr);write_canal2(tmp);break;
					case 0xD5:
						req_pr.laddr=((unsigned long)canal2_rx_buf[2]<<24) | ((unsigned long)canal2_rx_buf[3]<<16) | ((unsigned long)canal2_rx_buf[4]<<8) |canal2_rx_buf[5];
						req_pr.cnt=((unsigned int)canal2_rx_buf[6]<<8) | canal2_rx_buf[7];
						tmp = read_flash(&req_pr);write_canal2(tmp);break;
					case 0xD6:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp = read_preset(&req_pr);write_canal2(tmp);break;
					case 0xE0:
						req_pr.addr=canal2_rx_buf[2];req_pr.cnt=canal2_rx_buf[3];
						tmp = write_mem(&req_pr);write_canal2(tmp);break;
					case 0xE1:
						req_pr.addr=canal2_rx_buf[2];req_pr.cnt=canal2_rx_buf[3];
						tmp=write_time(&req_pr);write_canal2(tmp);break;
					case 0xE3:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_frmem(&req_pr);write_canal2(tmp);break;
					case 0xE4:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_ram(&req_pr);write_canal2(tmp);break;
					case 0xE6:
						req_pr.addr=((unsigned short)canal2_rx_buf[2]<<8) | canal2_rx_buf[3];
						req_pr.cnt=((unsigned short)canal2_rx_buf[4]<<8) | canal2_rx_buf[5];
						tmp=write_preset(&req_pr);write_canal2(tmp);break;
					case 0xFE:reset_cmd();break;
                }
                mstr2=0;
                canal2_rx_cnt=0;
            }
        vTaskDelayUntil( &PRxLastExecutionTime, CANAL_DELAY );
    }
}


