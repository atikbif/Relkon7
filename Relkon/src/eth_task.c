/*
 * eth_task.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "eth_task.h"
#include "k_main.h"
#include "k_arp.h"
#include "k_udp.h"
#include "k_ip.h"
#include "k_http.h"
#include "http_data.h"

#include "hcanal.h"
#include "canal.h"
#include "rk.h"
#include "modbustcp.h"
#include "main.h"
#include "crc.h"
#include "FreeRTOS.h"

unsigned char eth_test;
unsigned char eth_stat=0;
unsigned short eth_pkt_cnt=0;
unsigned char s_ip[4];

volatile unsigned short testConnect = 0;
volatile unsigned short ethPktCnt = 0;
volatile unsigned short ethWDGTmr = 0;

request req_udp,req_tcp;

extern plc_stat _Sys;

portTickType ExLastExecutionTime;

void EthTask( void *pvParameters )
{
	GPIO_ENETConfiguration();
	ExLastExecutionTime = xTaskGetTickCount();
	for(;;)
	{
		// состояние сети ethernet
		switch(eth_stat)
		{
			case 0:
				// стартовая настройка
				if(ENET_Configuration())	// конфигурация mac уровня
				{
					//s_ip[0]=192;s_ip[1]=168;s_ip[2]=1;s_ip[3]=15; // настройка ip адреса контроллера
					get_ip(s_ip);s_ip[3]=1;set_gate(s_ip); // настройка ip адреса шлюза
					init_arp_tab();	// инициализация arp таблицы
					// инициализация протоколов udp,tcp/ip,http
					udp_init();
					tcp_init();
					udp_listen(12144,udp_protocol);
					tcp_listen(502,tcp_modbus);
					//http_enable();web_file_init();
					eth_stat++;
				}
            	vTaskDelayUntil( &ExLastExecutionTime, ( portTickType ) 1000 / portTICK_RATE_MS );
            	break;
			case 1:
				vTaskSuspendAll();
				konlite_call();	// сканирование принятых данных
				test_rx_overflow();
				xTaskResumeAll();
				break;
		}
		testConnect++;
		if(testConnect>=1000) {
			testConnect = 0;
			if(get_link_status()==0) eth_stat=0;	// проверка связи с mac уровнем
			ethWDGTmr++;
			if(ethWDGTmr>=600) {
			  ethWDGTmr=0;
			  if(ethPktCnt==0) eth_stat=0;
			  ethPktCnt=0;
			}
		}
		vTaskDelayUntil( &ExLastExecutionTime, ETH_DELAY );
	}
}

// запрос по протоколу modbus tcp
// pkt1 и pkt2 - структуры, описывающие принятые заголовки tcp и ip
void tcp_modbus(tcp_pkt* pkt1,ip_pkt* pkt2)
{
	ethPktCnt++;
	tcp_answer_head(pkt1,pkt2);	// преобразование заголовков tcp и ip на случай вероятного ответа
	req_tcp.tx_buf = pkt1->buf.ptr; req_tcp.rx_buf = pkt1->buf.ptr;
	req_tcp.can_name = CAN_TCP;
	pkt1->buf.len=0;
	// разбор команды
	switch(pkt1->buf.ptr[7])
	{
		case 0x01:
			req_tcp.addr = ((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt = ((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpread_coils(&req_tcp);break;
		case 0x02:
			req_tcp.addr = ((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt = ((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpread_dinputs(&req_tcp);break;
		case 0x03:
			req_tcp.addr=((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt=((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpread_holdregs(&req_tcp);break;
		case 0x04:
			req_tcp.addr=((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt=((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpread_inregs(&req_tcp);break;
		case 0x05:
			req_tcp.addr=((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt=((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpwrite_single_coil(&req_tcp);break;
		case 0x06:
			req_tcp.addr=((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt=((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpwrite_single_reg(&req_tcp);break;
		case 0x10:
			req_tcp.addr=((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt=((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpwrite_multi_regs(&req_tcp);break;
		case 0x0F:
			req_tcp.addr=((unsigned short)pkt1->buf.ptr[7+1]<<8) | pkt1->buf.ptr[7+2];
			req_tcp.cnt=((unsigned short)pkt1->buf.ptr[7+3]<<8) | pkt1->buf.ptr[7+4];
			pkt1->buf.len = tcpwrite_multi_coils(&req_tcp);break;
	}
	if(pkt1->buf.len) send_tcp(pkt1,pkt2);
}

// протокол RKBIN поверх udp
// pkt1 и pkt2 - структуры, описывающие принятые заголовки udp и ip
void udp_protocol(udp_pkt* pkt1,ip_pkt* pkt2)
{
	ethPktCnt++;
	udp_answer_head(pkt1,pkt2);	// преобразование заголовков на случай вероятного ответа
	req_udp.tx_buf = pkt1->buf.ptr; req_udp.rx_buf = pkt1->buf.ptr;
	req_udp.mode = BIN_MODE;req_udp.can_name = CAN_UDP;
	// проверка сетевого адреса и CRC
	//if(((pkt1->buf.ptr[0]==_Sys.Adr)||(pkt1->buf.ptr[0]==0x00))&&(GetCRC16(pkt1->buf.ptr,pkt1->buf.len)==0))
	{
		pkt1->buf.len=0;
		// разбор команды
		switch(pkt1->buf.ptr[1])
		{
			case 0xA0:pkt1->buf.len=get_software_ver(&req_udp);break;
			case 0xA1:pkt1->buf.len=get_hardware_ver(&req_udp);break;
			case 0xA2:pkt1->buf.len=get_can_name(&req_udp);break;
			case 0xB0:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len = read_io(&req_udp);break;
			case 0xB1:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len = write_io(&req_udp);break;
			case 0xD0:
				req_udp.addr = pkt1->buf.ptr[2];req_udp.cnt = pkt1->buf.ptr[3];
				pkt1->buf.len=read_mem(&req_udp);break;
			case 0xD1:
				req_udp.addr=pkt1->buf.ptr[2];req_udp.cnt=pkt1->buf.ptr[3];
				pkt1->buf.len=read_time(&req_udp);break;
			case 0xD3:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len = read_frmem(&req_udp);break;
			case 0xD4:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len = read_ram(&req_udp);break;
			case 0xD5:
				req_udp.laddr=((unsigned long)pkt1->buf.ptr[2]<<24) | ((unsigned long)pkt1->buf.ptr[3]<<16) | ((unsigned long)pkt1->buf.ptr[4]<<8) |pkt1->buf.ptr[5];
				req_udp.cnt=((unsigned int)pkt1->buf.ptr[6]<<8) | pkt1->buf.ptr[7];
				pkt1->buf.len = read_flash(&req_udp);break;
			case 0xD6:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len = read_preset(&req_udp);break;
			case 0xE0:
				req_udp.addr=pkt1->buf.ptr[2];req_udp.cnt=pkt1->buf.ptr[3];
				pkt1->buf.len = write_mem(&req_udp);break;
			case 0xE1:
				req_udp.addr=pkt1->buf.ptr[2];req_udp.cnt=pkt1->buf.ptr[3];
				pkt1->buf.len=write_time(&req_udp);break;
			case 0xE3:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len=write_frmem(&req_udp);break;
			case 0xE4:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len=write_ram(&req_udp);break;
			case 0xE6:
				req_udp.addr=((unsigned short)pkt1->buf.ptr[2]<<8) | pkt1->buf.ptr[3];
				req_udp.cnt=((unsigned short)pkt1->buf.ptr[4]<<8) | pkt1->buf.ptr[5];
				pkt1->buf.len=write_preset(&req_udp);break;
			case 0xFE:reset_cmd();break;
		}
		if(pkt1->buf.len) {send_udp(pkt1,pkt2);}
	}
}


