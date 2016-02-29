/*
 * wifi.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "wifi.h"

#include "hci.h"
#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "socket.h"
#include "cc3000_common.h"
#include "netapp.h"

#include "canal.h"
#include "hcanal.h"
#include "crc.h"
#include "rk.h"
#include "modbus.h"

extern volatile unsigned char fl_rd;
request req_wifi;

unsigned char ssid_name[32];
unsigned char wifi_password[32];
unsigned char wifi_type;
unsigned char wifi_ip[4];

unsigned char rst_fl=0;
unsigned short rst_tmr=0;

//extern unsigned char canal2_tx_buf[BUF_SIZE];

unsigned short wf_cnt=0;
volatile unsigned char wifi_stat=0;
unsigned char wifi_rx[CC_BUFFER_SIZE+100],wifi_tx[CC_BUFFER_SIZE+100];

portTickType WFxLastExecutionTime,SCANWFxLastExecutionTime;
tSpiInformation sSpiInformation;

uint8_t tSpiReadHeader[] = {READ,0,0,0,0,0,0,0,0,0};
uint8_t wlan_tx_buffer[CC_BUFFER_SIZE];
uint8_t wlan_rx_buffer[CC_BUFFER_SIZE];

unsigned long rx_skt,tx_skt;
sockaddr tSocketAddr;

unsigned long ulSmartConfigFinished=0,ulSmartConnected=0;

unsigned char pucIP_Addr[4];
unsigned char pucIP_DefaultGWAddr[4];
unsigned char pucSubnetMask[4];
unsigned char pucDNS[4];

fd_set socks;
int iReadSocks;
struct timeval timeout;
signed short sRxMsgLen;
sockaddr tRxSocketAddr;
socklen_t tRxPacketLength;

unsigned char wifi_debug=0;


const unsigned char *sendDriverPatch(unsigned long *Length);
const unsigned char *sendBootLoaderPatch(unsigned long *Length);
const unsigned char *sendWLFWPatch(unsigned long *Length);

// функция обработки асинхронных событий
void CC3000_UsynchCallback(long lEventType, char *pcData, unsigned char ucLength)
{
	if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE) {ulSmartConfigFinished=1;}
	if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT) {ulSmartConnected=1; wifi_stat=3;}
	if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT) {ulSmartConnected=0;wifi_stat=2;}
	if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
	{
		//print_num(1,pcData[3]);
		//print_num(2,pcData[2]);
		//print_num(3,pcData[1]);
		//print_num(4,pcData[0]);
	}
}

// функции загрузки патчей (не применяются)
const unsigned char *sendDriverPatch(unsigned long *Length)
{
	*Length = 0;
	return NULL;
}

const unsigned char  *sendBootLoaderPatch(unsigned long *Length)
{
	*Length = 0;
	return NULL;
}

const unsigned char *sendWLFWPatch(unsigned long *Length)
{
	*Length = 0;
	return NULL;
}

// основная задача
void WIFITask( void *pvParameters )
{
	unsigned short tmr=0,tmp;
	// маска, IP, шлюз
	pucSubnetMask[0] = 0xFF;pucSubnetMask[1] = 0xFF;pucSubnetMask[2] = 0xFF;pucSubnetMask[3] = 0x0;
	pucIP_Addr[0] = wifi_ip[0];pucIP_Addr[1] = wifi_ip[1];pucIP_Addr[2] = wifi_ip[2];pucIP_Addr[3] = wifi_ip[3];
	pucIP_DefaultGWAddr[0] = wifi_ip[0];pucIP_DefaultGWAddr[1] = wifi_ip[1];pucIP_DefaultGWAddr[2] = wifi_ip[2];pucIP_DefaultGWAddr[3] = 1;
	pucDNS[0] = 0;pucDNS[1] = 0;pucDNS[2] = 0;pucDNS[3] = 0;

	// структура для обратки команд протокола
	req_wifi.tx_buf = (unsigned char*)wifi_tx; req_wifi.rx_buf = (unsigned char*)wifi_rx;
	req_wifi.mode = BIN_MODE;req_wifi.can_name = CAN_WF;

	cc_gpio_init();
	init_cctimer();
	WFxLastExecutionTime = xTaskGetTickCount();
	// задержка после включения
	vTaskDelayUntil( &WFxLastExecutionTime, 1000*WF_DELAY );
	// инициализация CC3000
	wlan_init( CC3000_UsynchCallback, (tFWPatches )sendWLFWPatch, (tDriverPatches )sendDriverPatch, (tBootLoaderPatches)sendBootLoaderPatch, ReadWlanInterruptPin, WlanInterruptEnable, WlanInterruptDisable, WriteWlanPin);
	wlan_start(0);
	// удаление сохранённых профилей
	wlan_ioctl_set_connection_policy(0,0,0);
	wlan_ioctl_del_profile(0);
	wlan_ioctl_del_profile(1);
	wlan_ioctl_del_profile(2);
	// определение длины имени точки доступа
	for(tmp=0;tmp<32;tmp++) {if(ssid_name[tmp]==0) break;}
	// формирование профиля
	wlan_add_profile (WLAN_SEC_UNSEC,ssid_name, tmp, NULL, 0,0,0,0, NULL, 0);
	//wlan_add_profile (WLAN_SEC_UNSEC,(unsigned char*)"DemoAP", 6, NULL, 0,0,0,0, NULL, 0);
	//wlan_add_profile(WLAN_SEC_WPA2,(unsigned char*)"kontel2", 7,NULL, 0,0,0,0, (unsigned char*)"dTKC7yx82",9);
	// задание модели поведения для сс3000
	wlan_ioctl_set_connection_policy(0,0,1);
	// передача ip, маски и шлюза в сс3000
	netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);
	// перезапуск СС3000
	wlan_stop();
	vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY*100 );
	wlan_start(0);
	// маскирование ненужных для обработки событий
	wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_UNSOL_INIT|HCI_EVNT_WLAN_ASYNC_PING_REPORT|HCI_EVNT_WLAN_UNSOL_DHCP|HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE );
	//netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);
	wifi_stat=1;	// init ok
	// ожидание подключения к точке доступа
	while(wifi_stat!=3) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
	//netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);

	// создание сокетов для приёма и передачи
	rx_skt = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	tx_skt = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	// сканирование порта 12144 с любого ip адреса
	tSocketAddr.sa_family = AF_INET;
	tSocketAddr.sa_data[0] = 12144 >> 8;
	tSocketAddr.sa_data[1] = 12144 & 0xFF;
	tSocketAddr.sa_data[2]=0;tSocketAddr.sa_data[3]=0;
	tSocketAddr.sa_data[4]=0;tSocketAddr.sa_data[5]=0;
	bind(rx_skt,&tSocketAddr, sizeof(sockaddr));
	rst_fl=0;
    for(;;)
    {
    	if(rst_fl)	// необходим перезапуск СС3000
    	{
    		rst_fl=0;wifi_stat=0;
    		wlan_stop();
			vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY*1000 );
			wlan_start(0);
			wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_UNSOL_INIT|HCI_EVNT_WLAN_ASYNC_PING_REPORT|HCI_EVNT_WLAN_UNSOL_DHCP|HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE );
			wifi_stat=1;
			// ожидание подключения к ранее настроенному профилю
			while(wifi_stat!=3) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
			tmr=400;
    	}
    	if(((wifi_stat==2)||(tmr>=400))&&(sSpiInformation.ulSpiState == eSPI_STATE_IDLE))
    	{
    		// переоткрытие сокетов в случае потери связи или по таймауту
    		wifi_debug++;
    		tmr=0;
    		closesocket(rx_skt);
    		closesocket(tx_skt);
			while(wifi_stat!=3) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
			netapp_dhcp((unsigned long *)pucIP_Addr, (unsigned long *)pucSubnetMask, (unsigned long *)pucIP_DefaultGWAddr, (unsigned long *)pucDNS);
			rx_skt = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
			tx_skt = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
			tSocketAddr.sa_family = AF_INET;
			tSocketAddr.sa_data[0] = 12144 >> 8;
			tSocketAddr.sa_data[1] = 12144 & 0xFF;
			tSocketAddr.sa_data[2]=0;tSocketAddr.sa_data[3]=0;
			tSocketAddr.sa_data[4]=0;tSocketAddr.sa_data[5]=0;
			bind(rx_skt,&tSocketAddr, sizeof(sockaddr));
			//bind(tx_skt,&tSocketAddr, sizeof(sockaddr));
    	}
    	tmr++;
    	// сканирование входящих данных
    	FD_ZERO(&socks);
		FD_SET(rx_skt, &socks);
		timeout.tv_sec = 0;
		timeout.tv_usec = 100;
    	iReadSocks = select(2, &socks, 0, 0, &timeout);
		if (iReadSocks > 0)
		{
			sRxMsgLen = recvfrom(rx_skt, wifi_rx, CC_BUFFER_SIZE, 0, &tSocketAddr, &tRxPacketLength);
			if (sRxMsgLen > 0)
			{
				//for(tmp=0;tmp<sRxMsgLen;tmp++) canal2_tx_buf[tmp]=wifi_rx[tmp];
				//write_canal2(sRxMsgLen);
				//tmr=0;
				if(sRxMsgLen<20)	// удаление нулей дополнения при коротких запросах
				{
					tmp=0;while(wifi_rx[sRxMsgLen-1-tmp]==0) {tmp++;if(tmp==sRxMsgLen) break;}
					sRxMsgLen -= tmp;
				}
				if (sRxMsgLen)
				{
					//if(GetCRC16(wifi_rx,sRxMsgLen+1)==0) //sRxMsgLen++;	// if crc end byte zero
					{

						tmp=0;
						// обработка команды
						switch(wifi_rx[1])
						{
							case 0xA0:tmp=get_software_ver(&req_wifi);break;
							case 0xA1:tmp=get_hardware_ver(&req_wifi);break;
							case 0xA2:
								if(wifi_rx[0]==0xFF) tmp=get_obj_name(&req_wifi);
								else tmp=get_can_name(&req_wifi);break;
							case 0xB0:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp = read_io(&req_wifi);break;
							case 0xB1:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp = write_io(&req_wifi);break;
							case 0xD0:
								req_wifi.addr = wifi_rx[2];req_wifi.cnt = wifi_rx[3];
								tmp=read_mem(&req_wifi);break;
							case 0xD1:
								req_wifi.addr=wifi_rx[2];req_wifi.cnt=wifi_rx[3];
								tmp=read_time(&req_wifi);break;
							case 0xD3:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp = read_frmem(&req_wifi);break;
							case 0xD4:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp = read_ram(&req_wifi);break;
							case 0xD5:
								req_wifi.laddr=((unsigned long)wifi_rx[2]<<24) | ((unsigned long)wifi_rx[3]<<16) | ((unsigned long)wifi_rx[4]<<8) |wifi_rx[5];
								req_wifi.cnt=((unsigned int)wifi_rx[6]<<8) | wifi_rx[7];
								tmp = read_flash(&req_wifi);break;
							case 0xD6:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp = read_preset(&req_wifi);break;
							case 0xE0:
								req_wifi.addr=wifi_rx[2];req_wifi.cnt=wifi_rx[3];
								tmp = write_mem(&req_wifi);break;
							case 0xE1:
								req_wifi.addr=wifi_rx[2];req_wifi.cnt=wifi_rx[3];
								tmp=write_time(&req_wifi);break;
							case 0xE3:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp=write_frmem(&req_wifi);break;
							case 0xE4:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp=write_ram(&req_wifi);break;
							case 0xE6:
								req_wifi.addr=((unsigned short)wifi_rx[2]<<8) | wifi_rx[3];
								req_wifi.cnt=((unsigned short)wifi_rx[4]<<8) | wifi_rx[5];
								tmp=write_preset(&req_wifi);break;
						}
						if(tmp)
						{
							//tSocketAddr.sa_data[0] = 0;//12145 >> 8;
							//tSocketAddr.sa_data[1] = 0;//12145 & 0xFF;
							// отправка ответа
							sendto(tx_skt, wifi_tx, tmp, 0, &tSocketAddr, sizeof(sockaddr));
							rst_tmr=0;	// обнуление таймера сброса
						}
					}
				}
			}
		}
		vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
    }
}

void SCAN_WIFITask( void *pvParameters )
{

	SCANWFxLastExecutionTime = xTaskGetTickCount();
	for(;;)
	{
		// проверка поступления новых событий или данных
		if((fl_rd)&&(!ReadWlanInterruptPin()))
		{
			if(sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
			{
				sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;
				fl_rd=0;
			}
		}
		wf_cnt++;
		if(wf_cnt==1000)
		{
			wf_cnt=0;hci_unsolicited_event_handler();
			rst_tmr++; // инкремент таймера сброса (обнуляется при поступлении запросов)
			if(rst_tmr >= 60*3){rst_tmr=0;rst_fl=1;}
		}
		// чтение событий или данных при их наличии
		scan_wifi(SCANWFxLastExecutionTime);
		vTaskDelayUntil( &SCANWFxLastExecutionTime, WF_DELAY );
	}
}


// отправка данных по SPI в сс3000
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
	unsigned char ucPad = 0;
	wait_spi();
	//pause_us(100);
	if(!(usLength & 0x0001)) {ucPad++;}
	pUserBuffer[0] = WRITE;
	pUserBuffer[1] = HI(usLength + ucPad);
	pUserBuffer[2] = LO(usLength + ucPad);
	pUserBuffer[3] = 0;
	pUserBuffer[4] = 0;
	usLength += (sizeof(btspi_hdr) + ucPad);

	if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
	{
		// ожидание инициализации
		while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
	}
	if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
	{
		// первый вызов отличается от последующих
		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		while(ReadWlanInterruptPin()) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
		ASSERT_CS();pause_us(50);
		SpiSend(pUserBuffer,0,4,WFxLastExecutionTime);
		pause_us(50);
		SpiSend(pUserBuffer+4,0,usLength-4,WFxLastExecutionTime);
		DEASSERT_CS();
		while(!ReadWlanInterruptPin()) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
		sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	}else
	{
		if(sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
		{
			while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE) {vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );}
			//vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY);
		}
		sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
		ASSERT_CS();
		//pause_us(50);
		//while(ReadWlanInterruptPin()) vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
		SpiSend(pUserBuffer,0,usLength,WFxLastExecutionTime);
		vTaskDelayUntil( &WFxLastExecutionTime, WF_DELAY );
		DEASSERT_CS();
		sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
	}
	return 0;
}

// чтение событий или данных
void scan_wifi(portTickType ptr)
{
	hci_hdr_t *hci_hdr;
	unsigned long data_to_recv=0;
	unsigned char *evnt_buff;
	hci_evnt_hdr_t *hci_evnt_hdr;
	hci_data_hdr_t *pDataHdr;
	if(sSpiInformation.ulSpiState == eSPI_STATE_READ_IRQ)
	{
		//vTaskDelayUntil( &ptr, WF_DELAY );
		ASSERT_CS();
		//pause_us(50);
		//vTaskDelayUntil( &ptr, WF_DELAY );
		evnt_buff =  sSpiInformation.pRxPacket;
		hci_hdr = (hci_hdr_t *)(evnt_buff + sizeof(btspi_hdr));
		SpiSend(tSpiReadHeader,sSpiInformation.pRxPacket,10,ptr);
		switch(hci_hdr->ucType)
		{
			case HCI_TYPE_DATA:
				pDataHdr = (hci_data_hdr_t *)(evnt_buff + sizeof(btspi_hdr));
				data_to_recv = pDataHdr->usLength;
				if (!((10 + data_to_recv) & 1))	{data_to_recv++;}
				if (data_to_recv) SpiSend(0, evnt_buff + 10, data_to_recv,ptr);
				break;
			case HCI_TYPE_EVNT:
				hci_evnt_hdr = (hci_evnt_hdr_t *)hci_hdr;
				data_to_recv = hci_evnt_hdr->ucLength - 1;
				if ((10 + data_to_recv) & 1) data_to_recv++;
				if (data_to_recv) SpiSend(0, evnt_buff + 10, data_to_recv,ptr);
				break;
		}
		//vTaskDelayUntil( &ptr, WF_DELAY*5 );
		DEASSERT_CS();
		//while(!ReadWlanInterruptPin()) vTaskDelayUntil( &ptr, WF_DELAY );
		sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
		sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + sizeof(btspi_hdr));
	}
}

// инициализация начального состояния
void SpiOpen(gcSpiHandleRx pfRxHandler)
{
	sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;
	sSpiInformation.SPIRxHandler = pfRxHandler;

	sSpiInformation.pRxPacket = wlan_rx_buffer;
	sSpiInformation.usRxPacketLength = 0;
	sSpiInformation.usTxPacketLength = 0;

    tSLInformation.WlanInterruptEnable();
}

// запрещение приёма по SPI
void SpiClose(void)
{
	if (sSpiInformation.pRxPacket) {sSpiInformation.pRxPacket = 0;}
    WlanInterruptDisable();
}

// разрешение приёма по SPI
void SpiResumeSpi(void)
{
    WlanInterruptEnable();
}

