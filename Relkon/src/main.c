/*
 * main.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "htime.h"
#include "lcd.h"
#include "inout.h"
#include "hain.h"
#include "hdac.h"
#include "fc_u.h"
#include "fram.h"
#include "canal.h"

#include "eth_task.h"
#include "sdcard.h"
#include "wifi.h"

#define SYS_STARTUP		0							// настройка RCC уже выпонена в startup файле (1/0)
#define FRAM_YEAR		1							// для хранения года используется FRAM
#define ASSERT_EXT		0							// ссылка на assert в head файле

plc_stat _Sys;										// структура с системными настройками контроллера
volatile unsigned char emu_mode=0;					// определяет режим эмуляции
volatile unsigned long _SysIdleTmr=0,_SysWaitTmr=0;	// переменные для определения загрузки процессора
volatile unsigned long _SysTmr=0,_SysRealTmr=0;
unsigned long max_load=0;							// максимальная загрузка процессора с момента включения
unsigned long cpu=0;
volatile unsigned long adc_sum[8]={0,0,0,0,0,0,0,0};// массив для фильтрации АЦП
extern volatile unsigned short _Sys_ADC[8];
extern volatile short _EA[8];
extern volatile short ext_adc;

/*extern unsigned char ssid_name[32];					// параметры для сети WIFI
extern unsigned char wifi_password[32];
extern unsigned char wifi_type;
extern unsigned char wifi_ip[4];*/

static unsigned char ip_addr[4]={0,0,0,0};			// IP и MAC для сети ethernet
static unsigned char mac_addr[6]={0,0,0,0,0,0};

unsigned char pult_dis,sd_dis;						// флаги разрешения пульта и sd карты

extern volatile unsigned short canalpu_rx_cnt;
extern volatile unsigned short pu_tmr;

extern const unsigned short S4_max;					// максимально допустимый номер нижней строки
extern volatile unsigned long tcp_tmr;

volatile float adc_coeff = 125.0/127;

static void prvFlashTask( void *pvParameters );

portTickType FLLastExecutionTime;

#if FRAM_YEAR
	extern tm times;
#endif

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
#if SYS_STARTUP == 0
  SystemInit();
#endif

  nvic_init();			// настройка векторов прерываний
  led_init();			// инициализация светодиода, индицирующего работу процессора
  init_calendar();		// инициализация календаря
  _SPI_init();			// SPI для внешней FRAM
  

  // чтение заводских установок

  read_data(0x7B,0x00,128,(unsigned char*)&_Sys.FR.b1[0]);
  read_data(0x7B,0x80,128,(unsigned char*)&_Sys.FR.b1[0x80]);
  read_data(0x7C,0x00,128,(unsigned char*)&_Sys.FR.b1[0x100]);
  read_data(0x7C,0x80,128,(unsigned char*)&_Sys.FR.b1[0x180]);
  read_data(0x7D,0x00,128,(unsigned char*)&_Sys.FR.b1[0x200]);
  read_data(0x7D,0x80,128,(unsigned char*)&_Sys.FR.b1[0x280]);
  read_data(0x7E,0x00,128,(unsigned char*)&_Sys.FR.b1[0x300]);
  read_data(0x7E,0x80,128,(unsigned char*)&_Sys.FR.b1[0x380]);

  // чтение системных настроек контроллера

  //read_data(0x7F,0x58,1,&rf_ch);

  read_data(0x7F,0x00,6,(unsigned char*)&_Sys.Mem.b1[0]);

  _Sys.Adr=_Sys.Mem.b1[0];
  if((_Sys.Adr==0)||(_Sys.Adr==0xFF)) _Sys.Adr=0x01;

  _Sys.Can1_Baudrate=_Sys.Mem.b1[1];
  if(_Sys.Can1_Baudrate>5) _Sys.Can1_Baudrate=5;
  _Sys.Can1_Type = _Sys.Mem.b1[2];
  if(_Sys.Can1_Type>1) _Sys.Can1_Type=0;

  _Sys.Can2_Baudrate=_Sys.Mem.b1[3];
  if(_Sys.Can2_Baudrate>5) _Sys.Can2_Baudrate=5;
  _Sys.Can2_Type = _Sys.Mem.b1[4];
  if(_Sys.Can2_Type>1) _Sys.Can2_Type=0;

  emu_mode = _Sys.Mem.b1[5];
  if(emu_mode>2) emu_mode=0;
  
  if(emu_mode==0) init_adc();

  read_data(0x7F,0x4C,4,ip_addr);set_ip(ip_addr);
  read_data(0x7F,0x50,6,mac_addr);set_mac(mac_addr);
#if FRAM_YEAR
  read_data(0x7F,0x56,1,(unsigned char*)&times.year);
  if(times.year>99) times.year=0;
#endif

  read_data(0x7F,0x57,1,(unsigned char*)&pult_dis);
  read_data(0x7F,0x59,1,(unsigned char*)&sd_dis);

/*  read_data(0x7F,0x5A,32,ssid_name);
  read_data(0x7F,0x7A,32,wifi_password);
  read_data(0x7F,0x9A,1,&wifi_type);
  read_data(0x7F,0x9B,4,wifi_ip);*/

  // начальные значения номеров строк пульта
  _Sys.S1=0x00;_Sys.S2=0x00;_Sys.S3=0x00;_Sys.S4=0x00;

  // системные миллисекундные задачи
  xTaskCreate( R1Task, ( signed portCHAR * ) "R1", configMINIMAL_STACK_SIZE*2, NULL, R1_TASK_PRIORITY, NULL );
  xTaskCreate( R5Task, ( signed portCHAR * ) "R5", configMINIMAL_STACK_SIZE*2, NULL, R5_TASK_PRIORITY, NULL );
  xTaskCreate( R10Task, ( signed portCHAR * ) "R10", configMINIMAL_STACK_SIZE*2, NULL, R10_TASK_PRIORITY, NULL );
  xTaskCreate( R100Task, ( signed portCHAR * ) "R100", configMINIMAL_STACK_SIZE*2, NULL, R100_TASK_PRIORITY, NULL );
  xTaskCreate( R1000Task, ( signed portCHAR * ) "R1000", configMINIMAL_STACK_SIZE*2, NULL, R1000_TASK_PRIORITY, NULL );
  // служебная вспомогательная задача (светодиод,фильтр ацп,...)
  xTaskCreate( prvFlashTask, (signed portCHAR *) "Flash", configMINIMAL_STACK_SIZE*2, NULL, mainFLASH_TASK_PRIORITY, NULL );
  // стандартный пульт или коммуникационный канал
  if(pult_dis!=0x31) xTaskCreate( LCDTask, ( signed portCHAR * ) "Lcd", configMINIMAL_STACK_SIZE*2, NULL, LCD_TASK_PRIORITY, NULL );
    else xTaskCreate( PultCanTask, ( signed portCHAR * ) "Lcd", configMINIMAL_STACK_SIZE*5, NULL, Canal_TASK_PRIORITY, NULL );
  // задача обработки входов/выходов включая модули расширения
  xTaskCreate( InOutTask, ( signed portCHAR * ) "InOut", configMINIMAL_STACK_SIZE*2, NULL, InOut_TASK_PRIORITY, NULL );
  // два основных коммуникационных канала RS485
  if(_Sys.Can1_Type==0) xTaskCreate( BinCanTask, ( signed portCHAR * ) "Canal", configMINIMAL_STACK_SIZE*2, NULL, Canal_TASK_PRIORITY, NULL );
  else xTaskCreate( AsciiCanTask, ( signed portCHAR * ) "Canal", configMINIMAL_STACK_SIZE*2, NULL, Canal_TASK_PRIORITY, NULL );
  if(_Sys.Can2_Type==0) xTaskCreate( BinCan2Task, ( signed portCHAR * ) "Canal2", configMINIMAL_STACK_SIZE*2, NULL, Canal_TASK_PRIORITY, NULL );
  else xTaskCreate( AsciiCan2Task, ( signed portCHAR * ) "Canal2", configMINIMAL_STACK_SIZE*2, NULL, Canal_TASK_PRIORITY, NULL );
  // WIFI задачи
/*  if(wifi_ip[0])
  {
    xTaskCreate( WIFITask, ( signed portCHAR * ) "Wifi", configMINIMAL_STACK_SIZE, NULL, WF_TASK_PRIORITY, NULL );
    xTaskCreate( SCAN_WIFITask, ( signed portCHAR * ) "SCANWifi", configMINIMAL_STACK_SIZE, NULL, WF_TASK_PRIORITY+1, NULL );
  }*/
  // проводной ethernet
  if(ip_addr[0]) xTaskCreate( EthTask, ( signed portCHAR * ) "Eth", configMINIMAL_STACK_SIZE*3, NULL, ETH_TASK_PRIORITY, NULL );
  // задача работы с sd картой
  if(sd_dis==0x31) xTaskCreate( ArchiveTask, ( signed portCHAR * ) "SD_Task", configMINIMAL_STACK_SIZE*2, NULL, SD_TASK_PRIORITY, NULL );

  Relkon_init(); // блок инициализации - формируется верхним уровнем (раздел #init релкона)

  init_timer();	// аппаратный таймер для определения загрузки процессора
  vTaskStartScheduler();  // старт планировщика задач
  while (1){}
}

void vApplicationTickHook( void )	// вызывается каждую миллисекунду
{
	static unsigned char ext_cnt=0,anum=0;
	double ain;
	if(emu_mode==0){
		switch(ext_cnt)
		{
			case 0:
				if(ext_adc<0) _EA[5-anum]=0;
				else {
					ain = (double)ext_adc * 1.08;
					if(ain>=32767) _EA[5-anum]=32767;
					else _EA[5-anum]=ain;
				}
				anum++;
				if(anum>5) anum=0;
				adc_write_set(anum);
				break;
			case 1:case 2:
				break;
			case 3:
				get_ext_adc();
				break;
			case 4:case 5:
				break;
			case 6:
				get_ext_adc();
				break;
		}
		ext_cnt++;
		if(ext_cnt>=7) ext_cnt=0;
	}
	_SysTmr++;tcp_tmr++;
	if (_Sys.S4 >= S4_max) _Sys.S4 = S4_max - 1;	// ограничение номера нижней строки
	rst_pu_toggle();	// переключение RS485 пультового канала на приём в случае отсутствия передачи данных
	if(canalpu_rx_cnt) pu_tmr++;else pu_tmr=0;
	if(_SysTmr==1000)								// пересчёт загрузки процессора
	{
	    _SysTmr=0;
	    if(_SysWaitTmr>100000) _SysWaitTmr=100000;
	    _SysRealTmr=1000-_SysWaitTmr/100;
	    if(_SysRealTmr>999) _SysRealTmr=999;
	    if(_SysRealTmr>max_load) max_load=_SysRealTmr;
	    _SysWaitTmr=0;
	}
}

void vApplicationIdleHook(void)	// вызывается в промежутках ожидания задач
{
	_SysIdleTmr=get_idle_tmr();
}

// вспомогательная служебная задача
static void prvFlashTask( void *pvParameters )
{
    static unsigned short s_tmr=0;
	static float adc_res = 0;
	static unsigned short adc_max[8];
	static unsigned short adc_min[8];
	static unsigned short cur_adc = 0;

    unsigned char tmp;
    //init_adc();
	init_dac();		// инициализация ацп и цап
    update_code();				// обновление кодового слова в FRAM
    FLLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{

    	// фильтр АЦП
    	if(emu_mode==0){
			for(tmp=0;tmp<8;tmp++) {
				cur_adc = get_adc(tmp);
				if(s_tmr % 8 == 0) {adc_min[tmp]=adc_max[tmp]=cur_adc;}
				adc_sum[tmp]+=cur_adc;
				if(cur_adc<adc_min[tmp]) adc_min[tmp] = cur_adc;
				if(cur_adc>adc_max[tmp]) adc_max[tmp] = cur_adc;
			}
			s_tmr++;
			if(s_tmr % 8 == 0) {
				for(tmp=0;tmp<8;tmp++) {
					adc_res=(adc_sum[tmp]-adc_min[tmp]-adc_max[tmp])*16.0/(8-2)*adc_coeff+0.5;
					adc_sum[tmp]=0;
					if(adc_res>65535) adc_res=65535;_Sys_ADC[tmp]=adc_res;
				}
			}
		}
    	
    	if((s_tmr%64)==0) {get_time();toggle_led();}	// обновление текущего времени и мигание светодиода
        vTaskDelayUntil( &FLLastExecutionTime, mainFLASH_DELAY );
	}
}

#if ASSERT_EXT == 0
/* Minimal __assert_func used by the assert() macro */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/* Minimal __assert() uses __assert__func() */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}
#endif
