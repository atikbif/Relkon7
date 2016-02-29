/*
 * lcd.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "lcd.h"
#include "fc_u.h"
#include "fram.h"
#include "hain.h"
#include "print.h"
#include "htime.h"
#include "main.h"
#include "FreeRTOS.h"

volatile unsigned long* led_ptr;
volatile unsigned long* key_ptr;
unsigned long led_base;
unsigned char lcd_buf[4][20]={"                    ","                    ","                    ","                    "};
unsigned char TxLCDBuf[84];
volatile unsigned char led=0x00;
volatile unsigned char _SysKey;
static unsigned char key;
unsigned char 	diagn_mod=0;
unsigned char mod_pos=0;
static unsigned short 	key_tmr=0;
ed_var ed;
unsigned char diagn_str1=0;

extern unsigned long Sum_err;
extern const unsigned char str1[][20];
extern const unsigned char str2[][20];
extern const unsigned char str3[][20];
extern const unsigned char str4[][20];
extern plc_stat _Sys;
extern tm times,wr_times;
extern volatile unsigned char _Sys_SPI_Buzy;
extern const unsigned short S4_max;
extern volatile unsigned char err_mod[256];

//extern unsigned char eth_stat;
//extern unsigned short eth_pkt_cnt;

void LCDTask( void *pvParameters )
{
    unsigned char tmp;
    unsigned char lcd_crc;
    unsigned short EE_adr;
    portTickType xLastExecutionTime;


    xLastExecutionTime = xTaskGetTickCount();

    init_lcd_canal();

  	led_base=(unsigned long)(&led);
	led_base=((led_base-0x20000000)<<5)+0x22000000;
	led_ptr=(unsigned long*)led_base;

	led_base=(unsigned long)(&key);
	led_base=((led_base-0x20000000)<<5)+0x22000000;
	key_ptr=(unsigned long*)led_base;

    for(;;)
    {
    	if((diagn_mod==0)&&(ed.mode==0)) key=_SysKey; else key=0;
    	if(diagn_mod)
    	{
    		if(key_tmr<TOGGLE_DIAGN) key_tmr++;else
            switch(_SysKey)
            {
            	case KEY_F1:	// сброс суммарной ошибки по модулям ввода/вывода
            		Sum_err=0;break;
            	case (KEY_S | KEY_F2):	// выход из режима диагностики
            		diagn_mod=0;key_tmr=0;break;
            	case KEY_D:	// перелистывание отдельных модулей
            		if(mod_pos<0xC0) mod_pos++;
            		key_tmr=TOGGLE_DIAGN-4;break;
            	case KEY_U:	// перелистывание отдельных модулей
            		if(mod_pos) mod_pos--;
            		key_tmr=TOGGLE_DIAGN-4;break;
            	case KEY_R: // перелистывание групп модулей
            		if(mod_pos<0x41) mod_pos=0x41;
            		else{if(mod_pos<0x81) mod_pos=0x81;else {if(mod_pos<0xA1) mod_pos=0xA1;}}
            		key_tmr=TOGGLE_DIAGN-4;break;
            	case KEY_L: // перелистывание групп модулей
            	    if(mod_pos>=0xA1) mod_pos=0x81;
            	    else {if(mod_pos>=0x81) mod_pos=0x41;	else {if(mod_pos>=0x41) mod_pos=0x01;}}
            	    key_tmr=TOGGLE_DIAGN-4;break;
            	case KEY_E: // перелистывание первой строки
            		if(diagn_str1) diagn_str1=0; else diagn_str1=1;key_tmr=0;break;
            }
    		print_diagn();
    	}else
    	{
    		if(key_tmr<TOGGLE_DIAGN) key_tmr++;
    		// edit_pos - определяет важность положения курсора (1,10,100,...)
    		// curs_x, curs_y - текущие координаты курсора
    		// edit_right - раничное правое положение курсора в пределах числа
    		// edit_left - раничное левое положение курсора в пределах числа
    		// edit_search_num - номер переменной для поиска на дисплее, доступной для редактирования
    		// edit_search - флаг запуска поиска
    		// edit_p - координаты точки в редактируемой переменной
    		// edit_var - буферизованное значение редактируемой переменной
    		// edit_type - тип переменной (кол-во байт и знаковость)
    		// edit_index - адрес заводской уставки или системного времени
    		else switch(_SysKey)
    		{
    			case (KEY_S | KEY_F2):	// переход в режим диагностики
    				diagn_mod=1;key_tmr=0;break;
    			case KEY_R:	// перелистывание нижней строки или курсора при редактировании
    				if(ed.mode==0) {if(_Sys.S4<(S4_max-1)) _Sys.S4++;}
                    else
    			    {
                    	if(ed.curs_x==ed.right) {ed.search_num++;ed.search=1;}
    			        else{ed.curs_x++;if(ed.curs_x==ed.p) ed.curs_x++;ed.pos--;}
    			    }key_tmr=TOGGLE_DIAGN-4;break;
    			case KEY_L: // перелистывание нижней строки или курсора при редактировании
    			    if(ed.mode==0) {if(_Sys.S4) _Sys.S4--;}
    			    else
    			    {
    			    	if(ed.curs_x==ed.left)
    			        {if(ed.search_num>1) {ed.search_num--;ed.search=1;}}
    			        else{ed.curs_x--;if(ed.curs_x==ed.p) ed.curs_x--;ed.pos++;}
    			    }
    			    key_tmr=TOGGLE_DIAGN-4;break;
    			case KEY_U: // увеличение значения переменной
    				if(ed.mode)
					{
						switch(ed.pos){
						case 0:ed.var++;break;
						case 1:ed.var+=10;break;
						case 2:ed.var+=100;break;
						case 3:ed.var+=1000;break;
						case 4:ed.var+=10000;break;
						case 5:ed.var+=100000;break;
						default:ed.var+=1000000;break;}
						if(ed.var>=9999999) ed.var=9999999;
						if(ed.index>=65530)
						{
							switch(ed.index-65530)
							{
								case SEC_TYPE:case MIN_TYPE: if(ed.var>59) ed.var=59;break;
								case HOUR_TYPE:if(ed.var>23) ed.var=23;break;
								case DATE_TYPE:if(ed.var>31) ed.var=31;break;
								case MONTH_TYPE:if(ed.var>12) ed.var=12;break;
								case YEAR_TYPE:if(ed.var>99) ed.var=99;break;
							}
						}
					}key_tmr=TOGGLE_DIAGN-4;break;
    			case KEY_D: // уменьшение значения переменной
    				if(ed.mode)
					{
						switch(ed.pos){
						case 0:ed.var=ed.var-1;break;
						case 1:ed.var=ed.var-10;break;
						case 2:ed.var=ed.var-100;break;
						case 3:ed.var=ed.var-1000;break;
						case 4:ed.var-=10000;break;
						case 5:ed.var-=100000;break;
						default:ed.var-=1000000;break;}
						if(ed.var<-9999999) ed.var=-9999999;
						if(((ed.type&0x80)==0)&&(ed.var<0)) ed.var=0;
					}key_tmr=TOGGLE_DIAGN-4;break;
    			case KEY_E: // запись значения переменной
					if(ed.mode)
					{
						if(ed.index>=65530)
						{
							wr_times.sec=times.sec;wr_times.min=times.min;wr_times.hour=times.hour;
							wr_times.date=times.date;wr_times.month=times.month;wr_times.year=times.year;
							wr_times.day=times.day;
							switch(ed.index-65530)
							{
								case SEC_TYPE:wr_times.sec=ed.var;break;
								case MIN_TYPE:wr_times.min=ed.var;break;
								case HOUR_TYPE:wr_times.hour=ed.var;break;
								case DATE_TYPE:wr_times.date=ed.var;break;
								case MONTH_TYPE:wr_times.month=ed.var;break;
								case YEAR_TYPE:wr_times.year=ed.var;break;
							}
							set_time();
						}
						if((ed.index)&&(ed.index<1025))
						{
							EE_adr=0x7B00+ed.index-1;
							if((ed.type & 0x7F)==0x01)
							{
								*((unsigned char*)ed.point)=ed.var;
								while(_Sys_SPI_Buzy) vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 1 / portTICK_RATE_MS );
								portDISABLE_INTERRUPTS();
								_Sys_SPI_Buzy=1;
								portENABLE_INTERRUPTS();
								write_enable();
								write_data(EE_adr>>8,EE_adr&0xFF,1,(unsigned char*)ed.point);
								portDISABLE_INTERRUPTS();
								_Sys_SPI_Buzy=0;
								portENABLE_INTERRUPTS();

							}
							if((ed.type & 0x7F)==0x02)
							{
								*((unsigned short*)ed.point)=ed.var;
								while(_Sys_SPI_Buzy) vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 1 / portTICK_RATE_MS );
								portDISABLE_INTERRUPTS();
								_Sys_SPI_Buzy=1;
								portENABLE_INTERRUPTS();
								write_enable();
								write_data(EE_adr>>8,EE_adr&0xFF,2,(unsigned char*)ed.point);
								portDISABLE_INTERRUPTS();
								_Sys_SPI_Buzy=0;
								portENABLE_INTERRUPTS();

							}
							if((ed.type & 0x7F)==0x03)
							{
								*((long*)ed.point)=ed.var;
								while(_Sys_SPI_Buzy) vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 1 / portTICK_RATE_MS );
								portDISABLE_INTERRUPTS();
								_Sys_SPI_Buzy=1;
								portENABLE_INTERRUPTS();
								write_enable();
								write_data(EE_adr>>8,EE_adr&0xFF,4,(unsigned char*)ed.point);
								portDISABLE_INTERRUPTS();
								_Sys_SPI_Buzy=0;
								portENABLE_INTERRUPTS();

							}
							ed.index=0;
						}
						else
						{
							if((ed.type & 0x7F)==0x01) *((unsigned char*)ed.point)=ed.var;
							if((ed.type & 0x7F)==0x02) *((unsigned short*)ed.point)=ed.var;
							if((ed.type & 0x7F)==0x03) *((long*)ed.point)=ed.var;
						}
						ed.mode=0;ed.curs_on=0;
					}key_tmr=0;break;
    			case (KEY_S | KEY_E): // вход/выход - режим редактирования
					if(ed.mode==0) {ed.mode=1;ed.search=1;ed.search_num=1;}else {ed.mode=0;ed.curs_on=0;}
    			key_tmr=0;break;
    		}

    		if((ed.mode)&&(ed.search==0))	// фиксация номеров строк на момент редактирования
			{_Sys.S1=ed.str1;_Sys.S2=ed.str2;_Sys.S3=ed.str3;_Sys.S4=ed.str4;}


			for(tmp=0;tmp<20;tmp++)
			{
				lcd_buf[0][tmp]=str1[_Sys.S1][tmp];
				lcd_buf[1][tmp]=str2[_Sys.S2][tmp];
				lcd_buf[2][tmp]=str3[_Sys.S3][tmp];
				lcd_buf[3][tmp]=str4[_Sys.S4][tmp];
			}
			ed.cnt=0;
			print_var();

			if(ed.search) // если поиск не дал результатов
			{
				if(ed.search_num>1) ed.search_num=1;
				else{ed.mode=0;ed.curs_on=0;ed.search=0;}
			}
			if(ed.curs_on)
			{
				if((++ed.curs_cnt)>=CURS_FR1)
				{if(lcd_buf[ed.curs_y][ed.curs_x]!=0xFF) lcd_buf[ed.curs_y][ed.curs_x]=0xFF;}
				if(ed.curs_cnt>=CURS_FR2) ed.curs_cnt=0;
			}
    	}

    	//print_long(eth_stat,1,1,3,0);
    	//print_long(eth_pkt_cnt,2,1,5,0);

    	for(tmp=0;tmp<20;tmp++) TxLCDBuf[1+tmp]=lcd_buf[0][tmp];
    	for(tmp=0;tmp<20;tmp++) TxLCDBuf[21+tmp]=lcd_buf[2][tmp];
    	for(tmp=0;tmp<20;tmp++) TxLCDBuf[41+tmp]=lcd_buf[1][tmp];
    	for(tmp=0;tmp<20;tmp++) TxLCDBuf[61+tmp]=lcd_buf[3][tmp];

    	lcd_crc=0;TxLCDBuf[0]=0x02;TxLCDBuf[82]=led;
    	for(tmp=0;tmp<82;tmp++)
    	{
    		lcd_crc=(( TxLCDBuf[tmp+1] ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>1) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>2) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>3) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>4) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>5) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>6) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	    lcd_crc=(((TxLCDBuf[tmp+1]>>7) ^ lcd_crc)&0x01)?((lcd_crc ^ 0x18)>>1)|0x80:lcd_crc>>1;
    	}
    	TxLCDBuf[83]=lcd_crc;

    	write_lcd(84);
    	vTaskDelayUntil( &xLastExecutionTime, LCD_DELAY );
    }
}


