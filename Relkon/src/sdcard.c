/*
 * sdcard.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "sdcard.h"
#include "fat_filelib.h"
#include "FreeRTOS.h"

unsigned char sd_stat=0;
volatile unsigned char sd_buf[512];
volatile unsigned char sd_error_cnt=0;
unsigned long sd_size;
unsigned long sd_addr;
unsigned short sd_err=0;

extern unsigned long sd_fl;

portTickType SDLastExecutionTime;

// чтение одного или нескольких секторов в буфер
int media_read(unsigned long sector, unsigned char *buffer, unsigned long sector_count)
{
    unsigned long i;
    char n,res;

    for (i=0;i<sector_count;i++)
    {
    	n=0;do{res=SD_readSector(sector,buffer,512);n++;}while((res<0)&&(n<100));
    	if(n>=100) {return 0;}
        sector ++;
        buffer += 512;
    }
    return 1;
}

// запись одного или нескольких секторов в SD карту
int media_write(unsigned long sector, unsigned char *buffer, unsigned long sector_count)
{
    unsigned long i;
    char n,res;
    for (i=0;i<sector_count;i++)
    {
    	n=0;do{res=SD_writeSector(sector,buffer);n++;}while((res<00)&&(n<100));
    	if(n>=100) {return 0;}
        sector ++;
        buffer += 512;
    }
    return 1;
}

// основная задача
void ArchiveTask(void *pvParameters)
{
	//fl_init();
	//fl_attach_media(media_read, media_write);
    SDLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
        switch(sd_stat)
        {
            case 0:
            	// инициализация sd карты
            	vTaskDelayUntil( &SDLastExecutionTime,  (portTickType) 100 / portTICK_RATE_MS);
            	CS_SDInit();
            	vTaskDelayUntil( &SDLastExecutionTime,  (portTickType) 100 / portTICK_RATE_MS);
            	sd_stat++;break;
            case 1:sd_stat=0;if(SD_Init()==0) sd_stat=4;break;
            case 2:
            	// получена команда чтения сектора
            	if(SD_readSector(sd_addr,(unsigned char*)sd_buf,512)==0) {sd_error_cnt=0;sd_stat=6;}else {sd_error_cnt++;sd_stat=3;}break;
            case 3:
            	// ошибка чтения сектора
            	sd_stat=2;if(sd_error_cnt>=5) sd_stat=0;break;
            case 4:
            	// инициализация файловой системы (FAT32)
            	//sd_buf[0]='0';sd_buf[1]='1';sd_buf[2]='2';sd_addr=0;sd_stat=7;break;
            	//if(SD_getDriveSize(&sd_size)) sd_stat=5;break;

            	fl_init();
            	if (fl_attach_media(media_read, media_write) == FAT_INIT_OK) sd_stat=5;break;
            case 5: break; // спешная инициализация FAT32
            case 6:break; // успешное чтение сектора
            case 7:
            	// получена команда записи сектора
            	if(SD_writeSector(sd_addr,(unsigned char*)sd_buf)==0) {sd_error_cnt=0;sd_stat=9;}else {sd_error_cnt++;sd_stat=8;}break;
            case 8:
            	// ошибка записи сектора
            	sd_stat=7;if(sd_error_cnt>=5) sd_stat=0;break;
            case 9:break;// успешная запись сектора
        }
        vTaskDelayUntil( &SDLastExecutionTime,  (portTickType) 20 / portTICK_RATE_MS);
    }
}

void SD_Cmd(unsigned char cmd, unsigned short paramx, unsigned short paramy)
{
	CS_SDSend(0xff);
	CS_SDSend(0x40 | cmd);
	CS_SDSend((unsigned char) (paramx >> 8));
	CS_SDSend((unsigned char) (paramx));
	CS_SDSend((unsigned char) (paramy >> 8));
	CS_SDSend((unsigned char) (paramy));
	CS_SDSend(0x95);
	CS_SDSend(0xff);
}

unsigned char SD_Resp8b(void)
{
	unsigned char i;
	unsigned char resp;
	for(i=0;i<8;i++){resp = CS_SDSend(0xff);if(resp != 0xff) return(resp);}
	return(resp);
}

unsigned short SD_Resp16b(void)
{
	unsigned short resp;
	resp = ( SD_Resp8b() << 8 ) & 0xff00;
	resp |= CS_SDSend(0xff);
	return(resp);
}

char SD_writeSector(unsigned long address, unsigned char* buf)
{
	unsigned long place;
	unsigned short i;
	unsigned short t=0;
	place=512*address;
	SD_Cmd(CMDWRITE, (unsigned short) (place >> 16), (unsigned short) place);
	SD_Resp8b();
	CS_SDSend(0xfe);
	for(i=0;i<512;i++) CS_SDSend(buf[i]);
	CS_SDSend(0xff);
	CS_SDSend(0xff);
	CS_SDSend(0xff);
	while(CS_SDSend(0xff)!=0xff)
    {
        t++;if(t>=1000) {sd_err++;return(-1);}
        vTaskDelayUntil( &SDLastExecutionTime, ( ( portTickType ) 5 / portTICK_RATE_MS ) );
    }
	sd_err=0;
	return(0);
}

char SD_readSector(unsigned long address, unsigned char* buf, unsigned short len)
{
	unsigned char cardresp;
	unsigned char firstblock;
	unsigned char c;
	unsigned short fb_timeout=1000;
	unsigned long i;
	unsigned long place;
	place=512*address;
	SD_Cmd(CMDREAD, (unsigned short) (place >> 16), (unsigned short) place);
	cardresp=SD_Resp8b(); /* Card response */
    if(cardresp!=0x00) {sd_err++;return -1;}
	do{firstblock=SD_Resp8b(); vTaskDelayUntil( &SDLastExecutionTime, ( ( portTickType ) 2 / portTICK_RATE_MS ) );}
        while(firstblock==0xff && fb_timeout--);
    if(firstblock!=0xFE) {sd_err++;return(-1);}
	for(i=0;i<512;i++){c = CS_SDSend(0xff);	if(i<len) buf[i] = c;}
	CS_SDSend(0xff);
	CS_SDSend(0xff);
	sd_err=0;
	return(0);
}

char SD_getDriveSize(unsigned long* drive_size )
{
	unsigned char cardresp, i, by;
	unsigned char iob[16];
	unsigned short c_size, c_size_mult, read_bl_len,err=0;
	SD_Cmd(CMDREADCSD, 0, 0);
	do {cardresp = SD_Resp8b();err++;} while ((cardresp != 0xFE)&&(err<10000));
	for( i=0; i<16; i++) {iob[i] = SD_Resp8b();}
	CS_SDSend(0xff);
	CS_SDSend(0xff);
	if(err>=10000) return 0;
	c_size = iob[6] & 0x03; // bits 1..0
	c_size <<= 10;
	c_size += (unsigned short)iob[7]<<2;
	c_size += iob[8]>>6;
    by= iob[5] & 0x0F;
	read_bl_len = 1;
	read_bl_len <<= by;
	by=iob[9] & 0x03;
	by <<= 1;
	by += iob[10] >> 7;
	c_size_mult = 1;
	c_size_mult <<= (2+by);
	*drive_size = (unsigned long)(c_size+1) * (unsigned long)c_size_mult * (unsigned long)read_bl_len;
	return 0;
}

char SD_Init(void)
{
	short i;
	volatile unsigned char resp;
	i=1000; do{SD_Cmd(0, 0, 0);resp=SD_Resp8b();vTaskDelayUntil( &SDLastExecutionTime, ( ( portTickType ) 1 / portTICK_RATE_MS ) );}while(resp!=1 && i--);
	if(resp!=1){if(resp==0xff) return(-1); else return(-2);}
	i=10000;	do{SD_Cmd(1, 0, 0);resp=SD_Resp8b();vTaskDelayUntil( &SDLastExecutionTime, ( ( portTickType ) 1 / portTICK_RATE_MS ) );} while(resp==1 && i--);
	if(resp!=0){return(-3);}
	return(0);
}

char initInterface(void)
{
  unsigned long sc;
  CS_SDInit();
  if (SD_Init() < 0)  {return(-1);}
  SD_getDriveSize(&sc);
  return(0);
}

