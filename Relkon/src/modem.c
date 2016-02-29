/*
 * modem.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "modem.h"
#include "string_func.h"
#include "canal.h"
#include "hcanal.h"

static unsigned char number[11];// вспомогательный массив для сохранения номера входящих смс
static unsigned char passw[6*4+1];// вспомогательный массив для хранения пароля для смс
static const char cmti[] = "+CMTI:";

extern volatile unsigned int canal_rx_cnt;
extern volatile unsigned int canal2_rx_cnt;
extern unsigned char mstr1,mstr2;

const unsigned char ascii_code[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

// возвращает номер отправителя смс
void get_number(unsigned char* ptr)
{
    unsigned char tmp;
    for(tmp=0;tmp<10;tmp++) ptr[tmp]=number[tmp+1];
    ptr[tmp]=0;
}

// устанавливает пароль для смс (строго 6 символов)
void set_password(unsigned char* ptr)
{
    unsigned char tmp;
    for(tmp=0;tmp<6;tmp++)
    {
        passw[tmp*4]='0';
        if(ptr[tmp]<0xC0)
        {
            passw[tmp*4+1]='0';
            passw[tmp*4+2]=ascii_code[ptr[tmp]>>4];
            passw[tmp*4+3]=ascii_code[ptr[tmp]%16];
        }
        else
        {
            passw[tmp*4+1]='4';
            passw[tmp*4+2]=ascii_code[(ptr[tmp]-0xB0)>>4];
            passw[tmp*4+3]=ascii_code[(ptr[tmp]-0xB0)%16];
        }
    }
    passw[6*4]=0;
}

// обработка принятого смс
unsigned char get_sms_text(unsigned char can,unsigned char* ptr,unsigned char max)
{
    unsigned short u,t,f=0;
    unsigned char* rx_buf;
    rx_buf = get_can_rx_ptr(can);
    if(rx_buf==0) return 0;
    // поиск пароля в тексте
    u=find((unsigned char*)passw,rx_buf,BUF_SIZE);
	if(u==0) return 0;
	// сохранение номера отправителя
	number[0]=rx_buf[u-32];number[1]=rx_buf[u-33];
	number[2]=rx_buf[u-30];number[3]=rx_buf[u-31];
	number[4]=rx_buf[u-28];number[5]=rx_buf[u-29];
	number[6]=rx_buf[u-26];number[7]=rx_buf[u-27];
	number[8]=rx_buf[u-24];number[9]=rx_buf[u-25];
	number[10]=rx_buf[u-22];
	t=u-1+24;
	// чтение текста сообщения
	for(u=0;u<max;u++)
	{
		if(rx_buf[t]=='0')
		{
			if(rx_buf[t+1]=='0')
			{
				ptr[f]=to_hex(&rx_buf[t+2]);
				f++;t+=4;
			}
			else
			{
				if(rx_buf[t+1]=='4')
				{
					ptr[f]=0xB0+to_hex(&rx_buf[t+2]);
					f++;t+=4;
				}else return f;
			}
		}else return f;
	}
	return f;
}

// команда чтения sms
unsigned char read_sms(unsigned char can,unsigned char i)
{
    unsigned char tmp;
    unsigned char* tx_buf;
    tx_buf = get_can_tx_ptr(can);
    if(tx_buf==0) return 0;

    tx_buf[0]='A';
	tx_buf[1]='T';
	tx_buf[2]='+';
	tx_buf[3]='C';
	tx_buf[4]='M';
	tx_buf[5]='G';
	tx_buf[6]='R';
	tx_buf[7]='=';
	tmp=8;tx_buf[tmp]='0';
	while(i>=100){tx_buf[tmp]++;i-=100;}
	if(tx_buf[tmp]>'0') {tx_buf[++tmp]='0';}
	while(i>=10){tx_buf[tmp]++;i-=10;}
	if(tx_buf[tmp]>'0'){tmp++;}
	tx_buf[tmp]='0'+i;
	tmp++;tx_buf[tmp]=0x0D;
	switch(can)
	{
		case 1:write_canal2(tmp+1);break;
		case 2:write_canal(tmp+1);break;
		default: return 0;
	}
	return 1;
}

// определение индекса ячейки принятого смс
unsigned char get_index(unsigned char can)
{
    unsigned short u,t,f;
    unsigned char* rx_buf;
    rx_buf = get_can_rx_ptr(can);
    if(rx_buf==0) return 0;
    for(u=0;u<=BUF_SIZE-6;u++)
	{
		f=0;for(t=0;t<6;t++) {if(cmti[t]!=rx_buf[u+t]) {f=1;break;}}
		if(f==0) break;
	}
	if(f==1) return 0;
	t = rx_buf[u+12]-'0';
	if((rx_buf[u+13]>='0')&&(rx_buf[u+13]<='9'))
	{
		t=t*10+rx_buf[u+13]-'0';
		if((rx_buf[u+14]>='0')&&(rx_buf[u+14]<='9')) t=t*10+rx_buf[u+14]-'0';
	}
	return t;
}

// удаление смс по индексу
void del_sms(unsigned char can_num,unsigned char i)
{
    unsigned char tmp;
    unsigned char* tx_buf;
    tx_buf=get_can_tx_ptr(can_num);
    if(tx_buf==0) return;
    tx_buf[0]='A';
	tx_buf[1]='T';
	tx_buf[2]='+';
	tx_buf[3]='C';
	tx_buf[4]='M';
	tx_buf[5]='G';
	tx_buf[6]='D';
	tx_buf[7]='=';
	tmp=8;tx_buf[tmp]='0';
	while(i>=100){tx_buf[tmp]++;i-=100;}
	if(tx_buf[tmp]>'0') {tx_buf[++tmp]='0';}
	while(i>=10){tx_buf[tmp]++;i-=10;}
	if(tx_buf[tmp]>'0'){tmp++;}
	tx_buf[tmp]='0'+i;
	tmp++;tx_buf[tmp]=0x0D;
	switch(can_num)
	{
		case 1:write_canal2(tmp+1);break;
		case 2:write_canal(tmp+1);break;
	}
}

// вызов в режиме передачи данных с +7
void call_data7(unsigned char can_num,unsigned char* num)
{
    unsigned char tmp,n;
    unsigned char* tx_buf;
    tx_buf=get_can_tx_ptr(can_num);
    if(tx_buf==0) return;
    for(tmp=0;tmp<20;tmp++) {if(num[tmp]==0) break;}
    tx_buf[0]='A';
	tx_buf[1]='T';
	tx_buf[2]='D';
	tx_buf[3]='+';
	tx_buf[4]='7';
	for(n=0;n<tmp;n++) tx_buf[5+n]=num[n];
	tx_buf[5+n]=0x0D;
	switch(can_num)
	{
		case 1:write_canal2(n+6);
		case 2:write_canal(n+6);
	}
}

// вызов в голосовом режиме с +7
void call7(unsigned char can_num,unsigned char* num)
{
    unsigned char tmp,n;
    unsigned char* tx_buf;
    tx_buf=get_can_tx_ptr(can_num);
    if(tx_buf==0) return;
    for(tmp=0;tmp<20;tmp++) {if(num[tmp]==0) break;}
    tx_buf[0]='A';
	tx_buf[1]='T';
	tx_buf[2]='D';
	tx_buf[3]='+';
	tx_buf[4]='7';
	for(n=0;n<tmp;n++) tx_buf[5+n]=num[n];
	tx_buf[5+n]=';';
	tx_buf[6+n]=0x0D;
	switch(can_num)
	{
		case 1:write_canal2(n+7);
		case 2:write_canal(n+7);
	}
}

// вызов в режиме передачи данных
void call_data(unsigned char can_num,unsigned char* num)
{
    unsigned char tmp,n;
    unsigned char* tx_buf;
    tx_buf=get_can_tx_ptr(can_num);
    if(tx_buf==0) return;
    for(tmp=0;tmp<20;tmp++) {if(num[tmp]==0) break;}
    tx_buf[0]='A';
	tx_buf[1]='T';
	tx_buf[2]='D';
	for(n=0;n<tmp;n++) tx_buf[3+n]=num[n];
	tx_buf[3+n]=0x0D;
	switch(can_num)
	{
		case 1:write_canal2(n+4);
		case 2:write_canal(n+4);
	}
}

// вызов в голосовом режиме
void call(unsigned char can_num,unsigned char* num)
{
    unsigned char tmp,n;
    unsigned char* tx_buf;
    tx_buf=get_can_tx_ptr(can_num);
    if(tx_buf==0) return;
    for(tmp=0;tmp<20;tmp++) {if(num[tmp]==0) break;}
    tx_buf[0]='A';
	tx_buf[1]='T';
	tx_buf[2]='D';
	for(n=0;n<tmp;n++) tx_buf[3+n]=num[n];
	tx_buf[3+n]=';';
	tx_buf[4+n]=0x0D;
	switch(can_num)
	{
		case 1:write_canal2(n+5);
		case 2:write_canal(n+5);
	}
}

// команда отправки смс
void send_sms(unsigned char can_num,unsigned char* num,unsigned char* sms)
{
    unsigned short var,tmp,conv[12];
    unsigned char* tx_buf;
    tx_buf = get_can_tx_ptr(can_num);
    if(tx_buf==0) return;
    for(tmp=0;tmp<10;tmp++) conv[tmp+1]=num[tmp];
    conv[0]='7';conv[11]='F';
    for(tmp=0;tmp<6;tmp++) {var=conv[tmp*2];conv[tmp*2]=conv[tmp*2+1];conv[tmp*2+1]=var;}
    tx_buf[0]='0';tx_buf[1]='0';
	tx_buf[2]='1';tx_buf[3]='1';
	tx_buf[4]='0';tx_buf[5]='0';
	tx_buf[6]='0';tx_buf[7]='B';
	tx_buf[8]='9';tx_buf[9]='1';
	for(tmp=0;tmp<12;tmp++) tx_buf[10+tmp]=conv[tmp];
	tx_buf[22]='0';tx_buf[23]='0';
	tx_buf[24]='0';tx_buf[25]='8';
	tx_buf[26]='A';tx_buf[27]='A';
	tx_buf[28]='8';tx_buf[29]='C';
	for(tmp=0;tmp<70;tmp++)
	{
		tx_buf[30+tmp*4]='0';
		tx_buf[30+tmp*4+1]='0';
		tx_buf[30+tmp*4+2]='2';
		tx_buf[30+tmp*4+3]='0';
	}
	tmp=0;
	while(sms[tmp])
	{
		tx_buf[30+tmp*4]='0';
		if(sms[tmp]<0xC0)
		{
			tx_buf[30+tmp*4+1]='0';
			tx_buf[30+tmp*4+2]=ascii_code[sms[tmp]>>4];
			tx_buf[30+tmp*4+3]=ascii_code[sms[tmp]%16];
		}else
		{
			tx_buf[30+tmp*4+1]='4';
			tx_buf[30+tmp*4+2]=ascii_code[(sms[tmp]-0xB0)>>4];
			tx_buf[30+tmp*4+3]=ascii_code[(sms[tmp]-0xB0)%16];
		}
		tmp++;
		if(tmp>=70) break;
	}
	tx_buf[310]=0x1A;
    switch(can_num)
    {
    	case 1:write_canal2(311);break;
    	case 2:write_canal(311);break;
    }
}

// отправка заголовка смс
void write_head(unsigned char can_num)
{
	unsigned char* tx_buf;
	tx_buf = get_can_tx_ptr(can_num);
	if(tx_buf==0) return;
	tx_buf[0]='A';tx_buf[1]='T';tx_buf[2]='+';
	tx_buf[3]='C';tx_buf[4]='M';tx_buf[5]='G';
	tx_buf[6]='S';tx_buf[7]='=';tx_buf[8]='1';
	tx_buf[9]='5';tx_buf[10]='4';tx_buf[11]=0x0D;
	switch(can_num)
	{
		case 1: write_canal2(12);break;
		case 2: write_canal(12);break;
	}
}

// сброс счётчика принятых байт
void clear_buf(unsigned char can_num)
{
    unsigned short tmp;
    unsigned char* rx_buf;
    rx_buf=get_can_rx_ptr(can_num);
    if(rx_buf==0) return;
    for(tmp=0;tmp<BUF_SIZE;tmp++) rx_buf[tmp]=0;
    switch(can_num)
    {
    	case 1:mstr2=0;canal2_rx_cnt=0;break;
    	case 2:mstr1=0;canal_rx_cnt=0;break;
    }
}

