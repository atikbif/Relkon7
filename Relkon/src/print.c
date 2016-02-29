/*
 * print.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "print.h"
#include "lcd.h"
#include "htime.h"
#include "main.h"

extern unsigned char lcd_buf[4][20];
extern tm times;
extern unsigned char diagn_str1;
extern unsigned long fl_in,fl_out,fl_adc,fl_dac;
extern volatile unsigned long _SysRealTmr;
extern unsigned long max_load;
extern unsigned int r100;
extern unsigned long Sum_err;
extern volatile unsigned char err_mod[256];
extern unsigned char mod_pos;
extern ed_var ed;
extern plc_stat _Sys;

unsigned char ascii[7];

// печать беззнаковой переменной nsigned long в буфер
unsigned char print_long_buf(unsigned long val,unsigned char* ptr)
{
	unsigned char n10=0,n100=0,n1000=0,n10000=0,n100000=0,n1000000=0;
	unsigned char first=0;
	while(val>=1000000){val-=1000000;n1000000++;}
	while(val>=100000){val-=100000;n100000++;}
	while(val>=10000){val-=10000;n10000++;}
	while(val>=1000){val-=1000;n1000++;}
	while(val>=100){val-=100;n100++;}
	while(val>=10){val-=10;n10++;}
	if(n1000000) {ptr[first]=n1000000+'0';first++;}
	if((n100000)||(first)) {ptr[first]=n100000+'0';first++;}
	if((n10000)||(first)) {ptr[first]=n10000+'0';first++;}
	if((n1000)||(first)) {ptr[first]=n1000+'0';first++;}
	if((n100)||(first)) {ptr[first]=n100+'0';first++;}
	if((n10)||(first)) {ptr[first]=n10+'0';first++;}
	ptr[first]=val+'0';
	return (first+1);
}

// ечать переменной типа long в буфер пульта
void print_long(long val, unsigned char str_num,unsigned char pos,unsigned char s,unsigned char point)
{
    unsigned char n10=0,n100=0,n1000=0,n10000=0,n100000=0,n1000000=0;
    if(val<0){lcd_buf[str_num-1][pos-2]='-';val=val*(-1);}
    while(val>=1000000){val-=1000000;n1000000++;}
    while(val>=100000){val-=100000;n100000++;}
    while(val>=10000){val-=10000;n10000++;}
    while(val>=1000){val-=1000;n1000++;}
    while(val>=100){val-=100;n100++;}
    while(val>=10){val-=10;n10++;}
    str_num--;pos--;
    switch(s)
    {
        case 7:
            lcd_buf[str_num][pos++]='0'+n1000000;
            if(point==6) lcd_buf[str_num][pos++]='.';
        case 6:
            lcd_buf[str_num][pos++]='0'+n100000;
            if(point==5) lcd_buf[str_num][pos++]='.';
        case 5:
            lcd_buf[str_num][pos++]='0'+n10000;
            if(point==4) lcd_buf[str_num][pos++]='.';
        case 4:
            lcd_buf[str_num][pos++]='0'+n1000;
            if(point==3) lcd_buf[str_num][pos++]='.';
        case 3:
            lcd_buf[str_num][pos++]='0'+n100;
            if(point==2) lcd_buf[str_num][pos++]='.';
        case 2:
            lcd_buf[str_num][pos++]='0'+n10;
            if(point==1) lcd_buf[str_num][pos++]='.';
        default:
            lcd_buf[str_num][pos]='0'+val;
    }
}

// вывод переменной типа float  пульт
void print_float(float val, unsigned char str_num,unsigned char pos,unsigned char s,unsigned char fpoint)
{
    long temp=0;
    switch(fpoint)
    {
        case 0:temp=(long)val;break;
        case 1:temp=(long)(val*10);break;
        case 2:temp=(long)(val*100);break;
        case 3:temp=(long)(val*1000);break;
    }
    print_long(temp,str_num,pos,s,fpoint);
}

// вывод подстроки в пульт
void print_str(char* ptr, unsigned char str_num,unsigned char pos,unsigned char width)
{
    unsigned char tmp=0;
    str_num--;pos--;
    while(tmp<width) {if(pos>=20) break;lcd_buf[str_num][pos++]=ptr[tmp++];}
}

// вывод времени на пульт
void print_time(unsigned char str_num,unsigned char pos,unsigned char type)
{
    if(ed.mode)	// если активен режим редактирования
    {
        ed.cnt++;
        if((ed.search)&&(ed.cnt==ed.search_num))
        {

            ed.search=0;
            switch(type)
            {
                case SEC_TYPE:ed.point=(void*)(&times.sec);ed.var=times.sec;break;
                case MIN_TYPE:ed.point=(void*)(&times.min);ed.var=times.min;break;
                case HOUR_TYPE:ed.point=(void*)(&times.hour);ed.var=times.hour;break;
                case DATE_TYPE:ed.point=(void*)(&times.date);ed.var=times.date;break;
                case MONTH_TYPE:ed.point=(void*)(&times.month);ed.var=times.month;break;
                case YEAR_TYPE:ed.point=(void*)(&times.year);ed.var=times.year;break;
            }

            ed.var_num=ed.cnt;
            ed.index=65530+type;

            ed.str1=_Sys.S1;ed.str2=_Sys.S2;ed.str3=_Sys.S3;ed.str4=_Sys.S4;
            ed.curs_on=1;
            ed.curs_x=pos;
            ed.curs_y=str_num-1;
            ed.pos=0;
            ed.left=pos-1;
            ed.right=ed.curs_x;
            ed.p=ed.right+1;
        }
        else
        {
            if(ed.cnt==ed.var_num) {print_long(ed.var,str_num,pos,2,0);}
            else
            {
                switch(type)
                {
                    case SEC_TYPE:print_long(times.sec,str_num,pos,2,0); break;
                    case MIN_TYPE:print_long(times.min,str_num,pos,2,0); break;
                    case HOUR_TYPE:print_long(times.hour,str_num,pos,2,0); break;
                    case DATE_TYPE:print_long(times.date,str_num,pos,2,0); break;
                    case MONTH_TYPE:print_long(times.month,str_num,pos,2,0); break;
                    case YEAR_TYPE:print_long(times.year,str_num,pos,2,0); break;
                }
            }
        }
    }else
    {
        switch(type)
        {
            case SEC_TYPE:print_long(times.sec,str_num,pos,2,0); break;
            case MIN_TYPE:print_long(times.min,str_num,pos,2,0); break;
            case HOUR_TYPE:print_long(times.hour,str_num,pos,2,0); break;
            case DATE_TYPE:print_long(times.date,str_num,pos,2,0); break;
            case MONTH_TYPE:print_long(times.month,str_num,pos,2,0); break;
            case YEAR_TYPE:print_long(times.year,str_num,pos,2,0); break;
        }
    }
}

// отображение переменной, доступной для редактирования, на пульт
void print_edit(void* val,unsigned char str_num,unsigned char pos,unsigned char width,unsigned char point,unsigned char type)
{
    if(ed.mode)
    {
        ed.cnt++;
        if((ed.search)&&(ed.cnt==ed.search_num))
        {
            ed.search=0;ed.point=val;ed.var_num=ed.cnt;ed.index=0;
            switch(type)
            {
                case 0x01:ed.var=*(unsigned char*) val;break;
                case 0x02:ed.var=*(unsigned short*) val;break;
                case 0x03:ed.var=*(unsigned long*) val;break;
                case 0x81:ed.var=*(char*) val;break;
                case 0x82:ed.var=*(short*) val;break;
                case 0x83:ed.var=*(long*) val;break;
            }
            ed.str1=_Sys.S1;ed.str2=_Sys.S2;ed.str3=_Sys.S3;ed.str4=_Sys.S4;
            ed.curs_on=1;
            ed.curs_x=pos+width-2;
            if(point) ed.curs_x++;
            ed.curs_y=str_num-1;
            ed.pos=0;
            ed.left=pos-1;
            ed.right=ed.curs_x;
            ed.p=ed.right-point;
            if(point==0) ed.p++;
            ed.type=type;
        }
        else
        {
            if(ed.cnt==ed.var_num)
            {
                print_long(ed.var,str_num,pos,width,point);
            }
            else
            {
                switch(type)
                {
                    case 0x01:print_long(*(unsigned char*)val,str_num,pos,width,point);break;
                    case 0x02:print_long(*(unsigned short*)val,str_num,pos,width,point);break;
                    case 0x03:print_long(*(unsigned long*)val,str_num,pos,width,point);break;
                    case 0x81:print_long(*(char*)val,str_num,pos,width,point);break;
                    case 0x82:print_long(*(short*)val,str_num,pos,width,point);break;
                    case 0x83:print_long(*(long*)val,str_num,pos,width,point);break;
                }
            }
        }
    }else
    {
        switch(type)
        {
            case 0x01:print_long(*(unsigned char*)val,str_num,pos,width,point);break;
            case 0x02:print_long(*(unsigned short*)val,str_num,pos,width,point);break;
            case 0x03:print_long(*(unsigned long*)val,str_num,pos,width,point);break;
            case 0x81:print_long(*(char*)val,str_num,pos,width,point);break;
            case 0x82:print_long(*(short*)val,str_num,pos,width,point);break;
            case 0x83:print_long(*(long*)val,str_num,pos,width,point);break;
        }
    }
}

// отображение заводской установки, доступной для редактирования, на пульт
void print_edit_ee(unsigned short ind,unsigned char str_num,unsigned char pos,unsigned char width,unsigned char point,unsigned char type)
{
    if(ind>1023) return;
    if(ed.mode)
    {
        ed.cnt++;
        if((ed.search)&&(ed.cnt==ed.search_num))
        {
            ed.search=0;ed.point=(void*)(&_Sys.FR.b1[ind]);ed.var_num=ed.cnt;ed.index=ind+1;
            switch(type)
            {
                case 0x01:ed.var=*(unsigned char*) ed.point;break;
                case 0x02:ed.var=*(unsigned short*) ed.point;break;
                case 0x03:ed.var=*(unsigned long*) ed.point;break;
                case 0x81:ed.var=*(char*) ed.point;break;
                case 0x82:ed.var=*(short*) ed.point;break;
                case 0x83:ed.var=*(long*) ed.point;break;
            }
            ed.str1=_Sys.S1;ed.str2=_Sys.S2;ed.str3=_Sys.S3;ed.str4=_Sys.S4;
            ed.curs_on=1;
            ed.curs_x=pos+width-2;
            if(point) ed.curs_x++;
            ed.curs_y=str_num-1;
            ed.pos=0;
            ed.left=pos-1;
            ed.right=ed.curs_x;
            ed.p=ed.right-point;
            if(point==0) ed.p++;
            ed.type=type;
        }
        else
        {
            if(ed.cnt==ed.var_num)
            {
                print_long(ed.var,str_num,pos,width,point);
            }
            else
            {
                switch(type)
                {
                    case 0x01:print_long(*(unsigned char*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
                    case 0x02:print_long(*(unsigned short*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
                    case 0x03:print_long(*(unsigned long*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
                    case 0x81:print_long(*(char*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
                    case 0x82:print_long(*(short*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
                    case 0x83:print_long(*(long*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
                }
            }
        }
    }else
    {
        switch(type)
        {
            case 0x01:print_long(*(unsigned char*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
            case 0x02:print_long(*(unsigned short*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
            case 0x03:print_long(*(unsigned long*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
            case 0x81:print_long(*(char*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
            case 0x82:print_long(*(short*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
            case 0x83:print_long(*(long*)(&_Sys.FR.b1[ind]),str_num,pos,width,point);break;
        }
    }
}

// вывод диагностического меню
void print_diagn(void)
{
	unsigned short tmp;
	for(tmp=0;tmp<20;tmp++) {lcd_buf[0][tmp]=' ';lcd_buf[1][tmp]=' ';lcd_buf[2][tmp]=' ';lcd_buf[3][tmp]=' ';}
	switch(diagn_str1)
	{
		case 0:
			print_long(_SysRealTmr,1,1,3,1);print_long(max_load,1,10,3,1);
			lcd_buf[0][5]='m';lcd_buf[0][6]='a';lcd_buf[0][7]='x';lcd_buf[0][8]=':';
			print_long(Sum_err,1,16,5,0);
			break;
		case 1:
		/////////////////////////////////////////////////////////////////////////////
							// VERSION
		/////////////////////////////////////////////////////////////////////////////
			lcd_buf[0][0]='B';lcd_buf[0][1]='e';lcd_buf[0][2]='p';lcd_buf[0][3]='c';
			lcd_buf[0][4]=0xB8;lcd_buf[0][5]=0xC7;lcd_buf[0][7]=':';
			lcd_buf[0][8]='0';lcd_buf[0][9]='2';lcd_buf[0][10]='8';
			print_long(r100/10,1,17,4,0);
			break;
	}
	if(mod_pos==0) mod_pos=1;
	if(mod_pos<0x41)
	{
		lcd_buf[1][0]='I';lcd_buf[1][1]='N';print_long(mod_pos+3,2,5,2,0);lcd_buf[1][7]=':';
		print_long(err_mod[mod_pos],2,9,3,0);
		if(fl_in&(1<<(mod_pos-1))) lcd_buf[1][19]='+';else lcd_buf[1][19]='-';
	}
	if((mod_pos+1)<0x41)
	{
		lcd_buf[2][0]='I';lcd_buf[2][1]='N';print_long(mod_pos+4,3,5,2,0);lcd_buf[2][7]=':';
		print_long(err_mod[mod_pos+1],3,9,3,0);
		if(fl_in&(1<<(mod_pos+1-1))) lcd_buf[2][19]='+';else lcd_buf[2][19]='-';
	}
	if((mod_pos+2)<0x41)
	{
		lcd_buf[3][0]='I';lcd_buf[3][1]='N';print_long(mod_pos+5,4,5,2,0);lcd_buf[3][7]=':';
		print_long(err_mod[mod_pos+2],4,9,3,0);
		if(fl_in&(1<<(mod_pos+2-1))) lcd_buf[3][19]='+';else lcd_buf[3][19]='-';
	}
	if((mod_pos>=0x41)&&(mod_pos<0x81))
	{
		lcd_buf[1][0]='O';lcd_buf[1][1]='U';lcd_buf[1][2]='T';
		print_long(mod_pos-0x40+3,2,5,2,0);lcd_buf[1][7]=':';
		print_long(err_mod[mod_pos],2,9,3,0);
		if(fl_out&(1<<(mod_pos-0x41))) lcd_buf[1][19]='+';else lcd_buf[1][19]='-';
	}
	if(((mod_pos+1)>=0x41)&&((mod_pos+1)<0x81))
	{
		lcd_buf[2][0]='O';lcd_buf[2][1]='U';lcd_buf[2][2]='T';
		print_long(mod_pos+1-0x40+3,3,5,2,0);lcd_buf[2][7]=':';
		print_long(err_mod[mod_pos+1],3,9,3,0);
		if(fl_out&(1<<(mod_pos+1-0x41))) lcd_buf[2][19]='+';else lcd_buf[2][19]='-';
	}
	if(((mod_pos+2)>=0x41)&&((mod_pos+2)<0x81))
	{
		lcd_buf[3][0]='O';lcd_buf[3][1]='U';lcd_buf[3][2]='T';
		print_long(mod_pos+2-0x40+3,4,5,2,0);lcd_buf[3][7]=':';
		print_long(err_mod[mod_pos+2],4,9,3,0);
		if(fl_out&(1<<(mod_pos+2-0x41))) lcd_buf[3][19]='+';else lcd_buf[3][19]='-';
	}

	if((mod_pos>=0x81)&&(mod_pos<0xA1))
	{
		lcd_buf[1][0]='A';lcd_buf[1][1]='D';lcd_buf[1][2]='C';
		print_long((mod_pos-0x81)*4+9,2,5,3,0);
		lcd_buf[1][7]=lcd_buf[1][8]='.';
		print_long((mod_pos-0x81)*4+12,2,10,3,0);
		lcd_buf[1][13]=':';
		print_long(err_mod[mod_pos],2,15,3,0);
		if(fl_adc&(1<<(mod_pos-0x81))) lcd_buf[1][19]='+';else lcd_buf[1][19]='-';
	}
	if(((mod_pos+1)>=0x81)&&((mod_pos+1)<0xA1))
	{
		lcd_buf[2][0]='A';lcd_buf[2][1]='D';lcd_buf[2][2]='C';
		print_long((mod_pos+1-0x81)*4+9,3,5,3,0);lcd_buf[2][7]=':';
		lcd_buf[2][7]=lcd_buf[2][8]='.';
		print_long((mod_pos+1-0x81)*4+12,3,10,3,0);
		lcd_buf[2][13]=':';
		print_long(err_mod[mod_pos+1],3,15,3,0);
		if(fl_adc&(1<<(mod_pos+1-0x81))) lcd_buf[2][19]='+';else lcd_buf[2][19]='-';
	}
	if(((mod_pos+2)>=0x81)&&((mod_pos+2)<0xA1))
	{
		lcd_buf[3][0]='A';lcd_buf[3][1]='D';lcd_buf[3][2]='C';
		print_long((mod_pos+2-0x81)*4+9,4,5,3,0);lcd_buf[3][7]=':';
		lcd_buf[3][7]=lcd_buf[3][8]='.';
		print_long((mod_pos+2-0x81)*4+12,4,10,3,0);
		lcd_buf[3][13]=':';
		print_long(err_mod[mod_pos+2],4,15,3,0);
		if(fl_adc&(1<<(mod_pos+2-0x81))) lcd_buf[3][19]='+';else lcd_buf[3][19]='-';
	}

	if((mod_pos>=0xA1)&&(mod_pos<0xC1))
	{
		lcd_buf[1][0]='D';lcd_buf[1][1]='A';lcd_buf[1][2]='C';
		print_long((mod_pos-0xA1)*2+5,2,5,2,0);
		lcd_buf[1][6]=lcd_buf[1][7]='.';
		print_long((mod_pos-0xA1)*2+6,2,9,2,0);
		lcd_buf[1][11]=':';
		print_long(err_mod[mod_pos],2,13,3,0);
		if(fl_dac&(1<<(mod_pos-0xA1))) lcd_buf[1][19]='+';else lcd_buf[1][19]='-';
	}
	if(((mod_pos+1)>=0xA1)&&((mod_pos+1)<0xC1))
	{
		lcd_buf[2][0]='D';lcd_buf[2][1]='A';lcd_buf[2][2]='C';
		print_long((mod_pos+1-0xA1)*2+5,3,5,2,0);
		lcd_buf[2][6]=lcd_buf[2][7]='.';
		print_long((mod_pos+1-0xA1)*2+6,3,9,2,0);
		lcd_buf[2][11]=':';
		print_long(err_mod[mod_pos+1],3,13,3,0);
		if(fl_dac&(1<<(mod_pos+1-0xA1))) lcd_buf[2][19]='+';else lcd_buf[2][19]='-';
	}
	if(((mod_pos+2)>=0xA1)&&((mod_pos+2)<0xC1))
	{
		lcd_buf[3][0]='D';lcd_buf[3][1]='A';lcd_buf[3][2]='C';
		print_long((mod_pos+2-0xA1)*2+5,4,5,2,0);
		lcd_buf[3][6]=lcd_buf[3][7]='.';
		print_long((mod_pos+2-0xA1)*2+6,4,9,2,0);
		lcd_buf[3][11]=':';
		print_long(err_mod[mod_pos+2],4,13,3,0);
		if(fl_dac&(1<<(mod_pos+2-0xA1))) lcd_buf[3][19]='+';else lcd_buf[3][19]='-';
	}
}

unsigned char* conv_to_ascii(unsigned long value)
{
	ascii[6] = (value % 10000000)/1000000 + '0';
	ascii[5] = (value % 1000000)/100000 + '0';
	ascii[4] = (value % 100000)/10000 + '0';
	ascii[3] = (value % 10000)/1000 + '0';
	ascii[2] = (value % 1000)/100 + '0';
	ascii[1] = (value% 100)/10 + '0';
	ascii[0] = (value % 10) + '0';
	return ascii;
}
