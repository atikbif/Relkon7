/*
 * string_func.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "string_func.h"

// поиск строки str в буфере buf длиной(буфера) l
unsigned char find(unsigned char* str,unsigned char* buf, unsigned short l)
{
    unsigned short len,u,t,f;
	len=0;
	while(str[len]) {len++;if(len>l) return 0;}
    for(u=0;u<=l-len;u++)
    {
        f=0;for(t=0;t<len;t++) {if(str[t]!=buf[u+t]) {f=1;break;}}
        if(f==0) return (u+1);
    }
    return 0;
}

// преобразование двух ascii символов в бинарное значение
unsigned char to_hex(unsigned char* ptr)
{
    unsigned char res='?';
    if((ptr[0]>='0')&&(ptr[0]<='9')) res=(ptr[0]-'0')<<4;
    else
    {
        if((ptr[0]>='A')&&(ptr[0]<='F')) res=((ptr[0]-'A')+10)<<4;
    }
    if((ptr[1]>='0')&&(ptr[1]<='9')) res|=ptr[1]-'0';
    else
    {
        if((ptr[1]>='A')&&(ptr[1]<='F')) res|=(ptr[1]-'A')+10;
    }
    return res;
}

