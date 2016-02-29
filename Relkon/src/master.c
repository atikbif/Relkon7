/*
 * master.c
 *
 *  Relkon ver 1.0
 *  Author: Роман
 *
 */

#include "master.h"
#include "crc.h"
#include "canal.h"
#include "hcanal.h"
#include "hinout.h"
#include "main.h"

extern volatile unsigned short rx_mod_cnt;
extern volatile unsigned char obj_name[20];
extern unsigned char mstr1,mstr2;
extern plc_stat _Sys;
static const unsigned char ascii_code[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

static unsigned short bin_to_ascii(unsigned char* ptr,unsigned short lng,unsigned char mode);
static unsigned short convert_to_bin(unsigned short start_pos,unsigned short end_pos,unsigned char* ptr);
static char check_ascii(unsigned char symb);

char test_user(unsigned char* ptr,unsigned char cnt)
{
    unsigned char tmp;
    unsigned char* rx_buf;
    rx_buf = get_can_rx_ptr(3);
    if(rx_buf==0) return 0;
    if(rx_mod_cnt==((unsigned short)cnt)+3)
	{
		if(GetCRC16(rx_buf,((unsigned short)cnt)+3)==0)
		{
			for(tmp=0;tmp<cnt;tmp++) ptr[tmp]=rx_buf[tmp+1];
            return 1;
		}
	}
    return 0;
}

void read_user(unsigned char plc_addr,unsigned char addr,unsigned char cnt)
{
    unsigned short crc_val;
    unsigned char* tx_buf;
    if((unsigned short)addr+(unsigned short)cnt > 256) {rx_mod_cnt=0;return;}
    tx_buf=get_can_tx_ptr(3);
    tx_buf[0]=plc_addr;
	tx_buf[1]=0xD0;
	tx_buf[2]=addr;
    tx_buf[3]=cnt;
    crc_val=GetCRC16(tx_buf,4);
    tx_buf[4]=crc_val>>8;
    tx_buf[5]=crc_val&0xFF;
    write_module(6);
}

char test_xram(unsigned char* ptr,unsigned short cnt)
{
    unsigned short tmp;
    unsigned char* rx_buf;
    rx_buf = get_can_rx_ptr(3);
    if(rx_buf==0) return 0;
    if(rx_mod_cnt==cnt+3)
	{
		if(GetCRC16(rx_buf,cnt+3)==0)
		{
			for(tmp=0;tmp<cnt;tmp++) ptr[tmp]=rx_buf[tmp+1];
            return 1;
		}
	}
    return 0;
}

void read_xram(unsigned char plc_addr,unsigned short xram_addr,unsigned short cnt)
{
    unsigned short crc_val;
    unsigned char* tx_buf;
    tx_buf = get_can_tx_ptr(3);
    if(tx_buf==0) return;
    tx_buf[0]=plc_addr;
	tx_buf[1]=0xD4;
	tx_buf[2]=xram_addr >> 8;
	tx_buf[3]=xram_addr & 0xFF;
    tx_buf[4]=cnt >> 8;
	tx_buf[5]=cnt & 0xFF;
    crc_val=GetCRC16(tx_buf,6);
    tx_buf[6]=crc_val>>8;
    tx_buf[7]=crc_val&0xFF;
    write_module(8);
}

char test_xram_51(unsigned char* ptr)
{
    unsigned char tmp;
    unsigned char* rx_buf;
	rx_buf = get_can_rx_ptr(3);
	if(rx_buf==0) return 0;
    if(rx_mod_cnt==11)
	{
		if(GetCRC16(rx_buf,11)==0)
		{
			for(tmp=0;tmp<8;tmp++) ptr[tmp]=rx_buf[tmp+1];
            return 1;
		}
	}
    return 0;
}

void read_xram_51(unsigned char plc_addr,unsigned short xram_addr)
{
    unsigned short crc_val;
    unsigned char* tx_buf;
	tx_buf = get_can_tx_ptr(3);
	if(tx_buf==0) return;
    tx_buf[0]=plc_addr;
	tx_buf[1]=0xD4;
	tx_buf[2]=xram_addr >> 8;
	tx_buf[3]=xram_addr & 0xFF;
    crc_val=GetCRC16(tx_buf,4);
    tx_buf[4]=crc_val>>8;
    tx_buf[5]=crc_val&0xFF;
    write_module(6);
}

void can_cmd(request* r)
{
	unsigned short crc_val,tmp;
	unsigned char* tx_buf;
	tx_buf = get_can_tx_ptr(r->canal);
	if(tx_buf==0) return;
	switch(r->cmd)
	{
		case WR_RAM:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0xE4;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			for(tmp=0;tmp < r->amount;tmp++) tx_buf[6+tmp]=r->tx[tmp];
			crc_val=GetCRC16(tx_buf,6+tmp);
			tx_buf[6+tmp]=crc_val>>8;
			tx_buf[7+tmp]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8+tmp);break;
				case 2:write_canal(8+tmp);break;
				case 3:write_module(8+tmp);break;
			}
			break;
		case WR_US:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0xE0;
			tx_buf[2]=r->mem_addr & 0xFF;
			tx_buf[3]=r->amount & 0xFF;
			for(tmp=0;tmp < r->amount;tmp++) tx_buf[4+tmp]=r->tx[tmp];
			crc_val=GetCRC16(tx_buf,4+tmp);
			tx_buf[4+tmp]=crc_val>>8;
			tx_buf[5+tmp]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(6+tmp);break;
				case 2:write_canal(6+tmp);break;
				case 3:write_module(6+tmp);break;
			}
			break;
		case WR_XRAM:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x64;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			for(tmp=0;tmp < r->amount;tmp++) tx_buf[4+tmp]=r->tx[tmp];
			crc_val=GetCRC16(tx_buf,4+tmp);
			tx_buf[4+tmp]=crc_val>>8;
			tx_buf[5+tmp]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(6+tmp);break;
				case 2:write_canal(6+tmp);break;
				case 3:write_module(6+tmp);break;
			}
			break;
		case WR_REG:
			if((r->amount >= 128)||(r->amount == 0)) break;
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x10;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=0;
			tx_buf[5]=r->amount;
			tx_buf[6]=r->amount*2;
			for(tmp=0;tmp<r->amount;tmp++)
			{
				tx_buf[7+tmp*2]=r->tx[tmp*2+1];
				tx_buf[8+tmp*2]=r->tx[tmp*2];
			}
			tx_buf[7+r->amount*2]=getLRC(tx_buf,7+r->amount*2);
			switch(r->canal)
			{
				case 1:
					mstr2=0x10;
					write_canal2(bin_to_ascii(tx_buf,8+r->amount*2,_MODBUS));
					break;
				case 2:
					mstr1=0x10;
					write_canal(bin_to_ascii(tx_buf,8+r->amount*2,_MODBUS));
					break;
			}

			break;
		case WR_HLD:
			if((r->amount >= 128)||(r->amount == 0)) break;
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x10;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=0;
			tx_buf[5]=r->amount;
			tx_buf[6]=r->amount*2;
			for(tmp=0;tmp<r->amount;tmp++)
			{
				tx_buf[7+tmp*2]=r->tx[tmp*2+1];
				tx_buf[8+tmp*2]=r->tx[tmp*2];
			}
			crc_val=GetCRC16(tx_buf,7+r->amount*2);
			tx_buf[7+r->amount*2]=crc_val>>8;
			tx_buf[8+r->amount*2]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(9+r->amount*2);break;
				case 2:write_canal(9+r->amount*2);break;
				case 3:write_module(9+r->amount*2);break;
			}
			break;
		case WR_COIL:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x05;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			if(r->tx[0])
			{
				tx_buf[4]=0xFF;
				tx_buf[5]=0;
			}
			else
			{
				tx_buf[4]=0;
				tx_buf[5]=0;
			}

			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
		case RD_RAM:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0xD4;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
		case RD_US:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0xD0;
			tx_buf[2]=r->mem_addr & 0xFF;
			tx_buf[3]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,4);
			tx_buf[4]=crc_val>>8;
			tx_buf[5]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(6);break;
				case 2:write_canal(6);break;
				case 3:write_module(6);break;
			}
			break;
		case RD_US_ASCII:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0xD0;
			tx_buf[2]=r->mem_addr & 0xFF;
			tx_buf[3]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,4);
			tx_buf[4]=crc_val>>8;
			tx_buf[5]=crc_val&0xFF;
			tmp=bin_to_ascii(tx_buf,6,_RELKON);tx_buf[0]='$';
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(tmp);break;
				case 2:write_canal(tmp);break;
				case 3:write_module(tmp);break;
			}
			break;
		case RD_XRAM:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x54;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			crc_val=GetCRC16(tx_buf,4);
			tx_buf[4]=crc_val>>8;
			tx_buf[5]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(6);break;
				case 2:write_canal(6);break;
				case 3:write_module(6);break;
			}
			break;
		case RD_HLD:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x03;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
		case RD_INP:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x04;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
		case RD_DINP:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x02;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
		case RD_COILS:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0x01;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
		case RD_IO:
			tx_buf[0]=r->plc_addr;
			tx_buf[1]=0xB0;
			tx_buf[2]=r->mem_addr >> 8;
			tx_buf[3]=r->mem_addr & 0xFF;
			tx_buf[4]=r->amount >> 8;
			tx_buf[5]=r->amount & 0xFF;
			crc_val=GetCRC16(tx_buf,6);
			tx_buf[6]=crc_val>>8;
			tx_buf[7]=crc_val&0xFF;
			clear_rx_cnt(r->canal);
			switch(r->canal)
			{
				case 1:write_canal2(8);break;
				case 2:write_canal(8);break;
				case 3:write_module(8);break;
			}
			break;
        case RD_XRAM51:
            tx_buf[0]=r->plc_addr;
            tx_buf[1]=0x54;
            tx_buf[2]=r->mem_addr >> 8;
            tx_buf[3]=r->mem_addr & 0xFF;
            crc_val=GetCRC16(tx_buf,4);
            tx_buf[4]=crc_val>>8;
            tx_buf[5]=crc_val&0xFF;
            clear_rx_cnt(r->canal);
            switch(r->canal)
			{
				case 1:write_canal2(6);break;
				case 2:write_canal(6);break;
				case 3:write_module(6);break;
			}
            break;
        case RD_RAM51:
            tx_buf[0]=r->plc_addr;
            tx_buf[1]=0x50;
            tx_buf[2]=r->mem_addr;
            crc_val=GetCRC16(tx_buf,3);
            tx_buf[3]=crc_val>>8;
            tx_buf[4]=crc_val&0xFF;
            clear_rx_cnt(r->canal);
            switch(r->canal)
			{
				case 1:write_canal2(5);break;
				case 2:write_canal(5);break;
				case 3:write_module(5);break;
			}
            break;
        case RD_XRAM51_ASCII:
            tx_buf[0]=r->plc_addr;
            tx_buf[1]=0x54;
            tx_buf[2]=r->mem_addr >> 8;
            tx_buf[3]=r->mem_addr & 0xFF;
            crc_val=GetCRC16(tx_buf,4);
            tx_buf[4]=crc_val>>8;
            tx_buf[5]=crc_val&0xFF;
            tmp=bin_to_ascii(tx_buf,6,_RELKON);tx_buf[0]='$';
            clear_rx_cnt(r->canal);
            switch(r->canal)
			{
				case 1:write_canal2(tmp);break;
				case 2:write_canal(tmp);break;
				case 3:write_module(tmp);break;
			}
            break;
        case RD_RAM51_ASCII:
            tx_buf[0]=r->plc_addr;
            tx_buf[1]=0x50;
            tx_buf[2]=r->mem_addr;
            crc_val=GetCRC16(tx_buf,3);
            tx_buf[3]=crc_val>>8;
            tx_buf[4]=crc_val&0xFF;
            tmp=bin_to_ascii(tx_buf,5,_RELKON);tx_buf[0]='$';
            clear_rx_cnt(r->canal);
            switch(r->canal)
			{
				case 1:write_canal2(tmp);break;
				case 2:write_canal(tmp);break;
				case 3:write_module(tmp);break;
			}
            break;
	}
}

char can_check(request* r)
{
    unsigned short tmp=0;
	unsigned char* rx_buf;
	rx_buf = get_can_rx_ptr(r->canal);
	if(rx_buf==0) return -1;
	switch(r->cmd)
	{
		case RD_RAM:
			if(get_rx_cnt(r->canal)==1+r->amount+2)
			{
				if(GetCRC16(rx_buf,3+r->amount)==0)	{r->rx = &rx_buf[1];return 1;}
			}
			break;
		case RD_US:
			if(get_rx_cnt(r->canal)==1+r->amount+2)
			{
				if(GetCRC16(rx_buf,3+r->amount)==0)	{r->rx = &rx_buf[1];return 1;}
			}
			break;
		case RD_US_ASCII:
			if(get_rx_cnt(r->canal)==1+(1+r->amount+2)*2+1)
			{
				if(rx_buf[get_rx_cnt(r->canal)-1]==0x0D)
                {
                    if(rx_buf[0]!='!') break;
                    tmp=convert_to_bin(1,get_rx_cnt(r->canal)-2,rx_buf);
                    if(tmp)
                    {
                        if(GetCRC16(rx_buf,tmp)) tmp=0;
                        if(tmp)
                        {
                            r->rx = &rx_buf[1];
                            return 1;
                        }
                    }
                }
			}
			break;
		case RD_XRAM:
			if(get_rx_cnt(r->canal)==1+8+2)
			{
				if(GetCRC16(rx_buf,11)==0) {r->rx = &rx_buf[1];return 1;}
			}
			break;
		case RD_IO:
			if(get_rx_cnt(r->canal)==1+r->amount+2)
			{
				if(GetCRC16(rx_buf,3+r->amount)==0)	{r->rx = &rx_buf[1];return 1;}
			}
			break;
		case RD_HLD:
			if(get_rx_cnt(r->canal)==3+r->amount*2+2)
			{
				if(GetCRC16(rx_buf,3+r->amount*2+2)==0)	{r->rx = &rx_buf[3];return 1;}
			}
			break;
		case RD_INP:
			if(get_rx_cnt(r->canal)==3+r->amount*2+2)
			{
				if(GetCRC16(rx_buf,3+r->amount*2+2)==0)	{r->rx = &rx_buf[3];return 1;}
			}
			break;
		case RD_DINP:
			if(get_rx_cnt(r->canal)>=3+(r->amount>>3)+2)
			{
				if(GetCRC16(rx_buf,get_rx_cnt(r->canal))==0)	{r->rx = &rx_buf[3];return 1;}
			}
			break;
		case RD_COILS:
			if(get_rx_cnt(r->canal)>=3+(r->amount>>3)+2)
			{
				if(GetCRC16(rx_buf,get_rx_cnt(r->canal))==0)	{r->rx = &rx_buf[3];return 1;}
			}
			break;
        case RD_XRAM51:
            if(get_rx_cnt(r->canal)==1+8+2)
			{
				if(GetCRC16(rx_buf,get_rx_cnt(r->canal))==0)	{r->rx = &rx_buf[1];return 1;}
			}
            break;
        case RD_RAM51:
            if(get_rx_cnt(r->canal)==1+8+2)
			{
				if(GetCRC16(rx_buf,get_rx_cnt(r->canal))==0)	{r->rx = &rx_buf[1];return 1;}
			}
            break;
        case RD_XRAM51_ASCII:
            if(get_rx_cnt(r->canal)==1+(1+8+2)*2+1)
			{
				if(rx_buf[get_rx_cnt(r->canal)-1]==0x0D)
                {
                    if(rx_buf[0]!='!') break;
                    tmp=convert_to_bin(1,get_rx_cnt(r->canal)-2,rx_buf);
                    if(tmp)
                    {
                        if(GetCRC16(rx_buf,tmp)) tmp=0;
                        if(tmp)
                        {
                            r->rx = &rx_buf[1];
                            return 1;
                        }
                    }
                }
			}
            break;
        case RD_RAM51_ASCII:
            if(get_rx_cnt(r->canal)==1+(1+8+2)*2+1)
			{
				if(rx_buf[get_rx_cnt(r->canal)-1]==0x0D)
                {
                    if(rx_buf[0]!='!') break;
                    tmp=convert_to_bin(1,get_rx_cnt(r->canal)-2,rx_buf);
                    if(tmp)
                    {
                        if(GetCRC16(rx_buf,tmp)) tmp=0;
                        if(tmp)
                        {
                            r->rx = &rx_buf[1];
                            return 1;
                        }
                    }
                }
			}
            break;
	}
	return 0;
}

void set_disp_num(unsigned char id)
{
	unsigned char n100,n10,tmp;
	for(tmp=0;tmp<20;tmp++) obj_name[tmp]=' ';
	n100 = id/100;id-=n100*100;n10=id/10;id-=n10*10;
	obj_name[0]=n100 + '0';
	obj_name[1]=n10+'0';
	obj_name[2]=id+'0';
}

unsigned char get_disp_num(void)
{
	unsigned char n;
	n = (obj_name[0]-'0')*100 + (obj_name[1]-'0')*10 + obj_name[2];
	return n;
}

void write_reg(unsigned char can_num, unsigned char net_adr,unsigned char adr_h,unsigned char adr_l,unsigned char* ptr,unsigned char cnt)
{
	unsigned char* tx_buf;
	unsigned short tmp;
    if((cnt>=128)||(cnt==0)) return;
    tx_buf = get_can_tx_ptr(can_num);
	tx_buf[0]=net_adr;
	tx_buf[1]=0x10;
	tx_buf[2]=adr_h;
	tx_buf[3]=adr_l;
	tx_buf[4]=0;
	tx_buf[5]=cnt;
	tx_buf[6]=cnt*2;
	for(tmp=0;tmp<cnt;tmp++)
	{
		tx_buf[7+tmp*2]=ptr[tmp*2+1];
		tx_buf[8+tmp*2]=ptr[tmp*2];
	}
	tx_buf[7+cnt*2]=getLRC(tx_buf,7+cnt*2);
	if(can_num==1)
	{
		mstr2=0x10;
		if(_Sys.Can2_Type==0) write_canal2(8+cnt*2);
		else write_canal2(bin_to_ascii(tx_buf,8+cnt*2,_MODBUS));
	}
	else
	{
		mstr1=0x10;
		if(_Sys.Can1_Type==0) write_canal(8+cnt*2);
		else write_canal(bin_to_ascii(tx_buf,8+cnt*2,_MODBUS));
	}
}

static unsigned short bin_to_ascii(unsigned char* ptr,unsigned short lng,unsigned char mode)
{
    unsigned short  tmp;
    tmp=lng;
    while(lng){ptr[((lng-1)<<1)+1]=ascii_code[ptr[lng-1]>>4];ptr[((lng-1)<<1)+2]=ascii_code[ptr[lng-1]&0x0F];lng--;}
    if(mode==_RELKON){ptr[0]='!';ptr[((tmp-1)<<1)+3]=0x0D;return ((tmp<<1)+2);}
    ptr[0]=':';ptr[((tmp-1)<<1)+3]=0x0D;ptr[((tmp-1)<<1)+4]=0x0A;
    return ((tmp<<1)+3);
}

static unsigned short convert_to_bin(unsigned short start_pos,unsigned short end_pos,unsigned char* ptr)
{
    unsigned short lng,tmp;
    char code_h,code_l;
    lng=end_pos-start_pos+1;
    if(lng & 0x01) return 0;
    lng=lng>>1;
    for(tmp=0;tmp<lng;tmp++)
    {
        code_h=check_ascii(ptr[start_pos+(tmp<<1)]);
        code_l=check_ascii(ptr[start_pos+(tmp<<1)+1]);
        if((code_h<0)||(code_l<0)) return 0;
        ptr[tmp]=(code_h<<4)|(code_l);
    }
    return lng;
}

static char check_ascii(unsigned char symb)
{
    unsigned char tmp;
    for(tmp=0;tmp<16;tmp++) if(symb==ascii_code[tmp]) return tmp;
    return -1;
}
