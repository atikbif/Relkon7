/*
 * k_ip.c
 *
 *  Created on: Mar 15, 2012
 *      Author: Роман
 */

#include "k_ip.h"
#include "k_arp.h"

static unsigned char ip[4]={192,168,0,15};
static unsigned char mask[4]={255,255,255,0};
static unsigned char gate[4]={192,168,0,1};


static ip_pkt ip_p;
static unsigned char mac[6];
static unsigned short ip_id=0;
static unsigned char gate_fl=0;

extern unsigned short eth_pkt_cnt;

void set_gate_cmd(void)
{
	gate_fl=1;
}

void clr_gate_cmd(void)
{
	gate_fl=0;
}

void set_mask(unsigned char* new_m)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) mask[tmp]=new_m[tmp];
}

void set_gate(unsigned char* new_gate)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) gate[tmp]=new_gate[tmp];
}

void get_mask(unsigned char* m)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) m[tmp]=mask[tmp];
}

void get_gate(unsigned char* g)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) g[tmp]=gate[tmp];
}

void get_ip(unsigned char* ip_data)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) ip_data[tmp]=ip[tmp];
}

void set_ip(unsigned char* ip_data)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) ip[tmp]=ip_data[tmp];
}

ip_pkt* ip_rcv(mac_pkt* pkt)
{
	unsigned short tmp;
	ip_p.id[0]=pkt->buf.ptr[4];
	ip_p.id[1]=pkt->buf.ptr[5];
	ip_p.pr = pkt->buf.ptr[9];
	for(tmp=0;tmp<4;tmp++)
	{
		ip_p.ip_s[tmp]=pkt->buf.ptr[12+tmp];
		ip_p.ip_d[tmp]=pkt->buf.ptr[16+tmp];
	}
	tmp = (pkt->buf.ptr[0] & 0x0F)<<2;
	ip_p.buf.ptr = &pkt->buf.ptr[tmp];
	ip_p.buf.len = (((unsigned short)pkt->buf.ptr[2]<<8) | pkt->buf.ptr[3]) - tmp;//pkt->buf.len - tmp;
	if(ip_p.buf.len>1500) ip_p.buf.len=0;
	if((pkt->buf.ptr[0] >> 4) != 0x04) ip_p.buf.len=0;
	return &ip_p;
}

void send_ip(ip_pkt* pkt)
{
	unsigned char* ptr;
	unsigned char* m;
	unsigned short tmp;
	if(gate_fl==0) {m = get_mac_tab(&pkt->ip_d[0]);} // поиск мак адреса в таблице
	else {m = get_gate_ptr();} // используется мак шлюза
	if(m)//(m)&&(m[0]))
	{
		//eth_pkt_cnt++;
		ptr = get_mac_tx();
		get_mac(mac);
		for(tmp=0;tmp<6;tmp++)
		{
			ptr[tmp]=m[tmp];
			ptr[tmp+6]=mac[tmp];
		}
		ptr[12]=0x08;ptr[13]=0x00;
		ptr[14]=0x45;
		ptr[15]=0x00;
		tmp = 20 + pkt->buf.len;
		ptr[16]=tmp>>8;ptr[17]=tmp&0xFF;
		//ptr[18]=pkt->id[0];ptr[19]=pkt->id[1];
		ptr[18]=ip_id>>8;ptr[19]=ip_id&0xFF;ip_id++;
		ptr[20]=0x00;//0x40;
		ptr[21]=0x00;
		ptr[22]=0x40;//0x80;
		ptr[23]=pkt->pr;
		ptr[24]=0x00;ptr[25]=0x00;
		get_ip(ip);
		for(tmp=0;tmp<4;tmp++)
		{
			ptr[26+tmp]=ip[tmp];
			ptr[30+tmp]=pkt->ip_d[tmp];
		}
		for(tmp=0;tmp<pkt->buf.len;tmp++) ptr[34+tmp] = pkt->buf.ptr[tmp];
		send_mac_data(14+20+pkt->buf.len);
	}
}

unsigned char check_ip_req(ip_pkt* pkt)
{
	unsigned char tmp,res=1;
	if(pkt->buf.len == 0) res=0;
	for(tmp=0;tmp<4;tmp++) if(pkt->ip_d[tmp]!=ip[tmp]) {res=0;break;}
	return res;
}

unsigned char get_ip_type(ip_pkt* pkt)
{
	if(pkt->pr == 0x01) return ICMP_TYPE;
	if(pkt->pr == 0x11) return UDP_TYPE;
	if(pkt->pr == 0x06) return TCP_TYPE;
	return UNDEFIP_TYPE;
}

unsigned char check_mask(unsigned char* ips)
{
	unsigned char res=1;
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++)
	{
		if((ip[tmp] & mask[tmp]) != (ips[tmp] & mask[tmp])) {res=0;break;}
	}
	return res;
}
