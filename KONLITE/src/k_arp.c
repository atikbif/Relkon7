/*
 * k_arp.c
 *
 *  Created on: Mar 15, 2012
 *      Author: Роман
 */

#include "k_arp.h"
#include "heth.h"
#include "k_ip.h"
#include "k_mac.h"


static struct
{
	unsigned char ip[4];
	unsigned char mac[6];
	unsigned char stat;
}arp_tab[ARP_TAB_SIZE];

static unsigned char arp_index=0;

static arp_pkt arp_p,arp_send;
static unsigned char ip[4];
static unsigned char mac[6];
static unsigned char gate_mac[6]={0,0,0,0,0,0};
static unsigned char gate_ip[4]={0,0,0,0};

void init_arp_tab(void)
{
	unsigned short tmp,i;
	for(tmp=0;tmp<ARP_TAB_SIZE;tmp++)
	{
		arp_tab[tmp].stat=0;
		for(i=0;i<4;i++) arp_tab[tmp].ip[i]=0;
	}
	arp_index=0;
}

unsigned char* get_mac_tab(unsigned char* ip_req)
{
	unsigned short tmp,i,res;
	for(tmp=0;tmp<ARP_TAB_SIZE;tmp++)
	{
		if(arp_tab[tmp].stat)
		{
			res=1;for(i=0;i<4;i++) if(arp_tab[tmp].ip[i]!=ip_req[i]) {res=0;}
			if(res) return arp_tab[tmp].mac;
		}
	}
	return 0;
}

void add_arp_tab(unsigned char* ip_tab,unsigned char* mac_tab)
{
	unsigned char* ptr;
	unsigned short tmp;
	ptr = get_mac_tab(ip_tab);
	if(ptr==0)
	{
		for(tmp=0;tmp<4;tmp++) arp_tab[arp_index].ip[tmp]=ip_tab[tmp];
		for(tmp=0;tmp<6;tmp++) arp_tab[arp_index].mac[tmp]=mac_tab[tmp];
		arp_tab[arp_index].stat=1;
		arp_index++;if(arp_index>=ARP_TAB_SIZE) arp_index=0;
	}
	else
	{
		for(tmp=0;tmp<6;tmp++) ptr[tmp]=mac_tab[tmp];
	}
}

arp_pkt* arp_rcv(mac_pkt* pkt)
{
	unsigned short tmp;
	for(tmp=0;tmp<6;tmp++)
	{
		arp_p.mac_d[tmp] = pkt->mac_d[tmp];
		arp_p.mac_s[tmp] = pkt->mac_s[tmp];
	}
	for(tmp=0;tmp<4;tmp++)
	{
		arp_p.ip_d[tmp] = pkt->buf.ptr[24+tmp];
		arp_p.ip_s[tmp] = pkt->buf.ptr[14+tmp];
	}
	arp_p.oper[0] =  pkt->buf.ptr[6];
	arp_p.oper[1] =  pkt->buf.ptr[7];
	return &arp_p;
}

unsigned char check_arp_req(arp_pkt* pkt)
{
	unsigned char tmp,res=1;
	if((pkt->oper[0]==0x00)&&(pkt->oper[1]==0x01))
	{
		get_ip(ip);
		for(tmp=0;tmp<4;tmp++) if(pkt->ip_d[tmp]!=ip[tmp]) {res=0;break;}
	}else res=0;
	return res;
}

unsigned char check_arp_answer_gate(arp_pkt* pkt)
{
	unsigned char tmp,res=1;
	if((pkt->oper[0]==0x00)&&(pkt->oper[1]==0x02))
	{
		get_gate((unsigned char*)gate_ip);
		for(tmp=0;tmp<4;tmp++) if(pkt->ip_s[tmp]!=gate_ip[tmp]) {res=0;break;}
	}else res=0;
	return res;
}

void send_arp(arp_pkt* pkt)
{
	unsigned char* ptr;
	unsigned short tmp;
	unsigned char broadcast=1;
	get_ip(ip);
	get_mac(mac);
	ptr = get_mac_tx();
	for(tmp=0;tmp<6;tmp++)
	{
		if(pkt->mac_d[tmp] != 0xFF) broadcast = 0;
		ptr[tmp]=pkt->mac_d[tmp];
		ptr[tmp+6]=mac[tmp];
	}
	ptr[12]=0x08;ptr[13]=0x06;
	ptr[14]=0x00;ptr[15]=0x01;
	ptr[16]=0x08;ptr[17]=0x00;
	ptr[18]=0x06;ptr[19]=0x04;
	ptr[20]=pkt->oper[0];ptr[21]=pkt->oper[1];
	for(tmp=0;tmp<6;tmp++)
	{
		ptr[22+tmp]=mac[tmp];
		if(broadcast==0) ptr[32+tmp]=pkt->mac_d[tmp]; else ptr[32+tmp]=0;
	}
	for(tmp=0;tmp<4;tmp++)
	{
		ptr[28+tmp]=ip[tmp];
		ptr[38+tmp]=pkt->ip_d[tmp];
	}
	send_mac_data(42);
}

void arp_answer(arp_pkt* pkt)
{
	unsigned short tmp;
	for(tmp=0;tmp<6;tmp++) pkt->mac_d[tmp]=pkt->mac_s[tmp];
	for(tmp=0;tmp<4;tmp++) pkt->ip_d[tmp]=pkt->ip_s[tmp];
	pkt->oper[0]=0x00;pkt->oper[1]=0x02;
	send_arp(pkt);
}

void set_gate_mac(unsigned char* m)
{
	unsigned short tmp;
	for(tmp=0;tmp<6;tmp++)
	{
		gate_mac[tmp]=m[tmp];
	}
}

void get_gate_mac(void)
{
	unsigned short tmp;
	get_gate((unsigned char*)gate_ip);	// проверка ip шлюза
	if(gate_ip[0])		// шлюз настроен
	{
		for(tmp=0;tmp<6;tmp++) arp_send.mac_d[tmp]=0xFF;
		for(tmp=0;tmp<4;tmp++) arp_send.ip_d[tmp]=gate_ip[tmp];
		arp_send.oper[0]=0x00;arp_send.oper[1]=0x01; // запрос мак адреса
		send_arp(&arp_send);
	}
}

unsigned char* get_gate_ptr(void)
{
	return (unsigned char*)gate_mac;
}
