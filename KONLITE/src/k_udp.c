/*
 * k_udp.c
 *
 *  Created on: Mar 15, 2012
 *      Author: Роман
 */

#include "k_udp.h"

static struct
{
	unsigned short port;
	unsigned char stat;
	void (*function)(udp_pkt* pkt1,ip_pkt* pkt2);
}udp_tab[UDP_PORT_SIZE];

static udp_pkt udp_p;

extern unsigned short eth_pkt_cnt;

void udp_init(void)
{
	unsigned short tmp;
	for(tmp=0;tmp<UDP_PORT_SIZE;tmp++)
	{
		udp_tab[tmp].port=12144;
		udp_tab[tmp].stat = 0;
	}
}

udp_pkt* udp_rcv(ip_pkt* pkt)
{
	udp_p.p_s[0]=pkt->buf.ptr[0];
	udp_p.p_s[1]=pkt->buf.ptr[1];
	udp_p.p_d[0]=pkt->buf.ptr[2];
	udp_p.p_d[1]=pkt->buf.ptr[3];
	udp_p.buf.ptr = &pkt->buf.ptr[8];
	udp_p.buf.len = (((unsigned short)pkt->buf.ptr[4]) << 8) | pkt->buf.ptr[5];
	return &udp_p;
}

unsigned char udp_listen(unsigned short port, void (*func)(udp_pkt* pkt1,ip_pkt* pkt2))
{
	unsigned short tmp;
	for(tmp=0;tmp<UDP_PORT_SIZE;tmp++)
	{
		if(udp_tab[tmp].stat==0)
		{
			udp_tab[tmp].port = port;
			udp_tab[tmp].function = func;
			udp_tab[tmp].stat = 1;
			return 1;
		}
	}
	return 0;
}

void portudp_scan(udp_pkt* pkt1, ip_pkt* pkt2)
{
	unsigned short tmp;
	for(tmp=0;tmp<UDP_PORT_SIZE;tmp++)
	{
		if(udp_tab[tmp].stat)
		{
			if(udp_tab[tmp].port == ((((unsigned short)pkt1->p_d[0]) << 8) | (pkt1->p_d[1])))
			{
				udp_tab[tmp].function(pkt1,pkt2);
			}
		}
	}
}

void udp_answer_head(udp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned short tmp;
	unsigned char p_h,p_l;
	for(tmp=0;tmp<4;tmp++) pkt2->ip_d[tmp] = pkt2->ip_s[tmp];
	p_h = pkt1->p_s[0];p_l = pkt1->p_s[1];
	pkt1->p_s[0]=pkt1->p_d[0];pkt1->p_s[1]=pkt1->p_d[1];
	pkt1->p_d[0] = p_h; pkt1->p_d[1] = p_l;
}

void send_udp(udp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned short tmp;
	pkt2->buf.len = pkt1->buf.len+8;
	pkt2->pr = 0x11;
	pkt2->buf.ptr[0]=pkt1->p_s[0];
	pkt2->buf.ptr[1]=pkt1->p_s[1];
	pkt2->buf.ptr[2]=pkt1->p_d[0];
	pkt2->buf.ptr[3]=pkt1->p_d[1];
	pkt2->buf.ptr[4]=(pkt1->buf.len+8) >> 8;
	pkt2->buf.ptr[5]=(pkt1->buf.len+8) & 0xFF;
	pkt2->buf.ptr[6]=0;
	pkt2->buf.ptr[7]=0;
	for(tmp=0;tmp<pkt1->buf.len;tmp++) pkt2->buf.ptr[8+tmp]=pkt1->buf.ptr[tmp];
	send_ip(pkt2);
}
