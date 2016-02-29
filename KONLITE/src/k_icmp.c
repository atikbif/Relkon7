/*
 * k_icmp.c
 *
 *  Created on: Mar 15, 2012
 *      Author: Роман
 */

#include "k_icmp.h"

static icmp_pkt icmp_p;

icmp_pkt* icmp_rcv(ip_pkt* pkt)
{
	icmp_p.type = pkt->buf.ptr[0];
	icmp_p.code = pkt->buf.ptr[1];
	icmp_p.id[0] = pkt->buf.ptr[4];icmp_p.id[1] = pkt->buf.ptr[5];
	icmp_p.num[0] = pkt->buf.ptr[6];icmp_p.num[1] = pkt->buf.ptr[7];
	icmp_p.buf.ptr=&pkt->buf.ptr[8];
	icmp_p.buf.len = pkt->buf.len - 8;
	return &icmp_p;
}

unsigned char check_ping(icmp_pkt* pkt)
{
	if(pkt->type == 0x08) return 1;
	return 0;
}

void ping_echo(icmp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned short tmp;
	for(tmp=0;tmp<4;tmp++) pkt2->ip_d[tmp] = pkt2->ip_s[tmp];
	pkt2->pr = 0x01;
	pkt2->buf.ptr[0]=0x00;
	pkt2->buf.ptr[1]=0x00;
	pkt2->buf.ptr[2]=0x00;
	pkt2->buf.ptr[3]=0x00;
	pkt2->buf.ptr[4]=pkt1->id[0];pkt2->buf.ptr[5]=pkt1->id[1];
	pkt2->buf.ptr[6]=pkt1->num[0];pkt2->buf.ptr[7]=pkt1->num[1];
	for(tmp=0;tmp < pkt1->buf.len;tmp++) pkt2->buf.ptr[8+tmp]=pkt1->buf.ptr[tmp];
	pkt2->buf.len= 8 + pkt1->buf.len;
	send_ip(pkt2);
}
