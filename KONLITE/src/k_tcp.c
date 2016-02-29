/*
 * k_tcp.c
 *
 *  Created on: Mar 27, 2012
 *      Author: Роман
 */

#include "k_tcp.h"

static struct
{
	unsigned short port;
	unsigned char stat;
	void (*function)(tcp_pkt* pkt1,ip_pkt* pkt2);
}tcp_tab[TCP_PORT_SIZE];

static tcp_pkt tcp_p;
static unsigned long tmp_buf[4];

extern unsigned short eth_pkt_cnt;
volatile unsigned long tcp_tmr=0;

void tcp_init(void)
{
	unsigned short tmp;
	for(tmp=0;tmp<TCP_PORT_SIZE;tmp++)
	{
		tcp_tab[tmp].port=13144;
		tcp_tab[tmp].stat = 0;
	}
}

unsigned char tcp_listen(unsigned short port, void (*func)(tcp_pkt* pkt1,ip_pkt* pkt2))
{
	unsigned short tmp;
	for(tmp=0;tmp<TCP_PORT_SIZE;tmp++)
	{
		if(tcp_tab[tmp].stat==0)
		{
			tcp_tab[tmp].port = port;
			tcp_tab[tmp].function = func;
			tcp_tab[tmp].stat = 1;
			return 1;
		}
	}
	return 0;
}

void porttcp_scan(tcp_pkt* pkt1, ip_pkt* pkt2)
{
	unsigned short tmp,res=0;

	for(tmp=0;tmp<TCP_PORT_SIZE;tmp++)
	{
		if(tcp_tab[tmp].stat){if(tcp_tab[tmp].port == pkt1->p_d) {res=1;break;}}
	}
	if(res==0) return;

	if(pkt1->buf.len==0)
	{
		if(pkt1->fl == (FL_FIN | FL_ACK))
		{
			answer_tcpack(pkt1,pkt2);
			pkt1->fl = FL_FIN | FL_ACK;
			send_tcp(pkt1,pkt2);
			return;
		}
		if(pkt1->fl == FL_SYN)
		{
			answer_syn(pkt1,pkt2);
			return;
		}
		if(pkt1->fl & FL_PSH)
		{
			answer_tcpack(pkt1,pkt2);
			return;
		}
	}
	else {tcp_tab[tmp].function(pkt1,pkt2);}

}

tcp_pkt* tcp_rcv(ip_pkt* pkt)
{
	tcp_p.p_s = (((unsigned short)pkt->buf.ptr[0]) << 8) | pkt->buf.ptr[1];
	tcp_p.p_d = (((unsigned short)pkt->buf.ptr[2]) << 8) | pkt->buf.ptr[3];
	tcp_p.n_tr =  (((unsigned long)pkt->buf.ptr[4]) << 24) | (((unsigned long)pkt->buf.ptr[5]) << 16) | (((unsigned long)pkt->buf.ptr[6]) << 8) | pkt->buf.ptr[7];
	tcp_p.n_rcv =  (((unsigned long)pkt->buf.ptr[8]) << 24) | (((unsigned long)pkt->buf.ptr[9]) << 16) | (((unsigned long)pkt->buf.ptr[10]) << 8) | pkt->buf.ptr[11];
	tcp_p.fl = pkt->buf.ptr[13] & 0x3F;
	tcp_p.window = (((unsigned short)pkt->buf.ptr[14]) << 8) | pkt->buf.ptr[15];
	tcp_p.buf.ptr = &pkt->buf.ptr[(pkt->buf.ptr[12] >> 4)*4];
	tcp_p.buf.len = pkt->buf.len - (pkt->buf.ptr[12] >> 4)*4;
	return &tcp_p;
}

void answer_syn(tcp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) pkt2->ip_d[tmp] = pkt2->ip_s[tmp];
	tmp_buf[0]=pkt1->p_s;
	pkt1->p_s = pkt1->p_d;
	pkt1->p_d = tmp_buf[0];
	tmp_buf[0]=pkt1->n_tr;
	pkt1->n_tr = tcp_tmr;
	pkt1->n_rcv = tmp_buf[0]+1;
	pkt1->fl = FL_ACK | FL_SYN;
	pkt1->buf.len=0;
	send_tcp(pkt1,pkt2);
}

void answer_tcpack(tcp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) pkt2->ip_d[tmp] = pkt2->ip_s[tmp];
	tmp_buf[0]=pkt1->p_s;
	pkt1->p_s = pkt1->p_d;
	pkt1->p_d = tmp_buf[0];
	tmp_buf[0]=pkt1->n_tr;
	pkt1->n_tr = pkt1->n_rcv;
	pkt1->n_rcv = tmp_buf[0]+1;
	pkt1->fl = FL_ACK;
	pkt1->buf.len=0;
	send_tcp(pkt1,pkt2);
}

void tcp_answer_head(tcp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned char tmp;
	for(tmp=0;tmp<4;tmp++) pkt2->ip_d[tmp] = pkt2->ip_s[tmp];
	tmp_buf[0]=pkt1->p_s;
	pkt1->p_s = pkt1->p_d;
	pkt1->p_d = tmp_buf[0];
	tmp_buf[0]=pkt1->n_tr;
	pkt1->n_tr = pkt1->n_rcv;
	pkt1->n_rcv = tmp_buf[0]+pkt1->buf.len;
	pkt1->fl = FL_ACK | FL_PSH;
}

void send_tcp(tcp_pkt* pkt1,ip_pkt* pkt2)
{
	unsigned short tmp;
	pkt2->buf.len = pkt1->buf.len+20;
	pkt2->pr = 0x06;
	pkt2->buf.ptr[0]=pkt1->p_s >> 8;
	pkt2->buf.ptr[1]=pkt1->p_s & 0xFF;
	pkt2->buf.ptr[2]=pkt1->p_d >> 8;
	pkt2->buf.ptr[3]=pkt1->p_d & 0xFF;
	pkt2->buf.ptr[4]=(pkt1->n_tr >> 24) & 0xFF;
	pkt2->buf.ptr[5]=(pkt1->n_tr >> 16) & 0xFF;
	pkt2->buf.ptr[6]=(pkt1->n_tr >> 8) & 0xFF;
	pkt2->buf.ptr[7]=(pkt1->n_tr) & 0xFF;
	pkt2->buf.ptr[8]=(pkt1->n_rcv >> 24) & 0xFF;
	pkt2->buf.ptr[9]=(pkt1->n_rcv >> 16) & 0xFF;
	pkt2->buf.ptr[10]=(pkt1->n_rcv >> 8) & 0xFF;
	pkt2->buf.ptr[11]=(pkt1->n_rcv) & 0xFF;
	pkt2->buf.ptr[12]=0x50;
	pkt2->buf.ptr[13]=pkt1->fl;
	pkt2->buf.ptr[14]=0x04; pkt2->buf.ptr[15]=0x00;
	pkt2->buf.ptr[16]=0x00;pkt2->buf.ptr[17]=0x00;
	pkt2->buf.ptr[18]=0x00;pkt2->buf.ptr[19]=0x00;
	for(tmp=0;tmp<pkt1->buf.len;tmp++) pkt2->buf.ptr[20+tmp]=pkt1->buf.ptr[tmp];
	send_ip(pkt2);
}
