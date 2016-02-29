/*
 * k_mac.c
 *
 *  Created on: Mar 15, 2012
 *      Author: Роман
 */

#include "k_mac.h"
#include "heth.h"

static unsigned char mac[6]={0x15,0xda,0xe9,0xf5,0x8e,0xc2};

static mac_pkt m_pkt;

void get_mac(unsigned char* m)
{
	unsigned char tmp;
	for(tmp=0;tmp<6;tmp++) m[tmp]=mac[tmp];
}

void set_mac(unsigned char* m)
{
	unsigned char tmp;
	for(tmp=0;tmp<6;tmp++) mac[tmp]=m[tmp];
}

mac_pkt* get_mac_pkt(eth_buf* buf)
{
	unsigned short tmp;
	for(tmp=0;tmp<6;tmp++)
	{
		m_pkt.mac_d[tmp] = buf->ptr[tmp];
		m_pkt.mac_s[tmp] = buf->ptr[6+tmp];
	}
	m_pkt.buf.ptr = &buf->ptr[12+2];
	m_pkt.type[0] = buf->ptr[12];
	m_pkt.type[1] = buf->ptr[13];
	m_pkt.buf.len = buf->len - (12+2) - 4;
	return &m_pkt;
}

unsigned char check_mac(mac_pkt* pkt)
{
	unsigned short tmp;
	unsigned char res = 1,MACi;
	if(pkt->buf.len == 0) res=0;
	for(tmp=0;tmp<6;tmp++)
	{
		MACi=pkt->mac_d[tmp];
		if((MACi != mac[tmp])&&(MACi != 0xFF)) res=0;
	}
	return res;
}

unsigned char get_eth_type(mac_pkt* pkt)
{
	if((pkt->type[0]==0x08)&&(pkt->type[1]==0x00)) return IP_TYPE;
	if((pkt->type[0]==0x08)&&(pkt->type[1]==0x06)) return ARP_TYPE;
	return UNDEF_TYPE;
}
