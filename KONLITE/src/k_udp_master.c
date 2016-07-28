#include "k_udp_master.h"
#include "k_arp.h"

static arp_pkt arp_send;
static unsigned char udp_data[512];
static unsigned char ip_data[1024];
static udp_request* sent_req[ID_LIST_CNT];

static unsigned char id_cnt = 0;
static udp_pkt pkt1;
static ip_pkt pkt2;
static ip_pkt* buf_pkt;

extern unsigned char eth_stat;

void mac_request(unsigned char *ip)
{
	unsigned char tmp;
	for(tmp=0;tmp<6;tmp++) arp_send.mac_d[tmp]=0xFF;
	for(tmp=0;tmp<4;tmp++) arp_send.ip_d[tmp]=ip[tmp];
	arp_send.oper[0]=0x00;arp_send.oper[1]=0x01; // запрос мак адреса
	send_arp(&arp_send);
}
void udp_cmd(udp_request *req)
{
	unsigned short tmp;
	if(eth_stat==0) return;
	if(get_mac_tab(req->ip)==0) {
		mac_request(req->ip);
	}else {
		switch(req->cmd) {
			case UDP_RD_USER:
				udp_data[0] = 0x00;
				udp_data[1] = 0xD0;
				udp_data[2] = req->addr & 0xFF;
				udp_data[3] = req->cnt & 0xFF;
				
				pkt1.p_s[0] = pkt1.p_d[0] = 12144 >> 8;
				pkt1.p_s[1] = pkt1.p_d[1] = 12144 & 0xFF;
				pkt1.buf.ptr = udp_data;
				pkt1.buf.len = 4;
				
				pkt2.buf.ptr = ip_data;
				get_ip(&pkt2.ip_s[0]);
				pkt2.ip_d[0] = req->ip[0];
				pkt2.ip_d[1] = req->ip[1];
				pkt2.ip_d[2] = req->ip[2];
				pkt2.ip_d[3] = req->ip[3];
				send_udp_to_buf(&pkt1, &pkt2);
				tmp = get_id();
				set_id(tmp+1);
				tmp+=1000;
				req->id = tmp;
				pkt2.id[0] = tmp >> 8;
				pkt2.id[1] = tmp & 0xFF;
				req->result = UDP_WAIT_ANSWER;
				add_id(req);
				buf_pkt = &pkt2;
				break;
			case UDP_WR_USER:
				if(req->tx==0) break;
				udp_data[0] = 0x00;
				udp_data[1] = 0xE0;
				udp_data[2] = req->addr & 0xFF;
				udp_data[3] = req->cnt & 0xFF;
				for(tmp=0;tmp < ((req->cnt)&0xFF);tmp++) {
					udp_data[4+tmp] = req->tx[tmp];
				}
				
				pkt1.p_s[0] = pkt1.p_d[0] = 12144 >> 8;
				pkt1.p_s[1] = pkt1.p_d[1] = 12144 & 0xFF;
				pkt1.buf.ptr = udp_data;
				pkt1.buf.len = 4+((req->cnt)&0xFF);
				
				pkt2.buf.ptr = ip_data;
				get_ip(&pkt2.ip_s[0]);
				pkt2.ip_d[0] = req->ip[0];
				pkt2.ip_d[1] = req->ip[1];
				pkt2.ip_d[2] = req->ip[2];
				pkt2.ip_d[3] = req->ip[3];
				send_udp_to_buf(&pkt1, &pkt2);
				tmp = get_id();
				set_id(tmp+1);
				tmp+=1000;
				req->id = tmp;
				pkt2.id[0] = tmp >> 8;
				pkt2.id[1] = tmp & 0xFF;
				req->result = UDP_WAIT_ANSWER;
				add_id(req);
				buf_pkt = &pkt2;
				break;
			default:
				break;
		}
	}
}

void id_list_init(void)
{
	unsigned char tmp;
	for(tmp=0;tmp<ID_LIST_CNT;tmp++) {
		sent_req[tmp] = 0;
	}
	clear_buf_ip_pkt();
}

void add_id(udp_request *r)
{
	sent_req[id_cnt] = r;
	id_cnt++;
	if(id_cnt>=ID_LIST_CNT) id_cnt=0;
}

udp_request* get_req_by_id(unsigned short id)
{
	unsigned char tmp;
	if(id==0) return 0;
	for(tmp=0;tmp<ID_LIST_CNT;tmp++) {
		if(sent_req[tmp]==0) continue;
		if(sent_req[tmp]->id==id) return sent_req[tmp];
	}
	return 0;
}

ip_pkt* get_buf_ip_pkt()
{
	return buf_pkt;
}

void clear_buf_ip_pkt()
{
	buf_pkt = 0;
}