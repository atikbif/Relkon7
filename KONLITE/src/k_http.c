/*
 * k_http.c
 *
 *  Created on: Mar 29, 2012
 *      Author: Роман
 */

#include "k_http.h"
#include "string_func.h"
#include "http_data.h"
#include "print.h"

extern unsigned short eth_pkt_cnt;

static unsigned char http_en = 0;
static flist vf[MAX_WEB_FILE_CNT];
static unsigned char vf_cnt=0;
//static unsigned char tmp_str[30];



static const char hdr_data[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-type: text/plain\r\n"
  "Content-Length: ";

static const char hdr_txt[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-type: text/html\r\n"
  "Content-Length: ";

static const char hdr_izo[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-type: image/x-icon\r\n"
  "Content-Length: ";

static const char hdr_js[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-type: application/x-javascript\r\n"
  "Content-Length: ";

static const char hdr_css[] =
  "HTTP/1.1 200 OK\r\n"
  "Content-type: text/css\r\n"
  "Content-Length: ";

flist* get_web_file(tcp_pkt* pkt)
{
	unsigned short tmp,res=0;
	if(vf_cnt==0) return 0;
	for(tmp=0;tmp<vf_cnt;tmp++)
	{
		if(find(vf[tmp].name,pkt->buf.ptr,vf[tmp].namelen+10)) {res=1;break;}
	}
	if(res) return &vf[tmp];
	return &vf[0];
}

unsigned char add_web_file(flist* f)
{
	if(vf_cnt >= MAX_WEB_FILE_CNT) return 0;
	vf[vf_cnt].name = f->name;
	vf[vf_cnt].namelen = f->namelen;
	vf[vf_cnt].type = f->type;
	vf[vf_cnt].fdata = f->fdata;
	vf[vf_cnt].size = f->size;
	vf_cnt++;
	return 1;
}

void send_web_file(tcp_pkt* pkt1, ip_pkt* pkt2, flist* f)
{
	unsigned short tmp,headlen;
	unsigned long size,dyn_ptr=0;
	unsigned char* ptr=0;
	tcp_answer_head(pkt1,pkt2);
	switch(f->type)
	{
		case HTML_TXT:
			for(tmp=0;tmp<sizeof(hdr_txt)-1;tmp++) pkt1->buf.ptr[tmp]=hdr_txt[tmp];
			break;
		case DYN_DATA:
			for(tmp=0;tmp<sizeof(hdr_data)-1;tmp++) pkt1->buf.ptr[tmp]=hdr_data[tmp];
			break;
		case IZO:
			for(tmp=0;tmp<sizeof(hdr_izo)-1;tmp++) pkt1->buf.ptr[tmp]=hdr_izo[tmp];
			break;
		case JS:
			for(tmp=0;tmp<sizeof(hdr_js)-1;tmp++) pkt1->buf.ptr[tmp]=hdr_js[tmp];
			break;
		case CSS:
			for(tmp=0;tmp<sizeof(hdr_css)-1;tmp++) pkt1->buf.ptr[tmp]=hdr_css[tmp];
			break;
		default: return;
	}
	headlen = tmp;
	headlen+=print_long_buf(f->size,&pkt1->buf.ptr[tmp]);
	pkt1->buf.ptr[headlen++] = '\r';pkt1->buf.ptr[headlen++] = '\n';
	pkt1->buf.ptr[headlen++] = '\r';pkt1->buf.ptr[headlen++] = '\n';
	if((f->size + headlen) < 1000)
	{

		for(tmp=0;tmp < f->size;tmp++)
		{
			if(f->type == DYN_DATA) pkt1->buf.ptr[headlen+tmp] = get_webdyn(f,tmp);
			else pkt1->buf.ptr[headlen+tmp] = f->fdata[tmp];
		}
		pkt1->buf.len = f->size + headlen;
		send_tcp(pkt1,pkt2);
	}
	else
	{
		for(tmp=0;tmp < 1000 - headlen;tmp++)
		{
			if(f->type == DYN_DATA) pkt1->buf.ptr[headlen+tmp] = get_webdyn(f,tmp);
			else pkt1->buf.ptr[headlen+tmp] = f->fdata[tmp];
		}
		pkt1->buf.len = 1000;
		send_tcp(pkt1,pkt2);pkt1->n_tr += 1000;
		size = f->size - (1000 - headlen);
		ptr = &f->fdata[tmp];dyn_ptr=tmp;
		while(size>=1000)
		{
			for(tmp=0;tmp < 1000;tmp++)
			{
				if(f->type == DYN_DATA) pkt1->buf.ptr[tmp] = get_webdyn(f,dyn_ptr++);
				else pkt1->buf.ptr[tmp] = ptr[tmp];
			}
			send_tcp(pkt1,pkt2);pkt1->n_tr += 1000;
			if(f->type != DYN_DATA) ptr+=1000;size-=1000;
		}
		if(size)
		{
			pkt1->buf.len = size;
			for(tmp=0;tmp < size;tmp++)
			{
				if(f->type == DYN_DATA) pkt1->buf.ptr[tmp] = get_webdyn(f,dyn_ptr++);
				else pkt1->buf.ptr[tmp] = ptr[tmp];
			}
			send_tcp(pkt1,pkt2);
		}
	}
}

unsigned char check_http_req(tcp_pkt* pkt)
{
	if(find((unsigned char*)"GET",pkt->buf.ptr,pkt->buf.len)) return 1;
	if(find((unsigned char*)"POST",pkt->buf.ptr,pkt->buf.len)) return 2;
	return 0;
}

void http_enable(void)
{
	tcp_listen(80,web_req);
	http_en=1;
}

unsigned char get_http_stat(void)
{
	return http_en;
}

void web_req(tcp_pkt* pkt1,ip_pkt* pkt2)
{
	flist* ptr;
	if(check_http_req(pkt1))
	{
		eth_pkt_cnt++;
		ptr = get_web_file(pkt1);
		if(ptr) send_web_file(pkt1,pkt2,ptr);
	}
}
