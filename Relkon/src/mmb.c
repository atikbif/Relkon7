/*
 * mmb.c
 *
 *  Relkon ver 1.0
 *  Author: –оман
 *
 */

#include "mmb.h"
#include "hinout.h"
#include "crc.h"

extern const unsigned char mod_table[];
extern volatile unsigned char emu_mode;

extern unsigned char tx_mod_buf[MOD_BUF_SIZE];
extern unsigned char rx_mod_buf[MOD_BUF_SIZE];
extern volatile unsigned short rx_mod_cnt;
extern unsigned short tx_mod_cnt;

unsigned long fl_in=0,fl_out=0,fl_adc=0,fl_dac=0;	// флаги дл€ системной информации на пульте

volatile unsigned char err_mod[256];
static unsigned char err_mod_num=0;

static unsigned char step=2;
static unsigned char mod_num=0;
static unsigned char cur_crc;

unsigned char   IN[32];
unsigned char   OUT[32];

unsigned int Sum_err=0;

mmb_ain _ADC;
mmb_dac _DAC;

static volatile unsigned short temp,base=0,rx_base,rx_pos;
extern volatile unsigned char Tx_end;

void mmb_work(void)
{
	switch(step)
	{
		case 0:// разбор ответа от модулей
			// base - стартовый индекс группы модулей дл€ запроса
			// rx_base - сохранЄнный индекс начала группы дл€ разбора при приЄме ответов
			// mod_num - адрес модул€
			// temp - счЄтчик внутри группы
			// Tx_end - флаг завершени€ передачи данных
			// rx_pos - текущий индекс в приЄмном буфере

			if((Tx_end)&&(get_mmb_tmr() > ANSWER_PAUSE)) // если трансл€ци€ завершена и истЄк таймаут ожидани€ ответа
			{
				rx_pos=0;temp=0;
				if(mod_table[base]==0) {base=0;step=2;}else step++; // если конец массива зациклить на начало
				do
				{
					mod_num=mod_table[rx_base+temp];
					if(emu_mode==1)	// не анализировать данные в режиме эмул€ции
					{
						if(mod_num<0x41) continue;
						if((mod_num>=0x81)&&(mod_num<0xA1)) continue;
					}
					if(err_mod[mod_num]==0)
					{
						if(mod_num<0x41)
						{
							if(rx_mod_cnt>=2+rx_pos)
							{
								if(rx_mod_buf[rx_pos++]==mod_num)
								{
									cur_crc=0x0F;
									update_CRC4(&cur_crc,mod_num>>4);
									update_CRC4(&cur_crc,mod_num);
									update_CRC4(&cur_crc,rx_mod_buf[rx_pos]);
									if(cur_crc==(rx_mod_buf[rx_pos]>>4)) IN[mod_num-1]=rx_mod_buf[rx_pos] & 0x0f;else
									{
										err_mod[mod_num]++;Sum_err++;
									}
									rx_pos++;
								}else {err_mod[mod_num]++;Sum_err++;rx_pos++;}
							}else {err_mod[mod_num]++;Sum_err++;rx_pos+=2;}
						}else
						if(mod_num<0x81)
						{
							if(rx_mod_cnt>=2+rx_pos)
							{
								if((rx_mod_buf[rx_pos]!=mod_num)||(rx_mod_buf[rx_pos+1]!=mod_num))
								{
									err_mod[mod_num]++;Sum_err++;
									rx_pos+=2;
								}else rx_pos+=2;
							}else {err_mod[mod_num]++;Sum_err++;rx_pos+=2;}
						}else
						if(mod_num<0xA1)
						{
							if(rx_mod_cnt>=9+rx_pos)
							{
								if(rx_mod_buf[rx_pos]==mod_num)
								{
									cur_crc=0x0F;
									update_CRC4(&cur_crc,mod_num>>4);
									update_CRC4(&cur_crc,mod_num);
									update_CRC4(&cur_crc,rx_mod_buf[rx_pos+1]>>4);
									update_CRC4(&cur_crc,rx_mod_buf[rx_pos+1]);
									update_CRC4(&cur_crc,rx_mod_buf[rx_pos+2]>>4);
									if(cur_crc==(rx_mod_buf[rx_pos+2] & 0x0F))
									{
										_ADC.D1[mod_num-0x81]=(((unsigned int)rx_mod_buf[rx_pos+1])<<8)|(rx_mod_buf[rx_pos+2]&0xF0);
										cur_crc=0x0F;
										update_CRC4(&cur_crc,rx_mod_buf[rx_pos+3]>>4);
										update_CRC4(&cur_crc,rx_mod_buf[rx_pos+3]);
										update_CRC4(&cur_crc,rx_mod_buf[rx_pos+4]>>4);
										if(cur_crc==(rx_mod_buf[rx_pos+4] & 0x0F))
										{
											_ADC.D2[mod_num-0x81]=(((unsigned int)rx_mod_buf[rx_pos+3])<<8)|(rx_mod_buf[rx_pos+4]&0xF0);
											cur_crc=0x0F;
											update_CRC4(&cur_crc,rx_mod_buf[rx_pos+5]>>4);
											update_CRC4(&cur_crc,rx_mod_buf[rx_pos+5]);
											update_CRC4(&cur_crc,rx_mod_buf[rx_pos+6]>>4);
											if(cur_crc==(rx_mod_buf[rx_pos+6] & 0x0F))
											{
												_ADC.D3[mod_num-0x81]=(((unsigned int)rx_mod_buf[rx_pos+5])<<8)|(rx_mod_buf[rx_pos+6]&0xF0);
												cur_crc=0x0F;
												update_CRC4(&cur_crc,rx_mod_buf[rx_pos+7]>>4);
												update_CRC4(&cur_crc,rx_mod_buf[rx_pos+7]);
												update_CRC4(&cur_crc,rx_mod_buf[rx_pos+8]>>4);
												if(cur_crc==(rx_mod_buf[rx_pos+8] & 0x0F))
												{
													_ADC.D4[mod_num-0x81]=(((unsigned int)rx_mod_buf[rx_pos+7])<<8)|(rx_mod_buf[rx_pos+8]&0xF0);
												}else {err_mod[mod_num]++;Sum_err++;}
											}else {err_mod[mod_num]++;Sum_err++;}
										}else {err_mod[mod_num]++;Sum_err++;}
									}else {err_mod[mod_num]++;Sum_err++;}
								}else{err_mod[mod_num]++;Sum_err++;}
								rx_pos+=9;
							}else {err_mod[mod_num]++;Sum_err++;rx_pos+=9;}
						}else
						{
							if(rx_mod_cnt>=2+rx_pos)
							{
								if((rx_mod_buf[rx_pos]!=mod_num)||(rx_mod_buf[rx_pos+1]!=mod_num))
								{
									err_mod[mod_num]++;Sum_err++;rx_pos+=2;
								}else rx_pos+=2;
							}else {err_mod[mod_num]++;Sum_err++;rx_pos+=2;}
						}
					}
				}while(mod_table[rx_base+(++temp)]);
			}
			break;
		case 1:	// отправка запроса
			// base - стартовый индекс группы модулей
			// rx_base - сохранЄнный индекс начала группы дл€ разбора при приЄме ответов
			// mod_num - адрес модул€
			// temp - счЄтчик внутри группы
			// Tx_end - флаг завершени€ передачи данных

			if((mod_table[base])&&(emu_mode!=2))
			{
				tx_mod_cnt=1;tx_mod_buf[0]=0x00;
				rx_base = base;temp=0;
				do
				{
					mod_num=mod_table[base+temp];
					if(err_mod[mod_num]==0)
					{
						if(mod_num<0x41) {if(emu_mode==0) {tx_mod_buf[tx_mod_cnt++]=mod_num;fl_in|=(1 << (mod_num-1));}} else
						{
							if(mod_num<0x81)
							{
								fl_out|=(1<<(mod_num-0x41));
								tx_mod_buf[tx_mod_cnt++]=mod_num;
								tx_mod_buf[tx_mod_cnt]=OUT[mod_num-0x41];
								cur_crc=0x0F;
								update_CRC4(&cur_crc,mod_num>>4);
								update_CRC4(&cur_crc,mod_num);
								update_CRC4(&cur_crc,OUT[mod_num-0x41]);
								tx_mod_buf[tx_mod_cnt]|=cur_crc<<4;
								tx_mod_cnt++;
							}else
							{
								if(mod_num<0xA1) {if(emu_mode==0){tx_mod_buf[tx_mod_cnt++]=mod_num;fl_adc|=(1<<(mod_num-0x81));}}else
								{
									fl_dac|=(1<<(mod_num-0xA1));
									tx_mod_buf[tx_mod_cnt++]=mod_num;
									tx_mod_buf[tx_mod_cnt++]=_DAC.D1[mod_num-0xA1]>>8;
									tx_mod_buf[tx_mod_cnt]=_DAC.D1[mod_num-0xA1] & 0xF0;
									cur_crc=0x0F;
									update_CRC4(&cur_crc,mod_num>>4);
									update_CRC4(&cur_crc,mod_num);
									update_CRC4(&cur_crc,_DAC.D1[mod_num-0xA1]>>12);
									update_CRC4(&cur_crc,_DAC.D1[mod_num-0xA1]>>8);
									update_CRC4(&cur_crc,_DAC.D1[mod_num-0xA1]>>4);
									tx_mod_buf[tx_mod_cnt] |= cur_crc;
									tx_mod_cnt++;
									tx_mod_buf[tx_mod_cnt++]=_DAC.D2[mod_num-0xA1]>>8;
									tx_mod_buf[tx_mod_cnt]=_DAC.D2[mod_num-0xA1] & 0xF0;
									cur_crc=0x0F;
									update_CRC4(&cur_crc,_DAC.D2[mod_num-0xA1]>>12);
									update_CRC4(&cur_crc,_DAC.D2[mod_num-0xA1]>>8);
									update_CRC4(&cur_crc,_DAC.D2[mod_num-0xA1]>>4);
									tx_mod_buf[tx_mod_cnt]|=cur_crc;
									tx_mod_cnt++;
								}
							}
						}
					}
					temp++;
				}while(mod_table[base+temp]);
				tx_mod_buf[tx_mod_cnt++]=0xFF;write_module(tx_mod_cnt);
				Tx_end=0;base+=temp+1;
				step=0;
			}
			break;
		case 2:	// поиск сбойных модулей и повторные запросы
			//	err_mod_num - инкрементируемый индекс в массиве сбойных модулей
			do{
				if((err_mod[err_mod_num])&&(err_mod[err_mod_num]<=100))	// если обнаружен сбойный модуль
				{
					if(err_mod_num<0x41)
					{
						tx_mod_buf[0]=0x00;
						tx_mod_buf[1]=err_mod_num;
						tx_mod_buf[2]=0xFF;
						write_module(3);Tx_end=0;step=3;
						break;
					}else if(err_mod_num<0x81)
					{
						tx_mod_buf[0]=0x00;
						tx_mod_buf[1]=err_mod_num;
						tx_mod_buf[2]=OUT[err_mod_num-0x41];
						cur_crc=0x0F;
						update_CRC4(&cur_crc,err_mod_num>>4);
						update_CRC4(&cur_crc,err_mod_num);
						update_CRC4(&cur_crc,OUT[err_mod_num-0x41]);
						tx_mod_buf[2]|=cur_crc<<4;
						tx_mod_buf[3]=0xFF;
						write_module(4);Tx_end=0;step=3;
						break;
					}else if(err_mod_num<0xA1)
					{
						tx_mod_buf[0]=0x00;
						tx_mod_buf[1]=err_mod_num;
						tx_mod_buf[2]=0xFF;
						write_module(3);Tx_end=0;step=3;
						break;
					}else
					{
						tx_mod_buf[0]=0x00;
						tx_mod_buf[1]=err_mod_num;
						tx_mod_buf[2]=_DAC.D1[err_mod_num-0xA1]>>8;
						tx_mod_buf[3]=_DAC.D1[err_mod_num-0xA1] & 0xF0;
						cur_crc=0x0F;
						update_CRC4(&cur_crc,err_mod_num>>4);
						update_CRC4(&cur_crc,err_mod_num);
						update_CRC4(&cur_crc,_DAC.D1[err_mod_num-0xA1]>>12);
						update_CRC4(&cur_crc,_DAC.D1[err_mod_num-0xA1]>>8);
						update_CRC4(&cur_crc,_DAC.D1[err_mod_num-0xA1]>>4);
						tx_mod_buf[3]|=cur_crc;
						tx_mod_buf[4]=_DAC.D2[err_mod_num-0xA1]>>8;
						tx_mod_buf[5]=_DAC.D2[err_mod_num-0xA1] & 0xF0;
						cur_crc=0x0F;
						update_CRC4(&cur_crc,_DAC.D2[err_mod_num-0xA1]>>12);
						update_CRC4(&cur_crc,_DAC.D2[err_mod_num-0xA1]>>8);
						update_CRC4(&cur_crc,_DAC.D2[err_mod_num-0xA1]>>4);
						tx_mod_buf[5]|=cur_crc;
						tx_mod_buf[6]=0xFF;
						write_module(7);Tx_end=0;step=3;
						break;
					}
				}
			}
			while(++err_mod_num);
			if(err_mod_num==0) step=1;
			break;
		case 3:	// разбор ответа от сбойного модул€
			if((Tx_end)&&(get_mmb_tmr() > ANSWER_PAUSE))
			{
				temp=0;base=0;step=1;
				if(err_mod_num<0x41)
				{
					if(rx_mod_cnt>=2)
					{
						if(rx_mod_buf[0]==err_mod_num)
						{
							cur_crc=0x0F;
							update_CRC4(&cur_crc,err_mod_num>>4);
							update_CRC4(&cur_crc,err_mod_num);
							update_CRC4(&cur_crc,rx_mod_buf[1]);
							if(cur_crc==(rx_mod_buf[1]>>4))
							{
								IN[err_mod_num-1]=rx_mod_buf[rx_pos]&0x0f;
								err_mod[err_mod_num]=0;
							}
							else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
						}else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
					}else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
				}else
				if(err_mod_num<0x81)
				{
					if(rx_mod_cnt>=2)
					{
						if((rx_mod_buf[0]!=err_mod_num)||(rx_mod_buf[1]!=err_mod_num))
						{
							if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;
						} else err_mod[err_mod_num]=0;
					}else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
				}else
				if(err_mod_num<0xA1)
				{
					if(rx_mod_cnt>=9)
					{
						if(rx_mod_buf[0]==err_mod_num)
						{
							cur_crc=0x0F;
							update_CRC4(&cur_crc,err_mod_num>>4);
							update_CRC4(&cur_crc,err_mod_num);
							update_CRC4(&cur_crc,rx_mod_buf[1]>>4);
							update_CRC4(&cur_crc,rx_mod_buf[1]);
							update_CRC4(&cur_crc,rx_mod_buf[2]>>4);
							if(cur_crc==(rx_mod_buf[2] & 0x0F)) err_mod[err_mod_num]=0;
							else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
						}else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
					}else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
				}else
				{
					if(rx_mod_cnt>=2)
					{
						if((rx_mod_buf[0]!=err_mod_num)||(rx_mod_buf[1]!=err_mod_num))
						{
							if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;
						}else err_mod[err_mod_num]=0;
					}else {if(err_mod[err_mod_num]<100) err_mod[err_mod_num]++;Sum_err++;}
				}
				err_mod_num++;
			}
			break;
	}
}

