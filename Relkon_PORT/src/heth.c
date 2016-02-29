/*
 * heth.c
 *
 *  Created on: Mar 6, 2012
 *      Author: Роман
 */

#include "heth.h"
#include "k_mac.h"

#include "hcanal.h"

#include "stm32_eth.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"

extern unsigned char canal_tx_buf[512];

static unsigned char mac[6];
static eth_buf buf;

extern portTickType ExLastExecutionTime;
ETH_InitTypeDef ETH_InitStructure;

ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB]; /* Used from stm32_eth.c file */
ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];

unsigned long Rx_Buff[ETH_RXBUFNB] = {0};	// указатели на буфера
unsigned long Tx_Buff[ETH_TXBUFNB] = {0};

unsigned char ETH_RX[ETH_RXBUFNB][ETH_PKT_SIZE];	// сами буфера
unsigned char ETH_TX[ETH_TXBUFNB][ETH_PKT_SIZE];

extern unsigned short eth_pkt_cnt;

unsigned char* get_mac_tx(void)
{
	return (unsigned char*)ETH_GetCurrentTxBufferAddress();
}

void GPIO_ENETConfiguration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PA1 and PA7 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_SET);

	RCC_PLL3Config(RCC_PLL3Mul_10);
	RCC_PLL3Cmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET);
	RCC_MCOConfig(RCC_MCO_PLL3CLK);
}

unsigned long ENET_Configuration(void)
{
	unsigned long res=0;
	unsigned short tmp;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
	                        RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

	ETH_DeInit();
	get_mac(mac);
	ETH_MACAddressConfig(ETH_MAC_Address0, mac);
	GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);

	ETH_SoftwareReset();
	while(ETH_GetSoftwareResetStatus()==SET) vTaskDelayUntil( &ExLastExecutionTime, ( portTickType ) 1 / portTICK_RATE_MS );
	ETH_StructInit(&ETH_InitStructure);


	ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
	ETH_InitStructure.ETH_Watchdog = ETH_Watchdog_Disable;
	ETH_InitStructure.ETH_Jabber = ETH_Jabber_Disable;
	ETH_InitStructure.ETH_JumboFrame = ETH_JumboFrame_Disable;
	ETH_InitStructure.ETH_InterFrameGap = ETH_InterFrameGap_64Bit;
	ETH_InitStructure.ETH_CarrierSense = ETH_CarrierSense_Enable;
	ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
	ETH_InitStructure.ETH_ReceiveOwn = ETH_ReceiveOwn_Disable;
	ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
	ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
	ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
	ETH_InitStructure.ETH_BackOffLimit = ETH_BackOffLimit_10;
	ETH_InitStructure.ETH_DeferralCheck = ETH_DeferralCheck_Disable;
	ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
	ETH_InitStructure.ETH_PassControlFrames = ETH_PassControlFrames_ForwardPassedAddrFilter;
	ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Disable;
	ETH_InitStructure.ETH_DestinationAddrFilter = ETH_DestinationAddrFilter_Normal;
	ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
	ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
	ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
	ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
	ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
	ETH_InitStructure.ETH_FlushReceivedFrame = ETH_FlushReceivedFrame_Enable;
	ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
	ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
	res = ETH_Init(&ETH_InitStructure, PHY_ADDRESS);

	ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, ENABLE);
	for(tmp=0;tmp<ETH_RXBUFNB;tmp++) Rx_Buff[tmp] = (unsigned long)&ETH_RX[tmp][0];
	for(tmp=0;tmp<ETH_TXBUFNB;tmp++) Tx_Buff[tmp] = (unsigned long)&ETH_TX[tmp][0];
	ETH_DMATxDescChainInit(DMATxDscrTab, (unsigned char*)Tx_Buff, ETH_TXBUFNB);
	ETH_DMARxDescChainInit(DMARxDscrTab, (unsigned char*)Rx_Buff, ETH_RXBUFNB);
	for(tmp=0;tmp<ETH_RXBUFNB;tmp++){ETH_DMARxDescReceiveITConfig(DMARxDscrTab+tmp, ENABLE);}
	ETH_Start();
	ETH_ResumeDMAReception();
	return(res);
}

unsigned char get_link_status(void)
{
	if(((ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BSR)) & PHY_Linked_Status) == 0) return 0;
	return 1;
}

unsigned char test_rx_overflow(void)
{
	unsigned char i;
	if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
	{
		for(i=0;i<ETH_RXBUFNB;i++) DMARxDscrTab[i].Status = ETH_DMARxDesc_OWN;
		ETH->DMASR = ETH_DMASR_RBUS;
		ETH->DMARPDR = 0;
		return 1;
	}
	return 0;
}

void send_mac_data(unsigned short len)
{
	ETH_DMADESCTypeDef* ptr;
	ptr = (ETH_DMADESCTypeDef*)ETH_GetCurrentTxDescStartAddress();
	ptr->ControlBufferSize = (len & ETH_DMATxDesc_TBS1);
	ptr->Status = ETH_DMATxDesc_OWN | ETH_DMATxDesc_LS | ETH_DMATxDesc_FS | ETH_DMATxDesc_TCH | ETH_DMATxDesc_ChecksumTCPUDPICMPFull;//ETH_DMATxDesc_TER;// | ETH_DMATxDesc_TCH | ETH_DMATxDesc_IC;
	if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET) {ETH->DMASR = ETH_DMASR_TBUS;}
	ETH->DMATPDR = 0;
	vTaskDelayUntil( &ExLastExecutionTime, ( portTickType ) 1 / portTICK_RATE_MS );
}

eth_buf* get_mac_data(unsigned char num)
{
	if((DMARxDscrTab[num].Status & ETH_DMARxDesc_ES) != 0)
	{
		DMARxDscrTab[num].Status = ETH_DMARxDesc_OWN;
		ETH_ResumeDMAReception();
        buf.len=0;
	}
	else if( (DMARxDscrTab[num].Status & ETH_DMARxDesc_OWN ) == 0 )
	{
		buf.len = ( unsigned short ) ((DMARxDscrTab[num].Status & ETH_DMARxDesc_FL) >> 16UL);
		buf.ptr = &ETH_RX[num][0];
		if(buf.len==0)
		{
			DMARxDscrTab[num].Status = ETH_DMARxDesc_OWN;
			ETH_ResumeDMAReception();
		}

	}else{buf.len=0;}
	return (&buf);
}

void unlock_mac_rx(unsigned char num)
{
	DMARxDscrTab[num].Status = ETH_DMARxDesc_OWN;
	ETH_ResumeDMAReception();
}

void ETH_IRQHandler(void)
{
	unsigned long statusReg =ETH->DMASR;
	if ((statusReg & ETH_DMA_IT_AIS) != (u32)RESET) {ETH->DMASR = (u32)ETH_DMA_IT_AIS;}
	if ((statusReg & ETH_DMA_IT_RO) != (u32)RESET) {ETH->DMASR = (u32)ETH_DMA_IT_RO;}
	if ((statusReg & ETH_DMA_IT_RBU) != (u32)RESET) {ETH->DMASR = (u32)ETH_DMA_IT_RBU;}
	ETH->DMASR = (u32)ETH_DMA_IT_NIS; //Always Clear NIS
	if((statusReg & ETH_DMA_IT_R) != (u32)RESET)
	{
		ETH->DMASR = (u32)ETH_DMA_IT_R;
	}
	if((statusReg & ETH_DMA_IT_T) != (u32)RESET)
	{
		ETH->DMASR = (u32)ETH_DMA_IT_T;
	}
}
