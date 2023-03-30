#include "uart_imp.h"

// global definitions.
unsigned char RxBuf[RXBUFSIZE];// receive buffer
int RxHead =0; // circular buffer index
int RxTail =0; // circular buffer index
unsigned char TxBuf[TXBUFSIZE];// transmit buffer
int TxHead =0; // circular buffer index
int TxTail =0; // circular buffer index

unsigned char _getchar()
{
	unsigned char temp;
	while (RxTail == RxHead)
	{
		HAL_Delay(1000);
	}
	temp = RxBuf[RxTail];
	if (++RxTail > (RXBUFSIZE -1))
	{
		RxTail = 0;
	}
	return(temp);
}
