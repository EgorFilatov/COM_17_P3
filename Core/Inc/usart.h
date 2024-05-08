#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f0xx.h"

void uart1Init(uint32_t speed);
void uart1TxDma(uint8_t *dmaTxMemAddr, uint32_t TxBuffSize);
void uart1RxDma(uint8_t *dmaRxMemAddr, uint32_t RxBuffSize);

#endif
