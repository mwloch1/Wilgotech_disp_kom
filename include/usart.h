/*
 * usart.h
 *
 *  Created on: 17 maj 2018
 *      Author: Micha³
 */

#ifndef USART_H_
#define USART_H_

#include <stdlib.h>
#include "stm32f4xx_conf.h"

void USART_Initialize(void);
void SendChar(char);
void USART_Send(volatile char *s);

#endif /* USART_H_ */
