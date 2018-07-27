/*
 * usart.c
 *
 *  Created on: 17 maj 2018
 *      Author: Micha³
 */

#include "usart.h"

void USART_Initialize(void)
	{
		USART_InitTypeDef UART_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		// GPIO Config
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

		// UART Config

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

		USART_StructInit(&UART_InitStructure);
		UART_InitStructure.USART_BaudRate = 57600;
		UART_InitStructure.USART_WordLength = USART_WordLength_8b;
		UART_InitStructure.USART_StopBits = USART_StopBits_1;
		UART_InitStructure.USART_Parity = USART_Parity_No;
		UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART6,&UART_InitStructure);
		USART_Cmd(USART6, ENABLE);


		// NVIC Config

		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	}


void USART_Send(volatile char *c)
	{
     //Petla dziala do puki bedzie jakis znak do wyslania
     while(*c)
     {
        //Sprawdza czy rejestr danych zostal oprózniony
        while( !(USART6->SR & 0x00000040) );
        //Przeslij dane,
        USART_SendData(USART6, *c);
        *c++;
     }
}



void SendChar(char c)
	{
     //Sprawdza czy bufor nadawczy jest pusty
     while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
     USART_SendData(USART6, c);
	}

