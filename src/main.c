/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_conf.h"
#include "main_set.h"
#include "debugkom.h"
#include "delay.h"
#include "timer.h"
#include "usart.h"
#include "kom.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

char cbuf[256];

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/*
 * 	Delay jest ustawiony w oparciu o SysTickHandler i udostępnia opóźnienia 1-milisekundowe
 *
 * 	Moduł Timer pozwala rejestrować 256 timerów software'owych w oparciu o jeden timer HW
 *
 * 	Moduł Debug pozwala na wyświetlanie stanu urządzenia na porcie szeregowym oraz obsługę komend
 */

int main()
{
	SystemInit();
	DelayInit();
	Timer_Init();
	Debug_Init();
	USART_Initialize();

	GPIO_InitTypeDef  GPIO_InitStructure;
	//Konfiguracja pin�w z diodami LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Konfiguracja pinu z przyciskiem USER
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Wys�anie pr�bnego tekstu po USARTcie
    KOM_Init();
	while (1)
	{
		KOM_Main();
		Delay_ms(100);
	}
}
