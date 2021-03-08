
/*
 * main.c
 *
 *  Created on: 28-Feb-2021
 *      Author: Lingaswamy Shyaga
 */
#include <stdio.h>
#include "stm32f410xx.h"

int main()
{
	return 0;
}
void EXTI0_IRQHandler(void)
{
	//handle the  interrupt
	GPIO_IRQHandler(0);
}
