/*
 * 004Led_BT_interrupt.c
 *
 *  Created on: 28-Feb-2021
 *      Author: Lingaswamy Shyaga
 */

#include <stdio.h>
#include  "stm32f410xx.h"
#include  "stm32f410xx_gpio_driver.h"
//extern void initialise_monitor_handles(void);
void delay()
{
	uint32_t i;
	for(i=0;i<500000;i++);
}

int main()
{
//this is led gpio configuration

	GPIO_handle_t Gpio_Led,Gpio_Bttp;
	Gpio_Led.pGPIOx = GPIOA;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_5;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinMode   = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig_t.GPIO_Pinspeed  = GPIO_PIN_SPD_HIGH;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_PIN_OUT_TP_PUPL;
	//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_NO;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_UP;
	//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_DOWN;
	GPIO_PerlClkControle(GPIOA,ENABLE);

	GPIO_Init(&Gpio_Led);

	//this is btn configuration

	Gpio_Bttp.pGPIOx = GPIOC;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_13;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinMode   = GPIO_MODE_IT_FT;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_Pinspeed  = GPIO_PIN_SPD_HIGH;
	//Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_PIN_OUT_TP_PUPL;
	//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPDR;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_UP;
	//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_DOWN;
	GPIO_PerlClkControle(GPIOC,ENABLE);
	GPIO_Init(&Gpio_Bttp);


	//interrupt irq priority configuration
	GPIO_priorityIRQconfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRT_15);
	GPIO_interruptIRQConfig(IRQ_NO_EXTI9_5,ENABLE);

    while(1);
}
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_13);
	GPIO_ToggleOutputPin (GPIOA,GPIO_PIN_5);
	delay();
}


















