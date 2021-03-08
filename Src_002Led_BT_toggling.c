/*
 * 001Led_toggling.c
 *
 *  Created on: 17-Feb-2021
 *      Author: Lingaswamy Shyaga
 */
#include<stdio.h>
#include  "stm32f410xx.h"
#include  "stm32f410xx_gpio_driver.h"
//extern void initialise_monitor_handles(void);
void delay()
{
	uint32_t i;
	for(i=0;i<900000;i++);

}
#define LOW             0
#define BTN_PRESSED     LOW
int main()
{
//	initialise_monitor_handles();
	GPIO_handle_t Gpio_Led,Gpio_Bttp;
	Gpio_Led.pGPIOx = GPIOA;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_5;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinMode   = GPIO_MODE_OUT;
	Gpio_Led.GPIO_PinConfig_t.GPIO_Pinspeed  = GPIO_PIN_SPD_HIGH;
   // Gpio_Led.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_PIN_OUT_TP_PUPL;
		//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_NO;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_UP;
		//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_DOWN;
	GPIO_PerlClkControle(GPIOA,ENABLE);
	GPIO_Init(&Gpio_Led);

	Gpio_Bttp.pGPIOx = GPIOC;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinNumber = GPIO_PIN_13;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinMode   = GPIO_MODE_IN;
	Gpio_Bttp.GPIO_PinConfig_t.GPIO_Pinspeed  = GPIO_PIN_SPD_HIGH;
	//Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinOPType = GPIO_PIN_OUT_TP_PUPL;
	Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPDR;
	//Gpio_Bttp.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_UP;
	//Gpio_Led.GPIO_PinConfig_t.GPIO_PinPuPdControl = GPIO_PIN_PUPDR_DOWN;
	GPIO_PerlClkControle(GPIOC,ENABLE);
	GPIO_Init(&Gpio_Bttp);

	while(1)
	{
		if(GPIO_ReadfromInputPin(GPIOC,GPIO_PIN_13)== BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin (GPIOA,GPIO_PIN_5);

		}
	}
	return 0;
}
