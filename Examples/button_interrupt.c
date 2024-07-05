/*
 * @file button_interrupt.c
 *
 *  Created on: July 4, 2024
 *      Author: jay
 *
 *  
 *  @brief Example demonstrating interrupt-driven button input and LED control using GPIO driver on STM32H723xx microcontroller.
 *
 * MIT License
 *
 * Copyright (c) 2024 jay <jagmohankumar5246@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "stm32h723xx.h"
#include "stm32h723xx_gpio_driver.h"
#include <stdint.h>

void userButtonISR(void *useData); // Callback function for user button
void delay(void);


GPIO_Handle_t yellowLed,userButton;
uint8_t togglePin = 0;

int main(void){

	userButton.pGPIOx = GPIOC;
	userButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	userButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // Interrupt on falling edge
	userButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	userButton.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	userButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	yellowLed.pGPIOx = GPIOE;
	yellowLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	yellowLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	yellowLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	yellowLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	yellowLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&yellowLed);
    GPIO_Init(&userButton);
    
    GPIO_IRQConfig(userButton.GPIO_PinConfig.GPIO_PinNumber,ENABLE); // Enable EXTI line interrupt
    GPIO_IRQPriorityConfig(userButton.GPIO_PinConfig.GPIO_PinNumber,14);
    GPIO_PinNumber_t pinNO = yellowLed.GPIO_PinConfig.GPIO_PinNumber;
    GPIO_RegisterCallback(userButton.GPIO_PinConfig.GPIO_PinNumber,userButtonISR,(void *)&pinNO);

    GPIO_ToggleOutPin(yellowLed.pGPIOx,yellowLed.GPIO_PinConfig.GPIO_PinNumber);

	while(1){

		if(togglePin){
			GPIO_ToggleOutPin(yellowLed.pGPIOx,yellowLed.GPIO_PinConfig.GPIO_PinNumber);
			delay();
			togglePin = 0;
		}
	}

	return 0;
}

void userButtonISR(void *useData){

	togglePin = 1;
//    GPIO_ToggleOutPin(yellowLed.pGPIOx,*((GPIO_PinNumber_t *)useData)); // Optionally toggle LED directly in ISR
//	delay();
}

void delay(void){

	for(uint32_t x =0;x<1000000;x++);
}
