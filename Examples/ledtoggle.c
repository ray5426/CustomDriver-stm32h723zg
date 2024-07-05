/*
 * @file ledtoggle.c
 *
 *  Created on: June 28, 2024
 *      Author: jay
 *
 *  
 *  @brief Example demonstrating how to toggle LEDs using GPIO driver on STM32H723xx microcontroller.
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


void delay(void){

	for(uint32_t x =0;x<1000000;x++); // Loop to create a delay
}

int main(void){

	GPIO_Handle_t greenLed,yellowLed,redLed;

	greenLed.pGPIOx = GPIOB;
	greenLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	greenLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	greenLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	greenLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	greenLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	yellowLed.pGPIOx = GPIOE;
	yellowLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	yellowLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	yellowLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	yellowLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	yellowLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	redLed.pGPIOx = GPIOB;
	redLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	redLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	redLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	redLed.GPIO_PinConfig.GPIO_PinOType = GPIO_OP_TYPE_PP;
	redLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&greenLed);
	GPIO_Init(&yellowLed);
	GPIO_Init(&redLed);

	while(1){

		GPIO_ToggleOutPin(greenLed.pGPIOx,greenLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
		GPIO_ToggleOutPin(yellowLed.pGPIOx,yellowLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
		GPIO_ToggleOutPin(redLed.pGPIOx,redLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}

	return 0;
}
