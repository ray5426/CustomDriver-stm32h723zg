/*
 * @file stm32h723xx_gpio_driver.c
 *
 *  Created on: June 28, 2024
 *      Author: jay
 *
 * 
 * @brief This file contains implementations for GPIO peripheral operations
 *        for STM32H723xx microcontroller.
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
#include <stm32h723xx_gpio_driver.h>
#include <stdlib.h>

typedef struct {
    GPIO_Callback_t callback;    /**< Callback function pointer */
    void *userData;              /**< User data associated with callback */
    uint8_t pinNumber;           /**< GPIO pin number */
} GPIO_CallbackInfo_t;

static GPIO_CallbackInfo_t GPIO_Callbacks[__MAX_GPIO_EXTI_IRQ]; /*!< Array to store GPIO callbacks */
static uint8_t callbacks_enabled = 0;                           /*!< Flag indicating if callbacks are enabled */

/**
 * @brief Controls the clock for a GPIO peripheral.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 * @param state State to control the clock (ENABLE or DISABLE)
 */
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx,State_t state ){

    uint8_t pos;
    if(pGPIOx == GPIOA) pos = RCC_AHB4ENR_GPIOAEN_Pos;
    else if(pGPIOx == GPIOB) pos = RCC_AHB4ENR_GPIOBEN_Pos;
    else if(pGPIOx == GPIOC) pos = RCC_AHB4ENR_GPIOCEN_Pos;
    else if(pGPIOx == GPIOD) pos = RCC_AHB4ENR_GPIODEN_Pos;
    else if(pGPIOx == GPIOE) pos = RCC_AHB4ENR_GPIOEEN_Pos;
    else if(pGPIOx == GPIOF) pos = RCC_AHB4ENR_GPIOFEN_Pos;
    else if(pGPIOx == GPIOG) pos = RCC_AHB4ENR_GPIOGEN_Pos;
    else if(pGPIOx == GPIOH) pos = RCC_AHB4ENR_GPIOHEN_Pos;
    else if(pGPIOx == GPIOJ) pos = RCC_AHB4ENR_GPIOJEN_Pos;
    else if(pGPIOx == GPIOK) pos = RCC_AHB4ENR_GPIOKEN_Pos;

    switch(state){
        case ENABLE:
            GPIOx_PCLK_EN(pos);
            break;
        case DISABLE:
            GPIOx_PCLK_DI(pos);
            break;
    }
}

/**
 * @brief Initializes GPIO pin according to the specified parameters in the GPIO_Handle_t structure.
 *
 * @param pGPIOHandle Pointer to GPIO handle structure
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

    uint32_t temp = 0;

    GPIO_ClkControl(pGPIOHandle->pGPIOx,ENABLE);
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x03 << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else{

        temp = GPIO_MODE_IN << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x03 << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
        pGPIOHandle->pGPIOx->MODER |= temp;

        switch(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode){

        case GPIO_MODE_IT_FT:
            EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR1 |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            break;
        case GPIO_MODE_IT_RT:
            EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR1 |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            break;
        case GPIO_MODE_IT_RFT:
            EXTI->FTSR1 |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR1 |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            break;
        }

        uint8_t port;
        if(pGPIOHandle->pGPIOx == GPIOA)      port = SYSCFG_EXTIx_GPIOA_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOB) port = SYSCFG_EXTIx_GPIOB_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOC) port = SYSCFG_EXTIx_GPIOC_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOD) port = SYSCFG_EXTIx_GPIOD_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOE) port = SYSCFG_EXTIx_GPIOE_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOF) port = SYSCFG_EXTIx_GPIOF_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOG) port = SYSCFG_EXTIx_GPIOG_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOH) port = SYSCFG_EXTIx_GPIOH_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOJ) port = SYSCFG_EXTIx_GPIOJ_Pos;
        else if(pGPIOHandle->pGPIOx == GPIOK) port = SYSCFG_EXTIx_GPIOK_Pos;

        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = (port << (4 * temp2));

        EXTI->CPUIMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    }
    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << ( 2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;
    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

        uint32_t temp1,temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F  << ( 4 * temp2) );
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2));
    }
}

/**
 * @brief Deinitializes a GPIO peripheral.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

    uint8_t pos;
    GPIO_ClkControl(pGPIOx,DISABLE);

    if(pGPIOx == GPIOA) pos = RCC_AHB4RSTS_GPIOARSTR_Pos;
    else if(pGPIOx == GPIOB) pos = RCC_AHB4RSTS_GPIOBRSTR_Pos;
    else if(pGPIOx == GPIOC) pos = RCC_AHB4RSTS_GPIOCRSTR_Pos;
    else if(pGPIOx == GPIOD) pos = RCC_AHB4RSTS_GPIODRSTR_Pos;
    else if(pGPIOx == GPIOE) pos = RCC_AHB4RSTS_GPIOERSTR_Pos;
    else if(pGPIOx == GPIOF) pos = RCC_AHB4RSTS_GPIOFRSTR_Pos;
    else if(pGPIOx == GPIOG) pos = RCC_AHB4RSTS_GPIOGRSTR_Pos;
    else if(pGPIOx == GPIOH) pos = RCC_AHB4RSTS_GPIOHRSTR_Pos;
    else if(pGPIOx == GPIOJ) pos = RCC_AHB4RSTS_GPIOJRSTR_Pos;
    else if(pGPIOx == GPIOK) pos = RCC_AHB4RSTS_GPIOKRSTR_Pos;

    GPIOx_REG_RESET(pos);
}

/**
 * @brief Reads the input value of a specified GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 * @param PinNumber GPIO pin number
 * @return uint8_t GPIO pin input value (0 or 1)
 */
uint8_t GPIO_ReadInPin(GPIO_RegDef_t *pGPIOx,GPIO_PinNumber_t PinNumber){

    uint8_t data;
    data = (uint8_t)((pGPIOx->IDR >> PinNumber ) & 0x00000001);

    return data;
}

/**
 * @brief Reads the input port value of a GPIO peripheral.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 * @return uint16_t GPIO port input value
 */
uint16_t GPIO_ReadInPort(GPIO_RegDef_t *pGPIOx){

    uint16_t data;
    data = (uint16_t)(pGPIOx->IDR);

    return data;
}

/**
 * @brief Writes a value to the output data register of a specified GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 * @param PinNumber GPIO pin number
 * @param pinState State to write to the pin (GPIO_PIN_SET or GPIO_PIN_RESET)
 */
void GPIO_WriteOutPin(GPIO_RegDef_t *pGPIOx,GPIO_PinNumber_t PinNumber,GPIO_PinState_t pinState){

    switch (pinState)
    {
    case GPIO_PIN_SET:
        pGPIOx->ODR |= (1 << PinNumber);
        break;
    case GPIO_PIN_RESET:
        pGPIOx->ODR &= ~(1 << PinNumber);
        break;
    }
}

/**
 * @brief Writes a value to the output data register of a GPIO peripheral.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 * @param value Value to write to the GPIO output data register
 */
void GPIO_WriteOutPort(GPIO_RegDef_t *pGPIOx,uint16_t value){

    pGPIOx->ODR = value;

}

/**
 * @brief Toggles the output data of a specified GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral register definition structure
 * @param PinNumber GPIO pin number
 */
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx,GPIO_PinNumber_t PinNumber){

    pGPIOx->ODR ^= (1 << PinNumber);

}

/**
 * @brief Converts GPIO pin number to corresponding EXTI line number.
 *
 * @param GPIO_Pin GPIO pin number
 * @return EXTI_LineNumber Corresponding EXTI line number
 */
EXTI_LineNumber GPIO_PinToEXTILine(GPIO_PinNumber_t GPIO_Pin) {

    switch (GPIO_Pin) {
        case GPIO_PIN_0: return EXTI_LINE_0;
        case GPIO_PIN_1: return EXTI_LINE_1;
        case GPIO_PIN_2: return EXTI_LINE_2;
        case GPIO_PIN_3: return EXTI_LINE_3;
        case GPIO_PIN_4: return EXTI_LINE_4;
        case GPIO_PIN_5: return EXTI_LINE_5;
        case GPIO_PIN_6: return EXTI_LINE_6;
        case GPIO_PIN_7: return EXTI_LINE_7;
        case GPIO_PIN_8: return EXTI_LINE_8;
        case GPIO_PIN_9: return EXTI_LINE_9;
        case GPIO_PIN_10: return EXTI_LINE_10;
        case GPIO_PIN_11: return EXTI_LINE_11;
        case GPIO_PIN_12: return EXTI_LINE_12;
        case GPIO_PIN_13: return EXTI_LINE_13;
        case GPIO_PIN_14: return EXTI_LINE_14;
        case GPIO_PIN_15: return EXTI_LINE_15;
    }

    return -1;
}

/**
 * @brief Converts GPIO pin number to corresponding EXTI IRQ number.
 *
 * @param GPIO_Pin GPIO pin number
 * @return EXTI_IRQNumber Corresponding EXTI IRQ number
 */
EXTI_IRQNumber GPIO_PintoEXTI_IRQno(GPIO_PinNumber_t GPIO_Pin) {

    switch (GPIO_Pin) {
        case GPIO_PIN_0: return IRQ_NO_EXTI0;
        case GPIO_PIN_1: return IRQ_NO_EXTI1;
        case GPIO_PIN_2: return IRQ_NO_EXTI2;
        case GPIO_PIN_3: return IRQ_NO_EXTI3;
        case GPIO_PIN_4: return IRQ_NO_EXTI4;

        case GPIO_PIN_5:
        case GPIO_PIN_6:
        case GPIO_PIN_7:
        case GPIO_PIN_8:
        case GPIO_PIN_9: return IRQ_NO_EXTI9_5;

        case GPIO_PIN_10:
        case GPIO_PIN_11:
        case GPIO_PIN_12:
        case GPIO_PIN_13:
        case GPIO_PIN_14:
        case GPIO_PIN_15: return IRQ_NO_EXTI15_10;
    }

    return -1;
}

/**
 * @brief Configures the IRQ (Interrupt Request) for a GPIO pin.
 *
 * @param PinNumber GPIO pin number
 * @param state State to configure (ENABLE or DISABLE)
 */
void GPIO_IRQConfig(GPIO_PinNumber_t PinNumber, State_t state){

    uint8_t temp1,temp2;

    EXTI_LineNumber IRQNumber = GPIO_PinToEXTILine(PinNumber);
    temp1 = IRQNumber / 32;
    temp2 = IRQNumber % 32;

    switch (state){

        case ENABLE:
            NVIC->ISER[temp1] = ( 1 << temp2); 
            break;
        
        case DISABLE:
            NVIC->ICER[temp1] = ( 1 << temp2); 
            break;
        }
}

/**
 * @brief Configures the priority of the IRQ (Interrupt Request) for a GPIO pin.
 *
 * @param PinNumber GPIO pin number
 * @param IRQPriority IRQ priority to set
 */
void GPIO_IRQPriorityConfig(GPIO_PinNumber_t PinNumber,uint8_t IRQPriority){

    uint8_t temp1,temp2;

    EXTI_LineNumber IRQNumber = GPIO_PinToEXTILine(PinNumber);
    temp1 = IRQNumber / 4;
    temp2 = IRQNumber % 4;

    NVIC->IPR[temp1] = ( (IRQPriority << (8 - __NVIC_PRIO_BITS    ) )<< (temp2 * 8));

}

/**
 * @brief Default callback function for GPIO interrupts.
 */
void Default_GPIO_Callback() {
    // Default implementation (halt)
    while(1);
}

/**
 * @brief Enables GPIO interrupt callbacks.
 */
void GPIO_EnableCallbacks(void) {

    for (int i = 0; i < MAX_PIN_COUNT; i++) {
        GPIO_Callbacks[i].callback = (GPIO_Callback_t)Default_GPIO_Callback;
        GPIO_Callbacks[i].userData = NULL;
        GPIO_Callbacks[i].pinNumber = -1;
    }
    callbacks_enabled = 1;
}

/**
 * @brief Registers a callback function for a GPIO pin interrupt.
 *
 * @param PinNumber GPIO pin number
 * @param callback Callback function pointer
 * @param userdata User data associated with the callback
 */
void GPIO_RegisterCallback(GPIO_PinNumber_t PinNumber,GPIO_Callback_t callback,void *userdata){

    if(!callbacks_enabled) GPIO_EnableCallbacks();

    EXTI_LineNumber IRQNumber = GPIO_PintoEXTI_IRQno(PinNumber);
    if (callback != NULL) {
        GPIO_Callbacks[IRQNumber].callback = callback;
        GPIO_Callbacks[IRQNumber].userData = userdata;
        GPIO_Callbacks[IRQNumber].pinNumber = PinNumber;
    } else {
        GPIO_Callbacks[IRQNumber].callback = (GPIO_Callback_t)Default_GPIO_Callback;
        GPIO_Callbacks[IRQNumber].userData = NULL;
        GPIO_Callbacks[IRQNumber].pinNumber = -1;
    }
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI0 line.
 */
void EXTI0_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI0].callback(GPIO_Callbacks[IRQ_NO_EXTI0].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI0].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI0].pinNumber);
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI1 line.
 */
void EXTI1_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI1].callback(GPIO_Callbacks[IRQ_NO_EXTI1].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI1].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI1].pinNumber);
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI2 line.
 */
void EXTI2_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI2].callback(GPIO_Callbacks[IRQ_NO_EXTI2].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI2].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI2].pinNumber);
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI3 line.
 */
void EXTI3_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI3].callback(GPIO_Callbacks[IRQ_NO_EXTI3].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI3].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI3].pinNumber);
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI4 line.
 */
void EXTI4_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI4].callback(GPIO_Callbacks[IRQ_NO_EXTI4].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI4].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI4].pinNumber);
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI lines 5 to 9.
 */
void EXTI9_5_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI9_5].callback(GPIO_Callbacks[IRQ_NO_EXTI9_5].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI9_5].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI9_5].pinNumber);
}

/**
 * @brief Interrupt Service Routine (ISR) for EXTI lines 10 to 15.
 */
void EXTI15_10_IRQHandler(void){

    GPIO_Callbacks[IRQ_NO_EXTI15_10].callback(GPIO_Callbacks[IRQ_NO_EXTI15_10].userData);
    if(GPIO_Callbacks[IRQ_NO_EXTI15_10].pinNumber != -1)
            EXTI->CPUPR1 |= (1 << GPIO_Callbacks[IRQ_NO_EXTI15_10].pinNumber);

}
      
