/*
 * @file stm32h723xx_gpio_driver.h
 *
 *  Created on: June 28, 2024
 *      Author: jay
 *
 *  
 *  @brief This file contains definitions and functions for GPIO peripheral operations
 *  on the STM32H723xx microcontroller.
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

#ifndef INC_STM32H723XX_GPIO_DRIVER_H_
#define INC_STM32H723XX_GPIO_DRIVER_H_

#include <stm32h723xx.h>

/**
 * @brief GPIO pin operating mode options.
 */
typedef enum{
    GPIO_MODE_IN = 0,           /**< Input mode */
    GPIO_MODE_OUT,              /**< Output mode */
    GPIO_MODE_ALTFN,            /**< Alternate function mode */
    GPIO_MODE_ANALOG,           /**< Analog mode */
    GPIO_MODE_IT_FT,            /**< Interrupt Falling edge trigger */
    GPIO_MODE_IT_RT,            /**< Interrupt Rising edge trigger */
    GPIO_MODE_IT_RFT            /**< Interrupt Rising-Falling edge trigger */
}GPIO_Modetype_t;

/**
 * @brief GPIO pin output speed options.
 */
typedef enum{
    GPIO_SPEED_LOW,             /**< Low speed */
    GPIO_SPEED_MEDIUM,          /**< Medium speed */
    GPIO_SPEED_FAST,            /**< Fast speed */
    GPIO_SPEED_HIGH             /**< High speed */
}GPIO_OSpeed_t;

/**
 * @brief GPIO pin output type options.
 */
typedef enum{
    GPIO_OP_TYPE_PP,            /**< Push-Pull output type */
    GPIO_OP_TYPE_OD             /**< Open-Drain output type */
}GPIO_PinOType_t;

/**
 * @brief GPIO pin pull-up/pull-down configuration options.
 */
typedef enum {
    GPIO_NO_PUPD,               /**< No pull-up or pull-down */
    GPIO_PU,                    /**< Pull-up */
    GPIO_PD                     /**< Pull-down */
}GPIO_PinPuPdControl_t;

/**
 * @brief GPIO pin number definitions.
 */
typedef enum {
    GPIO_PIN_0  = 0,
    GPIO_PIN_1  = 1,
    GPIO_PIN_2  = 2,
    GPIO_PIN_3  = 3,
    GPIO_PIN_4  = 4,
    GPIO_PIN_5  = 5,
    GPIO_PIN_6  = 6,
    GPIO_PIN_7  = 7,
    GPIO_PIN_8  = 8,
    GPIO_PIN_9  = 9,
    GPIO_PIN_10 = 10,
    GPIO_PIN_11 = 11,
    GPIO_PIN_12 = 12,
    GPIO_PIN_13 = 13,
    GPIO_PIN_14 = 14,
    GPIO_PIN_15 = 15
} GPIO_PinNumber_t;

#define MAX_PIN_COUNT       16U

/**
 * @brief GPIO pin configuration structure.
 */
typedef struct{
    GPIO_PinNumber_t GPIO_PinNumber;                /**< GPIO pin number */
    GPIO_Modetype_t GPIO_PinMode;                   /**< GPIO pin mode */
    GPIO_OSpeed_t GPIO_PinSpeed;                    /**< GPIO pin output speed */
    GPIO_PinPuPdControl_t GPIO_PinPuPdControl;      /**< GPIO pin pull-up/pull-down configuration */
    GPIO_PinOType_t GPIO_PinOType;                  /**< GPIO pin output type */
    uint8_t GPIO_PinAltFunMode;                     /**< GPIO pin alternate function mode */
}GPIO_PinConfig_t;

/**
 * @brief GPIO handle structure for GPIO configuration.
 */
typedef struct{
    GPIO_RegDef_t *pGPIOx;              /**< Pointer to GPIO peripheral base address */
    GPIO_PinConfig_t GPIO_PinConfig;    /**< GPIO pin configuration settings */
}GPIO_Handle_t;

/**
 * @brief External Interrupt (EXTI) IRQ number definitions.
 */
typedef enum{
    IRQ_NO_EXTI0 = 0,
    IRQ_NO_EXTI1,
    IRQ_NO_EXTI2,
    IRQ_NO_EXTI3,
    IRQ_NO_EXTI4,
    IRQ_NO_EXTI9_5,
    IRQ_NO_EXTI15_10
}EXTI_IRQNumber;


#define __MAX_GPIO_EXTI_IRQ         7U

/**
 * @brief EXTI line number definitions.
 */
typedef enum {
    EXTI_LINE_0 = 6,
    EXTI_LINE_1 = 7,
    EXTI_LINE_2 = 8,
    EXTI_LINE_3 = 9,
    EXTI_LINE_4 = 10,
    EXTI_LINE_5 = 23,
    EXTI_LINE_6 = 23,
    EXTI_LINE_7 = 23,
    EXTI_LINE_8 = 23,
    EXTI_LINE_9 =  23,
    EXTI_LINE_10 = 40,
    EXTI_LINE_11 = 40,
    EXTI_LINE_12 = 40,
    EXTI_LINE_13 = 40,
    EXTI_LINE_14 = 40,
    EXTI_LINE_15 = 40
} EXTI_LineNumber;

/********************************************************************
 * 			APIs supported by the driver
 *******************************************************************/

/**
 * @brief Enable or disable GPIO peripheral clock.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 * @param state Enable or disable state (ENABLE or DISABLE)
 */
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, State_t state );

/**
 * @brief Initialize GPIO pin.
 *
 * @param pGPIOHandle Pointer to GPIO handle structure containing GPIO pin configuration
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief De-initialize GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Read the input state of a specific GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 * @param PinNumber GPIO pin number
 * @return uint8_t GPIO pin input state (HIGH or LOW)
 */
uint8_t GPIO_ReadInPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t PinNumber);

/**
 * @brief Read the input state of all GPIO pins in a GPIO port.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 * @return uint16_t GPIO port input state (16-bit value)
 */
uint16_t GPIO_ReadInPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Write a state to a specific GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 * @param PinNumber GPIO pin number
 * @param pinState State to be written to the GPIO pin (HIGH or LOW)
 */
void GPIO_WriteOutPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t PinNumber, GPIO_PinState_t pinState);

/**
 * @brief Write a value to a GPIO port.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 * @param value Value to be written to the GPIO port (16-bit value)
 */
void GPIO_WriteOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/**
 * @brief Toggle the current state of a specific GPIO pin.
 *
 * @param pGPIOx Pointer to GPIO peripheral base address
 * @param PinNumber GPIO pin number
 */
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx, GPIO_PinNumber_t PinNumber);

/**
 * @brief Configure interrupt for a GPIO pin.
 *
 * @param PinNumber GPIO pin number
 * @param state Enable or disable state (ENABLE or DISABLE)
 */
void GPIO_IRQConfig(GPIO_PinNumber_t PinNumber, State_t state);

/**
 * @brief Configure interrupt priority for a GPIO pin.
 *
 * @param PinNumber GPIO pin number
 * @param IRQPriority Interrupt priority value
 */
void GPIO_IRQPriorityConfig(GPIO_PinNumber_t PinNumber, uint8_t IRQPriority);

/**
 * @brief Function pointer type for GPIO callback functions.
 *
 * @param void* User data pointer for callback functions
 */
typedef void (*GPIO_Callback_t)(void *);

/**
 * @brief Enable GPIO callbacks.
 */
void GPIO_EnableCallbacks(void);

/**
 * @brief Default GPIO callback function.
 */
void Default_GPIO_Callback(void);

/**
 * @brief Register a callback function for a GPIO pin.
 *
 * @param PinNumber GPIO pin number
 * @param callback Callback function to be registered
 * @param userdata User data pointer for callback function
 */
void GPIO_RegisterCallback(GPIO_PinNumber_t PinNumber, GPIO_Callback_t callback, void *userdata);

#endif /* INC_STM32H723XX_GPIO_DRIVER_H_ */
