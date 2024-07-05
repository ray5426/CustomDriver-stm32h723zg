/*
 * @file stm32h723xx.h
 *
 *  Created on: June 25, 2024
 *      Author: jay
 *
 *  
 *  @brief This file is part of the STM32H723xx Peripheral Access Layer. It provides
 *  definitions and functions to access the memory-mapped registers of the
 *  STM32H723xx microcontroller.
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

#ifndef INC_STM32H723XX_H_
#define INC_STM32H723XX_H_

#include <stdint.h>

#define __IO volatile

/*
 * @brief Base addresses of flash and sram
 */
#define FLASH_BASEADDR            0x08000000U  /**< Base address of Flash memory */
#define RAM_BASEADDR              0x20000000U  /**< Base address of SRAM */
#define NVIC_BASEADDR             0xE000E100U  /**< Base address of NVIC */

/*
 * @brief Number of priority bits implemented in NVIC
 */
#define __NVIC_PRIO_BITS          4U           /**< Number of priority bits */

/*
 * @brief Base addresses of different buses
 */
#define PERIPHERAL_BASEADDR       0x40000000U  /**< Base address of peripheral registers */
#define APB1_BASEADDR             PERIPHERAL_BASEADDR  /**< Base address of APB1 bus */
#define APB2_BASEADDR             0x40010000U  /**< Base address of APB2 bus */
#define AHB1_BASEADDR             0x40020000U  /**< Base address of AHB1 bus */
#define AHB2_BASEADDR             0x48020000U  /**< Base address of AHB2 bus */
#define APB3_BASEADDR             0x50000000U  /**< Base address of APB3 bus */
#define AHB3_BASEADDR             0x51000000U  /**< Base address of AHB3 bus */
#define APB4_BASEADDR             0x58000000U  /**< Base address of APB4 bus */
#define AHB4_BASEADDR             0x58020000U  /**< Base address of AHB4 bus */

/*
 * @brief Base addresses of GPIO ports
 */
#define GPIOA_BASEADDR            (AHB4_BASEADDR + 0x0000U)  /**< Base address of GPIOA */
#define GPIOB_BASEADDR            (AHB4_BASEADDR + 0x0400U)  /**< Base address of GPIOB */
#define GPIOC_BASEADDR            (AHB4_BASEADDR + 0x0800U)  /**< Base address of GPIOC */
#define GPIOD_BASEADDR            (AHB4_BASEADDR + 0x0C00U)  /**< Base address of GPIOD */
#define GPIOE_BASEADDR            (AHB4_BASEADDR + 0x1000U)  /**< Base address of GPIOE */
#define GPIOF_BASEADDR            (AHB4_BASEADDR + 0x1400U)  /**< Base address of GPIOF */
#define GPIOG_BASEADDR            (AHB4_BASEADDR + 0x1800U)  /**< Base address of GPIOG */
#define GPIOH_BASEADDR            (AHB4_BASEADDR + 0x1C00U)  /**< Base address of GPIOH */
#define GPIOJ_BASEADDR            (AHB4_BASEADDR + 0x2400U)  /**< Base address of GPIOJ */
#define GPIOK_BASEADDR            (AHB4_BASEADDR + 0x2800U)  /**< Base address of GPIOK */

/*
 * @brief Base addresses of I2C peripherals
 */
#define I2C_1_BASEADDR            (APB1_BASEADDR + 0x5400U)  /**< Base address of I2C1 */
#define I2C_2_BASEADDR            (APB1_BASEADDR + 0x5800U)  /**< Base address of I2C2 */
#define I2C_3_BASEADDR            (APB1_BASEADDR + 0x5C00U)  /**< Base address of I2C3 */
#define I2C_4_BASEADDR            (APB4_BASEADDR + 0x1C00U)  /**< Base address of I2C4 */
#define I2C_5_BASEADDR            (APB1_BASEADDR + 0x6400U)  /**< Base address of I2C5 */

/*
 * @brief Base addresses of SPI peripherals
 */
#define SPI_1_BASEADDR            (APB2_BASEADDR + 0x3000U)  /**< Base address of SPI1 */
#define SPI_2_BASEADDR            (APB1_BASEADDR + 0x3800U)  /**< Base address of SPI2 */
#define SPI_3_BASEADDR            (APB1_BASEADDR + 0x3C00U)  /**< Base address of SPI3 */
#define SPI_4_BASEADDR            (APB2_BASEADDR + 0x3400U)  /**< Base address of SPI4 */
#define SPI_5_BASEADDR            (APB2_BASEADDR + 0x5000U)  /**< Base address of SPI5 */
#define SPI_6_BASEADDR            (APB4_BASEADDR + 0x1400U)  /**< Base address of SPI6 */

/*
 * @brief Base addresses of UART/USART peripherals
 */
#define USART_1_BASEADDR          (APB2_BASEADDR + 0x1000U)  /**< Base address of USART1 */
#define USART_2_BASEADDR          (APB1_BASEADDR + 0x4400U)  /**< Base address of USART2 */
#define USART_3_BASEADDR          (APB1_BASEADDR + 0x4800U)  /**< Base address of USART3 */
#define UART_4_BASEADDR           (APB1_BASEADDR + 0x4C00U)  /**< Base address of UART4 */
#define UART_5_BASEADDR           (APB1_BASEADDR + 0x5000U)  /**< Base address of UART5 */
#define USART_6_BASEADDR          (APB2_BASEADDR + 0x1400U)  /**< Base address of USART6 */
#define UART_7_BASEADDR           (APB1_BASEADDR + 0x7800U)  /**< Base address of UART7 */
#define UART_8_BASEADDR           (APB1_BASEADDR + 0x7C00U)  /**< Base address of UART8 */
#define UART_9_BASEADDR           (APB2_BASEADDR + 0x1800U)  /**< Base address of UART9 */
#define USART_10_BASEADDR         (APB2_BASEADDR + 0x1C00U)  /**< Base address of USART10 */
#define LPUART_1_BASEADDR         (APB4_BASEADDR + 0x0C00U)  /**< Base address of LPUART1 */

/*
 * @brief Base addresses of other peripherals
 */
#define EXTI_BASEADDR             (APB4_BASEADDR + 0x0000U)  /**< Base address of EXTI */
#define SYSCFG_BASEADDR           (APB4_BASEADDR + 0x0400U)  /**< Base address of SYSCFG */
#define RCC_BASEADDR              (AHB4_BASEADDR + 0x4400U)  /**< Base address of RCC */

/*
 * @brief GPIO register structure
 */
typedef struct {
    __IO uint32_t MODER;          /**< 0x00: GPIO port mode register */
    __IO uint32_t OTYPER;         /**< 0x04: GPIO port output type register */
    __IO uint32_t OSPEEDR;        /**< 0x08: GPIO port output speed register */
    __IO uint32_t PUPDR;          /**< 0x0C: GPIO port pull-up/pull-down register */
    __IO uint32_t IDR;            /**< 0x10: GPIO port input data register */
    __IO uint32_t ODR;            /**< 0x14: GPIO port output data register */
    __IO uint32_t BSRR;           /**< 0x18: GPIO port bit set/reset register */
    __IO uint32_t LCKR;           /**< 0x1C: GPIO port configuration lock register */
    __IO uint32_t AFR[2];         /**< 0x20 - 0x24: GPIO alternate function registers */
} GPIO_RegDef_t;

/*
 * @brief RCC Register Definition Structure
 */
typedef struct {
    __IO uint32_t CR;                /**< 0x00: Clock control register */
    __IO uint32_t HSICFGR;           /**< 0x04: HSI configuration register */
    __IO uint32_t CRRCR;             /**< 0x08: Clock recovery RC register */
    __IO uint32_t CSICFGR;           /**< 0x0C: CSI configuration register */
    __IO uint32_t CFGR;              /**< 0x10: Clock configuration register */
    uint32_t RESERVED1;              /**< 0x14: Reserved */
    __IO uint32_t D1CFGR;            /**< 0x18: Domain 1 configuration register */
    __IO uint32_t D2CFGR;            /**< 0x1C: Domain 2 configuration register */
    __IO uint32_t D3CFGR;            /**< 0x20: Domain 3 configuration register */
    uint32_t RESERVED2;              /**< 0x24: Reserved */
    __IO uint32_t PLLCKSELR;         /**< 0x28: PLL clock source selection register */
    __IO uint32_t PLLCFGR;           /**< 0x2C: PLL configuration register */
    __IO uint32_t PLL1DIVR;          /**< 0x30: PLL1 dividers register */
    __IO uint32_t PLL1FRACR;         /**< 0x34: PLL1 fractional register */
    __IO uint32_t PLL2DIVR;          /**< 0x38: PLL2 dividers register */
    __IO uint32_t PLL2FRACR;         /**< 0x3C: PLL2 fractional register */
    __IO uint32_t PLL3DIVR;          /**< 0x40: PLL3 dividers register */
    __IO uint32_t PLL3FRACR;         /**< 0x44: PLL3 fractional register */
    uint32_t RESERVED3;              /**< 0x48: Reserved */
    __IO uint32_t D1CCIPR;           /**< 0x4C: Domain 1 kernel clock configuration register */
    __IO uint32_t D2CCIP1R;          /**< 0x50: Domain 2 kernel clock configuration register 1 */
    __IO uint32_t D2CCIP2R;          /**< 0x54: Domain 2 kernel clock configuration register 2 */
    __IO uint32_t D3CCIPR;           /**< 0x58: Domain 3 kernel clock configuration register */
    uint32_t RESERVED4;              /**< 0x5C: Reserved */
    __IO uint32_t CIER;              /**< 0x60: Clock interrupt enable register */
    __IO uint32_t CIFR;              /**< 0x64: Clock interrupt flag register */
    __IO uint32_t CICR;              /**< 0x68: Clock interrupt clear register */
    uint32_t RESERVED5;              /**< 0x6C: Reserved */
    __IO uint32_t BDCR;              /**< 0x70: Backup domain control register */
    __IO uint32_t CSR;               /**< 0x74: Clock control and status register */
    uint32_t RESERVED6;              /**< 0x78: Reserved */
    __IO uint32_t AHB3RSTR;          /**< 0x7C: AHB3 peripheral reset register */
    __IO uint32_t AHB1RSTR;          /**< 0x80: AHB1 peripheral reset register */
    __IO uint32_t AHB2RSTR;          /**< 0x84: AHB2 peripheral reset register */
    __IO uint32_t AHB4RSTR;          /**< 0x88: AHB4 peripheral reset register */
    __IO uint32_t APB3RSTR;          /**< 0x8C: APB3 peripheral reset register */
    __IO uint32_t APB1LRSTR;         /**< 0x90: APB1L peripheral reset register */
    __IO uint32_t APB1HRSTR;         /**< 0x94: APB1H peripheral reset register */
    __IO uint32_t APB2RSTR;          /**< 0x98: APB2 peripheral reset register */
    __IO uint32_t APB4RSTR;          /**< 0x9C: APB4 peripheral reset register */
    __IO uint32_t GCR;               /**< 0xA0: Global control register */
    uint32_t RESERVED7;              /**< 0xA4: Reserved */
    __IO uint32_t D3AMR;             /**< 0xA8: Domain 3 activity monitoring register */
    uint32_t RESERVED8[9];           /**< 0xAC - 0xCC: Reserved */
    __IO uint32_t RSR;               /**< 0xD0: Reset status register */
    __IO uint32_t AHB3ENR;           /**< 0xD4: AHB3 peripheral clock enable register */
    __IO uint32_t AHB1ENR;           /**< 0xD8: AHB1 peripheral clock enable register */
    __IO uint32_t AHB2ENR;           /**< 0xDC: AHB2 peripheral clock enable register */
    __IO uint32_t AHB4ENR;           /**< 0xE0: AHB4 peripheral clock enable register */
    __IO uint32_t APB3ENR;           /**< 0xE4: APB3 peripheral clock enable register */
    __IO uint32_t APB1LENR;          /**< 0xE8: APB1L peripheral clock enable register */
    __IO uint32_t APB1HENR;          /**< 0xEC: APB1H peripheral clock enable register */
    __IO uint32_t APB2ENR;           /**< 0xF0: APB2 peripheral clock enable register */
    __IO uint32_t APB4ENR;           /**< 0xF4: APB4 peripheral clock enable register */
    uint32_t RESERVED9;              /**< 0xF8: Reserved */
    __IO uint32_t AHB3LPENR;         /**< 0xFC: AHB3 peripheral clock enable in low power mode register */
    __IO uint32_t AHB1LPENR;         /**< 0x100: AHB1 peripheral clock enable in low power mode register */
    __IO uint32_t AHB2LPENR;         /**< 0x104: AHB2 peripheral clock enable in low power mode register */
    __IO uint32_t AHB4LPENR;         /**< 0x108: AHB4 peripheral clock enable in low power mode register */
    __IO uint32_t APB3LPENR;         /**< 0x10C: APB3 peripheral clock enable in low power mode register */
    __IO uint32_t APB1LLPENR;        /**< 0x110: APB1L peripheral clock enable in low power mode register */
    __IO uint32_t APB1HLPENR;        /**< 0x114: APB1H peripheral clock enable in low power mode register */
    __IO uint32_t APB2LPENR;         /**< 0x118: APB2 peripheral clock enable in low power mode register */
    __IO uint32_t APB4LPENR;         /**< 0x11C: APB4 peripheral clock enable in low power mode register */
    uint32_t RESERVED10[4];          /**< 0x120 - 0x12C: Reserved */
    __IO uint32_t C1_RSR;            /**< 0x130: CPU1 reset status register */
    __IO uint32_t C1_AHB3ENR;        /**< 0x134: CPU1 AHB3 peripheral clock enable register */
    __IO uint32_t C1_AHB1ENR;        /**< 0x138: CPU1 AHB1 peripheral clock enable register */
    __IO uint32_t C1_AHB2ENR;        /**< 0x13C: CPU1 AHB2 peripheral clock enable register */
    __IO uint32_t C1_AHB4ENR;        /**< 0x140: CPU1 AHB4 peripheral clock enable register */
    __IO uint32_t C1_APB3ENR;        /**< 0x144: CPU1 APB3 peripheral clock enable register */
    __IO uint32_t C1_APB1LENR;       /**< 0x148: CPU1 APB1L peripheral clock enable register */
    __IO uint32_t C1_APB1HENR;       /**< 0x14C: CPU1 APB1H peripheral clock enable register */
    __IO uint32_t C1_APB2ENR;        /**< 0x150: CPU1 APB2 peripheral clock enable register */
    __IO uint32_t C1_APB4ENR;        /**< 0x154: CPU1 APB4 peripheral clock enable register */
    uint32_t RESERVED11;             /**< 0x158: Reserved */
    __IO uint32_t C1_AHB3LPENR;      /**< 0x15C: CPU1 AHB3 peripheral clock enable in low power mode register */
    __IO uint32_t C1_AHB1LPENR;      /**< 0x160: CPU1 AHB1 peripheral clock enable in low power mode register */
    __IO uint32_t C1_AHB2LPENR;      /**< 0x164: CPU1 AHB2 peripheral clock enable in low power mode register */
    __IO uint32_t C1_AHB4LPENR;      /**< 0x168: CPU1 AHB4 peripheral clock enable in low power mode register */
    __IO uint32_t C1_APB3LPENR;      /**< 0x16C: CPU1 APB3 peripheral clock enable in low power mode register */
    __IO uint32_t C1_APB1LLPENR;     /**< 0x170: CPU1 APB1L peripheral clock enable in low power mode register */
    __IO uint32_t C1_APB1HLPENR;     /**< 0x174: CPU1 APB1H peripheral clock enable in low power mode register */
    __IO uint32_t C1_APB2LPENR;      /**< 0x178: CPU1 APB2 peripheral clock enable in low power mode register */
    __IO uint32_t C1_APB4LPENR;      /**< 0x17C: CPU1 APB4 peripheral clock enable in low power mode register */
    uint32_t RESERVED12[32];         /**< 0x180 - 0x1FC: Reserved */
} RCC_RegDef_t;

/*
 * @brief EXTI Register Definition Structure
 */
typedef struct {
    __IO uint32_t RTSR1;           /**< 0x00: Rising Trigger selection register 1 */
    __IO uint32_t FTSR1;           /**< 0x04: Falling Trigger selection register 1 */
    __IO uint32_t SWIER1;          /**< 0x08: Software interrupt event register 1 */
    __IO uint32_t D3PMR1;          /**< 0x0C: Domain 3 pending mask register 1 */
    __IO uint32_t D3PCR1L;         /**< 0x10: Domain 3 pending clear register 1 low */
    __IO uint32_t D3PCR1H;         /**< 0x14: Domain 3 pending clear register 1 high */
    uint32_t RESERVED0[2];         /**< 0x18 - 0x1C: Reserved */
    __IO uint32_t RTSR2;           /**< 0x20: Rising Trigger selection register 2 */
    __IO uint32_t FTSR2;           /**< 0x24: Falling Trigger selection register 2 */
    __IO uint32_t SWIER2;          /**< 0x28: Software interrupt event register 2 */
    __IO uint32_t D3PMR2;          /**< 0x2C: Domain 3 pending mask register 2 */
    __IO uint32_t D3PCR2L;         /**< 0x30: Domain 3 pending clear register 2 low */
    __IO uint32_t D3PCR2H;         /**< 0x34: Domain 3 pending clear register 2 high */
    uint32_t RESERVED1[2];         /**< 0x38 - 0x3C: Reserved */
    __IO uint32_t RTSR3;           /**< 0x40: Rising Trigger selection register 3 */
    __IO uint32_t FTSR3;           /**< 0x44: Falling Trigger selection register 3 */
    __IO uint32_t SWIER3;          /**< 0x48: Software interrupt event register 3 */
    __IO uint32_t D3PMR3;          /**< 0x4C: Domain 3 pending mask register 3 */
    __IO uint32_t D3PCR3L;         /**< 0x50: Domain 3 pending clear register 3 low */
    __IO uint32_t D3PCR3H;         /**< 0x54: Domain 3 pending clear register 3 high */
    uint32_t RESERVED2[10];        /**< 0x58 - 0x7C: Reserved */
    __IO uint32_t CPUIMR1;         /**< 0x80: CPU interrupt mask register 1 */
    __IO uint32_t CPUEMR1;         /**< 0x84: CPU event mask register 1 */
    __IO uint32_t CPUPR1;          /**< 0x88: CPU pending register 1 */
    uint32_t RESERVED3;            /**< 0x8C: Reserved */
    __IO uint32_t CPUIMR2;         /**< 0x90: CPU interrupt mask register 2 */
    __IO uint32_t CPUEMR2;         /**< 0x94: CPU event mask register 2 */
    __IO uint32_t CPUPR2;          /**< 0x98: CPU pending register 2 */
    uint32_t RESERVED4;            /**< 0x9C: Reserved */
    __IO uint32_t CPUIMR3;         /**< 0xA0: CPU interrupt mask register 3 */
    __IO uint32_t CPUEMR3;         /**< 0xA4: CPU event mask register 3 */
    __IO uint32_t CPUPR3;          /**< 0xA8: CPU pending register 3 */
    uint32_t RESERVED5[5];         /**< 0xAC - 0xBC: Reserved */
} EXTI_RegDef_t;

/*
 * @brief SYSCFG Register Definition Structure
 */
typedef struct {
    uint32_t RESERVED0;            /**< 0x00: Reserved */
    __IO uint32_t PMCR;            /**< 0x04: Peripheral mode configuration register */
    __IO uint32_t EXTICR[4];       /**< 0x08 - 0x14: External interrupt configuration registers */
    __IO uint32_t CFGR;            /**< 0x18: Configuration register */
    uint32_t RESERVED1;            /**< 0x1C: Reserved */
    __IO uint32_t CCCSR;           /**< 0x20: Compensation cell control/status register */
    __IO uint32_t CCVR;            /**< 0x24: Compensation cell value register */
    __IO uint32_t CCCR;            /**< 0x28: Compensation cell code register */
    uint32_t RESERVED2;            /**< 0x2C: Reserved */
    __IO uint32_t ADC2ALT;         /**< 0x30: ADC2 alternate trigger enable register */
    uint32_t RESERVED3[60];        /**< 0x34 - 0x120: Reserved */
    __IO uint32_t PKGR;            /**< 0x124: Package register */
    uint32_t RESERVED4[118];       /**< 0x128 - 0x2FC: Reserved */
    __IO uint32_t UR[18];          /**< 0x300 - 0x348: User registers */
} SYSCFG_RegDef_t;

/*
 * @brief NVIC Register Definition Structure
 */
typedef struct {
    __IO uint32_t ISER[8];         /**< 0x000 - 0x01C: Interrupt Set-Enable Registers */
    uint32_t RESERVED0[24];        /**< 0x020 - 0x07C: Reserved */
    __IO uint32_t ICER[8];         /**< 0x080 - 0x09C: Interrupt Clear-Enable Registers */
    uint32_t RESERVED1[24];        /**< 0x0A0 - 0x0FC: Reserved */
    __IO uint32_t ISPR[8];         /**< 0x100 - 0x11C: Interrupt Set-Pending Registers */
    uint32_t RESERVED2[24];        /**< 0x120 - 0x17C: Reserved */
    __IO uint32_t ICPR[8];         /**< 0x180 - 0x19C: Interrupt Clear-Pending Registers */
    uint32_t RESERVED3[24];        /**< 0x1A0 - 0x1FC: Reserved */
    __IO uint32_t IABR[8];         /**< 0x200 - 0x21C: Interrupt Active Bit Registers */
    uint32_t RESERVED4[56];        /**< 0x220 - 0x2FC: Reserved */
    __IO uint32_t IPR[60];         /**< 0x300 - 0x3EC: Interrupt Priority Registers */
    uint32_t RESERVED5[644];       /**< 0x3F0 - 0xDFC: Reserved */
    __IO uint32_t STIR;            /**< 0xE00: Software Trigger Interrupt Register */
} NVIC_RegDef_t;

/*
 * @brief GPIO port base addresses
 */
#define GPIOA                    ((GPIO_RegDef_t *) GPIOA_BASEADDR) /**< GPIO Port A base address */
#define GPIOB                    ((GPIO_RegDef_t *) GPIOB_BASEADDR) /**< GPIO Port B base address */
#define GPIOC                    ((GPIO_RegDef_t *) GPIOC_BASEADDR) /**< GPIO Port C base address */
#define GPIOD                    ((GPIO_RegDef_t *) GPIOD_BASEADDR) /**< GPIO Port D base address */
#define GPIOE                    ((GPIO_RegDef_t *) GPIOE_BASEADDR) /**< GPIO Port E base address */
#define GPIOF                    ((GPIO_RegDef_t *) GPIOF_BASEADDR) /**< GPIO Port F base address */
#define GPIOG                    ((GPIO_RegDef_t *) GPIOG_BASEADDR) /**< GPIO Port G base address */
#define GPIOH                    ((GPIO_RegDef_t *) GPIOH_BASEADDR) /**< GPIO Port H base address */
#define GPIOJ                    ((GPIO_RegDef_t *) GPIOJ_BASEADDR) /**< GPIO Port J base address */
#define GPIOK                    ((GPIO_RegDef_t *) GPIOK_BASEADDR) /**< GPIO Port K base address */

/*
 * @brief RCC, EXTI, SYSCFG, NVIC base addresses
 */
#define RCC                    	 ((RCC_RegDef_t *) RCC_BASEADDR)    /**< RCC base address */
#define EXTI                     ((EXTI_RegDef_t *) EXTI_BASEADDR)   /**< EXTI base address */
#define SYSCFG                   ((SYSCFG_RegDef_t *) SYSCFG_BASEADDR) /**< SYSCFG base address */
#define NVIC                     ((NVIC_RegDef_t *) NVIC_BASEADDR)   /**< NVIC base address */

/*
 * @brief RCC AHB4ENR register bit positions
 */
#define RCC_AHB4ENR_GPIOAEN_Pos (0U)   /**< GPIOA clock enable bit position */
#define RCC_AHB4ENR_GPIOBEN_Pos (1U)   /**< GPIOB clock enable bit position */
#define RCC_AHB4ENR_GPIOCEN_Pos (2U)   /**< GPIOC clock enable bit position */
#define RCC_AHB4ENR_GPIODEN_Pos (3U)   /**< GPIOD clock enable bit position */
#define RCC_AHB4ENR_GPIOEEN_Pos (4U)   /**< GPIOE clock enable bit position */
#define RCC_AHB4ENR_GPIOFEN_Pos (5U)   /**< GPIOF clock enable bit position */
#define RCC_AHB4ENR_GPIOGEN_Pos (6U)   /**< GPIOG clock enable bit position */
#define RCC_AHB4ENR_GPIOHEN_Pos (7U)   /**< GPIOH clock enable bit position */
#define RCC_AHB4ENR_GPIOJEN_Pos (9U)   /**< GPIOJ clock enable bit position */
#define RCC_AHB4ENR_GPIOKEN_Pos (10U)  /**< GPIOK clock enable bit position */
#define RCC_AHB4ENR_SYSCFGEN_Pos (1U)  /**< SYSCFG clock enable bit position */

/*
 * @brief RCC AHB4RSTR register bit positions
 */
#define RCC_AHB4RSTS_GPIOARSTR_Pos (0U)    /**< GPIOA reset bit position */
#define RCC_AHB4RSTS_GPIOBRSTR_Pos (1U)    /**< GPIOB reset bit position */
#define RCC_AHB4RSTS_GPIOCRSTR_Pos (2U)    /**< GPIOC reset bit position */
#define RCC_AHB4RSTS_GPIODRSTR_Pos (3U)    /**< GPIOD reset bit position */
#define RCC_AHB4RSTS_GPIOERSTR_Pos (4U)    /**< GPIOE reset bit position */
#define RCC_AHB4RSTS_GPIOFRSTR_Pos (5U)    /**< GPIOF reset bit position */
#define RCC_AHB4RSTS_GPIOGRSTR_Pos (6U)    /**< GPIOG reset bit position */
#define RCC_AHB4RSTS_GPIOHRSTR_Pos (7U)    /**< GPIOH reset bit position */
#define RCC_AHB4RSTS_GPIOJRSTR_Pos (9U)    /**< GPIOJ reset bit position */
#define RCC_AHB4RSTS_GPIOKRSTR_Pos (10U)   /**< GPIOK reset bit position */
#define RCC_AHB4RSTS_SYSCFGENR_Pos (1U)    /**< SYSCFG reset bit position */

/*
 * @brief SYSCFG EXTIx register bit positions
 */
#define SYSCFG_EXTIx_GPIOA_Pos (0U)   /**< EXTI GPIOA interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOB_Pos (1U)   /**< EXTI GPIOB interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOC_Pos (2U)   /**< EXTI GPIOC interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOD_Pos (3U)   /**< EXTI GPIOD interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOE_Pos (4U)   /**< EXTI GPIOE interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOF_Pos (5U)   /**< EXTI GPIOF interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOG_Pos (6U)   /**< EXTI GPIOG interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOH_Pos (7U)   /**< EXTI GPIOH interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOJ_Pos (9U)   /**< EXTI GPIOJ interrupt configuration bit position */
#define SYSCFG_EXTIx_GPIOK_Pos (10U)  /**< EXTI GPIOK interrupt configuration bit position */

/*
 * @brief GPIO port clock enable/disable macros
 */
#define GPIOA_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOAEN_Pos)  )    /**< Enable GPIOA clock */
#define GPIOA_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOAEN_Pos) )    /**< Disable GPIOA clock */

#define GPIOB_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOBEN_Pos)  )    /**< Enable GPIOB clock */
#define GPIOB_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOBEN_Pos) )    /**< Disable GPIOB clock */

#define GPIOC_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOCEN_Pos)  )    /**< Enable GPIOC clock */
#define GPIOC_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOCEN_Pos) )    /**< Disable GPIOC clock */

#define GPIOD_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIODEN_Pos)  )    /**< Enable GPIOD clock */
#define GPIOD_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIODEN_Pos) )    /**< Disable GPIOD clock */

#define GPIOE_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOEEN_Pos)  )    /**< Enable GPIOE clock */
#define GPIOE_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOEEN_Pos) )    /**< Disable GPIOE clock */

#define GPIOF_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOFEN_Pos)  )    /**< Enable GPIOF clock */
#define GPIOF_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOFEN_Pos) )    /**< Disable GPIOF clock */

#define GPIOG_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOGEN_Pos)  )    /**< Enable GPIOG clock */
#define GPIOG_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOGEN_Pos) )    /**< Disable GPIOG clock */

#define GPIOH_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOHEN_Pos)  )    /**< Enable GPIOH clock */
#define GPIOH_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOHEN_Pos) )    /**< Disable GPIOH clock */

#define GPIOJ_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOJEN_Pos)  )    /**< Enable GPIOJ clock */
#define GPIOJ_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOJEN_Pos) )    /**< Disable GPIOJ clock */

#define GPIOK_PCLK_EN()     	 	 ( RCC->AHB4ENR |= ( 1 << RCC_AHB4ENR_GPIOKEN_Pos)  )    /**< Enable GPIOK clock */
#define GPIOK_PCLK_DI()     	 	 ( RCC->AHB4ENR &= ~( 1 << RCC_AHB4ENR_GPIOKEN_Pos) )    /**< Disable GPIOK clock */

#define GPIOx_PCLK_EN(pos)     	 	 ( RCC->AHB4ENR |= ( 1 << pos)  )    /**< Enable GPIO clock specified by position */
#define GPIOx_PCLK_DI(pos)     	 	 ( RCC->AHB4ENR &= ~( 1 << pos) )    /**< Disable GPIO clock specified by position */

/*
 * @brief SYSCFG Registers clock enable/disable macros
 */
#define SYSCFG_PCLK_EN()     	 	 ( RCC->APB4ENR |= ( 1 << RCC_AHB4ENR_SYSCFGEN_Pos)  )    /**< Enable SYSCFG clock */
#define SYSCFG_PCLK_DI()             ( RCC->APB4ENR &= ~( 1 << RCC_AHB4ENR_SYSCFGEN_Pos) )    /**< Disable SYSCFG clock */

/*
 * @brief GPIO port reset macros
 */
#define GPIOA_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOARSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOARSTR_Pos); \
    } while(0)

#define GPIOB_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOBRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOBRSTR_Pos); \
    } while(0)

#define GPIOC_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOCRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOCRSTR_Pos); \
    } while(0)

#define GPIOD_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIODRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIODRSTR_Pos); \
    } while(0)

#define GPIOE_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOERSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOERSTR_Pos); \
    } while(0)

#define GPIOF_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOFRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOFRSTR_Pos); \
    } while(0)

#define GPIOG_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOGRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOGRSTR_Pos); \
    } while(0)

#define GPIOH_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOHRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOHRSTR_Pos); \
    } while(0)

#define GPIOJ_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOJRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOJRSTR_Pos); \
    } while(0)

#define GPIOK_REG_RESET()       \
    do {                        \
        RCC->AHB4RSTR |= (1 << RCC_AHB4RSTS_GPIOKRSTR_Pos); \
        RCC->AHB4RSTR &= ~(1 << RCC_AHB4RSTS_GPIOKRSTR_Pos); \
    } while(0)

#define GPIOx_REG_RESET(pos)    \
    do {                        \
        RCC->AHB4RSTR |= (1 << pos); \
        RCC->AHB4RSTR &= ~(1 << pos); \
    } while(0)

/*
 * @brief SYSCFG Registers reset macros
 */
#define SYSCFG_REG_RESET()      \
    do {                        \
        RCC->APB4RSTR |= (1 << RCC_AHB4RSTS_SYSCFGENR_Pos); \
        RCC->APB4RSTR &= ~(1 << RCC_AHB4RSTS_SYSCFGENR_Pos); \
    } while(0)

/*
 * @brief Peripheral clock enable/disable macros
 */
#define I2C1_PCLK_EN()     	 	 ( RCC->APB1LENR   |= ( 1 << 21)  )    /**< Enable I2C1 clock */
#define I2C1_PCLK_DI()     	 	 ( RCC->APB1LENR   &= ~( 1 << 21) )    /**< Disable I2C1 clock */

#define SPI1_PCLK_EN()     	 	 ( RCC->APB2ENR   |= ( 1 << 12)  )    /**< Enable SPI1 clock */
#define SPI1_PCLK_DI()     	 	 ( RCC->APB2ENR   &= ~( 1 << 12) )    /**< Disable SPI1 clock */

#define USART3_PCLK_EN()     	 ( RCC->APB1LENR   |= ( 1 << 18)  )    /**< Enable USART3 clock */
#define USART3_PCLK_DI()     	 ( RCC->APB1LENR   &= ~( 1 << 18) )    /**< Disable USART3 clock */

/*
 * @brief State enumeration
 */
typedef enum{
    DISABLE = 0,    /**< State: Disabled */
    ENABLE = 1      /**< State: Enabled */
}State_t;

/*
 * @brief GPIO Pin State enumeration
 */
typedef enum{
    GPIO_PIN_RESET = 0, /**< GPIO Pin State: Reset */
    GPIO_PIN_SET = 1    /**< GPIO Pin State: Set */
}GPIO_PinState_t;

#endif /* INC_STM32H723XX_H_ */
