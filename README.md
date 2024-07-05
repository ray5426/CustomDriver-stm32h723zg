# STM32H723xx Peripheral Drivers

This repository contains low-level drivers for STM32H723xx microcontroller peripherals.

## Overview

This project aims to provide a clear understanding of basic peripherals such as GPIO, I2C, SPI, and USART on the STM32H723xx microcontroller.

## Contents

- [GPIO Driver](./driver/stm32h723xx_gpio_driver.h)
- [MCU file](./driver/stm32h723xx.h)

## GPIO Driver

The GPIO driver provides functions to initialize GPIO pins, read input states, and control output states. It includes interrupt handling and callback registration.

## TODO

- Implement PLL driver
- Implement I2C driver
- Implement SPI driver
- Implement USART driver
- Implement Low Power support
- Implement DMA support for the drivers

## Credits

This project was developed based on the knowledge acquired from the Udemy course "Learn bare metal driver development using Embedded C" by [FastBit Embedded].

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for more details.
