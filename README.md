# STM32H723xx Peripheral Drivers

This repository contains low-level drivers for STM32H723xx microcontroller peripherals. The drivers are structured to be easily integrated into STM32 projects, offering clear APIs and documentation for straightforward implementation.

## Overview

This project aims to provide a clear understanding of basic peripherals such as GPIO, I2C, SPI, and USART on the STM32H723xx microcontroller.

## Contents

- [GPIO Driver](./driver/Inc/stm32h723xx_gpio_driver.h)
- [MCU file](./driver/Inc/stm32h723xx.h)
- [I2C Driver](./driver/Inc/stm32h723xx_i2c_driver.h) (Work in Progress)
- [SPI Driver](./driver/Inc/stm32h723xx_spi_driver.h) (Work in Progress)
- [USART Driver](./driver/Inc/stm32h723xx_usart_driver.h) (Work in Progress)

## GPIO Driver

- GPIO pin initialization (mode, speed, type, pull-up/pull-down)
- Reading from GPIO pins (single pin and whole port)
- Writing to GPIO pins (single pin and whole port)
- Pin toggling
- Interrupt handling for GPIO pins

## TODO

- Implement I2C driver
- Implement SPI driver
- Implement USART driver
- Implement PLL driver

## Planned Enhancements:
1. DMA Support: Future updates will incorporate DMA capabilities for enhanced data transfer efficiency.
2. Error handling: Enhance error handling and input validation in GPIO functions.
3. System timer: For timing and delay requirements.

## Examples
1. ledtoggle.c: Example demonstrating LED toggling using GPIO output.
2. led_button.c: Example showing LED toggling based on a button press using GPIO input.
3. button_interrupt.c: Example illustrating interrupt-driven LED toggling on button press using GPIO.

## Credits

This repository draws inspiration from and acknowledges the learning gained from the Udemy course ["Learn bare metal driver development using Embedded C" by FastBit Embedded Brain Academy,Kiran Nayak](https://www.udemy.com/share/1013Ng3@VABkCc6GwTbk57b8SRY2AJWhxvYKb2aLY6O6tfoBFEM3sCUwh_wwuI8bjuvegg59WA==/), which provided valuable insights into STM32 microcontroller programming and driver development.

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for more details.
