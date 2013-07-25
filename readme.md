#7-segment LED display with I2C interface

The hardware for this project can be bought from http://item.taobao.com/item.htm?id=12715421542 (I'm not associated with the seller whatsoever)

The original hardware was designed as a 4-digit voltmeter using STM8S003F3P6 MCU. But the accuracy and resolution cannot be trusted.

I modify the firmware to turn the voltmeter into an I2C display device. The majority of the source code goes into bit banging the I2C slave protocol, as the original I2C pins are occupied by the LED. With full complier optimization I2C clock up to 50kHz is supported.

A emulation layer for the "Adafruit 4-Digit 7-Segment Display Backpack" http://www.adafruit.com/products/878 is also implemented. Original Adafruit Arduino demo codes are used for testing (with reduced I2C clock).

The free KickStart edition of "IAR Embedded Workbench for STMicroelectronics STM8" is used for development. STM8S/A Standard Peripherals Library (STM8S_StdPeriph_Driver V2.1.0) is also needed.

Descriptions for this project can be found at http://www.ba0sh1.com/hacking-the-cheap-led-voltmeter/
 
