# attiny85_mlx90614_temperature_alarm


Introduction
------------
Temperature monitor and alarm using Attiny85, MLX90614, and the SSD1306 OLED module.
Requires usitwix library and ssd1306xled

Hardware
--------
Attiny85, buzzer, MLX90614 infrared sensor, SSD1306 I2C 128x64 OLED module.
Programmer : USBtinyISP

Installation
------------
First set the fuse for ATtiny85 to internal 8MHz, I'm not sure if it's required.

Wiring
------
Attiny85 pin 7, 5 for MLX90614's SCL and SDA.
**Important : Don't forget to add 4.7k pull-up resistor for MLX90614**

Attiny85 pin 2, 3, for SSD1306's SCL and SDA.

t85's pin 6 for buzzer.

Pictures
--------
![My image](seilgu.github.com/attiny85_mlx90614_temperature_alarm/img/front.jpg)
![My image](seilgu.github.com/attiny85_mlx90614_temperature_alarm/img/back.jpg)
