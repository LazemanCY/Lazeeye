# Lazeeye

Lazeman design this tiny GPS record module for RC plane model.

It's a arduino project runs on ATmega328p. GPS related code is from Multiwii and "Adafruit_SSD1306" lib is used to provide OLED displaying. OLED is the common 128*64 type.

When power-up, best record will be shown, 3 seconds later, it will start to listen to GPS.
When GPS fixed, real-time data is shown in the left while best record in the right.
Record data is stored in EEPROM each 10s in case of power-off.
Long press button will reset the record.

OLED could be removed to reduce weight when in the air. 

If OLED is plugged again, short press button can wake-up the OLED.   

![](https://github.com/LazemanCY/Lazeeye/blob/master/image/module.jpg)
![](https://github.com/LazemanCY/Lazeeye/blob/master/image/test%20fly.jpg)
