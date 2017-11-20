# Lazeeye
/* 
Lazeman design this tiny GPS record module for RC plane model.

GPS related code is from Multiwii and "Adafruit_SSD1306" lib is used to provide OLED displaying.

When power-up, best record will be shown, 3 seconds later, it will start to listen to GPS.
When GPS fixed, real-time data is shown in the left while best record in the right.
Record data is stored in EEPROM each 10s in case of power-off.
Long press button will reset the record.

OLED could be removed to reduce weight when in the air. 

If OLED is plugged again, short press button can wake-up the OLED.   
*/
