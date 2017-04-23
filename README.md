Two I2C devices works together demo 
=======
*A Demo shown how to make 2 I2C devices work together on ESP8266.*

**Parts Required**
- NodeMCU v1.0 (base on ESP8266 ESP-12E), or any base on ESP8266 module (code *Wire.begin(SDA, SCL)* changed require for SDA/SCL pins.)
- I2C device: 
-- Monochrome 128x64 OLED graphic display : SSD1306 ( or SH1106) x1
-- Six-Axis (Gyro + Accelerometer) MotionTracking Device : MPU 6050 x1

**Hardware Connect**
- SDA of I2C devices (OLED/MPU6050) -> D2 (NodeMCU)
- SCL of I2C devices (OLED/MPU6050) -> D1 (NodeMCU)

![GitHub](https://github.com/benjenq/ESP8266_MPU6050_OLED_SSD1306/blob/master/ESP8266_MPU6050_OLED_SSD1306.JPG "icon,benjenq")

**Precautions**
- In theory this code could be built on Arduino AVR device (ATmega328P) vs Arduino IDE. But I got a **global variables > 2048 KB, out of memory** error and hard to reduce.

**VERSION**:   0.1

Depends on the following Arduino libraries.  
- [jrowberg's I2Cdev library for Arduino](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev)
- [olikraus's U8g2 library for Arduino](https://github.com/olikraus/U8g2_Arduino)

*3D Cube Code Reference*
- [MrAbalogu / Smart-Watch / MPU6050_3D_Cube](https://github.com/MrAbalogu/Smart-Watch/blob/master/MPU6050_3D_Cube/MPU6050_3D_Cube.ino)
