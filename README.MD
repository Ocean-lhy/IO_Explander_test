# 测试治具引脚映射 PY32->ESP32

- ESP32 常插引脚

| ESP32 引脚 | PY32 引脚 | 功能               |
| ---------- | --------- | ------------------ |
| IO17       | INT-PC1   | 中断引脚需外部上拉  |
| IO21       | SDA-PB4   | I2C SDA            |
| IO22       | SCL-PB3   | I2C SCL            |

- 测试引脚
| IO26       | IO1-PB5   | GPIO               |
| IO26       | IO2-PB1   | GPIO / ADC1        |
| IO26       | IO3-PA1   | GPIO               |
| IO26       | IO4-PA3   | GPIO / ADC2        |
| IO26       | IO5-PA4   | GPIO / ADC3        |
| IO26       | IO6-PA5   | GPIO               |
| IO26       | IO7-PA6   | GPIO / ADC4        |
| IO26       | IO8-PB0   | GPIO / PWM2        |
| IO26       | IO9-PA0   | GPIO / PWM1        |
| IO26       | IO10-PA7  | GPIO / PWM4        |
| IO26       | IO11-PB2  | GPIO / PWM3        |
| IO26       | IO12-PB6  | GPIO               |
| IO26       | IO13-PA2  | GPIO               |
| IO26       | IO14-PB7  | GPIO / Neopixel    |


# 备注
- ADC和PWM使能时中断关闭。
- IO5在I2C空闲休眠时中断关闭。
- IO1与IO6、IO2与IO3、IO7与IO12、IO8与IO9、IO10与IO14、IO11与IO13中断互斥。