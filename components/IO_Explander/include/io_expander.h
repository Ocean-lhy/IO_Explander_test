/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IO_EXPANDER_H
#define IO_EXPANDER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "i2c_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C设备地址
#define IO_EXPANDER_ADDR (0x6F)

// ====================================================================================
// 寄存器地址定义 (Register Address Definitions)
// ====================================================================================

// 系统信息寄存器 (System Information Registers)
#define REG_UID_L   (0x00)  // UID低字节（只读）
#define REG_UID_H   (0x01)  // UID高字节（只读）
#define REG_VERSION (0x02)  // 硬件版本(高4位)+固件版本(低4位)

// GPIO控制寄存器 (GPIO Control Registers)
#define REG_GPIO_M_L   (0x03)  // GPIO方向寄存器低字节 (0:输入, 1:输出)
#define REG_GPIO_M_H   (0x04)  // GPIO方向寄存器高字节
#define REG_GPIO_O_L   (0x05)  // GPIO输出寄存器低字节
#define REG_GPIO_O_H   (0x06)  // GPIO输出寄存器高字节
#define REG_GPIO_I_L   (0x07)  // GPIO输入寄存器低字节（只读）
#define REG_GPIO_I_H   (0x08)  // GPIO输入寄存器高字节（只读）
#define REG_GPIO_PU_L  (0x09)  // GPIO上拉控制低字节
#define REG_GPIO_PU_H  (0x0A)  // GPIO上拉控制高字节
#define REG_GPIO_PD_L  (0x0B)  // GPIO下拉控制低字节
#define REG_GPIO_PD_H  (0x0C)  // GPIO下拉控制高字节
#define REG_GPIO_IE_L  (0x0D)  // GPIO中断使能低字节
#define REG_GPIO_IE_H  (0x0E)  // GPIO中断使能高字节
#define REG_GPIO_IT_L  (0x0F)  // GPIO中断触发类型低字节 (0:下降沿, 1:上升沿)
#define REG_GPIO_IT_H  (0x10)  // GPIO中断触发类型高字节
#define REG_GPIO_IS_L  (0x11)  // GPIO中断状态低字节
#define REG_GPIO_IS_H  (0x12)  // GPIO中断状态高字节
#define REG_GPIO_DRV_L (0x13)  // GPIO驱动模式低字节 (0:推挽, 1:开漏)
#define REG_GPIO_DRV_H (0x14)  // GPIO驱动模式高字节

// ADC控制寄存器 (ADC Control Registers)
#define REG_ADC_CTRL (0x15)  // ADC控制寄存器
#define REG_ADC_D_L  (0x16)  // ADC数据低字节（12位有效）
#define REG_ADC_D_H  (0x17)  // ADC数据高字节

// 温度传感器寄存器 (Temperature Sensor Registers)
#define REG_TEMP_CTRL (0x18)  // 温度控制寄存器
#define REG_TEMP_D_L  (0x19)  // 温度数据低字节（12位有效）
#define REG_TEMP_D_H  (0x1A)  // 温度数据高字节

// PWM控制寄存器 (PWM Control Registers)
#define REG_PWM1_DUTY_L (0x1B)  // PWM1占空比低字节
#define REG_PWM1_DUTY_H (0x1C)  // PWM1占空比高字节 + 控制位
#define REG_PWM2_DUTY_L (0x1D)  // PWM2占空比低字节
#define REG_PWM2_DUTY_H (0x1E)  // PWM2占空比高字节 + 控制位
#define REG_PWM3_DUTY_L (0x1F)  // PWM3占空比低字节
#define REG_PWM3_DUTY_H (0x20)  // PWM3占空比高字节 + 控制位
#define REG_PWM4_DUTY_L (0x21)  // PWM4占空比低字节
#define REG_PWM4_DUTY_H (0x22)  // PWM4占空比高字节 + 控制位

// 系统配置寄存器 (System Configuration Registers)
#define REG_I2C_CFG       (0x23)  // I2C配置寄存器
#define REG_LED_CFG       (0x24)  // LED配置寄存器
#define REG_PWM_FREQ_L    (0x25)  // PWM频率低字节
#define REG_PWM_FREQ_H    (0x26)  // PWM频率高字节
#define REG_REF_VOLTAGE_L (0x27)  // 参考电压低字节
#define REG_REF_VOLTAGE_H (0x28)  // 参考电压高字节
#define REG_FACTORY_RESET (0x29)  // 恢复出厂设置寄存器

// 数据存储区域 (Data Storage Areas)
#define REG_LED_RAM_START (0x30)  // LED RGB565数据起始地址
#define REG_LED_RAM_END   (0x6F)  // LED RGB565数据结束地址 (64字节)
#define REG_RTC_RAM_START (0x70)  // RTC RAM起始地址
#define REG_RTC_RAM_END   (0x8F)  // RTC RAM结束地址 (32字节)

// ====================================================================================
// 引脚和通道定义 (Pin and Channel Definitions)
// ====================================================================================

// GPIO引脚定义 (GPIO Pin Definitions)
#define IO_EXP_GPIO_PIN_1  (0)   // IO1 - PB5
#define IO_EXP_GPIO_PIN_2  (1)   // IO2 - PB1 (支持ADC)
#define IO_EXP_GPIO_PIN_3  (2)   // IO3 - PA1
#define IO_EXP_GPIO_PIN_4  (3)   // IO4 - PA3 (支持ADC)
#define IO_EXP_GPIO_PIN_5  (4)   // IO5 - PA4 (支持ADC)
#define IO_EXP_GPIO_PIN_6  (5)   // IO6 - PA5
#define IO_EXP_GPIO_PIN_7  (6)   // IO7 - PA6 (支持ADC)
#define IO_EXP_GPIO_PIN_8  (7)   // IO8 - PB0 (支持PWM)
#define IO_EXP_GPIO_PIN_9  (8)   // IO9 - PA0 (支持PWM)
#define IO_EXP_GPIO_PIN_10 (9)   // IO10 - PA7 (支持PWM)
#define IO_EXP_GPIO_PIN_11 (10)  // IO11 - PB2 (支持PWM)
#define IO_EXP_GPIO_PIN_12 (11)  // IO12 - PB6
#define IO_EXP_GPIO_PIN_13 (12)  // IO13 - PA2
#define IO_EXP_GPIO_PIN_14 (13)  // IO14 - PB7 (支持LED控制)

// ADC通道定义 (ADC Channel Definitions)
#define ADC_CHANNEL_DISABLE  (0)  // 禁用ADC
#define IO_EXP_ADC_CHANNEL_1 (1)  // ADC通道1 - IO2(PB1)
#define IO_EXP_ADC_CHANNEL_2 (2)  // ADC通道2 - IO4(PA3)
#define IO_EXP_ADC_CHANNEL_3 (3)  // ADC通道3 - IO5(PA4)
#define IO_EXP_ADC_CHANNEL_4 (4)  // ADC通道4 - IO7(PA6)

// PWM通道定义 (PWM Channel Definitions)
#define IO_EXP_PWM_CHANNEL_1 (0)  // PWM通道1 - IO9(PA0)
#define IO_EXP_PWM_CHANNEL_2 (1)  // PWM通道2 - IO8(PB0)
#define IO_EXP_PWM_CHANNEL_3 (2)  // PWM通道3 - IO11(PB2)
#define IO_EXP_PWM_CHANNEL_4 (3)  // PWM通道4 - IO10(PA7)

// LED控制通道定义 (LED Control Channel Definitions)
#define IO_EXP_LED_CONTROL_CHANNEL (0)  // LED控制通道 - IO14(PB7) Neopixel控制

// ====================================================================================
// 寄存器位定义 (Register Bit Definitions)
// ====================================================================================

// ADC_CTRL寄存器位定义 (ADC Control Register Bits)
#define ADC_CTRL_CH_MASK (0x07)    // 通道选择掩码 (Bit 0-2)
#define ADC_CTRL_START   (1 << 6)  // 启动转换 (Bit 6)
#define ADC_CTRL_BUSY    (1 << 7)  // 转换忙标志 (Bit 7, 只读)

// TEMP_CTRL寄存器位定义 (Temperature Control Register Bits)
#define TEMP_CTRL_START (1 << 6)  // 启动温度转换 (Bit 6)
#define TEMP_CTRL_BUSY  (1 << 7)  // 温度转换忙标志 (Bit 7, 只读)

// PWM_DUTY_H寄存器位定义 (PWM Duty High Register Bits)
#define PWM_POLARITY (1 << 6)  // PWM极性控制 (Bit 6: 0=正极性, 1=负极性)
#define PWM_ENABLE   (1 << 7)  // PWM使能 (Bit 7: 0=禁用, 1=使能)

// I2C_CFG寄存器位定义 (I2C Configuration Register Bits)
#define I2C_CFG_SLEEP_MASK     (0x0F)    // 休眠时间掩码 (Bit 0-3)
#define I2C_CFG_SPEED_100K     (0 << 4)  // I2C速度100KHz (Bit 4)
#define I2C_CFG_SPEED_400K     (1 << 4)  // I2C速度400KHz (Bit 4)
#define I2C_CFG_WAKE_FALLING   (0 << 5)  // 唤醒下降沿 (Bit 5)
#define I2C_CFG_WAKE_RISING    (1 << 5)  // 唤醒上升沿 (Bit 5)
#define I2C_CFG_INTER_PULL_ON  (0 << 6)  // 内部上拉使能 (Bit 6)
#define I2C_CFG_INTER_PULL_OFF (1 << 6)  // 内部上拉禁止 (Bit 6)

// LED_CFG寄存器位定义 (LED Configuration Register Bits)
#define LED_CFG_NUM_MASK (0x3F)    // LED数量掩码 (Bit 0-5, 最大32个)
#define LED_CFG_REFRESH  (1 << 6)  // 刷新LED显示 (Bit 6)

// FACTORY_RESET寄存器位定义 (Factory Reset Register Bits)
#define FACTORY_RESET_TRIGGER (0x3A)  // 恢复出厂设置触发值

// ====================================================================================
// 数据结构定义 (Data Structure Definitions)
// ====================================================================================

/**
 * @brief IO扩展器配置结构体
 */
typedef struct {
    i2c_bus_handle_t i2c_bus;  ///< I2C总线句柄
    uint8_t device_address;    ///< 设备I2C地址 (默认0x6F)
    uint16_t timeout_ms;       ///< I2C通信超时时间（毫秒）
} io_expander_config_t;

/**
 * @brief IO扩展器句柄结构体
 */
typedef struct {
    io_expander_config_t config;         ///< 配置信息
    i2c_bus_device_handle_t i2c_device;  ///< I2C设备句柄
    bool initialized;                    ///< 初始化状态标志
} io_expander_handle_t;

/**
 * @brief GPIO模式枚举
 */
typedef enum {
    IO_EXP_GPIO_MODE_INPUT  = 0,  ///< 输入模式
    IO_EXP_GPIO_MODE_OUTPUT = 1   ///< 输出模式
} io_exp_gpio_mode_t;

/**
 * @brief GPIO上下拉模式枚举
 */
typedef enum {
    IO_EXP_GPIO_PULL_NONE = 0,  ///< 无上下拉（浮空）
    IO_EXP_GPIO_PULL_UP   = 1,  ///< 上拉
    IO_EXP_GPIO_PULL_DOWN = 2   ///< 下拉
} io_exp_gpio_pull_t;

/**
 * @brief GPIO驱动模式枚举
 */
typedef enum {
    IO_EXP_GPIO_DRIVE_PUSH_PULL  = 0,  ///< 推挽输出
    IO_EXP_GPIO_DRIVE_OPEN_DRAIN = 1   ///< 开漏输出
} io_exp_gpio_drive_t;

/**
 * @brief GPIO中断触发类型枚举
 */
typedef enum {
    IO_EXP_GPIO_INTR_DISABLE      = 0,  ///< 禁用中断
    IO_EXP_GPIO_INTR_FALLING_EDGE = 1,  ///< 下降沿触发
    IO_EXP_GPIO_INTR_RISING_EDGE  = 2   ///< 上升沿触发
} io_exp_gpio_intr_t;

/**
 * @brief RGB颜色结构体 (RGB565格式)
 */
typedef struct {
    uint16_t r;  ///< 红色分量 (0-31, 5位)
    uint16_t g;  ///< 绿色分量 (0-63, 6位)
    uint16_t b;  ///< 蓝色分量 (0-31, 5位)
} rgb_color_t;

// ====================================================================================
// 函数声明 (Function Declarations)
// ====================================================================================

// 系统初始化和管理函数 (System Initialization and Management Functions)

/**
 * @brief 初始化I2C扩展器
 * @param config 配置参数
 * @param handle 返回的句柄
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_init(const io_expander_config_t *config, io_expander_handle_t **handle);

/**
 * @brief 反初始化I2C扩展器
 * @param handle 句柄
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_deinit(io_expander_handle_t *handle);

/**
 * @brief 读取设备UID
 * @param handle 句柄
 * @param uid 返回的UID
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_read_uid(io_expander_handle_t *handle, uint16_t *uid);

/**
 * @brief 读取版本信息
 * @param handle 句柄
 * @param hw_version 硬件版本
 * @param fw_version 固件版本
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_read_version(io_expander_handle_t *handle, uint8_t *hw_version, uint8_t *fw_version);

// GPIO功能函数 (GPIO Functions)
/**
 * @brief 设置GPIO模式
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @param mode GPIO模式
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_set_mode(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_mode_t mode);

/**
 * @brief 设置GPIO输出电平
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @param level 输出电平(0或1)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_set_level(io_expander_handle_t *handle, uint8_t pin, uint8_t level);

/**
 * @brief 读取GPIO输入电平
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @param level 返回的电平值
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_get_level(io_expander_handle_t *handle, uint8_t pin, uint8_t *level);

/**
 * @brief 设置GPIO上下拉
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @param pull 上下拉模式
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_set_pull(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_pull_t pull);

/**
 * @brief 设置GPIO驱动模式
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @param drive 驱动模式
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_set_drive(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_drive_t drive);

/**
 * @brief 设置GPIO中断
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @param intr_type 中断类型
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_set_interrupt(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_intr_t intr_type);

/**
 * @brief 读取GPIO中断状态
 * @param handle 句柄
 * @param status 返回的中断状态(位掩码)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_get_interrupt_status(io_expander_handle_t *handle, uint16_t *status);

/**
 * @brief 清除GPIO中断状态
 * @param handle 句柄
 * @param pin GPIO引脚号(0-13)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_gpio_clear_interrupt(io_expander_handle_t *handle, uint8_t pin);

// ADC功能函数 (ADC Functions)
/**
 * @brief 启动ADC转换
 * @param handle 句柄
 * @param channel ADC通道(1-4)
 * @param result 转换结果(12位)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_adc_read(io_expander_handle_t *handle, uint8_t channel, uint16_t *result);

/**
 * @brief 检查ADC转换状态
 * @param handle 句柄
 * @param busy 返回转换状态
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_adc_is_busy(io_expander_handle_t *handle, bool *busy);

// 温度传感器功能函数 (Temperature Sensor Functions)
/**
 * @brief 读取温度
 * @param handle 句柄
 * @param temperature 返回的温度值(12位)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_temp_read(io_expander_handle_t *handle, uint16_t *temperature);

/**
 * @brief 检查温度转换状态
 * @param handle 句柄
 * @param busy 返回转换状态
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_temp_is_busy(io_expander_handle_t *handle, bool *busy);

// PWM功能函数 (PWM Functions)
/**
 * @brief 设置PWM频率
 * @param handle 句柄
 * @param frequency PWM频率(Hz)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_pwm_set_frequency(io_expander_handle_t *handle, uint16_t frequency);

/**
 * @brief 读取PWM频率
 * @param handle 句柄
 * @param frequency 返回PWM频率(Hz)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_pwm_get_frequency(io_expander_handle_t *handle, uint16_t *frequency);

/**
 * @brief 设置PWM占空比
 * @param handle 句柄
 * @param channel PWM通道(0-3)
 * @param duty 占空比(0-100)
 * @param polarity 极性(false:高电平有效, true:低电平有效)
 * @param enable 使能PWM
 * @return ESP_OK成功，其他值失败
 * @note 内部将0-100的占空比转换为12位值(0-0xFFF)
 */
esp_err_t io_expander_pwm_set_duty(io_expander_handle_t *handle, uint8_t channel, uint8_t duty, bool polarity,
                                   bool enable);

/**
 * @brief 读取PWM占空比
 * @param handle 句柄
 * @param channel PWM通道(0-3)
 * @param duty 返回占空比(0-100)
 * @param polarity 返回极性
 * @param enable 返回使能状态
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_pwm_get_duty(io_expander_handle_t *handle, uint8_t channel, uint8_t *duty, bool *polarity,
                                   bool *enable);

// LED控制功能函数 (LED Control Functions - 复用IO14-PB7引脚)
/**
 * @brief 设置LED数量
 * @param handle 句柄
 * @param num_leds LED数量(0-32)
 * @return ESP_OK成功，其他值失败
 * @note LED控制复用在IO14(PB7)引脚上，使用此功能时GPIO14的普通GPIO功能不可用
 */
esp_err_t io_expander_led_set_count(io_expander_handle_t *handle, uint8_t num_leds);

/**
 * @brief 设置单个LED颜色
 * @param handle 句柄
 * @param led_index LED索引(0-31)
 * @param color RGB颜色
 * @return ESP_OK成功，其他值失败
 * @note LED数据通过IO14(PB7)引脚输出到Neopixel LED串
 */
esp_err_t io_expander_led_set_color(io_expander_handle_t *handle, uint8_t led_index, rgb_color_t color);

/**
 * @brief 刷新LED显示
 * @param handle 句柄
 * @return ESP_OK成功，其他值失败
 * @note 触发IO14(PB7)引脚输出Neopixel时序信号
 */
esp_err_t io_expander_led_refresh(io_expander_handle_t *handle);

// RTC RAM功能函数 (RTC RAM Functions)
/**
 * @brief 写入RTC RAM
 * @param handle 句柄
 * @param offset 偏移地址(0-31)
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_rtc_ram_write(io_expander_handle_t *handle, uint8_t offset, const uint8_t *data, uint8_t length);

/**
 * @brief 读取RTC RAM
 * @param handle 句柄
 * @param offset 偏移地址(0-31)
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_rtc_ram_read(io_expander_handle_t *handle, uint8_t offset, uint8_t *data, uint8_t length);

// 系统配置和调试函数 (System Configuration and Debug Functions)
/**
 * @brief 设置I2C配置
 * @param handle 句柄
 * @param sleep_time 休眠时间(0-15, 0为不休眠)
 * @param speed_400k 是否使用400KHz速度
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_set_i2c_config(io_expander_handle_t *handle, uint8_t sleep_time, bool speed_400k, bool wake_mode,
                                     bool inter_pull_off);

/**
 * @brief 读取参考电压
 * @param handle 句柄
 * @param ref_voltage 参考电压值(16位)
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_get_ref_voltage(io_expander_handle_t *handle, uint16_t *ref_voltage);

// 读取GPIO寄存器状态（用于调试）
esp_err_t io_expander_gpio_get_mode_reg(io_expander_handle_t *handle, uint16_t *mode_reg);
esp_err_t io_expander_gpio_get_output_reg(io_expander_handle_t *handle, uint16_t *output_reg);
esp_err_t io_expander_gpio_get_input_reg(io_expander_handle_t *handle, uint16_t *input_reg);
esp_err_t io_expander_gpio_get_pull_up_reg(io_expander_handle_t *handle, uint16_t *pull_up_reg);
esp_err_t io_expander_gpio_get_pull_down_reg(io_expander_handle_t *handle, uint16_t *pull_down_reg);
esp_err_t io_expander_gpio_get_drive_reg(io_expander_handle_t *handle, uint16_t *drive_reg);

/**
 * @brief 进入Bootloader模式
 * @param handle 句柄
 * @return ESP_OK成功，其他值失败
 */
esp_err_t io_expander_bootloader_enter(io_expander_handle_t *handle);

/**
 * @brief 恢复出厂设置
 * @param handle 句柄
 * @return ESP_OK成功，其他值失败
 * @note 此操作会重置所有配置参数到默认值，包括GPIO模式、PWM设置等
 */
esp_err_t io_expander_factory_reset(io_expander_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif  // io_expander_H