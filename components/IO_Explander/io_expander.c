/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#include "io_expander.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IO_EXPANDER";

// ====================================================================================
// 内部函数声明 (Private Function Declarations)
// ====================================================================================
static esp_err_t io_expander_write_reg(io_expander_handle_t *handle, uint8_t reg_addr, uint8_t data);
static esp_err_t io_expander_read_reg(io_expander_handle_t *handle, uint8_t reg_addr, uint8_t *data);
static esp_err_t io_expander_write_16bit_reg(io_expander_handle_t *handle, uint8_t reg_addr_l, uint16_t data);
static esp_err_t io_expander_read_16bit_reg(io_expander_handle_t *handle, uint8_t reg_addr_l, uint16_t *data);
static esp_err_t io_expander_write_multi_reg(io_expander_handle_t *handle, uint8_t reg_addr, const uint8_t *data,
                                             uint8_t length);
static esp_err_t io_expander_read_multi_reg(io_expander_handle_t *handle, uint8_t reg_addr, uint8_t *data,
                                            uint8_t length);
static bool gpio_pins_conflict(uint8_t pin1, uint8_t pin2);

// ====================================================================================
// 静态数据定义 (Static Data Definitions)
// ====================================================================================

// 中断互斥关系映射表 (GPIO Interrupt Conflict Map)
static const uint8_t gpio_conflict_map[][2] = {
    {IO_EXP_GPIO_PIN_1, IO_EXP_GPIO_PIN_6},    // 1-6
    {IO_EXP_GPIO_PIN_2, IO_EXP_GPIO_PIN_3},    // 2-3
    {IO_EXP_GPIO_PIN_7, IO_EXP_GPIO_PIN_12},   // 7-12
    {IO_EXP_GPIO_PIN_8, IO_EXP_GPIO_PIN_9},    // 8-9
    {IO_EXP_GPIO_PIN_10, IO_EXP_GPIO_PIN_14},  // 10-14
    {IO_EXP_GPIO_PIN_11, IO_EXP_GPIO_PIN_13},  // 11-13
};

// ====================================================================================
// 私有函数实现 (Private Function Implementations)
// ====================================================================================

// I2C通信函数 (I2C Communication Functions)

/**
 * @brief 写入单个寄存器（带校验和重试）
 */
static esp_err_t io_expander_write_reg(io_expander_handle_t *handle, uint8_t reg_addr, uint8_t data)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t verify_data;
    int retry_count       = 0;
    const int max_retries = 3;

    do {
        // 执行写入
        ret = i2c_bus_write_byte(handle->i2c_device, reg_addr, data);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "写入寄存器0x%02x失败, 重试 %d/%d", reg_addr, retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));  // 延时10ms再重试
            continue;
        }

        // 延时确保写入完成
        vTaskDelay(pdMS_TO_TICKS(5));

        // 校验写入结果（特殊寄存器跳过或特殊处理）
        if (reg_addr == REG_UID_L || reg_addr == REG_UID_H || reg_addr == REG_VERSION || reg_addr == REG_GPIO_I_L ||
            reg_addr == REG_GPIO_I_H || reg_addr == REG_GPIO_IS_L || reg_addr == REG_GPIO_IS_H ||
            reg_addr == REG_TEMP_D_L || reg_addr == REG_TEMP_D_H || reg_addr == REG_ADC_D_L ||
            reg_addr == REG_ADC_D_H || reg_addr == REG_TEMP_CTRL || reg_addr == REG_FACTORY_RESET) {
            // 只读寄存器、温度控制寄存器、重置寄存器，跳过校验
            if (reg_addr == REG_TEMP_CTRL) {
                ESP_LOGI(TAG, "温度控制寄存器: 写入成功，跳过校验(START位自动清零，BUSY位动态变化)");
            } else if (reg_addr == REG_FACTORY_RESET) {
                ESP_LOGI(TAG, "恢复出厂设置寄存器: 写入成功，跳过校验(设置即生效)");
            }
            return ESP_OK;
        }

        ret = i2c_bus_read_byte(handle->i2c_device, reg_addr, &verify_data);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "读取寄存器0x%02x校验失败, 重试 %d/%d", reg_addr, retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 自动清零寄存器特殊处理
        bool verify_ok = false;
        if (reg_addr == REG_LED_CFG) {
            // LED配置：LED_CFG_REFRESH位(bit6)会自动清零，只校验其他位
            uint8_t mask = ~LED_CFG_REFRESH;  // 排除bit6
            verify_ok    = ((verify_data & mask) == (data & mask));
            ESP_LOGI(TAG, "LED配置寄存器校验: 写入0x%02x, 读取0x%02x, 掩码0x%02x, 结果%s", data, verify_data, mask,
                     verify_ok ? "通过" : "失败");
        } else if (reg_addr == REG_ADC_CTRL) {
            // ADC控制：ADC_CTRL_START位(bit6)会自动清零，只校验其他位
            uint8_t mask = ~ADC_CTRL_START;  // 排除bit6
            verify_ok    = ((verify_data & mask) == (data & mask));
            ESP_LOGI(TAG, "ADC控制寄存器校验: 写入0x%02x, 读取0x%02x, 掩码0x%02x, 结果%s", data, verify_data, mask,
                     verify_ok ? "通过" : "失败");
        } else {
            // 普通寄存器：完全匹配校验
            verify_ok = (verify_data == data);
        }

        if (verify_ok) {
            // 校验成功
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "寄存器0x%02x校验失败: 期望0x%02x, 实际0x%02x, 重试 %d/%d", reg_addr, data, verify_data,
                     retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

    } while (retry_count < max_retries);

    ESP_LOGE(TAG, "寄存器0x%02x写入失败，已重试%d次", reg_addr, max_retries);
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief 读取单个寄存器
 */
static esp_err_t io_expander_read_reg(io_expander_handle_t *handle, uint8_t reg_addr, uint8_t *data)
{
    if (handle == NULL || !handle->initialized || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_bus_read_byte(handle->i2c_device, reg_addr, data);
}

/**
 * @brief 写入16位寄存器（小端序，带校验和重试）
 */
static esp_err_t io_expander_write_16bit_reg(io_expander_handle_t *handle, uint8_t reg_addr_l, uint16_t data)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint16_t verify_data;
    int retry_count       = 0;
    const int max_retries = 3;

    uint8_t data_bytes[2] = {data & 0xFF, (data >> 8) & 0xFF};

    do {
        // 执行写入
        ret = io_expander_write_multi_reg(handle, reg_addr_l, data_bytes, 2);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "写入16位寄存器0x%02x失败, 重试 %d/%d", reg_addr_l, retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));  // 延时10ms再重试
            continue;
        }

        // 延时确保写入完成
        vTaskDelay(pdMS_TO_TICKS(5));

        // 校验写入结果（特殊寄存器跳过或特殊处理）
        if (reg_addr_l == REG_GPIO_I_L || reg_addr_l == REG_GPIO_IS_L || reg_addr_l == REG_ADC_D_L ||
            reg_addr_l == REG_TEMP_D_L) {
            // 只读寄存器，跳过校验
            return ESP_OK;
        }

        ret = io_expander_read_16bit_reg(handle, reg_addr_l, &verify_data);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "读取16位寄存器0x%02x校验失败, 重试 %d/%d", reg_addr_l, retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 16位寄存器通常不包含自动清零位，直接进行完全匹配校验
        if (verify_data == data) {
            // 校验成功
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "16位寄存器0x%02x校验失败: 期望0x%04x, 实际0x%04x, 重试 %d/%d", reg_addr_l, data, verify_data,
                     retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

    } while (retry_count < max_retries);

    ESP_LOGE(TAG, "16位寄存器0x%02x写入失败，已重试%d次", reg_addr_l, max_retries);
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief 读取16位寄存器（小端序）
 */
static esp_err_t io_expander_read_16bit_reg(io_expander_handle_t *handle, uint8_t reg_addr_l, uint16_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t data_bytes[2];
    esp_err_t ret = io_expander_read_multi_reg(handle, reg_addr_l, data_bytes, 2);
    if (ret == ESP_OK) {
        *data = (data_bytes[1] << 8) | data_bytes[0];
    }
    return ret;
}

/**
 * @brief 写入多个寄存器
 */
static esp_err_t io_expander_write_multi_reg(io_expander_handle_t *handle, uint8_t reg_addr, const uint8_t *data,
                                             uint8_t length)
{
    if (handle == NULL || !handle->initialized || data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_bus_write_bytes(handle->i2c_device, reg_addr, length, data);
}

/**
 * @brief 读取多个寄存器
 */
static esp_err_t io_expander_read_multi_reg(io_expander_handle_t *handle, uint8_t reg_addr, uint8_t *data,
                                            uint8_t length)
{
    if (handle == NULL || !handle->initialized || data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_bus_read_bytes(handle->i2c_device, reg_addr, length, data);
}

/**
 * @brief 检查GPIO引脚冲突
 */
static bool gpio_pins_conflict(uint8_t pin1, uint8_t pin2)
{
    for (int i = 0; i < sizeof(gpio_conflict_map) / sizeof(gpio_conflict_map[0]); i++) {
        if ((gpio_conflict_map[i][0] == pin1 && gpio_conflict_map[i][1] == pin2) ||
            (gpio_conflict_map[i][0] == pin2 && gpio_conflict_map[i][1] == pin1)) {
            return true;
        }
    }
    return false;
}

// ====================================================================================
// 公共函数实现 (Public Function Implementations)
// ====================================================================================

// 系统初始化和管理函数 (System Initialization and Management Functions)

/**
 * @brief 初始化I2C扩展器
 */
esp_err_t io_expander_init(const io_expander_config_t *config, io_expander_handle_t **handle)
{
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 分配内存
    io_expander_handle_t *h = malloc(sizeof(io_expander_handle_t));
    if (h == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // 复制配置
    memcpy(&h->config, config, sizeof(io_expander_config_t));

    // 创建I2C设备句柄
    h->i2c_device = i2c_bus_device_create(config->i2c_bus, config->device_address, 0);
    if (h->i2c_device == NULL) {
        ESP_LOGE(TAG, "创建I2C设备句柄失败");
        free(h);
        return ESP_FAIL;
    }

    h->initialized = true;

    // 测试通信
    uint16_t uid;
    esp_err_t ret = io_expander_read_uid(h, &uid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法与I2C扩展器通信: %s", esp_err_to_name(ret));
        i2c_bus_device_delete(&h->i2c_device);
        free(h);
        return ret;
    }

    ESP_LOGI(TAG, "I2C扩展器初始化成功，UID: 0x%04X", uid);
    *handle = h;
    return ESP_OK;
}

/**
 * @brief 反初始化I2C扩展器
 */
esp_err_t io_expander_deinit(io_expander_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->initialized = false;

    // 删除I2C设备句柄
    if (handle->i2c_device != NULL) {
        i2c_bus_device_delete(&handle->i2c_device);
    }

    free(handle);
    return ESP_OK;
}

esp_err_t io_expander_bootloader_enter(io_expander_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_write_reg(handle, 0xF0, 0x01);
}

/**
 * @brief 恢复出厂设置
 */
esp_err_t io_expander_factory_reset(io_expander_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "正在执行恢复出厂设置...");

    esp_err_t ret = io_expander_write_reg(handle, REG_FACTORY_RESET, FACTORY_RESET_TRIGGER);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "恢复出厂设置命令已发送");
        // 给设备一些时间来处理恢复操作
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        ESP_LOGE(TAG, "恢复出厂设置失败: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 读取设备UID
 */
esp_err_t io_expander_read_uid(io_expander_handle_t *handle, uint16_t *uid)
{
    if (uid == NULL) return ESP_ERR_INVALID_ARG;

    return io_expander_read_16bit_reg(handle, REG_UID_L, uid);
}

/**
 * @brief 读取版本信息
 */
esp_err_t io_expander_read_version(io_expander_handle_t *handle, uint8_t *hw_version, uint8_t *fw_version)
{
    if (hw_version == NULL || fw_version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t version_reg;
    esp_err_t ret = io_expander_read_reg(handle, REG_VERSION, &version_reg);
    if (ret == ESP_OK) {
        *hw_version = (version_reg >> 4) & 0x0F;
        *fw_version = version_reg & 0x0F;
    }
    return ret;
}

// GPIO功能函数 (GPIO Functions)

/**
 * @brief 设置GPIO模式
 */
esp_err_t io_expander_gpio_set_mode(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_mode_t mode)
{
    if (pin > IO_EXP_GPIO_PIN_14) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t gpio_mode;
    esp_err_t ret = io_expander_read_16bit_reg(handle, REG_GPIO_M_L, &gpio_mode);
    if (ret != ESP_OK) return ret;

    if (mode == IO_EXP_GPIO_MODE_OUTPUT) {
        gpio_mode |= (1 << pin);
    } else {
        gpio_mode &= ~(1 << pin);
    }

    return io_expander_write_16bit_reg(handle, REG_GPIO_M_L, gpio_mode);
}

/**
 * @brief 设置GPIO输出电平
 */
esp_err_t io_expander_gpio_set_level(io_expander_handle_t *handle, uint8_t pin, uint8_t level)
{
    if (pin > IO_EXP_GPIO_PIN_14) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t gpio_output;
    esp_err_t ret = io_expander_read_16bit_reg(handle, REG_GPIO_O_L, &gpio_output);
    if (ret != ESP_OK) return ret;

    if (level) {
        gpio_output |= (1 << pin);
    } else {
        gpio_output &= ~(1 << pin);
    }

    return io_expander_write_16bit_reg(handle, REG_GPIO_O_L, gpio_output);
}

/**
 * @brief 读取GPIO输入电平
 */
esp_err_t io_expander_gpio_get_level(io_expander_handle_t *handle, uint8_t pin, uint8_t *level)
{
    if (pin > IO_EXP_GPIO_PIN_14 || level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t gpio_input;
    esp_err_t ret = io_expander_read_16bit_reg(handle, REG_GPIO_I_L, &gpio_input);
    if (ret == ESP_OK) {
        *level = (gpio_input & (1 << pin)) ? 1 : 0;
    }
    return ret;
}

/**
 * @brief 设置GPIO上下拉
 */
esp_err_t io_expander_gpio_set_pull(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_pull_t pull)
{
    if (pin > IO_EXP_GPIO_PIN_14) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint16_t gpio_pu, gpio_pd;

    ret = io_expander_read_16bit_reg(handle, REG_GPIO_PU_L, &gpio_pu);
    if (ret != ESP_OK) return ret;

    ret = io_expander_read_16bit_reg(handle, REG_GPIO_PD_L, &gpio_pd);
    if (ret != ESP_OK) return ret;

    // 清除当前设置
    gpio_pu &= ~(1 << pin);
    gpio_pd &= ~(1 << pin);

    switch (pull) {
        case IO_EXP_GPIO_PULL_UP:
            gpio_pu |= (1 << pin);
            break;
        case IO_EXP_GPIO_PULL_DOWN:
            gpio_pd |= (1 << pin);
            break;
        case IO_EXP_GPIO_PULL_NONE:
        default:
            // 已经清除了，不需要额外操作
            break;
    }

    ret = io_expander_write_16bit_reg(handle, REG_GPIO_PU_L, gpio_pu);
    if (ret != ESP_OK) return ret;

    return io_expander_write_16bit_reg(handle, REG_GPIO_PD_L, gpio_pd);
}

/**
 * @brief 设置GPIO驱动模式
 */
esp_err_t io_expander_gpio_set_drive(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_drive_t drive)
{
    if (pin > IO_EXP_GPIO_PIN_14) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t gpio_drive;
    esp_err_t ret = io_expander_read_16bit_reg(handle, REG_GPIO_DRV_L, &gpio_drive);
    if (ret != ESP_OK) return ret;

    if (drive == IO_EXP_GPIO_DRIVE_OPEN_DRAIN) {
        gpio_drive |= (1 << pin);
    } else {
        gpio_drive &= ~(1 << pin);
    }

    return io_expander_write_16bit_reg(handle, REG_GPIO_DRV_L, gpio_drive);
}

/**
 * @brief 设置GPIO中断
 */
esp_err_t io_expander_gpio_set_interrupt(io_expander_handle_t *handle, uint8_t pin, io_exp_gpio_intr_t intr_type)
{
    if (pin > IO_EXP_GPIO_PIN_14) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint16_t gpio_ie, gpio_it;

    ret = io_expander_read_16bit_reg(handle, REG_GPIO_IE_L, &gpio_ie);
    if (ret != ESP_OK) return ret;

    ret = io_expander_read_16bit_reg(handle, REG_GPIO_IT_L, &gpio_it);
    if (ret != ESP_OK) return ret;

    if (intr_type == IO_EXP_GPIO_INTR_DISABLE) {
        gpio_ie &= ~(1 << pin);
    } else {
        // 检查中断冲突
        for (int i = 0; i < 14; i++) {
            if (i != pin && (gpio_ie & (1 << i)) && gpio_pins_conflict(pin, i)) {
                ESP_LOGW(TAG, "GPIO引脚 %d 和 %d 中断功能冲突", pin, i);
                return ESP_ERR_INVALID_STATE;
            }
        }

        gpio_ie |= (1 << pin);

        if (intr_type == IO_EXP_GPIO_INTR_RISING_EDGE) {
            gpio_it |= (1 << pin);
        } else {
            gpio_it &= ~(1 << pin);
        }
    }

    ret = io_expander_write_16bit_reg(handle, REG_GPIO_IE_L, gpio_ie);
    if (ret != ESP_OK) return ret;

    return io_expander_write_16bit_reg(handle, REG_GPIO_IT_L, gpio_it);
}

/**
 * @brief 读取GPIO中断状态
 */
esp_err_t io_expander_gpio_get_interrupt_status(io_expander_handle_t *handle, uint16_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return io_expander_read_16bit_reg(handle, REG_GPIO_IS_L, status);
}

/**
 * @brief 清除GPIO中断状态
 */
esp_err_t io_expander_gpio_clear_interrupt(io_expander_handle_t *handle, uint8_t pin)
{
    if (pin > IO_EXP_GPIO_PIN_14) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t gpio_is;
    esp_err_t ret = io_expander_read_16bit_reg(handle, REG_GPIO_IS_L, &gpio_is);
    if (ret != ESP_OK) return ret;

    gpio_is &= ~(1 << pin);  // 写0清除
    return io_expander_write_16bit_reg(handle, REG_GPIO_IS_L, gpio_is);
}

// ADC功能函数 (ADC Functions)

/**
 * @brief 启动ADC转换
 */
esp_err_t io_expander_adc_read(io_expander_handle_t *handle, uint8_t channel, uint16_t *result)
{
    if (channel < 1 || channel > 4 || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 设置通道并启动转换
    uint8_t adc_ctrl = (channel & ADC_CTRL_CH_MASK) | ADC_CTRL_START;
    ret              = io_expander_write_reg(handle, REG_ADC_CTRL, adc_ctrl);
    if (ret != ESP_OK) return ret;

    // 等待转换完成
    uint8_t busy_count = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        ret = io_expander_read_reg(handle, REG_ADC_CTRL, &adc_ctrl);
        if (ret != ESP_OK) return ret;

        if (++busy_count > 100) {  // 超时保护
            ESP_LOGW(TAG, "ADC转换超时");
            return ESP_ERR_TIMEOUT;
        }
    } while (adc_ctrl & ADC_CTRL_BUSY);

    // 读取转换结果
    return io_expander_read_16bit_reg(handle, REG_ADC_D_L, result);
}

/**
 * @brief 检查ADC转换状态
 */
esp_err_t io_expander_adc_is_busy(io_expander_handle_t *handle, bool *busy)
{
    if (busy == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t adc_ctrl;
    esp_err_t ret = io_expander_read_reg(handle, REG_ADC_CTRL, &adc_ctrl);
    if (ret == ESP_OK) {
        *busy = (adc_ctrl & ADC_CTRL_BUSY) != 0;
    }
    return ret;
}

/**
 * @brief 禁用ADC功能
 */
esp_err_t io_expander_adc_disable(io_expander_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // 设置ADC控制寄存器为禁用状态（通道0，无启动位）
    ESP_LOGI(TAG, "禁用ADC功能");
    return io_expander_write_reg(handle, REG_ADC_CTRL, ADC_CHANNEL_DISABLE);
}

// 温度传感器功能函数 (Temperature Sensor Functions)

/**
 * @brief 读取温度
 */
esp_err_t io_expander_temp_read(io_expander_handle_t *handle, uint16_t *temperature)
{
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 启动温度转换
    ret = io_expander_write_reg(handle, REG_TEMP_CTRL, TEMP_CTRL_START);
    if (ret != ESP_OK) return ret;

    // 等待转换完成
    uint8_t temp_ctrl;
    uint8_t busy_count = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        ret = io_expander_read_reg(handle, REG_TEMP_CTRL, &temp_ctrl);
        if (ret != ESP_OK) return ret;

        if (++busy_count > 100) {  // 超时保护
            ESP_LOGW(TAG, "温度转换超时");
            return ESP_ERR_TIMEOUT;
        }
    } while (temp_ctrl & TEMP_CTRL_BUSY);

    // 读取转换结果
    uint16_t temp_d;
    ret = io_expander_read_16bit_reg(handle, REG_TEMP_D_L, &temp_d);
    if (ret != ESP_OK) return ret;
    *temperature = temp_d & 0x0FFF;
    return ESP_OK;
}

/**
 * @brief 检查温度转换状态
 */
esp_err_t io_expander_temp_is_busy(io_expander_handle_t *handle, bool *busy)
{
    if (busy == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t temp_ctrl;
    esp_err_t ret = io_expander_read_reg(handle, REG_TEMP_CTRL, &temp_ctrl);
    if (ret == ESP_OK) {
        *busy = (temp_ctrl & TEMP_CTRL_BUSY) != 0;
    }
    return ret;
}

// PWM功能函数 (PWM Functions)

/**
 * @brief 设置PWM频率
 */
esp_err_t io_expander_pwm_set_frequency(io_expander_handle_t *handle, uint16_t frequency)
{
    return io_expander_write_16bit_reg(handle, REG_PWM_FREQ_L, frequency);
}

/**
 * @brief 读取PWM频率
 */
esp_err_t io_expander_pwm_get_frequency(io_expander_handle_t *handle, uint16_t *frequency)
{
    if (frequency == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_PWM_FREQ_L, frequency);
}

/**
 * @brief 设置PWM占空比
 */
esp_err_t io_expander_pwm_set_duty(io_expander_handle_t *handle, uint8_t channel, uint8_t duty, bool polarity,
                                   bool enable)
{
    if (channel > 3 || duty > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    // 将0-100的占空比转换为8位值(0-0xFFF)
    uint16_t duty_12bit = (duty * 0x0FFF) / 100;

    // 添加控制位到高位
    if (enable) duty_12bit |= (PWM_ENABLE << 8);
    if (polarity) duty_12bit |= (PWM_POLARITY << 8);

    uint8_t reg_addr_l = REG_PWM1_DUTY_L + (channel * 2);
    ESP_LOGI(TAG, "写入: %04X", duty_12bit);
    return io_expander_write_16bit_reg(handle, reg_addr_l, duty_12bit);
}

/**
 * @brief 读取PWM占空比
 */
esp_err_t io_expander_pwm_get_duty(io_expander_handle_t *handle, uint8_t channel, uint8_t *duty, bool *polarity,
                                   bool *enable)
{
    if (channel > 3 || duty == NULL || polarity == NULL || enable == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr_l  = REG_PWM1_DUTY_L + (channel * 2);
    uint16_t duty_16bit = 0;

    esp_err_t ret = io_expander_read_16bit_reg(handle, reg_addr_l, &duty_16bit);
    ESP_LOGI(TAG, "读取: %04X", duty_16bit);
    if (ret != ESP_OK) return ret;

    // 提取12位占空比值
    uint16_t duty_12bit = duty_16bit & 0x0FFF;

    // 转换为0-100的占空比
    *duty = (duty_12bit * 100) / 0x0FFF;

    // 提取极性
    *polarity = (duty_16bit & (PWM_POLARITY << 8)) != 0;

    // 提取使能状态
    *enable = (duty_16bit & (PWM_ENABLE << 8)) != 0;

    return ESP_OK;
}

// LED控制功能函数 (LED Control Functions)

/**
 * @brief 设置LED数量
 * @note LED控制复用在IO14(PB7)引脚上，使用此功能时GPIO14的普通GPIO功能不可用
 */
esp_err_t io_expander_led_set_count(io_expander_handle_t *handle, uint8_t num_leds)
{
    if (num_leds > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    if (num_leds > 0) {
        ESP_LOGI(TAG, "启用LED控制，IO14(PB7)复用为Neopixel控制引脚");
    }

    uint8_t led_cfg = num_leds & LED_CFG_NUM_MASK;
    return io_expander_write_reg(handle, REG_LED_CFG, led_cfg);
}

/**
 * @brief 设置单个LED颜色
 */
esp_err_t io_expander_led_set_color(io_expander_handle_t *handle, uint8_t led_index, rgb_color_t color)
{
    if (led_index > 31 || color.r > 31 || color.g > 63 || color.b > 31) {
        return ESP_ERR_INVALID_ARG;
    }

    // 转换为RGB565格式
    uint16_t rgb565 = ((color.r & 0x1F) << 11) | ((color.g & 0x3F) << 5) | (color.b & 0x1F);

    // 计算寄存器地址
    uint8_t reg_addr = REG_LED_RAM_START + (led_index * 2);

    // 写入RGB565数据（高位先行）
    uint8_t data[2] = {(rgb565 >> 8) & 0xFF, rgb565 & 0xFF};
    return io_expander_write_multi_reg(handle, reg_addr, data, 2);
}

/**
 * @brief 刷新LED显示
 */
esp_err_t io_expander_led_refresh(io_expander_handle_t *handle)
{
    uint8_t led_cfg;
    esp_err_t ret = io_expander_read_reg(handle, REG_LED_CFG, &led_cfg);
    if (ret != ESP_OK) return ret;

    led_cfg |= LED_CFG_REFRESH;
    return io_expander_write_reg(handle, REG_LED_CFG, led_cfg);
}

/**
 * @brief 禁用LED功能
 */
esp_err_t io_expander_led_disable(io_expander_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // 设置LED数量为0来禁用LED功能，恢复IO14(PB7)的普通GPIO功能
    ESP_LOGI(TAG, "禁用LED功能，恢复IO14(PB7)普通GPIO功能");
    return io_expander_write_reg(handle, REG_LED_CFG, 0);
}

// RTC RAM功能函数 (RTC RAM Functions)

/**
 * @brief 写入RTC RAM
 */
esp_err_t io_expander_rtc_ram_write(io_expander_handle_t *handle, uint8_t offset, const uint8_t *data, uint8_t length)
{
    if (offset > 31 || data == NULL || length == 0 || (offset + length) > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = REG_RTC_RAM_START + offset;
    return io_expander_write_multi_reg(handle, reg_addr, data, length);
}

/**
 * @brief 读取RTC RAM
 */
esp_err_t io_expander_rtc_ram_read(io_expander_handle_t *handle, uint8_t offset, uint8_t *data, uint8_t length)
{
    if (offset > 31 || data == NULL || length == 0 || (offset + length) > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = REG_RTC_RAM_START + offset;
    return io_expander_read_multi_reg(handle, reg_addr, data, length);
}

// 系统配置和调试函数 (System Configuration and Debug Functions)

/**
 * @brief 设置I2C配置
 */
esp_err_t io_expander_set_i2c_config(io_expander_handle_t *handle, uint8_t sleep_time, bool speed_400k, bool wake_mode,
                                     bool inter_pull_off)
{
    if (sleep_time > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t i2c_cfg = sleep_time & I2C_CFG_SLEEP_MASK;
    if (speed_400k) {
        i2c_cfg |= I2C_CFG_SPEED_400K;
    }

    if (wake_mode) {
        i2c_cfg |= I2C_CFG_WAKE_RISING;
    } else {
        i2c_cfg |= I2C_CFG_WAKE_FALLING;
    }

    if (inter_pull_off) {
        i2c_cfg |= I2C_CFG_INTER_PULL_OFF;
    } else {
        i2c_cfg |= I2C_CFG_INTER_PULL_ON;
    }
    return io_expander_write_reg(handle, REG_I2C_CFG, i2c_cfg);
}

/**
 * @brief 读取参考电压
 */
esp_err_t io_expander_get_ref_voltage(io_expander_handle_t *handle, uint16_t *ref_voltage)
{
    if (ref_voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_REF_VOLTAGE_L, ref_voltage);
}

// 读取GPIO寄存器状态（用于调试）
esp_err_t io_expander_gpio_get_mode_reg(io_expander_handle_t *handle, uint16_t *mode_reg)
{
    if (mode_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_GPIO_M_L, mode_reg);
}

esp_err_t io_expander_gpio_get_output_reg(io_expander_handle_t *handle, uint16_t *output_reg)
{
    if (output_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_GPIO_O_L, output_reg);
}

esp_err_t io_expander_gpio_get_input_reg(io_expander_handle_t *handle, uint16_t *input_reg)
{
    if (input_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_GPIO_I_L, input_reg);
}

esp_err_t io_expander_gpio_get_pull_up_reg(io_expander_handle_t *handle, uint16_t *pull_up_reg)
{
    if (pull_up_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_GPIO_PU_L, pull_up_reg);
}

esp_err_t io_expander_gpio_get_pull_down_reg(io_expander_handle_t *handle, uint16_t *pull_down_reg)
{
    if (pull_down_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_GPIO_PD_L, pull_down_reg);
}

esp_err_t io_expander_gpio_get_drive_reg(io_expander_handle_t *handle, uint16_t *drive_reg)
{
    if (drive_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return io_expander_read_16bit_reg(handle, REG_GPIO_DRV_L, drive_reg);
}