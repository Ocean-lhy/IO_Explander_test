extern "C" {
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "io_expander.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
}
#include <M5GFX.h>

#define TAG "IO_EXP_TEST"

// 按键定义
#define BUTTON_A_PIN (gpio_num_t)39
#define BUTTON_B_PIN (gpio_num_t)38
#define BUTTON_C_PIN (gpio_num_t)37

// 测试引脚定义 (ESP32的GPIO26用于检测IO_Explander输出)
#define TEST_GPIO_PIN (gpio_num_t)26

// 全局变量
i2c_bus_handle_t i2c_bus          = NULL;
io_expander_handle_t* io_expander = NULL;
M5GFX display;

// 设备信息
typedef struct {
    uint16_t uid;
    uint8_t hw_version;
    uint8_t fw_version;
    uint16_t temperature;
    uint16_t ref_voltage;
    bool info_valid;
} device_info_t;

static device_info_t device_info = {0};

// 测试状态
typedef enum {
    TEST_STATE_MENU,             // 菜单状态
    TEST_STATE_SELECT_PIN,       // 选择引脚状态
    TEST_STATE_RUNNING,          // 测试运行状态
    TEST_STATE_WAIT_NEXT,        // 等待下一个测试状态
    TEST_STATE_FUNCTION_TEST     // 功能寄存器测试状态
} test_state_t;

static test_state_t current_state = TEST_STATE_MENU;
static int current_pin            = 0;  // 当前测试的引脚 (0-13)
static int current_test           = 0;  // 当前测试项目

// 引脚功能映射
typedef struct {
    int pin;
    const char* name;
    bool has_adc;
    int adc_channel;
    bool has_pwm;
    int pwm_channel;
    bool has_led;
} pin_info_t;

static const pin_info_t pin_map[14] = {
    {0, "IO1-PB5", false, 0, false, 0, false},    // IO1
    {1, "IO2-PB1", true, 1, false, 0, false},     // IO2 - ADC1
    {2, "IO3-PA1", false, 0, false, 0, false},    // IO3
    {3, "IO4-PA3", true, 2, false, 0, false},     // IO4 - ADC2
    {4, "IO5-PA4", true, 3, false, 0, false},     // IO5 - ADC3
    {5, "IO6-PA5", false, 0, false, 0, false},    // IO6
    {6, "IO7-PA6", true, 4, false, 0, false},     // IO7 - ADC4
    {7, "IO8-PB0", false, 0, true, 1, false},     // IO8 - PWM2
    {8, "IO9-PA0", false, 0, true, 0, false},     // IO9 - PWM1
    {9, "IO10-PA7", false, 0, true, 3, false},    // IO10 - PWM4
    {10, "IO11-PB2", false, 0, true, 2, false},   // IO11 - PWM3
    {11, "IO12-PB6", false, 0, false, 0, false},  // IO12
    {12, "IO13-PA2", false, 0, false, 0, false},  // IO13
    {13, "IO14-PB7", false, 0, false, 0, true}    // IO14 - LED
};

// 按键状态检测
bool button_pressed(gpio_num_t pin)
{
    static uint32_t last_press_time[3] = {0};
    static bool last_state[3]          = {true, true, true};  // 按键默认高电平

    int index          = (pin == BUTTON_A_PIN) ? 0 : (pin == BUTTON_B_PIN) ? 1 : 2;
    bool current_state = gpio_get_level(pin);
    uint32_t now       = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 检测下降沿（按下）并防抖动
    if (last_state[index] && !current_state && (now - last_press_time[index] > 50)) {
        last_press_time[index] = now;
        last_state[index]      = current_state;
        return true;
    }

    last_state[index] = current_state;
    return false;
}

// 初始化按键
void init_buttons()
{
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << BUTTON_A_PIN) | (1ULL << BUTTON_B_PIN) | (1ULL << BUTTON_C_PIN),
                             .mode         = GPIO_MODE_INPUT,
                             .pull_up_en   = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type    = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}

// 读取设备信息
void read_device_info() {
    device_info.info_valid = false;
    
    // 读取UID
    if (io_expander_read_uid(io_expander, &device_info.uid) != ESP_OK) {
        ESP_LOGW(TAG, "读取UID失败");
        return;
    }
    
    // 读取版本信息
    if (io_expander_read_version(io_expander, &device_info.hw_version, &device_info.fw_version) != ESP_OK) {
        ESP_LOGW(TAG, "读取版本信息失败");
        return;
    }
    
    // 读取温度（启动温度采样并等待完成）
    esp_err_t temp_ret = io_expander_temp_read(io_expander, &device_info.temperature);
    if (temp_ret != ESP_OK) {
        ESP_LOGW(TAG, "读取温度失败: %d", temp_ret);
        device_info.temperature = 0;
    } else {
        ESP_LOGI(TAG, "温度采样成功: %d", device_info.temperature);
    }
    
    // 读取参考电压
    if (io_expander_get_ref_voltage(io_expander, &device_info.ref_voltage) != ESP_OK) {
        ESP_LOGW(TAG, "读取参考电压失败");
        device_info.ref_voltage = 0;
    }
    
    device_info.info_valid = true;
    ESP_LOGI(TAG, "设备信息读取成功 - UID:0x%04x, HW:%d, FW:%d, 温度:%d, 电压:%d", 
             device_info.uid, device_info.hw_version, device_info.fw_version, 
             device_info.temperature, device_info.ref_voltage);
}

// 初始化测试GPIO
void init_test_gpio()
{
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << TEST_GPIO_PIN),
                             .mode         = GPIO_MODE_INPUT,
                             .pull_up_en   = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type    = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}

// 复位引脚配置的复用功能
void reset_pin_config(int pin)
{
    ESP_LOGI(TAG, "复位引脚 %d (%s) 的配置", pin, pin_map[pin].name);
    
    // 1. 禁用中断并清除中断状态
    io_expander_gpio_set_interrupt(io_expander, pin, IO_EXP_GPIO_INTR_DISABLE);
    io_expander_gpio_clear_interrupt(io_expander, pin);
    
    // 2. 设置为GPIO输入模式（默认状态）
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);
    
    // 3. 关闭上下拉（浮空状态）
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_NONE);
    
    // 4. 如果是PWM引脚，禁用PWM输出
    if (pin_map[pin].has_pwm) {
        io_expander_pwm_set_duty(io_expander, pin_map[pin].pwm_channel, 0, false, false);
    }

    // 5. 如果是ADC引脚，禁用ADC采样
    if (pin_map[pin].has_adc) {
        io_expander_adc_disable(io_expander);
    }

    // 6. 如果是LED引脚，关闭LED
    if (pin_map[pin].has_led) {
        io_expander_led_disable(io_expander);
    }
    
    // 给一点时间让配置生效
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "引脚 %d 配置复位完成", pin);
}

// 显示功能
void display_clear()
{
    display.fillScreen(WHITE);
    display.setTextColor(BLACK);
    display.setTextSize(2);
    display.setCursor(10, 10);
}

void display_menu()
{
    display_clear();
    display.println("IO Explander 测试器");
    
    // 显示设备信息
    if (device_info.info_valid) {
        display.printf("序号: 0x%04X", device_info.uid);
        display.println("");
        display.printf("版本: HW-%d FW-%d", device_info.hw_version, device_info.fw_version);
        display.println("");
        
        display.printf("温度: %d°C", device_info.temperature);
        display.println("");
        
        // 参考电压转换
        float ref_voltage = device_info.ref_voltage / 1000.0; // 假设单位是mV
        display.printf("电压: %.2fV", ref_voltage);
        display.println("");
    } else {
        display.println("设备信息读取失败");
        display.println("");
    }
    
    display.println("A: 选择引脚");
    display.println("B: 开始测试");
    display.println("C: 功能测试");
    display.printf("当前引脚: %s", pin_map[current_pin].name);
}

void display_pin_select()
{
    display_clear();
    display.println("选择测试引脚:");
    display.println("");
    display.printf("-> %s", pin_map[current_pin].name);
    display.println("");
    display.printf("功能: GPIO");
    if (pin_map[current_pin].has_adc) display.printf(" ADC");
    if (pin_map[current_pin].has_pwm) display.printf(" PWM");
    if (pin_map[current_pin].has_led) display.printf(" LED");
    display.println("");
    display.println("");
    display.println("A: 上一个 B: 下一个");
    display.println("C: 确认选择");
}

void display_test_instruction()
{
    display_clear();
    display.printf("测试引脚: %s", pin_map[current_pin].name);
    display.println("");
    
    if (pin_map[current_pin].has_led) {
        display.println("LED测试无需连线");
        display.println("将直接通过IO14控制");
        display.println("32个Neopixel LED");
    } else {
        display.println("请将该引脚连接到");
        display.println("ESP32的GPIO26");
    }
    
    display.println("");
    display.println("按C开始测试");
    display.println("按A跳过此引脚");
}

void display_skip_message()
{
    display_clear();
    display.printf("已跳过: %s", pin_map[current_pin].name);
    display.println("");
    display.println("正在准备下一个引脚...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 显示1秒
}

// GPIO基本测试
bool test_gpio_basic(int pin)
{
    ESP_LOGI(TAG, "开始GPIO基本测试 - 引脚 %d", pin);
    bool test_passed = true;

    // 复位引脚配置
    reset_pin_config(pin);

    gpio_reset_pin(TEST_GPIO_PIN);
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);

    // 测试输出模式
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_OUTPUT);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // 测试高电平输出
    io_expander_gpio_set_level(io_expander, pin, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int test_level_high = gpio_get_level(TEST_GPIO_PIN);
    ESP_LOGI(TAG, "输出高电平，GPIO26读取: %d", test_level_high);

    // 校验高电平输出
    if (test_level_high != 1) {
        ESP_LOGE(TAG, "高电平输出测试失败! 期望:1, 实际:%d", test_level_high);
        test_passed = false;
    }

    // 测试低电平输出
    io_expander_gpio_set_level(io_expander, pin, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int test_level_low = gpio_get_level(TEST_GPIO_PIN);
    ESP_LOGI(TAG, "输出低电平，GPIO26读取: %d", test_level_low);

    // 校验低电平输出
    if (test_level_low != 0) {
        ESP_LOGE(TAG, "低电平输出测试失败! 期望:0, 实际:%d", test_level_low);
        test_passed = false;
    }

    // 测试输入模式 - 先设置GPIO26为输出，然后测试IO_Explander是否能正确读取
    gpio_reset_pin(TEST_GPIO_PIN);
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_OUTPUT);

    // 设置IO_Explander为输入模式
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_NONE);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // 测试输入高电平
    gpio_set_level(TEST_GPIO_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t input_level_high;
    io_expander_gpio_get_level(io_expander, pin, &input_level_high);
    ESP_LOGI(TAG, "输入模式测试高电平: %d", input_level_high);

    if (input_level_high != 1) {
        ESP_LOGE(TAG, "输入高电平测试失败! 期望:1, 实际:%d", input_level_high);
        test_passed = false;
    }

    // 测试输入低电平
    gpio_set_level(TEST_GPIO_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t input_level_low;
    io_expander_gpio_get_level(io_expander, pin, &input_level_low);
    ESP_LOGI(TAG, "输入模式测试低电平: %d", input_level_low);

    if (input_level_low != 0) {
        ESP_LOGE(TAG, "输入低电平测试失败! 期望:0, 实际:%d", input_level_low);
        test_passed = false;
    }

    // 恢复GPIO26为输入模式
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);

    if (test_passed) {
        ESP_LOGI(TAG, "GPIO基本测试通过");
    } else {
        ESP_LOGE(TAG, "GPIO基本测试失败");
    }

    return test_passed;
}

// 上下拉测试
bool test_gpio_pull(int pin)
{
    ESP_LOGI(TAG, "开始上下拉测试 - 引脚 %d", pin);
    bool test_passed = true;

    // 复位引脚配置
    reset_pin_config(pin);

    // 设置IO_Explander为输入模式
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);

    // 设置GPIO26为输入模式（浮空状态）
    gpio_reset_pin(TEST_GPIO_PIN);
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TEST_GPIO_PIN, GPIO_FLOATING);

    // 测试上拉
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_UP);
    vTaskDelay(200 / portTICK_PERIOD_MS);  // 给足够时间稳定

    uint8_t io_exp_level;
    io_expander_gpio_get_level(io_expander, pin, &io_exp_level);
    int gpio26_level = gpio_get_level(TEST_GPIO_PIN);

    ESP_LOGI(TAG, "上拉模式 - IO_Exp读取: %d, GPIO26读取: %d", io_exp_level, gpio26_level);

    // 校验上拉：由于上拉电阻，引脚应该为高电平
    if (io_exp_level != 1 || gpio26_level != 1) {
        ESP_LOGE(TAG, "上拉测试失败! IO_Exp期望:1实际:%d, GPIO26期望:1实际:%d", io_exp_level, gpio26_level);
        test_passed = false;
    }

    // 测试下拉
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_DOWN);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    io_expander_gpio_get_level(io_expander, pin, &io_exp_level);
    gpio26_level = gpio_get_level(TEST_GPIO_PIN);

    ESP_LOGI(TAG, "下拉模式 - IO_Exp读取: %d, GPIO26读取: %d", io_exp_level, gpio26_level);

    // 校验下拉：由于下拉电阻，引脚应该为低电平
    if (io_exp_level != 0 || gpio26_level != 0) {
        ESP_LOGE(TAG, "下拉测试失败! IO_Exp期望:0实际:%d, GPIO26期望:0实际:%d", io_exp_level, gpio26_level);
        test_passed = false;
    }

    // 恢复无上下拉
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_NONE);

    if (test_passed) {
        ESP_LOGI(TAG, "上下拉测试通过");
    } else {
        ESP_LOGE(TAG, "上下拉测试失败");
    }

    return test_passed;
}

// 中断测试
bool test_gpio_interrupt(int pin)
{
    ESP_LOGI(TAG, "开始中断测试 - 引脚 %d", pin);
    bool test_passed = true;

    // 复位引脚配置
    reset_pin_config(pin);

    // 设置为输入模式并启用上拉
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_UP);

    // 清除之前的中断状态
    io_expander_gpio_clear_interrupt(io_expander, pin);

    // 启用下降沿中断
    io_expander_gpio_set_interrupt(io_expander, pin, IO_EXP_GPIO_INTR_FALLING_EDGE);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "中断已设置为下降沿触发，通过GPIO26产生触发信号...");

    // 检查初始中断状态应该为0
    uint16_t interrupt_status_before;
    io_expander_gpio_get_interrupt_status(io_expander, &interrupt_status_before);
    ESP_LOGI(TAG, "触发前中断状态: 0x%04x", interrupt_status_before);

    // 通过GPIO26产生下降沿来触发中断
    gpio_reset_pin(TEST_GPIO_PIN);
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TEST_GPIO_PIN, 1);  // 先设置为高电平
    vTaskDelay(100 / portTICK_PERIOD_MS);

    gpio_set_level(TEST_GPIO_PIN, 0);      // 产生下降沿
    vTaskDelay(200 / portTICK_PERIOD_MS);  // 等待中断处理

    // 检查中断状态
    uint16_t interrupt_status_after;
    io_expander_gpio_get_interrupt_status(io_expander, &interrupt_status_after);
    ESP_LOGI(TAG, "触发后中断状态: 0x%04x", interrupt_status_after);

    // 校验中断是否被正确触发
    if (interrupt_status_after & (1 << pin)) {
        ESP_LOGI(TAG, "中断触发成功！引脚 %d 的中断被检测到", pin);
        io_expander_gpio_clear_interrupt(io_expander, pin);

        // 验证中断清除
        uint16_t interrupt_status_cleared;
        io_expander_gpio_get_interrupt_status(io_expander, &interrupt_status_cleared);
        if (interrupt_status_cleared & (1 << pin)) {
            ESP_LOGE(TAG, "中断清除失败！状态: 0x%04x", interrupt_status_cleared);
            test_passed = false;
        } else {
            ESP_LOGI(TAG, "中断清除成功");
        }
    } else {
        ESP_LOGE(TAG, "中断触发失败！期望检测到中断，但状态为: 0x%04x", interrupt_status_after);
        test_passed = false;
    }

    // 恢复GPIO26为输入模式
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);

    // 禁用中断
    io_expander_gpio_set_interrupt(io_expander, pin, IO_EXP_GPIO_INTR_DISABLE);

    if (test_passed) {
        ESP_LOGI(TAG, "中断测试通过");
    } else {
        ESP_LOGE(TAG, "中断测试失败");
    }

    return test_passed;
}

// ADC测试
bool test_adc(int pin, int adc_channel)
{
    ESP_LOGI(TAG, "开始ADC测试 - 引脚 %d, 通道 %d", pin, adc_channel);
    bool test_passed = true;

    // 复位引脚配置
    reset_pin_config(pin);

    // 设置为输入模式
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_NONE);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_12);  // GPIO26对应ADC2_CHANNEL_9

    // 测试两个电压值
    float test_voltages[] = {0.0, 3.3};  // 0V, 3.3V

    for (int i = 0; i < 2; i++) {
        // 通过GPIO26输出不同的电压
        if (i == 0) {
            // 0V - GPIO26输出低电平
            gpio_reset_pin(TEST_GPIO_PIN);
            gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(TEST_GPIO_PIN, 0);
        } else {
            // 3.3V - GPIO26输出高电平
            gpio_reset_pin(TEST_GPIO_PIN);
            gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(TEST_GPIO_PIN, 1);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);  // 等待稳定

                // 读取IO_Explander的ADC值（START位会自动清零）
        uint16_t adc_value;
        esp_err_t ret = io_expander_adc_read(io_expander, adc_channel, &adc_value);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC读取失败: %d", ret);
            test_passed = false;
            continue;
        }
        
        ESP_LOGI(TAG, "ADC采样成功: 通道%d, 值%d", adc_channel, adc_value);

        float io_exp_voltage = (adc_value / 4095.0) * 3.3;

        // 对比ESP32的ADC读取
        int esp32_adc;
        ret = adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_BIT_12, &esp32_adc);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC读取失败: %d", ret);
            test_passed = false;
            continue;
        }
        float esp32_voltage = (esp32_adc / 4095.0) * 3.3;

        ESP_LOGI(TAG, "测试电压 %.2fV - IO_Exp: %d (%.2fV), ESP32: %d (%.2fV)", test_voltages[i], adc_value,
                 io_exp_voltage, esp32_adc, esp32_voltage);

        // 简化的ADC校验（只检查0V和3.3V）
        if (i == 0) {
            // 0V测试：ADC值应该接近0
            if (adc_value > 200) {  // 允许一定噪声，但不应超过200
                ESP_LOGE(TAG, "0V测试失败! ADC值过高: %d", adc_value);
                test_passed = false;
            } else {
                ESP_LOGI(TAG, "0V测试通过，ADC值: %d", adc_value);
            }
        } else {
            // 3.3V测试：ADC值应该接近4095
            if (adc_value < 2000) {
                ESP_LOGE(TAG, "3.3V测试失败! ADC值过低: %d", adc_value);
                test_passed = false;
            } else {
                ESP_LOGI(TAG, "3.3V测试通过，ADC值: %d", adc_value);
            }
        }

        // 基本范围检查
        if (adc_value > 4095) {
            ESP_LOGE(TAG, "ADC值超出范围: %d", adc_value);
            test_passed = false;
        }
    }

    // 恢复GPIO26为输入模式
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);

    if (test_passed) {
        ESP_LOGI(TAG, "ADC测试通过");
    } else {
        ESP_LOGE(TAG, "ADC测试失败");
    }

    return test_passed;
}

// PWM测试
bool test_pwm(int pin, int pwm_channel)
{
    ESP_LOGI(TAG, "开始PWM测试 - 引脚 %d, 通道 %d", pin, pwm_channel);
    bool test_passed = true;

    // 复位引脚配置
    reset_pin_config(pin);

    // 设置PWM频率
    esp_err_t ret = io_expander_pwm_set_frequency(io_expander, 5000);  // 5kHz
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM频率设置失败: %d", ret);
        return false;
    }

    // 设置PWM引脚推挽输出
    io_expander_pwm_set_duty(io_expander, pwm_channel, 0, false, true);
    io_expander_gpio_set_drive(io_expander, pin, IO_EXP_GPIO_DRIVE_PUSH_PULL);


    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_12);  // GPIO26对应ADC2_CHANNEL_9

    // 测试不同占空比
    uint8_t duty_cycles[]     = {0, 25, 50, 75, 100};
    float expected_voltages[] = {0.0, 0.825, 1.65, 2.475, 3.3};  // 期望的平均电压

    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "设置PWM占空比: %d%%", duty_cycles[i]);

        ret = io_expander_pwm_set_duty(io_expander, pwm_channel, duty_cycles[i], false, true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PWM占空比设置失败: %d", ret);
            test_passed = false;
            continue;
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);  // 等待PWM稳定

        // 多次读取ADC值并求平均，以获得更准确的PWM平均电压
        int adc_sum = 0;
        int samples = 10;
        for (int j = 0; j < samples; j++) {
            int adc_value;
            esp_err_t ret = adc2_get_raw(ADC2_CHANNEL_9, ADC_WIDTH_BIT_12, &adc_value);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ADC读取失败: %d", ret);
                test_passed = false;
                continue;
            }
            adc_sum += adc_value;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        int adc_average        = adc_sum / samples;
        float measured_voltage = (adc_average / 4095.0) * 3.3;

        ESP_LOGI(TAG, "占空比 %d%% - 测量电压: %.2fV (ADC: %d), 期望: %.2fV", duty_cycles[i], measured_voltage,
                 adc_average, expected_voltages[i]);

        // 校验PWM输出（允许一定误差）
        float voltage_error = fabs(measured_voltage - expected_voltages[i]);
        if (voltage_error > 0.5) {  // 允许500mV误差（考虑到ADC精度和PWM滤波）
            ESP_LOGE(TAG, "PWM输出电压误差过大! 误差: %.2fV", voltage_error);
            test_passed = false;
        }

        // 特殊检查0%和100%占空比
        if (duty_cycles[i] == 0 && measured_voltage > 0.2) {
            ESP_LOGE(TAG, "0%%占空比时电压过高: %.2fV", measured_voltage);
            test_passed = false;
        } else if (duty_cycles[i] == 100 && measured_voltage < 2.8) {
            ESP_LOGE(TAG, "100%%占空比时电压过低: %.2fV", measured_voltage);
            test_passed = false;
        }
    }

    // 禁用PWM
    io_expander_pwm_set_duty(io_expander, pwm_channel, 0, false, false);

    if (test_passed) {
        ESP_LOGI(TAG, "PWM测试通过");
    } else {
        ESP_LOGE(TAG, "PWM测试失败");
    }

    return test_passed;
}

// LED测试
bool test_led(int pin)
{
    ESP_LOGI(TAG, "开始LED测试 - 引脚 %d", pin);
    bool test_passed = true;

    // 复位引脚配置
    reset_pin_config(pin);

    io_expander_gpio_set_drive(io_expander, pin, IO_EXP_GPIO_DRIVE_PUSH_PULL);

    // 设置LED数量为32个
    esp_err_t ret = io_expander_led_set_count(io_expander, 32);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置LED数量失败: %d", ret);
        return false;
    }
    ESP_LOGI(TAG, "设置LED数量: 32个");

    // 定义RGB三种基本颜色 (RGB565格式，5位红，6位绿，5位蓝)
    rgb_color_t colors[3] = {
        {31, 0, 0},   // 纯红色 (最大红色值)
        {0, 63, 0},   // 纯绿色 (最大绿色值)
        {0, 0, 31}    // 纯蓝色 (最大蓝色值)
    };
    
    const char* color_names[3] = {"红色", "绿色", "蓝色"};

    // 测试三种颜色
    for (int color_idx = 0; color_idx < 3; color_idx++) {
        ESP_LOGI(TAG, "测试%s显示...", color_names[color_idx]);

        // 为所有32个LED设置相同的颜色
        for (int led_idx = 0; led_idx < 32; led_idx++) {
            ret = io_expander_led_set_color(io_expander, led_idx, colors[color_idx]);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "设置LED %d 颜色失败: %d", led_idx, ret);
                test_passed = false;
                continue;
            }
        }

        // 刷新LED显示
        ret = io_expander_led_refresh(io_expander);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "%s刷新失败: %d", color_names[color_idx], ret);
            test_passed = false;
        } else {
            ESP_LOGI(TAG, "%s显示刷新成功", color_names[color_idx]);
        }

        // 保持显示3秒，便于观察
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    // 测试渐变效果 - 每个LED显示不同颜色
    ESP_LOGI(TAG, "测试渐变效果 - 红绿蓝渐变...");
    
    for (int led_idx = 0; led_idx < 32; led_idx++) {
        rgb_color_t gradient_color;
        
        if (led_idx < 11) {
            // 前11个LED：红色到绿色渐变
            float ratio = (float)led_idx / 10.0f;
            gradient_color.r = (uint16_t)(31 * (1.0f - ratio));
            gradient_color.g = (uint16_t)(63 * ratio);
            gradient_color.b = 0;
        } else if (led_idx < 22) {
            // 中间11个LED：绿色到蓝色渐变
            float ratio = (float)(led_idx - 11) / 10.0f;
            gradient_color.r = 0;
            gradient_color.g = (uint16_t)(63 * (1.0f - ratio));
            gradient_color.b = (uint16_t)(31 * ratio);
        } else {
            // 最后10个LED：蓝色到红色渐变
            float ratio = (float)(led_idx - 22) / 9.0f;
            gradient_color.r = (uint16_t)(31 * ratio);
            gradient_color.g = 0;
            gradient_color.b = (uint16_t)(31 * (1.0f - ratio));
        }

        ret = io_expander_led_set_color(io_expander, led_idx, gradient_color);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "设置LED %d 渐变颜色失败: %d", led_idx, ret);
            test_passed = false;
        }
    }

    // 刷新渐变效果
    ret = io_expander_led_refresh(io_expander);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "渐变效果刷新失败: %d", ret);
        test_passed = false;
    } else {
        ESP_LOGI(TAG, "渐变效果显示成功");
    }

    // 保持渐变显示3秒
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // 关闭所有LED
    ESP_LOGI(TAG, "关闭所有LED...");
    rgb_color_t off_color = {0, 0, 0};  // 黑色 = 关闭
    
    for (int led_idx = 0; led_idx < 32; led_idx++) {
        ret = io_expander_led_set_color(io_expander, led_idx, off_color);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "关闭LED %d 失败: %d", led_idx, ret);
            test_passed = false;
        }
    }

    // 刷新显示关闭效果
    ret = io_expander_led_refresh(io_expander);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "关闭LED刷新失败: %d", ret);
        test_passed = false;
    } else {
        ESP_LOGI(TAG, "所有LED已关闭");
    }

    // 最后禁用LED功能，恢复GPIO功能
    io_expander_led_disable(io_expander);
    ESP_LOGI(TAG, "LED功能已禁用，IO14恢复GPIO功能");

    if (test_passed) {
        ESP_LOGI(TAG, "LED测试通过");
    } else {
        ESP_LOGE(TAG, "LED测试失败");
    }

    return test_passed;
}

// 显示测试结果
void display_test_result(const char* test_name, bool passed)
{
    if (passed) {
        display.setTextColor(GREEN);
        display.print(test_name);
        display.println(" - 通过");
    } else {
        display.setTextColor(RED);
        display.print(test_name);
        display.println(" - 失败");
    }
    display.setTextColor(BLACK);  // 恢复默认颜色
}

// 运行完整的引脚测试
void run_pin_test(int pin)
{
    ESP_LOGI(TAG, "开始测试引脚: %s", pin_map[pin].name);

    // 在开始所有测试前先复位引脚配置
    ESP_LOGI(TAG, "测试前复位引脚配置");
    reset_pin_config(pin);

    display_clear();
    display.printf("测试: %s", pin_map[pin].name);
    display.println("");
    display.println("测试进行中...");

    bool overall_result = true;

    // 1. GPIO基本功能测试
    display.println("1. GPIO基本测试...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    bool gpio_result = test_gpio_basic(pin);
    overall_result &= gpio_result;

    // 更新显示
    display_clear();
    display.printf("测试: %s", pin_map[pin].name);
    display.println("");
    display_test_result("1. GPIO基本", gpio_result);

    // 2. 上下拉测试
    display.println("2. 上下拉测试...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    bool pull_result = test_gpio_pull(pin);
    overall_result &= pull_result;

    // 更新显示
    display_clear();
    display.printf("测试: %s", pin_map[pin].name);
    display.println("");
    display_test_result("1. GPIO基本", gpio_result);
    display_test_result("2. 上下拉", pull_result);

    // 3. 中断测试
    display.println("3. 中断测试...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
    bool interrupt_result = test_gpio_interrupt(pin);
    overall_result &= interrupt_result;

    // 更新显示
    display_clear();
    display.printf("测试: %s", pin_map[pin].name);
    display.println("");
    display_test_result("1. GPIO基本", gpio_result);
    display_test_result("2. 上下拉", pull_result);
    display_test_result("3. 中断", interrupt_result);

    // 4. ADC测试（如果支持）
    bool adc_result = true;  // 默认通过，如果不支持ADC
    if (pin_map[pin].has_adc) {
        display.println("4. ADC测试...");
        vTaskDelay(200 / portTICK_PERIOD_MS);
        adc_result = test_adc(pin, pin_map[pin].adc_channel);
        overall_result &= adc_result;

        // 更新显示
        display_clear();
        display.printf("测试: %s", pin_map[pin].name);
        display.println("");
        display_test_result("1. GPIO基本", gpio_result);
        display_test_result("2. 上下拉", pull_result);
        display_test_result("3. 中断", interrupt_result);
        display_test_result("4. ADC", adc_result);
    }

    // 5. PWM测试（如果支持）
    bool pwm_result = true;  // 默认通过，如果不支持PWM
    if (pin_map[pin].has_pwm) {
        display.println("5. PWM测试...");
        vTaskDelay(200 / portTICK_PERIOD_MS);
        pwm_result = test_pwm(pin, pin_map[pin].pwm_channel);
        overall_result &= pwm_result;

        // 更新显示
        display_clear();
        display.printf("测试: %s", pin_map[pin].name);
        display.println("");
        display_test_result("1. GPIO基本", gpio_result);
        display_test_result("2. 上下拉", pull_result);
        display_test_result("3. 中断", interrupt_result);
        if (pin_map[pin].has_adc) {
            display_test_result("4. ADC", adc_result);
        }
        display_test_result("5. PWM", pwm_result);
    }

    // 6. LED测试（如果支持）
    bool led_result = true;  // 默认通过，如果不支持LED
    if (pin_map[pin].has_led) {
        display.println("6. LED测试...");
        vTaskDelay(200 / portTICK_PERIOD_MS);
        led_result = test_led(pin);
        overall_result &= led_result;

        // 更新显示
        display_clear();
        display.printf("测试: %s", pin_map[pin].name);
        display.println("");
        display_test_result("1. GPIO基本", gpio_result);
        display_test_result("2. 上下拉", pull_result);
        display_test_result("3. 中断", interrupt_result);
        if (pin_map[pin].has_adc) {
            display_test_result("4. ADC", adc_result);
        }
        if (pin_map[pin].has_pwm) {
            display_test_result("5. PWM", pwm_result);
        }
        display_test_result("6. LED", led_result);
    }

    ESP_LOGI(TAG, "引脚 %s 测试完成，总体结果: %s", pin_map[pin].name, overall_result ? "通过" : "失败");

    // 显示最终结果
    display.println("");
    if (overall_result) {
        display.setTextColor(GREEN);
        display.println("全部测试通过!");
    } else {
        display.setTextColor(RED);
        display.println("部分测试失败!");
    }
    display.setTextColor(BLACK);

    display.println("A: 下一个引脚");
    display.println("B: 重复测试");
    display.println("C: 返回菜单");
}

// 功能寄存器测试
void run_function_test() {
    display_clear();
    display.println("功能寄存器测试");
    display.println("");
    display.println("测试进行中...");
    
    bool overall_result = true;
    
    // 1. RTC RAM测试
    display.println("1. RTC RAM测试...");
    ESP_LOGI(TAG, "开始RTC RAM测试");
    
    uint8_t test_data[16] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 
                            0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10};
    uint8_t read_data[16] = {0};
    
    bool rtc_ram_test = true;
    
    // 写入测试数据
    if (io_expander_rtc_ram_write(io_expander, 0, test_data, 16) != ESP_OK) {
        ESP_LOGE(TAG, "RTC RAM写入失败");
        rtc_ram_test = false;
    } else {
        // 读取验证
        if (io_expander_rtc_ram_read(io_expander, 0, read_data, 16) != ESP_OK) {
            ESP_LOGE(TAG, "RTC RAM读取失败");
            rtc_ram_test = false;
        } else {
            // 比较数据
            for (int i = 0; i < 16; i++) {
                if (test_data[i] != read_data[i]) {
                    ESP_LOGE(TAG, "RTC RAM数据不匹配 位置%d: 期望0x%02x, 实际0x%02x", 
                             i, test_data[i], read_data[i]);
                    rtc_ram_test = false;
                    break;
                }
            }
        }
    }
    
    overall_result &= rtc_ram_test;
    
    // 2. I2C配置测试
    display.println("2. I2C配置测试...");
    ESP_LOGI(TAG, "开始I2C配置测试");
    
    bool i2c_config_test = true;
    
    // 测试I2C配置设置
    if (io_expander_set_i2c_config(io_expander, 5, false, false, false) != ESP_OK) {
        ESP_LOGE(TAG, "I2C配置设置失败");
        i2c_config_test = false;
    }
    
    overall_result &= i2c_config_test;
    
    // 3. LED控制测试
    display.println("3. LED控制测试...");
    ESP_LOGI(TAG, "开始LED控制测试");
    
    bool led_test = true;
    
    // 设置LED数量
    if (io_expander_led_set_count(io_expander, 8) != ESP_OK) {
        ESP_LOGE(TAG, "LED数量设置失败");
        led_test = false;
    } else {
        // 设置LED颜色
        rgb_color_t red_color = {31, 0, 0};  // 红色
        if (io_expander_led_set_color(io_expander, 0, red_color) != ESP_OK) {
            ESP_LOGE(TAG, "LED颜色设置失败");
            led_test = false;
        } else {
            // 刷新LED显示（REFRESH位会自动清零）
            if (io_expander_led_refresh(io_expander) != ESP_OK) {
                ESP_LOGE(TAG, "LED刷新失败");
                led_test = false;
            } else {
                ESP_LOGI(TAG, "LED刷新成功，REFRESH位已自动清零");
            }
        }
    }
    
    overall_result &= led_test;
    
    // 显示测试结果
    display_clear();
    display.println("功能寄存器测试结果:");
    display.println("");
    
    if (rtc_ram_test) {
        display.setTextColor(GREEN);
        display.println("1. RTC RAM - 通过");
    } else {
        display.setTextColor(RED);
        display.println("1. RTC RAM - 失败");
    }
    display.setTextColor(BLACK);
    
    if (i2c_config_test) {
        display.setTextColor(GREEN);
        display.println("2. I2C配置 - 通过");   
    } else {
        display.setTextColor(RED);
        display.println("2. I2C配置 - 失败");
    }
    display.setTextColor(BLACK);
    
    if (led_test) {
        display.setTextColor(GREEN);
        display.println("3. LED控制 - 通过");
    } else {
        display.setTextColor(RED);
        display.println("3. LED控制 - 失败");
    }
    display.setTextColor(BLACK);
    
    display.println("");
    if (overall_result) {
        display.setTextColor(GREEN);
        display.println("全部功能测试通过!");
    } else {
        display.setTextColor(RED);
        display.println("部分功能测试失败!");
    }
    display.setTextColor(BLACK);
    
    display.println("");
    display.println("A: 工厂重置测试");
    display.println("C: 返回菜单");
    
    ESP_LOGI(TAG, "功能寄存器测试完成，总体结果: %s", overall_result ? "通过" : "失败");
}

// 工厂重置测试
void run_factory_reset_test() {
    display_clear();
    display.println("工厂重置测试");
    display.println("");
    display.println("警告: 将重置所有设置!");
    display.println("确认按C键继续");
    display.println("取消按A键返回");
    
    // 等待用户确认
    bool waiting = true;
    while (waiting) {
        if (button_pressed(BUTTON_A_PIN)) {
            // 取消，返回功能测试
            run_function_test();
            return;
        }
        
        if (button_pressed(BUTTON_C_PIN)) {
            // 确认执行工厂重置
            display_clear();
            display.println("执行工厂重置...");
            display.println("(设置即生效，无需校验)");
            
            esp_err_t ret = io_expander_factory_reset(io_expander);
            
            vTaskDelay(2000 / portTICK_PERIOD_MS); // 等待重置完成
            
            if (ret == ESP_OK) {
                display.setTextColor(GREEN);
                display.println("工厂重置成功!");
                ESP_LOGI(TAG, "工厂重置执行成功，所有设置已恢复默认值");
                
                // 重新读取设备信息（因为可能已经改变）
                read_device_info();
            } else {
                display.setTextColor(RED);
                display.println("工厂重置失败!");
                ESP_LOGE(TAG, "工厂重置执行失败: %d", ret);
            }
            display.setTextColor(BLACK);
            
            display.println("");
            display.println("C: 返回菜单");
            
            waiting = false;
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    // 初始化显示屏
    display.begin();
    display.setRotation(1);
    display.setFont(&fonts::efontCN_12);
    display_clear();
    display.println("初始化中...");

    // 初始化按键和测试GPIO
    init_buttons();
    init_test_gpio();
    
    // 初始化I2C总线
    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = (gpio_num_t)21;
    conf.scl_io_num       = (gpio_num_t)22;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags        = 0;
    i2c_bus               = i2c_bus_create(I2C_NUM_0, &conf);

    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C总线初始化失败");
        return;
    }

    // 扫描I2C设备
    display.println("扫描I2C设备...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint8_t i2c_addr[128] = {0};
    i2c_bus_scan(i2c_bus, i2c_addr, 128);
    bool found_device = false;
    for (int i = 0; i < 128; i++) {
        if (i2c_addr[i] != 0) {
            ESP_LOGI(TAG, "发现I2C设备地址: 0x%02x", i2c_addr[i]);
            if (i2c_addr[i] == 0x6F) {
                found_device = true;
            }
        }
    }

    if (!found_device) {
        display.println("未找到IO Explander!");
        display.println("请检查连接");
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    // 初始化IO扩展器
    io_expander_config_t io_expander_config = {
        .i2c_bus        = i2c_bus,
        .device_address = 0x6F,
        .timeout_ms     = 1000,
    };

    esp_err_t ret = io_expander_init(&io_expander_config, &io_expander);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IO扩展器初始化失败: %d", ret);
        display.println("IO扩展器初始化失败!");
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    // 读取设备信息
    uint16_t uid;
    uint8_t hw_version, fw_version;
    if (io_expander_read_uid(io_expander, &uid) == ESP_OK) {
        ESP_LOGI(TAG, "设备UID: 0x%04x", uid);
    }
    if (io_expander_read_version(io_expander, &hw_version, &fw_version) == ESP_OK) {
        ESP_LOGI(TAG, "硬件版本: %d, 固件版本: %d", hw_version, fw_version);
    }

    ESP_LOGI(TAG, "IO扩展器初始化成功，读取设备信息");

    // 读取设备信息
    read_device_info();
    
    ESP_LOGI(TAG, "开始主循环");
    display_menu();

    // 主循环
    while (1) {
        // 检测按键
        if (button_pressed(BUTTON_A_PIN)) {
            switch (current_state) {
                case TEST_STATE_MENU:
                    current_state = TEST_STATE_SELECT_PIN;
                    display_pin_select();
                    break;
                case TEST_STATE_SELECT_PIN:
                    current_pin = (current_pin - 1 + 14) % 14;
                    display_pin_select();
                    break;
                case TEST_STATE_RUNNING:
                    // 跳过当前引脚测试，直接到下一个引脚
                    ESP_LOGI(TAG, "跳过引脚 %s 的测试", pin_map[current_pin].name);
                    display_skip_message();
                    current_pin = (current_pin + 1) % 14;
                    display_test_instruction();
                    break;
                case TEST_STATE_WAIT_NEXT:
                    // 下一个引脚
                    current_pin   = (current_pin + 1) % 14;
                    current_state = TEST_STATE_RUNNING;
                    display_test_instruction();
                    break;
                case TEST_STATE_FUNCTION_TEST:
                    // A键执行工厂重置测试
                    run_factory_reset_test();
                    break;
                default:
                    break;
            }
        }

        if (button_pressed(BUTTON_B_PIN)) {
            switch (current_state) {
                case TEST_STATE_MENU:
                    current_state = TEST_STATE_RUNNING;
                    display_test_instruction();
                    break;
                case TEST_STATE_SELECT_PIN:
                    current_pin = (current_pin + 1) % 14;
                    display_pin_select();
                    break;
                case TEST_STATE_WAIT_NEXT:
                    // 重复测试当前引脚
                    run_pin_test(current_pin);
                    current_state = TEST_STATE_WAIT_NEXT;  // 保持在等待状态
                    break;
                default:
                    break;
            }
        }

        if (button_pressed(BUTTON_C_PIN)) {
            switch (current_state) {
                case TEST_STATE_MENU:
                    // C键改为功能测试
                    current_state = TEST_STATE_FUNCTION_TEST;
                    run_function_test();
                    break;
                case TEST_STATE_SELECT_PIN:
                    current_state = TEST_STATE_RUNNING;
                    display_test_instruction();
                    break;
                case TEST_STATE_RUNNING:
                    // 开始测试
                    run_pin_test(current_pin);
                    current_state = TEST_STATE_WAIT_NEXT;
                    break;
                case TEST_STATE_WAIT_NEXT:
                    current_state = TEST_STATE_MENU;
                    display_menu();
                    break;
                case TEST_STATE_FUNCTION_TEST:
                    current_state = TEST_STATE_MENU;
                    display_menu();
                    break;
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}