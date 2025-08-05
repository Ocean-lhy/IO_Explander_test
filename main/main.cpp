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

// 测试状态
typedef enum {
    TEST_STATE_MENU,        // 菜单状态
    TEST_STATE_SELECT_PIN,  // 选择引脚状态
    TEST_STATE_RUNNING,     // 测试运行状态
    TEST_STATE_WAIT_NEXT    // 等待下一个测试状态
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

// 初始化测试GPIO
void init_test_gpio()
{
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << TEST_GPIO_PIN),
                             .mode         = GPIO_MODE_INPUT,
                             .pull_up_en   = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type    = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    // 配置ADC用于模拟读取
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);  // GPIO26对应ADC1_CHANNEL_7
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
    display.println("");
    display.println("A: 选择引脚");
    display.println("B: 开始测试");
    display.println("C: 退出");
    display.println("");
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
    display.println("请将该引脚连接到");
    display.println("ESP32的GPIO26");
    display.println("");
    display.println("连接完成后按C开始");
    display.println("按A跳过此引脚");
}

// GPIO基本测试
bool test_gpio_basic(int pin)
{
    ESP_LOGI(TAG, "开始GPIO基本测试 - 引脚 %d", pin);
    bool test_passed = true;

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

    // 设置IO_Explander为输入模式
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);

    // 设置GPIO26为输入模式（浮空状态）
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

    // 设置为输入模式
    io_expander_gpio_set_mode(io_expander, pin, IO_EXP_GPIO_MODE_INPUT);
    io_expander_gpio_set_pull(io_expander, pin, IO_EXP_GPIO_PULL_NONE);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 测试多个电压值
    float test_voltages[] = {0.0, 1.65, 3.3};  // 0V, 1.65V(中值), 3.3V

    for (int i = 0; i < 3; i++) {
        // 通过GPIO26输出不同的电压（使用DAC或PWM模拟）
        if (i == 0) {
            // 0V - GPIO26输出低电平
            gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(TEST_GPIO_PIN, 0);
        } else if (i == 1) {
            // 1.65V - 通过50%占空比PWM模拟（这只是近似）
            gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);
            // 实际应用中可能需要外部分压电路
        } else {
            // 3.3V - GPIO26输出高电平
            gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_OUTPUT);
            gpio_set_level(TEST_GPIO_PIN, 1);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);  // 等待稳定

        // 读取IO_Explander的ADC值
        uint16_t adc_value;
        esp_err_t ret = io_expander_adc_read(io_expander, adc_channel, &adc_value);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC读取失败: %d", ret);
            test_passed = false;
            continue;
        }

        float io_exp_voltage = (adc_value / 4095.0) * 3.3;

        // 对比ESP32的ADC读取
        int esp32_adc       = adc1_get_raw(ADC1_CHANNEL_7);
        float esp32_voltage = (esp32_adc / 4095.0) * 3.3;

        ESP_LOGI(TAG, "测试电压 %.2fV - IO_Exp: %d (%.2fV), ESP32: %d (%.2fV)", test_voltages[i], adc_value,
                 io_exp_voltage, esp32_adc, esp32_voltage);

        // 校验ADC读取（允许一定误差）
        float voltage_diff = fabs(io_exp_voltage - esp32_voltage);
        if (voltage_diff > 0.3) {  // 允许300mV误差
            ESP_LOGE(TAG, "ADC读取差异过大! 差异: %.2fV", voltage_diff);
            test_passed = false;
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

    // 设置PWM频率
    esp_err_t ret = io_expander_pwm_set_frequency(io_expander, 1000);  // 1kHz
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM频率设置失败: %d", ret);
        return false;
    }

    // 设置GPIO26为输入，用于ADC检测PWM输出
    gpio_set_direction(TEST_GPIO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TEST_GPIO_PIN, GPIO_FLOATING);

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
            adc_sum += adc1_get_raw(ADC1_CHANNEL_7);
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

    ESP_LOGI(TAG, "IO扩展器初始化成功，开始主循环");
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
                case TEST_STATE_WAIT_NEXT:
                    // 下一个引脚
                    current_pin   = (current_pin + 1) % 14;
                    current_state = TEST_STATE_RUNNING;
                    display_test_instruction();
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
                    ESP_LOGI(TAG, "退出测试");
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
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}