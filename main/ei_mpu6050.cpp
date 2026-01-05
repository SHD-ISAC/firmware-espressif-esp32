#include "ei_fusion.h"
#include "ei_mpu6050.h"
#include <esp_log.h>
#include "driver/i2c.h"
#include <string.h>

static const char *TAG = "ei_mpu6050";
static i2c_port_t s_i2c_port = I2C_NUM_0;
static const uint8_t MPU_ADDR = 0x68;

// --- 静态缓存 ---
static float fusion_data_buffer[3]; 

// --- I2C 读写辅助函数 ---
static esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(uint8_t reg_addr, uint8_t *data, size_t len) {
    if (len == 0) return ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// --- 采样函数 ---
bool ei_mpu6050_sample(float *out_buffer, size_t out_len) {
    if (!out_buffer || out_len < 3) return false;
    uint8_t data[6];
    
    // 读取加速度 (0x3B)
    if (i2c_read_regs(0x3B, data, 6) != ESP_OK) { 
        ESP_LOGE(TAG, "I2C Read Failed");
        return false; 
    }

    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);
    
    // 量程 +/- 2g
    const float scale = 16384.0f;
    const float g = 9.81f;
    
    out_buffer[0] = ((float)ax / scale) * g;
    out_buffer[1] = ((float)ay / scale) * g;
    out_buffer[2] = ((float)az / scale) * g;
    
    return true;
}

static float *mpu6050_read_data_wrapper(int n_samples) {
    ei_mpu6050_sample(fusion_data_buffer, 3);
    return fusion_data_buffer;
}

static ei_device_fusion_sensor_t mpu6050_sensor = {
    .name = "MPU6050",
    .num_axis = 3,
    .frequencies = { 62.5f, 100.0f, 0 },
    .sensors = { 
        { "accX", "m/s2" },
        { "accY", "m/s2" },
        { "accZ", "m/s2" }
    },
    .read_data = &mpu6050_read_data_wrapper,
    .axis_flag_used = 0
};

bool ei_mpu6050_init(int sda_pin, int scl_pin, int i2c_port_num) {
    s_i2c_port = (i2c_port_num == 1) ? I2C_NUM_1 : I2C_NUM_0;
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)sda_pin;
    conf.scl_io_num = (gpio_num_t)scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(s_i2c_port, &conf);
    i2c_driver_install(s_i2c_port, conf.mode, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    // [参考卖家代码优化] 1. 复位设备
    i2c_write_byte(0x6B, 0x80); 
    vTaskDelay(pdMS_TO_TICKS(100));

    // [参考卖家代码优化] 2. 唤醒并设置时钟源为 PLL X-Gyro (比内部时钟更稳)
    // 寄存器 0x6B: 0x01 (CLKSEL=1)
    esp_err_t r = i2c_write_byte(0x6B, 0x01);
    
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 Init Failed: %d", r);
    } else {
        ESP_LOGI(TAG, "MPU6050 Woke up (PLL Clock)");
    }

    // [参考卖家代码优化] 3. 配置陀螺仪量程 +/- 250dps (虽然这里只采加速度，但配置好是个好习惯)
    // 寄存器 0x1B: 0x00
    i2c_write_byte(0x1B, 0x00);

    // [参考卖家代码优化] 4. 配置加速度量程 +/- 2g
    // 寄存器 0x1C: 0x00
    i2c_write_byte(0x1C, 0x00);

    // 验证 ID
    uint8_t who = 0;
    i2c_read_regs(0x75, &who, 1);
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02x", who);

    if (ei_add_sensor_to_fusion_list(mpu6050_sensor) == false) {
        ESP_LOGE(TAG, "Failed to register MPU6050!");
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 registered!");
    return true;
}