#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 MPU6050 (参数：SDA 引脚, SCL 引脚, I2C 端口号)
bool ei_mpu6050_init(int sda_pin, int scl_pin, int i2c_port_num);

// 读取一次数据并填充到 buffer (加速度 x, y, z)
bool ei_mpu6050_sample(float *out_buffer, size_t out_len);

// 停止驱动
void ei_mpu6050_stop();

// 注册到 Edge Impulse 框架
void ei_mpu6050_register(void);

#ifdef __cplusplus
}
#endif