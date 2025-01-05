//
// Created by Administrator on 2025/1/2.
//
#include "MPU6050.h"

namespace mpu {

    MPU6050::MPU6050(MPU6050_init_t init) {
        i2c_ = init.i2c;
    }

    void MPU6050::Init() {
        bsp::i2c_mode_e old_mode = i2c_->GetMode();
        i2c_->SetMode(bsp::I2C_MODE_BLOCKING);

        // 初始化配置寄存器
        //解除休眠状态
        uint8_t SendData  = 0x00;
        i2c_->MemoryWrite(0xD1, MPU6050_PWR_MGMT_1, &SendData, 1);

        //采样率分频器
        SendData = 0x07;
        i2c_->MemoryWrite(0xD1, MPU6050_SMPLRT_DIV, &SendData, 1);
        //低通滤波器
        SendData = 0x06;
        i2c_->MemoryWrite(0xD1, MPU6050_CONFIG, &SendData, 1);
        //陀螺仪
        SendData = 0x08;   //± 500 °/s
        i2c_->MemoryWrite(0xD1, MPU6050_GYRO_CONFIG, &SendData, 1);
        //加速度计
        SendData = 0x00;   //± 2g
        i2c_->MemoryWrite(0xD1, MPU6050_ACCEL_CONFIG, &SendData, 1);

    }

    void MPU6050::Read_MPU_Data(){
        i2c_->MemoryRead();
    }
}