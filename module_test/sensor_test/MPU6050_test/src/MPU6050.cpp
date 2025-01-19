/*###########################################################
 # Copyright (c) 2025. BNU-HKBU UIC RoboMaster              #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

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
        i2c_->SetMode(bsp::I2C_MODE_DMA);

        // 初始化配置寄存器
        uint8_t SendData  = 0x80;         //解除休眠状态
        i2c_->MemoryWrite(0xD1, MPU6050_PWR_MGMT_1, &SendData, 1);
        SendData = 0x07;                  //采样率分频器
        i2c_->MemoryWrite(0xD1, MPU6050_SMPLRT_DIV, &SendData, 1);
        SendData = 0x06;                  //数字低通滤波器配置
        i2c_->MemoryWrite(0xD1, MPU6050_CONFIG, &SendData, 1);
        SendData = 0x08;                  //陀螺仪(± 250 °/s, ± 500 °/s, ± 1000 °/s, ± 2000 °/s) ± 500 °/s
        i2c_->MemoryWrite(0xD1, MPU6050_GYRO_CONFIG, &SendData, 1);
        SendData = 0x00;                  //加速度计(± 2g, ± 4g, ± 8g, ± 16g) ± 2g
        i2c_->MemoryWrite(0xD1, MPU6050_ACCEL_CONFIG, &SendData, 1);

    }

    void MPU6050::Read_MPU_Data(){
        i2c_->MemoryRead();
    }
}