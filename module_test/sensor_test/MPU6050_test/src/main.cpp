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
#include "main.h"

#include "bsp_i2c.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"

// clang-format off
#include "MPU6050.h"

// clang-format on

static bsp::I2C* i2c3 = nullptr;
static mpu::MPU6050* mpu6050 = nullptr;


void RM_RTOS_Init(void) {
    HAL_Delay(500);
    print_use_uart(&huart1);
    HAL_Delay(500);
    print("MPU6050_test Test\r\n");

}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (true) {
        clear_screen();
    }
}
