/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "shoot_task_PWM.h"

static driver::MotorPWMBase* flywheel_left = nullptr;
static driver::MotorPWMBase* flywheel_right = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

bsp::GPIO* shoot_key = nullptr;

bool jam_notify_flags = false;

control::ConstrainedPID::PID_Init_t steering_motor_theta_normal_pid_init = {
    .kp = 20,
    .ki = 0,
    .kd = 0,
    .max_out = 2 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};
control::ConstrainedPID::PID_Init_t steering_motor_theta_fast_pid_init = {
    .kp = 25,
    .ki = 0,
    .kd = 0,
    .max_out = 4 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};
control::ConstrainedPID::PID_Init_t steering_motor_theta_burst_pid_init = {
    .kp = 30,
    .ki = 0,
    .kd = 0,
    .max_out = 5 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};

void jam_callback(void* args) {
    driver::Motor2006* motor = static_cast<driver::Motor2006*>(args);
    jam_notify_flags = true;
    float target = motor->GetTarget();
    if (target > motor->GetOutputShaftTheta()) {
        float prev_target = motor->GetTarget() - 2 * PI / 32;
        motor->SetTarget(prev_target);
    } else {
        float prev_target = motor->GetTarget() + 2 * PI / 32;
        motor->SetTarget(prev_target);
    }
}

osThreadId_t shootTaskHandle;

/**
     * @brief 自动卸载弹药函数
     *
     * 此函数用于根据裁判系统的射击数据和电机的输出轴角速度，自动调整转向电机的目标角度，以实现弹药的自动卸载。
     *
     * 首先，检查裁判系统的射击数据中是否设置了锁定转子时间的标志（htim_sign）。
     * 如果未设置（即htim_sign为false），则记录当前时间（HAL_GetTick()）作为锁定转子时间，并将htim_sign设置为true。
     *
     * 接着，检查电机的输出轴角速度（GetOutputShaftOmega()）与裁判系统射击数据中的锁定转子角度（locked_rotor_angle）之差是否大于π/36（2 * PI / 72）。
     * 同时，确保htim_sign为true，并且从锁定转子时间到现在的时间差（以秒为单位）大于5秒。
     * 如果以上条件都满足，则调整转向电机的目标角度（SetTarget），将其减小π/36，并将htim_sign设置为false。
     *
     * 注意：此函数依赖于外部定义的referee和steering_motor对象，以及它们的成员函数。
     * HAL_GetTick()函数用于获取自系统启动以来的毫秒数。
 */
void auto_unload_ammunition(){

    if(referee->shoot_data.htim_sign == false) {
//        referee->shoot_data.locked_rotor_time = HAL_GetTick();
        referee->shoot_data.htim_sign = true;
    }
    if(steering_motor->GetOutputShaftOmega() - referee->shoot_data.locked_rotor_angle > 2 * PI / 72 && referee->shoot_data.htim_sign == true && (HAL_GetTick() - referee->shoot_data.locked_rotor_time)/1000.0f > 5){
        steering_motor->SetTarget(steering_motor->GetTarget() - 2 * PI / 72, false);
        referee->shoot_data.htim_sign = false;
    }
}
void auto_unload_ammunition_new(){
    if(referee->shoot_data.htim_sign == false) {
//        referee->shoot_data.locked_rotor_time = HAL_GetTick();
        referee ->shoot_data.locked_rotor_time = bsp::GetHighresTickMicroSec();
        referee->shoot_data.htim_sign = true;
        shoot_load_mode = SHOOT_MODE_UNLOAD;
    }
    if(steering_motor->GetOutputShaftOmega() - referee->shoot_data.locked_rotor_angle > 2 * PI / 72 && referee->shoot_data.htim_sign == true && (HAL_GetTick() - referee->shoot_data.locked_rotor_time)/1000.0f > 5){
        steering_motor->SetTarget(steering_motor->GetTarget() - 2 * PI / 72, false);
        referee->shoot_data.htim_sign = false;
        shoot_load_mode = SHOOT_MODE_IDLE;
    }
}

void
shootTask(void* arg) {
    UNUSED(arg);
    // 启动等待作为锁定转子时间，并将htim_sign设置为true。
    osDelay(1000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }
    // 等待IMU初始化
    while (ahrs->IsCailbrated()) {
        osDelay(SHOOT_OS_DELAY);
    }

    steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());

    Ease* flywheel_speed_ease = new Ease(0, 0.3);

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL ||
            !referee->game_robot_status.mains_power_shooter_output) {
            // 死了
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
            shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
            flywheel_speed_ease->SetTarget(900);
        } else {
            flywheel_speed_ease->SetTarget(0);
        }
        flywheel_speed_ease->Calc();
        flywheel_left->SetOutput(flywheel_speed_ease->GetOutput());
        flywheel_right->SetOutput(flywheel_speed_ease->GetOutput());

        // 检测摩擦轮是否就绪
        // 由shoot_task切换到SHOOT_FRIC_MODE_PREPARED
        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING && flywheel_speed_ease->IsAtTarget()) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
        }

        if (shoot_flywheel_mode != SHOOT_FRIC_MODE_PREPARED) {
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta());
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        /* 摩擦轮就绪后执行以下部分 */

        if (shoot_load_mode == SHOOT_MODE_STOP) {
            steering_motor->Hold();
            // todo: 这里为什么要是true才能停下？target_angel为什么会超过范围？
        }
        if (shoot_load_mode == SHOOT_MODE_IDLE) {
            uint8_t loaded = shoot_key->Read();
            if (loaded) {
                steering_motor->Hold(true);
            } else {
                // 没有准备就绪，则旋转拔弹电机
                referee->shoot_data.locked_rotor_angle = steering_motor->GetOutputShaftOmega();
                print("locked_rotor_angle: %d\n", referee->shoot_data.locked_rotor_angle);
                steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, false);
                auto_unload_ammunition_new();

            }
        }

        int heat_limit = referee->game_robot_status.shooter_heat_limit;
        int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
        const int shooter_heat_threashold = 25;
        if (heat_buffer > heat_limit - shooter_heat_threashold) {
            // 临时解决方案
            steering_motor->Hold();
            print("overheat!\n");
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        UNUSED(heat_limit);
        UNUSED(heat_buffer);

        if (shoot_load_mode == SHOOT_MODE_SINGLE) {
            referee->shoot_data.locked_rotor_angle = steering_motor->GetOutputShaftOmega();
            print("locked_rotor_angle: %d\n", referee->shoot_data.locked_rotor_angle);
            steering_motor->SetTarget(steering_motor->GetOutputShaftTheta() + 2 * PI / 8, true);
            auto_unload_ammunition_new();
            uint8_t loaded = shoot_key->Read();
            // 等到这一粒子弹发射出去再变成IDLE，否则会因为当前有子弹而直接锁定造成无法发射子弹
            if (!loaded) {
                shoot_load_mode = SHOOT_MODE_IDLE;
            }
        }
        if (shoot_load_mode == SHOOT_MODE_BURST) {
            referee->shoot_data.locked_rotor_angle = steering_motor->GetOutputShaftOmega();
            print("locked_rotor_angle: %d\n", referee->shoot_data.locked_rotor_angle);
            steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, true);
            auto_unload_ammunition_new();
        }

        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
//    flywheel_left = new driver::MotorPWMBase(&htim1, 1, 1000000, 500, 1000);
//    flywheel_right = new driver::MotorPWMBase(&htim1, 4, 1000000, 500, 1000);
    flywheel_left = new driver::MotorPWMBase(&htim1, 1, 1000000, 500, 1000);
    flywheel_right = new driver::MotorPWMBase(&htim1, 4, 1000000, 500, 1000);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);

    steering_motor = new driver::Motor2006(can1, 0x207);

    steering_motor->SetTransmissionRatio(36);
    control::ConstrainedPID::PID_Init_t steering_motor_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 2 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    steering_motor->ReInitPID(steering_motor_theta_pid_init, driver::MotorCANBase::THETA);
    control::ConstrainedPID::PID_Init_t steering_motor_omega_pid_init = {
        .kp = 1000,
        .ki = 1,
        .kd = 0,
        .max_out = 10000,
        .max_iout = 4000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
                control::ConstrainedPID::OutputFilter |          // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
                control::ConstrainedPID::ErrorHandle,            // 错误处理

    };
    steering_motor->ReInitPID(steering_motor_omega_pid_init, driver::MotorCANBase::OMEGA);
    steering_motor->SetMode(driver::MotorCANBase::THETA | driver::MotorCANBase::OMEGA);

    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);

    shoot_key = new bsp::GPIO(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);
    if(CAR_DEBUG->RM_Debug_mode.shooter_debug_mode == Debug_true){
        print("SHOOT_INIT_OK\n");
    }
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
}