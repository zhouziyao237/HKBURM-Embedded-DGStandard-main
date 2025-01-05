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

#include "shoot_task_CAM.h"

//static driver::MotorPWMBase* flywheel_left = nullptr;
//static driver::MotorPWMBase* flywheel_right = nullptr;

static driver::Motor3508* flywheel_left = nullptr;
static driver::Motor3508* flywheel_right = nullptr;

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

control::ConstrainedPID::PID_Init_t flywheel_pid_init = {
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
    .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
            control::ConstrainedPID::OutputFilter |         // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
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

void steering_stop(){
    steering_motor->SetTarget(0);
    steering_motor->Hold(true);
}

void Steering_IDLE(){
    steering_motor->SetTarget(4 * PI / 8, false);
    osDelay(10);
}

void Steering_SINGLE(){
    int last_steering_target = steering_motor->GetTheta();
    steering_motor->SetTarget(PI, false);
    while (true){
        if(last_steering_target != steering_motor->GetTheta()){
            break;
        } else {
            osDelay(SHOOT_OS_DELAY);
        }
    }
    steering_motor->Hold(true);
    shoot_load_mode = SHOOT_MODE_IDLE;
}

void Stering_BURST(){
    steering_motor->SetTarget(steering_motor->GetTarget() + 2 * PI / 8, false);
    osDelay(10);
}
osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
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

        // if (remote_mode == REMOTE_MODE_KILL ||
        //     !referee->game_robot_status.mains_power_shooter_output) {
        if (remote_mode == REMOTE_MODE_KILL) { //(临时解决方案)
            // 死了
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }

        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING ||
            shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARED) {
            // 启动摩擦轮电机(旋转)
            flywheel_speed_ease->SetTarget(5);  // 摩擦轮转速(转速控制)
        } else {
            // 关闭摩擦轮电机(停止旋转)
            flywheel_speed_ease->SetTarget(0);    // 摩擦轮转速(转速控制)
            shoot_load_mode = SHOOT_MODE_STOP;
        }

        flywheel_speed_ease->Calc();
        flywheel_left->SetTarget(flywheel_speed_ease->GetOutput());
        flywheel_right->SetTarget(-flywheel_speed_ease->GetOutput());

        // 检测摩擦轮是否就绪
        // 由shoot_task切换到SHOOT_FRIC_MODE_PREPARED
        if (shoot_flywheel_mode == SHOOT_FRIC_MODE_PREPARING && flywheel_speed_ease->IsAtTarget()) {
            shoot_flywheel_mode = SHOOT_FRIC_MODE_PREPARED;
        }

        /* 摩擦轮就绪后执行以下部分 */

        if (shoot_load_mode == SHOOT_MODE_STOP) {
            steering_stop();
        }

        if (shoot_load_mode == SHOOT_MODE_IDLE){
            steering_stop();
        }

//                 todo 临时解决方案
//        int heat_limit = referee->game_robot_status.shooter_heat_limit;
//        int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
//        const int shooter_heat_threashold = 50000;
//        if (heat_buffer > heat_limit - shooter_heat_threashold) {
//            steering_stop();
//            print("overheat!\n");
//            osDelay(SHOOT_OS_DELAY);
//            continue;
//        }
//        UNUSED(heat_limit);
//        UNUSED(heat_buffer);

        if (shoot_load_mode == SHOOT_MODE_SINGLE) {
            Steering_SINGLE();
        }

        if (shoot_load_mode == SHOOT_MODE_BURST) {
            Stering_BURST();
        }

        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {

    flywheel_left = new driver::Motor3508(can2, 0x203);
    flywheel_right = new driver::Motor3508(can2, 0x202);
    steering_motor = new driver::Motor2006(can2, 0x204);

    flywheel_left->SetTransmissionRatio(36);
    flywheel_right->SetTransmissionRatio(36);
    steering_motor->SetTransmissionRatio(19);
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
    steering_motor->SetMode(driver::MotorCANBase::OMEGA);
    steering_motor->RegisterErrorCallback(jam_callback, steering_motor);

    flywheel_left->ReInitPID(flywheel_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_left->SetMode(driver::MotorCANBase::OMEGA);
    flywheel_left->RegisterErrorCallback(jam_callback, flywheel_left);

    flywheel_right->ReInitPID(flywheel_pid_init, driver::MotorCANBase::OMEGA);
    flywheel_right->SetMode(driver::MotorCANBase::OMEGA);
    flywheel_right->RegisterErrorCallback(jam_callback, flywheel_right);

    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);

    shoot_key = new bsp::GPIO(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);

    if(CAR_DEBUG->RM_Debug_mode.shooter_debug_mode == Debug_true){
        print("SHOOT_INIT_OK\n");
    }
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    steering_motor->SetTarget(0);
    flywheel_left->SetTarget(0);
    flywheel_right->SetTarget(0);
}

void shoot_remote_mode(){
    char s[20];
    switch (shoot_flywheel_mode) {
        case SHOOT_FRIC_MODE_PREPARING:
            strcpy(s, "PREPAREING");
            break;
        case SHOOT_FRIC_MODE_STOP:
            strcpy(s, "STOP");
            break;
        case SHOOT_FRIC_MODE_PREPARED:
            strcpy(s, "PREPARED");
            break;
        case SHOOT_FRIC_MODE_DISABLE:
            strcpy(s, "DISABLE");
            break;
        default:
            strcpy(s, "UNKNOWN");
            break;
    }

    print("Shoot Fric Mode:%s\r\n", s);
    switch (shoot_load_mode) {
        case SHOOT_MODE_STOP:
            strcpy(s, "STOP");
            break;
        case SHOOT_MODE_IDLE:
            strcpy(s, "IDLE");
            break;
        case SHOOT_MODE_DISABLE:
            strcpy(s, "DISABLE");
            break;
        case SHOOT_MODE_UNLOAD:
            strcpy(s, "UNLOAD");
            break;
        case SHOOT_MODE_SINGLE:
            strcpy(s, "SINGLE");
            break;
        case SHOOT_MODE_BURST:
            strcpy(s, "BURST");
            break;
    }
    print("Shoot Mode:%s\r\n", s);
}