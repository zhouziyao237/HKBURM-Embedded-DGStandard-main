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

#include "chassis.h"

#include "bsp_error_handler.h"
#include "bsp_os.h"

namespace control {

    Chassis::Chassis(const chassis_t chassis) {
        // acquired from user
        model_ = chassis.model;

        // data initialization using acquired model
        switch (chassis.model) {
                // 麦克纳姆轮
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                // 新建电机关联
                motors_ = new driver::MotorCANBase*[FourWheel::motor_num];
                motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
                motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
                motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
                motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

                // 功率限制系统
                power_limit_ = new PowerLimit(FourWheel::motor_num);

                // 速度初始化
                speeds_ = new float[FourWheel::motor_num];
                for (int i = 0; i < FourWheel::motor_num; ++i)
                    speeds_[i] = 0;

                // 轮子数
                wheel_num_ = FourWheel::motor_num;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
        chassis_offset_ = chassis.offset;
        power_limit_on_ = chassis.power_limit_on;
        if (power_limit_on_)
            driver::MotorCANBase::RegisterPreOutputCallback(UpdatePowerLimitWrapper, this);
        // 底盘是否有超级电容
        if (chassis.has_super_capacitor) {
            has_super_capacitor_ = true;
            super_capacitor_ = chassis.super_capacitor;
        }
    }

    Chassis::~Chassis() {
        driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); }, nullptr);
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                motors_[FourWheel::front_left] = nullptr;
                motors_[FourWheel::front_right] = nullptr;
                motors_[FourWheel::back_left] = nullptr;
                motors_[FourWheel::back_right] = nullptr;
                delete[] motors_;
                motors_ = nullptr;

                delete[] speeds_;
                speeds_ = nullptr;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetSpeed(const float x_speed, const float y_speed, const float turn_speed) {
        Heartbeat();
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                float scale = 1;
                float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
                scale = move_sum > max_motor_speed_ ? max_motor_speed_ / move_sum : 1.0f;
                speeds_[FourWheel::front_left] =
                    scale * (y_speed + x_speed + turn_speed * (1 - chassis_offset_));  // 2
                speeds_[FourWheel::back_left] =
                    scale * (y_speed - x_speed + turn_speed * (1 + chassis_offset_));  // 3
                speeds_[FourWheel::front_right] =
                    -scale * (y_speed - x_speed - turn_speed * (1 - chassis_offset_));  // 1
                speeds_[FourWheel::back_right] =
                    -scale * (y_speed + x_speed - turn_speed * (1 + chassis_offset_));  // 4
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetPower(bool power_limit_on, float power_limit, float chassis_power,
                           float chassis_power_buffer, bool enable_supercap) {
        if (!power_limit_on_ && power_limit_on) {
            driver::MotorCANBase::RegisterPreOutputCallback(UpdatePowerLimitWrapper, this);
        } else if (power_limit_on_ && !power_limit_on) {
            driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); },
                                                            nullptr);
        }
        power_limit_on_ = power_limit_on;
        power_limit_info_.power_limit = power_limit;
        power_limit_info_.WARNING_power = power_limit * 0.9;
        power_limit_info_.WARNING_power_buff = 50;

        // 检测功率是否发生变化，如果发生变化则更新超级电容
        current_chassis_power_ = chassis_power;

        current_chassis_power_buffer_ = chassis_power_buffer;
        if (has_super_capacitor_ && super_capacitor_->IsOnline()) {
            if (enable_supercap && !super_capacitor_enable_) {
                super_capacitor_enable_ = true;
            } else if (!enable_supercap && super_capacitor_enable_) {
                super_capacitor_enable_ = false;
            }

            super_capacitor_->SetPowerTotal(max(power_limit_info_.power_limit - 25.0f, 30.0f));
        }
    }

    void Chassis::Update() {
        /*
         * 判断电机是否都在线
        * */
        bool need_shutdown = !IsOnline();
        for (int i = 0; i < wheel_num_; i++) {
            if (!motors_[i]->IsOnline()) {
                need_shutdown = true;
                break;
            }
        }
        // todo: ???什么东西这是
        if (need_shutdown) {
            Disable();
        }

        if (has_super_capacitor_ && super_capacitor_->IsOnline()) {
            super_capacitor_->TransmitSettings();
            super_capacitor_->UpdateCurrentBuffer(current_chassis_power_buffer_);
        }

        if (!chassis_enable_) {
            for (int i = 0; i < wheel_num_; i++) {
                motors_[i]->Disable();
            }
            return;
        } else {
            for (int i = 0; i < wheel_num_; i++) {
                if (!motors_[i]->IsEnable())
                    motors_[i]->Enable();
            }
        }

        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL:
                power_limit_info_.buffer_total_current_limit = 3500 * FourWheel::motor_num;
                power_limit_info_.power_total_current_limit =
                    5000 * FourWheel::motor_num / 80.0 * power_limit_info_.power_limit;
                break;

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }

        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                motors_[FourWheel::front_left]->SetTarget(speeds_[FourWheel::front_left]);
                motors_[FourWheel::front_right]->SetTarget(speeds_[FourWheel::front_right]);
                motors_[FourWheel::back_left]->SetTarget(speeds_[FourWheel::back_left]);
                motors_[FourWheel::back_right]->SetTarget(speeds_[FourWheel::back_right]);
//                print("motor_run\r\n");
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    /**
     * @brief 包装函数，用于处理CAN桥接更新事件中的XY坐标
     *
     * 该函数是一个包装函数，用于处理CAN桥接更新事件中的XY坐标。
     * 它将接收到的扩展ID和数据传递给Chassis类的CanBridgeUpdateEventXY方法。
     *
     * @param ext_id CAN桥接的扩展ID
     * @param data CAN桥接的数据
     * @param args 指向Chassis类的指针
     */
    void Chassis::CanBridgeUpdateEventXYWrapper(communication::can_bridge_ext_id_t ext_id,
                                                communication::can_bridge_data_t data, void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventXY(ext_id, data);
    }

    /**
     * @brief 包装函数，用于处理CAN桥接更新事件中的转向信息
     *
     * 该函数是一个包装函数，用于处理CAN桥接更新事件中的转向信息。
     * 它将接收到的扩展ID和数据传递给Chassis类的CanBridgeUpdateEventTurn方法。
     *
     * @param ext_id CAN桥接的扩展ID
     * @param data CAN桥接的数据
     * @param args 指向Chassis类的指针
     */
    void Chassis::CanBridgeUpdateEventTurnWrapper(communication::can_bridge_ext_id_t ext_id,
                                                  communication::can_bridge_data_t data,
                                                  void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventTurn(ext_id, data);
    }

    /**
     * @brief 包装函数，用于处理CAN桥接更新事件中的功率限制
     *
     * 该函数是一个包装函数，用于处理CAN桥接更新事件中的功率限制。
     * 它将接收到的扩展ID和数据传递给Chassis类的CanBridgeUpdateEventPowerLimit方法。
     *
     * @param ext_id CAN桥接的扩展ID
     * @param data CAN桥接的数据
     * @param args 指向Chassis类的指针
     */
    void Chassis::CanBridgeUpdateEventPowerLimitWrapper(communication::can_bridge_ext_id_t ext_id,
                                                        communication::can_bridge_data_t data,
                                                        void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventPowerLimit(ext_id, data);
    }

    /**
     * @brief 包装函数，用于处理CAN桥接更新事件中的当前功率
     *
     * 这是一个包装函数，用于处理CAN桥接更新事件中的当前功率。
     * 它将接收到的扩展ID和数据传递给Chassis类的CanBridgeUpdateEventCurrentPower方法。
     *
     * @param ext_id CAN桥接的扩展ID
     * @param data CAN桥接的数据
     * @param args 指向Chassis类的指针
     */
    void Chassis::CanBridgeUpdateEventCurrentPowerWrapper(communication::can_bridge_ext_id_t ext_id,
                                                          communication::can_bridge_data_t data,
                                                          void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventCurrentPower(ext_id, data);
    }

    /**
     * @brief 更新底盘的XY坐标事件
     *
     * 当通过CAN桥接接收到XY坐标更新事件时，此函数会被调用以更新底盘的X和Y速度。
     *
     * @param ext_id 包含CAN消息扩展ID的结构体
     * @param data 包含CAN消息数据的结构体
     *
     * 如果can_bridge_tx_id_不为0x00，则检查接收到的扩展ID（ext_id.data.tx_id）是否与预设的发送ID（can_bridge_tx_id_）匹配。
     * 如果不匹配，则函数直接返回，不执行任何操作。
     *
     * 如果匹配，则从接收到的数据中提取X速度（data.data_two_float.data[0]）和Y速度（data.data_two_float.data[1]），
     * 并分别更新底盘的X速度（can_bridge_vx_）和Y速度（can_bridge_vy_）。
     */
    void Chassis::CanBridgeUpdateEventXY(communication::can_bridge_ext_id_t ext_id,
                                         communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        can_bridge_vx_ = data.data_two_float.data[0];
        can_bridge_vy_ = data.data_two_float.data[1];
    }

    /**
     * @brief 更新底盘转向事件
     *
     * 该函数用于处理通过CAN桥接接收到的转向事件更新。
     *
     * @param ext_id 包含CAN消息扩展ID的结构体
     * @param data 包含CAN消息数据的结构体
     *
     * 函数首先检查接收到的扩展ID是否与预期的发送ID匹配。
     * 如果不匹配，则直接返回。如果匹配，函数将提取转向速度（can_bridge_vt_）和启用状态（is_enable）。
     *
     * 如果is_enable大于0.1且底盘当前未启用，则调用Enable()函数启用底盘。
     * 如果is_enable小于0.1且底盘当前已启用，则调用Disable()函数禁用底盘。
     *
     * 最后，根据底盘的启用状态，调用SetSpeed()函数设置底盘的速度。
     * 如果底盘启用，则设置速度为(can_bridge_vx_, can_bridge_vy_, can_bridge_vt_)；
     * 如果底盘未启用，则将速度设置为(0, 0, 0)。
     */
    void Chassis::CanBridgeUpdateEventTurn(communication::can_bridge_ext_id_t ext_id,
                                           communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        can_bridge_vt_ = data.data_two_float.data[1];
        float is_enable = data.data_two_float.data[0];
        if (is_enable > 0.1f && !chassis_enable_) {
            Enable();
        } else if (is_enable < 0.1f && chassis_enable_) {
            Disable();
        }
        if (chassis_enable_) {
            SetSpeed(can_bridge_vx_, can_bridge_vy_, can_bridge_vt_);
        } else {
            SetSpeed(0, 0, 0);
        }
    }

    /**
     * @brief 更新底盘功率限制事件
     *
     * 该函数用于处理通过CAN桥接接收到的底盘功率限制事件更新。
     *
     * @param ext_id 包含CAN消息扩展ID的结构体
     * @param data 包含CAN消息数据的结构体
     *
     * 如果can_bridge_tx_id_不为0x00，并且与ext_id中的tx_id不匹配，则函数直接返回。
     *
     * 函数首先检查功率限制是否启用。如果data中的第一个字节（is_enable）为true，并且当前功率限制未启用，
     * 则注册一个预输出回调函数UpdatePowerLimitWrapper，并设置power_limit_on_为true。
     * 如果is_enable为false且当前功率限制已启用，则注销预输出回调函数，并将power_limit_on_设置为false。
     *
     * 接下来，检查是否启用超级电容。如果data中的第二个字节（enable_supercap）为true，并且底盘具有超级电容，
     * 则将super_capacitor_enable_设置为true；否则，将其设置为false。
     *
     * 最后，更新功率限制信息。power_limit_info_.power_limit设置为data中的第二个浮点数（功率限制值），
     * power_limit_info_.WARNING_power设置为功率限制值的90%。如果底盘具有超级电容，则设置超级电容的总功率，
     * 该值至少为30.0f，最大为功率限制值减去25.0f。
     */
    void Chassis::CanBridgeUpdateEventPowerLimit(communication::can_bridge_ext_id_t ext_id,
                                                 communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        bool is_enable = data.data_eight_uint8.data[0];
        if (is_enable) {
            if (!power_limit_on_) {
                driver::MotorCANBase::RegisterPreOutputCallback(UpdatePowerLimitWrapper, this);
            }
            power_limit_on_ = true;
        } else {
            if (power_limit_on_) {
                driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); },
                                                                nullptr);
            }
            power_limit_on_ = false;
        }
        bool enable_supercap = data.data_eight_uint8.data[1];
        if (enable_supercap && has_super_capacitor_) {
            super_capacitor_enable_ = true;
        } else {
            super_capacitor_enable_ = false;
        }
        power_limit_info_.power_limit = data.data_two_float.data[1];
        power_limit_info_.WARNING_power = data.data_two_float.data[1] * 0.9f;
        if (has_super_capacitor_) {
            super_capacitor_->SetPowerTotal(max(power_limit_info_.power_limit - 25.0f, 30.0f));
        }
    }
    void Chassis::CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        current_chassis_power_ = data.data_two_float.data[0];
        current_chassis_power_buffer_ = data.data_two_float.data[1];
    }
    void Chassis::CanBridgeSetTxId(uint8_t tx_id) {
        can_bridge_tx_id_ = tx_id;
    }
    void Chassis::UpdatePowerLimitWrapper(void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->UpdatePowerLimit();
    }
    void Chassis::UpdatePowerLimit() {
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                float input[FourWheel::motor_num];
                float output[FourWheel::motor_num];
                for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                    input[i] = motors_[i]->GetOutput();
                if ((!super_capacitor_enable_) || (!super_capacitor_->IsOnline())) {
                    power_limit_->Output(power_limit_on_, power_limit_info_, current_chassis_power_,
                                         current_chassis_power_buffer_, input, output);
                } else {
                    power_limit_t power_limit_info = power_limit_info_;
                    float supercap_percent = super_capacitor_->GetPercentage();
                    if (supercap_percent < 0.25f) {
                        power_limit_info.power_limit =
                            (0.9f + supercap_percent) * power_limit_info.power_limit;
                        power_limit_info.WARNING_power = power_limit_info.power_limit * 0.9f;
                        power_limit_->Output(power_limit_on_, power_limit_info,
                                             current_chassis_power_, current_chassis_power_buffer_,
                                             input, output);
                    } else {
                        for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                            output[i] = input[i];
                    }
                }

                for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                    motors_[i]->SetOutput((int16_t)output[i]);
                break;
            }
            default:
                break;
        }
    }
    void Chassis::Enable() {
        chassis_enable_ = true;
        if (has_super_capacitor_) {
            super_capacitor_->Enable();
        }
    }
    void Chassis::Disable() {
        chassis_enable_ = false;
        if (has_super_capacitor_) {
            super_capacitor_->Disable();
        }
    }

    void Chassis::SetMaxMotorSpeed(float max_speed) {
        max_motor_speed_ = max_speed;
    }

    float Chassis::Chassis_DeBug_ret_velocity(){
        return can_bridge_vx_, can_bridge_vy_, can_bridge_vt_;
    }

    RMCM_Speed Chassis::Chassis_DeBug_Motor_Speed(){
        RMCM_Speed RMCM;
        RMCM.motor_fl = speeds_[FourWheel::front_left];
        RMCM.motor_fr = speeds_[FourWheel::front_right];
        RMCM.motor_bl = speeds_[FourWheel::back_left];
        RMCM.motor_br = speeds_[FourWheel::back_right];
        return RMCM;
    }

    void Chassis::Chassis_DeBug_model(bool newline){
        print("Chassis_Enable_Model: %s\r\n", chassis_enable_ ? "Enable" : "Disable");
        print("Speed: VX: %.2f, VY: %.2f, VT: %.2f\r\n", Chassis_DeBug_ret_velocity());
        auto[motor_fl, motor_fr, motor_bl, motor_br] = Chassis_DeBug_Motor_Speed();
        print("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\r\n", motor_fl, motor_fr, motor_bl, motor_br);
        if(newline){print("\r\n");}
    }

    /*
     *------------------------------------------- 在此分割 -------------------------------------------
     *                                    以下为底盘CAN总线控制发送部分
     *----------------------------------------------------------------------------------------------
     * */

    ChassisCanBridgeSender::ChassisCanBridgeSender(communication::CanBridge* can_bridge,
                                                   uint8_t rx_id)
        : can_bridge_(can_bridge), device_rx_id_(rx_id) {
        rx_id_.data.rx_id = rx_id;
        rx_id_.data.type = communication::CAN_BRIDGE_TYPE_TWO_FLOAT;
    }
    void ChassisCanBridgeSender::SetChassisRegId(uint8_t xy_reg_id, uint8_t turn_on_reg_id,
                                                 uint8_t power_limit_reg_id,
                                                 uint8_t current_power_reg_id) {
        chassis_xy_reg_id_ = xy_reg_id;
        chassis_turn_on_reg_id_ = turn_on_reg_id;
        chassis_power_limit_reg_id_ = power_limit_reg_id;
        chassis_current_power_reg_id_ = current_power_reg_id;
    }
    void ChassisCanBridgeSender::Enable() {
        if (!chassis_enable_) {
            chassis_enable_ = true;
            data_.data_two_float.data[0] = 1.0f;
            data_.data_two_float.data[1] = 0.0f;
            rx_id_.data.reg = chassis_turn_on_reg_id_;
            can_bridge_->Send(rx_id_, data_);
        }
    }
    void ChassisCanBridgeSender::Disable() {
        if (chassis_enable_) {
            chassis_enable_ = false;
        }
        data_.data_two_float.data[0] = 0;
        data_.data_two_float.data[1] = 0;
        rx_id_.data.reg = chassis_turn_on_reg_id_;
        can_bridge_->Send(rx_id_, data_);
    }
    void ChassisCanBridgeSender::SetSpeed(const float x_speed, const float y_speed,
                                          const float turn_speed) {
        if (chassis_enable_) {
            chassis_X_speed = x_speed;
            chassis_Y_speed = y_speed;
            data_.data_two_float.data[0] = x_speed;
            data_.data_two_float.data[1] = y_speed;
            rx_id_.data.reg = chassis_xy_reg_id_;
            can_bridge_->Send(rx_id_, data_);
            chassis_turn_speed = turn_speed;
            data_.data_two_float.data[0] = 1.0f;
            data_.data_two_float.data[1] = turn_speed;
            rx_id_.data.reg = chassis_turn_on_reg_id_;
            can_bridge_->Send(rx_id_, data_);
        } else {
            data_.data_two_float.data[0] = 0.0f;
            data_.data_two_float.data[1] = 0.0f;
            can_bridge_->Send(rx_id_, data_);
        }
    }
    void ChassisCanBridgeSender::SetPower(bool power_limit_on, float power_limit,
                                          float chassis_power, float chassis_power_buffer,
                                          bool enable_supercap, bool force_update) {
        UNUSED(force_update);
        if (chassis_enable_) {
            {
                chassis_power_limit_on_ = power_limit_on;
                chassis_power_limit_ = power_limit;
                data_.data_eight_uint8.data[0] = power_limit_on;
                data_.data_eight_uint8.data[1] = enable_supercap;
                data_.data_two_float.data[1] = power_limit;
                rx_id_.data.reg = chassis_power_limit_reg_id_;
                can_bridge_->Send(rx_id_, data_);
            }
            {
                data_.data_two_float.data[0] = chassis_power;
                data_.data_two_float.data[1] = chassis_power_buffer;
                rx_id_.data.reg = chassis_current_power_reg_id_;
                can_bridge_->Send(rx_id_, data_);
            }
        }

    }

    void ChassisCanBridgeSender::debug_chassis_can_model(bool ret_print, bool newline){

        if(ret_print == true){
            print("Chassis_Enable_Model: %s\r\n", chassis_enable_ ? "Enable" : "Disable");
            print("Chiassis_DeBug_DBUS CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\n", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
            print("VX: %.2f, VY: %.2f, VT: %.2f\r\n", chassis_X_speed, chassis_Y_speed, chassis_turn_speed);
            if(newline){print("\r\n");}
        }
    }

    void ChassisCanBridgeSender::debug_chassis_RCCH(uint8_t debug_reg_id){

        data_.data_four_int16.data[0] = dbus->ch0;
        data_.data_four_int16.data[1] = dbus->ch1;
        data_.data_four_int16.data[2] = dbus->ch2;
        data_.data_four_int16.data[3] = dbus->ch3;

        rx_id_.data.type = communication::CAN_BRIDGE_TYPE_FOUR_INT16;
        rx_id_.data.reg = debug_reg_id;
        can_bridge_->Send(rx_id_, data_);
    }

} /* namespace control */
