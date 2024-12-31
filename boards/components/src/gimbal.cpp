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

#include "gimbal.h"

#include "MotorCanBase.h"
#include "utils.h"

namespace control {

    Gimbal::Gimbal(gimbal_t gimbal) {
        // acquired from user
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        pitch_angle_ = data_.pitch_offset_;
        yaw_angle_ = data_.yaw_offset_;

        pitch_lower_limit_ = wrap<float>(data_.pitch_offset_ - data_.pitch_max_, 0, 2 * PI);
        pitch_upper_limit_ = wrap<float>(data_.pitch_offset_ + data_.pitch_max_, 0, 2 * PI);
        yaw_lower_limit_ = wrap<float>(data_.yaw_offset_ - data_.yaw_max_, 0, 2 * PI);
        yaw_upper_limit_ = wrap<float>(data_.yaw_offset_ + data_.yaw_max_, 0, 2 * PI);
    }

    Gimbal::~Gimbal() {
    }

    gimbal_data_t* Gimbal::GetData() {
        return &data_;
    }

    void Gimbal::Update() {
        pitch_angle_ = clip<float>(pitch_angle_, pitch_lower_limit_, pitch_upper_limit_);
        pitch_motor_->SetTarget(pitch_angle_);

        //        float pt_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
        //        pt_diff = wrap<float>(pt_diff, -PI, PI);
        //
        //        if (abs(pt_diff) < data_.pitch_eposition) {
        //            pt_diff = 0;
        //        }
        //        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
        //
        //        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
        //        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

        if (!data_.yaw_circle_) {
            yaw_angle_ = clip<float>(yaw_angle_, yaw_lower_limit_, yaw_upper_limit_);
        }
        yaw_motor_->SetTarget(yaw_angle_);

        //        float yt_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
        //        yt_diff = wrap<float>(yt_diff, -PI, PI);
        //
        //        if (abs(yt_diff) < data_.yaw_eposition) {
        //            yt_diff = 0;
        //        }
        //
        //        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
        //        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
        //        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);
        //
        //        pitch_motor_->SetOutput(po_out);
        //        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::UpdateIMU(float pitch, float yaw) {
        float pt_diff = pitch_angle_ - data_.pitch_offset_ - pitch;
        float actual_pitch_angle = pitch_motor_->GetTheta();
        float new_pitch_diff = wrapping_clip<float>(
            pt_diff + actual_pitch_angle, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        new_pitch_diff = new_pitch_diff - actual_pitch_angle;
        if (pt_diff != new_pitch_diff) {
            pitch_angle_ = wrap<float>(pitch_angle_ + new_pitch_diff - pt_diff, 0, 2 * PI);
        }
        pt_diff = wrap<float>(new_pitch_diff, -PI, PI);

        if (abs(pt_diff) < data_.pitch_eposition) {
            pt_diff = 0;
        }
        pitch_motor_->SetTarget(pt_diff + actual_pitch_angle);

        //        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);
        //        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
        //        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

        float yt_diff = yaw_angle_ - data_.yaw_offset_ - yaw;
        float actual_yaw_angle = yaw_motor_->GetTheta();
        if (!data_.yaw_circle_) {
            float new_yaw_diff = wrapping_clip<float>(yt_diff + actual_yaw_angle, yaw_lower_limit_,
                                                      yaw_upper_limit_, 0, 2 * PI);
            new_yaw_diff = new_yaw_diff - actual_yaw_angle;
            if (yt_diff != new_yaw_diff) {
                yaw_angle_ = wrap<float>(yaw_angle_ + new_yaw_diff - yt_diff, 0, 2 * PI);
            }
            yt_diff = wrap<float>(new_yaw_diff, -PI, PI);
        } else {
            yt_diff = wrap<float>(yt_diff, -PI, PI);
        }

        if (abs(yt_diff) < data_.yaw_eposition) {
            yt_diff = 0;
        }

        yaw_motor_->SetTarget(yt_diff + actual_yaw_angle);

        //        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
        //        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
        //        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);
        //        pitch_motor_->SetOutput(po_out);
        //        yaw_motor_->SetOutput(yo_out);
    }

    /**
     * @brief 设置云台的目标绝对俯仰角和偏航角
     *
     * 根据给定的绝对俯仰角（abs_pitch）和偏航角（abs_yaw），设置云台的目标俯仰角和偏航角。
     * 如果数据结构中指定了俯仰角或偏航角是反向的，则对输入角度取反。
     * 然后，将输入角度限制在指定的最大和最小范围内，并根据偏移量进行调整。
     * 最后，将调整后的角度包装在指定的上下限内，并考虑圆周偏航模式。
     *
     * @param abs_pitch 绝对俯仰角，单位为弧度
     * @param abs_yaw 绝对偏航角，单位为弧度
     */
    void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.yaw_inverted)
            abs_yaw = -abs_yaw;
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        pitch_angle_ = wrapping_clip<float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_,
                                            pitch_upper_limit_, 0, 2 * PI);
        if (data_.yaw_circle_) {
            yaw_angle_ = wrap<float>(clipped_yaw + data_.yaw_offset_, 0, 2 * PI);
        } else {
            yaw_angle_ = wrapping_clip<float>(clipped_yaw + data_.yaw_offset_, yaw_lower_limit_,
                                              yaw_upper_limit_, 0, 2 * PI);
        }
    }

    /**
     * @brief 设置云台的目标相对俯仰角和偏航角
     *
     * 根据给定的相对俯仰角（rel_pitch）和偏航角（rel_yaw），更新云台当前的目标俯仰角和偏航角。
     * 如果数据结构中指定了俯仰角或偏航角是反向的，则对输入角度取反。
     * 然后，将当前角度与输入角度相加，并将结果包装在0到2π的范围内。
     *
     * @param rel_pitch 相对俯仰角，单位为弧度
     * @param rel_yaw 相对偏航角，单位为弧度
     */
    void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        if (data_.yaw_inverted)
            rel_yaw = -rel_yaw;
        pitch_angle_ = wrap<float>(pitch_angle_ + rel_pitch, 0, 2 * PI);
        yaw_angle_ = wrap<float>(yaw_angle_ + rel_yaw, 0, 2 * PI);
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.yaw_offset_ = wrap<float>(yaw_offset + data_.yaw_offset_, 0, 2 * PI);
    }
    void Gimbal::TargetReal(float new_pitch, float new_yaw) {
        if (data_.pitch_inverted)
            new_pitch = -new_pitch;
        if (data_.yaw_inverted)
            new_yaw = -new_yaw;
        pitch_angle_ = wrap<float>(pitch_motor_->GetTheta() + new_pitch, 0, 2 * PI);
        yaw_angle_ = wrap<float>(yaw_motor_->GetTheta() + new_yaw, 0, 2 * PI);
    }

}  // namespace control
