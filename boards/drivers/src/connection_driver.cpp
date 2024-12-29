/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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

#include "connection_driver.h"

#include "bsp_os.h"
#include "bsp_print.h"

namespace driver {
    /**
     * @brief 检查连接是否在线
     *
     * 此成员函数用于判断当前连接驱动是否在线。
     *
     * @return 如果连接驱动在线，则返回 true；否则返回 false。
     *         如果 last_uptime_ 为 0，表示没有记录最后上线时间，直接返回 false。
     *         否则，计算当前时间与 last_uptime_ 的差值，如果该差值小于 online_threshold_，
     *         则认为连接驱动仍然在线，返回 true；否则返回 false。
     */
    bool ConnectionDriver::IsOnline() const {
        if (last_uptime_ == 0) {
            return false;
        }
        return bsp::GetHighresTickMilliSec() - last_uptime_ < online_threshold_;
    }
    void ConnectionDriver::Heartbeat() {
        last_uptime_ = bsp::GetHighresTickMilliSec();
    }
    uint32_t ConnectionDriver::GetLastUptime() {
        return last_uptime_;
    }

    uint32_t ConnectionDriver::GetSetThreshold() {
        return online_threshold_;
    }

    void ConnectionDriver::SetThreshold(uint32_t threshold) {
        online_threshold_ = threshold;
    }
}  // namespace driver
