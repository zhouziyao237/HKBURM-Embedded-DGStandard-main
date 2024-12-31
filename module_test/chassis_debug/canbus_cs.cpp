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

#include "main.h"

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

remote::DBUS* dbus = nullptr;

control::Chassis* chassis = nullptr;
control::ChassisCanBridgeSender *CanBridgeSender = nullptr;

//communication::CanBridge* chassis_can = nullptr;
communication::CanBridge* gimbal_can = nullptr;

void RM_RTOS_Init(void) {
   HAL_Delay(200);
   print_use_uart(&huart8);


   can2 = new bsp::CAN(&hcan2, true);

   gimbal_can = new communication::CanBridge(can2, 0x51);
   CanBridgeSender = new control::ChassisCanBridgeSender(gimbal_can, 0x52);
   CanBridgeSender->SetChassisRegId(0x70, 0x71, 0x72, 0x73);

//   can1 = new bsp::CAN(&hcan1, true);
//   chassis_can = new communication::CanBridge(can1, 0x52);
//   chassis_can->RegisterRxCallback(0x70, chassis_main->CanbusresdebugWrapper, nullptr);

   print("CAN init finish\r\n");

   dbus = new remote::DBUS(&huart1);
   HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
   UNUSED(args);

   osDelay(500);  // DBUS initialization needs time


   while (true) {
       CanBridgeSender->Enable();
       print("send\n");
       osDelay(100);
   }
}
