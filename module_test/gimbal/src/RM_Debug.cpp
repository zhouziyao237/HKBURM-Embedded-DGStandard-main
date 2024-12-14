//
// Created by Administrator on 2024/11/22.
//
#include "bsp_print.h"
#include "RM_Debug.h"

#define Debug_true 1
#define Debug_false 0

communication::DebugModel* CAR_DEBUG = nullptr;

void init_rm_debug() {
    print("RM_INIT>>\n");
    CAR_DEBUG = new communication::DebugModel();
    CAR_DEBUG->RM_Debug_mode.debug_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.gimbal_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.chassis_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.shooter_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.supercapacitor_debug_mode = Debug_false;
    CAR_DEBUG->RM_Debug_mode.vision_debug_mode = Debug_false;
}