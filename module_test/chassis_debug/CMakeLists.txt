project(DGStandard_chassis ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_model_test_TypeA_sbus
        TARGET DJI_Board_TypeA_general
        SOURCES sbus.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_model_test_TypeA_can_bridge
        TARGET DJI_Board_TypeA_general
        SOURCES can_bridge_chassis.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_model_test_TypeA_can_bridge_sender
        TARGET DJI_Board_TypeA_general
        SOURCES can_bridge_chassis_sender.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_model_test_TypeA_main
        TARGET DJI_Board_TypeA_general
        SOURCES main.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_model_test_TypeA_canbus_cs
        TARGET DJI_Board_TypeA_general
        SOURCES canbus_cs.cpp)