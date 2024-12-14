project(example_chassis_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES main.cpp)


uicrm_add_arm_executable(${PROJECT_NAME}_chassis_sbus
        TARGET ${BOARD_NAME}
        SOURCES sbus.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_chassis_can_bridge
        TARGET ${BOARD_NAME}
        SOURCES can_bridge_chassis.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_chassis_can_bridge_sender
        TARGET ${BOARD_NAME}
        SOURCES can_bridge_chassis_sender.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_chassis_main
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)