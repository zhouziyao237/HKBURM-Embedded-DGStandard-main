project(sensor_test_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_MPU_6050
        TARGET ${BOARD_NAME}
        SOURCES mian.cpp)
