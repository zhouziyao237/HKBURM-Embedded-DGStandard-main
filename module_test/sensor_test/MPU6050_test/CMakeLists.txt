project(MPU6050_test ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_MPU_6050_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/MPU6050.cpp
        INCLUDES include)
