project(DGStandard_chassis_main ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/buzzer_task.cpp
        INCLUDES include)

uicrm_add_arm_executable(${PROJECT_NAME}_typeA
        TARGET DJI_Board_TypeA_general
        SOURCES
        src/main.cpp
        src/buzzer_task.cpp
        INCLUDES include)