################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ros_lib_kinetic/duration.cpp \
../ros_lib_kinetic/time.cpp 

OBJS += \
./ros_lib_kinetic/duration.o \
./ros_lib_kinetic/time.o 

CPP_DEPS += \
./ros_lib_kinetic/duration.d \
./ros_lib_kinetic/time.d 


# Each subdirectory must supply rules for building sources it contributes
ros_lib_kinetic/%.o: ../ros_lib_kinetic/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 '-DDEVICE_CRC=1' '-D__MBED__=1' '-DDEVICE_I2CSLAVE=1' '-D__FPU_PRESENT=1' '-DDEVICE_PORTOUT=1' '-DDEVICE_PORTINOUT=1' -DTARGET_RTOS_M4_M7 '-DDEVICE_RTC=1' '-DDEVICE_SERIAL_ASYNCH=1' -D__CMSIS_RTOS -DTOOLCHAIN_GCC '-DDEVICE_CAN=1' -DTARGET_CORTEX_M '-DDEVICE_I2C_ASYNCH=1' -DTARGET_LIKE_CORTEX_M4 '-DDEVICE_ANALOGOUT=1' -DTARGET_M4 -DTARGET_STM32L4 '-DDEVICE_SPI_ASYNCH=1' '-DDEVICE_LPTICKER=1' '-DDEVICE_PWMOUT=1' -DTARGET_STM32L432xC -DTARGET_CORTEX '-DDEVICE_I2C=1' '-DTRANSACTION_QUEUE_SIZE_SPI=2' -D__CORTEX_M4 '-DDEVICE_STDIO_MESSAGES=1' -DTARGET_FAMILY_STM32 '-DMBED_BUILD_TIMESTAMP=1565576313.11' -DTARGET_FF_ARDUINO '-DDEVICE_PORTIN=1' -DTARGET_RELEASE '-DTARGET_NAME=NUCLEO_L432KC' -DTARGET_STM -DTARGET_STM32L432KC '-DDEVICE_SERIAL_FC=1' '-DDEVICE_USTICKER=1' '-DDEVICE_TRNG=1' -DTARGET_LIKE_MBED -D__MBED_CMSIS_RTOS_CM '-DDEVICE_SLEEP=1' -DTOOLCHAIN_GCC_ARM '-DDEVICE_SPI=1' '-DDEVICE_INTERRUPTIN=1' '-DDEVICE_SPISLAVE=1' '-DDEVICE_ANALOGIN=1' '-DDEVICE_SERIAL=1' '-DDEVICE_FLASH=1' -DTARGET_NUCLEO_L432KC -DARM_MATH_CM4 -DMBED_DEBUG '-DMBED_TRAP_ERRORS_ENABLED=1' -DMBED_DEBUG '-DMBED_TRAP_ERRORS_ENABLED=1' -DNDEBUG -DNDEBUG -I"/home/tanaka/workspace/MDD_Master" -I"..//usr/src/mbed-sdk" -I"/home/tanaka/workspace/MDD_Master/library" -I"/home/tanaka/workspace/MDD_Master/mbed" -I"/home/tanaka/workspace/MDD_Master/mbed/TARGET_NUCLEO_L432KC/TOOLCHAIN_GCC_ARM" -I"/home/tanaka/workspace/MDD_Master/mbed/drivers" -I"/home/tanaka/workspace/MDD_Master/mbed/hal" -I"/home/tanaka/workspace/MDD_Master/mbed/platform" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/BufferedSerial" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/BufferedSerial/Buffer" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/actionlib" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/actionlib_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/actionlib_tutorials" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/bond" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/control_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/diagnostic_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/dynamic_reconfigure" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/gazebo_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/geometry_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/laser_assembler" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/map_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/nav_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/nodelet" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/pcl_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/polled_camera" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/ros" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/roscpp" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/roscpp_tutorials" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/rosgraph_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/rospy_tutorials" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/rosserial_arduino" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/rosserial_mbed" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/rosserial_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/sensor_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/shape_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/smach_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/std_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/std_srvs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/stereo_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/tf" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/tf2_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/theora_image_transport" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/topic_tools" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/trajectory_msgs" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/turtle_actionlib" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/turtlesim" -I"/home/tanaka/workspace/MDD_Master/ros_lib_kinetic/visualization_msgs"  -include"/home/tanaka/workspace/MDD_Master/mbed_config.h" -O0 -funsigned-char -fno-delete-null-pointer-checks -fomit-frame-pointer -fmessage-length=0 -g3 -Wall -Wextra -Wvla -Wno-unused-parameter -Wno-missing-field-initializers -ffunction-sections -fdata-sections -c -fno-exceptions -fno-rtti -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


