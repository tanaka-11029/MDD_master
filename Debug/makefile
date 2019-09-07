################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include ros_lib_kinetic/BufferedSerial/Buffer/subdir.mk
-include ros_lib_kinetic/BufferedSerial/subdir.mk
-include ros_lib_kinetic/subdir.mk
-include mbed/TARGET_NUCLEO_L432KC/TOOLCHAIN_GCC_ARM/subdir.mk
-include library/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: pre-build main-build

# Main-build Target
main-build: MDD_Master-debug.elf

# Tool invocations
MDD_Master-debug.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU G++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -L"/home/tanaka/workspace/MDD_Master/mbed/TARGET_NUCLEO_L432KC/TOOLCHAIN_GCC_ARM" -Wl,--wrap=_memalign_r -Wl,--wrap=exit -Wl,--wrap=atexit -DMBED_DEBUG -DMBED_TRAP_ERRORS_ENABLED=1 -Wl,-n -Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -Wl,--end-group -T"/home/tanaka/workspace/MDD_Master/Debug/linker-script-debug.ld" -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -o "MDD_Master-debug.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

pre-build:
	-@echo 'Creating makefile.defs:'
	-echo "export PREPROC_CMD = /home/tanaka/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.arm-none.linux64_1.17.0.201812190825/tools/compiler/bin/arm-none-eabi-cpp -E -P" > /home/tanaka/workspace/MDD_Master/makefile.defs
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "MDD_Master-debug.elf" "MDD_Master.bin"
	arm-none-eabi-size -B "MDD_Master-debug.elf"
	make ldclean
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: main-build pre-build post-build

-include ../makefile.targets
