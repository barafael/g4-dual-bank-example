################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
g474-bankswap.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
g474-bankswap.list \

OBJCOPY_BIN += \
g474-bankswap.bin \


# All Target
all: g474-bankswap.elf secondary-outputs

# Tool invocations
g474-bankswap.elf: $(OBJS) $(USER_OBJS) /home/ra/STM32CubeIDE/workspace_1.3.0/g474-bankswap/STM32G474RETX_FLASH.ld
	arm-none-eabi-gcc -o "g474-bankswap.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/ra/STM32CubeIDE/workspace_1.3.0/g474-bankswap/STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="g474-bankswap.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

g474-bankswap.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "g474-bankswap.list"
	@echo 'Finished building: $@'
	@echo ' '

g474-bankswap.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "g474-bankswap.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
