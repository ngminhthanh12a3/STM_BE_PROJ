################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := SERIAL_COMMUNICATION
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SERIAL_COMMUNICATION.elf \

MAP_FILES += \
SERIAL_COMMUNICATION.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SERIAL_COMMUNICATION.list \

OBJCOPY_HEX += \
SERIAL_COMMUNICATION.hex \

OBJCOPY_BIN += \
SERIAL_COMMUNICATION.bin \


# All Target
all: main-build

# Main-build Target
main-build: SERIAL_COMMUNICATION.elf secondary-outputs

# Tool invocations
SERIAL_COMMUNICATION.elf SERIAL_COMMUNICATION.map: $(OBJS) $(USER_OBJS) C:\Users\ADMIN\Documents\_Nam4\HK1\DTYSinh\project\SERIAL_COMMUNICATION\STM32F103RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "SERIAL_COMMUNICATION.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\ADMIN\Documents\_Nam4\HK1\DTYSinh\project\SERIAL_COMMUNICATION\STM32F103RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SERIAL_COMMUNICATION.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SERIAL_COMMUNICATION.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SERIAL_COMMUNICATION.list"
	@echo 'Finished building: $@'
	@echo ' '

SERIAL_COMMUNICATION.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "SERIAL_COMMUNICATION.hex"
	@echo 'Finished building: $@'
	@echo ' '

SERIAL_COMMUNICATION.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SERIAL_COMMUNICATION.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) SERIAL_COMMUNICATION.bin SERIAL_COMMUNICATION.elf SERIAL_COMMUNICATION.hex SERIAL_COMMUNICATION.list SERIAL_COMMUNICATION.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
