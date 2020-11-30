################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/Components/lsm6dsl/subdir.mk
-include Core/Components/lps22hb/subdir.mk
-include Core/Components/lis3mdl/subdir.mk
-include Core/Components/hts221/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
B-L475E_Flash_Sample_Ctrl.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
B-L475E_Flash_Sample_Ctrl.list \

OBJCOPY_BIN += \
B-L475E_Flash_Sample_Ctrl.bin \


# All Target
all: B-L475E_Flash_Sample_Ctrl.elf secondary-outputs

# Tool invocations
B-L475E_Flash_Sample_Ctrl.elf: $(OBJS) $(USER_OBJS) C:\Users\holyc\Documents\EE\ Semester\ 9\ECSE\ 444\B-L475E_Flash_Sample_Ctrl\STM32L475VGTX_FLASH.ld
	arm-none-eabi-gcc -o "B-L475E_Flash_Sample_Ctrl.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\holyc\Documents\EE Semester 9\ECSE 444\B-L475E_Flash_Sample_Ctrl\STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="B-L475E_Flash_Sample_Ctrl.map" -Wl,--gc-sections -static -L"C:\Users\holyc\Documents\EE Semester 9\ECSE 444\B-L475E_Flash_Sample_Ctrl\Core\Lib" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

B-L475E_Flash_Sample_Ctrl.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "B-L475E_Flash_Sample_Ctrl.list"
	@echo 'Finished building: $@'
	@echo ' '

B-L475E_Flash_Sample_Ctrl.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "B-L475E_Flash_Sample_Ctrl.bin"
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
