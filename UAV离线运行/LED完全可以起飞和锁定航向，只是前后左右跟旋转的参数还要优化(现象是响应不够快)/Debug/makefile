################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv5/tools/compiler/c2000_6.2.0

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./pwm.obj" \
"./nrf24l01.obj" \
"./hardware_I2C.obj" \
"./fast_math.obj" \
"./control.obj" \
"./capture.obj" \
"./MPU6050.obj" \
"./LED.obj" \
"./IMU.obj" \
"./I2C.obj" \
"./Data_Transfer.obj" \
"./DSP2833x_usDelay.obj" \
"./DSP2833x_Xintf.obj" \
"./DSP2833x_SysCtrl.obj" \
"./DSP2833x_Spi.obj" \
"./DSP2833x_Sci.obj" \
"./DSP2833x_PieVect.obj" \
"./DSP2833x_PieCtrl.obj" \
"./DSP2833x_I2C.obj" \
"./DSP2833x_GlobalVariableDefs.obj" \
"./DSP2833x_EPwm.obj" \
"./DSP2833x_ECap.obj" \
"./DSP2833x_DefaultIsr.obj" \
"./DSP2833x_CpuTimers.obj" \
"./DSP2833x_CodeStartBranch.obj" \
"./DSP2833x_ADC_cal.obj" \
"../CMD/DSP2833x_Headers_nonBIOS.cmd" \
"../CMD/F28335.cmd" \
-l"rts2800_fpu32.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include SRC/subdir_vars.mk
-include CMD/subdir_vars.mk
-include SRC/subdir_rules.mk
-include CMD/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: LED.out

# Tool invocations
LED.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/bin/cl2000" -v28 -ml --float_support=fpu32 -g --define="_DEBUG" --define="LARGE_MODEL" --diag_warning=225 -z -m"LED.map" --heap_size=1000 --stack_size=1000 --warn_sections -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/lib" -i"C:/ti/ccsv5/tools/compiler/c2000_6.2.0/include" -i"F:/workspace/LED" --reread_libs --xml_link_info="LED_linkInfo.xml" --rom_model -o "LED.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "LED.out"
	-$(RM) "SRC\DSP2833x_CpuTimers.pp" "SRC\DSP2833x_DefaultIsr.pp" "SRC\DSP2833x_ECap.pp" "SRC\DSP2833x_EPwm.pp" "SRC\DSP2833x_GlobalVariableDefs.pp" "SRC\DSP2833x_I2C.pp" "SRC\DSP2833x_PieCtrl.pp" "SRC\DSP2833x_PieVect.pp" "SRC\DSP2833x_Sci.pp" "SRC\DSP2833x_Spi.pp" "SRC\DSP2833x_SysCtrl.pp" "SRC\DSP2833x_Xintf.pp" "SRC\Data_Transfer.pp" "SRC\I2C.pp" "SRC\IMU.pp" "SRC\LED.pp" "SRC\MPU6050.pp" "SRC\capture.pp" "SRC\control.pp" "SRC\fast_math.pp" "SRC\hardware_I2C.pp" "SRC\nrf24l01.pp" "SRC\pwm.pp" 
	-$(RM) "DSP2833x_ADC_cal.obj" "DSP2833x_CodeStartBranch.obj" "DSP2833x_CpuTimers.obj" "DSP2833x_DefaultIsr.obj" "DSP2833x_ECap.obj" "DSP2833x_EPwm.obj" "DSP2833x_GlobalVariableDefs.obj" "DSP2833x_I2C.obj" "DSP2833x_PieCtrl.obj" "DSP2833x_PieVect.obj" "DSP2833x_Sci.obj" "DSP2833x_Spi.obj" "DSP2833x_SysCtrl.obj" "DSP2833x_Xintf.obj" "DSP2833x_usDelay.obj" "Data_Transfer.obj" "I2C.obj" "IMU.obj" "LED.obj" "MPU6050.obj" "capture.obj" "control.obj" "fast_math.obj" "hardware_I2C.obj" "nrf24l01.obj" "pwm.obj" 
	-$(RM) "SRC\DSP2833x_ADC_cal.pp" "SRC\DSP2833x_CodeStartBranch.pp" "SRC\DSP2833x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

