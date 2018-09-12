################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./CLARKE_F.obj" \
"./F2837xS_Adc.obj" \
"./F2837xS_CodeStartBranch.obj" \
"./F2837xS_DefaultISR.obj" \
"./F2837xS_EPwm.obj" \
"./F2837xS_GlobalVariableDefs.obj" \
"./F2837xS_Gpio.obj" \
"./F2837xS_PieCtrl.obj" \
"./F2837xS_PieVect.obj" \
"./F2837xS_SysCtrl.obj" \
"./F2837xS_usDelay.obj" \
"./PARK_F.obj" \
"./blinky_cpu01_main.obj" \
"./configuration_ADC_ePWM.obj" \
"./iPARK_F.obj" \
"./utility Src Files/DLOG_4CH_F.obj" \
"../2837xS_FLASH_SGEN_lnk_cpu1.cmd" \
"../F2837xS_Headers_nonBIOS.cmd" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/controlSUITE/libs/dsp/SGEN/v101/lib/C28x_SGEN_Lib_fpu32.lib" \
-l"C:/ti/controlSUITE/libs/math/IQmath/v160/lib/IQmath.lib" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utility\ Src\ Files/subdir_vars.mk
-include subdir_rules.mk
-include utility\ Src\ Files/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
blinky_cpu01.out \

EXE_OUTPUTS__QUOTED += \
"blinky_cpu01.out" \

BIN_OUTPUTS += \
blinky_cpu01.hex \

BIN_OUTPUTS__QUOTED += \
"blinky_cpu01.hex" \


# All Target
all: blinky_cpu01.out

# Tool invocations
blinky_cpu01.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --cla_support=cla1 --tmu_support=tmu0 --vcu_support=vcu2 --opt_for_speed=5 --fp_mode=relaxed -g --define=CPU1 --display_error_number --diag_suppress=10063 --diag_warning=225 -z -m"blinky_cpu01.map" --stack_size=0x100 --warn_sections -i"C:/ti/lib" -i"C:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/lib" -i"C:/ti/controlSUITE/libs/dsp/SGEN/v101/lib" -i"C:/ti/include" -i"C:/ti/controlSUITE/libs/math/IQmath/v160/lib" --reread_libs --display_error_number --xml_link_info="blinky_cpu01_linkInfo.xml" --entry_point=code_start --rom_model -o "blinky_cpu01.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

blinky_cpu01.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/bin/hex2000"  -o "blinky_cpu01.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "CLARKE_F.obj" "F2837xS_Adc.obj" "F2837xS_CodeStartBranch.obj" "F2837xS_DefaultISR.obj" "F2837xS_EPwm.obj" "F2837xS_GlobalVariableDefs.obj" "F2837xS_Gpio.obj" "F2837xS_PieCtrl.obj" "F2837xS_PieVect.obj" "F2837xS_SysCtrl.obj" "F2837xS_usDelay.obj" "PARK_F.obj" "blinky_cpu01_main.obj" "configuration_ADC_ePWM.obj" "iPARK_F.obj" "utility Src Files\DLOG_4CH_F.obj" 
	-$(RM) "CLARKE_F.d" "F2837xS_Adc.d" "F2837xS_DefaultISR.d" "F2837xS_EPwm.d" "F2837xS_GlobalVariableDefs.d" "F2837xS_Gpio.d" "F2837xS_PieCtrl.d" "F2837xS_PieVect.d" "F2837xS_SysCtrl.d" "PARK_F.d" "blinky_cpu01_main.d" "configuration_ADC_ePWM.d" "iPARK_F.d" "utility Src Files\DLOG_4CH_F.d" 
	-$(RM) "F2837xS_CodeStartBranch.d" "F2837xS_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

