################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
code/%.o: ../code/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/18600/Desktop/nb666/nb666/workspace_ccstheia(4)/mega_car_test/workspace_ccstheia/text" -I"C:/Users/18600/Desktop/nb666/nb666/workspace_ccstheia(4)/mega_car_test/workspace_ccstheia/text/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/18600/Desktop/nb666/nb666/workspace_ccstheia(4)/mega_car_test/workspace_ccstheia/text/code/te'x't/h" -I"C:/Users/18600/Desktop/nb666/nb666/workspace_ccstheia(4)/mega_car_test/workspace_ccstheia/text/code/inc" -I"C:/Users/18600/Desktop/nb666/nb666/workspace_ccstheia(4)/mega_car_test/workspace_ccstheia/text/code/TIMA1_PWM_RGB" -gdwarf-3 -Wall -MMD -MP -MF"code/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


