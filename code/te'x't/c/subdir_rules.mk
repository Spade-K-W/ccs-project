################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
code/te'x't/c/%.o: ../code/te'x't/c/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/yuzhi/Desktop/nb666/workspace_ccstheia(4)/workspace_ccstheia/text" -I"C:/Users/yuzhi/Desktop/nb666/workspace_ccstheia(4)/workspace_ccstheia/text/Debug" -I"D:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI/mspm0_sdk_2_10_00_04/source" -I"C:/Users/yuzhi/Desktop/nb666/workspace_ccstheia(4)/workspace_ccstheia/text/code/te'x't/h" -I"C:/Users/yuzhi/Desktop/nb666/workspace_ccstheia(4)/workspace_ccstheia/text/code/inc" -I"C:/Users/yuzhi/Desktop/nb666/workspace_ccstheia(4)/workspace_ccstheia/text/code/TIMA1_PWM_RGB" -gdwarf-3 -Wall -MMD -MP -MF"code/te'x't/c/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


