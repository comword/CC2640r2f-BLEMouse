################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OSAL/osal.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_bufmgr.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal_bufmgr.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_bufmgr.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_cbtimer.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal_cbtimer.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_cbtimer.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_clock.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal_clock.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_clock.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_memory_icall.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal_memory_icall.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_memory_icall.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_pwrmgr.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal_pwrmgr.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_pwrmgr.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_snv_wrapper.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/mcu/cc26xx/osal_snv_wrapper.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_snv_wrapper.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

OSAL/osal_timers.obj: E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/common/osal_timers.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/build_components.opt" --cmd_file="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/config/factory_config.opt" --cmd_file="E:/ccs_work/ble5_hid_gyromouse_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="E:/ccs_work/ble5_hid_gyromouse_stack_library" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/controller/cc26xx_r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/rom" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/common/cc26xx/npi/stack" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/profiles/roles" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target/_common/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/hal/src/target" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/icall/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/npi/src" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/osal/src/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv/cc26xx" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/nv" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/ble5stack/services/src/saddr" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/rf_patches" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source/ti/devices/cc26x0r2/inc" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/source" --include_path="E:/ti/simplelink_cc2640r2_sdk_1_35_00_33/kernel/tirtos/packages" --include_path="E:/ti/xdctools_3_50_01_12_core/packages" --include_path="E:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DEVICE_FAMILY=cc26x0r2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=6 --define=OSAL_SNV=1 --define=POWER_SAVING --define=STACK_LIBRARY --define=USE_CORE_SDK --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="OSAL/osal_timers.d" --obj_directory="OSAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


