################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
PROFILES/gap.obj: /Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/profiles/roles/gap.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/build_components.opt" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/factory_config.opt" --cmd_file="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/rom" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx/npi/stack" --include_path="/src/stack" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/profiles/roles" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/npi/src" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/osal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/saddr" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/rf_patches" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages" --include_path="/Applications/ti/xdctools_3_51_01_18_core/packages" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=GATT_NO_CLIENT --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=4 --define=OSAL_SNV=1 --define=POWER_SAVING --define=RF_SINGLEMODE --define=STACK_LIBRARY --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="PROFILES/$(basename $(<F)).d_raw" --obj_directory="PROFILES" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

PROFILES/gapbondmgr.obj: /Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/profiles/roles/gapbondmgr.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/build_components.opt" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/factory_config.opt" --cmd_file="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/rom" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx/npi/stack" --include_path="/src/stack" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/profiles/roles" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/npi/src" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/osal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/saddr" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/rf_patches" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages" --include_path="/Applications/ti/xdctools_3_51_01_18_core/packages" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=GATT_NO_CLIENT --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=4 --define=OSAL_SNV=1 --define=POWER_SAVING --define=RF_SINGLEMODE --define=STACK_LIBRARY --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="PROFILES/$(basename $(<F)).d_raw" --obj_directory="PROFILES" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

PROFILES/gattservapp_util.obj: /Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/host/gattservapp_util.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/build_components.opt" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/factory_config.opt" --cmd_file="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/rom" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx/npi/stack" --include_path="/src/stack" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/profiles/roles" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/npi/src" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/osal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/saddr" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/rf_patches" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages" --include_path="/Applications/ti/xdctools_3_51_01_18_core/packages" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=GATT_NO_CLIENT --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=4 --define=OSAL_SNV=1 --define=POWER_SAVING --define=RF_SINGLEMODE --define=STACK_LIBRARY --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="PROFILES/$(basename $(<F)).d_raw" --obj_directory="PROFILES" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

PROFILES/sm_ecc.obj: /Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/host/sm_ecc.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/build_components.opt" --cmd_file="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/config/factory_config.opt" --cmd_file="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="/Users/henorvell/workspace_v8/hid_gyromouse_cc2640r2_stack_library" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/rom" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/common/cc26xx/npi/stack" --include_path="/src/stack" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/profiles/roles" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/hal/src/target" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/icall/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/npi/src" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/osal/src/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv/cc26xx" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/nv" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack/services/src/saddr" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/rf_patches" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/devices/cc26x0r2/inc" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source" --include_path="/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages" --include_path="/Applications/ti/xdctools_3_51_01_18_core/packages" --include_path="/Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=GATT_NO_CLIENT --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=4 --define=OSAL_SNV=1 --define=POWER_SAVING --define=RF_SINGLEMODE --define=STACK_LIBRARY --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="PROFILES/$(basename $(<F)).d_raw" --obj_directory="PROFILES" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

