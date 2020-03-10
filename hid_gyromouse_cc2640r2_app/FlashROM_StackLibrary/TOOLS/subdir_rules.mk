################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-1744192865:
	@$(MAKE) --no-print-directory -Onone -f TOOLS/subdir_rules.mk build-1744192865-inproc

build-1744192865-inproc: ../TOOLS/app_ble.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"/mnt/NData/Softwares/ti/xdctools_3_50_07_20_core/xs" --xdcpath="/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/kernel/tirtos/packages;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/kernel/tirtos/packages;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640R2F -r release -c "/mnt/NData/Softwares/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS" --compileOptions "-mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path=\"/mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app\" --include_path=\"/mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app/Application\" --include_path=\"/mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app/Startup\" --include_path=\"/mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app/PROFILES\" --include_path=\"/mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app/Include\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/controller/cc26xx_r2/inc\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/inc\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/common/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/icall/inc\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/batt/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/dev_info\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/hid_dev/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/hid_dev_kbd\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/hid_dev_kbd/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/roles\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/roles/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/scan_param\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/profiles/scan_param/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/target\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/hal/src/inc\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/hal/src/target/_common\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/hal/src/target/_common/cc26xx\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/heapmgr\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/icall/src/inc\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/osal/src/inc\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/services/src/saddr\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack/services/src/sdata\" --include_path=\"/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/devices/cc26x0r2\" --include_path=\"/mnt/NData/Softwares/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include\" --define=BOARD_DISPLAY_USE_LCD=1 --define=BOARD_DISPLAY_USE_UART=1 --define=BOARD_DISPLAY_USE_UART_ANSI=0 --define=CC2640R2_MOUSEBOARD --define=CC26XX --define=Board_EXCLUDE_NVS_EXTERNAL_FLASH --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=xDisplay_DISABLE_ALL --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=ICALL_MAX_NUM_ENTITIES=7 --define=ICALL_MAX_NUM_TASKS=5 --define=ICALL_STACK0_ADDR --define=POWER_SAVING --define=RF_SINGLEMODE --define=STACK_LIBRARY --define=USE_ICALL --define=xdc_runtime_Assert_DISABLE_ALL --define=xdc_runtime_Log_DISABLE_ALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1744192865 ../TOOLS/app_ble.cfg
configPkg/compiler.opt: build-1744192865
configPkg/: build-1744192865


