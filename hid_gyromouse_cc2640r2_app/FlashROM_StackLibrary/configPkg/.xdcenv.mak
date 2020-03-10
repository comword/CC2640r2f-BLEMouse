#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/kernel/tirtos/packages;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack
override XDCROOT = /mnt/NData/Softwares/ti/xdctools_3_50_07_20_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/kernel/tirtos/packages;/mnt/NData/Softwares/ti/simplelink_cc2640r2_sdk_2_20_00_49/source/ti/blestack;/mnt/NData/Softwares/ti/xdctools_3_50_07_20_core/packages;..
HOSTOS = Linux
endif
