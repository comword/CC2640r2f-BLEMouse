#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack;/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source;/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages
override XDCROOT = /Applications/ti/xdctools_3_51_01_18_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source/ti/blestack;/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/source;/Users/henorvell/ti/simplelink_cc2640r2_sdk_2_30_00_28/kernel/tirtos/packages;/Applications/ti/xdctools_3_51_01_18_core/packages;..
HOSTOS = MacOS
endif
