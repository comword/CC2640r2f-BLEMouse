# invoke SourceDir generated makefile for app_ble.pem3
app_ble.pem3: .libraries,app_ble.pem3
.libraries,app_ble.pem3: package/cfg/app_ble_pem3.xdl
	$(MAKE) -f /mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app/TOOLS/src/makefile.libs

clean::
	$(MAKE) -f /mnt/Data/Getong/CC2640r2f-BLEMouse/hid_gyromouse_cc2640r2_app/TOOLS/src/makefile.libs clean

