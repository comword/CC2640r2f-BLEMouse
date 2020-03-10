deps_config := \
	/mnt/NData/esp32/esp-idf/components/app_trace/Kconfig \
	/mnt/NData/esp32/esp-idf/components/aws_iot/Kconfig \
	/mnt/NData/esp32/esp-idf/components/bt/Kconfig \
	/mnt/NData/esp32/esp-idf/components/driver/Kconfig \
	/mnt/NData/esp32/esp-idf/components/esp32/Kconfig \
	/mnt/NData/esp32/esp-idf/components/esp_adc_cal/Kconfig \
	/mnt/NData/esp32/esp-idf/components/esp_event/Kconfig \
	/mnt/NData/esp32/esp-idf/components/esp_http_client/Kconfig \
	/mnt/NData/esp32/esp-idf/components/esp_http_server/Kconfig \
	/mnt/NData/esp32/esp-idf/components/ethernet/Kconfig \
	/mnt/NData/esp32/esp-idf/components/fatfs/Kconfig \
	/mnt/NData/esp32/esp-idf/components/freemodbus/Kconfig \
	/mnt/NData/esp32/esp-idf/components/freertos/Kconfig \
	/mnt/NData/esp32/esp-idf/components/heap/Kconfig \
	/mnt/NData/esp32/esp-idf/components/libsodium/Kconfig \
	/mnt/NData/esp32/esp-idf/components/log/Kconfig \
	/mnt/NData/esp32/esp-idf/components/lwip/Kconfig \
	/mnt/NData/esp32/esp-idf/components/mbedtls/Kconfig \
	/mnt/NData/esp32/esp-idf/components/mdns/Kconfig \
	/mnt/NData/esp32/esp-idf/components/mqtt/Kconfig \
	/mnt/NData/esp32/esp-idf/components/nvs_flash/Kconfig \
	/mnt/NData/esp32/esp-idf/components/openssl/Kconfig \
	/mnt/NData/esp32/esp-idf/components/pthread/Kconfig \
	/mnt/NData/esp32/esp-idf/components/spi_flash/Kconfig \
	/mnt/NData/esp32/esp-idf/components/spiffs/Kconfig \
	/mnt/NData/esp32/esp-idf/components/tcpip_adapter/Kconfig \
	/mnt/NData/esp32/esp-idf/components/vfs/Kconfig \
	/mnt/NData/esp32/esp-idf/components/wear_levelling/Kconfig \
	/mnt/NData/esp32/esp-idf/components/bootloader/Kconfig.projbuild \
	/mnt/NData/esp32/esp-idf/components/esptool_py/Kconfig.projbuild \
	/mnt/NData/esp32/esp-idf/components/partition_table/Kconfig.projbuild \
	/mnt/NData/esp32/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
