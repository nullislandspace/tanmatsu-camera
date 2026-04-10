idf_component_register(
	SRCS
		"main.c"
		"sdcard.c"
		"fastopen.c"
		"camera_sensor.c"
		"camera_pipeline.c"
		"bmp_writer.c"
		"photo.c"
		"usb_device.c"
	PRIV_REQUIRES
		esp_lcd
		nvs_flash
		fatfs
		sdmmc
		esp_driver_sdmmc
		esp_driver_cam
		esp_driver_isp
		esp_driver_ppa
		esp_driver_jpeg
		esp_mm
		vfs
		badge-bsp
	INCLUDE_DIRS
		"."
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-variable" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-const-variable" APPEND)
