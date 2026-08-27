idf_component_register(
	SRCS
		"main.c"
		"sdcard.c"
		"fastopen.c"
		"camera_sensor.c"
		"camera_pipeline.c"
		"yuv_convert.c"
		"bmp_writer.c"
		"fbdraw.c"
		"photo.c"
		"viewer.c"
		"video.c"
		"avi_mux.c"
		"usb_device.c"
		"splash.c"
		"focus/dw9714p.c"
		"focus/focus_driver.c"
		"focus/focus_simulator.c"
		"focus/autofocus.c"
		"autoexposure.c"
		"config.c"
		"intfs.c"
		"icons.c"
		"microphone.c"
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
		esp_h264
		shine
		wear_levelling
		pax-codecs
		esp_cam_sensor_ov9281
	INCLUDE_DIRS
		"."
		"focus"
)

# The YUV422 unpack runs over every pixel of every frame, and the
# project default is CONFIG_COMPILER_OPTIMIZATION_DEBUG (-Og), which
# roughly doubles what that loop costs. Raise it for this one file
# only: doing it project-wide would change the codegen of the whole
# camera pipeline, and the point of keeping this in its own
# translation unit is that nothing else is affected.
set_source_files_properties(yuv_convert.c PROPERTIES COMPILE_OPTIONS "-O2")

idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-variable" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-const-variable" APPEND)
