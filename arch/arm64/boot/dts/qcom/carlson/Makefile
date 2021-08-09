ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-y = qm215-qrd-overlay.dtbo
	qm215-qrd-overlay.dtbo-base := qm215.dtb
else
	dtb-$(CONFIG_ARCH_QM215) += qm215.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
