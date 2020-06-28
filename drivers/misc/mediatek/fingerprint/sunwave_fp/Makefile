ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
cflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_driver/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_driver/include

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(CONFIG_TRUSTONIC_TEE_VERSION)/gud/MobiCoreKernelApi/include \
			-I$(srctree)/drivers/misc/mediatek/gud/$(CONFIG_TRUSTONIC_TEE_VERSION)/gud/MobiCoreKernelApi/public
endif
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/
obj-$(CONFIG_SUNWAVE_FINGERPRINT)	+= sf_ctl.o sf_spi.o