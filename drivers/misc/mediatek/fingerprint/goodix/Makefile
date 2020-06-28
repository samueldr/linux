# hebiao@wind-mobi.com 20161021 begin
#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
##liukangping 20170112 begin	
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/teei/V1.0/tz_driver/include
##liukangping 20170112 begin	
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(CONFIG_TRUSTONIC_TEE_VERSION)/gud/MobiCoreKernelApi/include \
			-I$(srctree)/drivers/misc/mediatek/gud/$(CONFIG_TRUSTONIC_TEE_VERSION)/gud/MobiCoreKernelApi/public
endif
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/
obj-$(CONFIG_GOODIX_FINGERPRINT)	+= gf_spi_tee.o gf_spi_factory.o
# hebiao@wind-mobi.com 20161021 end