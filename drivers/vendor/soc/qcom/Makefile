# When adding new entries keep the list in alphabetical order
CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)

obj-y		+= sdlog/
obj-y		+= vlog/
obj-y		+= pmlog/
obj-y		+= logcat/
obj-y		+= klog/
obj-y		+= imem_info/
obj-y		+= vcfg/
obj-y		+= debugpolicy/
obj-y		+= dpn/
obj-$(CONFIG_VENDOR_SUBSYS_RESTART) +=	subsys_restart/
