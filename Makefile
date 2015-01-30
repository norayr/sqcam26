# makefile for sq905 for 2.6 kernels
# 

MODULE_NAME := sqcam

KERNEL_VERSION := `uname -r`
#KERNEL_DIR := /lib/modules/$(KERNEL_VERSION)/build
KERNEL_DIR := /usr/src/linux/

sqcam-objs := sq905.o

obj-m += sqcam.o

# The function remap_page_range has been replaced with remap_pfn_range in
# kernel versions >=2.6.10, define HAS_REMAP_PAGE_RANGE if you need the
# driver to use the old function.
#EXTRA_CFLAGS += -Wall -DSQCAM_DEBUG -DHAS_REMAP_PAGE_RANGE
EXTRA_CFLAGS += -Wall

EXTRA_LDFLAGS := -d

module:
	make -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules

sq905.o : gamma.h

gamma.h : makegamma
	./makegamma

makegamma : makegamma.c
	gcc -std=c99 -o makegamma -lm makegamma.c

install:
	mkdir -p /lib/modules/$(KERNEL_VERSION)/kernel/drivers/usb/media
	cp sqcam.ko /lib/modules/$(KERNEL_VERSION)/kernel/drivers/usb/media
	depmod -a

clean:
	rm -f sqcam.o sq905.o sqcam.ko sqcam.mod.* .sq*.cmd 
