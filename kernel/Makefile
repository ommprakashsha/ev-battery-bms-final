# Kernel module Makefile — EV BMS Driver
obj-m += bms_driver.o

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD        := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean

load:
	sudo insmod bms_driver.ko
	unload:
	sudo rmmod bms_driver

status:
	@lsmod | grep bms || echo 'Module not loaded'
	@cat /proc/bms_status 2>/dev/null || echo '/proc/bms_status not available'

.PHONY: all clean load unload status
