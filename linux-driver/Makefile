obj-m += mynet_drv.o

# Point this at your buildroot checkout.
BR2_DIR    ?= /home/minimoog/Developer/buildroot


KVER       := $(notdir $(firstword $(wildcard $(BR2_DIR)/output/build/linux-[0-9]*)))
KDIR       ?= $(BR2_DIR)/output/build/$(KVER)
TOOLCHAIN  := $(BR2_DIR)/output/host/bin
CROSS      := x86_64-buildroot-linux-gnu-

 
PWD := $(shell pwd)
 
default:
	$(MAKE) -C $(KDIR) M=$(PWD) \
		CC=$(TOOLCHAIN)/$(CROSS)gcc \
		CROSS_COMPILE=$(TOOLCHAIN)/$(CROSS) \
		ARCH=x86_64 \
		modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: default clean
 
