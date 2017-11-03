ifeq ($(EN_DEBUG), 1)
EXTRA_CFLAGS += -DUSE_DEBUG
endif

CURRENT := $(shell uname -r)
KDIR := /lib/modules/$(CURRENT)/build
PWD := $(shell pwd)
RM := rm
PLATFORM=
COMPILER=

obj-m      := smsc7500.o

smsc7500-objs := smsclan7500.o smsc7500usbnet.o

modules:
ifeq "$(PLATFORM)" ""
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
else
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) ARCH=$(PLATFORM) CROSS_COMPILE=$(COMPILER) modules
endif

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean
	$(RM) -f Module.markers modules.order

.PHONY: modules clean

