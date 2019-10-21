KRELEASE ?= $(shell uname -r)
KBUILD ?= /lib/modules/$(KRELEASE)/build
obj-m := it87_serial.o

modules:
	$(MAKE) -C $(KBUILD) M=$(PWD) modules

install: modules
	/usr/bin/install -m 644 -D it87_serial.ko /lib/modules/$(KRELEASE)/kernel/drivers/misc/it87_serial.ko

clean:
	$(MAKE) -C $(KBUILD) M=$(PWD) clean

.PHONY: modules install clean
