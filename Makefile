KRELEASE ?= $(shell uname -r)
KBUILD ?= /lib/modules/$(KRELEASE)/build
obj-m := it87_serial.o

modules:
	$(MAKE) -C $(KBUILD) M=$(PWD) modules
clean:
	$(MAKE) -C $(KBUILD) M=$(PWD) clean

.PHONY: modules clean
