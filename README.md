# moxa-it87-serial-driver

IT8786 support six standard serial ports and RS485 automatic direction control.

This driver provide an interface under misc device for controlling serial register.

## Screenshot

- /sys/class/misc/it87_serial/

```
dev  power  serial1  serial2  subsystem  uevent
```

- /sys/class/misc/it87_serial/serial1

For initial version, provide an interface to control RS485 mode control.
```
serial1_rs485
```

## Install required packages

make, linux-headers-\<KERNEL_RELEASE>

```bash
apt install --no-install-recommends -qqy make
apt install --no-install-recommends -qqy linux-headers-$(uname -r)
```

## Build package

1. Run `make` to build kernel module
2. Once build successful, `it87_serial.ko` could be found under current directory
