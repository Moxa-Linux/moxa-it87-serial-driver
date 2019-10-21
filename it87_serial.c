// SPDX-License-Identifier: GPL-2.0-only
/*
 *  it87_serial.c - Serial port register control for IT87XX Super I/O chips
 *  Copyright (C) 2019 Moxa, Inc.
 *  Remus Wu <remusty.wu@moxa.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/slab.h> /* kcalloc */
#include <linux/uaccess.h> /* copy_to_user */

#define DRVNAME	"it87_serial"

/* Chip Id numbers */
#define IT8786E_DEVID	0x8786

/* IO Ports */
#define	REG_2E	0x2e	/* The register to read/write */
#define	REG_4E	0x4e	/* Secondary register to read/write */

/* Configuration Registers and Functions */
#define	LDNREG	0x07	/* Register: Logical device select */
#define	CHIPID	0x20	/* Register: Device ID */
#define	CHIPREV	0x22	/* Register: Device Revision */

static inline int superio_enter(int ioreg)
{
	/*
	 * Try to reserve ioreg and ioreg + 1 for exclusive access.
	 */
	if (!request_muxed_region(ioreg, 2, DRVNAME))
		return -EBUSY;

	outb(0x87, ioreg);
	outb(0x01, ioreg);
	outb(0x55, ioreg);
	outb(ioreg == REG_4E ? 0xaa : 0x55, ioreg);
	return 0;
}

static inline void superio_exit(int ioreg)
{
	outb(0x02, ioreg);
	outb(0x02, ioreg + 1);
	release_region(ioreg, 2);
}

static inline void superio_select(int ioreg, int ldn)
{
	outb(LDNREG, ioreg);
	outb(ldn, ioreg + 1);
}

static inline int superio_inb(int ioreg, int reg)
{
	outb(reg, ioreg);
	return inb(ioreg + 1);
}

static inline void superio_outb(int ioreg, int reg, int val)
{
	outb(reg, ioreg);
	outb(val, ioreg + 1);
}

static inline int superio_inw(int ioreg, int reg)
{
	int val;

	outb(reg++, ioreg);
	val = inb(ioreg + 1) << 8;
	outb(reg, ioreg);
	val |= inb(ioreg + 1);
	return val;
}

struct it87_serial {
	u8 ldn;
	u8 act;
	u8 rs485;
};

struct it87_chip {
	u32 devid;
	spinlock_t lock;
	int num_serial;
	struct it87_serial *serial_port;
};

#define IT87_SERIAL_LDN(port, _ldn) { port.ldn = _ldn; }

static struct it87_chip it87_chip = {
	.lock = __SPIN_LOCK_UNLOCKED(it87_chip.lock),
};

/* Serial port configuration registers */
#define IT87_REG_SER_ACT	0x30
#define IT87_REG_SER_SCR	0xf0

/* Special Configuration Register 0xF0 */
#define IT87_REG_SER_SCR_RS485	0x80
#define IT87_REG_SER_SCR_RS485_GET(port)	(port->rs485 >> 7)
#define IT87_REG_SER_SCR_RS485_SET(port, val)	\
	if (val) \
		port->rs485 |= IT87_REG_SER_SCR_RS485; \
	else \
		port->rs485 &= ~IT87_REG_SER_SCR_RS485;

static u8 it87_serial_read(int port, u8 reg)
{
	u8 val;

	superio_enter(REG_2E);
	superio_select(REG_2E, it87_chip.serial_port[port].ldn);
	val = superio_inb(REG_2E, IT87_REG_SER_SCR);
	superio_exit(REG_2E);
	return val;
}

static void it87_serial_write(int port, u8 reg, u8 val)
{
	superio_enter(REG_2E);
	superio_select(REG_2E, it87_chip.serial_port[port].ldn);
	superio_outb(REG_2E, reg, val);
	superio_exit(REG_2E);
}

static ssize_t it87_serial_rs485_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int port;
	struct it87_serial *serial;

	port = simple_strtoul(&attr->attr.name[strlen("serial")], NULL, 10) - 1;
	serial = &it87_chip.serial_port[port];

	spin_lock(&it87_chip.lock);
	serial->rs485 = it87_serial_read(port, IT87_REG_SER_SCR);
	spin_unlock(&it87_chip.lock);

	return sprintf(buf, "%d\n", IT87_REG_SER_SCR_RS485_GET(serial));
}

static ssize_t it87_serial_rs485_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int port;
	struct it87_serial *serial;

	port = simple_strtoul(&attr->attr.name[strlen("serial")], NULL, 10) - 1 ;
	serial = &it87_chip.serial_port[port];

	spin_lock(&it87_chip.lock);
	IT87_REG_SER_SCR_RS485_SET(serial, simple_strtoul(buf, NULL, 10));
	it87_serial_write(port, IT87_REG_SER_SCR, serial->rs485);
	spin_unlock(&it87_chip.lock);

	return count;
}

static DEVICE_ATTR(serial1_rs485, (S_IWUSR | S_IRUGO),
		   it87_serial_rs485_show, it87_serial_rs485_store);
static DEVICE_ATTR(serial2_rs485, (S_IWUSR | S_IRUGO),
		   it87_serial_rs485_show, it87_serial_rs485_store);
static DEVICE_ATTR(serial3_rs485, (S_IWUSR | S_IRUGO),
		   it87_serial_rs485_show, it87_serial_rs485_store);
static DEVICE_ATTR(serial4_rs485, (S_IWUSR | S_IRUGO),
		   it87_serial_rs485_show, it87_serial_rs485_store);
static DEVICE_ATTR(serial5_rs485, (S_IWUSR | S_IRUGO),
		   it87_serial_rs485_show, it87_serial_rs485_store);
static DEVICE_ATTR(serial6_rs485, (S_IWUSR | S_IRUGO),
		   it87_serial_rs485_show, it87_serial_rs485_store);

static struct attribute *it87_serial1[] =
{
	&dev_attr_serial1_rs485.attr,
	NULL
};

static struct attribute *it87_serial2[] =
{
	&dev_attr_serial2_rs485.attr,
	NULL
};

static struct attribute *it87_serial3[] =
{
	&dev_attr_serial3_rs485.attr,
	NULL
};

static struct attribute *it87_serial4[] =
{
	&dev_attr_serial4_rs485.attr,
	NULL
};

static struct attribute *it87_serial5[] =
{
	&dev_attr_serial5_rs485.attr,
	NULL
};

static struct attribute *it87_serial6[] =
{
	&dev_attr_serial6_rs485.attr,
	NULL
};

static const struct attribute_group it87_serial1_groups = {
	.name = "serial1",
	.attrs = it87_serial1,
};

static const struct attribute_group it87_serial2_groups = {
	.name = "serial2",
	.attrs = it87_serial2,
};

static const struct attribute_group it87_serial3_groups = {
	.name = "serial3",
	.attrs = it87_serial3,
};

static const struct attribute_group it87_serial4_groups = {
	.name = "serial4",
	.attrs = it87_serial4,
};

static const struct attribute_group it87_serial5_groups = {
	.name = "serial5",
	.attrs = it87_serial5,
};

static const struct attribute_group it87_serial6_groups = {
	.name = "serial6",
	.attrs = it87_serial6,
};

static const struct attribute_group *it87_serial_groups[] = {
	&it87_serial1_groups,
	&it87_serial2_groups,
	&it87_serial3_groups,
	&it87_serial4_groups,
	&it87_serial5_groups,
	&it87_serial6_groups,
	NULL
};

static struct miscdevice it87_serial_miscdev = {
	.name	= DRVNAME,
	.minor	= MISC_DYNAMIC_MINOR,
};

static int it87_find_chip(struct it87_chip *chip)
{
	int ret = 0;
	u16 chip_type;
	u8 chip_rev;
	struct it87_serial *port;

	spin_lock(&chip->lock);
	ret = superio_enter(REG_2E);
	if (ret)
		return ret;

	chip_type = superio_inw(REG_2E, CHIPID);
	chip_rev  = superio_inb(REG_2E, CHIPREV) & 0x0f;
	superio_exit(REG_2E);
	spin_unlock(&chip->lock);

	switch (chip_type) {
	case IT8786E_DEVID:
		chip->num_serial = 6;
		port = kcalloc(chip->num_serial,
			       sizeof(const struct it87_serial), GFP_KERNEL);
		IT87_SERIAL_LDN(port[0], 0x01);
		IT87_SERIAL_LDN(port[1], 0x02);
		IT87_SERIAL_LDN(port[2], 0x08);
		IT87_SERIAL_LDN(port[3], 0x09);
		IT87_SERIAL_LDN(port[4], 0x0b);
		IT87_SERIAL_LDN(port[5], 0x0c);
		chip->serial_port = port;
		break;
	case 0xffff:
		pr_err("no device\n");
		return -ENODEV;
	default:
		pr_err("Unknown Chip found, Chip %04x Revision %x\n",
		       chip_type, chip_rev);
		return -ENODEV;
	}
	pr_info("Found Chip IT%04x rev %x.\n", chip_type, chip_rev);

	return ret;
}

static int it87_find_serial(struct it87_chip *chip)
{
	int ret = 0, i;

	spin_lock(&chip->lock);
	ret = superio_enter(REG_2E);
	if (ret)
		return ret;

	for (i = 0; i < chip->num_serial; i++) {
		superio_select(REG_2E, chip->serial_port[i].ldn);
		chip->serial_port[i].act = superio_inb(REG_2E, IT87_REG_SER_ACT);
		if ((chip->serial_port[i].act & 0x01)) {
			ret = sysfs_create_group(&it87_serial_miscdev.this_device->kobj,
						 it87_serial_groups[i]);
			if (ret < 0)
				pr_err("Failed to create group for %s\n", DRVNAME);

			pr_info("Found Serial Port %d\n", i + 1);
		}
	}

	superio_exit(REG_2E);
	spin_unlock(&chip->lock);

	return ret;
}

static int __init it87_serial_init(void)
{
	int ret = 0;
	struct it87_chip *chip = &it87_chip;

	ret = it87_find_chip(chip);
	if (ret < 0)
		goto exit_misc_deregister;

	misc_register(&it87_serial_miscdev);
	ret = it87_find_serial(chip);
	if (ret < 0)
		goto exit_misc_deregister;

	return 0;
exit_misc_deregister:
	misc_deregister(&it87_serial_miscdev);
	return ret;
}

static void __exit it87_serial_exit(void)
{
	misc_deregister(&it87_serial_miscdev);
}

module_init(it87_serial_init);
module_exit(it87_serial_exit);

MODULE_DESCRIPTION("Serial Port Register Control for IT8786 Super I/O chips");
MODULE_AUTHOR("Remus Wu <remusty.wu@moxa.com>");
MODULE_LICENSE("GPL");
