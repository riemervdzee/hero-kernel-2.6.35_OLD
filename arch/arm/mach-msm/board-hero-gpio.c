/* arch/arm/mach-msm/board-hero-gpio.c
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/sysdev.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "gpio_chip.h"
#include "board-hero.h"

#ifdef DEBUG_HERO_GPIO
#define DBG(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define DBG(fmt, arg...) do {} while (0)
#endif

#define	HERO_CPLD_INT_STATUS	(HERO_CPLD_BASE + 0x0E)
#define	HERO_CPLD_INT_LEVEL		(HERO_CPLD_BASE + 0x08)
#define	HERO_CPLD_INT_MASK		(HERO_CPLD_BASE + 0x0C)

/*CPLD misc reg offset*/
static const int _g_CPLD_MISCn_Offset[] = {	0x0A,		/*misc1 reg*/
						0x00,		/*misc2 reg*/
						0x02,		/*misc3 reg*/
						0x04,		/*misc4 reg*/
						0x06};		/*misc5 reg*/
/*CPLD INT Bank*/
/*BANK0: int1 status, int2 level, int3 mask*/
static const int _g_INT_BANK_Offset[][3] = {{0x0E, 0x08, 0x0C} };

static uint8_t hero_cpld_initdata[4]  = {
	[0] = 0x80, /* for serial debug UART3, low current	misc2*/
	[1] = 0x34, /* jog & tp enable, I2C pull		misc3*/
	[3] = 0x04, /* mmdi 32k en				misc5*/
};

/*save current working int mask, so the value can be restored after resume.
Hero has only bank0.*/
static uint8_t hero_int_mask[] = {
	[0] = 0xfb, /* enable all interrupts, bit 2 is not used */
};

/*Sleep have to prepare the wake up source in advance.
default to disable all wakeup sources when suspend.*/
static uint8_t hero_sleep_int_mask[] = {
	[0] = 0x00,	/* bit2 is not used */
};

static int hero_suspended;

static int hero_gpio_read(struct gpio_chip *chip, unsigned n)
{
	if (n < HERO_GPIO_INT_B0_BASE)	/*MISCn*/
		return !!(readb(CPLD_GPIO_REG(n)) & CPLD_GPIO_BIT_POS_MASK(n));
	else if (n <= HERO_GPIO_END)	/*gpio n is INT pin*/
		return !!(readb(CPLD_INT_LEVEL_REG_G(n)) &
						CPLD_GPIO_BIT_POS_MASK(n));
	return 0;
}

/*CPLD Write only register :MISC2, MISC3, MISC4, MISC5 => reg=0,2,4,6
Reading from write-only registers is undefined, so the writing value
should be kept in shadow for later usage.*/
int hero_gpio_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	unsigned long flags;
	uint8_t reg_val;
	if (n > HERO_GPIO_END)
		return -1;

	local_irq_save(flags);
	reg_val = readb(CPLD_GPIO_REG(n));
	if (on)
		reg_val |= CPLD_GPIO_BIT_POS_MASK(n);
	else
		reg_val &= ~CPLD_GPIO_BIT_POS_MASK(n);
	writeb(reg_val, CPLD_GPIO_REG(n));

	DBG("gpio=%d, l=0x%x\r\n", n, readb(HERO_CPLD_INT_LEVEL));

	local_irq_restore(flags);

	return 0;
}

static int hero_gpio_configure(struct gpio_chip *chip, unsigned int gpio,
				   unsigned long flags)
{
	if (flags & (GPIOF_OUTPUT_LOW | GPIOF_OUTPUT_HIGH))
		hero_gpio_write(chip, gpio, flags & GPIOF_OUTPUT_HIGH);

	DBG("gpio=%d, l=0x%x\r\n", gpio, readb(HERO_CPLD_INT_LEVEL));

	return 0;
}

static int hero_gpio_get_irq_num(struct gpio_chip *chip, unsigned int gpio,
				unsigned int *irqp, unsigned long *irqnumflagsp)
{
	DBG("gpio=%d, l=0x%x\r\n", gpio, readb(HERO_CPLD_INT_LEVEL));
	DBG("HERO_GPIO_INT_B0_BASE=%d, HERO_GPIO_LAST_INT=%d\r\n",
	    HERO_GPIO_INT_B0_BASE, HERO_GPIO_LAST_INT);
	if ((gpio < HERO_GPIO_INT_B0_BASE) ||
	     (gpio > HERO_GPIO_LAST_INT))
		return -ENOENT;
	*irqp = HERO_GPIO_TO_INT(gpio);
	DBG("*irqp=%d\r\n", *irqp);
	if (irqnumflagsp)
		*irqnumflagsp = 0;
	return 0;
}

/*write 1 to clear INT status bit.*/
static void hero_gpio_irq_ack(unsigned int irq)
{
	/*write 1 to clear*/
	writeb(HERO_INT_BIT_MASK(irq), CPLD_INT_STATUS_REG(irq));
}

/*unmask/enable the INT
static void hero_gpio_irq_unmask(unsigned int irq)*/
static void hero_gpio_irq_enable(unsigned int irq)
{
	unsigned long flags;
	uint8_t reg_val;

	local_irq_save(flags);	/*disabling all interrupts*/

	reg_val = readb(CPLD_INT_MASK_REG(irq)) | HERO_INT_BIT_MASK(irq);
	DBG("(irq=%d,0x%x, 0x%x)\r\n", irq, CPLD_INT_MASK_REG(irq),
	    HERO_INT_BIT_MASK(irq));
	DBG("hero_suspended=%d\r\n", hero_suspended);
	/*printk(KERN_INFO "hero_gpio_irq_mask irq %d => %d:%02x\n",
	       irq, bank, reg_val);*/
	if (!hero_suspended)
		writeb(reg_val, CPLD_INT_MASK_REG(irq));

	reg_val = readb(CPLD_INT_MASK_REG(irq));
	DBG("reg_val= 0x%x\r\n", reg_val);
	DBG("l=0x%x\r\n", readb(HERO_CPLD_INT_LEVEL));

	local_irq_restore(flags); /*restore the interrupts*/
}

/*mask/disable INT
static void hero_gpio_irq_mask(unsigned int irq)*/
static void hero_gpio_irq_disable(unsigned int irq)
{
	unsigned long flags;
	uint8_t reg_val;

	local_irq_save(flags);
	reg_val = readb(CPLD_INT_MASK_REG(irq)) & ~HERO_INT_BIT_MASK(irq);
	/*CPLD INT MASK is r/w now.*/

	/*printk(KERN_INFO "hero_gpio_irq_unmask irq %d => %d:%02x\n",
	       irq, bank, reg_val);*/
	DBG("(%d,0x%x, 0x%x, 0x%x)\r\n", irq, reg_val, CPLD_INT_MASK_REG(irq),
	    HERO_INT_BIT_MASK(irq));
	DBG("hero_suspended=%d\r\n", hero_suspended);
	if (!hero_suspended)
		writeb(reg_val, CPLD_INT_MASK_REG(irq));

	reg_val = readb(CPLD_INT_MASK_REG(irq));
	DBG("reg_val= 0x%x\r\n", reg_val);
	DBG("l=0x%x\r\n", readb(HERO_CPLD_INT_LEVEL));

	local_irq_restore(flags);
}

/*preparing enable/disable wake source before sleep*/
int hero_gpio_irq_set_wake(unsigned int irq, unsigned int on)
{
	unsigned long flags;
	uint8_t mask = HERO_INT_BIT_MASK(irq);

	local_irq_save(flags);

	if (on)	/*wake on -> mask the bit*/
		hero_sleep_int_mask[CPLD_INT_TO_BANK(irq)] |= mask;
	else	/*no wake -> unmask the bit*/
		hero_sleep_int_mask[CPLD_INT_TO_BANK(irq)] &= ~mask;
	local_irq_restore(flags);
	return 0;
}

/*Hero has only one INT Bank.*/
static void hero_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int j;
	unsigned v;
	int int_base = HERO_INT_START;

	v = readb(HERO_CPLD_INT_STATUS);	/*INT1 status reg, BANK0*/

	for (j = 0; j < 8 ; j++) {	/*8 bit per bank*/
		if (v & (1U << j)) {	/*got the INT Bit*/
			DBG("generic_handle_irq j=0x%x\r\n", j);
			generic_handle_irq(int_base + j);
		}
	}

	desc->chip->ack(irq);	/*clear CPLD INT in SOC side.*/
	DBG("irq=%d, l=0x%x\r\n", irq, readb(HERO_CPLD_INT_LEVEL));
}

/*Save current working sources before sleep, so we can restore it after
 * resume.*/
static int hero_sysdev_suspend(struct sys_device *dev, pm_message_t state)
{
	hero_suspended = 1;
	/*save current masking*/
	hero_int_mask[0] = readb(HERO_CPLD_BASE +
					HERO_GPIO_INT_B0_MASK_REG);

	/*set waking source before sleep.*/
	writeb(hero_sleep_int_mask[0],
	       HERO_CPLD_BASE +  HERO_GPIO_INT_B0_MASK_REG);

	return 0;
}

/*All the registers will be kept till a power loss...*/
int hero_sysdev_resume(struct sys_device *dev)
{
	/*restore the working mask saved before sleep*/
	writeb(hero_int_mask[0], HERO_CPLD_BASE +
					HERO_GPIO_INT_B0_MASK_REG);
	hero_suspended = 0;
	return 0;
}

/**
 * linux/irq.h :: struct irq_chip
 * @enable:		enable the interrupt (defaults to chip->unmask if NULL)
 * @disable:	disable the interrupt (defaults to chip->mask if NULL)
 * @ack:		start of a new interrupt
 * @mask:		mask an interrupt source
 * @mask_ack:		ack and mask an interrupt source
 * @unmask:		unmask an interrupt source
 */
static struct irq_chip hero_gpio_irq_chip = {
	.name      = "herogpio",
	.ack       = hero_gpio_irq_ack,
	.mask      = hero_gpio_irq_disable,	/*hero_gpio_irq_mask,*/
	.unmask    = hero_gpio_irq_enable,	/*hero_gpio_irq_unmask,*/
	.set_wake  = hero_gpio_irq_set_wake,
	/*.set_type  = hero_gpio_irq_set_type,*/
};

/*Thomas:For CPLD*/
static struct gpio_chip hero_gpio_chip = {
	.start = HERO_GPIO_START,
	.end = HERO_GPIO_END,
	.configure = hero_gpio_configure,
	.get_irq_num = hero_gpio_get_irq_num,
	.read = hero_gpio_read,
	.write = hero_gpio_write,
/*	.read_detect_status = hero_gpio_read_detect_status,
	.clear_detect_status = hero_gpio_clear_detect_status */
};

struct sysdev_class hero_sysdev_class = {
	.name = "herogpio_irq",
	.suspend = hero_sysdev_suspend,
	.resume = hero_sysdev_resume,
};

static struct sys_device hero_irq_device = {
	.cls    = &hero_sysdev_class,
};

int hero_init_gpio(void)
{
	int i;
	if (!machine_is_hero())
		return 0;

	DBG("%d,%d\r\n", HERO_INT_START, HERO_INT_END);
	DBG("NR_MSM_IRQS=%d, NR_GPIO_IRQS=%d\r\n", NR_MSM_IRQS, NR_GPIO_IRQS);
	for (i = HERO_INT_START; i <= HERO_INT_END; i++) {
		set_irq_chip(i, &hero_gpio_irq_chip);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	register_gpio_chip(&hero_gpio_chip);

	/*setup CPLD INT connecting to SOC's gpio 17 */
	set_irq_type(MSM_GPIO_TO_INT(17), IRQF_TRIGGER_HIGH);
	set_irq_chained_handler(MSM_GPIO_TO_INT(17), hero_gpio_irq_handler);
	set_irq_wake(MSM_GPIO_TO_INT(17), 1);

	if (sysdev_class_register(&hero_sysdev_class) == 0)
		sysdev_register(&hero_irq_device);

	return 0;
}

int hero_init_cpld(unsigned int sys_rev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hero_cpld_initdata); i++)
		writeb(hero_cpld_initdata[i], HERO_CPLD_BASE + i * 2);
	return 0;
}

postcore_initcall(hero_init_gpio);
