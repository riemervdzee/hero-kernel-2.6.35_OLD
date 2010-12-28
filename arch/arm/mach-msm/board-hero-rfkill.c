/* linux/arch/arm/mach-msm/board-hero-rfkill.c
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

/* Control bluetooth power for hero platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include "gpio_chip.h"
#include "board-hero.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6300";

extern int hero_bt_fastclock_power(int on);

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
		hero_bt_fastclock_power(1);
		gpio_set_value(HERO_GPIO_BT_32K_EN, 1);
		udelay(10);
		gpio_direction_output(101, 1);
	} else {
		gpio_direction_output(101, 0);
		gpio_set_value(HERO_GPIO_BT_32K_EN, 0);
		hero_bt_fastclock_power(0);
	}
	return 0;
}

static struct rfkill_ops hero_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int hero_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	rc = gpio_request(HERO_GPIO_BT_32K_EN, "rfkill");
	rc = gpio_request(101, "rfkill");

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &hero_rfkill_ops, NULL);
	if (!bt_rfk)
		return -ENOMEM;

	/* userspace cannot take exclusive control */

	rfkill_set_states(bt_rfk, default_state, false);

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_destroy(bt_rfk);
	return rc;
}

static int hero_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	gpio_free(HERO_GPIO_BT_32K_EN);
	gpio_free(101);

	return 0;
}

static struct platform_driver hero_rfkill_driver = {
	.probe = hero_rfkill_probe,
	.remove = hero_rfkill_remove,
	.driver = {
		.name = "hero_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init hero_rfkill_init(void)
{
	return platform_driver_register(&hero_rfkill_driver);
}

static void __exit hero_rfkill_exit(void)
{
	platform_driver_unregister(&hero_rfkill_driver);
}

module_init(hero_rfkill_init);
module_exit(hero_rfkill_exit);
MODULE_DESCRIPTION("hero rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
