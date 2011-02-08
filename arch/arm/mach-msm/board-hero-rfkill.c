/* linux/arch/arm/mach-msm/board-hero-rfkill.c
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
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
#include "proc_comm.h"
#include "board-hero.h"

extern int hero_bt_fastclock_power(int on);

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6350";

static int hero_bt_status;

/*
 * Info on the following tables:
 * - HERO_GPIO_UART1 corresponds to the bluetooth device
 * - With HERO_GPIO_WB_SHUT_DOWN_N we can control the power-mode
 */
static uint32_t hero_bt_init_table[] = {
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_CTS, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RX,  0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_TX,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	
	PCOM_GPIO_CFG(HERO_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
			GPIO_8MA),
};

static uint32_t hero_bt_on_table[] = {
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_CTS, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RX,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_TX,  3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	
	PCOM_GPIO_CFG(HERO_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
			GPIO_8MA),
};

static uint32_t hero_bt_off_table[] = {
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_CTS, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_RX,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_UART1_TX,  3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	
	PCOM_GPIO_CFG(HERO_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
			GPIO_8MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void hero_config_bt_on(void)
{
	config_bt_table(hero_bt_on_table, ARRAY_SIZE(hero_bt_on_table));
	mdelay(2);

	gpio_direction_output(HERO_GPIO_WB_SHUT_DOWN_N, 1);
	mdelay(15);
	gpio_direction_output(HERO_GPIO_WB_SHUT_DOWN_N, 0);
	mdelay(1);
	gpio_direction_output(HERO_GPIO_WB_SHUT_DOWN_N, 1);
	mdelay(1);

	hero_bt_fastclock_power(1);
	mdelay(2);

	hero_bt_status = 1;
}

static void hero_config_bt_off(void)
{
	gpio_direction_output(HERO_GPIO_WB_SHUT_DOWN_N, 0);
	hero_bt_fastclock_power(0);
	config_bt_table(hero_bt_off_table, ARRAY_SIZE(hero_bt_off_table));
	mdelay(5);

	hero_bt_status = 0;
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		hero_config_bt_on();
	else 
		hero_config_bt_off();

	return 0;
}

static struct rfkill_ops hero_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int hero_rfkill_probe(struct platform_device *pdev)
{
	int ret;
	bool default_state = true;  /* off */

	ret = gpio_request(HERO_GPIO_WB_SHUT_DOWN_N, "hero_gpio_wb_shut_down_n");
	if (ret) {
		printk(KERN_ERR "%s: Could not request gpio: %d\n", __func__, ret);
		goto err_gpio_shutdown;
	}

	hero_bt_status = 0;
	config_bt_table(hero_bt_init_table, ARRAY_SIZE(hero_bt_init_table));
	mdelay(5);
	gpio_direction_output(HERO_GPIO_WB_SHUT_DOWN_N, 0);

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &hero_rfkill_ops, NULL);
	if (!bt_rfk){
		printk(KERN_ERR "%s: Could not allocate memmory\n", __func__);
		ret = -ENOMEM;
		goto err_rfkill_alloc;
	}

	/* userspace cannot take exclusive control */
	rfkill_set_states(bt_rfk, default_state, false);

	ret = rfkill_register(bt_rfk);
	if (ret) {
		printk(KERN_ERR "%s: failed to register rfkill: %d\n", __func__, ret);
		goto err_rfkill_reg;
	}

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	gpio_free(HERO_GPIO_WB_SHUT_DOWN_N);
err_gpio_shutdown:
	return ret;
}

static int hero_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	gpio_free(HERO_GPIO_WB_SHUT_DOWN_N);

	return 0;
}

static struct platform_driver hero_rfkill_driver = {
	.probe	= hero_rfkill_probe,
	.remove	= hero_rfkill_remove,
	.driver	= {
		.name	= "hero_rfkill",
		.owner	= THIS_MODULE,
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
