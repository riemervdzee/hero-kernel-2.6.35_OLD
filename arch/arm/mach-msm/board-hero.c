/* linux/arch/arm/mach-msm/board-hero.c
 *
 * Copyright (C) 2008 HTC Corporation.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8973.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/bma150.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>
#include <linux/delay.h>
#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/sdio_ids.h>

#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>
#include <asm/mach/mmc.h>

#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_debugger.h>
#include <mach/msm_serial_hs.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm_fb.h>
#include <mach/h2w_v1.h>
#include <mach/microp_i2c.h>
#include <mach/htc_battery.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>

#include "proc_comm.h"
#include "devices.h"
#include "gpio_chip.h"
#include "board-hero.h"

static unsigned int hwid = 0;
static unsigned int skuid = 0;
static unsigned int engineerid = 0;

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver     = GUAGE_MODEM,
	.charger          = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id   = -1,
	.dev  = {
		.platform_data = &htc_battery_pdev_data,
	},
};

unsigned int hero_get_hwid(void)
{
	return hwid;
}

unsigned int hero_get_skuid(void)
{
	return skuid;
}

unsigned hero_engineerid(void)
{
	return engineerid;
}

static int hero_ts_power(int on)
{
	printk(KERN_INFO "hero_ts_power:%d\n", on);
	if (on) {
		gpio_set_value(HERO_GPIO_TP_EN, 1);
		msleep(250);
		/* enable touch panel level shift */
		gpio_set_value(HERO_TP_LS_EN, 1);
		msleep(2);
	} else {
		gpio_set_value(HERO_TP_LS_EN, 0);
		udelay(50);
		gpio_set_value(HERO_GPIO_TP_EN, 0);
	}
	return 0;
}

static struct synaptics_i2c_rmi_platform_data hero_ts_data[] = {
	{
		.version = 0x0101,
		.power = hero_ts_power,
		.sensitivity_adjust = 7,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -50 * 0x10000 / 4334,
		.inactive_right = -50 * 0x10000 / 4334,
		.inactive_top = -40 * 0x10000 / 6696,
		.inactive_bottom = -40 * 0x10000 / 6696,
		.snap_left_on = 50 * 0x10000 / 4334,
		.snap_left_off = 60 * 0x10000 / 4334,
		.snap_right_on = 50 * 0x10000 / 4334,
		.snap_right_off = 60 * 0x10000 / 4334,
		.snap_top_on = 100 * 0x10000 / 6696,
		.snap_top_off = 110 * 0x10000 / 6696,
		.snap_bottom_on = 100 * 0x10000 / 6696,
		.snap_bottom_off = 110 * 0x10000 / 6696,
		.dup_threshold = 10,
	},
	{
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_right = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_top = ((6946 - 6696) / 2) * 0x10000 / 6696,
		.inactive_bottom = ((6946 - 6696) / 2) * 0x10000 / 6696,
	}
};

static int hero_microp_intr_debounce(uint8_t *pin_status);
static void hero_microp_intr_function(uint8_t *pin_status);

static struct microp_pin_config microp_pins_skuid_0[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(10, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(16, MICROP_PIN_CONFIG_GPO),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x01 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "lcd-backlight",
		.pin    = 6,
		.config = MICROP_PIN_CONFIG_PWM,
		.freq   = MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels = { 30, 48, 66, 84, 102, 133, 163, 194, 224, 255 },
		.dutys	= {  8, 16, 34, 61,  96, 138, 167, 195, 227, 255 },
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

/* XC and enable LABC */
static struct microp_pin_config microp_pins_skuid_1[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(10, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(16, MICROP_PIN_CONFIG_GPO_INV),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x01 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.name	= "35mm_adc",
		.pin	= 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	},
	{
		.name   = "microp_intrrupt",
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

/* XD, add jogball backlight function*/
static struct microp_pin_config microp_pins_skuid_2[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x03 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_PWM,
		.auto_if_on = 1,
		.i_am_jogball_function = 1,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.name	= "35mm_adc",
		.pin	= 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	},
	{
		.name   = "microp_intrrupt",
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

/* XE, 11pin mic function*/
static struct microp_pin_config microp_pins_skuid_3[] = {
	MICROP_PIN(23, MICROP_PIN_CONFIG_PULL_UP),
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{	.name = "microp-pullup",
		.pin = 23,
		.config = MICROP_PIN_CONFIG_PULL_UP1,
		.mask = { 0x00, 0x00, 0x03 },
	},
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 10,
		.config = MICROP_PIN_CONFIG_PWM,
		.auto_if_on = 1,
		.i_am_jogball_function = 1,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 16,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 1,
	},
	{
		.name   = "adc",
		.pin    = 24,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 0, 0, 6, 24, 60, 425, 497, 569, 638 },
	},
	{
		.name	= "35mm_adc",
		.pin	= 25,
		.adc_pin = 7,
		.config = MICROP_PIN_CONFIG_ADC_READ,
	},
	{
		.name   = "microp_intrrupt",
		.pin	 = 17,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask 	 = { 0x00, 0x01, 0x00 },
		.intr_debounce = hero_microp_intr_debounce,
		.intr_function = hero_microp_intr_function,
		.init_intr_function = 1,
	}
};

static struct microp_i2c_platform_data microp_data = {
	.num_pins   = ARRAY_SIZE(microp_pins_skuid_0),
	.pin_config = microp_pins_skuid_0,
	.gpio_reset = HERO_GPIO_UP_RESET_N,
	.cabc_backlight_enable = 0,
	.microp_enable_early_suspend = 1,
	.microp_enable_reset_button = 1,
};

#define DEBOUNCE_LENGTH 4
static int hero_microp_intr_debounce(uint8_t *pin_status)
{
/*Per HW RD's request, wait 300 mill-seconds.*/
#if 1
	mdelay(100);
	return 0;
#else
	static int count;
	static uint8_t data[DEBOUNCE_LENGTH];

	if (pin_status[0] == 0 && pin_status[1] == 0 && pin_status[2] == 0) {
		mdelay(5);
		return 1;
	}
	/*
	printk(KERN_INFO "hero_microp_intr_debounce : %02X %02X %02X\n",
		pin_status[0], pin_status[1], pin_status[2]);
	*/
	if (count < DEBOUNCE_LENGTH - 1) {
		data[count] = pin_status[1] & 0x01;
		count++;
	} else {
		data[DEBOUNCE_LENGTH - 1] = pin_status[1] & 0x01;
		for (count = 0; count < DEBOUNCE_LENGTH - 1; count++)
			if (data[count] != data[count + 1])
				break;
		if (count == DEBOUNCE_LENGTH - 1) {
			count = 0;
			return 0;
		}
		for (count = 0; count < DEBOUNCE_LENGTH - 1; count++)
			data[count] = data[count + 1];
	}

	mdelay(20);

	return 1;
#endif
}

void hero_headset_mic_select(uint8_t select)
{
	microp_i2c_set_pin_mode(4, select, microp_data.dev_id);
}

static void hero_microp_intr_function(uint8_t *pin_status)
{
	static int last_insert = 0;
	int insert;
	/*
	printk(KERN_INFO "hero_microp_intr_function : %02X %02X %02X\n",
		pin_status[0], pin_status[1], pin_status[2]);
	*/
	if (pin_status[1] & 0x01) {
		insert = 0;
	} else {
		insert = 1;
	}

	if (last_insert != insert) {
		printk(KERN_INFO "hero_microp_intr_function : %s\n", insert ? "inserted" : "not inserted");
		microp_i2c_set_pin_mode(4, insert, microp_data.dev_id);
#ifdef CONFIG_HTC_HEADSET_V1
		cnf_driver_event("H2W_extend_headset", &insert);
#endif
		last_insert = insert;
	}
}

static struct akm8973_platform_data compass_platform_data = {
	.layouts = HERO_LAYOUTS,
	.project_name = HERO_PROJECT_NAME,
	.reset = HERO_GPIO_COMPASS_RST_N,
	.intr = HERO_GPIO_COMPASS_INT_N,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = HERO_GPIO_GSENSOR_INT_N,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = &hero_ts_data,
		.irq = MSM_GPIO_TO_INT(HERO_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(HERO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(HERO_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
	{
		I2C_BOARD_INFO(BMA150_I2C_NAME, 0x38),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(HERO_GPIO_GSENSOR_INT_N),
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_hero_camera_on_gpios,
	.camera_gpio_off = config_hero_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name	= "mt9p012",
	.sensor_reset	= HERO_GPIO_CAM_RST_N,
	.sensor_pwd		= HERO_CAM_PWDN,
	.vcm_pwd		= HERO_GPIO_VCM_PWDN,
	.pdata			= &msm_camera_device_data,
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name		= "msm_camera_mt9p012",
	.dev		= {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};

#ifdef CONFIG_HERO_EXPERIMENTAL_USBADSP
/*static int mahimahi_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1 }; MAHIMAHI */
static int hero_phy_init_seq[] = {0x40, 0x31, 0x1, 0x0D, 0x1, 0x10, -1};

static void hero_usb_phy_reset(void)
{
	gpio_set_value(HERO_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(HERO_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

#if 0
//TODO, check if neccesary?
static void hero_usb_hw_reset(bool enable)
{
	u32 id;
	int ret;
	u32 func;

	id = PCOM_CLKRGM_APPS_RESET_USBH;
	if (enable)
		func = PCOM_CLK_REGIME_SEC_RESET_ASSERT;
	else
		func = PCOM_CLK_REGIME_SEC_RESET_DEASSERT;

	ret = msm_proc_comm(func, &id, NULL);

	if (ret)
		pr_err("%s: Cannot set reset to %d (%d)\n", __func__, enable, ret);
}
#endif


static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq	= hero_phy_init_seq,
	.phy_reset		= hero_usb_phy_reset,
//	.hw_reset		= hero_usb_hw_reset, TODO, check if neccesary?
//	.usb_connected	= notify_usb_connected, TODO!
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
#endif

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id    = 0x0ff9, /* usb_mass_storage */
		.num_functions = ARRAY_SIZE(usb_functions_ums),
		.functions     = usb_functions_ums,
	},
	{
		.product_id    = 0x0c99, /* usb_mass_storage + adb */
		.num_functions = ARRAY_SIZE(usb_functions_ums_adb),
		.functions     = usb_functions_ums_adb,
	},
	{
		.product_id    = 0x0FFE, /* internet sharing */
		.num_functions = ARRAY_SIZE(usb_functions_rndis),
		.functions     = usb_functions_rndis,
	},
	/*
	 * TODO: check whether this is working or not. Kinda a guess
	 */
	{
		.product_id    = 0x0FFC,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions     = usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory),
		.functions	= usb_functions_accessory,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_adb),
		.functions	= usb_functions_accessory_adb,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id    = 0x0c07,
		.num_functions = ARRAY_SIZE(usb_functions_adb_diag),
		.functions     = usb_functions_adb_diag,
	},
#endif
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Hero",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id   = -1,
	.dev  = {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID    = 0x0bb4,
	.vendorDescr = "HTC",
};

static struct platform_device rndis_device = {
	.name = "rndis",
	.id   = -1,
	.dev  = {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id         = 0x0bb4,
	.product_id        = 0x0c01,
	.version           = 0x0100,
	.product_name      = "Android Phone",
	.manufacturer_name = "HTC",
	.num_products      = ARRAY_SIZE(usb_products),
	.products          = usb_products,
	.num_functions     = ARRAY_SIZE(usb_functions_all),
	.functions         = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name = "android_usb",
	.id   = -1,
	.dev  = {
		.platform_data = &android_usb_pdata,
	},
};

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name         = "pmem",
	.start        = SMI32_MSM_PMEM_MDP_BASE,
	.size         = SMI32_MSM_PMEM_MDP_SIZE,
	.no_allocator = 0,
	.cached       = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name         = "pmem_adsp",
	.start        = SMI32_MSM_PMEM_ADSP_BASE,
	.size         = SMI32_MSM_PMEM_ADSP_SIZE,
	.no_allocator = 0,
	.cached       = 0,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name         = "pmem_camera",
	.start        = SMI32_MSM_PMEM_CAMERA_BASE,
	.size         = SMI32_MSM_PMEM_CAMERA_SIZE,
	.no_allocator = 1,
	.cached       = 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name = "android_pmem",
	.id   = 0,
	.dev  = {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id   = 1,
	.dev  = {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id   = 4,
	.dev  = {
		.platform_data = &android_pmem_camera_pdata
	},
};

static struct resource resources_hw3d[] = {
	{
		.start	= 0xA0000000,
		.end	= 0xA00fffff,
		.flags	= IORESOURCE_MEM,
		.name	= "regs",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "smi",
		.start  = SMI32_MSM_PMEM_GPU0_BASE,
		.end    = SMI32_MSM_PMEM_GPU0_BASE + SMI32_MSM_PMEM_GPU0_SIZE - 1,
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "ebi",
		.start  = SMI32_MSM_PMEM_GPU1_BASE,
		.end    = SMI32_MSM_PMEM_GPU1_BASE + SMI32_MSM_PMEM_GPU1_SIZE - 1,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
		.name	= "gfx",
	},
};

static struct platform_device hw3d_device = {
	.name          = "msm_hw3d",
	.id            = 0,
	.num_resources = ARRAY_SIZE(resources_hw3d),
	.resource      = resources_hw3d,
};

static struct resource ram_console_resources[] = {
	{
		.start = SMI32_MSM_RAM_CONSOLE_BASE,
		.end   = SMI32_MSM_RAM_CONSOLE_BASE + SMI32_MSM_RAM_CONSOLE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name          = "ram_console",
	.id            = -1,
	.num_resources = ARRAY_SIZE(ram_console_resources),
	.resource      = ram_console_resources,
};

#else /* !CONFIG_HERO_EXPERIMENTAL_USBADSP */

static void hero_phy_reset(void)
{
	gpio_set_value(HERO_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(HERO_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

static struct msm_pmem_setting pmem_setting_32 = {
	.pmem_start        = SMI32_MSM_PMEM_MDP_BASE,
	.pmem_size         = SMI32_MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start   = SMI32_MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size    = SMI32_MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start   = SMI32_MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size    = SMI32_MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start   = SMI32_MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size    = SMI32_MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = SMI32_MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size  = SMI32_MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = SMI32_MSM_RAM_CONSOLE_BASE,
	.ram_console_size  = SMI32_MSM_RAM_CONSOLE_SIZE,
};

#endif

static struct pwr_sink hero_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 100000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 125000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_BLUETOOTH,
		.ua_max	= 15000,
	},
	{
		.id	= PWRSINK_CAMERA,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_SDCARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_VIDEO,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id	= PWRSINK_SYSTEM_LOAD,
		.ua_max	= 100000,
		.percent_util = 38,
	},
};

static int hero_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void hero_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void hero_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int hero_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data hero_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(hero_pwrsink_table),
	.sinks		= hero_pwrsink_table,
	.suspend_late	= hero_pwrsink_suspend_late,
	.resume_early	= hero_pwrsink_resume_early,
	.suspend_early	= hero_pwrsink_suspend_early,
	.resume_late	= hero_pwrsink_resume_late,
};

static struct platform_device hero_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &hero_pwrsink_data,
	},
};
/* Switch between UART3 and GPIO */
static uint32_t uart3_on_gpio_table[] = {
	/* RX */
	PCOM_GPIO_CFG(HERO_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_NO_PULL, 0),
	/* TX */
	PCOM_GPIO_CFG(HERO_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, 0),
};

/* default TX,RX to GPI */
static uint32_t uart3_off_gpio_table[] = {
	/* RX, H2W DATA */
	PCOM_GPIO_CFG(HERO_GPIO_H2W_DATA, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	/* TX, H2W CLK */
	PCOM_GPIO_CFG(HERO_GPIO_H2W_CLK, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
};

static int hero_h2w_path = H2W_GPIO;

static void h2w_configure(int route)
{
	printk(KERN_INFO "H2W route = %d \n", route);
	switch (route) {
	case H2W_UART3:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, uart3_on_gpio_table + 0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, uart3_on_gpio_table + 1, 0);
		hero_h2w_path = H2W_UART3;
		printk(KERN_INFO "H2W -> UART3\n");
		break;
	case H2W_GPIO:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, uart3_off_gpio_table + 0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, uart3_off_gpio_table + 1, 0);
		hero_h2w_path = H2W_GPIO;
		printk(KERN_INFO "H2W -> GPIO\n");
		break;
	}
}

static void h2w_defconfig(void)
{
	h2w_configure(H2W_GPIO);
}

static void set_h2w_dat(int n)
{
	gpio_set_value(HERO_GPIO_H2W_DATA, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(HERO_GPIO_H2W_CLK, n);
}

static void set_h2w_dat_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(HERO_GPIO_H2W_DATA);
	else
		gpio_configure(HERO_GPIO_H2W_DATA, GPIOF_DRIVE_OUTPUT);
}

static void set_h2w_clk_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(HERO_GPIO_H2W_CLK);
	else
		gpio_configure(HERO_GPIO_H2W_CLK, GPIOF_DRIVE_OUTPUT);
}

static int get_h2w_dat(void)
{
	return gpio_get_value(HERO_GPIO_H2W_DATA);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(HERO_GPIO_H2W_CLK);
}

static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	int enable;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (hero_h2w_path) {
	case H2W_GPIO:
		enable = 1;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	case H2W_UART3:
		enable = 0;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	default:
		hero_h2w_path = -1;
		return -EINVAL;
	}

	h2w_configure(hero_h2w_path);
	return ret;
}

static void hero_h2w_power(int on)
{
	if (on)
		gpio_set_value(HERO_GPIO_EXT_3V_EN, 1);
	else
		gpio_set_value(HERO_GPIO_EXT_3V_EN, 0);
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&hero_h2w_path, S_IWUSR | S_IRUGO);

/* TODO combine to one struct, hardcode the differences? */
static struct h2w_platform_data hero_h2w_data = {
	.h2w_power        = HERO_GPIO_EXT_3V_EN,
	.cable_in1        = HERO_GPIO_CABLE_IN1_XAXB,
	.cable_in2        = HERO_GPIO_CABLE_IN2,
	.h2w_clk          = HERO_GPIO_H2W_CLK,
	.h2w_data         = HERO_GPIO_H2W_DATA,
	.headset_mic_35mm = HERO_GPIO_HEADSET_MIC,
	.debug_uart       = H2W_UART3,
	.config           = h2w_configure,
	.defconfig        = h2w_defconfig,
	.set_dat          = set_h2w_dat,
	.set_clk          = set_h2w_clk,
	.set_dat_dir      = set_h2w_dat_dir,
	.set_clk_dir      = set_h2w_clk_dir,
	.get_dat          = get_h2w_dat,
	.get_clk          = get_h2w_clk,
	.headset_mic_sel  = hero_headset_mic_select,
	.flags            = HTC_11PIN_HEADSET_SUPPORT | HTC_H2W_SUPPORT,
};

static struct h2w_platform_data hero_h2w_data_xc = {
	.h2w_power        = HERO_GPIO_EXT_3V_EN,
	.cable_in1        = HERO_GPIO_CABLE_IN1,
	.cable_in2        = HERO_GPIO_CABLE_IN2,
	.h2w_clk          = HERO_GPIO_H2W_CLK,
	.h2w_data         = HERO_GPIO_H2W_DATA,
	.headset_mic_35mm = HERO_GPIO_HEADSET_MIC,
	.ext_mic_sel      = HERO_GPIO_AUD_EXTMIC_SEL,
	.debug_uart       = H2W_UART3,
	.config           = h2w_configure,
	.defconfig        = h2w_defconfig,
	.set_dat          = set_h2w_dat,
	.set_clk          = set_h2w_clk,
	.set_dat_dir      = set_h2w_dat_dir,
	.set_clk_dir      = set_h2w_clk_dir,
	.get_dat          = get_h2w_dat,
	.get_clk          = get_h2w_clk,
	.flags            = HTC_11PIN_HEADSET_SUPPORT | HTC_H2W_SUPPORT,
};

static struct h2w_platform_data hero_h2w_data_xe = {
	.h2w_power        = HERO_GPIO_EXT_3V_EN,
	.cable_in1        = HERO_GPIO_CABLE_IN1,
	.cable_in2        = HERO_GPIO_CABLE_IN2,
	.h2w_clk          = HERO_GPIO_H2W_CLK,
	.h2w_data         = HERO_GPIO_H2W_DATA,
	.headset_mic_35mm = HERO_GPIO_HEADSET_MIC,
	.ext_mic_sel      = HERO_GPIO_AUD_EXTMIC_SEL,
	.debug_uart       = H2W_UART3,
	.config           = h2w_configure,
	.defconfig        = h2w_defconfig,
	.set_dat          = set_h2w_dat,
	.set_clk          = set_h2w_clk,
	.set_dat_dir      = set_h2w_dat_dir,
	.set_clk_dir      = set_h2w_clk_dir,
	.get_dat          = get_h2w_dat,
	.get_clk          = get_h2w_clk,
	.flags            = _35MM_MIC_DET_L2H |
						HTC_11PIN_HEADSET_SUPPORT | HTC_H2W_SUPPORT,
};

static struct platform_device hero_h2w = {
	.name = "h2w",
	.id   = -1,
	.dev  = {
		.platform_data = &hero_h2w_data,
	},
};

static struct platform_device hero_rfkill = {
	.name = "hero_rfkill",
	.id = -1,
};

#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(3, "BT"),
	SND(44, "BT_EC_OFF"),
	SND(10, "HEADSET_AND_SPEAKER"),
	SND(256, "CURRENT"),

	/* Bluetooth accessories. */

	SND(12, "HTC BH S100"),
	SND(13, "HTC BH M100"),
	SND(14, "Motorola H500"),
	SND(15, "Nokia HS-36W"),
	SND(16, "PLT 510v.D"),
	SND(17, "M2500 by Plantronics"),
	SND(18, "Nokia HDW-3"),
	SND(19, "HBH-608"),
	SND(20, "HBH-DS970"),
	SND(21, "i.Tech BlueBAND"),
	SND(22, "Nokia BH-800"),
	SND(23, "Motorola H700"),
	SND(24, "HTC BH M200"),
	SND(25, "Jabra JX10"),
	SND(26, "320Plantronics"),
	SND(27, "640Plantronics"),
	SND(28, "Jabra BT500"),
	SND(29, "Motorola HT820"),
	SND(30, "HBH-IV840"),
	SND(31, "6XXPlantronics"),
	SND(32, "3XXPlantronics"),
	SND(33, "HBH-PV710"),
	SND(34, "Motorola H670"),
	SND(35, "HBM-300"),
	SND(36, "Nokia BH-208"),
	SND(37, "Samsung WEP410"),
	SND(38, "Jabra BT8010"),
	SND(39, "Motorola S9"),
	SND(40, "Jabra BT620s"),
	SND(41, "Nokia BH-902"),
	SND(42, "HBH-DS220"),
	SND(43, "HBH-DS980"),
};
#undef SND

static struct msm_snd_endpoints hero_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device hero_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev	= {
		.platform_data = &hero_snd_endpoints,
	},
};

static struct msm_i2c_device_platform_data hero_i2c_device_data = {
	.i2c_clock = 100000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#else
	&msm_device_uart1,
#endif
	&msm_device_uart3,
	&msm_camera_sensor_mt9p012,
	&htc_battery_pdev,
	&hero_rfkill,
	&hero_h2w,
#ifdef CONFIG_HTC_PWRSINK
	&hero_pwr_sink,
#endif
	&hero_snd,
#ifdef CONFIG_HERO_EXPERIMENTAL_USBADSP
	&msm_device_hsusb,
	&usb_mass_storage_device,
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_device,
#endif
	&android_usb_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&hw3d_device,
	&ram_console_device,
#endif
};

extern struct sys_timer msm_timer;

static void __init hero_init_irq(void)
{
	printk(KERN_DEBUG "hero_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint opt_disable_uart3;

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);



static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void hero_reset(void)
{
	gpio_set_value(HERO_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	PCOM_GPIO_CFG(HERO_GPIO_I2C_CLK, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(HERO_GPIO_I2C_DAT , 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
};


static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

void config_hero_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_hero_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
	config_hero_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data hero_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
#if defined(CONFIG_JESUS_PHONE)
	.wait_for_irq_khz = 176000,
#else
	.wait_for_irq_khz = 128000,
#endif
};

static unsigned hero_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};


static struct perflock_platform_data hero_perflock_data = {
	.perf_acpu_table = hero_perf_acpu_table,
	.table_size = ARRAY_SIZE(hero_perf_acpu_table),
};


#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq       = MSM_GPIO_TO_INT(HERO_GPIO_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject        = 0x32,
};
#endif

static void __init hero_init(void)
{
	int rc;
	printk("hero_init() revision = 0x%X\n", system_rev);

	android_usb_pdata.serial_number = board_serialno();

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	msm_hw_reset_hook = hero_reset;

	gpio_request(HERO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(HERO_TP_LS_EN, 0);
	gpio_request(HERO_GPIO_EXT_3V_EN, "hero_gpio_ext_3v_en");
	gpio_request(HERO_GPIO_CABLE_IN2, "hero_gpio_cable_in2");
	gpio_request(HERO_GPIO_AUD_EXTMIC_SEL, "hero_gpio_aud_extmic_sel");

	msm_acpu_clock_init(&hero_clock_data);
	perflock_init(&hero_perflock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1, INT_UART3_RX);
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_HERO_EXPERIMENTAL_USBADSP
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
#else
	msm_add_usb_devices(hero_phy_reset);
	msm_add_mem_devices(&pmem_setting_32);
#endif

	msm_init_pmic_vibrator();

	rc = hero_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &hero_i2c_device_data;

	if (system_rev == 0 || system_rev == 1) {
		for (rc = 0; rc < ARRAY_SIZE(i2c_devices); rc++) {
			if (!strcmp(i2c_devices[rc].type, MICROP_I2C_NAME))
				i2c_devices[rc].irq = MSM_GPIO_TO_INT(HERO_GPIO_UP_INT_N_XAXB);
			if (!strcmp(i2c_devices[rc].type, AKM8973_I2C_NAME))
				i2c_devices[rc].irq = MSM_GPIO_TO_INT(HERO_GPIO_COMPASS_INT_N_XAXB);
		}
	} else if (system_rev == 2 || system_rev == 3) /*XC and XD*/
		hero_h2w.dev.platform_data = &hero_h2w_data_xc;
	else /*above XE*/
		hero_h2w.dev.platform_data = &hero_h2w_data_xe;

	if (hero_engineerid() || system_rev > 2) {
		if (system_rev >= 4) {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_skuid_3);
			microp_data.pin_config = microp_pins_skuid_3;
		} else if (system_rev == 3) {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_skuid_2);
			microp_data.pin_config = microp_pins_skuid_2;
		} else {
			microp_data.num_pins = ARRAY_SIZE(microp_pins_skuid_1);
			microp_data.pin_config = microp_pins_skuid_1;
		}
		microp_data.cabc_backlight_enable = 1;
	}

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	
	hero_init_panel();
}

static void __init hero_fixup(struct machine_desc *desc, struct tag *tags,
				  char **cmdline, struct meminfo *mi)
{
	mi->nr_banks		= 1;
	mi->bank[0].start	= PHYS_OFFSET;
	mi->bank[0].node	= PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size	= MSM_EBI_SMI32_256MB_SIZE;

	hwid		= parse_tag_hwid((const struct tag *)tags);
	skuid		= parse_tag_skuid((const struct tag *)tags);
	engineerid	= parse_tag_engineerid((const struct tag *)tags);

}

static void __init hero_map_io(void)
{
	msm_map_common_io();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

MACHINE_START(HERO, "hero")
/* Maintainer: Kant Kang <kant_kang@htc.com> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = MSM_EBI_BASE + 0x100,
	.fixup          = hero_fixup,
	.map_io         = hero_map_io,
	.init_irq       = hero_init_irq,
	.init_machine   = hero_init,
	.timer          = &msm_timer,
MACHINE_END
