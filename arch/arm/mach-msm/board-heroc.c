/* linux/arch/arm/mach-msm/board-heroc.c
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
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/cy8c_tmg_ts.h>
#include <linux/akm8973.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/bma150.h>
#include <mach/tpa6130.h>

#include <linux/delay.h>

#include <asm/gpio.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <mach/system.h>
#include <mach/vreg.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_debugger.h>
#include <mach/msm_serial_hs.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm_fb.h>
#include <mach/h2w_v1.h>
#include <mach/microp_i2c.h>
#include <mach/htc_battery.h>
#include <mach/drv_callback.h>

#include "proc_comm.h"
#include "devices.h"
#include "gpio_chip.h"
#include "board-heroc.h"
//#include <mach/htc_headset.h>
//#include <mach/i2c-msm.h>
#include <mach/audio_jack.h>
#include <mach/perflock.h>
// msm_hsusb

static unsigned int hwid = 0;
static unsigned int skuid = 0;
static unsigned int engineerid = 0;

extern ssize_t htc_battery_show_batt_attr(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);
static struct htc_battery_platform_data htc_battery_pdev_data = {
//	.func_show_batt_attr = htc_battery_show_batt_attr,
//	.gpio_mbat_in = HERO_GPIO_MBAT_IN,
//	.gpio_mchg_en_n = HERO_GPIO_MCHG_EN_N,
//	.gpio_iset = HERO_GPIO_ISET,
	.guage_driver = GUAGE_MODEM,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
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
#ifdef CONFIG_HEROC_TS
static int heroc_ts_power(int on)
{
	printk(KERN_INFO "heroc_ts_power:%d\n", on);
        if (on) {
                gpio_set_value(HEROC_GPIO_TP_EN, 1);
                msleep(2);				// was msleep(2)
                /* enable touch panel level shift */
                gpio_set_value(HEROC_TP_LS_EN, 1);
                msleep(2);
        } else {
                gpio_set_value(HEROC_TP_LS_EN, 0);
                gpio_set_value(HEROC_GPIO_TP_EN, 0);
        }
        return 0;
}

static struct cy8c_i2c_platform_data heroc_cypress_ts_data = {
	.version = 0x0001,
	.abs_x_min = 0,
	.abs_x_max = 319,
	.abs_y_min = 0,
	.abs_y_max = 479,
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 15,
	.power = heroc_ts_power,
};

static struct synaptics_i2c_rmi_platform_data heroc_ts_data[] = {
	{
		.version = 0x0101,
		.power = heroc_ts_power,
		.sensitivity_adjust = 7,
		.flags = SYNAPTICS_FLIP_X | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
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
                .display_width = 320,
                .display_height = 480,
		.dup_threshold = 10,
	},
	{
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_right = ((4674 - 4334) / 2 + 200) * 0x10000 / 4334,
		.inactive_top = ((6946 - 6696) / 2) * 0x10000 / 6696,
		.inactive_bottom = ((6946 - 6696) / 2) * 0x10000 / 6696,
		 .display_width = 320,
                .display_height = 480,
	}
};
#endif

static struct microp_pin_config microp_pins_0[] = {
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(6, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(10, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
		.suspend_off = 1,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 8,
		.config = MICROP_PIN_CONFIG_GPO,
		.suspend_off = 1,
	},
	{
		.name	= "low-power",
		.pin	= 9,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name   = "adc",
		.pin    = 17,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 2, 4, 12, 26, 51, 117, 204, 470, 651 },
	},
	{
		.name   = "microp_intrrupt",
		.pin	= 16,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask	 = { 0x00, 0x00, 0x00 },
//		.intr_debounce = heroc_microp_intr_debounce,
//                .intr_function = heroc_microp_intr_function,
                .init_intr_function = 0,
	}
};

static struct microp_pin_config microp_pins_1[] = {
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(6, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(10, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO_INV),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{
		.name   = "green",
		.pin    = 3,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name   = "amber",
		.pin    = 5,
		.config = MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "button-backlight",
		.pin	= 7,
		.config = MICROP_PIN_CONFIG_GPO,
		.suspend_off = 1,
	},
	{
		.name	= "jogball-backlight",
		.pin	= 8,
		.config = MICROP_PIN_CONFIG_GPO,
		.suspend_off = 1,
	},
	{
		.name	= "low-power",
		.pin	= 9,
		.config = MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name = "microp_11pin_mic",
		.pin = 1,
		.config = MICROP_PIN_CONFIG_MIC,
		.init_value = 0,
	},
	{
		.name	= "35mm_adc",
		.pin	= 16,
		.adc_pin = 1,
		.intr_pin = 1,
		.config = MICROP_PIN_CONFIG_UP_ADC,
		.levels = { 200, 0x3FF, 0, 33, 38, 82, 95, 167 },
	},
	{
		.name   = "adc",
		.pin    = 17,
		.config = MICROP_PIN_CONFIG_ADC,
		.levels = { 0, 2, 4, 9, 24, 53, 125, 220, 532, 693 },
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 18,
		.config  = MICROP_PIN_CONFIG_INTR_ALL,
		.mask	 = { 0x00, 0x00, 0x00 },
//		.intr_debounce = heroc_microp_intr_debounce,
//                .intr_function = heroc_microp_intr_function,
                .init_intr_function = 0,
	},
};


static struct microp_i2c_platform_data microp_data = {
	.num_pins   = ARRAY_SIZE(microp_pins_1),
	.pin_config = microp_pins_1,
	.gpio_reset = HEROC_GPIO_UP_RESET_N,
	.cabc_backlight_enable = 1,
	.microp_enable_early_suspend = 1,
	.microp_mic_status = 0,
	.microp_enable_reset_button = 1,
};

void heroc_headset_mic_select(uint8_t select)
{
	microp_i2c_set_pin_mode(4, select, microp_data.dev_id);
}

static void heroc_microp_intr_function(uint8_t *pin_status)
{
	static int last_insert = 0;
	int insert;
	/*
	printk(KERN_INFO "heroc_microp_intr_function : %02X %02X %02X\n",
		pin_status[0], pin_status[1], pin_status[2]);
	*/
	if (pin_status[1] & 0x01) {
		insert = 0;
	} else {
		insert = 1;
	}

	if (last_insert != insert) {
		printk(KERN_INFO "heroc_microp_intr_function : %s\n", insert ? "inserted" : "not inserted");
		microp_i2c_set_pin_mode(4, insert, microp_data.dev_id);
#ifdef CONFIG_HTC_HEADSET_V1
		cnf_driver_event("H2W_extend_headset", &insert);
#endif
		last_insert = insert;
	}
}

static struct akm8973_platform_data compass_platform_data = {
	.layouts = HEROC_LAYOUTS,
	.project_name = HEROC_PROJECT_NAME,
	.reset = HEROC_GPIO_COMPASS_RST_N,
	.intr = HEROC_GPIO_COMPASS_INT_N,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = HEROC_GPIO_GSENSOR_INT_N,
};

static struct tpa6130_platform_data headset_amp_platform_data = {
	.gpio_hp_sd = HEROC_GPIO_HTC_HP_SD,
	.enable_rpc_server = 1,
};

static struct i2c_board_info i2c_devices[] = {
#ifdef CONFIG_HEROC_TS
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.platform_data = &heroc_ts_data,
		.irq = MSM_GPIO_TO_INT(HEROC_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(CYPRESS_TMG_NAME, 0x13),
		.platform_data = &heroc_cypress_ts_data,
		.irq = MSM_GPIO_TO_INT(HEROC_GPIO_TP_ATT_N)
	},
#endif
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(HEROC_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(HEROC_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO(BMA150_I2C_NAME, 0x38),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(HEROC_GPIO_GSENSOR_INT_N),
	},
	{
		I2C_BOARD_INFO(TPA6130_I2C_NAME, 0xC0 >> 1),
		.platform_data = &headset_amp_platform_data,
	},
#ifdef CONFIG_HEROC_CAMERA
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1)
	}
#endif
};

static struct i2c_board_info i2c_bma150 = {
        I2C_BOARD_INFO(BMA150_I2C_NAME, 0x38),
        .platform_data = &gsensor_platform_data,
        .irq = MSM_GPIO_TO_INT(HEROC_GPIO_GSENSOR_INT_N),
};


#ifdef CONFIG_HEROC_CAMERA
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
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA), /* MCLK */
};

void config_camera_on_gpios(void)
{
        config_gpio_table(camera_on_gpio_table,
                ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
        config_gpio_table(camera_off_gpio_table,
                ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
        {
                .start  = MSM_VFE_PHYS,
                .end    = MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = INT_VFE,
                .end	= INT_VFE,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,

};

/*static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
        .camera_flash           = flashlight_control,
        .num_flash_levels       = FLASHLIGHT_NUM,
        .low_temp_limit         = 5,
        .low_cap_limit          = 15,

};*/

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
//      .resource = msm_camera_resources,
//      .num_resources = ARRAY_SIZE(msm_camera_resources),
//       .flash_cfg      = &msm_camera_sensor_flash_cfg,
	 .sensor_reset   = HEROC_GPIO_CAM_RST_N,
	 .sensor_pwd     = HEROC_CAM_PWDN,
	 .vcm_pwd        = HEROC_GPIO_VCM_PWDN,
	 .pdata          = &msm_camera_device_data,
	 .flash_type     = MSM_CAMERA_FLASH_NONE,
	 .need_suspend   = 1
	 //.waked_up	= NULL,
	 //.event_wait	= NULL
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev        = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

static void heroc_phy_reset(void)
{
	printk("heroc_phy_reset\n");
	gpio_set_value(HEROC_GPIO_USB_PHY_RST_N, 0);
	mdelay(10);
	gpio_set_value(HEROC_GPIO_USB_PHY_RST_N, 1);
	mdelay(10);
}

static struct pwr_sink heroc_pwrsink_table[] = {
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

static int heroc_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void heroc_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void heroc_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int heroc_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data heroc_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(heroc_pwrsink_table),
	.sinks		= heroc_pwrsink_table,
	.suspend_late	= heroc_pwrsink_suspend_late,
	.resume_early	= heroc_pwrsink_resume_early,
	.suspend_early	= heroc_pwrsink_suspend_early,
	.resume_late	= heroc_pwrsink_resume_late,
};

static struct platform_device heroc_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &heroc_pwrsink_data,
	},
};
/* Switch between UART3 and GPIO */
static uint32_t uart3_on_gpio_table[] = {
	/* RX */
	PCOM_GPIO_CFG(HEROC_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_NO_PULL, 0),
	/* TX */
	PCOM_GPIO_CFG(HEROC_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, 0),
};

/* default TX,RX to GPI */
static uint32_t uart3_off_gpi_table[] = {
	/* RX, H2W DATA */
	PCOM_GPIO_CFG(HEROC_GPIO_H2W_DATA, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	/* TX, H2W CLK */
	PCOM_GPIO_CFG(HEROC_GPIO_H2W_CLK, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
};

/* set TX,RX to GPO */
static uint32_t uart3_off_gpo_table[] = {
        /* RX, H2W DATA */
        PCOM_GPIO_CFG(HEROC_GPIO_H2W_DATA, 0,
                      GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
        /* TX, H2W CLK */
        PCOM_GPIO_CFG(HEROC_GPIO_H2W_CLK, 0,
                      GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

#ifdef CONFIG_HEROC_H2W
static int heroc_h2w_path = H2W_GPIO;

static void h2w_configure(int route)
{
	printk(KERN_INFO "H2W route = %d \n", route);
	switch (route) {
	case H2W_UART3:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+1, 0);
		heroc_h2w_path = H2W_UART3;
		printk(KERN_INFO "H2W -> UART3\n");
		break;
	case H2W_GPIO:
	default:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+1, 0);
		heroc_h2w_path = H2W_GPIO;
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
	gpio_set_value(HEROC_GPIO_H2W_DATA, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(HEROC_GPIO_H2W_CLK, n);
}

static void set_h2w_dat_dir(int n)
{
#if (0)
	if (n == 0) /* input */
		gpio_direction_input(HEROC_GPIO_H2W_DATA);
	else
		gpio_configure(HEROC_GPIO_H2W_DATA, GPIOF_DRIVE_OUTPUT);
#else
	if (n == 0) /* input */
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+0, 0);
	else
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpo_table+0, 0);
#endif
}

static void set_h2w_clk_dir(int n)
{
#if (0)
	if (n == 0) /* input */
		gpio_direction_input(HEROC_GPIO_H2W_CLK);
	else
		gpio_configure(HEROC_GPIO_H2W_CLK, GPIOF_DRIVE_OUTPUT);
#else
	if (n == 0) /* input */
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpi_table+1, 0);
	else
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpo_table+1, 0);
#endif
}

static int get_h2w_dat(void)
{
	return gpio_get_value(HEROC_GPIO_H2W_DATA);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(HEROC_GPIO_H2W_CLK);
}

#ifdef CONFIG_HTC_HEADSET_V1
static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	int enable;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (heroc_h2w_path) {
	case H2W_GPIO:
		enable = 1;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	case H2W_UART3:
		enable = 0;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	default:
		heroc_h2w_path = -1;
		return -EINVAL;
	}

	h2w_configure(heroc_h2w_path);
	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		  &heroc_h2w_path, S_IWUSR | S_IRUGO);

#endif
static struct h2w_platform_data heroc_h2w_data = {
	.h2w_power	 	= HEROC_GPIO_H2W_POWER,
	.cable_in1	 	= HEROC_GPIO_CABLE_IN1,
	.cable_in2	 	= HEROC_GPIO_CABLE_IN2,
	.h2w_clk 		= HEROC_GPIO_H2W_CLK,
	.h2w_data 		= HEROC_GPIO_H2W_DATA,
	.headset_mic_35mm 	= HEROC_GPIO_HEADSET_MIC,
//	.ext_mic_sel 		= HEROC_GPIO_AUD_EXTMIC_SEL,
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat 		= set_h2w_dat,
	.set_clk 		= set_h2w_clk,
	.set_dat_dir 		= set_h2w_dat_dir,
	.set_clk_dir 		= set_h2w_clk_dir,
	.get_dat 		= get_h2w_dat,
	.get_clk 		= get_h2w_clk,
	.headset_mic_sel        = heroc_headset_mic_select,
	.flags 			= REVERSE_MIC_SEL,
};

static struct platform_device heroc_h2w = {
	.name = "h2w",
	.id = -1,
	.dev = {
		.platform_data = &heroc_h2w_data,
	},
};

static uint32_t h2w_gpio_table[] = {
        PCOM_GPIO_CFG(HEROC_GPIO_HEADSET_MIC, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* MCLK */
};

void config_h2w_gpios(void)
{
        config_gpio_table(h2w_gpio_table, ARRAY_SIZE(h2w_gpio_table));
}

#endif

#ifdef CONFIG_HEROC_AUDIO_JACK
static struct audio_jack_platform_data heroc_jack_data = {
        .gpio   = HEROC_GPIO_35MM_HEADSET_DET,
};

static struct platform_device heroc_audio_jack = {
	.name		= "audio-jack",
	.id			= -1,
	.dev		= {
		.platform_data	= &heroc_jack_data,
	},
};
#endif

static struct platform_device heroc_rfkill = {
	.name = "heroc_rfkill",
	.id = -1,
};

static struct msm_pmem_setting pmem_setting_32 = {
	.pmem_start = SMI32_MSM_PMEM_MDP_BASE,
	.pmem_size = SMI32_MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = SMI32_MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = SMI32_MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = SMI32_MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = SMI32_MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = SMI32_MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = SMI32_MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = SMI32_MSM_RAM_CONSOLE_BASE,
	.ram_console_size = SMI32_MSM_RAM_CONSOLE_SIZE,
#ifdef CONFIG_BUILD_CIQ
       .pmem_ciq_start = MSM_PMEM_CIQ_BASE,
       .pmem_ciq_size = MSM_PMEM_CIQ_SIZE,
       .pmem_ciq1_start = MSM_PMEM_CIQ1_BASE,
       .pmem_ciq1_size = MSM_PMEM_CIQ1_SIZE,
       .pmem_ciq2_start = MSM_PMEM_CIQ2_BASE,
       .pmem_ciq2_size = MSM_PMEM_CIQ2_SIZE,
       .pmem_ciq3_start = MSM_PMEM_CIQ3_BASE,
       .pmem_ciq3_size = MSM_PMEM_CIQ3_SIZE,
#endif
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

static struct platform_device *core_devices[] __initdata = {
	&msm_device_smd,
        &msm_device_nand,
};
/*
static struct msm_i2c_device_platform_data heroc_i2c_device_data = {
        .i2c_clock = 100000,
        .clock_strength = GPIO_8MA,
        .data_strength = GPIO_4MA,
};*/

static struct platform_device *devices[] __initdata = {
        &msm_device_i2c,
#ifdef CONFIG_HEROC_H2W
	&heroc_h2w,
#endif
	&htc_battery_pdev,
#ifdef CONFIG_HEROC_AUDIO_JACK
	&heroc_audio_jack,
#endif
	&heroc_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&heroc_pwr_sink,
#endif
	&hero_snd,
#ifdef CONFIG_HEROC_CAMERA
	&msm_camera_sensor_s5k3e2fx,
#endif
};

static struct platform_device *serial_devices[] __initdata = {
#ifdef CONFIG_HEROC_SERIAL
#ifdef CONFIG_SERIAL_MSM_HS
        &msm_device_uart_dm1,
#else
        &msm_device_uart1,
#endif
        &msm_device_uart3,
#endif
};

extern struct sys_timer msm_timer;

static void __init heroc_init_irq(void)
{
	printk("heroc_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint opt_disable_uart3;

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);

static void clear_bluetooth_rx_irq_status(void)
{
        #define GPIO_INT_CLEAR_2 (MSM_GPIO1_BASE + 0x800 + 0x94)
        writel((1U << (HEROC_GPIO_UART1_RX-43)), GPIO_INT_CLEAR_2);
}

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void heroc_reset(void)
{
	gpio_set_value(HEROC_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
};


static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
//	config_heroc_camera_off_gpios();
}


static struct msm_acpu_clock_platform_data heroc_clock_data = {
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


static unsigned heroc_perf_acpu_table[] = {
        264000000,
        480000000,
        518400000,
};

static struct perflock_platform_data heroc_perflock_data = {
        .perf_acpu_table = heroc_perf_acpu_table,
        .table_size = ARRAY_SIZE(heroc_perf_acpu_table),
};


#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(HEROC_GPIO_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

static void __init heroc_init(void)
{
	int rc;
	printk(KERN_INFO "heroc_init() revision=%d\n", system_rev);

	config_gpios();
	config_h2w_gpios();

	msm_hw_reset_hook = heroc_reset;

	msm_acpu_clock_init(&heroc_clock_data);
	perflock_init(&heroc_perflock_data);


#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
				      &msm_device_uart3.dev, 1, INT_UART3_RX);
#endif

	platform_add_devices(core_devices, ARRAY_SIZE(core_devices));
	clear_bluetooth_rx_irq_status();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
	platform_add_devices(serial_devices, ARRAY_SIZE(serial_devices));	

	msm_add_usb_devices(heroc_phy_reset);
	msm_add_mem_devices(&pmem_setting_32);

	msm_init_pmic_vibrator();
	
	rc = heroc_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	// This requires a ported msm_i2c driver that supports overrides
//	msm_device_i2c.dev.platform_data = &heroc_i2c_device_data; 
	platform_add_devices(devices, ARRAY_SIZE(devices));


/*	for (rc=0;rc<ARRAY_SIZE(i2c_devices);rc++){
            if (!strcmp(i2c_devices[rc].type,AKM8973_I2C_NAME)){
                if (!system_rev)
                    i2c_devices[rc].irq = 0x1E; //XA
                else
                    i2c_devices[rc].irq = 0x1C;
            }
        } 
	
	if(system_rev < 2) {
		microp_data.num_pins   = ARRAY_SIZE(microp_pins_0);
		microp_data.pin_config = microp_pins_0;
	} */

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

}


static void __init heroc_fixup(struct machine_desc *desc, struct tag *tags,
			      char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = MSM_EBI_SMI32_256MB_SIZE;

        hwid            = parse_tag_hwid((const struct tag *)tags);
        skuid           = parse_tag_skuid((const struct tag *)tags);
        engineerid      = parse_tag_engineerid((const struct tag *)tags);

}

static void __init heroc_map_io(void)
{
	msm_map_common_io();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

MACHINE_START(HEROC, "heroc")
/* Maintainer: Adnan Begovic <abegovic@huskers.unl.edu> */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = MSM_EBI_BASE + 0x100,
	.fixup          = heroc_fixup,
	.map_io         = heroc_map_io,
	.init_irq       = heroc_init_irq,
	.init_machine   = heroc_init,
	.timer          = &msm_timer,
MACHINE_END
