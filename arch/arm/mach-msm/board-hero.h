/* linux/arch/arm/mach-msm/board-hero.h
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
#ifndef __ARCH_ARM_MACH_MSM_BOARD_HERO_H
#define __ARCH_ARM_MACH_MSM_BOARD_HERO_H

#include <mach/board.h>

#define MSM_SMI_BASE		0x00000000
#define MSM_SMI_SIZE		0x00800000

#define MSM_PMEM_GPU0_BASE	0x00000000
#define MSM_PMEM_GPU0_SIZE	0x00700000

#if defined(CONFIG_MSM_AMSS_SUPPORT_256MB_EBI1)
/* AMSS supports unified 128/256MB EBI1 */
#define MSM_EBI_BASE				0x19200000
#define MSM_EBI_SMI64_128MB_SIZE	0x06600000
#define MSM_EBI_SMI32_256MB_SIZE	0x0C600000

#define SMI64_MSM_FB_BASE			0x00700000
#define SMI64_MSM_FB_SIZE			0x0009B000

#define SMI64_MSM_RAM_CONSOLE_BASE	0x007A0000
#define SMI64_MSM_RAM_CONSOLE_SIZE	128 * SZ_1K

#define SMI64_MSM_PMEM_MDP_BASE		0x02000000
#define SMI64_MSM_PMEM_MDP_SIZE		0x00800000

#define SMI64_MSM_PMEM_ADSP_BASE    0x02800000

#define SMI64_MSM_PMEM_ADSP_SIZE	0x00800000

#define SMI64_MSM_PMEM_CAMERA_BASE	0x03000000
#define SMI64_MSM_PMEM_CAMERA_SIZE	0x01000000

#define SMI64_MSM_PMEM_GPU1_BASE	0x1F800000
#define SMI64_MSM_PMEM_GPU1_SIZE	0x800000

#define SMI32_MSM_FB_BASE			0x00700000
#define SMI32_MSM_FB_SIZE			0x0009B000

#define SMI32_MSM_RAM_CONSOLE_BASE	0x007A0000
#define SMI32_MSM_RAM_CONSOLE_SIZE	128 * SZ_1K

#define SMI32_MSM_PMEM_GPU1_BASE	0x25800000
#define SMI32_MSM_PMEM_GPU1_SIZE	0x00800000

#define SMI32_MSM_PMEM_MDP_BASE		0x26000000
#define SMI32_MSM_PMEM_MDP_SIZE		0x00800000

#define SMI32_MSM_PMEM_ADSP_BASE	0x26800000
#define SMI32_MSM_PMEM_ADSP_SIZE	0x00800000

#define SMI32_MSM_PMEM_CAMERA_BASE	0x27000000
#define SMI32_MSM_PMEM_CAMERA_SIZE	0x01000000

#else
/* AMSS supports only 128MB EBI1. */
#define MSM_EBI_BASE				0x10000000

#define SMI64_MSM_FB_BASE			0x00700000
#define SMI64_MSM_FB_SIZE			0x00100000

#define SMI64_MSM_PMEM_MDP_BASE		0x02000000
#define SMI64_MSM_PMEM_MDP_SIZE		0x00800000

#define SMI64_MSM_PMEM_ADSP_BASE    0x02800000
#define SMI64_MSM_PMEM_ADSP_SIZE	0x00800000

#define SMI64_MSM_PMEM_CAMERA_BASE	0x03000000
#define SMI64_MSM_PMEM_CAMERA_SIZE	0x01000000

#define SMI64_MSM_LINUX_BASE		MSM_EBI_BASE
#define SMI64_MSM_LINUX_SIZE		0x06500000

#define SMI64_MSM_RAM_CONSOLE_BASE	MSM_EBI_BASE + 0x06D00000
#define SMI64_MSM_RAM_CONSOLE_SIZE	128 * SZ_1K

#define SMI64_MSM_PMEM_GPU1_SIZE	0x800000
#define SMI64_MSM_PMEM_GPU1_BASE	(SMI64_MSM_RAM_CONSOLE_BASE - SMI64_MSM_PMEM_GPU1_SIZE)

#define SMI32_MSM_LINUX_BASE		MSM_EBI_BASE
#define SMI32_MSM_LINUX_SIZE		0x5400000

#define SMI32_MSM_PMEM_MDP_BASE		(SMI32_MSM_LINUX_BASE + SMI32_MSM_LINUX_SIZE)
#define SMI32_MSM_PMEM_MDP_SIZE		0x800000
 
#define SMI32_MSM_PMEM_ADSP_BASE	(SMI32_MSM_PMEM_MDP_BASE + SMI32_MSM_PMEM_MDP_SIZE)
#define SMI32_MSM_PMEM_ADSP_SIZE	0x800000

#define SMI32_MSM_FB_BASE			(SMI32_MSM_PMEM_ADSP_BASE + SMI32_MSM_PMEM_ADSP_SIZE)
#define SMI32_MSM_FB_SIZE			0x9b000
 
#define SMI32_MSM_RAM_CONSOLE_BASE	(MSM_EBI_BASE + 0x6d00000)
#define SMI32_MSM_RAM_CONSOLE_SIZE	128 * SZ_1K

#define SMI32_MSM_PMEM_GPU1_SIZE	0x800000
#define SMI32_MSM_PMEM_GPU1_BASE	(SMI32_MSM_RAM_CONSOLE_BASE - SMI32_MSM_PMEM_GPU1_SIZE)

#define SMI32_MSM_PMEM_CAMERA_BASE	0
#define SMI32_MSM_PMEM_CAMERA_SIZE	0

#endif

#define EBI1_DUAL_128MB_128MB	17
#define EBI1_MONO_256MB		2
#define EBI1_MONO_128MB		1

#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>

/*
** SOC GPIO
*/
#define HERO_BALL_UP_0     94
#define HERO_BALL_LEFT_0   18
#define HERO_BALL_DOWN_0   49
#define HERO_BALL_RIGHT_0  19

#define HERO_POWER_KEY     20
#define HERO_VOLUME_UP     36
#define HERO_VOLUME_DOWN   39

#define HERO_GPIO_PS_HOLD   (25)
#define HERO_MDDI_1V5_EN	(28)
#define HERO_BL_PWM			(27)
//#define HERO_TP_LS_EN    	(1)
#define HERO20_TP_LS_EN			(88)

/* H2W */
#define HERO_GPIO_CABLE_IN1		(83)
#define HERO_GPIO_CABLE_IN2		(37)
#define HERO_GPIO_UART3_RX		(86)
#define HERO_GPIO_UART3_TX		(87)
#define HERO_GPIO_H2W_DATA		(86)
#define HERO_GPIO_H2W_CLK		(87)

#define HERO_GPIO_AUDIO_JACK	(90)

#define HERO_GPIO_UART1_RTS		(43)
#define HERO_GPIO_UART1_CTS		(44)

/*
** CPLD GPIO
**
** Hero Altera CPLD can keep the registers value and
** doesn't need a shadow to backup.
**/
#define HERO_CPLD_BASE   0xFA000000	/* VA */
#define HERO_CPLD_START  0x98000000	/* PA */
#define HERO_CPLD_SIZE   SZ_4K

#define HERO_GPIO_START (128)				/* Pseudo GPIO number */

/* Hero has one INT BANK only. */
#define HERO_GPIO_INT_B0_MASK_REG           (0x0c)	/*INT3 MASK*/
#define HERO_GPIO_INT_B0_STAT_REG           (0x0e)	/*INT1 STATUS*/

/* LED control register */
#define HERO_CPLD_LED_BASE					(HERO_CPLD_BASE + 0x10)		/* VA */
#define HERO_CPLD_LED_START					(HERO_CPLD_START + 0x10)	/* PA */
#define HERO_CPLD_LED_SIZE					0x08

/* MISCn: GPO pin to Enable/Disable some functions. */
#define HERO_GPIO_MISC1_BASE               	(HERO_GPIO_START + 0x00)
#define HERO_GPIO_MISC2_BASE               	(HERO_GPIO_START + 0x08)
#define HERO_GPIO_MISC3_BASE               	(HERO_GPIO_START + 0x10)
#define HERO_GPIO_MISC4_BASE               	(HERO_GPIO_START + 0x18)
#define HERO_GPIO_MISC5_BASE               	(HERO_GPIO_START + 0x20)

/* INT BANK0: INT1: int status, INT2: int level, INT3: int Mask */
#define HERO_GPIO_INT_B0_BASE              	(HERO_GPIO_START + 0x28)

/* MISCn GPIO: */
#define HERO_GPIO_CPLD128_VER_0            	(HERO_GPIO_MISC1_BASE + 4)
#define HERO_GPIO_CPLD128_VER_1            	(HERO_GPIO_MISC1_BASE + 5)
#define HERO_GPIO_CPLD128_VER_2            	(HERO_GPIO_MISC1_BASE + 6)
#define HERO_GPIO_CPLD128_VER_3            	(HERO_GPIO_MISC1_BASE + 7)

#define HERO_GPIO_H2W_DAT_DIR              	(HERO_GPIO_MISC2_BASE + 2)
#define HERO_GPIO_H2W_CLK_DIR              	(HERO_GPIO_MISC2_BASE + 3)
#define HERO_GPIO_H2W_SEL0                 	(HERO_GPIO_MISC2_BASE + 6)
#define HERO_GPIO_H2W_SEL1                 	(HERO_GPIO_MISC2_BASE + 7)

#define HERO_GPIO_I2C_PULL                 	(HERO_GPIO_MISC3_BASE + 2)
//#define HERO_GPIO_TP_EN					(HERO_GPIO_MISC3_BASE + 4)
#define HERO_GPIO_JOG_EN                   	(HERO_GPIO_MISC3_BASE + 5)
#define HERO_GPIO_JOG_LED_EN               	(HERO_GPIO_MISC3_BASE + 6)
#define HERO_GPIO_APKEY_LED_EN             	(HERO_GPIO_MISC3_BASE + 7)

#define HERO_GPIO_VCM_PWDN                 	(HERO_GPIO_MISC4_BASE + 0)
#define HERO_GPIO_USB_H2W_SW               	(HERO_GPIO_MISC4_BASE + 1)
#define HERO_GPIO_COMPASS_RST_N            	(HERO_GPIO_MISC4_BASE + 2)
//#define HERO_GPIO_USB_PHY_RST_N            	(HERO_GPIO_MISC4_BASE + 5)
#define HERO_GPIO_WIFI_PA_RESETX           	(HERO_GPIO_MISC4_BASE + 6)
#define HERO_GPIO_WIFI_EN                  	(HERO_GPIO_MISC4_BASE + 7)

#define HERO_GPIO_BT_32K_EN                	(HERO_GPIO_MISC5_BASE + 0)
#define HERO_GPIO_MAC_32K_EN               	(HERO_GPIO_MISC5_BASE + 1)
#define HERO_GPIO_MDDI_32K_EN              	(HERO_GPIO_MISC5_BASE + 2)
#define HERO_GPIO_COMPASS_32K_EN           	(HERO_GPIO_MISC5_BASE + 3)

/* INT STATUS/LEVEL/MASK : INT GPIO should be the last. */
#define HERO_GPIO_NAVI_ACT_N           		(HERO_GPIO_INT_B0_BASE + 0)
#define HERO_GPIO_COMPASS_IRQ         		(HERO_GPIO_INT_B0_BASE + 1)
#define HERO_GPIO_SEARCH_ACT_N				(HERO_GPIO_INT_B0_BASE + 2)
#define HERO_GPIO_AUD_HSMIC_DET_N      		(HERO_GPIO_INT_B0_BASE + 3)
//#define HERO_GPIO_SDMC_CD_N				(HERO_GPIO_INT_B0_BASE + 4)
#define HERO_GPIO_CAM_BTN_STEP1_N          	(HERO_GPIO_INT_B0_BASE + 5)
#define HERO_GPIO_CAM_BTN_STEP2_N          	(HERO_GPIO_INT_B0_BASE + 6)
//#define HERO_GPIO_TP_ATT_N				(HERO_GPIO_INT_B0_BASE + 7)

#define HERO_GPIO_END						(HERO_GPIO_INT_B0_BASE + 7)
#define HERO_GPIO_LAST_INT					(HERO_GPIO_INT_B0_BASE + 7)

/* Bit position in the CPLD MISCn by the CPLD GPIOn: only bit0-7 is used. */
#define CPLD_GPIO_BIT_POS_MASK(n)		(1U << ((n) & 7))
#define CPLD_GPIO_REG_OFFSET(n)			_g_CPLD_MISCn_Offset[((n)-HERO_GPIO_START) >> 3]
#define CPLD_GPIO_REG(n)				(CPLD_GPIO_REG_OFFSET(n) + HERO_CPLD_BASE)

/*
** CPLD INT Start
*/
#define HERO_INT_START 					(NR_MSM_IRQS + NR_GPIO_IRQS)	/* pseudo number for CPLD INT */
/* Using INT status/Bank0 for GPIO to INT */
#define HERO_GPIO_TO_INT(n)				((n-HERO_GPIO_INT_B0_BASE) + HERO_INT_START)
#define HERO_INT_END 					(HERO_GPIO_TO_INT(HERO_GPIO_END))

/* get the INT reg by GPIO number */
#define CPLD_INT_GPIO_TO_BANK(n)			(((n)-HERO_GPIO_INT_B0_BASE) >> 3)
#define CPLD_INT_STATUS_REG_OFFSET_G(n)		_g_INT_BANK_Offset[CPLD_INT_GPIO_TO_BANK(n)][0]
#define CPLD_INT_LEVEL_REG_OFFSET_G(n)		_g_INT_BANK_Offset[CPLD_INT_GPIO_TO_BANK(n)][1]
#define CPLD_INT_MASK_REG_OFFSET_G(n)		_g_INT_BANK_Offset[CPLD_INT_GPIO_TO_BANK(n)][2]
#define CPLD_INT_STATUS_REG_G(n)			(HERO_CPLD_BASE + CPLD_INT_STATUS_REG_OFFSET_G(n))
#define CPLD_INT_LEVEL_REG_G(n)				(HERO_CPLD_BASE + CPLD_INT_LEVEL_REG_OFFSET_G(n))
#define CPLD_INT_MASK_REG_G(n)				(HERO_CPLD_BASE + CPLD_INT_MASK_REG_OFFSET_G(n))

/* get the INT reg by INT number */
#define CPLD_INT_TO_BANK(i)					((i-HERO_INT_START) >> 3)
#define CPLD_INT_STATUS_REG_OFFSET(i)		_g_INT_BANK_Offset[CPLD_INT_TO_BANK(i)][0]
#define CPLD_INT_LEVEL_REG_OFFSET(i)		_g_INT_BANK_Offset[CPLD_INT_TO_BANK(i)][1]
#define CPLD_INT_MASK_REG_OFFSET(i)			_g_INT_BANK_Offset[CPLD_INT_TO_BANK(i)][2]
#define CPLD_INT_STATUS_REG(i)				(HERO_CPLD_BASE + CPLD_INT_STATUS_REG_OFFSET(i))
#define CPLD_INT_LEVEL_REG(i)				(HERO_CPLD_BASE + CPLD_INT_LEVEL_REG_OFFSET(i))
#define CPLD_INT_MASK_REG(i)				(HERO_CPLD_BASE + CPLD_INT_MASK_REG_OFFSET(i) )

/* return the bit mask by INT number */
#define HERO_INT_BIT_MASK(i) 			(1U << ((i - HERO_INT_START) & 7))


//ADDED

//#define MSM_LINUX_BASE         0x19200000
//#define MSM_LINUX_SIZE         0xC600000

#define MSM_FB_BASE             0x00700000
#define MSM_FB_SIZE             0x9b000

//#define MSM_RAM_CONSOLE_BASE    0x007A0000
//#define MSM_RAM_CONSOLE_SIZE    128 * SZ_1K

//#define MSM_PMEM_GPU1_BASE      0x25800000
//#define MSM_PMEM_GPU1_SIZE      0x800000

//#define MSM_PMEM_MDP_BASE       0x26000000
//#define MSM_PMEM_MDP_SIZE       0x800000

//#define MSM_PMEM_ADSP_BASE      0x26800000
//#define MSM_PMEM_ADSP_SIZE      0x800000

//#define MSM_PMEM_CAMERA_BASE    0x27000000
//#define MSM_PMEM_CAMERA_SIZE    0x1000000

//#define HERO_POWER_KEY                  (20)
//#define HERO_GPIO_PS_HOLD               (25)
#define HERO_GPIO_MDDI_1V8_EN           (26)
#define HERO_GPIO_UP_INT_N              (27)
#define HERO_GPIO_COMPASS_INT_N   (36)
#define HERO_GPIO_SDMC_CD_N             (38)
#define HERO_GPIO_GSENSOR_INT_N         (49)

/* BT */
//#define HERO_GPIO_UART1_RTS             (43)
//#define HERO_GPIO_UART1_CTS             (44)
#define HERO_GPIO_UART1_RX              (45)
#define HERO_GPIO_UART1_TX              (46)
#define HERO_GPIO_WB_SHUT_DOWN_N        (101)

#define HERO_GPIO_I2C_CLK                       (60)
#define HERO_GPIO_I2C_DAT                       (61)
#define HERO_GPIO_UP_RESET_N            (76)
#define HERO_GPIO_COMPASS_INT_N_XAXB   (83)/*for XA,XB*/
//#define HERO_GPIO_COMPASS_RST_N         (84)
#define HERO_PROJECT_NAME        "hero"
#define HERO_LAYOUTS                    { \
                        { {  0,  1, 0}, {-1,  0, 0}, {0, 0, 1} }, \
                        { {  0, -1, 0}, {-1,  0, 0}, {0, 0, 1} }, \
                        { { -1,  0, 0}, { 0, -1, 0}, {0, 0, 1} }, \
                        { {  1,  0, 0}, { 0,  0, 1}, {0, 1, 0} }  \
                                                                }
#define HERO_GPIO_MDDI_BRIDGE_ID        (85)
#define HERO_GPIO_TP_ATT_N              (90)
//#define HERO_GPIO_VCM_PWDN              (91)
#define HERO_GPIO_CAM_RST_N             (92)
#define HERO_GPIO_EXT_3V_EN             (93)
#define HERO_GPIO_UP_INT_N_XAXB              (94) /*for XA,XB*/
#define HERO_GPIO_MDDI_1V5_EN           (98)
#define HERO_GPIO_MDDI_RST_N            (99)
#define HERO_GPIO_USB_PHY_RST_N         (100)
//#define HERO_GPIO_WIFI_EN               (102)
#define HERO_CAM_PWDN                   (107)
#define HERO_TP_LS_EN                   (108)
#define HERO_GPIO_TP_EN                 (109)

//#define HERO_GPIO_TO_INT(x)             (x+64)/*from gpio_to_irq*/


//#define HERO_GPIO_START (128)                               /* Pseudo GPIO number */
//#define HERO_GPIO_MISC5_BASE                (HERO_GPIO_START + 0x20)
//#define HERO_GPIO_BT_32K_EN                 (HERO_GPIO_MISC5_BASE + 0)

/* JogBall, exist in XC */
#define HERO_GPIO_JOGBALL_EN            (98)
#define HERO_GPIO_JOGBALL_UP_0          (94)
#define HERO_GPIO_JOGBALL_LEFT_0        (39)
#define HERO_GPIO_JOGBALL_DOWN_0        (83)
#define HERO_GPIO_JOGBALL_RIGHT_0       (37)

/* H2W */
#define HERO_GPIO_CABLE_IN1_XAXB        (18)
//#define HERO_GPIO_CABLE_IN1             (49)
//#define HERO_GPIO_CABLE_IN2             (31)
//#define HERO_GPIO_UART3_RX              (86)
//#define HERO_GPIO_UART3_TX              (87)
//#define HERO_GPIO_H2W_DATA              (86)
//#define HERO_GPIO_H2W_CLK               (87)
#define HERO_GPIO_HEADSET_MIC           (17)
#define HERO_GPIO_35MM_HEADSET_DET      (27)
#define HERO_GPIO_AUD_EXTMIC_SEL        (82)


#define HERO_GPIO_VSYNC                 (97)

//int __init parse_tag_skuid(const struct tag * tags);
//int parse_tag_engineerid(const struct tag * tags);

//END


void config_hero_camera_on_gpios(void);
void config_hero_camera_off_gpios(void);
int hero_get_smi_size(void);
unsigned int hero_get_hwid(void);
unsigned int hero_get_skuid(void);
unsigned int hero_get_die_size(void);
unsigned int hero_engineerid(void);
unsigned int is_12pin_camera(void);
unsigned int camera_is_micron_5M(void);
int hero_is_5M_camera(void);
int hero_gpio_write(struct gpio_chip *chip, unsigned n, unsigned on);
extern int panel_type;

#endif /* GUARD */
