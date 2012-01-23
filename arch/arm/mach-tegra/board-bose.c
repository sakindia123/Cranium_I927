/*
/ * arch/arm/mach-tegra/board-n1.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
 

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/tegra_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/memblock.h>
#include <linux/i2c/mcs.h>
#include <linux/power_supply.h>
#include <linux/sec_jack.h>
#include <linux/mfd/max8907c.h>
#include <linux/fsa9480.h>
#include <linux/regulator/consumer.h>
#include <linux/mhl-sii9234.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/syscalls.h>
#include <linux/vfs.h>
#include <linux/file.h>
#include <linux/console.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/spdif.h>
#include <mach/audio.h>

#include <media/tegra_camera.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/tegra_das.h>
#include <mach/gpio-bose.h>
#include <linux/atmel_mxt224.h>
#include <linux/i2c/mxt224_bose.h>
#include <mach/sec_battery.h>
#include <linux/power/max8922-charger.h>
#include <linux/mfd/max8907c.h>

#include "board.h"
#include "clock.h"
#include "board-n1.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t2.h"
#if defined(CONFIG_VIDEO_S5K6AAFX)
#include <media/s5k6aafx.h>
#endif
#if defined(CONFIG_VIDEO_S5K4ECGX)
#include <media/s5k4ecgx.h>
#endif
#if defined(CONFIG_VIDEO_M5MO)
#include <media/m5mo.h>
#endif
#include <linux/nct1008.h>
#ifdef CONFIG_SAMSUNG_LPM_MODE
#include <linux/moduleparam.h>
#endif
#include <linux/kernel_sec_common.h>

#define HWREV_FOR_EXTERNEL_CHARGER	7
#define HWREV_FOR_CAMERA 9 //Cahnge MEGA CORE 1.2V LDO4 -> GPIO

/* WLAN STATIC BUF Implementation */
#define WLAN_STATIC_BUF	// this feature for using static buffer on wifi driver /Kernel/drivers/net/wireless/bcm4330
#ifdef WLAN_STATIC_BUF

#include <linux/skbuff.h>

#ifdef CONFIG_USB_ANDROID_ACCESSORY
#include <linux/usb/f_accessory.h>
#endif

#ifdef CONFIG_SAMSUNG_LPM_MODE
int charging_mode_from_boot;

/* Get charging_mode status from kernel CMDLINE parameter. */
__module_param_call("", lpm_boot,  &param_ops_int,
		&charging_mode_from_boot, 0, 0644);
MODULE_PARM_DESC(charging_mode_from_boot, "Charging mode parameter value.");
#endif

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24


#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
static struct mxt_callbacks *inform_charger_callbacks;

struct wifi_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wifi_mem_prealloc wifi_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}
EXPORT_SYMBOL(wlan_mem_prealloc);

#define DHD_SKB_HDRSIZE 		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

static int __init init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}
	
	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}
	
	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wifi_mem_array[i].mem_ptr =
				kmalloc(wifi_mem_array[i].size, GFP_KERNEL);

		if (!wifi_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

#endif /* WLAN_STATIC_BUF */


enum cable_type_t set_cable_status;
EXPORT_SYMBOL(set_cable_status);

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct board_usb_data {
	struct mutex ldo_en_lock;
	int usb_regulator_on[3];
};

static struct board_usb_data usb_data;
struct otg_id_open_data otg_open;
struct otg_detect_data otg_clk_data;
static unsigned int *sec_batt_level;
EXPORT_SYMBOL(otg_open);			// 20110517_HDLNC_PMIC
extern int __init register_smd_resource(void);


struct bootloader_message {
	char command[32];
	char status[32];
};

/* REBOOT_MODE */
#define REBOOT_MODE_NONE                0
#define REBOOT_MODE_DOWNLOAD            1
#define REBOOT_MODE_NORMAL              2
#define REBOOT_MODE_CHARGING            3
#define REBOOT_MODE_RECOVERY            4
#define REBOOT_MODE_FOTA                5
#define REBOOT_MODE_FASTBOOT            7

#define MISC_DEVICE "/dev/block/mmcblk0p5"

static int write_bootloader_message(char *cmd, int mode)
{
	int fd;
	struct file *filp;
	mm_segment_t oldfs;
	int ret = 0;
	loff_t pos = 2048L;  /* bootloader message offset in MISC.*/

	struct bootloader_message  bootmsg;

	memset(&bootmsg, 0, sizeof(struct bootloader_message));

	if (mode == REBOOT_MODE_RECOVERY) {
		strcpy(bootmsg.command, "boot-recovery");
		kernel_sec_set_path(SEC_PORT_USB, SEC_PORT_PATH_AP);
		kernel_sec_set_path(SEC_PORT_UART, SEC_PORT_PATH_CP);		
	}
	else if (mode == REBOOT_MODE_FASTBOOT)
		strcpy(bootmsg.command, "boot-fastboot");
	else if (mode == REBOOT_MODE_NORMAL)
		strcpy(bootmsg.command, "boot-reboot");
	else if (mode == REBOOT_MODE_NONE)
		strcpy(bootmsg.command, "boot-normal");
	else if (mode == REBOOT_MODE_FOTA)
		strcpy(bootmsg.command, "boot-fota");	
	else
		strcpy(bootmsg.command, cmd);

	bootmsg.status[0] = (char) mode;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(MISC_DEVICE, O_WRONLY, 0);

	if (fd < 0) {
		printk(KERN_INFO "failed to open MISC : '%s'. %d\n", MISC_DEVICE, fd);
		return 0;
	}

	filp = fget(fd);

	ret = vfs_write(filp, (const char *)&bootmsg, sizeof(struct bootloader_message), &pos);

	if (ret < 0)
		printk(KERN_INFO "failed to write on MISC \n");
	else
		printk(KERN_INFO "command : %s written on MISC\n", bootmsg.command);

	fput(filp);
	filp_close(filp, NULL);

	set_fs(oldfs);

	return ret;
}

/* Boot Mode Physical Addresses and Magic Token */
#define BOOT_MODE_P_ADDR	(0x20000000 - 0x0C)
#define BOOT_MAGIC_P_ADDR	(0x20000000 - 0x10)
#define BOOT_MAGIC_TOKEN	0x626F6F74

static void write_bootloader_mode(char boot_mode)
{
	void __iomem *to_io;

	to_io = ioremap(BOOT_MODE_P_ADDR, 4);
	writel((unsigned long)boot_mode, to_io);
	iounmap(to_io);

	/* Write a magic value to a 2nd memory location to distinguish between a
	 * cold boot and a reboot.
	 */
	to_io = ioremap(BOOT_MAGIC_P_ADDR, 4);
	writel(BOOT_MAGIC_TOKEN, to_io);
	iounmap(to_io);
}

static int n1_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode;

	if (code == SYS_RESTART) {
		mode = REBOOT_MODE_NORMAL;
		if (_cmd) {
			if (!strcmp((char *)_cmd, "recovery"))
				mode = REBOOT_MODE_RECOVERY;
			else if (!strcmp((char *)_cmd, "arm11_fota"))
				mode = REBOOT_MODE_FOTA;			
			else if (!strcmp((char *)_cmd, "bootloader"))
				mode = REBOOT_MODE_FASTBOOT;
			else if (!strcmp((char *)_cmd, "download"))
				mode = REBOOT_MODE_DOWNLOAD;
		}
	} else
		mode = REBOOT_MODE_NONE;

	pr_info("%s, Reboot Mode : %d \n", __func__, mode);

	switch( mode )
	{
		case REBOOT_MODE_FOTA:
		case REBOOT_MODE_NORMAL:
		case REBOOT_MODE_RECOVERY:
			write_bootloader_mode(mode);
	}

	write_bootloader_message(_cmd, mode);

	return NOTIFY_DONE;
}

static struct notifier_block n1_reboot_notifier = {
	.notifier_call = n1_notifier_call,
};

static struct plat_serial8250_port debug_uart_platform_data[] = {
	{
		.membase	= IO_ADDRESS(TEGRA_UARTB_BASE),
		.mapbase	= TEGRA_UARTB_BASE,
		.irq		= INT_UARTB,
		.flags		= UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.type		= PORT_TEGRA,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= 216000000,
	}, {
		.flags		= 0,
	}
};

static struct platform_device debug_uart = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = debug_uart_platform_data,
	},
};

static struct tegra_audio_platform_data tegra_spdif_pdata = {
	.dma_on = true,  /* use dma by default */
	.spdif_clk_rate = 5644800,
};

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};
static struct tegra_ulpi_config hsic_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "clk_dev2",
	.inf_type = TEGRA_USB_UHSIC,
};

static struct resource ram_console_resource[] = {
	{
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};


static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "clk_dev2",
};

#if defined (CONFIG_MACH_BOSE_ATT)
static struct platform_device hall_device_slider = {
	.name		  = "hall",
	.id		  = -1,
};
#endif

/* Bluetooth(BCM4330) rfkill */
static struct resource n1_bcm4330_rfkill_resources[] = {
	{
		.name   = "bcm4330_nreset_gpio",
		.start  = GPIO_BT_nRST,
		.end    = GPIO_BT_nRST,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4330_nshutdown_gpio",
		.start  = GPIO_BT_EN,
		.end    = GPIO_BT_EN,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4330_btwake_gpio",
		.start  = GPIO_BT_WAKE,
		.end    = GPIO_BT_WAKE,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4330_hostwake_gpio",
		.start  = TEGRA_GPIO_TO_IRQ(GPIO_BT_HOST_WAKE),
		.end    = TEGRA_GPIO_TO_IRQ(GPIO_BT_HOST_WAKE),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device n1_bcm4330_rfkill_device = {
	.name = "bcm4330_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(n1_bcm4330_rfkill_resources),
	.resource       = n1_bcm4330_rfkill_resources,
};

int __init n1_rfkill_init(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4330_32k_clk", n1_bcm4330_rfkill_device.name, \
				"blink", NULL);

	tegra_gpio_enable(GPIO_BT_HOST_WAKE);
	platform_device_register(&n1_bcm4330_rfkill_device);

	return 0;
}

static __initdata struct tegra_clk_init_table n1_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uartb",	"pll_p",	216000000,	true},
	{ "uartc",	"pll_m",	600000000,	false},
	{ "blink",	"clk_32k",	32768,		false},
	{ "pll_p_out4",	"pll_p",	24000000,	true},
	{ "pwm",	"clk_32k",	32768,		false},
	{ "pll_a",	NULL,		11289600,	true},
	{ "pll_a_out0",	NULL,		11289600,	true},
	{ "i2s1",	"pll_a_out0",	11289600,	true},
	{ "i2s2",	"pll_a_out0",	11289600,	true},
	{ "audio",	"pll_a_out0",	11289600,	true},
	{ "audio_2x",	"audio",	22579200,	true},
	{ "spdif_out",	"pll_a_out0",	5644800,	false},
	{ "kbc",	"clk_32k",	32768,		true},
	{ "disp1",	"pll_p",	216000000,		true},
	{ NULL,		NULL,		0,		0},
};

#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* woojin80.kim : Define samsung product id and config string.
 *                Sources such as 'android.c' and 'devs.c' refered below define
 */
#define SAMSUNG_VENDOR_ID		0x04e8


#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE
	/* USE DEVGURU HOST DRIVER */
	/* 0x6860 : MTP(0) + MS Composite (UMS) */
	/* 0x685E : UMS(0) + MS Composite (ADB) */

	#ifdef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
		#define SAMSUNG_KIES_PRODUCT_ID	0x685e	/* ums(0) + acm(1,2) */
	#else
		#define SAMSUNG_KIES_PRODUCT_ID	0x6860	/* mtp(0) + acm(1,2) */
	#endif


	#define SAMSUNG_DEBUG_PRODUCT_ID	0x685e	/* ums(0) + acm(1,2) + adb(3) (with MS Composite) */
	#define SAMSUNG_UMS_PRODUCT_ID	0x685B  /* UMS Only */
	#define SAMSUNG_MTP_PRODUCT_ID	0x685C  /* MTP Only */


	#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
		#define SAMSUNG_RNDIS_PRODUCT_ID	0x6861  /* RNDIS(0,1) + UMS (2) + MS Composite */
	#else
		#define SAMSUNG_RNDIS_PRODUCT_ID	0x6863  /* RNDIS only */
	#endif


	#define ANDROID_DEBUG_CONFIG_STRING	 "UMS + ACM + ADB (Debugging mode)"


	#ifdef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
		#define ANDROID_KIES_CONFIG_STRING	 "UMS + ACM (SAMSUNG KIES mode)"
	#else
		#define ANDROID_KIES_CONFIG_STRING	 "MTP + ACM (SAMSUNG KIES mode)"
	#endif


#else /* USE MCCI HOST DRIVER */
	#define SAMSUNG_KIES_PRODUCT_ID	0x6877	/* Shrewbury ACM+MTP*/
	#define SAMSUNG_DEBUG_PRODUCT_ID	0x681C	/* Shrewbury ACM+UMS+ADB*/
	#define SAMSUNG_UMS_PRODUCT_ID	0x681D
	#define SAMSUNG_MTP_PRODUCT_ID	0x68A9
	#define SAMSUNG_RNDIS_PRODUCT_ID	0x6863
	#define ANDROID_DEBUG_CONFIG_STRING	 "ACM + UMS + ADB (Debugging mode)"
	#define ANDROID_KIES_CONFIG_STRING	 "ACM + MTP (SAMSUNG KIES mode)"

#endif /* CONFIG_USB_ANDROID_SAMSUNG_ESCAPE */



#define       ANDROID_UMS_CONFIG_STRING	 "UMS Only (Not debugging mode)"
#define       ANDROID_MTP_CONFIG_STRING	 "MTP Only (Not debugging mode)"


#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
	#define       ANDROID_RNDIS_CONFIG_STRING	 "RNDIS + UMS (Not debugging mode)"
#else
	#define       ANDROID_RNDIS_CONFIG_STRING	 "RNDIS Only (Not debugging mode)"
#endif



/* Refered from S1, P1 */
#define USBSTATUS_UMS				0x0
#define USBSTATUS_SAMSUNG_KIES 		0x1
#define USBSTATUS_MTPONLY			0x2
#define USBSTATUS_ASKON				0x4
#define USBSTATUS_VTP				0x8
#define USBSTATUS_ADB				0x10
#define USBSTATUS_DM				0x20
#define USBSTATUS_ACM				0x40
#define USBSTATUS_SAMSUNG_KIES_REAL	0x80


/* Android USB OTG Gadget */
#include <linux/usb/android_composite.h>
#define S3C_VENDOR_ID		0x18d1
#define S3C_PRODUCT_ID		0x0001
#define S3C_ADB_PRODUCT_ID	0x0005

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};


#ifdef CONFIG_USB_ANDROID_RNDIS
	static char *usb_functions_rndis[] = {"rndis",};
#endif


#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* Do not use below compoiste */
#else
	static char *usb_functions_ums_adb[] = {"usb_mass_storage", "adb",};
#endif



#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE /* USE DEVGURU HOST DRIVER */
/* kies mode : using MS Composite*/
		#ifdef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
			static char *usb_functions_ums_acm[] = {"usb_mass_storage",	"acm",};
		#else
			static char *usb_functions_mtp_acm[] = {"mtp", "acm",};
		#endif

		/* debug mode : using MS Composite*/
		static char *usb_functions_ums_acm_adb[] = {"usb_mass_storage", "acm", "adb",};

	#else /* USE MCCI HOST DRIVER */
		/* kies mode */
		static char *usb_functions_acm_mtp[] = {"acm", "mtp",};
		/* debug mode */
		static char *usb_functions_acm_ums_adb[] = {"acm", "usb_mass_storage", "adb",};
	#endif
	
	/* mtp only mode */
	static char *usb_functions_mtp[] = {"mtp",};
#ifdef CONFIG_USB_ANDROID_ACCESSORY
/* accessory mode */
static char *usb_functions_accessory[] = {
	"accessory",
};
static char *usb_functions_accessory_adb[] = {
	"accessory",
	"adb",
};
#endif
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE /* USE DEVGURU HOST DRIVER */
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif	
		"usb_mass_storage",	"acm", "adb",
		#ifdef CONFIG_USB_ANDROID_RNDIS
			"rndis",
		#endif
		#ifndef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
			"mtp",
		#endif
	#else /* USE MCCI HOST DRIVER */
		"acm", "usb_mass_storage", "adb",
		#ifdef CONFIG_USB_ANDROID_RNDIS
			"rndis",
		#endif

		"mtp",
	#endif

#else /* original */
	#ifdef CONFIG_USB_ANDROID_RNDIS
		"rndis",
	#endif
	"usb_mass_storage",	"adb",
	#ifdef CONFIG_USB_ANDROID_ACM
		"acm",
	#endif
#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */
};


static struct android_usb_product usb_products[] = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE /* USE DEVGURU HOST DRIVER */
		#ifdef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
			{
				.product_id	= SAMSUNG_DEBUG_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_ums_acm_adb),
				.functions	= usb_functions_ums_acm_adb,
				.bDeviceClass	= 0xEF,
				.bDeviceSubClass= 0x02,
				.bDeviceProtocol= 0x01,
				.s		= ANDROID_DEBUG_CONFIG_STRING,
				.mode		= USBSTATUS_ADB,
			},
			{
				.product_id	= SAMSUNG_KIES_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_ums_acm),
				.functions	= usb_functions_ums_acm,
				.bDeviceClass	= 0xEF,
				.bDeviceSubClass= 0x02,
				.bDeviceProtocol= 0x01,
				.s		= ANDROID_KIES_CONFIG_STRING,
				.mode		= USBSTATUS_SAMSUNG_KIES,
			},
			{
				.product_id	= SAMSUNG_UMS_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_ums),
				.functions	= usb_functions_ums,
				.bDeviceClass	= USB_CLASS_PER_INTERFACE,
				.bDeviceSubClass= 0,
				.bDeviceProtocol= 0,
				.s		= ANDROID_UMS_CONFIG_STRING,
				.mode		= USBSTATUS_UMS,
			},
			
			#ifdef CONFIG_USB_ANDROID_RNDIS
				{
					.product_id	= SAMSUNG_RNDIS_PRODUCT_ID,
					.num_functions	= ARRAY_SIZE(usb_functions_rndis),
					.functions	= usb_functions_rndis,
					
				#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
					.bDeviceClass	= 0xEF,
					.bDeviceSubClass= 0x02,
					.bDeviceProtocol= 0x01,
				#else
					#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
						.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
					#else
						.bDeviceClass	= USB_CLASS_COMM,
					#endif
					.bDeviceSubClass= 0,
					.bDeviceProtocol= 0,
				#endif
					.s		= ANDROID_RNDIS_CONFIG_STRING,
					.mode		= USBSTATUS_VTP,
				},
			#endif
			
		#else /* Not used KIES_UMS */
			{
				.product_id	= SAMSUNG_DEBUG_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_ums_acm_adb),
				.functions	= usb_functions_ums_acm_adb,
				.bDeviceClass	= 0xEF,
				.bDeviceSubClass= 0x02,
				.bDeviceProtocol= 0x01,
				.s		= ANDROID_DEBUG_CONFIG_STRING,
				.mode		= USBSTATUS_ADB,
			},
			{
				.product_id	= SAMSUNG_KIES_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_mtp_acm),
				.functions	= usb_functions_mtp_acm,
				.bDeviceClass	= 0xEF,
				.bDeviceSubClass= 0x02,
				.bDeviceProtocol= 0x01,
				.s		= ANDROID_KIES_CONFIG_STRING,
				.mode		= USBSTATUS_SAMSUNG_KIES,
				.multi_conf_functions[0] = usb_functions_mtp,
				.multi_conf_functions[1] = usb_functions_mtp_acm,					
			},
			{
				.product_id	= SAMSUNG_UMS_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_ums),
				.functions	= usb_functions_ums,
				.bDeviceClass	= USB_CLASS_PER_INTERFACE,
				.bDeviceSubClass= 0,
				.bDeviceProtocol= 0,
				.s		= ANDROID_UMS_CONFIG_STRING,
				.mode		= USBSTATUS_UMS,
			},
#ifdef CONFIG_USB_ANDROID_ACCESSORY	
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory),
		.functions	= usb_functions_accessory,
		.bDeviceClass	= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_ACCESSORY_CONFIG_STRING,
		.mode		= USBSTATUS_ACCESSORY,
	},	
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_adb),
		.functions	= usb_functions_accessory_adb,
		.bDeviceClass	= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_ACCESSORY_ADB_CONFIG_STRING,
		.mode		= USBSTATUS_ACCESSORY,
	},		
#endif	

			#ifdef CONFIG_USB_ANDROID_RNDIS
			{
				.product_id	= SAMSUNG_RNDIS_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_rndis),
				.functions	= usb_functions_rndis,
				#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
					.bDeviceClass	= 0xEF,
					.bDeviceSubClass= 0x02,
					.bDeviceProtocol= 0x01,
				#else
					#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
						.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
					#else
						.bDeviceClass	= USB_CLASS_COMM,
					#endif
					.bDeviceSubClass= 0,
					.bDeviceProtocol= 0,
				#endif
				.s		= ANDROID_RNDIS_CONFIG_STRING,
				.mode		= USBSTATUS_VTP,
			},
			#endif
			
			{
				.product_id	= SAMSUNG_MTP_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_mtp),
				.functions	= usb_functions_mtp,
				.bDeviceClass	= USB_CLASS_PER_INTERFACE,
				.bDeviceSubClass= 0,
				.bDeviceProtocol= 0x01,
				.s		= ANDROID_MTP_CONFIG_STRING,
				.mode		= USBSTATUS_MTPONLY,
			},
		#endif
		
	#else  /* USE MCCI HOST DRIVER */
		{
			.product_id	= SAMSUNG_DEBUG_PRODUCT_ID, /* change sequence */
			.num_functions	= ARRAY_SIZE(usb_functions_acm_ums_adb),
			.functions	= usb_functions_acm_ums_adb,
			.bDeviceClass	= USB_CLASS_COMM,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_DEBUG_CONFIG_STRING,
			.mode		= USBSTATUS_ADB,
		},
		{
			.product_id	= SAMSUNG_KIES_PRODUCT_ID, /* change sequence */
			.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp),
			.functions	= usb_functions_acm_mtp,
			.bDeviceClass	= USB_CLASS_COMM,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_KIES_CONFIG_STRING,
			.mode		= USBSTATUS_SAMSUNG_KIES,

		},
		{
			.product_id	= SAMSUNG_UMS_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_ums),
			.functions	= usb_functions_ums,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_UMS_CONFIG_STRING,
			.mode		= USBSTATUS_UMS,
		},
			#ifdef CONFIG_USB_ANDROID_RNDIS
			{
				.product_id	= SAMSUNG_RNDIS_PRODUCT_ID,
				.num_functions	= ARRAY_SIZE(usb_functions_rndis),
				.functions	= usb_functions_rndis,
				#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
					.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
				#else
					.bDeviceClass	= USB_CLASS_COMM,
				#endif
				.bDeviceSubClass= 0,
				.bDeviceProtocol= 0,
				.s		= ANDROID_RNDIS_CONFIG_STRING,
				.mode		= USBSTATUS_VTP,
			},
			#endif
		{
			.product_id	= SAMSUNG_MTP_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_mtp),
			.functions	= usb_functions_mtp,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0x01,
			.s		= ANDROID_MTP_CONFIG_STRING,
			.mode		= USBSTATUS_MTPONLY,
		},
	#endif
#else /* original */
	{
		.product_id	= S3C_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= S3C_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
#endif
};


/* standard android USB platform data */

// Information should be changed as real product for commercial release
static struct android_usb_platform_data android_usb_pdata = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	.vendor_id		= SAMSUNG_VENDOR_ID,
	.product_id		= SAMSUNG_KIES_PRODUCT_ID,
	.manufacturer_name	= "SAMSUNG",
	.product_name		= "SAMSUNG_Android",
#else
	.vendor_id		= S3C_VENDOR_ID,
	.product_id		= S3C_PRODUCT_ID,
	.manufacturer_name	= "Android",//"Samsung",
	.product_name		= "Android",//"Samsung SMDKV210",
#endif
	.serial_number		= NULL,
	.num_products 		= ARRAY_SIZE(usb_products),
	.products 		= usb_products,
	.num_functions 		= ARRAY_SIZE(usb_functions_all),
	.functions 		= usb_functions_all,
};

struct platform_device s3c_device_android_usb = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &android_usb_pdata,
	},
};


static struct usb_mass_storage_platform_data ums_pdata = {
#ifdef CONFIG_TARGET_LOCALE_KOR
	.vendor			= "SAMSUNG",
	.product		= "SHW-M250S",
#else
	.vendor			= "Android   ",//"Samsung",
	.product		= "UMS Composite",//"SMDKV210",
#endif
	.release		= 1,
	.nluns			= 2,
};
struct platform_device s3c_device_usb_mass_storage= {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &ums_pdata,
	},
};


#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
/* ethaddr is filled by board_serialno_setup */
	.vendorID       = SAMSUNG_VENDOR_ID,
	.vendorDescr    = "Samsung",
	.ethaddr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
};
struct platform_device s3c_device_rndis= {
	.name   = "rndis",
	.id     = -1,
	.dev    = {
		.platform_data = &rndis_pdata,
	},
};
#endif


#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */



static void sec_jack_set_micbias_state(bool on)
{
	printk(KERN_INFO "Board-n1 : %s (on %d)\n", __func__, on);
	wm8994_set_sub_mic_bias(on);
	/* to be developed */
}


static struct tegra_ulpi_config n1_ehci2_ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "clk_dev2",
};

static struct tegra_ehci_platform_data n1_ehci2_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 0,
	.phy_config = &n1_ehci2_ulpi_phy_config,
};

static struct tegra_i2c_platform_data n1_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data n1_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 400000, 400000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
};

static struct tegra_i2c_platform_data n1_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
};

static struct tegra_i2c_platform_data n1_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_dvc		= true,
};

static struct tegra_audio_platform_data tegra_audio_pdata[] = {
	/* For I2S1 */
	[0] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 44100,
		.i2s_clk_rate	= 2822400,
		.dap_clk	= "clk_dev1",
		.audio_sync_clk = "audio_2x",
		.mode		= I2S_BIT_FORMAT_I2S,
		.fifo_fmt	= I2S_FIFO_PACKED,
		.bit_size	= I2S_BIT_SIZE_16,
		.i2s_bus_width = 32,
		.dsp_bus_width = 16,
	},
	/* For I2S2 */
	[1] = {
		.i2s_master	= true,
		.dma_on		= true,  /* use dma by default */
		.i2s_master_clk = 8000,
		.dsp_master_clk = 8000,
		.i2s_clk_rate	= 2000000,
		.dap_clk	= "clk_dev1",
		.audio_sync_clk = "audio_2x",
		.mode		= I2S_BIT_FORMAT_DSP,
		.fifo_fmt	= I2S_FIFO_16_LSB,
		.bit_size	= I2S_BIT_SIZE_16,
		.i2s_bus_width = 32,
		.dsp_bus_width = 16,
	}
};

static struct tegra_das_platform_data tegra_das_pdata = {
	.tegra_dap_port_info_table = {
		[0] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		/* I2S1 <--> DAC1 <--> DAP1 <--> Hifi Codec */
		[1] = {
			.dac_port = tegra_das_port_i2s1,
			.codec_type = tegra_audio_codec_type_hifi,
			.device_property = {
				.num_channels = 2,
				.bits_per_sample = 16,
				.rate = 44100,
				.dac_dap_data_comm_format = dac_dap_data_format_i2s,
			},
		},
		[2] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		[3] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
		[4] = {
			.dac_port = tegra_das_port_none,
			.codec_type = tegra_audio_codec_type_none,
			.device_property = {
				.num_channels = 0,
				.bits_per_sample = 0,
				.rate = 0,
				.dac_dap_data_comm_format = 0,
			},
		},
	},

	.tegra_das_con_table = {
		[0] = {
			.con_id = tegra_das_port_con_id_hifi,
			.num_entries = 4,
			.con_line = {
				[0] = {tegra_das_port_i2s1, tegra_das_port_dap1, true},
				[1] = {tegra_das_port_dap1, tegra_das_port_i2s1, false},
				[2] = {tegra_das_port_i2s2, tegra_das_port_dap4, true},
				[3] = {tegra_das_port_dap4, tegra_das_port_i2s2, false},
			},
		},
	}
};

#ifdef CONFIG_MHL_SII9234
#define GPIO_INPUT	0
#define GPIO_OUTPUT	1
#define GPIO_LEVEL_LOW	0
#define GPIO_LEVEL_HIGH	1
#define GPIO_LEVEL_NONE	2

static unsigned int mhl_gpio_table[][3] = {
	{GPIO_MHL_SEL, GPIO_OUTPUT, GPIO_LEVEL_LOW},
	{GPIO_MHL_RST, GPIO_OUTPUT, GPIO_LEVEL_HIGH},
	{GPIO_MHL_INT, GPIO_INPUT, GPIO_LEVEL_NONE},
	{GPIO_MHL_WAKE_UP, GPIO_INPUT, GPIO_LEVEL_NONE},
	{GPIO_HDMI_LDO_EN, GPIO_OUTPUT, GPIO_LEVEL_LOW},
};

const char *mhl_gpio_name[] = {
	"mhl_sel",
	"mhl_rst",
	"mhl_int",
	"mhl_wake_up",
	"hdmi_ldo_en"
};

static void n1_mhl_gpio_init(void)
{
	int array_size = ARRAY_SIZE(mhl_gpio_table);
	int i;
	unsigned int gpio;
	int ret;

	for (i = 0; i < array_size; i++) {
		gpio = mhl_gpio_table[i][0];
		ret = gpio_request(gpio, mhl_gpio_name[i]);
		if (ret){
			printk(KERN_ERR "Failed to request gpio [%s]\n",
						mhl_gpio_name[i]);
			continue;
		}
		tegra_gpio_enable(gpio);
		if (mhl_gpio_table[i][1] == GPIO_OUTPUT)
			gpio_direction_output(gpio, mhl_gpio_table[i][2]);
		else
			gpio_direction_input(gpio);
	}
}


static struct regulator *n1_mhl_33v_reg = NULL;
static struct regulator *n1_mhl_18v_reg = NULL;
extern	void Sii9234_int_irq_disable();

static int sii9234_power_onoff(int onoff)
{
	if (onoff) {
		printk("[mhl] power on\n");
		gpio_set_value(GPIO_MHL_RST, 1);
		if (!n1_mhl_33v_reg) { /* LDO5 */
			n1_mhl_33v_reg = regulator_get(NULL, "VCC_3V3_MHL");
			if (IS_ERR_OR_NULL(n1_mhl_33v_reg)) {
				pr_err("mhl: failed to get reg VCC_3V3_MHL\n");
				n1_mhl_33v_reg = NULL;
				return PTR_ERR(n1_mhl_33v_reg);
			}
		}
		regulator_enable(n1_mhl_33v_reg);
		if (!n1_mhl_18v_reg) { /* LDO7 */
			n1_mhl_18v_reg = regulator_get(NULL, "VCC_1V8_MHL");
			if (IS_ERR_OR_NULL(n1_mhl_18v_reg)) {
				pr_err("mhl: failed to get reg VCC_1V8_MHL\n");
				n1_mhl_18v_reg = NULL;
				regulator_disable(n1_mhl_33v_reg);
				n1_mhl_33v_reg = NULL;
				return PTR_ERR(n1_mhl_18v_reg);
			}
		}
		regulator_enable(n1_mhl_18v_reg);
		msleep(10);
		gpio_set_value(GPIO_HDMI_LDO_EN, 1);
		msleep(5);
		gpio_set_value(GPIO_MHL_RST, 0);
		msleep(10);
		gpio_set_value(GPIO_MHL_RST, 1);
	} else {
		printk("[mhl] power off\n");

		Sii9234_int_irq_disable();
		
		gpio_set_value(GPIO_HDMI_LDO_EN, 0);
		if (n1_mhl_33v_reg != NULL)
			regulator_disable(n1_mhl_33v_reg);
		if (n1_mhl_18v_reg != NULL)
			regulator_disable(n1_mhl_18v_reg);
	}
	return 0;
}

#if defined(CONFIG_MHL_SII9234)
static int sii_powersate=0;

void sii9234_cfg_power(int onoff)
	 
{
	printk("<0> cfg power");//gaurav
	if (sii_powersate!=onoff)
	{
		sii9234_power_onoff(onoff);
	}
	else
		printk("Already %s", onoff?"ON":"OFF");

}

int sii9234_switch_onoff(bool onoff)
{
	sii_powersate = onoff;
	if (onoff) {
		gpio_set_value(GPIO_MHL_SEL, 1);
		printk(KERN_ERR "mhl switch on\n");
	} else {
		gpio_set_value(GPIO_MHL_SEL, 0);
		printk(KERN_ERR "mhl switch off\n");
	}
	return 0;
}

#else
static int sii9234_switch_onoff(int onoff)
{
	if (onoff) {
		gpio_set_value(GPIO_MHL_SEL, 1);
		printk(KERN_ERR "mhl switch on\n");
	} else {
		gpio_set_value(GPIO_MHL_SEL, 0);
		printk(KERN_ERR "mhl switch off\n");
	}
	return 0;
}
#endif

struct mhl_platform_data mhl_pdata = {
	.mhl_sel	= GPIO_MHL_SEL,
	.mhl_rst	= GPIO_MHL_RST,
	.mhl_int	= GPIO_MHL_INT,
	.mhl_wake_up	= GPIO_MHL_WAKE_UP,
	.power_onoff	= sii9234_power_onoff,
#ifdef CONFIG_MHL_SWITCH
	.switch_onoff	= sii9234_switch_onoff,
#endif
};

struct platform_device n1_mhl_device = {
		.name	= "mhl",
		.id	= -1,
		.dev	={
			.platform_data = &mhl_pdata,
		}
};
#endif

static void n1_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &n1_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &n1_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &n1_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &n1_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);
}

#define TEGRA_GPIO_NULL		255

#define GPIO_BANK(x)		((x) >> 5)
#define GPIO_PORT(x)		(((x) >> 3) & 0x3)
#define GPIO_BIT(x)		((x) & 0x7)

#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x80 +		\
				 GPIO_PORT(x) * 4)
#define GPIO_IN(x)		(GPIO_REG(x) + 0x30)

static int n1_tegra_gpio_get(unsigned offset)
{
	return (__raw_readl(GPIO_IN(offset)) >> GPIO_BIT(offset)) & 0x1;
}

#define GPIO_KEY(_id, _gpio, _iswake, _act_low)		\
	{					\
		.code = _id,			\
		.gpio = _gpio,	\
		.active_low = _act_low,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button n1_keys[] = {
#if defined (CONFIG_MACH_BOSE_ATT)
	[0] = GPIO_KEY(KEY_POWER, GPIO_EXT_WAKEUP, 1, 0),
	[1] = GPIO_KEY(KEY_VOLUMEUP, GPIO_VOL_UP, 0, 1),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, GPIO_VOL_DOWN, 0, 1),
#else
	[0] = GPIO_KEY(KEY_HOME, GPIO_HOME_KEY, 1, 1),
	[1] = GPIO_KEY(KEY_POWER, GPIO_EXT_WAKEUP, 1, 0),
	[2] = GPIO_KEY(KEY_VOLUMEUP, GPIO_VOL_UP, 0, 1),
	[3] = GPIO_KEY(KEY_VOLUMEDOWN, GPIO_VOL_DOWN, 0, 1),
#endif
};

static bool gpio_keys_wakeup;
static int gpio_keys_lp_state;
#if defined (CONFIG_MACH_BOSE_ATT)
/* index of n1_keys array */
static int gpio_keys_wake_lp2[] = {0, 1, 2};
#else
/* index of n1_keys array */
static int gpio_keys_wake_lp2[] = {0, 1, 2, 3};
#endif
/* save gpio state from lp2 state */
static int gpio_keys_wake_state[ARRAY_SIZE(gpio_keys_wake_lp2)];
/* save wakeup status register */
static u32 gpio_keys_wakeup_status;
static int gpio_keys_wake_state2[ARRAY_SIZE(gpio_keys_wake_lp2)];

#define PMC_WAKE_STATUS		0x14

void n1_save_wakeup_key(int lp_state)
{
	int state;
	int i;
	int idx;

	gpio_keys_wakeup = true;
	gpio_keys_lp_state = lp_state;
	
	
	if (lp_state == 2 || lp_state == 1) {
		/* save gpio state */
		for (i = 0; i < ARRAY_SIZE(gpio_keys_wake_lp2); i++) {
			idx = gpio_keys_wake_lp2[i];
			state = (n1_tegra_gpio_get(n1_keys[idx].gpio) ? 1 : 0) ^ n1_keys[idx].active_low;
			pr_debug("[%s] %d:%d\n", __func__, n1_keys[idx].code, !!state);
			gpio_keys_wake_state[i] = state;
			gpio_keys_wake_state2[i] = 0;
		}
	} else {
		gpio_keys_wakeup_status =
			readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	}
}

/* caution: this function has some hard-coded value */
/* if key environment is modifed, this function has to be re-written */
void n1_check_key_pressed(void)
{
	int i;
	int state;
	
	if (gpio_keys_wakeup && (gpio_keys_lp_state == 2 || gpio_keys_lp_state == 1)) {
		/* volume up key */
#if defined(CONFIG_MACH_BOSE_ATT)
		i = 1;
#else
		i = 2;
#endif
		if (gpio_keys_wake_state[i] && !gpio_keys_wake_state2[i]) {
			state  = !n1_tegra_gpio_get(GPIO_VOL_UP);
			if(!state)
				gpio_keys_wake_state2[i] = 1;
			pr_debug("[%s] %d:%d->%d\n", __func__, KEY_VOLUMEUP, state, gpio_keys_wake_state2[i]);
		}
		/* volume down key */
#if defined(CONFIG_MACH_BOSE_ATT)
		i = 2;
#else
		i = 3;
#endif
		if (gpio_keys_wake_state[i] && !gpio_keys_wake_state2[i]) {
			state  = !n1_tegra_gpio_get(GPIO_VOL_DOWN);
			if(!state)
				gpio_keys_wake_state2[i] = 1;
			pr_debug("[%s] %d:%d->%d\n", __func__, KEY_VOLUMEDOWN, state, gpio_keys_wake_state2[i]);
		}
	}
}

/* Only key resume code can use this function */
/* Because this function return correct key code at one time after wakeup */
static int n1_wakeup_key(void)
{
#ifdef CONFIG_MACH_BOSE_ATT
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	if (status & TEGRA_WAKE_GPIO_PU5) {
		writel(TEGRA_WAKE_GPIO_PU5,
			IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
	}

	return status & TEGRA_WAKE_GPIO_PU5 ? KEY_POWER : KEY_RESERVED;
#else
	int i;
	int idx;
	int ret= KEY_RESERVED;
	
	if(!gpio_keys_wakeup)
		ret = 0;
	else {
		if (gpio_keys_lp_state == 2 || gpio_keys_lp_state == 1) {
			/* this function returns only one key code.
			** if key is missed on wakeup, please review this code.
			*/
			for (i = 0; i < ARRAY_SIZE(gpio_keys_wake_lp2); i++) {
				idx = gpio_keys_wake_lp2[i];
				if (gpio_keys_wake_state[i]) {
					ret = n1_keys[idx].code;
					break;
				}
			}
		} else {
			if (gpio_keys_wakeup_status & TEGRA_WAKE_GPIO_PU5) {
				ret = KEY_POWER;
#if !defined(CONFIG_MACH_BOSE_ATT)
			} else if (gpio_keys_wakeup_status & TEGRA_WAKE_GPIO_PO5) {
				ret = KEY_HOME;
#endif
			} else
				ret = KEY_RESERVED;
		}
		pr_debug("[%s] %d wakeup from lp%d\n", __func__, ret, gpio_keys_lp_state);
	}

	gpio_keys_wakeup = false;

	return ret;
#endif
}

/* This can find a volume key pressed twice */
static int n1_wakeup_key_twice(void)
{
	int i;
	int idx;
	int ret= KEY_RESERVED;
	
	for (i = 0; i < ARRAY_SIZE(gpio_keys_wake_lp2); i++) {
		idx = gpio_keys_wake_lp2[i];
		if (gpio_keys_wake_state2[i]) {
			ret = n1_keys[idx].code;
			break;
		}
	}

	return ret;
}

static bool n1_ckech_lpm(void)
{
	return charging_mode_from_boot ? true : false;
}

static struct gpio_keys_platform_data n1_keys_platform_data = {
	.buttons	= n1_keys,
	.nbuttons	= ARRAY_SIZE(n1_keys),
	.wakeup_key	= n1_wakeup_key,
	.wakeup_key_twice	= n1_wakeup_key_twice,
#ifdef CONFIG_SAMSUNG_LPM_MODE
	.check_lpm = n1_ckech_lpm,
#endif
};

static struct platform_device n1_keys_device = {
	.name	= "sec_key",
	.id	= 0,
	.dev	= {
		.platform_data	= &n1_keys_platform_data,
	},
};

#ifdef CONFIG_TEGRA_DYNAMIC_CAMERA_CLK_RATE
#define NUM_OF_CAMERA_CLKS 8 
#define NUM_OF_CAMERA_CLKS_RESTORE 8 
#define NUM_OF_CAMERA_CLK_LEVELS 4
static char *tegra_camera_clk_devs[] = {
		"2d", "3d", 
		"tegra-avp", "epp", "mpe",
		"tegra-avp",
		"tegra-avp",
		"tegra_grhost"
};

static char *tegra_camera_clk_cons[] = {
		"2d", "3d", 
		"vde", NULL, NULL,
		"sclk",
		"emc", 
		"emc"
};

static unsigned long tegra_camera_clk_rates[NUM_OF_CAMERA_CLK_LEVELS][NUM_OF_CAMERA_CLKS] = {
	/* 2d     3d      vde    epp     mpe     avp.sclk avp.emc gr.emc */
	/* take still picture */
	{300*MHZ,300*MHZ,240*MHZ,300*MHZ,240*MHZ,240*MHZ,600*MHZ,600*MHZ},	//nv_patch_20110721
	/* camera preview */
	{200*MHZ,200*MHZ,200*MHZ,200*MHZ,10*MHZ, 120*MHZ,300*MHZ,300*MHZ},
	/* low resolution video recording */
	{200*MHZ,200*MHZ,200*MHZ,200*MHZ,200*MHZ,120*MHZ,300*MHZ,300*MHZ},
	/* high resolition video recording */
	{300*MHZ,300*MHZ,240*MHZ,300*MHZ,240*MHZ,120*MHZ,300*MHZ,600*MHZ}
};

static unsigned long* ventana_get_graphics_clk_freqs(int usecase, int width, int height, int framerate)
{
	int product = width*height*framerate;

	if (usecase == CAMERA_USECASE_CAMERA_CAPTURE)
		return &tegra_camera_clk_rates[0][0];
	else if (usecase == CAMERA_USECASE_PREVIEW)
		return &tegra_camera_clk_rates[1][0];
	else if (usecase == CAMERA_USECASE_VIDEO_RECORD
		&& (product < 1280*720*30))
		return &tegra_camera_clk_rates[2][0];
	return &tegra_camera_clk_rates[3][0];
}

static struct tegra_camera_clk_config tegra_camera_clks = {
	.clk_devs = tegra_camera_clk_devs,
	.clk_cons = tegra_camera_clk_cons,
	.clk_restore_devs = tegra_camera_clk_devs,
	.clk_restore_cons = tegra_camera_clk_cons,
	.n_clks = NUM_OF_CAMERA_CLKS,
	.n_restore_clks = NUM_OF_CAMERA_CLKS_RESTORE,
	.get_graphics_clk_freqs = ventana_get_graphics_clk_freqs,
};
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
	.dev = {
#ifdef CONFIG_TEGRA_DYNAMIC_CAMERA_CLK_RATE
		.platform_data = &tegra_camera_clks,
#endif
	}	

};

static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, 3 pole */
		.adc_high = 0,
		.delay_ms = 0,
		.check_count = 0,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 900, unstable zone, default to 3pole if it stays
		 * in this range for a 200ms (20ms delays, 10 samples)
		 */
		.adc_high = 900,
		.delay_ms = 20,
		.check_count = 10,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/*900 < adc <= 2000, unstable zone, default to 3pole if it
		 * stays in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 2000,
		.delay_ms = 0,
		.check_count = 0,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 2000 < adc <= 3700, 4 pole zone */
		.adc_high = 3700,
		.delay_ms = 0,
		.check_count = 0,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 3700, unstable zone, default to 3pole if it stays
		 * in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=130, stable zone */
		.code		= KEY_MEDIA,
		.adc_low		= 0,
		.adc_high		= 150,
	},
	{
		/* 170 <= adc <= 310, stable zone */
		.code		= KEY_VOLUMEUP,
		.adc_low		= 151,
		.adc_high		= 340,
	},
	{
		/* 360 <= adc <= 710, stable zone */
		.code		= KEY_VOLUMEDOWN,
		.adc_low		= 341,
		.adc_high		= 710,
	},
};

static int sec_jack_get_adc_value(void)
{
	static int mili_volt;
	max8907c_adc_read_aux2(&mili_volt);
	printk(KERN_DEBUG
		"Board N1 : Enterring sec_jack_get_adc_value = %d\n", mili_volt);
	return  mili_volt;
}

#if defined (CONFIG_MACH_BOSE_ATT)
static int adc_data;
static int adc_result;
#define ADC_DATA_ARR_SIZE 6
int sec_BAT_VF_get_adc_value(void)
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;
	

	for (i = 0; i < ADC_DATA_ARR_SIZE; i++) 
	{
		max8907c_adc_read_aux1(&adc_data);

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}
	
	        adc_result = (adc_total - adc_max - adc_min) / (ADC_DATA_ARR_SIZE - 2);
	
	return adc_result;
}
#endif

struct sec_jack_platform_data sec_jack_pdata = {
	.set_micbias_state = sec_jack_set_micbias_state,
	.get_adc_value = sec_jack_get_adc_value,
	.zones = sec_jack_zones,
	.num_zones = ARRAY_SIZE(sec_jack_zones),
	.buttons_zones = sec_jack_buttons_zones,
	.num_buttons_zones = ARRAY_SIZE(sec_jack_buttons_zones),
	.det_gpio = GPIO_DET_3_5,
	.send_end_gpio = GPIO_EAR_SEND_END,
#if defined(CONFIG_MACH_BOSE_ATT)
	.det_active_high = 1,
#else
	.det_active_high = 0,
#endif
	.send_end_active_high = 0,
	.jack_status = SEC_JACK_NO_DEVICE,
};

static struct platform_device sec_device_jack = {
	.name			= "sec_jack",
	.id			= 1, /* will be used also for gpio_event id */
	.dev.platform_data	= &sec_jack_pdata,
};

#ifdef CONFIG_BATTERY_SEC

static void sec_bat_get_init_cable_state(struct power_supply *psy)
{
	union power_supply_propval value;
	/* if there was a cable status change before the charger was
	ready, send this now */
	printk("%s : set_cable_status(%d)\n", __func__, set_cable_status);

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return;
	}
	if (set_cable_status == CABLE_TYPE_NONE) {
		value.intval = POWER_SUPPLY_TYPE_BATTERY;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	} else if (set_cable_status == CABLE_TYPE_USB) {
		value.intval = POWER_SUPPLY_TYPE_USB;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	} else if (set_cable_status == CABLE_TYPE_AC) {
		value.intval = POWER_SUPPLY_TYPE_MAINS;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	} else if (set_cable_status == CABLE_TYPE_DOCK) { /* Dock and Charger (ta or usb) is attached */
		value.intval = POWER_SUPPLY_TYPE_DOCK;
		psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	}
}

extern struct nct1008_temp_callbacks *callbacks;

static int sec_bat_get_temperature(void)
{
	int batt_temp = 25;
	batt_temp = callbacks->get_temperature(callbacks);
	return batt_temp;
}

static void sec_bat_get_level(unsigned int *batt_soc)
{
	sec_batt_level = batt_soc;
}

static struct sec_bat_platform_data sec_bat_pdata = {
	.fuel_gauge_name	= "fuelgauge",
	.charger_name		= "max8907c-charger",
#ifdef CONFIG_MAX8922_CHARGER
	.sub_charger_name	= "max8922-charger",
#endif
	.adc_arr_size			= NULL,
	.adc_table			= NULL,
	.adc_channel			= NULL,
	.get_init_cable_state	= sec_bat_get_init_cable_state,
	.get_temperature		= sec_bat_get_temperature,
	.get_batt_level	        = sec_bat_get_level,
	.get_BAT_VF_adcvalue   = sec_BAT_VF_get_adc_value,
};

static struct platform_device sec_device_battery = {
	.name = "sec-battery",
	.id = -1,
	.dev.platform_data = &sec_bat_pdata,
};
#endif /* CONFIG_BATTERY_SEC */

#ifdef CONFIG_MAX8922_CHARGER
static int max8922_cfg_gpio(struct max8922_platform_data *pdata)
{
	if (system_rev < HWREV_FOR_EXTERNEL_CHARGER)
		return -ENODEV;

	tegra_gpio_enable(pdata->gpio_chg_en);
	gpio_request(pdata->gpio_chg_en, "chg_en");
//20110517_HDLNC_PMIC
#if !defined (CONFIG_MACH_BOSE_ATT)	

	gpio_direction_output(pdata->gpio_chg_en, 0);
#endif

	tegra_gpio_enable(pdata->gpio_chg_ing);
	gpio_request(pdata->gpio_chg_ing, "chg_ing");
	gpio_direction_input(pdata->gpio_chg_ing);

	tegra_gpio_enable(pdata->gpio_ta_nconnected);
	gpio_request(pdata->gpio_ta_nconnected, "ta_nconnected");
	gpio_direction_input(pdata->gpio_ta_nconnected);

	return 0;
}

static int max8922_charger_topoff_cb(void)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if (!psy) {
		pr_err("%s: Max8907c-Charger : Fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	value.intval = POWER_SUPPLY_STATUS_FULL;
	return psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
}

static struct max8922_platform_data max8922_pdata = {
	.topoff_cb = max8922_charger_topoff_cb,
	.cfg_gpio = max8922_cfg_gpio,
	.gpio_chg_en = GPIO_CHG_EN,
	.gpio_chg_ing = GPIO_CHG_ING_N,
	.gpio_ta_nconnected = GPIO_TA_nCONNECTED,
};

static struct platform_device max8922_device_charger = {
	.name = "max8922-charger",
	.id = -1,
	.dev.platform_data = &max8922_pdata,
};
#endif /* CONFIG_MAX8922_CHARGER */

static struct platform_device *n1_devices[] __initdata = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
	&s3c_device_usb_mass_storage,
#endif
	&debug_uart,
	&tegra_uarta_device,
	&tegra_uartc_device,
	&pmu_device,
	&tegra_udc_device,
	/* &tegra_ehci2_device, */
	&tegra_gart_device,
	&n1_keys_device,
#ifdef CONFIG_MHL_SII9234
	&n1_mhl_device,
#endif
	/* &tegra_wdt_device, */
	&tegra_i2s_device1,
	&tegra_spdif_device,
	&tegra_avp_device,
	&tegra_camera,
	&sec_device_jack,
#ifdef CONFIG_BATTERY_SEC
	&sec_device_battery,
#endif
	&tegra_das_device,
	&ram_console_device,
#if defined (CONFIG_MACH_BOSE_ATT)
	&hall_device_slider,
#endif
};

static void n1_keys_init(void)
{
	int i;

#if !defined (CONFIG_MACH_BOSE_ATT)
	if (system_rev == 0)
		n1_keys[0].gpio = TEGRA_GPIO_PX3;
	else if (system_rev == 1) {
		n1_keys[0].gpio = TEGRA_GPIO_PK6;
		n1_keys[2].gpio = TEGRA_GPIO_PR0;
		n1_keys[3].gpio = TEGRA_GPIO_PR1;
	}
	else if (system_rev == 2) {
		n1_keys[0].gpio = TEGRA_GPIO_PX3;
	}
	else if (system_rev == 3 || system_rev == 4) {
		n1_keys[0].gpio = TEGRA_GPIO_PK6;
		n1_keys[0].wakeup = 0;
	}
#endif
	
	if (system_rev == 0) {
		for (i = 0; i < 2; i++)
			tegra_gpio_enable(n1_keys[i].gpio);
	} else {
		for (i = 0; i < ARRAY_SIZE(n1_keys); i++)
			tegra_gpio_enable(n1_keys[i].gpio);
	}

}

#if defined (CONFIG_MACH_BOSE_ATT) //  STMPE
int __init  n1_keys_stmpe1801b_init(void)
	
{
	printk("[n1_keys_stmpe1801b_init] , %s \n",__func__ );


	/* Qwerty key  INT GPIO initialize */
	int error;
	
	printk("[Board-N1] , %s \n",__func__ );

	error = gpio_request(GPIO_QKEY_IRQ, "Qwertykey_int");
	if (error != 0) {
		pr_err("Qertykey_int request FAIL error = %d", error);
	}
	gpio_direction_input(GPIO_QKEY_IRQ);
	tegra_gpio_enable(GPIO_QKEY_IRQ);
	
}

int __init  hall_ic_init(void)
	
{
	/* HALL IC INT GPIO initialize */
	int error;
	
	printk("[Board-N1] , %s \n",__func__ );

	error = gpio_request(GPIO_HALL_INT, "hall_ic_int");
	if (error != 0) {
		pr_err("GPIO_HALL_INT request FAIL error = %d", error);
	}
	gpio_direction_input(GPIO_HALL_INT);
	tegra_gpio_enable(GPIO_HALL_INT);
	
}
#endif

/**********************************/
static struct charging_status_callbacks {
	void	(*tsp_set_charging_cable) (int type);
} charging_cbs;

static bool is_cable_attached;

static void bose_touch_init_hw(struct mxt224_platform_data *pdata)
{
	int error;
	pr_info("(%s,%d)\n", __func__, __LINE__);

	/* TSP INT GPIO initialize */
	tegra_gpio_enable(pdata->gpio_read_done);
	error = gpio_request(pdata->gpio_read_done, "tsp_int");
	if (error != 0) {
		pr_err("tsp_int request FAIL error = %d", error);
	}
	gpio_direction_input(pdata->gpio_read_done);

	pdata->board_rev = system_rev;

	/* touch vdd io power level selecting */
	if (pdata->board_rev >= 9) {	/* after rev0.6(board_rev=9) vdd 1.8V */
		pdata->reg_vdd_level = 1800000;
	} else {	/* before rev0.6 vdd 1.8V */
		pdata->reg_vdd_level = 2800000;
	}
}

static void bose_touch_exit_hw(struct mxt224_platform_data *pdata)
{
	pr_info("(%s,%d)\n", __func__, __LINE__);
	gpio_free(pdata->gpio_read_done);
	tegra_gpio_disable(pdata->gpio_read_done);
}

/*atmel_mxt224E*/
static void mxt224_power_on(struct mxt224_data *pdata)
{
	pr_info("(%s,%d)\n", __func__, __LINE__);
	/* first pin configuration */
	gpio_direction_input(pdata->gpio_read_done);
	tegra_pinmux_set_pullupdown(pdata->i2c_pingroup, pdata->i2c_resume_pupd);

	/* and then, power on (on sequence: vdd -> avdd) */

	if (!IS_ERR(pdata->reg_vdd)) {
		regulator_enable(pdata->reg_vdd);
	}

	if (!IS_ERR(pdata->reg_avdd)) {
		regulator_enable(pdata->reg_avdd);
	}
	msleep(100);
	pr_info("[TSP] Power On!!");

}

static void mxt224_power_off(struct mxt224_data *pdata)
{
	pr_info("(%s,%d)\n", __func__, __LINE__);
	/* first power off (off sequence: avdd -> vdd) */
	if (!IS_ERR(pdata->reg_avdd)) {
		regulator_disable(pdata->reg_avdd);
	}

	if (!IS_ERR(pdata->reg_vdd)) {
		regulator_disable(pdata->reg_vdd);
	}
	/*and then, pin configuration */
	gpio_direction_output(pdata->gpio_read_done, 0);
	tegra_pinmux_set_pullupdown(pdata->i2c_pingroup, pdata->i2c_suspend_pupd);
	pr_info("[TSP] Power Off!!");

}

static void mxt224_register_callback(void *function)
{
	printk("mxt224_register_callback\n");
	charging_cbs.tsp_set_charging_cable = function;
}

static void mxt224_read_ta_status(bool *ta_status)
{
	*ta_status = is_cable_attached;
}

void n1_inform_charger_connection(u8 attached)
{
	/* TODO: need to implement this once uUSB charging available in HW */
	printk("\nBoard file [FSA9480]: Charger Callback \n");
	if(charging_cbs.tsp_set_charging_cable)
		charging_cbs.tsp_set_charging_cable(attached);
	is_cable_attached = attached;
}
EXPORT_SYMBOL(n1_inform_charger_connection);


#define MXT224_MAX_MT_FINGERS		10

/*
	Configuration for MXT224
*/
#define MXT224_THRESHOLD_BATT		40
#define MXT224_THRESHOLD_BATT_INIT		55
#define MXT224_THRESHOLD_CHRG		70
#define MXT224_NOISE_THRESHOLD_BATT		30
#define MXT224_NOISE_THRESHOLD_CHRG		40
#define MXT224_MOVFILTER_BATT		11
#define MXT224_MOVFILTER_CHRG		46
#define MXT224_ATCHCALST		9
#define MXT224_ATCHCALTHR		30

static u8 t7_config[] = {GEN_POWERCONFIG_T7,
				48, 255, 25};
static u8 t8_config[] = {GEN_ACQUISITIONCONFIG_T8,
				10, 0, 5, 1, 0, 0, MXT224_ATCHCALST, MXT224_ATCHCALTHR};
static u8 t9_config[] = {TOUCH_MULTITOUCHSCREEN_T9,
				131, 0, 0, 19, 11, 0, 32,  MXT224_THRESHOLD_BATT, 2, 7, 0, 15, 1,
				MXT224_MOVFILTER_BATT, MXT224_MAX_MT_FINGERS, 5, 40, 10, 31, 3,
				223, 1, 0, 0, 0, 0, 143, 55, 143, 90, 18};

static u8 t18_config[] = {SPT_COMCONFIG_T18,
				0, 1};
static u8 t20_config[] = {PROCI_GRIPFACESUPPRESSION_T20,
				7, 0, 0, 0, 0, 0, 0, 30, 20, 4, 15, 10};
static u8 t22_config[] = {PROCG_NOISESUPPRESSION_T22,
				143, 0, 0, 0, 0, 0, 0, 3, MXT224_NOISE_THRESHOLD_BATT, 0, 0,  29, 34, 39,
				49, 58, 3};
static u8 t28_config[] = {SPT_CTECONFIG_T28,
				0, 0, 3, 16, 19, 60};
static u8 end_config[] = {RESERVED_T255};

static const u8 *mxt224_config[] = {
	t7_config,
	t8_config,
	t9_config,
	t18_config,
	t20_config,
	t22_config,
	t28_config,
	end_config,
};

/*
	Configuration for MXT224-E
*/
#define MXT224E_THRESHOLD_BATT              50
#define MXT224E_THRESHOLD_CHRG              40
#define MXT224E_CALCFG_BATT                 64
#define MXT224E_CALCFG_CHRG                 82
#define MXT224E_ATCHFRCCALTHR_WAKEUP        8
#define MXT224E_ATCHFRCCALRATIO_WAKEUP      180
#define MXT224E_ATCHFRCCALTHR_NORMAL        40
#define MXT224E_ATCHFRCCALRATIO_NORMAL      55

static u8 t7_config_e[] = {GEN_POWERCONFIG_T7,
				48, 255, 25};

static u8 t8_config_e[] = {GEN_ACQUISITIONCONFIG_T8,
				27, 0, 5, 1, 0, 0, 4, 35, MXT224E_ATCHFRCCALTHR_WAKEUP,
				MXT224E_ATCHFRCCALRATIO_WAKEUP};

static u8 t9_config_e[] = {TOUCH_MULTITOUCHSCREEN_T9,
				139, 0, 0, 19, 11, 0, 32, MXT224E_THRESHOLD_BATT, 2, 7,
				10, 3, 1, 46, MXT224_MAX_MT_FINGERS, 5, 40, 10, 31, 3,
				223, 1, 10, 10, 10, 10, 143, 40, 143, 80,
				18, 15, 50, 50, 0};

static u8 t15_config_e[] = {TOUCH_KEYARRAY_T15,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t18_config_e[] = {SPT_COMCONFIG_T18,
				0, 0};

static u8 t19_config_e[] = {SPT_GPIOPWM_T19,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t23_config_e[] = {TOUCH_PROXIMITY_T23,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t25_config_e[] = {SPT_SELFTEST_T25,
				0, 0, 0, 0, 0, 0, 0, 0};

static u8 t40_config_e[] = {PROCI_GRIPSUPPRESSION_T40,
				0, 0, 0, 0, 0};

static u8 t42_config_e[] = {PROCI_TOUCHSUPPRESSION_T42,
				0, 32, 120, 100, 0, 0, 0, 0};

static u8 t46_config_e[] = {SPT_CTECONFIG_T46,
				0, 3, 16, 32, 0, 0, 1, 0, 0};

static u8 t47_config_e[] = {PROCI_STYLUS_T47,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t48_config_e[] = {PROCG_NOISESUPPRESSION_T48,
				3, 132, MXT224E_CALCFG_BATT, 0, 0, 0, 0, 0, 10, 20,
				0, 0, 0, 6, 6, 0, 0, 100, 4, 64,
				10, 0, 20, 5, 0, 38, 0, 5, 0, 0,
				0, 0, 0, 0, 32, MXT224E_THRESHOLD_BATT, 2, 3, 1, 46,
				MXT224_MAX_MT_FINGERS, 5, 40, 10, 10, 10, 10, 143, 40, 143,
				80, 18, 15, 0 };

static u8 t48_config_chrg_e[] = {PROCG_NOISESUPPRESSION_T48,
				3, 132, MXT224E_CALCFG_CHRG, 0, 0, 0, 0, 0, 10, 20,
				0, 0, 0, 6, 6, 0, 0, 64, 4, 64,
				10, 0, 20, 5, 0, 38, 0, 20, 0, 0,
				0, 0, 0, 0, 0, MXT224E_THRESHOLD_CHRG, 2, 5, 2, 47,
				MXT224_MAX_MT_FINGERS, 5, 40, 246, 246, 10, 10, 150, 50, 143,
				80, 18, 15, 0 };

static u8 end_config_e[] = {RESERVED_T255};

static const u8 *mxt224e_config[] = {
	t7_config_e,
	t8_config_e,
	t9_config_e,
	t15_config_e,
	t18_config_e,
	t19_config_e,
	t23_config_e,
	t25_config_e,
	t40_config_e,
	t42_config_e,
	t46_config_e,
	t47_config_e,
	t48_config_e,
	end_config_e,
};

static struct mxt224_platform_data mxt224_pdata = {
	.max_finger_touches = MXT224_MAX_MT_FINGERS,
	.gpio_read_done = GPIO_TSP_INT,
	.config = mxt224_config,
	.config_e = mxt224e_config,
	.min_x = 0,
	.max_x = 479,
	.min_y = 0,
	.max_y = 799,
	.min_z = 0,
	.max_z = 255,
	.min_w = 0,
	.max_w = 30,
	.atchcalst = MXT224_ATCHCALST,
	.atchcalsthr = MXT224_ATCHCALTHR,
	.tchthr_batt = MXT224_THRESHOLD_BATT,
	.tchthr_batt_init = MXT224_THRESHOLD_BATT_INIT,
	.tchthr_charging = MXT224_THRESHOLD_CHRG,
	.noisethr_batt = MXT224_NOISE_THRESHOLD_BATT,
	.noisethr_charging = MXT224_NOISE_THRESHOLD_CHRG,
	.movfilter_batt = MXT224_MOVFILTER_BATT,
	.movfilter_charging = MXT224_MOVFILTER_CHRG,
	.tchthr_batt_e = MXT224E_THRESHOLD_BATT,
	.tchthr_charging_e = MXT224E_THRESHOLD_CHRG,
	.calcfg_batt_e = MXT224E_CALCFG_BATT,
	.calcfg_charging_e = MXT224E_CALCFG_CHRG,
	.atchfrccalthr_e = MXT224E_ATCHFRCCALTHR_NORMAL,
	.atchfrccalratio_e = MXT224E_ATCHFRCCALRATIO_NORMAL,
	.t48_config_batt_e = t48_config_e,
	.t48_config_chrg_e = t48_config_chrg_e,
	.power_on = mxt224_power_on,
	.power_off = mxt224_power_off,
	.register_cb = mxt224_register_callback,
	.read_ta_status = mxt224_read_ta_status,

	.reg_vdd_name = "TSP_VDD_LVSIO",	/* OUT15 */
	.reg_vdd = NULL,
	.reg_vdd_level = NULL,
	
	.reg_avdd_name = "TSP_AVDD_3V3",	/* OUT13 */
	.reg_avdd = NULL,
	.reg_avdd_level = 3300000,

	.i2c_pingroup = TEGRA_PINGROUP_DDC,
	.i2c_resume_pupd = TEGRA_PUPD_PULL_UP,
	.i2c_suspend_pupd = TEGRA_PUPD_PULL_DOWN,

	.board_rev = NULL,
	.platform_name = MXT224_DEV_NAME,
};

static struct i2c_board_info atmel_i2c_touch_info[] = {
	{
		I2C_BOARD_INFO(MXT224_DEV_NAME, 0x4a),
		.irq		= TEGRA_GPIO_TO_IRQ(GPIO_TSP_INT),
		.platform_data = &mxt224_pdata,
	},
};

static void n1_camera_init(void)
{
#if defined (CONFIG_MACH_BOSE_ATT)
	tegra_gpio_enable(GPIO_CAM_PMIC_EN2);
	tegra_gpio_enable(GPIO_CAM_MEGA_nRST);
	tegra_gpio_enable(GPIO_CAM_VT_nRST);
	tegra_gpio_enable(GPIO_CAM_VT_nSTBY);
	tegra_gpio_enable(GPIO_CAM_MEGA_INT);
	if (HWREV_FOR_CAMERA <= system_rev) {
		tegra_gpio_enable(GPIO_CAM_MEGA_CORE_1P2_EN);
	}

	gpio_request(GPIO_CAM_PMIC_EN2, "CAMERA_PMIC_EN2");
	gpio_request(GPIO_CAM_MEGA_nRST, "CAMERA_CAM_MEGA_nRST");
	gpio_request(GPIO_CAM_VT_nRST, "CAMERA_CAM_VT_nRST");
	gpio_request(GPIO_CAM_VT_nSTBY, "CAMERA_CAM_VT_nSTBY");
	gpio_request(GPIO_CAM_MEGA_INT, "MEGA_INT");
	if (HWREV_FOR_CAMERA <= system_rev) {
		gpio_request(GPIO_CAM_MEGA_CORE_1P2_EN, "GPIO_CAM_MEGA_CORE_1P2_EN");
	}

	gpio_direction_output(GPIO_CAM_PMIC_EN2, 0);
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);
	gpio_direction_output(GPIO_CAM_VT_nRST, 0);
	gpio_direction_output(GPIO_CAM_VT_nSTBY, 0);
	gpio_direction_input(GPIO_CAM_MEGA_INT);
	if (HWREV_FOR_CAMERA <= system_rev) {
		gpio_direction_output(GPIO_CAM_MEGA_CORE_1P2_EN, 0);
	}
#else
	tegra_gpio_enable(GPIO_CAM_PMIC_EN2);	//CAM_SENSOR_A2.8V
	tegra_gpio_enable(GPIO_CAM_MEGA_nRST);	//CAM_MEGA_nRST
	tegra_gpio_enable(GPIO_CAM_VT_nRST);	//GPIO_PT4
	tegra_gpio_enable(GPIO_CAM_VT_nSTBY);	//GPIO_PD5
	tegra_gpio_enable(GPIO_CAM_MEGA_STBY);	//GPIO_PD6
	tegra_gpio_enable(GPIO_CAM_FLASH_EN);	//GPIO_PX3
	tegra_gpio_enable(GPIO_CAM_FLASH_SET);	//GPIO_PX2

	gpio_request(GPIO_CAM_PMIC_EN2, "CAMERA_PMIC_EN2");
	gpio_request(GPIO_CAM_MEGA_nRST, "CAMERA_CAM_MEGA_nRST");
	gpio_request(GPIO_CAM_VT_nRST, "CAMERA_CAM_VT_nRST");
	gpio_request(GPIO_CAM_VT_nSTBY, "CAMERA_CAM_VT_nSTBY");
	gpio_request(GPIO_CAM_MEGA_STBY, "CAMERA_CAM_MEGA_STBY");
	gpio_request(GPIO_CAM_FLASH_EN, "GPIO_CAM_FLASH_EN");
	gpio_request(GPIO_CAM_FLASH_SET, "GPIO_CAM_FLASH_SET");

	gpio_direction_output(GPIO_CAM_PMIC_EN2, 0);
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 0);
	gpio_direction_output(GPIO_CAM_VT_nRST, 0);
	gpio_direction_output(GPIO_CAM_VT_nSTBY, 0);
	gpio_direction_output(GPIO_CAM_MEGA_STBY, 0);
	gpio_direction_output(GPIO_CAM_FLASH_EN, 0);
	gpio_direction_output(GPIO_CAM_FLASH_SET, 0);
#endif	
}

struct tegra_pingroup_config mclk = {
	TEGRA_PINGROUP_CSUS,
	TEGRA_MUX_VI_SENSOR_CLK,
	TEGRA_PUPD_PULL_DOWN,
	TEGRA_TRI_TRISTATE
};

struct i2c_client *i2c_client_pmic;

#if defined (CONFIG_MACH_BOSE_ATT)
extern struct regulator *reg_mipi_1v2;
#endif

#if defined(CONFIG_VIDEO_S5K4ECGX)
static struct s5k4ecgx_reg_8 s5k4ecgx_power_on_1[] = {
	{0x06, 0x03},	// VT_CORE_1.5V - LDO2
	{0x00, 0x11}, 
	{S5K4ECGX_TABLE_WAIT_MS_8, 0x01},

	{0x05, 0x02},	// CAM_IO_1.8V - LDO1
	{0x00, 0x31},
	{S5K4ECGX_TABLE_WAIT_MS_8, 0x01},

	{S5K4ECGX_TABLE_END_8, 0x0},
};

static struct s5k4ecgx_reg_8 s5k4ecgx_power_on_2[] = {
	{0x04, 0x04},	// CAM_CORE_1.2V - BUCK
	{0x00, 0xB1},
	{S5K4ECGX_TABLE_WAIT_MS_8, 0x01},

	{S5K4ECGX_TABLE_END_8, 0x0},
};

static void n1_s5k4ecgx_power_on()
{
	pr_err("%s,\n", __func__);

	gpio_set_value(GPIO_CAM_PMIC_EN2, 1);	//CAM_SENSOR_A2.8V
	msleep(1);

	s5k4ecgx_write_table_8(i2c_client_pmic, s5k4ecgx_power_on_1);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 1);	//CAM_VT_nSTBY
	msleep(1);

	tegra_pinmux_set_func(&mclk);	//CAM_MCLK
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	msleep(1);

	gpio_set_value(GPIO_CAM_VT_nRST, 1);
	msleep(150);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 0);
	msleep(1);

	s5k4ecgx_write_table_8(i2c_client_pmic, s5k4ecgx_power_on_2);

	gpio_set_value(GPIO_CAM_MEGA_STBY, 1);
	msleep(1);

	gpio_set_value(GPIO_CAM_MEGA_nRST, 1);
	msleep(1);
}

static struct s5k4ecgx_reg s5k4ecgx_power_off[] = {
	{0x00, 0x91},				// CAM_IO_1.8V - LDO1
	{0x00, 0x81},				// VT_CORE_1.5V - LDO2
	{0x00, 0x01},				// CAM_CORE_1.2V - BUCK

	{S5K4ECGX_TABLE_END, 0x0},
};

static void n1_s5k4ecgx_power_off()
{
	pr_err("%s,\n", __func__);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 0);
	msleep(1);

	gpio_set_value(GPIO_CAM_VT_nRST, 0);
	msleep(1);

	gpio_set_value(GPIO_CAM_MEGA_nRST, 0);
	msleep(1);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);
	msleep(1);

	gpio_set_value(GPIO_CAM_MEGA_STBY, 0);
	msleep(1);

	s5k4ecgx_write_table_8(i2c_client_pmic, s5k4ecgx_power_off);
	msleep(1);

	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);
	msleep(1);
	//mdelay(20);

}

static int n1_s5k4ecgx_flash(int enable)
{
	/* Turn main flash on or off by asserting a value on the EN line. */
	printk("========== flash enable = %d \n", enable);
	gpio_set_value(GPIO_CAM_FLASH_EN, enable);

	return 0;
}

static int n1_s5k4ecgx_torch(int enable)
{
	printk("========== torch enable = %d \n", enable);
	switch(enable) {
		case 0:
			gpio_set_value(GPIO_CAM_FLASH_EN, 0);
			gpio_set_value(GPIO_CAM_FLASH_SET, 0);
			break;
		case 1:
			gpio_set_value(GPIO_CAM_FLASH_EN, 1);
			gpio_set_value(GPIO_CAM_FLASH_SET, 1);
			break;
	}

	return 0;
}

struct s5k4ecgx_platform_data n1_s5k4ecgx_data = {
	.power_on = n1_s5k4ecgx_power_on,
	.power_off = n1_s5k4ecgx_power_off,
	.flash_onoff = n1_s5k4ecgx_flash,
	.torch_onoff = n1_s5k4ecgx_torch,
};

static const struct i2c_board_info sec_s5k4ecgx_camera[] = {
	{
		I2C_BOARD_INFO("s5k4ecgx", 0xAC>>1),
		.platform_data = &n1_s5k4ecgx_data,
	},
};

static const struct i2c_board_info sec_pmic_camera[] = {
	{
		I2C_BOARD_INFO("s5k4ecgx_pmic", 0x7c>>1),
	},
};
#endif

#if defined(CONFIG_VIDEO_S5K6AAFX)
struct tegra_pingroup_config s5k6aafx_mclk = {
	TEGRA_PINGROUP_CSUS, 
	TEGRA_MUX_VI_SENSOR_CLK, 
	TEGRA_PUPD_PULL_DOWN, 
	TEGRA_TRI_TRISTATE
};

#if defined (CONFIG_MACH_BOSE_ATT)
static struct s5k6aafx_reg_8 s5k6aafx_power_on_1[] = {
	{0x04, 0x04}, /*BUCK 1.2V 000 00100 CAM_ISP_CORE_1.2V*/
	{0x00, 0x81},
	{S5K6AAFX_TABLE_END_8, 0x0},
};

static struct s5k6aafx_reg_8 s5k6aafx_power_on_2[] = {
	{0x06, 0x03}, // VT_CORE_1.5V - LDO2
	{0x00, 0x91},
	{S5K6AAFX_TABLE_WAIT_MS_8, 0x01},

	{0x05, 0x02}, // VT_IO_1.8V - LDO1
	{0x00, 0xB1},
	{S5K6AAFX_TABLE_WAIT_MS_8, 0x01},

	{0x07, 0x02}, // VT_CAM_1.8V - LDO3
	{0x00, 0xB9},
	{S5K6AAFX_TABLE_END_8, 0x0},	
};

static void n1_s5k6aafx_power_on(void)
{
	printk("(%s) : S5K6AAFX (Front Camera) Power on. HW Rev.%d\n", __func__, system_rev);

	//regulator_enable(reg_mipi_1v2);
	
	s5k6aafx_write_table_8(i2c_client_pmic, s5k6aafx_power_on_1);

	gpio_set_value(GPIO_CAM_PMIC_EN2, 1);

	mdelay(2); 

	s5k6aafx_write_table_8(i2c_client_pmic, s5k6aafx_power_on_2);

	udelay(20); //11.08.05

	gpio_set_value(GPIO_CAM_VT_nSTBY, 1);

	tegra_pinmux_set_func(&s5k6aafx_mclk);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);

	udelay(100);

	gpio_set_value(GPIO_CAM_VT_nRST, 1);

	mdelay(5);
}

static struct s5k6aafx_reg_8 s5k6aafx_power_off_2[] = {
	{0x00, 0xB1},
	{0x00, 0x91},
	{S5K6AAFX_TABLE_END_8, 0x0},
};

static struct s5k6aafx_reg_8 s5k6aafx_power_off_1[] = {
	{0x00, 0x81},
	{0x00, 0x01},
	{S5K6AAFX_TABLE_END_8, 0x0},	
};

static void n1_s5k6aafx_power_off(void)
{
	printk("(%s) : S5K6AAFX (Front Camera) Power off. HW Rev.%d\n", __func__, system_rev);	
	
	gpio_set_value(GPIO_CAM_VT_nRST, 0);

	udelay(100);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 0);

	s5k6aafx_write_table_8(i2c_client_pmic, s5k6aafx_power_off_2);

	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);

	s5k6aafx_write_table_8(i2c_client_pmic, s5k6aafx_power_off_1);

	//regulator_disable(reg_mipi_1v2);	

	mdelay(1);
}
#else

/*static struct s5k6aafx_reg_8 s5k6aafx_power_on_1[] = {
	{0x04, 0x04},	// CAM_CORE_1.2V - BUCK
	{0x00, 0x81},
	{S5K6AAFX_TABLE_WAIT_MS_8, 0x01},

	{S5K6AAFX_TABLE_END_8, 0x0},
};*/

static struct s5k6aafx_reg_8 s5k6aafx_power_on_1[] = {
	{0x06, 0x03},	// VT_CORE_1.5V - LDO2
	{0x00, 0x11},
	{S5K6AAFX_TABLE_WAIT_MS_8, 0x01},

	{0x07, 0x02},	// VT_CAM_1.8V - LDO3
	{0x00, 0x19},
	{S5K6AAFX_TABLE_WAIT_MS_8, 0x01},

	{0x05, 0x02},	// CAM_IO_1.8V - LDO1
	{0x00, 0x39},
	{S5K6AAFX_TABLE_WAIT_MS_8, 0x01},

	{S5K6AAFX_TABLE_END_8, 0x0},
};

static void n1_s5k6aafx_power_on(void)
{
	pr_err("%s,\n", __func__);

	gpio_set_value(GPIO_CAM_PMIC_EN2, 1);
	msleep(1);

	s5k6aafx_write_table_8(i2c_client_pmic, s5k6aafx_power_on_1);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 1);
	msleep(1);

	tegra_pinmux_set_func(&s5k6aafx_mclk);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	msleep(1);

	gpio_set_value(GPIO_CAM_VT_nRST, 1);
	msleep(1);
}

static struct s5k6aafx_reg_8 s5k6aafx_power_off_1[] = {
	{0x00, 0x31},	// VT_CAM_1.8V - LDO3
	{0x00, 0x11},	// CAM_IO_1.8V - LDO1
	{0x00, 0x01},	// VT_CORE_1.5V - LDO2

	{S5K6AAFX_TABLE_END_8, 0x0},
};

static void n1_s5k6aafx_power_off(void)
{
	pr_err("%s,\n", __func__);

	gpio_set_value(GPIO_CAM_VT_nRST, 0);
	msleep(1);

	gpio_set_value(GPIO_CAM_MEGA_nRST, 0);
	msleep(1);

	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);
	msleep(1);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 0);
	msleep(20);

	s5k6aafx_write_table_8(i2c_client_pmic, s5k6aafx_power_off_1);
	msleep(1);

	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);
	msleep(1);
}
#endif

struct s5k6aafx_platform_data n1_s5k6aafx_data = {
	.power_on = n1_s5k6aafx_power_on,
	.power_off = n1_s5k6aafx_power_off
};

static const struct i2c_board_info sec_s5k6aafx_camera[] = {
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78>>1),
		.platform_data = &n1_s5k6aafx_data,
	},
};
#endif

#if defined(CONFIG_VIDEO_M5MO)	    
#if defined (CONFIG_MACH_BOSE_ATT)

static struct m5mo_reg m5mo_power_on_rev_06_1[] = {
	{0x04, 0x04}, /*BUCK 1.2V CAM_ISP_CORE_1.2V*/		
	{0x00, 0x81},
	{M5MO_TABLE_WAIT_US, 0x01},
		
	{M5MO_TABLE_END, 0x0},
};

static struct m5mo_reg m5mo_power_on_rev_06_2[] = {	
	{0x06, 0x03}, /*ldo2  1.5V VT_CORE_1.5V*/
	{0x00, 0x91},
	{M5MO_TABLE_WAIT_US, 0x01},
	
	{0x05, 0x02}, /*ldo1 1.8V CAM_ISP_1.8V*/
	{0x00, 0xB1},
	{M5MO_TABLE_WAIT_US, 0x01},

	{0x07, 0x02}, /*ldo3 1.8V VT_SENSOR_IO_1.8V*/
	{0x00, 0xB9},
	{M5MO_TABLE_WAIT_US, 0x01},

	{0x09, 0x0A}, /*ldo5 1.8V CAM_SENSOR_IO_1.8V*/
	{0x00, 0xBB},
	{M5MO_TABLE_END, 0x00},
};

static struct m5mo_reg m5mo_power_on_1[] = {
	{0x04, 0x04}, /*BUCK 1.2V CAM_ISP_CORE_1.2V*/		
	{0x00, 0x81},
	{M5MO_TABLE_WAIT_US, 0x01},
		
	{0x08, 0x04}, /*ldo4 1.2V CAM_SENSOR_CORE_1.2V*/
	{0x00, 0x85},
	{M5MO_TABLE_END, 0x0},
};

static struct m5mo_reg m5mo_power_on_2[] = {	
	{0x06, 0x03}, /*ldo2  1.5V VT_CORE_1.5V*/
	{0x00, 0x95},
	{M5MO_TABLE_WAIT_US, 0x01},

	{0x05, 0x02}, /*ldo1 1.8V CAM_ISP_1.8V*/
	{0x00, 0xB5},

	{0x07, 0x02}, /*ldo3 1.8V VT_SENSOR_IO_1.8V*/
	{0x00, 0xBD},
	{M5MO_TABLE_WAIT_US, 0x0A},

	{0x09, 0x0A}, /*ldo5 1.8V CAM_SENSOR_IO_1.8V*/
	{0x00, 0xBF},	
	{M5MO_TABLE_END, 0x00},	
};

#else

static struct m5mo_reg m5mo_power_on_1[] = {
	{0x04, 0x04}, /*BUCK 1.2V 000 00100 CAM_ISP_CORE_1.2V*/		
	{M5MO_TABLE_WAIT_US, 0x01},
		
	{0x08, 0x04}, /*ldo4 1.2V 00000100 CAM_SENSOR_CORE_1.2V*/
	{0x00, 0x85}, /*Enable CAM_ISP_CORE_1.2V*/	
    {M5MO_TABLE_END, 0x0},
};

static struct m5mo_reg m5mo_power_on_2[] = {
	{0x09, 0x0A}, /*ldo5 1.8V 000 01010*/
	{0x00, 0x87}, /*Enable CAM_SENSOR_IO_1.8V*/	
	{M5MO_TABLE_WAIT_US, 0x01},

	{0x05, 0x02}, /*ldo1 1.8V 000 00010*/
	{0x00, 0xA7}, /*Enable CAM_ISP_1.8V*/	
	{M5MO_TABLE_END, 0x0},
};

#endif

static void n1_m5mo_power_on()
{
	printk("(%s) : M5MO (Rear Camera) Power on. HW Rev.%d\n", __func__, system_rev);
	
	//regulator_enable(reg_mipi_1v2);

#if defined (CONFIG_MACH_BOSE_ATT)
	if(HWREV_FOR_CAMERA <= system_rev) {
		m5mo_write_table(i2c_client_pmic, m5mo_power_on_rev_06_1, 2);	// CAM_ISP_CORE_1.2V 		
	} else {
		m5mo_write_table(i2c_client_pmic, m5mo_power_on_1, 2);	// CAM_ISP_CORE_1.2V, CAM_SENSOR_CORE_1.2V 	
	}
#else
	m5mo_write_table(i2c_client_pmic, m5mo_power_on_1, 2);	// CAM_ISP_CORE_1.2V, CAM_SENSOR_CORE_1.2V 
#endif

#if defined (CONFIG_MACH_BOSE_ATT)
	if(HWREV_FOR_CAMERA <= system_rev) {
		gpio_set_value(GPIO_CAM_MEGA_CORE_1P2_EN, 1); //  CAM_SENSOR_CORE_1.2V 
	}
#endif

	gpio_set_value(GPIO_CAM_PMIC_EN2, 1);	// CAM_SENSOR_A2.8V, CAM_AF_2.8V

	mdelay(6);

#if defined (CONFIG_MACH_BOSE_ATT)
	if(HWREV_FOR_CAMERA <= system_rev) {
		m5mo_write_table(i2c_client_pmic, m5mo_power_on_rev_06_2, 2);	// VT_CAM_SENSOR_1.5V, CAM_SENSOR_IO_1.8V, VT_SENSOR_IO_1, CAM_ISP_1.8V 	
	} else {
		m5mo_write_table(i2c_client_pmic, m5mo_power_on_2, 2);	// VT_CAM_SENSOR_1.5V, CAM_SENSOR_IO_1.8V, VT_SENSOR_IO_1, CAM_ISP_1.8V 	
	}
#else
	m5mo_write_table(i2c_client_pmic, m5mo_power_on_2, 2);	//  CAM_SENSOR_IO_1.8V, CAM_ISP_1.8V 	
#endif

	udelay(100); // 11.08.05

	gpio_set_value(GPIO_CAM_VT_nSTBY, 1);

	tegra_pinmux_set_func(&mclk);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_NORMAL);
	
	udelay(500);

	gpio_set_value(GPIO_CAM_VT_nRST, 1);

	msleep(150);

	gpio_set_value(GPIO_CAM_VT_nSTBY, 0);

	msleep(1);

	gpio_set_value(GPIO_CAM_MEGA_nRST, 1);
	
	mdelay(5);
}

#if defined (CONFIG_MACH_BOSE_ATT)

static struct m5mo_reg m5mo_power_off_rev_06_2[] = {
	{0x00, 0xB9},
	{0x00, 0xB1},
	{0x00, 0x91},
	{M5MO_TABLE_END, 0x0},
};

static struct m5mo_reg m5mo_power_off_rev_06_1[] = {
	{0x00, 0x81},
	{0x00, 0x01},
	{M5MO_TABLE_END, 0x0},
};

static struct m5mo_reg m5mo_power_off_2[] = {
	{0x00, 0xBD},
	{0x00, 0xB5},
	{0x00, 0x95},
	{M5MO_TABLE_END, 0x0},
};

static struct m5mo_reg m5mo_power_off_1[] = {
	{0x00, 0x85},
	{0x00, 0x81},
	{0x00, 0x01},
	{M5MO_TABLE_END, 0x0},
};

#else

static struct m5mo_reg m5mo_power_off_2[] = {
	{0x00, 0x87},
	{0x00, 0x85},
	{M5MO_TABLE_END, 0x0},	
};

static struct m5mo_reg m5mo_power_off_1[] = {
	{0x00, 0x81},
	{0x00, 0x01},
	{M5MO_TABLE_END, 0x0},
};

#endif

static void n1_m5mo_power_off()
{
	printk("(%s) : M5MO (Rear Camera) Power off. HW Rev.%d\n", __func__, system_rev);

	msleep(3);

	gpio_set_value(GPIO_CAM_MEGA_nRST, 0);

	gpio_set_value(GPIO_CAM_VT_nRST, 0);

	udelay(1000);  //11.08.05

	tegra_pinmux_set_func(&mclk);
	tegra_pinmux_set_tristate(TEGRA_PINGROUP_CSUS, TEGRA_TRI_TRISTATE);

#if defined (CONFIG_MACH_BOSE_ATT)
	if(HWREV_FOR_CAMERA <= system_rev) {
		m5mo_write_table(i2c_client_pmic, m5mo_power_off_rev_06_2, 2);
	} else {
		m5mo_write_table(i2c_client_pmic, m5mo_power_off_2, 2);
	}
#else
	m5mo_write_table(i2c_client_pmic, m5mo_power_off_2, 2);
#endif

	gpio_set_value(GPIO_CAM_PMIC_EN2, 0);

#if defined (CONFIG_MACH_BOSE_ATT)
	if (HWREV_FOR_CAMERA <= system_rev) {
		gpio_set_value(GPIO_CAM_MEGA_CORE_1P2_EN, 0);
	}
#endif

#if defined (CONFIG_MACH_BOSE_ATT)
	if(HWREV_FOR_CAMERA <= system_rev) {
		m5mo_write_table(i2c_client_pmic, m5mo_power_off_rev_06_1, 2);
	} else {
		m5mo_write_table(i2c_client_pmic, m5mo_power_off_1, 2);
	}
#else
	m5mo_write_table(i2c_client_pmic, m5mo_power_off_1, 2);
#endif

	//regulator_disable(reg_mipi_1v2);

	mdelay(1);
}

static unsigned int n1_m5mo_isp_int_read()
{
	return gpio_get_value(GPIO_CAM_MEGA_INT);
}

struct m5mo_platform_data n1_m5mo_data = {
	.power_on = n1_m5mo_power_on,
	.power_off = n1_m5mo_power_off,
	.isp_int_read = n1_m5mo_isp_int_read
};

static const struct i2c_board_info sec_m5mo_camera[] = {
{
		I2C_BOARD_INFO("m5mo", 0x3E>>1),
		.platform_data = &n1_m5mo_data,
	},
};

static const struct i2c_board_info sec_pmic_camera[] = {
	{
		I2C_BOARD_INFO("m5mo_pmic", 0x7c>>1),
	},
};

#endif //end of CONFIG_VIDEO_M5MO


static int __init camera_init(void)
{
	int status;
	n1_camera_init();

#if defined (CONFIG_MACH_BOSE_ATT)
	if(system_rev >= 4) {
		status = i2c_register_board_info(16, sec_pmic_camera,
					ARRAY_SIZE(sec_pmic_camera));
		status = i2c_register_board_info(3, sec_m5mo_camera,
					ARRAY_SIZE(sec_m5mo_camera));
		status = i2c_register_board_info(3, sec_s5k6aafx_camera,
					ARRAY_SIZE(sec_s5k6aafx_camera));
	} else {
		status = i2c_register_board_info(3, sec_pmic_camera,
					ARRAY_SIZE(sec_pmic_camera));
		status = i2c_register_board_info(3, sec_m5mo_camera,
					ARRAY_SIZE(sec_m5mo_camera));
		status = i2c_register_board_info(3, sec_s5k6aafx_camera,
					ARRAY_SIZE(sec_s5k6aafx_camera));
	}
#else
	if(system_rev >= 4) {
		status = i2c_register_board_info(16, sec_pmic_camera,
					ARRAY_SIZE(sec_pmic_camera));
		status = i2c_register_board_info(3, sec_s5k4ecgx_camera,
					ARRAY_SIZE(sec_s5k4ecgx_camera));
		status = i2c_register_board_info(3, sec_s5k6aafx_camera,
					ARRAY_SIZE(sec_s5k6aafx_camera));
	} else {
		status = i2c_register_board_info(3, sec_pmic_camera,
					ARRAY_SIZE(sec_pmic_camera));
		status = i2c_register_board_info(3, sec_s5k4ecgx_camera,
					ARRAY_SIZE(sec_s5k4ecgx_camera));
		status = i2c_register_board_info(3, sec_s5k6aafx_camera,
					ARRAY_SIZE(sec_s5k6aafx_camera));
	}
#endif

	return 0;
}


static int __init bose_touch_init(void)
{
	bose_touch_init_hw(&mxt224_pdata);

	i2c_register_board_info(1, atmel_i2c_touch_info,
			ARRAY_SIZE(atmel_i2c_touch_info));
	return 0;
}

void tegra_usb1_ldo_en(int active, int instance)
{
	struct regulator *reg = regulator_get(NULL, "VAP_USB_3V3");
	int ret = 0;
	int try_times = 3;

	if (IS_ERR_OR_NULL(reg)) {
		pr_err("%s: failed to get VAP_USB_3V3 regulator\n",
							__func__);
		return;
	}
	pr_info("Board N1 : %s=%d instance=%d present regulator=%d\n", 
		__func__, active, instance, 
			usb_data.usb_regulator_on[instance]);	
	mutex_lock(&usb_data.ldo_en_lock);
	
	if (active) {
		if (!usb_data.usb_regulator_on[instance]) {
			do {
				ret = regulator_enable(reg);
				if (ret == 0) 
					break;
				msleep(3);
			} while (try_times--);
			if (ret == 0)
				usb_data.usb_regulator_on[instance] = 1;
			else
				pr_err("%s: failed to turn on \\
				VAP_USB_3V3 regulator\n", __func__);
		}
	} else {
		regulator_disable(reg);
		usb_data.usb_regulator_on[instance] = 0;
	}
	regulator_put(reg);
	
	mutex_unlock(&usb_data.ldo_en_lock);
}

void tegra_usb1_power(int active)
{
	active = !!active;

	if (active)
		otg_open.otg_enabled = 1;
	else
		otg_open.otg_enabled = 0;

	gpio_direction_output(GPIO_USB_OTG_EN, active);
	pr_info("Board N1 : %s = %d\n", __func__, active);
}

static void tegra_set_otg_func(void (*check_detection)(int *, int), 
				int * clk_cause)
{
	otg_clk_data.check_detection = check_detection;
	otg_clk_data.clk_cause = clk_cause;
}

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
       [0] = {
                       .instance = 0,
                       .vbus_irq = MAX8907C_INT_BASE + MAX8907C_IRQ_VCHG_DC_R,
                       .vbus_gpio = -1,
                       .usb_ldo_en = tegra_usb1_ldo_en,
       },
       [1] = {
                       .instance = 1,
                       .vbus_gpio = -1,
       },
       [2] = {
                       .instance = 2,
                       .vbus_gpio = -1,
       },
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_OTG,
			.power_down_on_bus_suspend = 0,
			.host_notify = 1,
			.sec_whlist_table_num = 1,
	},
	[1] = {
			.phy_config = &hsic_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
	},
};

struct platform_device *tegra_usb_otg_host_register(void)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(tegra_ehci1_device.name,
					tegra_ehci1_device.id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, tegra_ehci1_device.resource,
		tegra_ehci1_device.num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  tegra_ehci1_device.dev.dma_mask;
	pdev->dev.coherent_dma_mask = tegra_ehci1_device.dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
							GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, &tegra_ehci_pdata[0],
				sizeof(struct tegra_ehci_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host contoller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.host_register = &tegra_usb_otg_host_register,
	.host_unregister = &tegra_usb_otg_host_unregister,
	.acc_power = tegra_usb1_power,
	.usb_ldo_en = tegra_usb1_ldo_en,
	.set_otg_func = tegra_set_otg_func,
	.otg_id_open = &otg_open,
	.batt_level = &sec_batt_level,
};

#define AHB_ARBITRATION_DISABLE		0x0
#define   USB_ENB			(1 << 6)
#define   USB2_ENB	`		(1 << 18)
#define   USB3_ENB			(1 << 17)

#define AHB_ARBITRATION_PRIORITY_CTRL   0x4
#define   AHB_PRIORITY_WEIGHT(x)	(((x) & 0x7) << 29)
#define   PRIORITY_SELEECT_USB		(1 << 6)
#define   PRIORITY_SELEECT_USB2		(1 << 18)
#define   PRIORITY_SELEECT_USB3		(1 << 17)

#define AHB_GIZMO_AHB_MEM		0xc
#define   ENB_FAST_REARBITRATE		(1 << 2)

#define AHB_GIZMO_APB_DMA		0x10

#define AHB_GIZMO_USB			0x1c
#define AHB_GIZMO_USB2			0x78
#define AHB_GIZMO_USB3			0x7c
#define   IMMEDIATE			(1 << 18)
#define   MAX_AHB_BURSTSIZE(x)		(((x) & 0x3) << 16)
#define	  DMA_BURST_1WORDS		MAX_AHB_BURSTSIZE(0)
#define	  DMA_BURST_4WORDS		MAX_AHB_BURSTSIZE(1)
#define	  DMA_BURST_8WORDS		MAX_AHB_BURSTSIZE(2)
#define	  DMA_BURST_16WORDS		MAX_AHB_BURSTSIZE(3)

#define AHB_MEM_PREFETCH_CFG3		0xe0
#define AHB_MEM_PREFETCH_CFG4		0xe4
#define AHB_MEM_PREFETCH_CFG1		0xec
#define AHB_MEM_PREFETCH_CFG2		0xf0
#define   PREFETCH_ENB			(1 << 31)
#define   MST_ID(x)			(((x) & 0x1f) << 26)
#define   USB_MST_ID			MST_ID(6)
#define   USB2_MST_ID			MST_ID(18)
#define   USB3_MST_ID			MST_ID(17)
#define   ADDR_BNDRY(x)			(((x) & 0x1f) << 21)
#define		SPEC_THROTTLE(x)		(((x) & 0x1f) << 16)
#define   INACTIVITY_TIMEOUT(x)		(((x) & 0xffff) << 0)


static inline unsigned long gizmo_readl(unsigned long offset)
{
	return readl(IO_TO_VIRT(TEGRA_AHB_GIZMO_BASE + offset));
}

static inline void gizmo_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_AHB_GIZMO_BASE + offset));
}
static void n1_usb_init(void)
{
	unsigned long val; 
	int ret;

	mutex_init(&usb_data.ldo_en_lock);
	usb_data.usb_regulator_on[0] = 0;
	usb_data.usb_regulator_on[1] = 0;
	usb_data.usb_regulator_on[2] = 0;
	
	tegra_gpio_enable(GPIO_USB_OTG_EN);
	ret = gpio_request(GPIO_USB_OTG_EN, "GPIO_USB_OTG_EN");
	if (ret) {
		pr_err("%s: gpio_request() for USB_OTG_EN failed\n",
			__func__);
		return;
	}
	ret = gpio_direction_output(GPIO_USB_OTG_EN, 0);

	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));

	/*boost USB1 performance*/
	val = gizmo_readl(AHB_GIZMO_AHB_MEM);
	val |= ENB_FAST_REARBITRATE;
	gizmo_writel(val, AHB_GIZMO_AHB_MEM);

	val = gizmo_readl(AHB_GIZMO_USB);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB);

	val = gizmo_readl(AHB_GIZMO_USB3);
	val |= IMMEDIATE;
	gizmo_writel(val, AHB_GIZMO_USB3);


	val = gizmo_readl(AHB_ARBITRATION_PRIORITY_CTRL);
	val |= PRIORITY_SELEECT_USB | AHB_PRIORITY_WEIGHT(7) | PRIORITY_SELEECT_USB3;	
	gizmo_writel(val, AHB_ARBITRATION_PRIORITY_CTRL);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG1);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | MST_ID(0x5) | ADDR_BNDRY(0xC) | SPEC_THROTTLE(0) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG1);        

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG2);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB_MST_ID | ADDR_BNDRY(0xC) | SPEC_THROTTLE(0) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG2);

	val = gizmo_readl(AHB_MEM_PREFETCH_CFG3);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB | USB3_MST_ID| ADDR_BNDRY(0xC) | SPEC_THROTTLE(0) | INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(val, AHB_MEM_PREFETCH_CFG3);     

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
}

static int __init n1_hsic_init(void)
{
#ifdef CONFIG_SAMSUNG_LPM_MODE
	int ret = 0;

	if (!charging_mode_from_boot) {
		register_smd_resource();
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
		ret = platform_device_register(&tegra_ehci2_device);
	}
	return ret;
#else
	register_smd_resource();
	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	return platform_device_register(&tegra_ehci2_device);
#endif
}
late_initcall(n1_hsic_init);

static int n1_jack_init(void)
{
	int ret = 0;

	ret = gpio_request(GPIO_MICBIAS1_EN, "micbias_enabl");
	if (ret < 0) {
		gpio_free(GPIO_MICBIAS1_EN);
		return ret;
	}

	ret = gpio_request(GPIO_MICBIAS2_EN, "ear_micbias_enable");
	if (ret < 0) {
		gpio_free(GPIO_MICBIAS2_EN);
		return ret;
	}

	ret = gpio_direction_output(GPIO_MICBIAS1_EN, 0);
	if (ret < 0)
		goto cleanup;

	ret = gpio_direction_output(GPIO_MICBIAS2_EN, 0);
	if (ret < 0)
		goto cleanup;

	tegra_gpio_enable(GPIO_MICBIAS1_EN);
	tegra_gpio_enable(GPIO_MICBIAS2_EN);

	tegra_gpio_enable(GPIO_DET_3_5);
	tegra_gpio_enable(GPIO_EAR_SEND_END);

cleanup:
	gpio_free(GPIO_MICBIAS1_EN);
	gpio_free(GPIO_MICBIAS2_EN);
	return ret;

}

static void tegra_n1_codec_init(void)
{
	gpio_request(GPIO_CODEC_LDO_EN, "codec_ldo_en");
	tegra_gpio_enable(GPIO_CODEC_LDO_EN);
	gpio_direction_output(GPIO_CODEC_LDO_EN, 0);
	msleep(50);
	gpio_set_value(GPIO_CODEC_LDO_EN, 1);

	/* Earmic sel disable */
	gpio_request(GPIO_EAR_SEL, "ear_sel");
	tegra_gpio_enable(GPIO_EAR_SEL);
	gpio_direction_output(GPIO_EAR_SEL, 0);
}

static void n1_fsa9480_init(void)
{
	gpio_request(GPIO_UART_SEL, "uart_sel");
	gpio_request(GPIO_JACK_nINT, "jack_nint");

	//gpio_direction_output(GPIO_UART_SEL, 1);
	gpio_direction_input(GPIO_JACK_nINT);

	tegra_gpio_enable(GPIO_UART_SEL);
	tegra_gpio_enable(GPIO_JACK_nINT);
	if(system_rev >= 0x05)
	{
		gpio_request(GPIO_UART_SEL_EN, "uart_sel_en");
		gpio_direction_output(GPIO_UART_SEL_EN, 0);
		tegra_gpio_enable(GPIO_UART_SEL_EN);
	}
}

#ifdef CONFIG_PN544
static void n1_nfc_init(void)
{
	gpio_request(GPIO_NFC_IRQ, "nfc_irq");
	gpio_request(GPIO_NFC_EN, "nfc_en");
	gpio_request(GPIO_NFC_FIRMWARE, "nfc_firm");

	gpio_direction_input(GPIO_NFC_IRQ);
	gpio_direction_output(GPIO_NFC_EN, 0);
	gpio_direction_output(GPIO_NFC_FIRMWARE, 0);

	tegra_gpio_enable(GPIO_NFC_IRQ);
	tegra_gpio_enable(GPIO_NFC_EN);
	tegra_gpio_enable(GPIO_NFC_FIRMWARE);
}
#endif

static int __init n1_gps_init(void)
{
	int error;
	struct device *gps_dev;
	struct clk *clk32 = clk_get_sys(NULL, "blink");
	if (!IS_ERR(clk32)) {
		clk_set_rate(clk32, clk32->parent->rate);
		clk_enable(clk32);
	}

	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev)) {
		pr_err("Failed to create gps device!\n");
		error = PTR_ERR(gps_dev);
		goto err;
	}

	gpio_request(GPIO_GPS_NRST, "GPS_NRST");
	gpio_request(GPIO_GPS_EN, "GPS_EN");

	gpio_direction_output(GPIO_GPS_NRST, 1);
	gpio_direction_output(GPIO_GPS_EN, 0);

	tegra_gpio_enable(GPIO_GPS_NRST);
	tegra_gpio_enable(GPIO_GPS_EN);

	gpio_export(GPIO_GPS_NRST, 1);
	gpio_export(GPIO_GPS_EN, 1);

	gpio_export_link(gps_dev, "GPS_nRST", GPIO_GPS_NRST);
	gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_EN);

	return 0;

err:
	return error;
}

static struct i2c_board_info isa1200_i2c_vibrator_info[] = {
	{
		I2C_BOARD_INFO("isa1200",  0x48),
	},
};

static int __init n1_vibrator_init(void)
{
	i2c_register_board_info(17, isa1200_i2c_vibrator_info,
			ARRAY_SIZE(isa1200_i2c_vibrator_info));
	
	return 0;

}

static void n1_power_off(void)
{
	int ret;

	ret = max8907c_power_off();
	if (ret)
		pr_err("n1: failed to power off\n");

	while (1)
		;
}

static void __init n1_power_off_init(void)
{
	pm_power_off = n1_power_off;
}

#ifdef CONFIG_KERNEL_DEBUG_SEC
/* Debug level control */
static ssize_t show_sec_debug_level(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int sec_debug_level = kernel_sec_get_debug_level();
	char buffer[5];

	printk("%s %x\n", __func__, sec_debug_level);

	memcpy(buffer, &sec_debug_level, 4);
	buffer[4] = '\0';
	return sprintf(buf, "%s\n", buffer);
}

static ssize_t store_sec_debug_level(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int sec_debug_level = 0;
	memcpy(&sec_debug_level, buf, 4);

	printk("%s %x\n", __func__, sec_debug_level);

	if (!(sec_debug_level == KERNEL_SEC_DEBUG_LEVEL_LOW
			|| sec_debug_level == KERNEL_SEC_DEBUG_LEVEL_MID
			|| sec_debug_level == KERNEL_SEC_DEBUG_LEVEL_HIGH))
		return -EINVAL;

	kernel_sec_set_debug_level(sec_debug_level);

	return count;
}

static DEVICE_ATTR(sec_debug_level, 0644, show_sec_debug_level, store_sec_debug_level);
/* -- Debug level control */
#endif

static void __init tegra_n1_init(void)
{
	char serial[20];
	int ret = 0;

	tegra_common_init();
	tegra_clk_init_from_table(n1_clk_init_table);
	n1_pinmux_init();

	pr_info("Board HW Revision: system_rev = %d\n", system_rev);

	n1_i2c_init();

	snprintf(serial, sizeof(serial), "%llx", tegra_chip_uid());
	android_usb_pdata.serial_number = kstrdup(serial, GFP_KERNEL);
	tegra_i2s_device1.dev.platform_data = &tegra_audio_pdata[0];
	tegra_i2s_device2.dev.platform_data = &tegra_audio_pdata[1];
	tegra_spdif_device.dev.platform_data = &tegra_spdif_pdata;
	tegra_das_device.dev.platform_data = &tegra_das_pdata;
	tegra_ehci2_device.dev.platform_data = &n1_ehci2_ulpi_platform_data;
	platform_add_devices(n1_devices, ARRAY_SIZE(n1_devices));

#ifdef CONFIG_MAX8922_CHARGER
	if (system_rev >= HWREV_FOR_EXTERNEL_CHARGER)
		platform_device_register(&max8922_device_charger);
#endif

	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create sec class!\n");

	n1_jack_init();
	n1_sdhci_init();
#ifdef CONFIG_MHL_SII9234
	n1_mhl_gpio_init();
#endif
	n1_gpio_i2c_init(charging_mode_from_boot);
	n1_regulator_init();
	bose_touch_init();
	n1_keys_init();
#if defined (CONFIG_MACH_BOSE_ATT)
	n1_keys_stmpe1801b_init();
	hall_ic_init();
#endif
	n1_usb_init();
	n1_gps_init();
	n1_panel_init();
	n1_power_off_init();
	n1_emc_init();
	n1_rfkill_init();
	n1_sensors_init();
	tegra_n1_codec_init();
	if (system_rev == 0)
		n1_kbc_init();
	n1_fsa9480_init();
#ifdef CONFIG_PN544	
	n1_nfc_init();
#endif
	camera_init();
	init_wifi_mem();		// for wifi static buf
	n1_vibrator_init();

	register_reboot_notifier(&n1_reboot_notifier);

	/* console suspend disable */
	console_suspend_enabled = 0;

#ifdef CONFIG_KERNEL_DEBUG_SEC
	/* Add debug level node */
	struct device *platform = n1_devices[0]->dev.parent;
	ret = device_create_file(platform, &dev_attr_sec_debug_level);
	if (ret)
		printk("Fail to create sec_debug_level file\n");
#endif
}

int __init tegra_n1_protected_aperture_init(void)
{
	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_n1_protected_aperture_init);

void __init tegra_n1_reserve(void)
{
	u64 ram_console_start;
	int ret;

	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	if (system_rev < 2) /* 512 MB */
		tegra_reserve(SZ_128M, SZ_8M, SZ_16M);
	else	/* 1GB */
		tegra_reserve(SZ_256M, SZ_8M, SZ_16M);

	/* Reserve memory for the ram console. */
	ram_console_start = memblock_end_of_DRAM() - SZ_1M;

	ret = memblock_remove(ram_console_start, SZ_1M);
	if (ret < 0) {
		pr_err("Failed to reserve 0x%x bytes for ram_console at "
			"0x%llx, err = %d.\n",
			SZ_1M, ram_console_start, ret);
	} else {
		ram_console_resource[0].start = ram_console_start;
		ram_console_resource[0].end = ram_console_start + SZ_1M - 1;
	}
}

MACHINE_START(TEGRA_GENERIC, "n1")
	.boot_params    = 0x00000100,
	.phys_io        = IO_APB_PHYS,
	.io_pg_offst    = ((IO_APB_VIRT) >> 18) & 0xfffc,
	.init_irq       = tegra_init_irq,
	.init_machine   = tegra_n1_init,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_n1_reserve,
	.timer          = &tegra_timer,
MACHINE_END
