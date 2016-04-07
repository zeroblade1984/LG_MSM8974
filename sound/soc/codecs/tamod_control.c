/* 
 * WCD9320 Taiko Audio Mod Controls(TAMOD)
 * 
 * 	Author: Cezar Rey Templonuevo <zeroblade1984@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <sound/soc.h>
#include <linux/mfd/wcd9xxx/wcd9320_registers.h>
#include "tamod_control.h"

/* Variables Section */
bool hp_digigain_con = false;
bool hp_toggle = false;
u32 hp_digigain = 0x02;

struct snd_soc_codec *wcd9320_codec;

/* Headset */
static ssize_t hp_digigain_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	u32 usr = hp_digigain;

	sprintf(buf, "Status: %s\n", hp_digigain_con ? "on" : "off");
	sprintf(buf, "User: %u\n",usr);

	return strlen(buf);
}

static ssize_t hp_digigain_store(struct kobject *kobj, 
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	u32 usr;

	if (sysfs_streq(buf, "on")) {
		hp_digigain_con = true;

		if (hp_toggle) {
			taiko_write(wcd9320_codec, 
				TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL, hp_digigain);
			taiko_write(wcd9320_codec, 
				TAIKO_A_CDC_RX2_VOL_CTL_B2_CTL, hp_digigain);
		}

		return count;
	}

	if (sysfs_streq(buf, "off")) {
		hp_digigain_con = false;

		taiko_write(wcd9320_codec,
			TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL,
			TAIKO_A_CDC_RX_VOL_CTL_B2_CTL_DEF);

		taiko_write(wcd9320_codec,
			TAIKO_A_CDC_RX2_VOL_CTL_B2_CTL,
			TAIKO_A_CDC_RX_VOL_CTL_B2_CTL_DEF);

		return count;
	}

	if (sscanf(buf, "%d", &usr)) {
		if (usr > TAIKO_A_CDC_RXX_VOL_MASK)
			return -EINVAL;

		hp_digigain = usr;

		if (hp_toggle && hp_digigain_con) {
			taiko_write(wcd9320_codec, 
				TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL, hp_digigain);

			taiko_write(wcd9320_codec, 
				TAIKO_A_CDC_RX2_VOL_CTL_B2_CTL, hp_digigain);
		}

		return count;
	}

	return count;
}
static struct kobj_attribute hp_digigain_interface = 
	__ATTR(hp_digigain, 0644, hp_digigain_show, hp_digigain_store);

/* sysfs parts */
static struct attribute *taiko_control_attrs[] = {
	NULL,
};

static struct attribute_group taiko_control_interface_group = {
	.attrs = taiko_control_attrs,
};

static struct attribute *taiko_headset_attrs[] = {
	&hp_digigain_interface.attr,
	NULL,
};

static struct attribute_group taiko_headset_interface_group = {
	.attrs = taiko_headset_attrs,
	.name  = "headset",
};

static struct kobject *taiko_control_kobject;

static int __init taiko_control_init(void)
{
	int ret;

	taiko_control_kobject = kobject_create_and_add("taiko", kernel_kobj);
	if (!taiko_control_kobject) {
		pr_err("Taiko Control: Failed to create kobject interface\n");
	}
	ret = sysfs_create_group(taiko_control_kobject, &taiko_control_interface_group);
	if (ret) {
		kobject_put(taiko_control_kobject);
	}

	ret = sysfs_create_group(taiko_control_kobject, &taiko_headset_interface_group);
	if (ret) {
		pr_err("Taiko Control: Failed to create sysfs group(s)\n");
	}

	return ret;
}

static void __exit taiko_control_exit(void)
{
	kobject_put(taiko_control_kobject);

	return;
}

module_init(taiko_control_init);
module_exit(taiko_control_exit);

MODULE_DESCRIPTION("Taiko Audio Codec controller");
MODULE_AUTHOR("Cezar Rey Templonuevo");
MODULE_LICENSE("GPL v2");
