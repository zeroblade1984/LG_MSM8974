/*
 *  drivers/power/rt9428_battery.c
 *  Driver to Richtek RT9428 Fuelgauge IC
 *
 *  Copyright (C) 2014 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include <linux/power/rt9428_battery.h>
#ifdef CONFIG_LGE_PM_RT9428_SHUTDOWN_SOC_USED
#include <linux/spmi.h>
#endif
#ifdef CONFIG_LGE_PM_CHARGING_BQ24262_CHARGER
#include <linux/power/bq24262_charger.h>
#endif
#ifdef CONFIG_BQ24296_CHARGER
#include <linux/i2c/bq24296_charger.h>
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#ifdef CONFIG_64BIT
#include <soc/qcom/lge/board_lge.h>
#else
#include <mach/board_lge.h>
#endif
#endif

#ifdef CONFIG_LGE_PM_RT9428_POLLING
#define RT9428_POLLING_PERIOD         60000 /* 60 sec */
#define RT9428_POLLING_PERIOD_7       10000
#define RT9428_POLLING_PERIOD_3       5000
#endif
#define RT9428_SOC_RESCALING 93
#define SOC_EXCECPTION_PERIOD_BASE 30000 /* 30 sec */

struct rt9428_chip {
	struct i2c_client *i2c;
	struct device *dev;
	struct rt9428_platform_data *pdata;
	struct power_supply *batt_psy;
	struct power_supply fg_psy;
	struct delayed_work dwork;
	struct mutex io_lock;
	struct mutex var_lock;
	struct miscdevice rt9428_io_misc;
	int alert_irq;
	int last_capacity;	/* unit 0.1% */
	int capacity;		/* unit 0.1% */
	int last_vcell;
	int vcell;
	int qs_flag;
	unsigned char suspend:1;
	unsigned char online:1;
	unsigned char reg_addr;
	unsigned int reg_data;
	struct wake_lock update_lock;
	struct wake_lock irq_lock;
	bool update_lock_flag;
#ifdef CONFIG_LGE_PM_RT9428_POLLING
	struct wake_lock polling_lock;
	struct delayed_work polling_work;
#endif
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	struct delayed_work update_polling_work;
	struct mutex soc_update_lock;
#endif
};

struct rt9428_io_desc {
	unsigned char id[2];
	/* unsigned short vgcomp0; */
	unsigned int vgpara1_hcap;
	unsigned int vgpara2_hcap;
	unsigned int vgpara3_hcap;
	unsigned int vgpara4_hcap;
	unsigned int vgpara5_hcap;
	unsigned int vgpara1_lcap;
	unsigned int vgpara2_lcap;
	unsigned int vgpara3_lcap;
	unsigned int vgpara4_lcap;
	unsigned int vgpara5_lcap;
	int r1_gain_tempcold_hcap;
	int r1_gain_temphot_hcap;
	int r1_gain_tempcold2_hcap;
	int r2_gain_tempcold_hcap;
	int r2_gain_temphot_hcap;
	int r2_gain_tempcold2_hcap;
	int r3_gain_tempcold_hcap;
	int r3_gain_temphot_hcap;
	int r3_gain_tempcold2_hcap;
	int r4_gain_tempcold_hcap;
	int r4_gain_temphot_hcap;
	int r4_gain_tempcold2_hcap;
	int r1_gain_tempcold_lcap;
	int r1_gain_temphot_lcap;
	int r1_gain_tempcold2_lcap;
	int r2_gain_tempcold_lcap;
	int r2_gain_temphot_lcap;
	int r2_gain_tempcold2_lcap;
	int r3_gain_tempcold_lcap;
	int r3_gain_temphot_lcap;
	int r3_gain_tempcold2_lcap;
	int r4_gain_tempcold_lcap;
	int r4_gain_temphot_lcap;
	int r4_gain_tempcold2_lcap;
	unsigned int temp_base;
	unsigned int high_temp_base;
	unsigned int low_temp_base;
	unsigned int low_temp2_base;
	int soc_comp;
#ifdef CONFIG_LGE_PM_RT9428_CUT_OFF_UNDER_3P2_USED
	int low_cut_off_gain;
#endif
};

static unsigned char rt9428_init_regval[] = {
	0x5F,			/*0x0D */
};

//static char *rtdef_fg_name = "fuelgauge";

static char *rt_fg_supply_list[] = {
	"fg_psy",
};

static enum power_supply_property rt_fg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
};

#ifdef CONFIG_LGE_PM
static struct rt9428_chip *ref;
#endif

static inline int rt9428_read_device(struct i2c_client *i2c,
				     int reg, int bytes, void *dest)
{
	int ret;

	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret < 0)
			return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
	return ret;
}

/*default read one byte*/
static int rt9428_reg_read(struct i2c_client *i2c, int reg)
{
	struct rt9428_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0 || ret > 255) {
		pr_err("%s:read byte fail, reg:%x, ret:%x\n", __func__, reg, ret);
		ret = 0;
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}

#if 0
/*default write one byte*/
static int rt9428_reg_write(struct i2c_client *i2c, int reg, unsigned char data)
{
	struct rt9428_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	RT_DBG("I2C Write (client : 0x%x) reg = 0x%x, data = 0x%x\n",
	       (unsigned int)i2c, (unsigned int)reg, (unsigned int)data);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, data);
	mutex_unlock(&chip->io_lock);
	return ret;
}
#endif /* #if 0 */

static int rt9428_reg_read_word(struct i2c_client *i2c, int reg)
{
	struct rt9428_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	RT_DBG("I2C Read (client : 0x%x) reg = 0x%x\n",
	       (unsigned int)i2c, (unsigned int)reg);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_read_word_data(i2c, reg);
	mutex_unlock(&chip->io_lock);
	if (ret < 0)
		pr_err("%s:read reg 0x%x io fail\n", __func__, reg);
	return (ret < 0) ? ret : swab16(ret);
}

static int rt9428_reg_write_word(struct i2c_client *i2c, int reg,
				 unsigned int data)
{
	struct rt9428_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	RT_DBG("I2C Write (client : 0x%x) reg = 0x%x, data = 0x%x\n",
	       (unsigned int)i2c, (unsigned int)reg, (unsigned int)data);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_write_word_data(i2c, reg, swab16(data));
	mutex_unlock(&chip->io_lock);
	return ret;
}

#if 0
static int rt9428_assign_bits(struct i2c_client *i2c, int reg,
			      unsigned char mask, unsigned char data)
{
	struct rt9428_chip *chip = i2c_get_clientdata(i2c);
	unsigned char value;
	int ret;

	mutex_lock(&chip->io_lock);
	ret = rt9428_read_device(i2c, reg, 1, &value);
	if (ret < 0)
		goto out;
	value &= ~mask;
	value |= (data & mask);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}

static int rt9428_set_bits(struct i2c_client *i2c, int reg, unsigned char mask)
{
	return rt9428_assign_bits(i2c, reg, mask, mask);
}

static int rt9428_clr_bits(struct i2c_client *i2c, int reg, unsigned char mask)
{
	return rt9428_assign_bits(i2c, reg, mask, 0);
}
#endif /* #if 0 */

static void rt9428_update_info(struct rt9428_chip *chip);

bool rt9428_battery_present_checker(void)
{
#ifdef CONFIG_LGE_PM_CHARGING_BQ24262_CHARGER
	int batt_temp = 0;

	batt_temp = bq24262_get_prop_batt_temp_raw();
	if (batt_temp <= -300 || batt_temp >= 790)
		return false;
	else
		return true;
#endif
#ifdef CONFIG_BQ24296_CHARGER
	int batt_temp = 0;

	batt_temp = bq24296_get_batt_temp_origin();
	if (batt_temp <= -300 || batt_temp >= 790)
		return false;
	else
		return true;
#endif
	return true;
}

static int rt9428_get_capacity(struct rt9428_chip *chip)
{
	int capacity = 0;

	capacity = rt9428_reg_read(chip->i2c, RT9428_REG_SOCH);
	capacity *= 10;

	if (capacity < 0)
		capacity = -EIO;
	else {
#ifdef CONFIG_LGE_PM_BATTERY_RT9428_4P4V_USED
		/* 50% ~ */
		if (capacity > 500)
			capacity -= ((capacity - 500) * 120 / 5300);
#endif
		/* ~ 10% */
		if (capacity < 100) {
			capacity = (capacity - chip->pdata->soc_comp) * 100;
			capacity /= (100 - chip->pdata->soc_comp);
			if (capacity > 0) {
				capacity = DIV_ROUND_UP(capacity, 10);
			}
			else
				capacity = 0;
		}
		/* 10% ~  */
		else {
			capacity = DIV_ROUND_UP(capacity, 10);
			/* soc rescaling */
			capacity = ((capacity*100000)/RT9428_SOC_RESCALING)/1000;
		}

		capacity = capacity > 100 ? 100 : capacity;
		capacity = capacity < 0   ? 0 : capacity;

		if (!chip->last_capacity)
			chip->last_capacity = chip->capacity = capacity;
	}
	return capacity;
}

static int rt9428_get_vcell(struct rt9428_chip *chip)
{
	int regval;
	int rc;
	bool batt_present;

	batt_present = rt9428_battery_present_checker();
	if (!batt_present) {
		pr_err("%s: no battery, skip get_vcell\n", __func__);
		return RT9428_DEFAULT_VCELL;
	}

	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_VBATH);
	if (regval < 0)
		rc = -EIO;
	else {
		if (!chip->last_vcell)
			chip->last_vcell = chip->vcell =
			    ((regval >> 4) * 5) >> 2;
		rc = ((regval >> 4) * 1250) / 1000;
	}
	return rc;
}

#ifdef CONFIG_LGE_PM
static void external_charger_enable(bool enable)
{
	union power_supply_propval val = {0,};
	val.intval = enable;
	ref->batt_psy->set_property(ref->batt_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
}
#endif

static void rt9428_execute_qs_command(struct rt9428_chip *chip)
{
	unsigned int regval;
	struct power_supply *psy;
	union power_supply_propval pval;
	int soc1, soc2;
	int rc;
	dev_info(chip->dev, "%s: ++\n", __func__);
	/* get charger power supply first */
	psy = power_supply_get_by_name("ac");
	if (!psy) {
		pr_err("%s:cannot get charger supply\n", __func__);
		goto qs_error;
	}
	/* check RI is 1 or 0 */
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_STATUS);
	if (regval < 0) {
		pr_err("%s:i2c read fail\n", __func__);
		goto qs_error;
	}
	/* If RI = 1, clear it first */
	if (regval & 0x01) {
		regval &= (~0x01);
		rt9428_reg_write_word(chip->i2c, RT9428_REG_STATUS, regval);
	}
	/* write the CVS */
	regval = chip->pdata->vgpara5_hcap;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_MFAH, regval);

	/* check the ta is existed or not */
	rc = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &pval);
	if (rc < 0) {
		pr_err("%s:get charger online fail\n", __func__);
		goto qs_error;
	} else {
		if (pval.intval)
			goto with_ta;
		else
			goto without_ta;
	}
with_ta:
	pr_err("%s:with TA process\n", __func__);
	/* set ADC */
	regval = 0x5AAA;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_MFAH, regval);
	regval = 0x5A5A;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_MFAH, regval);
	/* delay 100ms */
	mdelay(100);
	/* set QS */
	regval = 0x4000;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_CTRLH, regval);
	/* delay 50ms */
	mdelay(50);
	/* read soc 1 */
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_SOCH);
	if (regval < 0) {
		pr_err("%s:get soc1 fail\n", __func__);
		goto qs_error;
	} else
		soc1 = regval >> 8;
	/* turn off charger */
	external_charger_enable(0);
	if (rc < 0) {
		pr_err("%s:disable charger fail\n", __func__);
		goto qs_error;
	}
	/* set ADC again */
	regval = 0x5AAA;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_MFAH, regval);
	regval = 0x5A5A;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_MFAH, regval);
	/* delay 100ms */
	mdelay(100);
	/* set QS */
	regval = 0x4000;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_CTRLH, regval);
	/* delay 50ms */
	mdelay(50);
	/* read soc 2 */
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_SOCH);
	if (regval < 0) {
		pr_err("%s:get soc1 fail\n", __func__);
		goto qs_error;
	} else
		soc2 = regval >> 8;
	/* turn on charger again */
	external_charger_enable(1);
	/* get final soc by ratio */
	pr_err("%s:soc1 = %d\n", __func__, soc1);
	pr_err("%s:soc2 = %d\n", __func__, soc2);
	/* this compensation code is for charging current 1.0A, 1.2A  */
	soc1 = (soc2 *4 + soc1)/5;
	pr_err("%s:soc_new = %d\n", __func__, soc1);
	goto update_soc_new;
without_ta:
	pr_err("%s:without TA process\n", __func__);
	if (regval < 0) {
		pr_err("%s:get soc1 fail\n", __func__);
		goto qs_error;
	}
	/* set QS */
	regval = 0x4000;
	rt9428_reg_write_word(chip->i2c, RT9428_REG_CTRLH, regval);

	/* delay 50ms */
	mdelay(50);

	/* read soc */
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_SOCH);
	if (regval < 0) {
		pr_err("%s:get soc1 fail\n", __func__);
		goto qs_error;
	} else
		soc1 = regval >> 8;
	pr_err("%s:soc_new = %d\n", __func__, soc1);
update_soc_new:
	/* update the new soc */
	regval = 0x8600 | (soc1 & 0xFF);
	rc = rt9428_reg_write_word(chip->i2c, RT9428_REG_MFAH, regval);
	if (rc < 0) {
		pr_err("%s:write the new soc fail\n", __func__);
		goto qs_error;
	}
	chip->qs_flag = 1;
	rt9428_update_info(chip);
	chip->qs_flag = 0;

	pr_err("%s:soc quick sensing done\n", __func__);
qs_error:
	pr_err("%s: --\n", __func__);
}

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static ssize_t rt_fg_show_attrs_factory(struct device *, struct device_attribute *,
				char *);
static ssize_t rt_fg_store_attrs_factory(struct device *, struct device_attribute *,
				 const char *, size_t count);

#define RT_FG_ATTR_FACTORY(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0664},		\
	.show = rt_fg_show_attrs_factory,			\
	.store = rt_fg_store_attrs_factory,			\
}

static struct device_attribute rt_fuelgauge_attrs_factory[] = {
	RT_FG_ATTR_FACTORY(capacity),
	RT_FG_ATTR_FACTORY(voltage),
	RT_FG_ATTR_FACTORY(fuelrst),
};

enum {
	FG_CAPACITY = 0,
	FG_VOLTAGE,
	FG_FUELRST,
};

static ssize_t rt_fg_show_attrs_factory(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - rt_fuelgauge_attrs_factory;
	int i = 0;
	bool batt_present = rt9428_battery_present_checker();

	switch (offset) {
	case FG_CAPACITY:
		if (batt_present) {
			disable_irq(gpio_to_irq(ref->pdata->alert_gpio));
			i = rt9428_get_capacity(ref);
			enable_irq(gpio_to_irq(ref->pdata->alert_gpio));
		}
		else {
			pr_err("%s: no battery, set RT9428_DEFAULT_SOC\n",
					__func__);
			i = RT9428_DEFAULT_SOC;
		}
		break;
	case FG_VOLTAGE:
		if (batt_present) {
			disable_irq(gpio_to_irq(ref->pdata->alert_gpio));
			i = rt9428_get_vcell(ref);
			enable_irq(gpio_to_irq(ref->pdata->alert_gpio));
		}
		else {
			pr_err("%s: no battery, set RT9428_DEFAULT_VCELL\n",
					__func__);
			i = RT9428_DEFAULT_VCELL;
		}
		break;
	case FG_FUELRST:
	default:
		i = -EINVAL;
		break;
	}

	i = snprintf(buf, PAGE_SIZE, "%d\n", i);

	return i;
}

static ssize_t rt_fg_store_attrs_factory(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - rt_fuelgauge_attrs_factory;
	int ret = 0;
	bool batt_present = rt9428_battery_present_checker();

	switch (offset) {
	case FG_FUELRST:
		if (strncmp(buf, "reset", 5) == 0) {
			if (batt_present) {
				disable_irq(gpio_to_irq(ref->pdata->alert_gpio));
				rt9428_execute_qs_command(ref);
				enable_irq(gpio_to_irq(ref->pdata->alert_gpio));
				ret = count;
			}
			else {
				pr_err("%s: no battery, skip FG_FUELRST\n", __func__);
				ret = -EINVAL;
			}
		}
		else
			ret = -EINVAL;
		break;
	case FG_CAPACITY:
	case FG_VOLTAGE:
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rt_fg_create_attrs_factory(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(rt_fuelgauge_attrs_factory); i++) {
		rc = device_create_file(dev, &rt_fuelgauge_attrs_factory[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	pr_err("%s:failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &rt_fuelgauge_attrs_factory[i]);
create_attrs_succeed:
	return rc;
}
#endif

static ssize_t rt_fg_show_attrs(struct device *, struct device_attribute *,
				char *);
static ssize_t rt_fg_store_attrs(struct device *, struct device_attribute *,
				 const char *, size_t count);

#define RT_FG_ATTR(_name)				\
{							\
	.attr = {.name = #_name, .mode = 0664},		\
	.show = rt_fg_show_attrs,			\
	.store = rt_fg_store_attrs,			\
}

static struct device_attribute rt_fuelgauge_attrs[] = {
	RT_FG_ATTR(reg),
	RT_FG_ATTR(data),
	RT_FG_ATTR(regs),
	RT_FG_ATTR(qsense),
};

enum {
	FG_REG = 0,
	FG_DATA,
	FG_REGS,
	FG_QSENSE,
};

static ssize_t rt_fg_show_attrs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rt9428_chip *chip = dev_get_drvdata(dev->parent);
	const ptrdiff_t offset = attr - rt_fuelgauge_attrs;
	int i = 0;
	int j = 0;

	switch (offset) {
	case FG_REG:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%02x\n",
			       chip->reg_addr);
		break;
	case FG_DATA:
		chip->reg_data =
		    rt9428_reg_read_word(chip->i2c, chip->reg_addr);
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%04x\n",
			       chip->reg_data);
		dev_dbg(dev, "%s: (read) addr = 0x%x, data = 0x%x\n", __func__,
			chip->reg_addr, chip->reg_data);
		break;
	case FG_REGS:
		for (j = RT9428_REG_RANGE1_START; j <= RT9428_REG_RANGE1_STOP;
		     j++)
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "reg%02x 0x%02x\n", j,
				       rt9428_reg_read(chip->i2c, j));

		for (j = RT9428_REG_RANGE2_START; j <= RT9428_REG_RANGE2_STOP;
		     j++)
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "reg%02x 0x%02x\n", j,
				       rt9428_reg_read(chip->i2c, j));
		break;
	case FG_QSENSE:
	default:
		i = -EINVAL;
		break;
	}

	return i;
}

static ssize_t rt_fg_store_attrs(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct rt9428_chip *chip = dev_get_drvdata(dev->parent);
	const ptrdiff_t offset = attr - rt_fuelgauge_attrs;
	int ret = 0;
	int x = 0;

	switch (offset) {
	case FG_REG:
		if (sscanf(buf, "%x\n", &x) == 1) {
			if (x < RT9428_REG_MAX &&
			    ((x >= RT9428_REG_RANGE1_START
			      && x <= RT9428_REG_RANGE1_STOP)
			     || (x >= RT9428_REG_RANGE2_START
				 && x <= RT9428_REG_RANGE2_STOP))) {
				chip->reg_addr = x;
				ret = count;
			} else
				ret = -EINVAL;
		} else
			ret = -EINVAL;
		break;
	case FG_DATA:
		if (sscanf(buf, "%x\n", &x) == 1) {
			rt9428_reg_write_word(chip->i2c, chip->reg_addr, x);
			chip->reg_data = x;
			dev_dbg(dev, "%s: (write) addr = 0x%x, data = 0x%x\n",
				__func__, chip->reg_addr, chip->reg_data);
			/* for update immediately */
			chip->qs_flag = 1;
			rt9428_update_info(chip);
			chip->qs_flag = 0;

			ret = count;
		} else
			ret = -EINVAL;
		break;
	case FG_QSENSE:
		if (sscanf(buf, "%x\n", &x) == 1 && x == 1) {
			rt9428_execute_qs_command(chip);
			ret = count;
		}
		else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rt_fg_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(rt_fuelgauge_attrs); i++) {
		rc = device_create_file(dev, &rt_fuelgauge_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	goto create_attrs_succeed;

create_attrs_failed:
	pr_err("%s:failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &rt_fuelgauge_attrs[i]);
create_attrs_succeed:
	return rc;
}

static int rt_fg_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct rt9428_chip *chip = dev_get_drvdata(psy->dev->parent);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (chip->suspend) {
			if (chip->vcell != 0)
				val->intval = chip->vcell;
			else
				val->intval = RT9428_DEFAULT_VCELL;
		}
		else
			val->intval = rt9428_get_vcell(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		/*now casually return a constant value */
#ifdef CONFIG_LGE_PM_BATTERY_RT9428_4P4V_USED
		val->intval = 4400000;
#else
		val->intval = 4350000;
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->pdata->full_design;
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int rt_fg_set_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static void rt9428_update_info(struct rt9428_chip *chip)
{
	int btemp, original_btemp;
	int regval, capacity, capacity_raw, diff_capa = 0;
	struct rt9428_platform_data *pdata = chip->pdata;

	if (chip->update_lock_flag)
		pr_err("%s: exception, try update again.\n", __func__);
	else
		wake_lock(&chip->update_lock);

	/* wake_unlock irq_lock */
	if (wake_lock_active(&chip->irq_lock))
		wake_unlock(&chip->irq_lock);

	/*always clr ALRT */
	pr_debug("%s: ++\n", __func__);
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_CFG0);
	regval &= (~RT9428_SOCALRT_MASK);
	rt9428_reg_write_word(chip->i2c, RT9428_REG_CFG0, regval);
	/*get battery voltage store to chip->vcell */
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_VBATH);
	if (regval < 0) {
		if (wake_lock_active(&chip->update_lock))
			wake_unlock(&chip->update_lock);
		pr_err("%s:read vcell fail, release update_lock\n", __func__);
		return;
	}
	chip->vcell = ((regval >> 4) * 5) >> 2;
	/*get battery temp from battery power supply */
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		btemp = 250;
	} else {
		union power_supply_propval pval;
		chip->batt_psy->get_property(chip->batt_psy, POWER_SUPPLY_PROP_TEMP, &pval);
		btemp = pval.intval;
	}
	btemp /= 10;
	original_btemp = btemp;
	/* adjust temp for calculation vgcomp */
	if (original_btemp < (int)pdata->low_temp2_base)
		btemp = 0;
	else if (original_btemp >= (int)pdata->high_temp_base)
		btemp = 45;
	else if (original_btemp <= (int)pdata->low_temp_base)
		btemp = 5;
	else
		btemp = 25;
	pr_err("%s: original_temp=%d, btemp=%d\n", __func__, original_btemp,
		 btemp);

	mutex_lock(&chip->var_lock);
	/*adjust vgcom */
	{
		s32 temp_para1 = 0;
		s32 temp_para2 = 0;
		s32 temp_para3 = 0;
		s32 temp_para4 = 0;
		u16 data = 0;
		s32 temp_diff =
		    (btemp >
		     pdata->temp_base) ? btemp -
		    pdata->temp_base : pdata->temp_base - btemp;

		capacity_raw = rt9428_reg_read_word(chip->i2c, RT9428_REG_SOCH);
		capacity_raw = (capacity_raw * 10) >> 8;

		if (capacity_raw > 500) {
			/* Read RCOMP applied */
			temp_para1 = (pdata->vgpara1_hcap) & 0x00ff;
			temp_para2 = (pdata->vgpara2_hcap) & 0x00ff;
			temp_para3 = (pdata->vgpara3_hcap) & 0x00ff;
			temp_para4 = (pdata->vgpara4_hcap) & 0x00ff;
			/* Calculate RCOMP by temperature */
			if (btemp <= pdata->low_temp2_base) {
				temp_para1 = temp_para1 + temp_diff
				    * pdata->r1_gain_tempcold2_hcap / 100;
				temp_para2 = temp_para2 + temp_diff
				    * pdata->r2_gain_tempcold2_hcap / 100;
				temp_para3 = temp_para3 + temp_diff
				    * pdata->r3_gain_tempcold2_hcap / 100;
				temp_para4 = temp_para4 + temp_diff
				    * pdata->r4_gain_tempcold2_hcap / 100;
			} else if (btemp > pdata->temp_base) {
				temp_para1 = temp_para1 + temp_diff
				    * pdata->r1_gain_temphot_hcap / 100;
				temp_para2 = temp_para2 + temp_diff
				    * pdata->r2_gain_temphot_hcap / 100;
				temp_para3 = temp_para3 + temp_diff
				    * pdata->r3_gain_temphot_hcap / 100;
				temp_para4 = temp_para4 + temp_diff
				    * pdata->r4_gain_temphot_hcap / 100;
			} else {
				temp_para1 = temp_para1 + temp_diff
				    * pdata->r1_gain_tempcold_hcap / 100;
				temp_para2 = temp_para2 + temp_diff
				    * pdata->r2_gain_tempcold_hcap / 100;
				temp_para3 = temp_para3 + temp_diff
				    * pdata->r3_gain_tempcold_hcap / 100;
				temp_para4 = temp_para4 + temp_diff
				    * pdata->r4_gain_tempcold_hcap / 100;
			}
		} else {
			/* Read RCOMP applied */
			temp_para1 = (pdata->vgpara1_lcap) & 0x00ff;
			temp_para2 = (pdata->vgpara2_lcap) & 0x00ff;
			temp_para3 = (pdata->vgpara3_lcap) & 0x00ff;
			temp_para4 = (pdata->vgpara4_lcap) & 0x00ff;
			/* Calculate RCOMP by temperature */
			if (btemp <= pdata->low_temp2_base) {
				temp_para1 = temp_para1 + temp_diff
				    * pdata->r1_gain_tempcold2_lcap / 100;
				temp_para2 = temp_para2 + temp_diff
				    * pdata->r2_gain_tempcold2_lcap / 100;
				temp_para3 = temp_para3 + temp_diff
				    * pdata->r3_gain_tempcold2_lcap / 100;
				temp_para4 = temp_para4 + temp_diff
				    * pdata->r4_gain_tempcold2_lcap / 100;
			} else if (btemp > pdata->temp_base) {
				temp_para1 = temp_para1 + temp_diff
				    * pdata->r1_gain_temphot_lcap / 100;
				temp_para2 = temp_para2 + temp_diff
				    * pdata->r2_gain_temphot_lcap / 100;
				temp_para3 = temp_para3 + temp_diff
				    * pdata->r3_gain_temphot_lcap / 100;
				temp_para4 = temp_para4 + temp_diff
				    * pdata->r4_gain_temphot_lcap / 100;
			} else {
				temp_para1 = temp_para1 + temp_diff
				    * pdata->r1_gain_tempcold_lcap / 100;
				temp_para2 = temp_para2 + temp_diff
				    * pdata->r2_gain_tempcold_lcap / 100;
				temp_para3 = temp_para3 + temp_diff
				    * pdata->r3_gain_tempcold_lcap / 100;
				temp_para4 = temp_para4 + temp_diff
				    * pdata->r4_gain_tempcold_lcap / 100;
			}
		}
		/* RCOMP limitation */
		if (temp_para1 > 0xFF)
			temp_para1 = 0xFF;
		else if (temp_para1 < 0)
			temp_para1 = 0;
		if (temp_para2 > 0xFF)
			temp_para2 = 0xFF;
		else if (temp_para2 < 0)
			temp_para2 = 0;
		if (temp_para3 > 0xFF)
			temp_para3 = 0xFF;
		else if (temp_para3 < 0)
			temp_para3 = 0;
		if (temp_para4 > 0xFF)
			temp_para4 = 0xFF;
		else if (temp_para4 < 0)
			temp_para4 = 0;
		if (capacity_raw > 500) {
			/* Write RCOMP */
			data = (pdata->vgpara1_hcap & 0xff00) + temp_para1;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG1\n", __func__);

			data = (pdata->vgpara2_hcap & 0xff00) + temp_para2;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG2\n", __func__);

			data = (pdata->vgpara3_hcap & 0xff00) + temp_para3;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG3\n", __func__);

			data = (pdata->vgpara4_hcap & 0xff00) + temp_para4;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG4\n", __func__);

			data = pdata->vgpara5_hcap;
			if ((original_btemp < pdata->low_temp_base)
			    && (original_btemp >= pdata->low_temp2_base))
				data = (pdata->vgpara5_hcap & 0xff01) + 0x02;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG5\n", __func__);
		}
#ifdef CONFIG_LGE_PM_RT9428_CUT_OFF_UNDER_3P2_USED
		else if (capacity_raw < 100) {
			/* Write RCOMP */
			data = (pdata->vgpara1_lcap & 0xff00) + temp_para1;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG1\n", __func__);

			/* Adjust the discharge rate by gain factor */
			temp_para2 = (temp_para2 * pdata->low_cut_off_gain + 5) / 10;
			data = (pdata->vgpara2_lcap & 0xff00) + temp_para2;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG2\n", __func__);

			data = (pdata->vgpara3_lcap & 0xff00) + temp_para3;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG3\n", __func__);

			data = (pdata->vgpara4_lcap & 0xff00) + temp_para4;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG4\n", __func__);

			data = pdata->vgpara5_lcap;
			if ((original_btemp < pdata->low_temp_base)
			    && (original_btemp >= pdata->low_temp2_base))
				data = (pdata->vgpara5_lcap & 0xff01) + 0x02;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG5\n", __func__);
		}
#endif
		else {
			/* Write RCOMP */
			data = (pdata->vgpara1_lcap & 0xff00) + temp_para1;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG1\n", __func__);

			data = (pdata->vgpara2_lcap & 0xff00) + temp_para2;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG2\n", __func__);

			data = (pdata->vgpara3_lcap & 0xff00) + temp_para3;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG3\n", __func__);

			data = (pdata->vgpara4_lcap & 0xff00) + temp_para4;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG4\n", __func__);

			data = pdata->vgpara5_lcap;
			if ((original_btemp < pdata->low_temp_base)
			    && (original_btemp >= pdata->low_temp2_base))
				data = (pdata->vgpara5_lcap & 0xff01) + 0x02;
			if (rt9428_reg_write_word
			    (chip->i2c, RT9428_REG_MFAH, data) < 0)
				pr_err("%s:failed to write VG5\n", __func__);
		}
	}
	mutex_unlock(&chip->var_lock);
	/*get battery soc and store to chip->capacity */
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	mutex_lock(&chip->soc_update_lock);
#endif
	chip->last_capacity = chip->capacity;
	chip->last_vcell = chip->vcell;
	capacity = rt9428_get_capacity(chip);
	chip->vcell= rt9428_get_vcell(chip);

	pr_err("%s: last_capa:%d. capa:%d\n", __func__,
			chip->last_capacity, capacity);

	diff_capa = abs(chip->last_capacity - capacity);

	if ( diff_capa > 1 && !chip->qs_flag ) {
		int PERIOD = 0;

		chip->update_lock_flag = true;
		pr_err("%s: soc update exception!, weight:%d\n", __func__, diff_capa);
		if (chip->last_capacity - capacity < 0)
			(chip->capacity)++;
		else
			(chip->capacity)--;

		cancel_delayed_work(&chip->dwork);
		diff_capa = (diff_capa > 4) ?
			SOC_EXCECPTION_PERIOD_BASE : diff_capa;
		PERIOD = SOC_EXCECPTION_PERIOD_BASE
			+ SOC_EXCECPTION_PERIOD_BASE/diff_capa;
		schedule_delayed_work(&chip->dwork,	msecs_to_jiffies(PERIOD));
	}
	else {
		chip->capacity = capacity;
		chip->update_lock_flag = false;
	}
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
		mutex_unlock(&chip->soc_update_lock);
#endif
		if (wake_lock_active(&chip->update_lock))
			wake_unlock(&chip->update_lock);
		return;
	}
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	mutex_unlock(&chip->soc_update_lock);
#endif
	if (chip->update_lock_flag == false) {
		if (wake_lock_active(&chip->update_lock))
			wake_unlock(&chip->update_lock);
	}

	power_supply_changed(chip->batt_psy);
	pr_debug("%s: --\n", __func__);
}
#ifdef CONFIG_LGE_PM_RT9428_POLLING
void rt9428_reg_info(struct rt9428_chip *chip)
{
	int i=0, j;
	char buf[180]= {0,};

	for (j = RT9428_REG_RANGE1_START; j <= RT9428_REG_RANGE1_STOP; j++)
	{
		if (j == RT9428_REG_RANGE1_STOP)
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "%02x", rt9428_reg_read(chip->i2c, j));
		else
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "%02x:", rt9428_reg_read(chip->i2c, j));
	}
	pr_err("%s:[2h~Fh] %s\n", __func__, buf);
}

static void rt9428_polling_work(struct work_struct *work)
{
	struct rt9428_chip *chip;
	union power_supply_propval soc;

	int voltage_now = 0;
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	int regval = 0;
	bool alert_bit = 0;
#endif
	chip = container_of(work, struct rt9428_chip, polling_work.work);

	if (chip == NULL) {
		RT_DBG(":chip pointer is null.\n");
		return;
	}

	if (chip->suspend) {
		pr_err("%s:suspend, no read rt9428_reg\n", __func__);
		goto reschedule_work;
	}

	rt9428_reg_info(chip);
	voltage_now = rt9428_get_vcell(chip);
	chip->batt_psy->get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &soc);
	pr_err("%s:[polling_work] voltage_now : %dmV / capacity : %d%%\n"
			,__func__ ,voltage_now ,soc.intval);

#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_CFG0);
	/* IRQ_PIN_LOW W/A case of 4.33V ~ */
	alert_bit = (regval & RT9428_SOCALRT_MASK) >> 5;
	if (alert_bit == 1) {
		regval &= (~RT9428_SOCALRT_MASK);
		rt9428_reg_write_word(chip->i2c, RT9428_REG_CFG0, regval);
		pr_err("%s:IRQ_LOW_WA, clear alert bit.\n", __func__);
	}
#endif
reschedule_work:
	if (3 < soc.intval && soc.intval < 8)
		schedule_delayed_work(&chip->polling_work,
				msecs_to_jiffies(RT9428_POLLING_PERIOD_7));
	else if (soc.intval <= 3)
		schedule_delayed_work(&chip->polling_work,
				msecs_to_jiffies(RT9428_POLLING_PERIOD_3));
	else
		schedule_delayed_work(&chip->polling_work,
				msecs_to_jiffies(RT9428_POLLING_PERIOD));
}
#endif

#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
static void rt9428_update_polling_work(struct work_struct *work)
{
	struct rt9428_chip *chip;
	int capacity = 0, capa_diff = 0;

	chip = container_of(work, struct rt9428_chip, update_polling_work.work);

	if ( chip->capacity <= 5 ) {
		pr_err("%s: ++\n", __func__);
		capacity = rt9428_get_capacity(chip);
		capa_diff = abs(chip->capacity-capacity);
		schedule_delayed_work(&chip->update_polling_work,
				msecs_to_jiffies(SOC_EXCECPTION_PERIOD_BASE));
		if (capa_diff >= 1) {
			schedule_delayed_work(&chip->dwork, msecs_to_jiffies(100));
			pr_err("%s: soc_update under 5%%\n", __func__);
		}
	}
}
#endif

static irqreturn_t rt9428_irq_handler(int irqno, void *param)
{
	struct rt9428_chip *chip = (struct rt9428_chip *)param;

	pr_debug("%s: ++\n", __func__);

	/* for guarantee of irq function */
	/* It could enter system_pm_sleep before running dwork */
	wake_lock(&chip->irq_lock);

	if (chip->suspend) {
		pr_err("%s: run dwork after 20msec\n", __func__);
		schedule_delayed_work(&chip->dwork, msecs_to_jiffies(20));
		goto intr_end;
	}
	else
		rt9428_update_info(chip);

intr_end:
	pr_debug("%s: --\n", __func__);
	return IRQ_HANDLED;
}

static void rt9428_dwork_func(struct work_struct *work)
{
	struct rt9428_chip *chip =
	    (struct rt9428_chip *)container_of(work, struct rt9428_chip,
					       dwork.work);

	/* need to check system_pm_suspend or resume */
	if (chip->suspend) {
		pr_err("%s:suspend, run dwork after 20msec again\n", __func__);
		schedule_delayed_work(&chip->dwork, msecs_to_jiffies(20));
		return;
	}
	else {
	/* update rt9428 information */
		rt9428_update_info(chip);
	}
	pr_err("%s: Done\n", __func__);
}

static ssize_t rt9428_io_read(struct file *filp, char __user *buf,
			      size_t buf_size, loff_t *ppos)
{
	struct rt9428_chip *chip = container_of(filp->private_data,
						struct rt9428_chip,
						rt9428_io_misc);
	struct rt9428_io_desc *rid;
	int ret = 0;

	dev_info(chip->dev, "%s bufsize = %d\n", __func__, buf_size);
	rid = devm_kzalloc(chip->dev, sizeof(*rid), GFP_KERNEL);
	if (!rid) {
		pr_err("%s:err allocation for buf\n", __func__);
		return -ENOMEM;
	}
	if (buf_size < sizeof(*rid)) {
		pr_err("%s:buf size is smaller than io_desc\n", __func__);
		ret = -EINVAL;
		goto err_read;
	}
	if (*ppos != 0)
		goto err_read;

	rid->id[0] = 'r';
	rid->id[1] = 't';
	rid->vgpara1_hcap = chip->pdata->vgpara1_hcap;
	rid->vgpara2_hcap = chip->pdata->vgpara2_hcap;
	rid->vgpara3_hcap = chip->pdata->vgpara3_hcap;
	rid->vgpara4_hcap = chip->pdata->vgpara4_hcap;
	rid->vgpara5_hcap = chip->pdata->vgpara5_hcap;
	rid->vgpara1_lcap = chip->pdata->vgpara1_lcap;
	rid->vgpara2_lcap = chip->pdata->vgpara2_lcap;
	rid->vgpara3_lcap = chip->pdata->vgpara3_lcap;
	rid->vgpara4_lcap = chip->pdata->vgpara4_lcap;
	rid->vgpara5_lcap = chip->pdata->vgpara5_lcap;
	rid->r1_gain_tempcold_hcap = chip->pdata->r1_gain_tempcold_hcap;
	rid->r1_gain_temphot_hcap = chip->pdata->r1_gain_temphot_hcap;
	rid->r1_gain_tempcold2_hcap = chip->pdata->r1_gain_tempcold2_hcap;
	rid->r2_gain_tempcold_hcap = chip->pdata->r2_gain_tempcold_hcap;
	rid->r2_gain_temphot_hcap = chip->pdata->r2_gain_temphot_hcap;
	rid->r2_gain_tempcold2_hcap = chip->pdata->r2_gain_tempcold2_hcap;
	rid->r3_gain_tempcold_hcap = chip->pdata->r3_gain_tempcold_hcap;
	rid->r3_gain_temphot_hcap = chip->pdata->r3_gain_temphot_hcap;
	rid->r3_gain_tempcold2_hcap = chip->pdata->r3_gain_tempcold2_hcap;
	rid->r4_gain_tempcold_hcap = chip->pdata->r4_gain_tempcold_hcap;
	rid->r4_gain_temphot_hcap = chip->pdata->r4_gain_temphot_hcap;
	rid->r4_gain_tempcold2_hcap = chip->pdata->r4_gain_tempcold2_hcap;
	rid->r1_gain_tempcold_lcap = chip->pdata->r1_gain_tempcold_lcap;
	rid->r1_gain_temphot_lcap = chip->pdata->r1_gain_temphot_lcap;
	rid->r1_gain_tempcold2_lcap = chip->pdata->r1_gain_tempcold2_lcap;
	rid->r2_gain_tempcold_lcap = chip->pdata->r2_gain_tempcold_lcap;
	rid->r2_gain_temphot_lcap = chip->pdata->r2_gain_temphot_lcap;
	rid->r2_gain_tempcold2_lcap = chip->pdata->r2_gain_tempcold2_lcap;
	rid->r3_gain_tempcold_lcap = chip->pdata->r3_gain_tempcold_lcap;
	rid->r3_gain_temphot_lcap = chip->pdata->r3_gain_temphot_lcap;
	rid->r3_gain_tempcold2_lcap = chip->pdata->r3_gain_tempcold2_lcap;
	rid->r4_gain_tempcold_lcap = chip->pdata->r4_gain_tempcold_lcap;
	rid->r4_gain_temphot_lcap = chip->pdata->r4_gain_temphot_lcap;
	rid->r4_gain_tempcold2_lcap = chip->pdata->r4_gain_tempcold2_lcap;
	rid->temp_base = chip->pdata->temp_base;
	rid->high_temp_base = chip->pdata->high_temp_base;
	rid->low_temp_base = chip->pdata->low_temp_base;
	rid->low_temp2_base = chip->pdata->low_temp2_base;
	rid->soc_comp = chip->pdata->soc_comp;
#ifdef CONFIG_LGE_PM_RT9428_CUT_OFF_UNDER_3P2_USED
	rid->low_cut_off_gain = chip->pdata->low_cut_off_gain;
#endif
	if (copy_to_user(buf, rid, sizeof(*rid)) > 0) {
		pr_err("%s:copy to user buf fail\n", __func__);
		ret = -EINVAL;
		goto err_read;
	}
	*ppos = sizeof(*rid);
	ret = *ppos;
err_read:
	devm_kfree(chip->dev, rid);
	return ret;
}

static ssize_t rt9428_io_write(struct file *filp, const char __user *buf,
			       size_t buf_size, loff_t *offset)
{
	struct rt9428_chip *chip = container_of(filp->private_data,
						struct rt9428_chip,
						rt9428_io_misc);
	struct rt9428_io_desc *rid;
	int ret = 0;

	dev_info(chip->dev, "%s bufsize = %d\n", __func__, buf_size);
	rid = devm_kzalloc(chip->dev, sizeof(*rid), GFP_KERNEL);
	if (!rid) {
		pr_err("%s:no memory can be allocated\n", __func__);
		return -ENOMEM;
	}
	if (buf_size < sizeof(*rid)) {
		pr_err("%s:buf size is smaller than io_desc\n", __func__);
		ret = -EINVAL;
		goto err_write;
	}
	if (copy_from_user(rid, buf, sizeof(*rid)) > 0) {
		pr_err("%s:copy from user buf fail\n", __func__);
		ret = -EINVAL;
		goto err_write;
	}
	if (rid->id[0] != 'r' || rid->id[1] != 't') {
		ret = -EINVAL;
		goto err_write;
	}
	mutex_lock(&chip->var_lock);
	chip->pdata->vgpara1_hcap = rid->vgpara1_hcap;
	chip->pdata->vgpara2_hcap = rid->vgpara2_hcap;
	chip->pdata->vgpara3_hcap = rid->vgpara3_hcap;
	chip->pdata->vgpara4_hcap = rid->vgpara4_hcap;
	chip->pdata->vgpara5_hcap = rid->vgpara5_hcap;
	chip->pdata->vgpara1_lcap = rid->vgpara1_lcap;
	chip->pdata->vgpara2_lcap = rid->vgpara2_lcap;
	chip->pdata->vgpara3_lcap = rid->vgpara3_lcap;
	chip->pdata->vgpara4_lcap = rid->vgpara4_lcap;
	chip->pdata->vgpara5_lcap = rid->vgpara5_lcap;
	chip->pdata->r1_gain_tempcold_hcap = rid->r1_gain_tempcold_hcap;
	chip->pdata->r1_gain_temphot_hcap = rid->r1_gain_temphot_hcap;
	chip->pdata->r1_gain_tempcold2_hcap = rid->r1_gain_tempcold2_hcap;
	chip->pdata->r2_gain_tempcold_hcap = rid->r2_gain_tempcold_hcap;
	chip->pdata->r2_gain_temphot_hcap = rid->r2_gain_temphot_hcap;
	chip->pdata->r2_gain_tempcold2_hcap = rid->r2_gain_tempcold2_hcap;
	chip->pdata->r3_gain_tempcold_hcap = rid->r3_gain_tempcold_hcap;
	chip->pdata->r3_gain_temphot_hcap = rid->r3_gain_temphot_hcap;
	chip->pdata->r3_gain_tempcold2_hcap = rid->r3_gain_tempcold2_hcap;
	chip->pdata->r4_gain_tempcold_hcap = rid->r4_gain_tempcold_hcap;
	chip->pdata->r4_gain_temphot_hcap = rid->r4_gain_temphot_hcap;
	chip->pdata->r4_gain_tempcold2_hcap = rid->r4_gain_tempcold2_hcap;
	chip->pdata->r1_gain_tempcold_lcap = rid->r1_gain_tempcold_lcap;
	chip->pdata->r1_gain_temphot_lcap = rid->r1_gain_temphot_lcap;
	chip->pdata->r1_gain_tempcold2_lcap = rid->r1_gain_tempcold2_lcap;
	chip->pdata->r2_gain_tempcold_lcap = rid->r2_gain_tempcold_lcap;
	chip->pdata->r2_gain_temphot_lcap = rid->r2_gain_temphot_lcap;
	chip->pdata->r2_gain_tempcold2_lcap = rid->r2_gain_tempcold2_lcap;
	chip->pdata->r3_gain_tempcold_lcap = rid->r3_gain_tempcold_lcap;
	chip->pdata->r3_gain_temphot_lcap = rid->r3_gain_temphot_lcap;
	chip->pdata->r3_gain_tempcold2_lcap = rid->r3_gain_tempcold2_lcap;
	chip->pdata->r4_gain_tempcold_lcap = rid->r4_gain_tempcold_lcap;
	chip->pdata->r4_gain_temphot_lcap = rid->r4_gain_temphot_lcap;
	chip->pdata->r4_gain_tempcold2_lcap = rid->r4_gain_tempcold2_lcap;
	chip->pdata->temp_base = rid->temp_base;
	chip->pdata->high_temp_base = rid->high_temp_base;
	chip->pdata->low_temp_base = rid->low_temp_base;
	chip->pdata->low_temp2_base = rid->low_temp2_base;
	chip->pdata->soc_comp = rid->soc_comp;
#ifdef CONFIG_LGE_PM_RT9428_CUT_OFF_UNDER_3P2_USED
	chip->pdata->low_cut_off_gain = rid->low_cut_off_gain;
#endif
	mutex_unlock(&chip->var_lock);
	schedule_delayed_work(&chip->dwork, 0);
	ret = buf_size;
err_write:
	devm_kfree(chip->dev, rid);
	return ret;
}

static int rt9428_io_open(struct inode *inode, struct file *filp)
{
	struct rt9428_chip *chip = container_of(filp->private_data,
						struct rt9428_chip,
						rt9428_io_misc);
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static int rt9428_io_release(struct inode *inode, struct file *filp)
{
	struct rt9428_chip *chip = container_of(filp->private_data,
						struct rt9428_chip,
						rt9428_io_misc);
	dev_info(chip->dev, "%s\n", __func__);
	return 0;
}

static const struct file_operations rt9428_misc_fops = {
	.owner = THIS_MODULE,
	.read = rt9428_io_read,
	.write = rt9428_io_write,
	.open = rt9428_io_open,
	.release = rt9428_io_release,
};

static int rt_parse_dt(struct device *dev, struct rt9428_platform_data *pdata)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;
	struct device_node *np_profile = np;
	u32 prop_array[6];
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	int64_t batt_id;
	int profile_id = 0;
#endif
#if 0
	if (of_property_read_u32_array(np, "rt,vgcomp0", prop_array, 1) < 0) {
		dev_warn(dev, "no vgcom0 value, using default value\n");
		pdata->vgcomp0 = 0xAB;
	} else
		pdata->vgcomp0 = prop_array[0];
#endif /* #if 0 */

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	batt_id = read_lge_battery_id();

	switch ( batt_id ){
#if defined(CONFIG_LGE_PM_BATTERY_RT9428_BL52UH_2100mAh)
		/* C70 */
		case BATT_ID_ISL6296_C:
		case BATT_ID_DS2704_L:
			profile_id = RT9428_BATTERY_LGC;
			break;
		case BATT_ID_RA4301_VC0:
			profile_id = RT9428_BATTERY_BYD;
			break;
		case BATT_ID_ISL6296_L:
		case BATT_ID_DS2704_C:
			profile_id = RT9428_BATTERY_BYD_TOCAD;
			break;
#elif defined(CONFIG_LGE_PM_BATTERY_RT9428_BL51YF_3000mAh)
		/* stylus */
		case BATT_ID_SW3800_VC0:
		case BATT_ID_RA4301_VC1:
			profile_id = RT9428_BATTERY_LGC;
			break;

		case BATT_ID_RA4301_VC0:
		case BATT_ID_SW3800_VC1:
			profile_id = RT9428_BATTERY_TECHNOPHILE;
			break;
#elif defined(CONFIG_LGE_PM_BATTERY_RT9428_BLT12_4000mAh)
		/* e7II */
		/* need to implement after duel battery */
#endif
		default:
			profile_id = RT9428_BATTERY_DEFAULT;
			break;
	}

	switch (profile_id) {
		case RT9428_BATTERY_LGC:
			np_profile = of_find_node_by_name(np,"rt,lgc-battery-data");
			pr_err("%s:set rt,lgc-battery-data\n", __func__);
			break;
		case RT9428_BATTERY_TOCAD:
			np_profile = of_find_node_by_name(np,"rt,tocad-battery-data");
			pr_err("%s:set rt,tocad-battery-data\n", __func__);
			break;
		case RT9428_BATTERY_BYD:
			np_profile = of_find_node_by_name(np,"rt,byd-battery-data");
			pr_err("%s:set rt,byd-battery-data\n", __func__);
			break;
		case RT9428_BATTERY_BYD_TOCAD:
			/* c70 has BYD(old), TOCAD battery using same batt_id */
			np_profile = of_find_node_by_name(np,"rt,tocad-byd-battery-data");
			pr_err("%s:set rt,tocad-byd-battery-data\n", __func__);
			break;
		case RT9428_BATTERY_TECHNOPHILE:
			np_profile = of_find_node_by_name(np,"rt,technophile-battery-data");
			pr_err("%s:set rt,technophile-battery-data\n", __func__);
			break;
		case RT9428_BATTERY_DEFAULT:
			np_profile = of_find_node_by_name(np,"rt,default-battery-data");
			pr_err("%s:set rt,default-battery-data\n", __func__);
			break;
	}
#else
	pr_err("%s:no batt_id check, set batt profile from model dtsi.\n", __func__);
#endif

	if (of_property_read_u32_array(np_profile,
				"rt,vgpara_hcap", prop_array, 5) < 0) {
		dev_warn(dev, "no vgpara_hcap value, using default value\n");
		pdata->vgpara1_hcap = 0x8132;
		pdata->vgpara2_hcap = 0x8237;
		pdata->vgpara3_hcap = 0x8323;
		pdata->vgpara4_hcap = 0x84E1;
		pdata->vgpara5_hcap = 0x8501;
	} else {
		pr_err("%s:check vgpara1_hcap:%x\n", __func__, prop_array[0]);
		pdata->vgpara1_hcap = prop_array[0];
		pdata->vgpara2_hcap = prop_array[1];
		pdata->vgpara3_hcap = prop_array[2];
		pdata->vgpara4_hcap = prop_array[3];
		pdata->vgpara5_hcap = prop_array[4];
	}
	if (of_property_read_u32_array(np_profile,
				"rt,vgpara_lcap", prop_array, 5) < 0) {
		dev_warn(dev, "no vgpara_lcap value, using default value\n");
		pdata->vgpara1_lcap = 0x8132;
		pdata->vgpara2_lcap = 0x8237;
		pdata->vgpara3_lcap = 0x8323;
		pdata->vgpara4_lcap = 0x84E1;
		pdata->vgpara5_lcap = 0x8501;
	} else {
		pdata->vgpara1_lcap = prop_array[0];
		pdata->vgpara2_lcap = prop_array[1];
		pdata->vgpara3_lcap = prop_array[2];
		pdata->vgpara4_lcap = prop_array[3];
		pdata->vgpara5_lcap = prop_array[4];
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r1_gain_hcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r1_gain_hcap value, using default value\n");
		pdata->r1_gain_tempcold_hcap = -125;
		pdata->r1_gain_temphot_hcap = 350;
		pdata->r1_gain_tempcold2_hcap = -160;
	} else {
		pdata->r1_gain_tempcold_hcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r1_gain_temphot_hcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r1_gain_tempcold2_hcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r2_gain_hcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r2_gain_hcap value, using default value\n");
		pdata->r2_gain_tempcold_hcap = -50;
		pdata->r2_gain_temphot_hcap = -75;
		pdata->r2_gain_tempcold2_hcap = 280;
	} else {
		pdata->r2_gain_tempcold_hcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r2_gain_temphot_hcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r2_gain_tempcold2_hcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r3_gain_hcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r3_gain_hcap value, using default value\n");
		pdata->r3_gain_tempcold_hcap = 0;
		pdata->r3_gain_temphot_hcap = 300;
		pdata->r3_gain_tempcold2_hcap = 0;
	} else {
		pdata->r3_gain_tempcold_hcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r3_gain_temphot_hcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r3_gain_tempcold2_hcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r4_gain_hcap", prop_array, 6) <  0) {
		dev_warn(dev, "no r4_gain_hcap value, using default value\n");
		pdata->r4_gain_tempcold_hcap = -800;
		pdata->r4_gain_temphot_hcap = -200;
		pdata->r4_gain_tempcold2_hcap = -640;
	} else {
		pdata->r4_gain_tempcold_hcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r4_gain_temphot_hcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r4_gain_tempcold2_hcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r1_gain_lcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r1_gain_lcap value, using default value\n");
		pdata->r1_gain_tempcold_lcap = -125;
		pdata->r1_gain_temphot_lcap = 350;
		pdata->r1_gain_tempcold2_lcap = -160;
	} else {
		pdata->r1_gain_tempcold_lcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r1_gain_temphot_lcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r1_gain_tempcold2_lcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r2_gain_lcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r2_gain_lcap value, using default value\n");
		pdata->r2_gain_tempcold_lcap = -50;
		pdata->r2_gain_temphot_lcap = -75;
		pdata->r2_gain_tempcold2_lcap = 280;
	} else {
		pdata->r2_gain_tempcold_lcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r2_gain_temphot_lcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r2_gain_tempcold2_lcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r3_gain_lcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r3_gain_lcap value, using default value\n");
		pdata->r3_gain_tempcold_lcap = 0;
		pdata->r3_gain_temphot_lcap = 300;
		pdata->r3_gain_tempcold2_lcap = 0;
	} else {
		pdata->r3_gain_tempcold_lcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r3_gain_temphot_lcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r3_gain_tempcold2_lcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32_array(np_profile,
				"rt,r4_gain_lcap", prop_array, 6) < 0) {
		dev_warn(dev, "no r4_gain_lcap value, using default value\n");
		pdata->r4_gain_tempcold_lcap = -800;
		pdata->r4_gain_temphot_lcap = -200;
		pdata->r4_gain_tempcold2_lcap = -640;
	} else {
		pdata->r4_gain_tempcold_lcap =
		    (prop_array[0] ? -prop_array[1] : prop_array[1]);
		pdata->r4_gain_temphot_lcap =
		    (prop_array[2] ? -prop_array[3] : prop_array[3]);
		pdata->r4_gain_tempcold2_lcap =
		    (prop_array[4] ? -prop_array[5] : prop_array[5]);
	}
	if (of_property_read_u32(np, "rt,alert_threshold", &prop_array[0]) < 0) {
		dev_warn(dev,
			 "no alert_threshold value, using default value\n");
		pdata->alert_threshold = 1;
	} else {
		if (prop_array[0] >= RT9428_SOCL_MIN
		    && prop_array[0] <= RT9428_SOCL_MAX) {
			pdata->alert_threshold = prop_array[0];
			rt9428_init_regval[0] &= (~RT9428_SOCL_MASK);
			rt9428_init_regval[0] |=
			    ((~prop_array[0] + 1) & RT9428_SOCL_MASK);
		} else
			pr_err("%s:alert threshold value is 1, due to out of range (1~32)\n"
					, __func__);
	}
	if (of_property_read_u32(np, "rt,soc_comp", &prop_array[0]) < 0) {
		dev_warn(dev, "no soc_comp value, using default value\n");
		pdata->soc_comp = 0;
	} else
		pdata->soc_comp = prop_array[0];
	if (of_property_read_u32_array(np, "rt,temp_base", prop_array, 4) < 0) {
		dev_warn(dev, "no temp_base value, using default value\n");
		pdata->low_temp2_base = 0;
		pdata->low_temp_base = 10;
		pdata->temp_base = 25;
		pdata->high_temp_base = 40;

	} else {
		pdata->low_temp2_base = prop_array[0];
		pdata->low_temp_base = prop_array[1];
		pdata->temp_base = prop_array[2];
		pdata->high_temp_base = prop_array[3];
	}
	pdata->alert_gpio = of_get_named_gpio(np, "rt,alert_gpio", 0);
	if (of_property_read_u32(np, "rt,full_design", &prop_array[0]) < 0) {
		dev_warn(dev, "no full_design value, using default value\n");
		pdata->full_design = 2540;
	} else
		pdata->full_design = prop_array[0];
#ifdef CONFIG_LGE_PM_RT9428_CUT_OFF_UNDER_3P2_USED
	if (of_property_read_u32(np, "rt,low_cut_off_gain", &prop_array[0]) < 0) {
		dev_warn(dev, "no low_cut_off_gain value, using default value\n");
		pdata->low_cut_off_gain = 10;
	} else {
			pdata->low_cut_off_gain = prop_array[0];
			if (pdata->low_cut_off_gain > 10)
				pdata->low_cut_off_gain = 10;
	}
	pr_err("%s: rt,low_cut_off_gain:%d\n", __func__, pdata->low_cut_off_gain);
#endif
#endif /* #ifdef CONFIG_OF */
	return 0;
}

static int rt9428_intr_init(struct rt9428_chip *chip)
{
	int rc = 0;

	if (gpio_is_valid(chip->pdata->alert_gpio)) {
		rc = gpio_request_one(chip->pdata->alert_gpio, GPIOF_IN,
				      "rt9428_fg_intr");
		if (rc < 0) {
			pr_err("%s:gpio request error\n", __func__);
			goto err_intr;
		}
		chip->alert_irq = gpio_to_irq(chip->pdata->alert_gpio);
		if (chip->alert_irq < 0) {
			pr_err("%s:irq value is not valid\n", __func__);
			gpio_free(chip->pdata->alert_gpio);
			rc = -EINVAL;
			goto err_intr;
		}
		rc = devm_request_threaded_irq(chip->dev, chip->alert_irq, NULL,
					       rt9428_irq_handler,
					       IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
					       IRQF_DISABLED, "rt9428_fg_irq",
					       chip);
		if (rc < 0) {
			pr_err("%s:irq register failed\n", __func__);
			gpio_free(chip->pdata->alert_gpio);
			chip->alert_irq = -1;
			rc = -EINVAL;
			goto err_intr;
		}
		enable_irq_wake(chip->alert_irq);
	} else
		rc = -EINVAL;
err_intr:
	return rc;
}

static int rt9428_intr_deinit(struct rt9428_chip *chip)
{
	if (chip->alert_irq >= 0) {
		devm_free_irq(chip->dev, chip->alert_irq, chip);
		gpio_free(chip->pdata->alert_gpio);
	}
	return 0;
}

static int rt9428_chip_init(struct rt9428_chip *chip)
{
	int regval = 0;
	/*default: sleep 0 alrt 0 scen 1 */
	regval = rt9428_reg_read_word(chip->i2c, RT9428_REG_CFG0);
	regval &= 0xff00;
	regval |= rt9428_init_regval[0];
	rt9428_reg_write_word(chip->i2c, RT9428_REG_CFG0, regval);
	chip->online = 1;
	return 0;
}

#ifdef CONFIG_LGE_PM_RT9428_SHUTDOWN_SOC_USED
static void rt9428_shutdown(struct i2c_client *client)
{
	int ret = 0;
	u8 soc_raw = 0;
	struct rt9428_chip *chip = i2c_get_clientdata(client);
	struct spmi_controller *ctrl = spmi_busnum_to_ctrl(0);

	soc_raw = rt9428_reg_read(chip->i2c, RT9428_REG_SOCH);

	ret = spmi_ext_register_writel(ctrl, 0, BMS1_BMS_DATA_REG_0, &soc_raw, 1);

	if (ret < 0) {
		pr_err("%s:SPMI write failed, err = %d\n", __func__, ret);
	}
}
#endif

static int rt9428_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct rt9428_chip *chip;
	struct rt9428_platform_data *pdata = client->dev.platform_data;
	int regval, ret = 0;
	bool use_dt = client->dev.of_node;
	bool batt_present;

	pr_err("%s : start\n", __func__);

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		pr_err("%s : batt_psy is not yet ready\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_init;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s : i2c_check_functionality fail\n",
			__func__);
		ret = -EIO;
		goto err_init;
	}

	if (use_dt) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto err_init;
		}
		ret = rt_parse_dt(&client->dev, pdata);
		if (ret != 0){
			printk(KERN_ERR "%s : parse dt fail : return %d\n",	__func__, ret);
			goto err_init1;
		}
		chip->pdata = pdata;
	} else {
		if (!pdata) {
			ret = -EINVAL;
			goto err_init;
		}
		chip->pdata = pdata;
	}

	chip->i2c = client;
	chip->dev = &client->dev;

	chip->alert_irq = -1;	/* set default irq number = -1; */
	chip->qs_flag = 0;
	mutex_init(&chip->var_lock);
	mutex_init(&chip->io_lock);

	i2c_set_clientdata(client, chip);

	regval = rt9428_reg_read_word(client, RT9428_REG_DEVID0);
	if (regval < 0 || regval != 0x0012) {
		pr_err("%s:read device id fail or id is not correct\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_init1;
	}

	INIT_DELAYED_WORK(&chip->dwork, rt9428_dwork_func);
	wake_lock_init(&chip->update_lock, WAKE_LOCK_SUSPEND, "rt9428_update_lock");
	wake_lock_init(&chip->irq_lock, WAKE_LOCK_SUSPEND, "rt9428_irq_lock");
#ifdef CONFIG_LGE_PM_RT9428_POLLING
	INIT_DELAYED_WORK(&chip->polling_work, rt9428_polling_work);
#endif
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	INIT_DELAYED_WORK(&chip->update_polling_work, rt9428_update_polling_work);
	mutex_init(&chip->soc_update_lock);
#endif
	/*register power_supply for rt9428 fg */
	chip->fg_psy.name = "fuelgauge";
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
	chip->fg_psy.type = POWER_SUPPLY_TYPE_UNKNOWN;
#else
	chip->fg_psy.type = POWER_SUPPLY_TYPE_FUELGAUGE;
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)) */
	chip->fg_psy.supplied_to = rt_fg_supply_list;
	chip->fg_psy.properties = rt_fg_props;
	chip->fg_psy.num_properties = ARRAY_SIZE(rt_fg_props);
	chip->fg_psy.get_property = rt_fg_get_property;
	chip->fg_psy.set_property = rt_fg_set_property;
	ret = power_supply_register(&client->dev, &chip->fg_psy);
	if (ret < 0) {
		pr_err("%s:power supply register fail\n", __func__);
		goto err_init1;
	}

	rt9428_chip_init(chip);
	rt9428_intr_init(chip);

	ref = chip;

	chip->rt9428_io_misc.minor = MISC_DYNAMIC_MINOR;
	chip->rt9428_io_misc.name = "rt9428_io";
	chip->rt9428_io_misc.parent = &client->dev;
	chip->rt9428_io_misc.fops = &rt9428_misc_fops;
	ret = misc_register(&chip->rt9428_io_misc);
	if (ret < 0) {
		pr_err("%s:misc register fail\n", __func__);
		goto err_psy;
	}

	batt_present = rt9428_battery_present_checker();
	if (!batt_present)
		pr_err("%s: no battery, skip dwork, polling_work\n", __func__);
	else {
		/*queue update work immediately */
		schedule_delayed_work(&chip->dwork, msecs_to_jiffies(100));
#ifdef CONFIG_LGE_PM_RT9428_POLLING
		schedule_delayed_work(&chip->polling_work, msecs_to_jiffies(1000));
	}
#endif
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	schedule_delayed_work(&chip->update_polling_work, msecs_to_jiffies(1000));
#endif
	rt_fg_create_attrs(chip->fg_psy.dev);
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	rt_fg_create_attrs_factory(chip->dev);
#endif
	dev_info(&client->dev, "driver successfully loaded\n");

	pr_err("%s : Done\n", __func__);

	return 0;
err_psy:
	wake_lock_destroy(&chip->update_lock);
	wake_lock_destroy(&chip->irq_lock);
	power_supply_unregister(&chip->fg_psy);
err_init1:
	mutex_destroy(&chip->io_lock);
	if (use_dt)	devm_kfree(&client->dev, pdata);
err_init:
	devm_kfree(&client->dev, chip);
	return ret;
}

static int rt9428_i2c_remove(struct i2c_client *client)
{
	struct rt9428_chip *chip = i2c_get_clientdata(client);
	misc_deregister(&chip->rt9428_io_misc);
	rt9428_intr_deinit(chip);
	wake_lock_destroy(&chip->update_lock);
	wake_lock_destroy(&chip->irq_lock);
	power_supply_unregister(&chip->fg_psy);
#ifdef CONFIG_LGE_PM_RT9428_POLLING
	cancel_delayed_work(&chip->polling_work);
#endif
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	cancel_delayed_work(&chip->update_polling_work);
#endif

	return 0;
}

static int rt9428_i2c_suspend(struct device *dev)
{
	struct rt9428_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->dwork);
	chip->suspend = 1;
#ifdef CONFIG_LGE_PM_RT9428_POLLING
	cancel_delayed_work(&chip->polling_work);
#endif
#ifdef CONFIG_LGE_PM_RT9428_IRQ_LOW_WA
	cancel_delayed_work(&chip->update_polling_work);
#endif
	pr_err("%s: Done\n", __func__);
	return 0;
}

static int rt9428_i2c_resume(struct device *dev)
{
	struct rt9428_chip *chip = dev_get_drvdata(dev);

	chip->suspend = 0;
#ifdef CONFIG_LGE_PM_RT9428_POLLING
	schedule_delayed_work(&chip->polling_work, msecs_to_jiffies(HZ));
#endif
	pr_err("%s: Done\n", __func__);
	return 0;
}

static const struct i2c_device_id rt_i2c_id[] = {
	{RT9428_DEVICE_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, rt_i2c_id);

static const struct of_device_id rt_match_table[] = {
	{.compatible = "rt,rt9428",},
	{},
};

static const struct dev_pm_ops rt9428_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		rt9428_i2c_suspend,
		rt9428_i2c_resume
	)
};

static struct i2c_driver rt9428_i2c_driver = {
	.driver = {
		   .name = RT9428_DEVICE_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = rt_match_table,
		   .pm = &rt9428_pm_ops,
		   },
	.probe = rt9428_i2c_probe,
	.remove = rt9428_i2c_remove,
	.id_table = rt_i2c_id,
#ifdef CONFIG_LGE_PM_RT9428_SHUTDOWN_SOC_USED
	.shutdown = rt9428_shutdown,
#endif
};

static int __init rt9428_init(void)
{
	return i2c_add_driver(&rt9428_i2c_driver);
}
module_init(rt9428_init);

static void __exit rt9428_exit(void)
{
	i2c_del_driver(&rt9428_i2c_driver);
}
module_exit(rt9428_exit);

MODULE_AUTHOR("cy_huang <cy_huang@richtek.com>");
MODULE_DESCRIPTION("RT9428 Fuelgauge Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RT9428_DRV_VER);
