/***************************************************************************
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *    File  	: lgtp_device_mXT2954.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[mXT2954]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_3/lgtp_common.h>
#include <linux/input/unified_driver_3/lgtp_model_config.h>
#include <linux/input/unified_driver_3/lgtp_common_driver.h>
#include <linux/input/unified_driver_3/lgtp_platform_api.h>
#include <linux/input/unified_driver_3/lgtp_device_mXT2954.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/


/****************************************************************************
 * Macros
 ****************************************************************************/


/****************************************************************************
* Type Definitions
****************************************************************************/


/****************************************************************************
* Variables
****************************************************************************/
#if defined(CONFIG_MACH_MSM8974_T1LTE_GLOBAL_COM)
static const char defaultFirmware[] = "atmel/t1lte_global_com/t1_0803v25.fw";
#elif defined(CONFIG_MACH_MSM8974_T1WIFI_GLOBAL_COM)
static const char defaultFirmware[] = "atmel/t1wifi_global_com/t1_0803v25.fw";
#elif defined(CONFIG_MACH_MSM8974_T1WIFIN_GLOBAL_COM)
static const char defaultFirmware[] = "atmel/t1wifin_global_com/t1_0803v25.fw";
#endif

//====================================================================
// NORMAL : general touch(finger,key) is working
// OFF : touch is not working even knock-on ( lowest power saving )
// KNOCK_ON_ONLY : knock-on is only enabled
// KNOCK_ON_CODE : knock-on and knock-code are enabled
// NORMAL_HOVER : hover detection is enabled and general touch is working
// HOVER : only hover detection is enabled
//====================================================================
static unsigned char patchevent_mask = 0;
static unsigned char power_block_mask = 0;
static struct mutex i2c_suspend_lock;

static bool selftest_enable;
static bool selftest_show;

struct mxt2954_ts_data *ts = NULL;
static struct mxt2954_ts_data *global_ts;

/* Thermal(LowTemp) solution */
static bool curr_low_temp = 0;
/* IME solution */
static u8 ime_status_value = 0;
int selfd_check_usb_type = 0;

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern struct mutex *pMutexTouch;
extern struct wake_lock *pWakeLockTouch;
extern void release_all_touch_event(TouchDriverData *pDriverData);

/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static int mxt_t6_command(struct mxt2954_ts_data *ts, u16 cmd_offset, u8 value, bool wait);
static int mxt_set_t7_power_cfg(struct mxt2954_ts_data *ts, u8 sleep);


/****************************************************************************
* Local Functions
****************************************************************************/

static void write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu \n",
		my_date.tm_mon + 1,my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	TOUCH_LOG("write open %s, fd : %d\n", (fd >= 0)? "success":"fail",fd);
	if (fd >= 0) {
		if (time > 0) {
			sys_write(fd, time_string, strlen(time_string));
			TOUCH_LOG("Time write success\n");
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);

}

/****************************************************************************
* Device Specific Functions
****************************************************************************/
static char mxt_power_block_get(void)
{
	return power_block_mask;
}

static void mxt_power_block(char value)
{
	power_block_mask |= value;
}

static void mxt_power_unblock(char value)
{
	power_block_mask &= ~(value);
}

static char mxt_patchevent_get(char value)
{
	return patchevent_mask & value;
}

static void mxt_patchevent_set(char value)
{
	patchevent_mask |= value;
}

static void mxt_patchevent_unset(char value)
{
	patchevent_mask &= ~(value);
}

static inline size_t mxt_obj_size(const struct mxt2954_object *obj)
{
	return obj->size_minus_one + 1;
}

static inline size_t mxt_obj_instances(const struct mxt2954_object *obj)
{
	return obj->instances_minus_one + 1;
}

static int mxt_set_t7_power_cfg(struct mxt2954_ts_data *ts, u8 sleep)
{
	int error = 0;
	struct t7_config *new_config = NULL;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &ts->t7_cfg;

	error = Mxt2954_I2C_Write(ts->client, ts->T7_address, (u8 *)new_config, sizeof(ts->t7_cfg));
	if (error)
		return TOUCH_FAIL;

	TOUCH_LOG("Set T7 ACTV:%d IDLE:%d\n", new_config->active, new_config->idle);

	return TOUCH_SUCCESS;
}

struct mxt2954_object *mxt_get_object(struct mxt2954_ts_data *ts, u8 type)
{
	struct mxt2954_object *object = NULL;
	int i = 0;

	for (i = 0; i < ts->info->object_num; i++) {
		object = ts->object_table + i;
		if (object->type == type)
			return object;
	}

	TOUCH_LOG("Invalid object type T%u\n", type);
	return NULL;
}

int mxt_read_object(struct mxt2954_ts_data *ts, u8 type, u8 offset, u8 *val)
{
	struct mxt2954_object *object = NULL;
	int error = 0;

	object = mxt_get_object(ts, type);
	if (!object)
		return TOUCH_FAIL;

	error = Mxt2954_I2C_Read(ts->client, object->start_address + offset,2, val, 1);
	if (error)
		TOUCH_LOG("Error to read T[%d] offset[%d] val[%d]\n", type, offset, *val);

	return error;
}

int mxt_write_object(struct mxt2954_ts_data *ts, u8 type, u8 offset, u8 val)
{
	struct mxt2954_object *object = NULL;
	int error = 0;
	u16 reg = 0;

	object = mxt_get_object(ts, type);
	if (!object)
		return TOUCH_FAIL;

	reg = object->start_address;
	error = Mxt2954_I2C_Write(ts->client, reg + offset, &val, 1);
	if (error) {
		TOUCH_ERR("Error to write T[%d] offset[%d] val[%d]\n", type, offset, val);
		return TOUCH_FAIL;
	}

	return error;
}

static int mxt_bootloader_write(struct mxt2954_ts_data *ts, const u8 * const val, unsigned int count)
{
	int ret = 0;
	struct i2c_msg msg = {0};

	msg.addr = ts->bootloader_addr;
	msg.flags = ts->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		TOUCH_ERR("i2c send failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_bootloader_read(struct mxt2954_ts_data *ts, u8 *val, unsigned int count)
{
	int ret = 0;
	struct i2c_msg msg = { 0 };

	msg.addr = ts->bootloader_addr;
	msg.flags = ts->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(ts->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		TOUCH_ERR("i2c recv failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_send_bootloader_cmd(struct mxt2954_ts_data *ts, bool unlock)
{
	int ret = 0;
	u8 buf[2] = {0};

	TOUCH_LOG("mxt_send_bootloader_cmd (unlock) : %d\n", unlock);

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(ts, buf, 2);
	if (ret)
		return ret;

	return TOUCH_SUCCESS;
}

static int mxt_command_backup(struct mxt2954_ts_data *ts, u8 value)
{
	mxt_write_object(ts, MXT_GEN_COMMAND_T6, MXT_COMMAND_BACKUPNV, value);
	msleep(MXT_BACKUP_TIME);

	return TOUCH_SUCCESS;
}

static int mxt_command_reset(struct mxt2954_ts_data *ts, u8 value)
{
	int error = 0;

	error = mxt_write_object(ts, MXT_GEN_COMMAND_T6, MXT_COMMAND_RESET, value);
	msleep(MXT_RESET_TIME);

	if (error) {
		TOUCH_ERR("Not respond after reset command[%d]\n", value);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static void mxt_reset_slots(struct mxt2954_ts_data *ts)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(ts->client);

	TOUCH_LOG("Release all event\n");

	release_all_touch_event(pDriverData);

	ts->ts_data.prev_total_num = 0;
	memset(ts->ts_data.prev_data, 0, sizeof(ts->ts_data.prev_data));
	memset(ts->ts_data.curr_data, 0, sizeof(ts->ts_data.curr_data));
}

static int mxt_soft_reset(struct mxt2954_ts_data *ts)
{
	int ret = 0;

	TOUCH_FUNC();

	ret = mxt_t6_command(ts, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	msleep(MXT_RESET_TIME);

	return TOUCH_SUCCESS;
}

static int mxt_prepare_debug_data(struct mxt2954_ts_data *ts)
{
	struct mxt2954_raw_data *rawdata = NULL;
	int error = 0;

	if (ts->rawdata) {
		if (ts->rawdata->reference) {
			kfree(ts->rawdata->reference);
			ts->rawdata->reference = NULL;
		}
		if (ts->rawdata->delta) {
			kfree(ts->rawdata->delta);
			ts->rawdata->delta = NULL;
		}
		kfree(ts->rawdata);
		ts->rawdata = NULL;
	}

	rawdata = kzalloc(sizeof(struct mxt2954_raw_data), GFP_KERNEL);
	if (rawdata == NULL) {
		TOUCH_ERR("Fail to allocate sysfs data\n");
		error = -ENOMEM;
		return TOUCH_FAIL;
	}

	rawdata->num_xnode = ts->channel_size.size_x;
	rawdata->num_ynode = ts->channel_size.size_y;
	rawdata->num_nodes = rawdata->num_xnode * rawdata->num_ynode;

	TOUCH_LOG("x=%d, y=%d, total=%d\n", rawdata->num_xnode, rawdata->num_ynode, rawdata->num_nodes);

	rawdata->reference = kzalloc(rawdata->num_nodes * sizeof(u16), GFP_KERNEL);
	if (!rawdata->reference) {
		TOUCH_ERR("Fail to alloc reference of rawdata\n");
		error = -ENOMEM;
		goto err_alloc_reference;
	}

	rawdata->delta = kzalloc(rawdata->num_nodes * sizeof(s16), GFP_KERNEL);
	if (!rawdata->delta) {
		TOUCH_ERR("Fail to alloc delta of fdata\n");
		error = -ENOMEM;
		goto err_alloc_delta;
	}

	ts->rawdata = rawdata;
	return TOUCH_SUCCESS;

err_alloc_delta:
err_alloc_reference:
	TOUCH_LOG("kfree in %s\n", __func__);
	if (rawdata->delta)
		kfree(rawdata->delta);
	if (rawdata->reference)
		kfree(rawdata->reference);
	if (rawdata)
		kfree(rawdata);
	return TOUCH_SUCCESS;

}

static int mxt_set_diagnostic_mode(struct mxt2954_ts_data *ts, u8 dbg_mode)
{
	u8 cur_mode = 0;
	int ret = 0;
	int retry_cnt = 0;

	/*MXT_GEN_COMMANDPROCESSOR_T6*/
	ret = mxt_write_object(ts, MXT_GEN_COMMAND_T6, MXT_COMMAND_DIAGNOSTIC, dbg_mode);
	if (ret) {
		TOUCH_ERR("Failed change diagnositc mode to %d\n", dbg_mode);
		goto out;
	}

	if (dbg_mode & MXT_DIAG_MODE_MASK) {
		do {
			ret = mxt_read_object(ts, MXT_DEBUG_DIAGNOSTIC_T37, MXT_DIAGNOSTIC_MODE, &cur_mode);
			if (ret || retry_cnt++ >= 4) {
				TOUCH_ERR("Failed getting diagnositc mode(%d)\n", retry_cnt);
				goto out;
			}
			msleep(20);
		} while (cur_mode != dbg_mode);
		TOUCH_LOG("current dianostic chip mode is %d\n", cur_mode);
	}

out:
	return ret;
}

static int mxt_treat_dbg_data(struct mxt2954_ts_data *ts, struct mxt2954_object *dbg_object, u8 dbg_mode, u8 read_point, u16 num)
{
	struct mxt2954_raw_data *rawdata = ts->rawdata;
	u8 data_buffer[DATA_PER_NODE] = { 0 };
	int ret = 0;

	if (dbg_mode == MXT_DIAG_DELTA_MODE) {
		/* read delta data */
		Mxt2954_I2C_Read(ts->client, dbg_object->start_address + read_point, 2, data_buffer, DATA_PER_NODE);
		rawdata->delta[num] = ((u16)data_buffer[1] << 8) + (u16)data_buffer[0];

		ret = rawdata->delta[num];

	} else if (dbg_mode == MXT_DIAG_REFERENCE_MODE) {
		/* read reference data */
		Mxt2954_I2C_Read(ts->client, dbg_object->start_address + read_point, 2, data_buffer, DATA_PER_NODE);
		rawdata->reference[num] = ((u16)data_buffer[1] << 8) + (u16)data_buffer[0] - REF_OFFSET_VALUE;

		/*TOUCH_LOG("reference[%d] = %d\n", num, rawdata->reference[num]);*/

		ret = rawdata->reference[num];
	}
	return ret;
}

static bool mxt_check_xy_range(struct mxt2954_ts_data *ts, u16 node)
{
	u8 x_line = node / ts->channel_size.size_y;
	u8 y_line = node % ts->channel_size.size_y;

	return (y_line < ts->rawdata->num_ynode) ?
		(x_line < ts->rawdata->num_xnode) : false;
}

static int mxt_read_all_diagnostic_data(struct mxt2954_ts_data *ts, u8 dbg_mode, char *buf, int* len)
{
	struct mxt2954_object *dbg_object = NULL;
	u8 read_page = 0, cur_page = 0, end_page = 0, read_point = 0;
	u16 node = 0, num = 0,  cnt = 0;
	int ret = 0;
	int i = 0;
	int value = 0;

	int write_page = 1 << 14;

	if (ts->show_delta == false) {
		TouchDisableIrq();
	}
	wake_lock_timeout(pWakeLockTouch, msecs_to_jiffies(2000));
#if 0
	if (mutex_is_locked(pMutexTouch)) {
		TOUCH_LOG("mutex_is_locked\n");
	} else {
		mutex_lock(pMutexTouch);
	}
#endif

	/* to make the Page Num to 0 */
	ret = mxt_set_diagnostic_mode(ts, MXT_DIAG_CTE_MODE);
	if (ret) {
		TOUCH_ERR("fail to make the Page Num to 0\n");
		return TOUCH_FAIL;
	}
	/* change the debug mode */
	ret = mxt_set_diagnostic_mode(ts, dbg_mode);
	if (ret) {
		TOUCH_ERR("fail to change the debug mode\n");
		return TOUCH_FAIL;
	}
	/* get object info for diagnostic */
	dbg_object = mxt_get_object(ts, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!dbg_object) {
		TOUCH_ERR("fail to get object info for diagnostic\n");
		ret = -EINVAL;
		return TOUCH_FAIL;
	}

	/* LGE */
	mxt_prepare_debug_data(ts);

	end_page = (ts->channel_size.size_x * ts->channel_size.size_y) / NODE_PER_PAGE + 1;
	*len += snprintf(buf + *len , write_page - *len, "\n       ");
	for(i=0; i<ts->channel_size.size_y; i++)
		*len += snprintf(buf + *len , write_page - *len, "[Y%02d] ", i);

	/* read the dbg data */
	for (read_page = 0 ; read_page < end_page; read_page++) {
		for (node = 0; node < NODE_PER_PAGE; node++) {
			if (cnt%ts->channel_size.size_y == 0) {
				*len += snprintf(buf + *len , write_page - *len, "\n[X%02d] ", cnt/ts->channel_size.size_y);
				if (cnt/ts->channel_size.size_y == ts->channel_size.size_x) {
					*len += snprintf(buf + *len , write_page - *len, "\n");
					break;
				}
			}
			read_point = (node * DATA_PER_NODE) + 2;

			if (!mxt_check_xy_range(ts, cnt++)) {
				break;
			}

			value = mxt_treat_dbg_data(ts, dbg_object, dbg_mode, read_point, num);
			*len += snprintf(buf + *len , write_page - *len, "%6d", value);

			if (dbg_mode == MXT_DIAG_REFERENCE_MODE && ts->full_cap != NULL )
				ts->full_cap[num / ts->channel_size.size_y][num % ts->channel_size.size_y] = value;

			num++;
		}
		ret = mxt_set_diagnostic_mode(ts, MXT_DIAG_PAGE_UP);
		if (ret)
			return TOUCH_FAIL;
		do {
			msleep(20);
			ret = Mxt2954_I2C_Read(ts->client, dbg_object->start_address + MXT_DIAGNOSTIC_PAGE, 2, &cur_page, 1);
			if (ret) {
				TOUCH_ERR("Read fail page\n");
				return TOUCH_FAIL;
			}
		} while (cur_page != read_page + 1);
	}

	if (ts->rawdata) {
		if (ts->rawdata->reference) {
			kfree(ts->rawdata->reference);
			ts->rawdata->reference = NULL;
		}
		if (ts->rawdata->delta) {
			kfree(ts->rawdata->delta);
			ts->rawdata->delta = NULL;
		}
		kfree(ts->rawdata);
		ts->rawdata = NULL;
	}
#if 0
	mutex_unlock(pMutexTouch);
#endif
	if (ts->show_delta == false)
		TouchEnableIrq();

	return ret;
}

static int run_reference_read(void *device_data, char *buf, int *len)
{
	struct mxt2954_ts_data *ts = (struct mxt2954_ts_data *)device_data;
	int ret = 0;

	ret = mxt_read_all_diagnostic_data(ts, MXT_DIAG_REFERENCE_MODE, buf, len);

	return ret;
}

static int run_delta_read(void *device_data, char *buf, int *len)
{
	struct mxt2954_ts_data *ts = (struct mxt2954_ts_data *)device_data;
	int ret = 0;

	ret = mxt_read_all_diagnostic_data(ts, MXT_DIAG_DELTA_MODE, buf, len);

	return ret;
}

static void mxt_read_fw_version(struct mxt2954_ts_data *ts)
{
	TOUCH_LOG("==================================\n");
	TOUCH_LOG("Firmware Version = %d.%02d \n", ts->fw_ver[0], ts->fw_ver[1]);
	TOUCH_LOG("FW Product       = %s \n", ts->product);
	TOUCH_LOG("Binary Version   = %u.%u.%02X \n", ts->info->version >> 4, ts->info->version & 0xF, ts->info->build);
	TOUCH_LOG("Config CRC       = 0x%X  \n", ts->config_crc);
	TOUCH_LOG("Family Id        = 0x%02X \n", ts->info->family_id);
	TOUCH_LOG("Variant          = 0x%02X \n", ts->info->variant_id);
	TOUCH_LOG("==================================\n");
}

static int mxt_t25_command(struct mxt2954_ts_data *ts, u8 value, bool wait)
{
	u16 reg = 0;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret = 0;

	if (!selftest_enable) {
		return TOUCH_SUCCESS;
	}

	reg = ts->T25_address + 1 ;

	ret = Mxt2954_I2C_Write(ts->client, reg, &value, 1);
	if (ret) {
		TOUCH_ERR("Write Self Test Command fail!\n");
		return ret;
	}

	if (!wait)
		return TOUCH_SUCCESS;

	do {
		msleep(20);
		ret = Mxt2954_I2C_Read(ts->client, reg, 2, &command_register, 1);

		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		TOUCH_ERR("Command failed!\n");
		return -EIO;
	}

	return TOUCH_SUCCESS;
}

static int mxt_check_mem_access_params(struct mxt2954_ts_data *ts, loff_t off, size_t *count)
{
	ts->mem_size = 32768;

	if (off >= ts->mem_size)
		return -EIO;

	if (off + *count > ts->mem_size)
		*count = ts->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return TOUCH_SUCCESS;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	#if 0 /* 2015.06.17 evade for kernel crash */
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt2954_ts_data *ts = dev_get_drvdata(dev);
	#endif
	int ret = 0;

	ret = mxt_check_mem_access_params(ts, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = Mxt2954_I2C_Read(ts->client, off, 2, buf, count);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	#if 0 /* 2015.06.17 evade for kernel crash */
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt2954_ts_data *ts = dev_get_drvdata(dev);
	#endif
	int ret = 0;

	ret = mxt_check_mem_access_params(ts, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = Mxt2954_I2C_Write(ts->client, off, buf, count);

	return ret == 0 ? count : 0;
}

static ssize_t mxt_mfts_enable_show(struct i2c_client *client, char *buf)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", pDriverData->mfts_enable);

	return len;
}

static ssize_t mxt_mfts_enable_store(struct i2c_client *client, const char *buf, size_t count)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_LOG("%s = %d \n", __func__, value);

	pDriverData->mfts_enable = value;

	if (pDriverData->mfts_enable)
		ts->use_mfts = true;

	/* Touch IC Reset for Initial configration. */
	mxt_soft_reset(ts);

	/* Calibrate for Active touch IC */
	mxt_t6_command(ts, MXT_COMMAND_CALIBRATE, 1, false);

	return count;
}

static ssize_t mxt_testmode_ver_show(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "V%d.%02d(%s)", ts->fw_ver[0], ts->fw_ver[1], ts->product);

	return ret;
}


static ssize_t mxt_atcmd_fw_ver_show(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += sprintf(buf+ret, "V%d.%02d(%s)", ts->fw_ver[0], ts->fw_ver[1], ts->product);

	return ret;
}

static ssize_t mxt_selftest(struct mxt2954_ts_data *ts, char *buf, int len)
{
	int ret = len;
	int test_all_cmd = 0xFE;

	selftest_enable = true;
	selftest_show = true;

	mxt_t25_command(ts, test_all_cmd, false);
	msleep(500);

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "=======[Detailed Results]========\n");

	if (ts->self_test_status[0] == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,  "Need more time. Try Again !!!\n");
		return ret;
	}

	if (ts->self_test_status[0] == 0xFD) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Invalid Test Code. Try Again.");
	} else if (ts->self_test_status[0] == 0xFC) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "The test could not be completed due to an unrelated fault. Try again.");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "All Test Result: %s", (ts->self_test_status[0] == 0xFE) ? "Pass\n" : "Fail\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "AVdd power Test Result: %s", (ts->self_test_status[0] != 0x01) ? "Pass\n" : "Fail\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "Pin Fault Test Result: %s", (ts->self_test_status[0] != 0x12) ? "Pass\n" : "Fail\n");

		if (ts->self_test_status[0] == 0x12)
			ret += snprintf(buf+ret, PAGE_SIZE - ret, "# Fail # seq_num(%u) x_pin(%u) y_pin(%u)\n",
										ts->self_test_status[1],
										ts->self_test_status[2],
										ts->self_test_status[3]);
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "Signal Limit Test: %s", (ts->self_test_status[0] != 0x17) ? "Pass\n" : "Fail\n");
		if (ts->self_test_status[0] == 0x17)
			ret += snprintf(buf+ret, PAGE_SIZE - ret, "# Fail # type_num(%u) type_instance(%u)\n",
										ts->self_test_status[1],
										ts->self_test_status[2]);
	}

	selftest_show = false;

	return ret;
}

static ssize_t mxt_run_delta_show(struct i2c_client *client, char *buf)
{
	int len = 0;
	char *ref_buf = NULL;

	int write_page = 1 << 14;

	/* Run delta show - control interrupt state */
	ts->show_delta = true;

	/* allocate buffer for additional debugging information */
	ref_buf = kzalloc(write_page, GFP_KERNEL);
	if (ref_buf == NULL) {
		TOUCH_ERR("failed to allocate memory for delta\n");
		return -EINVAL;
	}

	write_file(DELTA_FILE_PATH, buf, 1);
	msleep(30);
	run_delta_read(ts, ref_buf, &len);
	write_file(DELTA_FILE_PATH, ref_buf, 0);
	msleep(30);

	kfree(ref_buf);

	if (ts->rawdata) {
		if (ts->rawdata->reference) {
			kfree(ts->rawdata->reference);
			ts->rawdata->reference = NULL;
		}
		if (ts->rawdata->delta) {
			kfree(ts->rawdata->delta);
			ts->rawdata->delta = NULL;
		}
		kfree(ts->rawdata);
		ts->rawdata = NULL;
	}

	ts->show_delta = false;

	return len;
}

static int check_lpwg_fail_reason(struct i2c_client *client, bool mode)
{
	u8 value[2] = {0,};
	struct mxt2954_object *object = NULL;

	TOUCH_LOG("LPWG_FAIL_REASON SET : [%d]\n", mode);

	object = mxt_get_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (object) {
		Mxt2954_I2C_Read(ts->client, object->start_address + 28, 2, value, 2);
		TOUCH_LOG("Debug reason ctrl 28th:[0x%x], 29th:[0x%x], use_debug_reason:[%d]\n",
				value[0],
				value[1],
				ts->use_debug_reason);
	} else {
		TOUCH_ERR("Failed get T93 Object\n");
	}

	return TOUCH_SUCCESS;
}

static ssize_t show_lpwg_debug_reason(struct i2c_client *client, char *buf)
{
	int count = 0;
	char c = 0;

	c = ts->use_debug_reason ? '1' : '0';
	count = sprintf(buf, "lpwg_debug_reason : [%c]\n", c);

	return count;
}

static ssize_t store_lpwg_debug_reason(struct i2c_client *client, const char *buf, size_t count)
{
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		ts->use_debug_reason = (i == 1);

		TOUCH_LOG("%s\n", i ? "[lpwg_debug_reason] enabled" : "[lpwg_debug_reason] disabled");

		check_lpwg_fail_reason(client, ts->use_debug_reason);

		return count;
	} else {
		TOUCH_LOG("lpwg_debug_reason write error\n");
		return -EINVAL;
	}
}

static ssize_t show_mxt_driver_debug_enable(struct i2c_client *client, char *buf)
{
	int count = 0;
	char c = 0;

	c = ts->driver_debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t store_mxt_driver_debug_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		ts->driver_debug_enabled = (i == 1);

		TOUCH_LOG("%s\n", i ? "driver_debug enabled" : "driver_debug disabled");
		return count;
	} else {
		TOUCH_LOG("driver_debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t show_mxt_debug_enable(struct i2c_client *client, char *buf)
{
	int count = 0;
	char c = 0;

	c = ts->debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t store_mxt_debug_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		ts->debug_enabled = (i == 1);

		TOUCH_LOG("%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		TOUCH_LOG("debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t store_mxt_t57_debug_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		ts->t57_debug_enabled = (i == 1);

		TOUCH_LOG("%s\n", i ? "t57 debug enabled" : "t57 debug disabled");
		return count;
	} else {
		TOUCH_LOG("t57_debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t show_mxt_patch_debug_enable(struct i2c_client *client, char *buf)
{
	int count = 0;
	char c = 0;

	if (ts->patch.patch == NULL) {
		TOUCH_LOG("patch not support \n");
		return count;
	}

	c = ts->patch.debug ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t store_mxt_patch_debug_enable(struct i2c_client *client, const char *buf, size_t count)
{
	int i = 0;

	if (ts->patch.patch == NULL) {
		TOUCH_LOG("patch not support \n");
		return count;
	}

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		ts->patch.debug = (i == 1);

		TOUCH_LOG("%s\n", i ? "patch debug enabled" : "patch debug disabled");
		return count;
	} else {
		TOUCH_LOG("patch_debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_force_rebase_show(struct i2c_client *client, char *buf)
{
	int len = 0;

	TOUCH_LOG("MXT_COMMAND_CALIBRATE \n");
	mxt_t6_command(ts, MXT_COMMAND_CALIBRATE, 1, false);

	return len;
}

static ssize_t show_mxt_info(struct i2c_client *client, char *buf)
{
	int ret = 0;

	mxt_read_fw_version(ts);

	ret += sprintf(buf+ret, "Firmware Version = %d.%02d \n", ts->fw_ver[0], ts->fw_ver[1]);
	ret += sprintf(buf+ret, "FW Product       = %s \n", ts->product);
	ret += sprintf(buf+ret, "Binary Version   = %u.%u.%02X \n", ts->info->version >> 4, ts->info->version & 0xF, ts->info->build);
	ret += sprintf(buf+ret, "Config CRC       = 0x%X \n", ts->config_crc);
	ret += sprintf(buf+ret, "Family Id        = 0x%02X \n", ts->info->family_id);
	ret += sprintf(buf+ret, "Variant          = 0x%02X \n", ts->info->variant_id);
	if (ts->info->family_id==0xA4 && ts->info->variant_id==0x0D) {
		ret += sprintf(buf+ret, "Product Id       = [MXT2954] \n");
	} else {
		ret += sprintf(buf+ret, "Product Id       = [Unknown IC] \n");
	}

	return ret;
}

static ssize_t show_device_name(struct i2c_client *client, char *buf)
{
	int ret = 0;

	TOUCH_FUNC();

	ret += sprintf(buf+ret, "%s\n", "mXT2954/Atmel");

	return ret;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
       int ret = 0;
       int pen_support = 1;   /* 1: Support , 0: Not support */

       ret = sprintf(buf, "%d\n", pen_support);

	return ret;
}

static ssize_t store_ime_status(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	if(ime_status_value == value)
		return count;

	TOUCH_LOG("%s : %d \n", __func__, value);

	if (value == 1){
		mxt_patch_event(global_ts, IME_MODE_ENABLE);
	} else if (value == 0) {
		mxt_patch_event(global_ts, IME_MODE_DISABLE);
	}

	ime_status_value = value;

	return count;
}

static ssize_t mxt_self_cap_show(struct i2c_client *client, char *buf)
{
       int ret = 0;
       int self_cap_support = 1;   /* 1: Support , 0: Not support */

       ret = sprintf(buf, "%d\n", self_cap_support);

	TOUCH_LOG("%s : %d \n", __func__, self_cap_support);

	return ret;
}

static ssize_t mxt_self_cap_store(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_LOG("%s : %d \n", __func__, value);

	return count;
}


static ssize_t mxt_noise_suppression_show(struct i2c_client *client, char *buf)
{
       int ret = 0;
       int self_cap_support = 1;   /* 1: Support , 0: Not support */

       ret = sprintf(buf, "%d\n", self_cap_support);

	TOUCH_LOG("%s : %d \n", __func__, self_cap_support);
	mxt_patch_event(global_ts,SELF_CAP_OFF_NOISE_SUPPRESSION );

	return ret;
}

static ssize_t mxt_noise_suppression_store(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_LOG("%s : %d \n", __func__, value);

	return count;
}

static ssize_t mxt_noise_recover_show(struct i2c_client *client, char *buf)
{
       int ret = 0;
       int self_cap_support = 1;   /* 1: Support , 0: Not support */

       ret = sprintf(buf, "%d\n", self_cap_support);

	TOUCH_LOG("%s : %d \n", __func__, self_cap_support);
	mxt_patch_event(global_ts,SELF_CAP_ON_NOISE_RECOVER );

	return ret;
}

static ssize_t mxt_noise_recover_store(struct i2c_client *client, const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	TOUCH_LOG("%s : %d \n", __func__, value);

	return count;
}

static LGE_TOUCH_ATTR(device_name, S_IRUGO | S_IWUSR, show_device_name, NULL);
static LGE_TOUCH_ATTR(mxt_info, S_IRUGO, show_mxt_info, NULL);
static LGE_TOUCH_ATTR(patch_debug_enable, S_IWUSR | S_IRUSR, show_mxt_patch_debug_enable, store_mxt_patch_debug_enable);
static LGE_TOUCH_ATTR(lpwg_fail_reason, S_IRUGO | S_IWUSR, show_lpwg_debug_reason, store_lpwg_debug_reason);
static LGE_TOUCH_ATTR(driver_debug_enable, S_IWUSR | S_IRUSR, show_mxt_driver_debug_enable, store_mxt_driver_debug_enable);
static LGE_TOUCH_ATTR(debug_enable, S_IWUSR | S_IRUSR, show_mxt_debug_enable, store_mxt_debug_enable);
static LGE_TOUCH_ATTR(t57_debug_enable, S_IWUSR | S_IRUSR, NULL, store_mxt_t57_debug_enable);
static LGE_TOUCH_ATTR(delta, S_IRUGO, mxt_run_delta_show, NULL);
static LGE_TOUCH_ATTR(rebase, S_IWUSR | S_IRUGO, mxt_force_rebase_show, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, mxt_testmode_ver_show, NULL);
static LGE_TOUCH_ATTR(fw_ver, S_IRUGO | S_IWUSR, mxt_atcmd_fw_ver_show, NULL);
static LGE_TOUCH_ATTR(version, S_IRUGO, show_mxt_info, NULL);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO | S_IWUSR, show_pen_support, NULL);
static LGE_TOUCH_ATTR(mfts, S_IWUSR | S_IRUSR, mxt_mfts_enable_show, mxt_mfts_enable_store);
static LGE_TOUCH_ATTR(ime_status, S_IRUGO | S_IWUSR, NULL, store_ime_status);
static LGE_TOUCH_ATTR(self_cap, S_IWUSR | S_IRUGO, mxt_self_cap_show, mxt_self_cap_store);
static LGE_TOUCH_ATTR(noise_suppression, S_IWUSR | S_IRUGO, mxt_noise_suppression_show, mxt_noise_suppression_store);
static LGE_TOUCH_ATTR(noise_recover, S_IWUSR | S_IRUGO, mxt_noise_recover_show, mxt_noise_recover_store);

static struct attribute *mXT2954_attribute_list[] = {
	&lge_touch_attr_device_name.attr,
	&lge_touch_attr_mxt_info.attr,
	&lge_touch_attr_patch_debug_enable.attr,
	&lge_touch_attr_lpwg_fail_reason.attr,
	&lge_touch_attr_driver_debug_enable.attr,
	&lge_touch_attr_debug_enable.attr,
	&lge_touch_attr_t57_debug_enable.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_rebase.attr,
	&lge_touch_attr_testmode_ver.attr,
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_version.attr,
	&lge_touch_attr_pen_support.attr,
	&lge_touch_attr_mfts.attr,
	&lge_touch_attr_ime_status.attr,
	&lge_touch_attr_self_cap.attr,
	&lge_touch_attr_noise_suppression.attr,
	&lge_touch_attr_noise_recover.attr,
	NULL,
};

static int mXT2954_Initialize(struct i2c_client *client)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int ret = 0;
	int pinstate[2] = { 0 };

	TOUCH_FUNC();

	mutex_init(&i2c_suspend_lock);

	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct mxt2954_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR("Failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->object_table = kzalloc((MXT_OBJECT_NUM_MAX * sizeof(struct mxt2954_object)), GFP_KERNEL);
	if (!ts->object_table) {
		TOUCH_ERR("Failed to allocate memory for mxt2954_object\n");
		return TOUCH_FAIL;
	}

	ts->client = client;
	ts->use_mfts = false;
	if (pDriverData->bootMode == BOOT_MINIOS)
		ts->minios = true;
	else
		ts->minios = false;

	gpio_set_value(TOUCH_GPIO_RESET, 0);

	/* S-unused Regulator L22 */
	ts->vdd_ana = regulator_get(&client->dev, "vdd_ana");
	if (IS_ERR(ts->vdd_ana)) {
		ret = PTR_ERR(ts->vdd_ana);
		TOUCH_ERR("failed to get regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}
	/* E-unused Regulator L22 */

	ts->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		TOUCH_ERR("failed to get regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ts->vcc_dig = regulator_get(&client->dev, "vcc_dig");
	if (IS_ERR(ts->vcc_dig)) {
		ret = PTR_ERR(ts->vcc_dig);
		TOUCH_ERR("failed to get regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	/* S-unused Regulator L22 */
	ret = regulator_set_voltage(ts->vdd_ana, 3300000, 3300000);
	if (ret < 0) {
		TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}
	/* E-unused Regulator L22 */

	ret = regulator_set_voltage(ts->vcc_dig, 3300000, 3300000);
	if (ret < 0) {
		TOUCH_ERR("failed to set regulator voltage ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	/* S-unused Regulator L22 */
	ret = regulator_enable(ts->vdd_ana);
	if (ret < 0) {
		TOUCH_ERR("failed to enable regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = regulator_disable(ts->vdd_ana);
	if (ret < 0) {
		TOUCH_ERR("failed to enable regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	TOUCH_LOG("Regulator(L22) power status : %d\n", regulator_is_enabled(ts->vdd_ana));
	/* E-unused Regulator L22 */

	gpio_set_value(TOUCH_LDO_AVDD, 1);

	ret = regulator_enable(ts->vcc_i2c);
	if (ret < 0) {
		TOUCH_ERR("failed to enable regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	ret = regulator_enable(ts->vcc_dig);
	if (ret < 0) {
		TOUCH_ERR("failed to enable regulator ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	msleep(1);
	gpio_set_value(TOUCH_GPIO_RESET, 1);
	msleep(MXT_RESET_TIME);

	TouchResetCtrl(0);
	msleep(5);
	TouchResetCtrl(1);
	msleep(MXT_RESET_TIME);

	if (gpio_is_valid(TOUCH_GPIO_INTERRUPT))
		pinstate[0] = gpio_get_value(TOUCH_GPIO_INTERRUPT);

	if (gpio_is_valid(TOUCH_GPIO_RESET))
		pinstate[1] = gpio_get_value(TOUCH_GPIO_RESET);

	TOUCH_LOG("pin state [INT : %d RST : %d]\n", pinstate[0], pinstate[1]);

	sysfs_bin_attr_init(&ts->mem_access_attr);
	ts->mem_access_attr.attr.name = "mem_access";
	ts->mem_access_attr.attr.mode = S_IWUSR | S_IRUSR;
	ts->mem_access_attr.read = mxt_mem_access_read;
	ts->mem_access_attr.write = mxt_mem_access_write;
	ts->mem_access_attr.size = ts->mem_size;

	if (sysfs_create_bin_file(&pDriverData->client->dev.kobj, &ts->mem_access_attr) < 0) {
		TOUCH_LOG("Failed to create %s\n", ts->mem_access_attr.attr.name);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static void mXT2954_Reset(struct i2c_client *client)
{
	TOUCH_FUNC();

	TouchResetCtrl(0);
	msleep(10);
	TouchResetCtrl(1);
	msleep(200);
}


//====================================================================
// Function : Dummy_QueryDeviceConnection
// Description
//   - Check if touch IC was connected
//   - will be called at module init ( to select proper device driver function )
//   - implement using "Maker ID Pin" or "Read special register of IC"
//   - In case of using "Read special register of IC", you should be implement it using "touch_i2c_read_for_query()"
//====================================================================
static int mXT2954_Connect(void)
{
	TOUCH_FUNC();

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : Dummy_InitRegister
// Description
//   - Initialize touch IC register
//   - will be called after IC reset ( by reset pin )
//====================================================================
static int mXT2954_InitRegister(struct i2c_client *client)
{
	int ret = 0;
	TOUCH_FUNC();

	ret = mxt_t6_command(ts, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret == TOUCH_FAIL) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : Dummy_ClearInterrupt
// Description
//   - Clear interrupt
//   - will be called before interrupt enable to clear interrupt happened during interrupt disabled time
//====================================================================
static void mXT2954_ClearInterrupt(struct i2c_client *client)
{
	TOUCH_FUNC();

	mxt_t6_command(ts, MXT_COMMAND_CALIBRATE, 1, false);
	ts->currState = STATE_NORMAL;

	return;
}

static int mxt_input_open(struct input_dev *dev)
{
	//struct mxt2954_ts_data *ts = input_get_drvdata(dev);
	TOUCH_FUNC();
	//mxt_start(ts);

	return TOUCH_SUCCESS;
}

static void mxt_input_close(struct input_dev *dev)
{
	//struct mxt2954_ts_data *ts = input_get_drvdata(dev);
	TOUCH_FUNC();
	//mxt_stop(ts);
}

static int mxt_t6_command(struct mxt2954_ts_data *ts, u16 cmd_offset, u8 value, bool wait)
{
	u16 reg = 0;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret = 0;

	reg = ts->T6_address + cmd_offset;

	ret = Mxt2954_I2C_Write(ts->client, reg, &value, 1);

	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = Mxt2954_I2C_Read(ts->client, reg, 2, &command_register, 1);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		TOUCH_LOG("%s Command failed!\n", __func__);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mxt_read_t100_config(struct mxt2954_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	struct mxt2954_object *object;
	int error;
	u16 range_x = 0;
	u16 range_y = 0;
	u8 cfg = 0;
	u8 tchaux = 0;
	u8 aux = 0;

	object = mxt_get_object(ts, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = Mxt2954_I2C_Read(client, object->start_address + MXT_T100_XRANGE, 2, &range_x, sizeof(range_x));
	if (error)
		return TOUCH_FAIL;

	le16_to_cpus(range_x);

	error = Mxt2954_I2C_Read(client, object->start_address + MXT_T100_YRANGE, 2, &range_y, sizeof(range_y));
	if (error)
		return TOUCH_FAIL;

	le16_to_cpus(range_y);

	error = Mxt2954_I2C_Read(client, object->start_address + MXT_T100_CFG1, 2, &cfg, 1);

	if (error)
		return TOUCH_FAIL;

	error = Mxt2954_I2C_Read(client, object->start_address + MXT_T100_TCHAUX, 2, &tchaux, 1);

	if (error)
		return TOUCH_FAIL;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		ts->max_x = range_y;
		ts->max_y = range_x;
	} else {
		ts->max_x = range_x;
		ts->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		ts->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		ts->t100_aux_ampl = aux++;
	else
		ts->t100_aux_ampl = aux;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		ts->t100_aux_area = aux++;

	dev_info(&client->dev, "T100 Touchscreen size X%uY%u\n", ts->max_x, ts->max_y);

	return TOUCH_SUCCESS;
}

int mxt_initialize_t100_input_device(struct mxt2954_ts_data *ts)
{
	struct device *dev = &ts->client->dev;
	TouchDriverData *pDriverData = i2c_get_clientdata(ts->client);
	int error = 0;

	error = mxt_read_t100_config(ts);
	if (error)
		dev_warn(dev, "Failed to initialize T100 resolution\n");

	/*channal size init for reference check*/
	error = Mxt2954_I2C_Read(ts->client, ts->T100_address + 8, 2, &ts->channel_size.start_x, 1);
	error = Mxt2954_I2C_Read(ts->client, ts->T100_address + 9, 2, &ts->channel_size.size_x, 1);
	error = Mxt2954_I2C_Read(ts->client, ts->T100_address + 19, 2, &ts->channel_size.start_y, 1);
	error = Mxt2954_I2C_Read(ts->client, ts->T100_address + 20, 2, &ts->channel_size.size_y, 1);

	ts->channel_size.size_y += 18;

	pDriverData->input_dev->phys = ts->phys;
	pDriverData->input_dev->id.bustype = BUS_I2C;
	pDriverData->input_dev->open = mxt_input_open;
	pDriverData->input_dev->close = mxt_input_close;

	if (!error)
		TOUCH_LOG("Succeed to read channel_size %d %d %d %d\n",
			ts->channel_size.start_x,
			ts->channel_size.start_y,
			ts->channel_size.size_x,
			ts->channel_size.size_y);

	return TOUCH_SUCCESS;
}

static void mxt_dump_message(struct mxt2954_ts_data *ts, u8 *message)
{
	print_hex_dump(KERN_ERR, "[Touch] MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, ts->T5_msg_size, false);
}

static void mxt_proc_t100_anti_message(struct mxt2954_ts_data *ts, u8 *message)
{
	TOUCH_FUNC();
#if 0 // Not Use this function !!
	u8 scr_status;
	u8 num_rpt_touch;

	scr_status = message[1] & 0x80;
	if (scr_status)
		num_rpt_touch = message[2];
	else if (message[1] == 0)
		num_rpt_touch = 0;

	data->anti->inter_area = (message[8] << 8) | message[7];
	data->anti->anti_area = (message[6] << 8) | message[5];
	data->anti->touch_area = (message[4] << 8) | message[3];

	if (global_mxt_data != NULL)
		global_mxt_data->anti->fcnt0_msg_cnt = num_rpt_touch;

	/* release all fingers after touch suppression */
/*
	if(scr_status & MXT_T100_FRIST_ID_SUPPRESSION) {
		TOUCH_INFO_MSG("T100_message First Report id has SUP!!! %02X Release all\n", scr_status);
		mxt_reset_slots(data);
		return;
	}
*/
#endif
}

static int mxt_proc_t100_message(struct mxt2954_ts_data *ts, u8 *message, TouchReadData *pData)
{
	int id;
	int x;
	int y;
	int area;
	int amplitude;
	u8 status;
	u8 vector;
	u8 height;
	u8 width;
	TouchDriverData *pDriverData = i2c_get_clientdata(ts->client);

	/* do not report events if input device not yet registered */

	if (pDriverData->lpwgSetting.lcdState == POWER_OFF) {
		TOUCH_LOG("return event\n");
		return TOUCH_SUCCESS;
	}

	id = message[0] - ts->T100_reportid_min - 2;	/* Finger ID */
	status = message[1];	/* Finger Status */
	x = (message[3] << 8) | message[2];	/* X coordinate */
	y = (message[5] << 8) | message[4];	/* Y coordinate */
	vector =  message[ts->t100_aux_vect];		/* Orientation */
	amplitude = message[ts->t100_aux_ampl];	/* Pressure */
	area = message[ts->t100_aux_area];

	height = message[7];	/* Major, Minor */
	width = message[8];	/* Major, Minor */
	pData->type = DATA_FINGER;

	if (status & MXT_T100_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		* status messages, indicating all the events that have
		* happened */

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE || (status & MXT_T100_STATUS_MASK) == MXT_T100_SUPPRESSION) {
			ts->ts_data.curr_data[id].id = id;
			ts->ts_data.curr_data[id].status = FINGER_RELEASED;
			if((status & MXT_T100_STATUS_MASK) == MXT_T100_SUPPRESSION)
				TOUCH_LOG(  "T100_message[%u] ###DETECT && SUPPRESSION (%02X)\n", id, status);
		}

			ts->ts_data.curr_data[id].id = id;
			ts->ts_data.curr_data[id].x_position = x;
			ts->ts_data.curr_data[id].y_position = y;
			ts->ts_data.curr_data[id].pressure = amplitude;
			ts->ts_data.curr_data[id].tool = MT_TOOL_FINGER;

			if (height >= width) {
				ts->ts_data.curr_data[id].touch_major = height;
				ts->ts_data.curr_data[id].touch_minor = width;
			} else {
				ts->ts_data.curr_data[id].touch_major = width;
				ts->ts_data.curr_data[id].touch_minor = height;
			}

			if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_PALM) {
				ts->ts_data.curr_data[id].pressure = 255;
			} else {
				ts->ts_data.curr_data[id].pressure = amplitude;
			}

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) {
			ts->ts_data.curr_data[id].status = FINGER_PRESSED;
		} else if((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE){
			ts->ts_data.curr_data[id].status = FINGER_MOVED;
		}

		/* Pen support */
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS) { /* PEN */
			//ts->ts_data.curr_data[id].is_pen= true;
			//ts->ts_data.curr_data[id].is_palm= false;
			if (ts->ts_data.curr_data[id].pressure == 255)
				ts->ts_data.curr_data[id].pressure = 254;
		} else if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_PALM) {  //(area >= data->T100_palm_threshold) {  /* PALM */
			//ts->ts_data.curr_data[id].is_pen= false;
			//ts->ts_data.curr_data[id].is_palm= true;
			ts->ts_data.curr_data[id].pressure = 255;
		} else { /* FINGER */
			//ts->ts_data.curr_data[id].is_pen= false;
			//ts->ts_data.curr_data[id].is_palm= false;
			if (ts->ts_data.curr_data[id].pressure == 255)
				ts->ts_data.curr_data[id].pressure = 254;
		}

		if (ts->driver_debug_enabled) {
			if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS)
				TOUCH_LOG("touch_pressed	<%d> : x[%3d] y[%3d] P[%3d] WM[%3d] Wm[%3d] \n",
				id, x, y, amplitude, ts->ts_data.curr_data[id].touch_major, ts->ts_data.curr_data[id].touch_minor);
		}
	} else {
		/* Touch Release */
		ts->ts_data.curr_data[id].id = id;
		ts->ts_data.curr_data[id].status = FINGER_RELEASED;

		if (ts->driver_debug_enabled)
			TOUCH_LOG("touch_release	<%d> : x[%3d] y[%3d]\n", id, x, y);
	}

	if (ts->debug_enabled) {
		TOUCH_LOG( "T100_message[%u] %s%s%s%s%s%s%s%s%s %s%s%s%s%s (0x%02X) x:%u y:%u z:%u area:%u amp:%u vec:%u h:%u w:%u\n",
			id,
			((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE) ? "MOVE" : "",
			((status & MXT_T100_STATUS_MASK) == 2) ? "UNSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 3) ? "SUP" : "",
			((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) ? "PRESS" : "",
			((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE) ? "RELEASE" : "",
			((status & MXT_T100_STATUS_MASK) == 6) ? "UNSUPSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 7) ? "UNSUPUP" : "",
			((status & MXT_T100_STATUS_MASK) == 8) ? "DOWNSUP" : "",
			((status & MXT_T100_STATUS_MASK) == 9) ? "DOWNUP" : "",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_FINGER) ? "FIN" : ".",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS) ? "PEN" : ".",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_PALM) ? "PALM" : ".",
			((status & MXT_T100_TYPE_MASK) == 0x40) ? "HOVER" : ".",
			((status & MXT_T100_TYPE_MASK) == 0x30) ? "ACTSTY" : ".",
			status, x, y, ts->ts_data.curr_data[id].pressure, area, amplitude, vector,
			height, width);
	}

	ts->update_input = true;

	return TOUCH_SUCCESS;
}

static int mxt_proc_t9_message(struct mxt2954_ts_data *ts, u8 *message, TouchReadData *pData)
{
	TouchFingerData *pFingerData = NULL;
	int id = 0;
	int x = 0;
	int y = 0;
	int area = 0;
	int amplitude = 0;
	u8 vector = 0;
	u8 status = 0;

	id = message[0] - ts->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));
	area = message[5];
	amplitude = message[6];
	vector = message[7];
	pData->type = DATA_FINGER;

	/* Handle 10/12 bit switching */
	if (ts->max_x < 1024)
		x >>= 2;
	if (ts->max_y < 1024)
		y >>= 2;

	if (unlikely(id < 0 || id >= MAX_FINGER)) {
		TOUCH_ERR("%s wrong id:%d\n", __func__, id);
		return TOUCH_FAIL;
	}
	if (status & MXT_T9_SUPPRESS) {
			TOUCH_LOG(" MXT_T9_SUPPRESS\n");
			mxt_reset_slots(ts);
	}
	if (status & MXT_T9_DETECT) {
		if ((status & MXT_T9_PRESS) || (status & MXT_T9_MOVE)) {
			pFingerData = &pData->fingerData[++id];
			pFingerData->id = id;
			pFingerData->x = x;
			pFingerData->y = y;
			pFingerData->width_major = area;
			pFingerData->width_minor = 0;
			pFingerData->orientation = vector;
			pFingerData->pressure = amplitude;
		}
	}

	if (status & MXT_T9_RELEASE) {
			memset(&pData->fingerData[id], 0x00, sizeof(TouchFingerData));
	}
	/*TOUCH_LOG("co:id[%d] x[%d] y[%d] ma[%d] mi[%d] o[%d] p[%d] t[%d]\n",
		pData->fingerData[id].id ,
		pData->fingerData[id].x,
		pData->fingerData[id].y,
		pData->fingerData[id].width_major,
		pData->fingerData[id].width_minor,
		pData->fingerData[id].orientation,
		pData->fingerData[id].pressure,
		pData->fingerData[id].type);*/

	return TOUCH_SUCCESS;
}
static void mxt_proc_t6_messages(struct mxt2954_ts_data *ts, u8 *msg, TouchReadData *pData)
{
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);
	struct mxt2954_object *object = NULL;

	if (crc != ts->config_crc) {
		ts->config_crc = crc;
		TOUCH_LOG("T6 Config Checksum: 0x%06X\n", crc);
	}

	/* Detect transition out of reset */
#if 0
	if ((ts->t6_status & MXT_T6_STATUS_RESET) && !(status & MXT_T6_STATUS_RESET))
		complete(&ts->reset_completion);
#endif

	/* Output debug if status has changed */
	if (status != ts->t6_status)
		TOUCH_LOG("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			(status == 0) ? " OK" : "",
			(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
			(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
			(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
			(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
			(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
			(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

	/* Save current status */
	ts->t6_status = status;

	if (status & MXT_T6_STATUS_CAL || status & MXT_T6_STATUS_RESET) {
		mxt_reset_slots(ts);

	/* ATMEL FOR SELF CAL */
		object = mxt_get_object(ts, MXT_SPT_USERDATA_T38);
		if (object) {
			Mxt2954_I2C_Read(ts->client, object->start_address, 2, ts->self_delta_chk, 23);
		}

		if (ts->self_delta_chk[22] == 1) {
			TOUCH_LOG("[%s][self_delta_check]\n", __func__);
			object = mxt_get_object(ts, MXT_SPT_TIMER_T61);
			if (object) {
				Mxt2954_I2C_Write(ts->client, object->start_address + mxt_obj_size(object) * 2 + 3, ts->self_delta_chk, 2);
				Mxt2954_I2C_Write(ts->client, object->start_address + mxt_obj_size(object) * 2 + 1, ts->self_delta_chk, 1);
			}
		}
	}

	/* Set KnockCode Delay after RESET */
	if (!ts->use_mfts) {
		if (status & MXT_T6_STATUS_RESET && ts->is_knockCodeDelay) {
			mxt_write_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 19, 43);
			TOUCH_LOG("Set Knock Code delay after RESET (700ms)\n");
		} else if (status & MXT_T6_STATUS_RESET && !ts->is_knockCodeDelay) {
			mxt_write_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 19, 0);
			TOUCH_LOG("Set Knock Code delay after RESET (0ms)\n");
		}
	}
}

static int mxt_proc_t25_message(struct mxt2954_ts_data *ts, u8 *message)
{
	u8 status = message[1];

	if (!selftest_enable)
		return 0;

	TOUCH_LOG("T25 Self Test completed %u\n",status);

	memset(ts->self_test_status, 0, sizeof(ts->self_test_status));

	if (selftest_show)
		ts->self_test_status[0] = status;

	if ( status == 0xFE ) {
		TOUCH_LOG("[SUCCESS] All tests passed\n");
		ts->self_test_result = true;
	} else {
		if (status == 0xFD) {
			TOUCH_ERR("[FAIL] Invalid test code\n");
		} else if (status == 0xFC)  {
			TOUCH_ERR("[FAIL] Unrelated fault\n");
		} else if (status == 0x01) {
			TOUCH_ERR("[FAIL] AVdd or XVdd is not present\n");
		} else if (status == 0x12) {
			TOUCH_ERR("[FAIL] Pin fault (SEQ_NUM %u, X_PIN %u, Y_PIN %u)\n", message[2], message[3], message[4]);
			if (selftest_show) {
				ts->self_test_status[1] = message[2];
				ts->self_test_status[2] = message[3];
				ts->self_test_status[3] = message[4];
			}
		} else if (status == 0x17) {
			TOUCH_ERR("[FAIL] Signal limit fault (TYPE_NUM %u, TYPE_INSTANCE %u)\n", message[2], message[3]);
			if (selftest_show) {
				ts->self_test_status[1] = message[2];
				ts->self_test_status[2] = message[3];
			}
		} else;
		ts->self_test_result = false;
	}

	selftest_enable = false;
	return TOUCH_SUCCESS;
}

static int mxt_proc_t37_message(struct mxt2954_ts_data *ts, u8 *msg_buf, TouchReadData *pData)
{
	struct mxt2954_object *object = NULL;
	u8 *buf = NULL;
	u8 result = 0;
	int i = 0;
	int cnt = 0;
	int tap_num = 0;
	int msg_size = 0;
	int x = 0;
	int y = 0;
	int ret = 0;

	object = mxt_get_object(ts, MXT_DEBUG_DIAGNOSTIC_T37);

	if (!object) {
		TOUCH_LOG("error Cannot get object_type T%d\n", MXT_DEBUG_DIAGNOSTIC_T37);
		goto error;
	}

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_LOG("error object_type T%d\n", object->type);
		goto error;
	}

retry:
	msleep(50);	/* to need time to write new data */

	ret = Mxt2954_I2C_Read(ts->client, object->start_address, 2, &result, 1);

	if (ret != 0)
		goto error;

	if (result != UDF_MESSAGE_COMMAND) {
		if (cnt == 5) {
			TOUCH_LOG("cnt = 5, result= %d\n", result);
			goto error;
		}

		msleep(20);
		cnt++;
		goto retry;
	}

	ret = Mxt2954_I2C_Read(ts->client, object->start_address + 2, 2, &result, 1);
	if (ret != 0)
		goto error;

	tap_num = result;

	if (ts->g_tap_cnt != tap_num && ts->mxt_multi_tap_enable) {
	    TOUCH_LOG("Tab count dismatch\n");
	    goto error;
	} else {
	    TOUCH_LOG("TAP Mode\n");
	}

	msg_size = tap_num * MAX_T37_MSG_SIZE ;
	buf = kmalloc(msg_size, GFP_KERNEL);
	if (!buf)
		goto error;

	ret = Mxt2954_I2C_Read(ts->client, object->start_address + 3, 2, buf, msg_size);

	if (ret != 0)
		goto error;

	for (i = 0; i < tap_num ; i++) {
		cnt = i * MAX_T37_MSG_SIZE;

		if (ts->driver_debug_enabled)
			TOUCH_LOG("cnt:[%d][%d]\n", cnt, i);

		x = (buf[cnt + 1] << 8) | buf[cnt];
		y = (buf[cnt + 3] << 8) | buf[cnt + 2];

		x = (buf[cnt + 5] << 8) | buf[cnt + 4];
		y = (buf[cnt + 7] << 8) | buf[cnt + 6];
		pData->knockData[i].x = x;
		pData->knockData[i].y = y;

		if (ts->driver_debug_enabled)
			TOUCH_LOG("[NC][Report] Cordinates x[%d]:[%d] y[%d]:[%d]\n", i, pData->knockData[i].x, i, pData->knockData[i].y);
	}

	if (buf)
		kfree(buf);

	if (ts->driver_debug_enabled)
		TOUCH_LOG("Send Knock Code type[%d] cnt [%d]\n", pData->type, pData->count);

	return TOUCH_SUCCESS;

error:
	TOUCH_LOG("T37 error\n");
	if (buf)
		kfree(buf);

	return TOUCH_FAIL;
}

static void mxt_proc_t93_messages(struct mxt2954_ts_data *ts, u8 *message, TouchReadData *pData)
{
	u8 lpwg_mode_msg = 0;
	u8 lpwg_fail_msg = 0;

	lpwg_mode_msg = message[1];
	lpwg_fail_msg = message[2];

	if (lpwg_fail_msg)
		TOUCH_LOG("T93 lpwg_mode_msg:0x%x, lpwg_fail_msg:0x%x \n", lpwg_mode_msg, lpwg_fail_msg);

	if (lpwg_mode_msg & 0x01) {
		mxt_t6_command(ts, MXT_COMMAND_DIAGNOSTIC, UDF_MESSAGE_COMMAND, false);
		pData->type = DATA_KNOCK_CODE;
		TOUCH_LOG("T93[%u] Knock Code !!!\n", lpwg_mode_msg);
		mxt_proc_t37_message(ts, message, pData);
	} else if (lpwg_mode_msg & 0x02) {
		TOUCH_LOG("wake_lock_timeout 3000ms for stable interrupt\n");
		wake_lock_timeout(pWakeLockTouch, msecs_to_jiffies(3000));
		pData->type = DATA_KNOCK_ON;
		TOUCH_LOG("T93[%u] Knock On !!!\n", lpwg_mode_msg);
	}

}

/* T-series of Atmel Touch IC
 * The Touch Suppression T42 does not report its own messages.
 * Screen suppression messages are reported through the linked
 * Multiple Touch Touchscreen T100 object. */
static void mxt_proc_t42_messages(struct mxt2954_ts_data *ts, u8 *msg)
{
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP) {
		TOUCH_LOG("Palm detected\n");
		//data->button_lock = true;
	} else {
		TOUCH_LOG("Palm released\n");
		//queue_delayed_work(touch_wq, &data->work_button_lock, msecs_to_jiffies(200));
	}
	mxt_reset_slots(ts);
}

static int mxt_proc_t48_messages(struct mxt2954_ts_data *ts, u8 *msg)
{
	u8 status = 0, state = 0;

	status = msg[1];
	state  = msg[4];

	TOUCH_LOG("T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return TOUCH_SUCCESS;
}

static void mxt_proc_t57_messages(struct mxt2954_ts_data *ts, u8 *message)
{
	u16 area = 0;
	u16 touch_area = 0;
	u16 anti_touch_area = 0;

	area =				(message[2] << 8 | message[1]);
	touch_area =			(message[4] << 8 | message[3]);
	anti_touch_area =		(message[6] << 8 | message[5]);

	if (ts->t57_debug_enabled || ts->ref_chk)
		TOUCH_LOG("T57 :%3d %3d %3d\n", area, touch_area, anti_touch_area);
}

//thermal solution
void trigger_low_temp_state_from_batt(int low_temp)
{

	TOUCH_LOG("%s curr_low_temp= %d, low_temp=%d \n", __func__,curr_low_temp,low_temp);
	if(low_temp == 1 && global_ts){
		mutex_lock(&i2c_suspend_lock);
		mxt_patch_event(global_ts, PATCH_LOW_TEMP);
		TOUCH_LOG("Very Cold please go to warm place!! \n");
		mutex_unlock(&i2c_suspend_lock);
		curr_low_temp = low_temp;
		return;
	}

	if (curr_low_temp == low_temp) {
                return;
	}
	else
		curr_low_temp = low_temp;


	if(global_ts)
	{
		if (low_temp == 0) {
			mutex_lock(&i2c_suspend_lock);
			mxt_patch_event(global_ts, PATCH_NOT_LOW_TEMP);
			TOUCH_LOG("Good to move !! \n");
			mutex_unlock(&i2c_suspend_lock);
		}
	}
}

void trigger_usb_state_from_otg(int usb_type)
{
	int error = 0;
	TOUCH_LOG("USB trigger USB_type: %d \n", usb_type);

	selfd_check_usb_type = usb_type;

	if (global_ts && global_ts->patch.event_cnt) {

		TOUCH_LOG("patch.event_cnt is %d\n", global_ts->patch.event_cnt);
		global_ts->global_object = mxt_get_object(global_ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
		if (!global_ts->global_object)
			return;

		if (global_ts->use_mfts == true) {
			TOUCH_LOG("MFTS : Not support USB trigger\n");
			return;
		}
#if 0
		wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(2000));
#endif
		if (mutex_is_locked(&i2c_suspend_lock)) {
			TOUCH_LOG("%s mutex_is_locked \n", __func__);
		} else {
			mutex_lock(&i2c_suspend_lock);
		}

		if (usb_type == 0) {
			if (mxt_patchevent_get(PATCH_EVENT_TA)) {
				TOUCH_LOG("TA Check in usb type :[%d] \n", usb_type);
				if (mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
					mxt_patchevent_unset(PATCH_EVENT_KNOCKON);
					TOUCH_LOG("Get & Unset for Knock On in usb type :[%d] \n", usb_type);
					if (global_ts->currState == STATE_KNOCK_ON_ONLY) {
						TOUCH_LOG("USB Type :%d & Knock On only state\n", usb_type);
						mxt_patch_event(global_ts, CHARGER_KNOCKON_WAKEUP);
					} else if (global_ts->currState == STATE_KNOCK_ON_CODE) {
						TOUCH_LOG("USB Type :%d & Knock On/Code state\n", usb_type);
						mxt_patch_event(global_ts, CHARGER_KNOCKON_WAKEUP + PATCH_EVENT_PAIR_NUM);
						/* Write TCHCNTTHR(Touch Count Threshold) for LPWG_MULTI_TAP */
						error = Mxt2954_I2C_Write(global_ts->client, global_ts->global_object->start_address+17, &(global_ts->g_tap_cnt), 1);
						if (error)
							TOUCH_ERR("Object Write Fail\n");
					}
				}
				global_ts->charging_mode = 0;
				mxt_patch_event(global_ts, CHARGER_UNplugged);
				mxt_patchevent_unset(PATCH_EVENT_TA);
			}
		} else {
			if (mxt_patchevent_get(PATCH_EVENT_KNOCKON)) {
				TOUCH_LOG("get Knock On patch in usb type :[%d] \n", usb_type);
				if (global_ts->currState == STATE_KNOCK_ON_ONLY){
					TOUCH_LOG("USB Type :%d & Knock On only state\n", usb_type);
					mxt_patch_event(global_ts, NOCHARGER_KNOCKON_WAKEUP);
				} else if (global_ts->currState == STATE_KNOCK_ON_CODE) {
					TOUCH_LOG("USB Type :%d & Knock On/Code state\n", usb_type);
					mxt_patch_event(global_ts, NOCHARGER_KNOCKON_WAKEUP + PATCH_EVENT_PAIR_NUM);
					/* Write TCHCNTTHR(Touch Count Threshold) for LPWG_MULTI_TAP */
					error = Mxt2954_I2C_Write(global_ts->client, global_ts->global_object->start_address+17, &(global_ts->g_tap_cnt), 1);
					if (error)
						TOUCH_ERR("Object Write Fail\n");
				}
				mxt_patchevent_unset(PATCH_EVENT_KNOCKON);
			}
			global_ts->charging_mode = 1;
			if (global_ts->minios == true && global_ts->use_mfts == false)
				mxt_patch_event(global_ts, CHARGER_PLUGGED_AAT);
			else
				mxt_patch_event(global_ts, CHARGER_PLUGGED);
			mxt_patchevent_set(PATCH_EVENT_TA);
		}

	mutex_unlock(&i2c_suspend_lock);
	} else {
		if (global_ts == NULL)
			TOUCH_ERR("global_ts is null\n");
	}
}

static int mxt_proc_message(struct mxt2954_ts_data *ts, u8 *message, TouchReadData *pData)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(ts->client);
	u8 report_id = message[0];
	u8 type = 0;
	bool dump = ts->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return TOUCH_SUCCESS;

	type = ts->reportids[report_id].type;

	if(ts->driver_debug_enabled) {
		TOUCH_LOG("mxt_interrupt T%d & Mode :%d\n", type, ts->lpwgSetting.mode);
		TOUCH_LOG("isSuspend : (%d) & LCD Status check : (%d)\n", pDriverData->isSuspend, pDriverData->lpwgSetting.lcdState);
	}

	if (type == MXT_GEN_COMMAND_T6) {
		mxt_proc_t6_messages(ts, message, pData);
	} else if (type == MXT_TOUCH_MULTI_T9) {
		mxt_proc_t9_message(ts, message, pData);
	} else if (type == MXT_SPT_SELFTEST_T25) {
		mxt_proc_t25_message(ts, message);
	} else if (type == MXT_PROCI_TOUCHSUPPRESSION_T42) {
		TOUCH_LOG("MXT_PROCI_TOUCHSUPPRESSION_T42");
		mxt_proc_t42_messages(ts, message);
	} else if (type == MXT_PROCG_NOISESUPPRESSION_T48) {
		TOUCH_LOG("MXT_PROCG_NOISESUPPRESSION_T48");
		mxt_proc_t48_messages(ts, message);
	} else if (type == MXT_PROCI_SHIELDLESS_T56) {
		TOUCH_LOG("MXT_PROCI_SHIELDLESS_T56");
	} else if (type == MXT_PROCI_EXTRATOUCHSCREENDATA_T57) {
		mxt_proc_t57_messages(ts, message);
	} else if (type == MXT_PROCG_NOISESUPPRESSION_T72) {
		TOUCH_LOG("MXT_PROCG_NOISESUPPRESSION_T72");
	} else if (type == MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93 && ts->lpwgSetting.mode) {
		if (pDriverData->isSuspend == false) {
			TOUCH_LOG("isSuspend:[%d] Not entry touch driver suspend\n", pDriverData->isSuspend);
			//return TOUCH_SUCCESS;
		}
		mxt_proc_t93_messages(ts, message, pData);
	} else if (type == MXT_TOUCH_MULTITOUCHSCREEN_T100 /* && (pDriverData->lpwgSetting.lcdState == POWER_ON) */) {

		if(ts->driver_debug_enabled)
			TOUCH_LOG("report_id = %d , ts->T100_reportid_min = %d\n", report_id, ts->T100_reportid_min);

		if (report_id == ts->T100_reportid_min || report_id == ts->T100_reportid_min + 1) {
			mxt_proc_t100_anti_message(ts, message);
		} else {
			mxt_proc_t100_message(ts, message, pData);
		}
	} else if (type == MXT_SPT_TIMER_T61){
		TOUCH_LOG("MXT_SPT_TIMER_T61\n");
		/* mxt_proc_t61_messages(ts, message); */
	} else {
		if (type != MXT_SPT_TIMER_T61) {
			TOUCH_LOG("%s : Unknown T%d\n", __func__, type);
			mxt_dump_message(ts, message);
		}
	}

	if (dump)
		mxt_dump_message(ts, message);

	if (mxt_power_block_get() == 0)
		mxt_patch_message(ts, (struct mxt2954_message*)message);

	return TOUCH_SUCCESS;
}

static int mxt_lpwg_debug_interrupt_control(struct mxt2954_ts_data* ts, int mode, u32 state)
{
	int ret = -1;
	u8 value[2] = {0,};
	u8 read_value[2] = {0,};
	struct mxt2954_object *object = NULL;

	TOUCH_LOG("%s\n", __func__);

	if(!mode)
		return TOUCH_SUCCESS;

	object = mxt_get_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (object) {
		ret = Mxt2954_I2C_Read(ts->client, object->start_address + 28, 2, read_value, 2);
		if(ret) {
			TOUCH_ERR("Read Failed T93\n");
			return TOUCH_FAIL;
		} else {
			TOUCH_LOG("Debug reason ctrl status : 0x%x, 0x%x\n", read_value[0], read_value[1]);
		}
	} else {
		return TOUCH_SUCCESS;
	}

	if(state == STATE_KNOCK_ON_ONLY) {
		value[0] = 0x1E;
		value[1] = 0x00;
	} else if(state== STATE_KNOCK_ON_CODE) {
		value[0] = 0x3C;
		value[1] = 0x00;
	}

	if((read_value[0] == value[0]) && (read_value[1] == value[1])){
		TOUCH_LOG("T93 FAILRPTEN Field write skip\n");
		return TOUCH_SUCCESS;
	}

	if (object) {
		ret = Mxt2954_I2C_Write(ts->client, object->start_address + 28, value, 2);
		if(ret) {
			TOUCH_ERR("Write Failed T93\n");
			return TOUCH_FAIL;
		}
		ret = Mxt2954_I2C_Read(ts->client, object->start_address + 28, 2, value, 2);
		if(ret) {
			TOUCH_ERR("Read Failed T93\n");
			return TOUCH_FAIL;
		} else {
			TOUCH_LOG("New debug reason ctrl status:0x%x, 0x%x\n", value[0], value[1]);
		}
	}
	return TOUCH_SUCCESS;
}

static int mxt_read_and_process_messages(struct mxt2954_ts_data *ts, u8 count, TouchReadData *pData)
{
	int ret = 0;
	int i = 0;
	u8 num_valid = 0;

	/* safety check for msg_buf */
	if (count > ts->max_reportid)
		return TOUCH_FAIL;

	if (ts->msg_buf == NULL)
		TOUCH_ERR("ts->msg_buf = NULL\n");

	/* Process remaining messages if necessary */
	ret = Mxt2954_I2C_Read(ts->client, ts->T5_address, 2, ts->msg_buf,  ts->T5_msg_size * count);
	if (ret) {
		TOUCH_ERR("Failed to read %u messages (%d)\n", count, ret);
		return TOUCH_FAIL;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(ts, ts->msg_buf + ts->T5_msg_size * i, pData);

		if (ret == TOUCH_SUCCESS)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;

}

static int mxt_process_messages(struct mxt2954_ts_data *ts, TouchReadData *pData)
{
	u8 num_left;
	u8 count;
	int ret = 0;
	int i;
	int report_num = 0;
	int report_count =0;
#if 0 /* 15.07.21 For test */
	int retrycnt = 0;

RETRY:
#endif
	if (!regulator_is_enabled(ts->vcc_i2c)) {
		TOUCH_LOG( "I2C Regulator Already Disabled.\n");
		return TOUCH_SUCCESS;
	}

	/* Read T44 and T5 together */
	ret = Mxt2954_I2C_Read(ts->client, ts->T44_address, 2, ts->msg_buf, ts->T5_msg_size + 1);
	if (ret) {
		TOUCH_ERR( "Failed to read T44 and T5 (%d)\n", ret);
		return TOUCH_FAIL;
	}
#if 0 /* 15.07.21 For test */
	if (ret < 0) {
		if (retrycnt >= RETRY_CNT) {
			TOUCH_ERR("Touch IC Failure : can not setting register\n");
			return TOUCH_FAIL;
		} else {
			TOUCH_ERR("Failed to read T44 and T5 (%d)\n", ret);
			TOUCH_ERR("Retry : Can not setting register(%d / %d)\n", retrycnt + 1, RETRY_CNT);

			TouchDisableIrq();

			mXT2954_Reset(ts->client);

			TouchEnableIrq();

			retrycnt++;

			goto RETRY;
		}
	}
#endif
	count = ts->msg_buf[0];

	if (ts->driver_debug_enabled)
		TOUCH_LOG("First count(msg_buf[0]) : %d\n", count);

	if (count == 0) {
		/* Zero message is too much occured.
		 * Remove this log until firmware fixed */
		//TOUCH_INFO_MSG("Interrupt triggered but zero messages\n");
		return TOUCH_SUCCESS;
	} else if (count > ts->max_reportid) {
		TOUCH_ERR("T44 count %d exceeded max report id\n", count);
		count = ts->max_reportid;
	}

	ts->ts_data.total_num = 0;

	/* Process first message */
	ret = mxt_proc_message(ts, ts->msg_buf + 1, pData);
	if (ret < 0) {
		TOUCH_ERR( "Unexpected invalid message\n");
		return TOUCH_FAIL;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(ts, num_left, pData);
		if (ret < 0) {
			TOUCH_ERR("Unexpected invalid message\n");
			return TOUCH_FAIL;
		} else if (ret != num_left) {
			TOUCH_ERR("Unexpected invalid message\n");
		}
	}

	for (i = 0; i < 10; i++) {

		if (ts->driver_debug_enabled)
			TOUCH_LOG("[1] curr_data[%d].status : (%d)\n", i, ts->ts_data.curr_data[i].status);

		if (ts->ts_data.curr_data[i].status == FINGER_INACTIVE &&
			ts->ts_data.prev_data[i].status != FINGER_INACTIVE &&
			ts->ts_data.prev_data[i].status != FINGER_RELEASED) {
			memcpy(&ts->ts_data.curr_data[i], &ts->ts_data.prev_data[i], sizeof(ts->ts_data.prev_data[i]));
			ts->ts_data.curr_data[i].skip_report = true;

		} else if (ts->ts_data.curr_data[i].status == FINGER_INACTIVE) {
			continue;
		}

		if (ts->ts_data.curr_data[i].status == FINGER_PRESSED || ts->ts_data.curr_data[i].status == FINGER_MOVED)
			ts->ts_data.total_num++;

		report_num++;
	}

	if(!report_num)
		return TOUCH_SUCCESS;

	for (i = 0; i < 10; i++) {
		if (ts->ts_data.curr_data[i].status == FINGER_INACTIVE || ts->ts_data.curr_data[i].skip_report) {
			if(ts->ts_data.curr_data[i].skip_report){

				if (ts->driver_debug_enabled)
					TOUCH_LOG("Skip Report - curr_data[%d].status : (%d)\n", i, ts->ts_data.curr_data[i].status);

				pData->fingerData[report_count].id = ts->ts_data.curr_data[i].id;
				pData->fingerData[report_count].x = ts->ts_data.curr_data[i].x_position;
				pData->fingerData[report_count].y = ts->ts_data.curr_data[i].y_position;
				pData->fingerData[report_count].pressure = ts->ts_data.curr_data[i].pressure;
				pData->fingerData[report_count].width_major = ts->ts_data.curr_data[i].touch_major;
				pData->fingerData[report_count].width_minor = ts->ts_data.curr_data[i].touch_minor;

				report_count++;
			}
			continue;
		}

		if (ts->driver_debug_enabled)
			TOUCH_LOG("[2] curr_data[%d].status : (%d)\n", i, ts->ts_data.curr_data[i].status);

		if (ts->ts_data.curr_data[i].status == FINGER_RELEASED && ts->ts_data.prev_data[i].status != FINGER_RELEASED) {
			/* RELEASE */
			/* pData->fingerData[i].id = 0; */
		}  else {
			if (ts->ts_data.curr_data[i].status == FINGER_RELEASED) {
				/* RELEASE */
				/* pData->fingerData[i].id = 0; */
			} else {
				/* PRESS + MOVE */
				pData->fingerData[report_count].id = ts->ts_data.curr_data[i].id;
				pData->fingerData[report_count].x = ts->ts_data.curr_data[i].x_position;
				pData->fingerData[report_count].y = ts->ts_data.curr_data[i].y_position;
				pData->fingerData[report_count].pressure = ts->ts_data.curr_data[i].pressure;
				pData->fingerData[report_count].width_major = ts->ts_data.curr_data[i].touch_major;
				pData->fingerData[report_count].width_minor = ts->ts_data.curr_data[i].touch_minor;

				report_count++;
			}
		}
	}

	if(ts->ts_data.total_num < ts->ts_data.prev_total_num)
		TOUCH_LOG( "Total_num(move+press)= %d\n", ts->ts_data.total_num);
	if (ts->ts_data.total_num) {
		ts->ts_data.prev_total_num = ts->ts_data.total_num;
		memcpy(ts->ts_data.prev_data, ts->ts_data.curr_data, sizeof(ts->ts_data.curr_data));
	} else {
		ts->ts_data.prev_total_num = 0;
		memset(ts->ts_data.prev_data, 0, sizeof(ts->ts_data.prev_data));
	}
	memset(ts->ts_data.curr_data, 0, sizeof(ts->ts_data.curr_data));

	pData->count = report_count;

	return TOUCH_SUCCESS;
}
//====================================================================
// Function : Dummy_InterruptHandler
// Description
//   - process interrupt
//   - will be called if interrupt detected by AP
//====================================================================
static int mXT2954_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	int ret = 0;

	mutex_lock(&i2c_suspend_lock);

	ret = mxt_process_messages(ts, pData);
	if (ret < 0) {
		TOUCH_ERR("mxt_process_messages error\n");
	}

	if (pData->type == DATA_KNOCK_CODE) {
		TOUCH_LOG("[%s] pData->type : %d / pData->count :%d\n", __func__, pData->type, pData->count);
		pData->count = ts->g_tap_cnt;
	}

	mutex_unlock(&i2c_suspend_lock);

	return ret;
}

static int mxt_read_id_info(struct mxt2954_ts_data *ts)
{
	int ret = 0;
	u8 id[MXT_INFOMATION_BLOCK_SIZE] = { 0, };

	/* Read IC information */
	ret = Mxt2954_I2C_Read(ts->client, 0, 2, id, MXT_INFOMATION_BLOCK_SIZE);
	if (ret) {
		TOUCH_ERR("Read fail. IC information\n");
		return TOUCH_FAIL;
	} else {
		TOUCH_LOG("Family:%02X / Variant:%02X / Binary:%u.%u.%02X / TX:%d / RX:%d / Objects:%d\n",
			id[0],
			id[1],
			id[2] >> 4, id[2] & 0xF, id[3],
			id[4],
			id[5],
			id[6]);

		if (ts->info)
			kfree(ts->info);

		ts->info = kzalloc(sizeof(struct mxt2954_info), GFP_KERNEL);
		ts->info->family_id = id[0];
		ts->info->variant_id = id[1];
		ts->info->version = id[2];
		ts->info->build = id[3];
		ts->info->matrix_xsize = id[4];
		ts->info->matrix_ysize = id[5];
		ts->info->object_num = id[6];
	}

	return TOUCH_SUCCESS;
}

static int mxt_verify_fw(struct mxt2954_fw_info *fw_info, const struct firmware *fw)
{
	struct mxt2954_ts_data *ts = fw_info->ts;
	struct mxt2954_fw_image *fw_img = NULL;
	struct patch_header *ppheader = NULL;
	char *extra_info = NULL;
	u8 *patch = NULL;
	u32 ppos = 0;

	if (!fw) {
		TOUCH_ERR("could not find firmware file\n");
		return TOUCH_FAIL;
	}

	fw_img = (struct mxt2954_fw_image *)fw->data;

	if (le32_to_cpu(fw_img->magic_code) != MXT_FW_MAGIC) {
		/* In case, firmware file only consist of firmware */
		TOUCH_LOG("Firmware file only consist of raw firmware\n");
		fw_info->fw_len = fw->size;
		fw_info->fw_raw_data = fw->data;
	} else {
		/*
		 * In case, firmware file consist of header,
		 * configuration, firmware.
		 */
		TOUCH_LOG("Firmware file consist of header, configuration, firmware\n");
		fw_info->bin_ver = fw_img->bin_ver;
		fw_info->build_ver = fw_img->build_ver;
		fw_info->hdr_len = le32_to_cpu(fw_img->hdr_len);
		fw_info->cfg_len = le32_to_cpu(fw_img->cfg_len);
		fw_info->fw_len = le32_to_cpu(fw_img->fw_len);
		fw_info->cfg_crc = le32_to_cpu(fw_img->cfg_crc);

		extra_info = fw_img->extra_info;
		ts->fw_ver[0] = extra_info[0];
		ts->fw_ver[1] = extra_info[1];
		memcpy(ts->product, &extra_info[4], 10);

		/* Check the firmware file with header */
		if ((fw_info->hdr_len != sizeof(struct mxt2954_fw_image))
			|| (fw_info->hdr_len + fw_info->cfg_len + fw_info->fw_len != fw->size)) {

			ppos = fw_info->hdr_len + fw_info->cfg_len + fw_info->fw_len;
			ppheader = (struct patch_header *)(fw->data + ppos);

			if (ppheader->magic == MXT_PATCH_MAGIC) {
				TOUCH_LOG("Firmware file has patch size: %d\n", ppheader->size);

				if (ppheader->size) {
					patch = NULL;
					if (ts->patch.patch) {
						kfree(ts->patch.patch);
						ts->patch.patch = NULL;
					}
					patch = kzalloc(ppheader->size, GFP_KERNEL);
					if (patch) {
						memcpy(patch, (u8 *)ppheader, ppheader->size);
						ts->patch.patch = patch;
						TOUCH_LOG("%s Patch Updated\n", __func__);
					} else {
						ts->patch.patch = NULL;
						TOUCH_ERR("Patch Update Failed\n");
					}
				}
			} else {
				TOUCH_ERR("Firmware file is invaild !!hdr size[%d] cfg,fw size[%d,%d] filesize[%d]\n",
					fw_info->hdr_len, fw_info->cfg_len, fw_info->fw_len, fw->size);
				return TOUCH_FAIL;
			}
		}

		if (!fw_info->cfg_len) {
			TOUCH_ERR("Firmware file dose not include configuration data\n");
			return TOUCH_FAIL;
		}

		if (!fw_info->fw_len) {
			TOUCH_ERR("Firmware file dose not include raw firmware data\n");
			return TOUCH_FAIL;
		}

		/* Get the address of configuration data */
		fw_info->cfg_raw_data = fw_img->data;

		/* Get the address of firmware data */
		fw_info->fw_raw_data = fw_img->data + fw_info->cfg_len;
	}

	return TOUCH_SUCCESS;
}

static int mxt_get_object_table(struct mxt2954_ts_data *ts)
{
	int error = 0;
	int i = 0;
	u16 reg = 0;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_TABLE_ELEMENT_SIZE] = {0,};
	struct mxt2954_object *object =NULL;

	if (ts->driver_debug_enabled) {
		TOUCH_ERR("ts=[0x%X]\n", (int)ts);
		TOUCH_ERR("ts->object_table=[0x%X]\n", (int)ts->object_table);
	}

	for (i = 0; i < ts->info->object_num; i++) {
		object = ts->object_table + i;

		reg = MXT_OBJECT_TABLE_START_ADDRESS + (MXT_OBJECT_TABLE_ELEMENT_SIZE * i);
		error = Mxt2954_I2C_Read(ts->client, reg, 2, buf, MXT_OBJECT_TABLE_ELEMENT_SIZE);
		if (error) {
			TOUCH_ERR("mxt_read_mem error\n");
			return TOUCH_FAIL;
		}

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		/* the real size of object is buf[3]+1 */
		object->size_minus_one = buf[3];
		/* the real instances of object is buf[4]+1 */
		object->instances_minus_one = buf[4];
		object->num_report_ids = buf[5];

	if (ts->driver_debug_enabled)
		TOUCH_LOG("\t Object:T%02d  / S_Address:%03d / Size:%03d / Instance:%d / Report ID's:%d\n",
			object->type,
			object->start_address,
			object->size_minus_one,
			object->instances_minus_one,
			object->num_report_ids);

		if (object->num_report_ids) {
			reportid += object->num_report_ids * (object->instances_minus_one+1);
			ts->max_reportid = reportid;
		}
	}

	/* Store maximum reportid */
	ts->max_reportid = reportid;

	return TOUCH_SUCCESS;
}

static u8 mxt_get_bootloader_version(struct mxt2954_ts_data *ts, u8 val)
{
	u8 buf[3] = {0};

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(ts, &buf[0], 3) != 0) {
			TOUCH_ERR("i2c failure\n");
			return -EIO;
		}

		TOUCH_LOG("Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		TOUCH_LOG("Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt2954_ts_data *ts, unsigned int state)
{
	u8 val = 0;
	int ret = 0;

recheck:
	ret = mxt_bootloader_read(ts, &val, 1);
	if (ret)
		return TOUCH_FAIL;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(ts, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			TOUCH_ERR("Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		TOUCH_ERR("Invalid bootloader state %02X != %02X\n", val, state);
		return -EINVAL;
	}

	return TOUCH_SUCCESS;
}

static int mxt_lookup_bootloader_address(struct mxt2954_ts_data *ts, u8 retry)
{
	u8 appmode = ts->client->addr;
	u8 bootloader = 0;
	u8 family_id = 0;

	if (ts->info)
		family_id = ts->info->family_id;

	TOUCH_LOG("appmode=0x%x\n", appmode);

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if ((retry % 2) || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		TOUCH_ERR("Appmode i2c address 0x%02x not found\n", appmode);
		return -EINVAL;
	}

	ts->bootloader_addr = bootloader;
	TOUCH_LOG("bootloader_addr=0x%x\n",  bootloader);

	return TOUCH_SUCCESS;
}

static int mxt_enter_bootloader(struct mxt2954_ts_data *ts)
{
	int error = 0;

	if (ts->object_table)
		memset(ts->object_table, 0x0, (MXT_OBJECT_NUM_MAX * sizeof(struct mxt2954_object)));

	/* Get object table information*/
	error = mxt_get_object_table(ts);
	if (error)
		return TOUCH_FAIL;

	/* Change to the bootloader mode */
	error = mxt_command_reset(ts, MXT_BOOT_VALUE);
	if (error)
		return TOUCH_FAIL;

return TOUCH_SUCCESS;

}

static int mxt_probe_bootloader(struct mxt2954_ts_data *ts, u8 retry)
{
	int ret = 0;
	u8 val = 0;
	bool crc_failure = false;

	TOUCH_FUNC();

	ret = mxt_lookup_bootloader_address(ts, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(ts, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	TOUCH_LOG("Detected bootloader, status:%02X%s\n", val, crc_failure ? ", APP_CRC_FAIL" : "");

	return TOUCH_SUCCESS;
}

static int mxt_flash_fw(struct mxt2954_fw_info *fw_info)
{
	struct mxt2954_ts_data *ts = fw_info->ts;
	const u8 *fw_data = fw_info->fw_raw_data;
	size_t fw_size = fw_info->fw_len;
	unsigned int frame_size = 0;
	unsigned int frame = 0;
	unsigned int pos = 0;
	int ret = 0;

	if (!fw_data) {
		TOUCH_ERR("firmware data is Null\n");
		return -ENOMEM;
	}

	/* T641 use 0x26 bootloader addr */
	ret = mxt_lookup_bootloader_address(ts, 1);
	if (ret) {
		TOUCH_ERR("Failed to lookup bootloader address\n");
		return ret;
	}

	ret = mxt_check_bootloader(ts, MXT_WAITING_BOOTLOAD_CMD);
	if (ret) {
		/*may still be unlocked from previous update attempt */
		ret = mxt_check_bootloader(ts, MXT_WAITING_FRAME_DATA);
		if (ret) {
			TOUCH_ERR("Failed to check bootloader\n");
			goto out;
		}
	} else {
		TOUCH_ERR("Unlocking bootloader\n");
		/* Unlock bootloader */
		//mxt_unlock_bootloader(client);
		mxt_send_bootloader_cmd(ts, true);
	}
	while (pos < fw_size) {
		ret = mxt_check_bootloader(ts, MXT_WAITING_FRAME_DATA);
		if (ret) {
			TOUCH_ERR("Fail updating firmware. wating_frame_data err\n");
			goto out;
		}

		frame_size = ((*(fw_data + pos) << 8) | *(fw_data + pos + 1));

		/*
		* We should add 2 at frame size as the the firmware data is not
		* included the CRC bytes.
		*/

		frame_size += 2;

		/* Write one frame to device */
		//mxt_fw_write(client, fw_data + pos, frame_size);
		mxt_bootloader_write(ts, fw_data + pos, frame_size);

		ret = mxt_check_bootloader(ts, MXT_FRAME_CRC_PASS);
		if (ret) {
			TOUCH_ERR("Fail updating firmware. frame_crc err\n");
			goto out;
		}

		pos += frame_size;
		frame++;

		if (frame % 50 == 0)
			TOUCH_LOG("\t Updated %5d / %5d bytes\n", pos, fw_size);

		msleep(20);
	}

	msleep(MXT_FW_RESET_TIME);

out:
	return ret;
}

static int mxt_flash_fw_on_probe(struct mxt2954_fw_info *fw_info)
{
	struct mxt2954_ts_data *ts = fw_info->ts;
	int error = 0;
	/* struct mxt_object *object = NULL; */
	/* int ret = 0; */

	error = mxt_read_id_info(ts);

	if (error) {
		/* need to check IC is in boot mode */
		/* T641 use 0x26 bootloader Addr */
		error = mxt_probe_bootloader(ts, 1);
		if (error) {
			TOUCH_LOG("Failed to verify bootloader's status\n");
			return TOUCH_FAIL;
		}

		TOUCH_LOG("Updating firmware from boot-mode\n");
		goto load_fw;
	}

	/* compare the version to verify necessity of firmware updating */
	TOUCH_LOG("Binary Version [IC:%u.%u.%02X] [FW:%u.%u.%02X]\n",
		ts->info->version >> 4,
		ts->info->version & 0xF,
		ts->info->build,
		fw_info->bin_ver >> 4,
		fw_info->bin_ver & 0xF,
		fw_info->build_ver);

	if (ts->info->version == fw_info->bin_ver && ts->info->build == fw_info->build_ver) {
		TOUCH_LOG("Binary Version is same\n");
		goto out;
	}

	error = mxt_enter_bootloader(ts);
	if (error) {
		TOUCH_ERR("Failed enter bootloader mode\n");
		return TOUCH_FAIL;
	}

load_fw:
	error = mxt_flash_fw(fw_info);
	if (error)
		TOUCH_LOG("Failed updating firmware\n");
	else
		TOUCH_LOG("succeeded updating firmware\n");

out :
	return TOUCH_SUCCESS;
}

static int mxt_make_reportid_table(struct mxt2954_ts_data *ts)
{
	struct mxt2954_object *objects = ts->object_table;
	struct mxt2954_reportid *reportids = ts->reportids;
	int i = 0;
	int j = 0;
	int id = 0;

	for (i = 0; i < ts->info->object_num; i++) {
		for (j = 0; j < objects[i].num_report_ids * (objects[i].instances_minus_one+1); j++) {

			id++;
			reportids[id].type = objects[i].type;
			reportids[id].index = j;
		}
	}

	return TOUCH_SUCCESS;
}

static int mxt_read_info_crc(struct mxt2954_ts_data *ts, u32 *crc_pointer)
{
	u16 crc_address = 0;
	u8 msg[3] = {0};
	int ret = 0;

	/* Read Info block CRC address */
	crc_address = MXT_OBJECT_TABLE_START_ADDRESS +
			ts->info->object_num * MXT_OBJECT_TABLE_ELEMENT_SIZE;
	ret = Mxt2954_I2C_Read(ts->client, crc_address, 2, msg, 3);
	if (ret) {
		TOUCH_ERR("mxt_read_mem error\n");
		return TOUCH_FAIL;
	}

	*crc_pointer = msg[0] | (msg[1] << 8) | (msg[2] << 16);

	return TOUCH_SUCCESS;
}

static int mxt_table_initialize(struct mxt2954_ts_data *ts)
{
	int ret = 0;
	u32 read_info_crc = 0;

	ret = mxt_read_id_info(ts);
	if (ret) {
		TOUCH_ERR("Failed to [mxt_read_id_info]\n");
		return TOUCH_FAIL;
	}

	if (ts->object_table)
		memset(ts->object_table, 0x0, (MXT_OBJECT_NUM_MAX * sizeof(struct mxt2954_object)));

	/* Get object table infomation */
	ret = mxt_get_object_table(ts);
	if (ret) {
		TOUCH_ERR("Failed to [mxt_get_object_table]\n");
		return TOUCH_FAIL;
	}

	if (ts->reportids)
		kfree(ts->reportids);

	ts->reportids = kzalloc(((ts->max_reportid + 1) * sizeof(struct mxt2954_reportid)), GFP_KERNEL);
	if (!ts->reportids) {
		TOUCH_ERR("Failed to allocate memory for mxt2954_reportid\n");
		return TOUCH_FAIL;
	}

	/* Make report id table */
	mxt_make_reportid_table(ts);

	/* Verify the info CRC */
	ret = mxt_read_info_crc(ts, &read_info_crc);
	if (ret) {
		TOUCH_ERR("Failed to [mxt_read_info_crc]\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mxt_read_message(struct mxt2954_ts_data *ts, struct mxt2954_message *message)
{
	struct mxt2954_object *object = NULL;

	object = mxt_get_object(ts, MXT_GEN_MESSAGE_T5);
	if (!object) {
		TOUCH_ERR("mxt_read_message-mxt_get_object error\n");
		return TOUCH_FAIL;
	}

	return Mxt2954_I2C_Read(ts->client, object->start_address, 2, message, sizeof(struct mxt2954_message));
}

static int mxt_read_message_reportid(struct mxt2954_ts_data *ts, struct mxt2954_message *message, u8 reportid)
{
	int try = 0;
	int error = 0;
	int fail_count = 0;

	fail_count = ts->max_reportid * 2;

	while (++try < fail_count) {
		error = mxt_read_message(ts, message);
		if (error) {
			TOUCH_ERR("mxt_read_message error\n");
			print_hex_dump(KERN_DEBUG, "[Touch] CRC : ", DUMP_PREFIX_NONE, 16, 1,
				   message, sizeof(struct mxt2954_message), false);
			return TOUCH_FAIL;
		}

		if (message->reportid == 0xff)
			continue;

		if (message->reportid == reportid)
			return TOUCH_SUCCESS;
	}

	return TOUCH_FAIL;
}

static int mxt_read_config_crc(struct mxt2954_ts_data *ts, u32 *crc)
{
	struct mxt2954_message message = {0};
	struct mxt2954_object *object = NULL;
	int error = 0;

	object = mxt_get_object(ts, MXT_GEN_COMMAND_T6);
	if (!object)
		return TOUCH_FAIL;

	/* Try to read the config checksum of the existing cfg */
	mxt_write_object(ts, MXT_GEN_COMMAND_T6, MXT_COMMAND_REPORTALL, 1);

	/* Read message from command processor, which only has one report ID */
	error = mxt_read_message_reportid(ts, &message, 1); /* data->max_reportid); */
	if (error) {
		TOUCH_ERR("Failed to retrieve CRC\n");
		return TOUCH_FAIL;
	}

	/* Bytes 1-3 are the checksum. */
	*crc = message.message[1] | (message.message[2] << 8) | (message.message[3] << 16);

	return TOUCH_SUCCESS;
}

static int mxt_write_config(struct mxt2954_fw_info *fw_info)
{
	struct mxt2954_ts_data *ts = fw_info->ts;
	struct mxt2954_object *object = NULL;
	struct mxt2954_cfg_data *cfg_data = NULL;
	u32 current_crc = 0;
	u32 t71_cfg_crc = 0;
	u8 buf_crc_t71[5] = { 0, };
	u8 i = 0;
	u8 val = 0;
	u16 reg = 0;
	u16 index = 0;
	int ret = 0;

	if (!fw_info->cfg_raw_data) {
		TOUCH_ERR("No cfg data in file\n");
		return ret;
	}

	/* Get config CRC from device */
	ret = mxt_read_config_crc(ts, &current_crc);
	if (ret) {
		TOUCH_ERR("fail to read config crc\n");
		return ret;
	}

	/* Check Version information */
	if (fw_info->bin_ver != ts->info->version) {
		TOUCH_ERR("Warning: version mismatch!\n");
		return TOUCH_SUCCESS;
	}
	if (fw_info->build_ver != ts->info->build) {
		TOUCH_ERR("Warning: build num mismatch!\n");
		return TOUCH_SUCCESS;
	}

	object = mxt_get_object(ts, MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71);

	ret = Mxt2954_I2C_Read(ts->client, object->start_address + 107, 2, buf_crc_t71, 5);
	if (ret)
		TOUCH_ERR("T71 CRC read fail\n");

	t71_cfg_crc = buf_crc_t71[3] << 16 | buf_crc_t71[2] << 8 | buf_crc_t71[1];
	TOUCH_LOG("T71 CRC[%06X] FW CRC[%06X]\n", t71_cfg_crc, fw_info->cfg_crc);

	/* Check config CRC */
	if (current_crc == fw_info->cfg_crc || t71_cfg_crc == fw_info->cfg_crc) {
		TOUCH_LOG("Same Config [IC FW : %06X] Skip Writing\n", current_crc);
		return TOUCH_SUCCESS;
	}

	/* Restore memory and stop event handing */
	ret = mxt_command_backup(ts, MXT_DISALEEVT_VALUE);
	if (ret) {
		TOUCH_ERR("Failed Restore NV and stop event\n");
		return TOUCH_FAIL;
	}

	TOUCH_LOG("Writing Config:[Binary FW(crc):%06X] [IC FW(crc):%06X]\n", fw_info->cfg_crc, current_crc);

	/* Saved diff value */
	if (buf_crc_t71[0] == 1) {
		TOUCH_LOG("buf_crc_t71[0] == 1\n");
		ret = Mxt2954_I2C_Read(ts->client, object->start_address + 60, 2, &ts->t71_diff_val, 14 + ts->t15_num_keys);
		if (ret) {
			TOUCH_ERR("Reference Diff value Read fail\n");
			return TOUCH_FAIL;
		}
	}

	/* Write config info */
	for (index = 0; index < fw_info->cfg_len;) {
		if (index + sizeof(struct mxt2954_cfg_data) >= fw_info->cfg_len) {
			TOUCH_ERR("index(%d) of cfg_data exceeded total size(%d)!!\n",
				index + sizeof(struct mxt2954_cfg_data), fw_info->cfg_len);
			return TOUCH_FAIL;
		}

		/* Get the info about each object */
		cfg_data = (struct mxt2954_cfg_data *)(&fw_info->cfg_raw_data[index]);

		index += sizeof(struct mxt2954_cfg_data) + cfg_data->size;
		if (index > fw_info->cfg_len) {
			TOUCH_ERR("index(%d) of cfg_data exceeded total size(%d) in T%d object!!\n",
				index, fw_info->cfg_len, cfg_data->type);
			return TOUCH_FAIL;
		}

		object = mxt_get_object(ts, cfg_data->type);
		if (!object) {
			TOUCH_ERR("T%d is Invalid object type\n", cfg_data->type);
			return TOUCH_FAIL;
		}

		/* Check and compare the size, instance of each object */
		if (cfg_data->size > (object->size_minus_one+1)) {
			TOUCH_ERR("T%d Object length exceeded!\n", cfg_data->type);
			return TOUCH_FAIL;
		}
		if (cfg_data->instance >= (object->instances_minus_one+1)) {
			TOUCH_ERR("T%d Object instances exceeded!\n", cfg_data->type);
			return TOUCH_FAIL;
		}

		TOUCH_LOG("\t Writing config for T%02d len %3d instance %d (%3d/%3d)\n",
			cfg_data->type, object->size_minus_one, cfg_data->instance, index, fw_info->cfg_len);

		reg = object->start_address + (object->size_minus_one+1) * cfg_data->instance;

		/* Write register values of each object */
		ret = Mxt2954_I2C_Write(ts->client, reg, cfg_data->register_val, cfg_data->size);
		if (ret) {
			TOUCH_ERR("Write T%d Object failed\n", object->type);
			return TOUCH_FAIL;
		}

		/*
		 * If firmware is upgraded, new bytes may be added to end of
		 * objects. It is generally forward compatible to zero these
		 * bytes - previous behaviour will be retained. However
		 * this does invalidate the CRC and will force a config
		 * download every time until the configuration is updated.
		 */
		if (cfg_data->size < (object->size_minus_one+1)) {
			TOUCH_ERR("Warning: zeroing %d byte(s) in T%d\n",
				 (object->size_minus_one+1) - cfg_data->size, cfg_data->type);

			for (i = cfg_data->size + 1; i < (object->size_minus_one+1); i++) {
				ret = Mxt2954_I2C_Write(ts->client, reg + i, &val, 1);
				if (ret)
					return TOUCH_FAIL;
			}
		}
	}

	TOUCH_LOG("Configuration Updated\n");

	/* Restore diff value */
	if (buf_crc_t71[0] == 1) {
		TOUCH_LOG("Restore diff value\n");
		object = mxt_get_object(ts, MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71);

		ret = Mxt2954_I2C_Write(ts->client, object->start_address + 60, (u8 *)&ts->t71_diff_val, 14 + ts->t15_num_keys);
		if (ret) {
			TOUCH_ERR("Reference Diff Restore fail\n");
			return TOUCH_FAIL;
		}

		buf_crc_t71[0] = 1;
		buf_crc_t71[1] = (u8)(fw_info->cfg_crc & 0x000000FF);
		buf_crc_t71[2] = (u8)((fw_info->cfg_crc & 0x0000FF00) >> 8);
		buf_crc_t71[3] = (u8)((fw_info->cfg_crc & 0x00FF0000) >> 16);
		buf_crc_t71[4] = (u8)((fw_info->cfg_crc & 0xFF000000) >> 24);

		ret = Mxt2954_I2C_Write(ts->client, object->start_address + 107, buf_crc_t71, 5);
		if (ret) {
			TOUCH_ERR("Reference Diff Restore fail\n");
			return TOUCH_FAIL;
		}

	}

	/* Backup to memory */
	ret = mxt_command_backup(ts, MXT_BACKUP_VALUE);
	if (ret) {
		TOUCH_ERR("Failed backup NV data\n");
		return TOUCH_FAIL;
	}

	/* Soft reset */
	ret = mxt_command_reset(ts, MXT_RESET_VALUE);
	if (ret) {
		TOUCH_ERR("Failed Reset IC\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mxt_init_t7_power_cfg(struct mxt2954_ts_data *ts)
{
	int error = 0;
	bool retry = false;

recheck:
	error = Mxt2954_I2C_Read(ts->client, ts->T7_address, 2, &(ts->t7_cfg), sizeof(ts->t7_cfg));
	if (error)
		return TOUCH_FAIL;

	if (ts->t7_cfg.active == 0 || ts->t7_cfg.idle == 0) {
		if (!retry) {
			TOUCH_ERR("T7 cfg zero, resetting\n");
			mxt_soft_reset(ts);
			retry = true;
			goto recheck;
		} else {
		    TOUCH_ERR("T7 cfg zero after reset, overriding\n");
		    ts->t7_cfg.active = 8;
		    ts->t7_cfg.idle = 24;
		    return mxt_set_t7_power_cfg(ts, MXT_POWER_CFG_RUN);
		}
	} else {
		TOUCH_LOG("Initialised power cfg: ACTV %d, IDLE %d\n", ts->t7_cfg.active, ts->t7_cfg.idle);
		return TOUCH_SUCCESS;
	}
}

static int mxt_init_t93_tab_count(struct mxt2954_ts_data *ts)
{
	struct mxt2954_object *object = NULL;
	int error = 0;

	object = mxt_get_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (object) {
		error = Mxt2954_I2C_Read(ts->client, object->start_address + 17, 2, &(ts->g_tap_cnt), 1);
		TOUCH_LOG("ts->g_tap_cnt : %d\n", ts->g_tap_cnt);
		return TOUCH_SUCCESS;
	} else {
		ts->g_tap_cnt = 0;
		return TOUCH_FAIL;
	}
}

static int  mxt_config_initialize(struct mxt2954_fw_info *fw_info)
{
	struct mxt2954_ts_data *ts = fw_info->ts;

	int ret = 0;

	ret = mxt_write_config(fw_info);
	if (ret) {
		TOUCH_ERR("Failed to write config from file\n");
		goto out;
	}

	if (ts->patch.patch) {
		ret = mxt_patch_init(ts, ts->patch.patch);
		if (ret) {
			TOUCH_ERR("Failed to mxt_patch_init\n");
		}
	}
	if (ret == 0) {
		global_ts = ts;
	} else {
		global_ts = NULL;
		TOUCH_ERR("Failed to get global_ts (NULL)\n");
	}

out:
	return TOUCH_SUCCESS;
}

static void mxt_free_object_table(struct mxt2954_ts_data *ts)
{
	TOUCH_LOG("%s\n", __func__);

	if (ts->raw_info_block)
		kfree(ts->raw_info_block);

	ts->info = NULL;
	ts->raw_info_block = NULL;

	ts->T5_address = 0;
	ts->T5_msg_size = 0;
	ts->T6_reportid = 0;
	ts->T7_address = 0;
	ts->T9_address = 0;
	ts->T9_reportid_min = 0;
	ts->T9_reportid_max = 0;
	ts->T15_reportid_min = 0;
	ts->T15_reportid_max = 0;
	ts->T18_address = 0;
	ts->T24_reportid = 0;
	ts->T35_reportid = 0;
	ts->T25_reportid = 0;
	ts->T42_reportid_min = 0;
	ts->T42_reportid_max = 0;
	ts->T42_address = 0;
	ts->T44_address = 0;
	ts->T46_address = 0;
	ts->T48_reportid = 0;
	ts->T56_address = 0;
	ts->T61_address = 0;
	ts->T61_reportid_min = 0;
	ts->T61_reportid_max = 0;
	ts->T65_address = 0;
	ts->T72_address = 0;
	ts->T93_address = 0;
	ts->T93_reportid = 0;
	ts->max_reportid = 0;
	ts->T100_address = 0;
	ts->T100_reportid_min = 0;
	ts->T100_reportid_max = 0;
}

static int mxt_parse_object_table(struct mxt2954_ts_data *ts)
{
	int i = 0;
	u8 reportid = 0;
	u8 min_id = 0;
	u8 max_id = 0;
	u16 end_address = 0;
	struct mxt2954_object *object = NULL;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	ts->mem_size = 0;

	TOUCH_LOG("mxt_parse_object_table\n");

	for (i = 0; i < ts->info->object_num; i++) {
		object = ts->object_table + i;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids * mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		TOUCH_LOG("\t T%02u Start:%u Size:%03u Instances:%u Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (ts->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				ts->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				ts->T5_msg_size = mxt_obj_size(object) - 1;
			}
			ts->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			ts->T6_reportid = min_id;
			ts->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			ts->T7_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			ts->T9_address = object->start_address;
			ts->T9_reportid_min = min_id;
			ts->T9_reportid_max = min_id + object->num_report_ids - 1;
			ts->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			ts->T15_reportid_min = min_id;
			ts->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			ts->T18_address = object->start_address;
			break;
		case MXT_PROCI_ONETOUCH_T24:
			ts->T24_reportid = min_id;
			break;
		case MXT_SPT_PROTOTYPE_T35:
			ts->T35_reportid = min_id;
			break;
		case MXT_SPT_SELFTEST_T25:
			ts->T25_reportid = min_id;
			ts->T25_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			ts->T42_address = object->start_address;
			ts->T42_reportid_min = min_id;
			ts->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			ts->T44_address = object->start_address;
			break;
		case MXT_SPT_CTECONFIG_T46:
			ts->T46_address = object->start_address;
			break;
		case MXT_SPT_NOISESUPPRESSION_T48:
			ts->T48_reportid = min_id;
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			ts->T56_address = object->start_address;
			break;
		case MXT_SPT_TIMER_T61:
			ts->T61_address = object->start_address;
			ts->T61_reportid_min = min_id;
			ts->T61_reportid_max = max_id;
			break;
		case MXT_PROCI_LENSBENDING_T65:
			ts->T65_address = object->start_address;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			ts->T71_address = object->start_address;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T72:
			ts->T72_address = object->start_address;
			break;
		case MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93:
			ts->T93_reportid = min_id;
			ts->T93_address = object->start_address;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			/* Only handle messages from first T100 instance */
			ts->T100_address = object->start_address;
			ts->T100_reportid_min = min_id;
			ts->T100_reportid_max = min_id + object->num_report_ids - 1;
			ts->num_touchids = object->num_report_ids - 2;
			TOUCH_LOG("T100_reportid_min:%d T100_reportid_max:%d\n", ts->T100_reportid_min, ts->T100_reportid_max);
			break;
		}

		end_address = object->start_address + mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (ts->driver_debug_enabled)
			TOUCH_ERR("\t Object:T%02u / S_Address:%u / E_Address:%u / mem_size:%u\n",
				object->type,
				object->start_address,
				end_address,
				ts->mem_size);

		if (end_address >= ts->mem_size)
			ts->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	ts->max_reportid = reportid;

	if (ts->msg_buf)
		kfree(ts->msg_buf);

	ts->msg_buf = kzalloc((ts->max_reportid * ts->T5_msg_size), GFP_KERNEL);
	if (!ts->msg_buf) {
		TOUCH_ERR("Failed to allocate memory for message buffer\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static int mxt_read_info_block(struct mxt2954_ts_data *ts)
{
	int error = 0;
	size_t size = 0;
	void *buf = NULL;
	struct mxt2954_info *info = NULL;

	TOUCH_FUNC();

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt2954_info);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		TOUCH_ERR("Failed to allocate memory for mxt2954_info [1]\n");
		return -ENOMEM;
	}

	error = Mxt2954_I2C_Read(ts->client, 0, 2, buf, size);
	if (error) {
		TOUCH_ERR("Mxt2954_I2C_Read error\n");
		goto err_free_mem;
	}

	/* Resize buffer to give space for rest of info block */
	info = (struct mxt2954_info *)buf;
	size += (MXT_OBJECT_NUM_MAX * sizeof(struct mxt2954_object)) + MXT_INFO_CHECKSUM_SIZE;
	buf = krealloc(buf, size, GFP_KERNEL);
	if (!buf) {
		TOUCH_ERR("Failed to allocate memory for mxt2954_info [2]\n");
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = Mxt2954_I2C_Read(ts->client, MXT_OBJECT_START, 2, buf + MXT_OBJECT_START, size - MXT_OBJECT_START);
	if (error) {
		TOUCH_ERR("Mxt2954_I2C_Read error\n");
		goto err_free_mem;
	}

	/* Save pointers in device data structure */
	ts->raw_info_block = buf;
	ts->info = (struct mxt2954_info *)buf;

	if (ts->object_table == NULL)
		ts->object_table = (struct mxt2954_object *)(buf + MXT_OBJECT_START);

	TOUCH_LOG("Family:%02X Variant:%02X Binary:%u.%u.%02X TX:%d RX:%d Objects:%d\n",
		 ts->info->family_id,
		 ts->info->variant_id,
		 ts->info->version >> 4,
		 ts->info->version & 0xF,
		 ts->info->build,
		 ts->info->matrix_xsize,
		 ts->info->matrix_ysize,
		 ts->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(ts);
	if (error) {
		TOUCH_ERR("Error %d reading object table\n", error);
		mxt_free_object_table(ts);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

err_free_mem:
	kfree(buf);
	ts->raw_info_block = NULL;
	ts->info = NULL;
	ts->object_table = NULL;
	return TOUCH_FAIL;

}

static void mxt_regulator_enable(struct i2c_client *client)
{
	int error = 0;
	TOUCH_FUNC();

	gpio_set_value(TOUCH_GPIO_RESET, 0);
	/* vdd_ana enable */
	gpio_set_value(TOUCH_LDO_AVDD, 1);

	if (ts->vcc_i2c) {
		error = regulator_enable(ts->vcc_i2c);
		if (error < 0) {
			TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
			return ;
		}
	}
	/* vcc_dig enable */
	error = regulator_enable(ts->vcc_dig);
	if (error < 0) {
		TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
		return ;
	}
	msleep(1);
	gpio_set_value(TOUCH_GPIO_RESET, 1);
	msleep(200);
	TOUCH_LOG("Power On Touch IC\n");

}

static void mxt_regulator_disable(struct i2c_client *client)
{
	int error = 0;
	TOUCH_FUNC();

	gpio_set_value(TOUCH_GPIO_RESET, 0);
	/* vdd_ana disable */
	gpio_set_value(TOUCH_LDO_AVDD, 0);

	if (ts->vcc_i2c) {
		error = regulator_disable(ts->vcc_i2c);
		if (error < 0) {
			TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
			return ;
		}
	}
	/* vcc_dig disable */
	error = regulator_disable(ts->vcc_dig);
	if (error < 0) {
		TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
		return ;
	}
	TOUCH_LOG("Power Off Touch IC\n");
}

static void mxt_active_mode_start(struct mxt2954_ts_data *ts)
{
	struct mxt2954_object *object;
	int error = 0;

	object = mxt_get_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (!object)
		return;

	if (ts->mxt_knock_on_enable || ts->currState== STATE_KNOCK_ON_ONLY){
		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			TOUCH_LOG("Resume : Double tap Enabled(TA)\n");
			mxt_patch_event(ts, CHARGER_KNOCKON_WAKEUP);
		} else {
			TOUCH_LOG("Resume : Double tap Enabled\n");
			mxt_patch_event(ts, NOCHARGER_KNOCKON_WAKEUP);
		}
	} else if (ts->mxt_multi_tap_enable || ts->currState== STATE_KNOCK_ON_CODE) {
		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			TOUCH_LOG("Resume : Multi tap Enabled(TA)\n");
			mxt_patch_event(ts, CHARGER_KNOCKON_WAKEUP + PATCH_EVENT_PAIR_NUM);
		} else {
			TOUCH_LOG("Resume : Multi tap Enabled\n");
			mxt_patch_event(ts, NOCHARGER_KNOCKON_WAKEUP + PATCH_EVENT_PAIR_NUM);
		}
		/* Write TCHCNTTHR(Touch Count Threshold) for LPWG_MULTI_TAP */
		error = Mxt2954_I2C_Write(ts->client, object->start_address+17, &(ts->g_tap_cnt), 1);
		if (error)
			TOUCH_LOG("Object Write Fail\n");

	} else {
		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			mxt_patch_event(ts, CHARGER_PLUGGED);
		} else {
			mxt_patch_event(ts, CHARGER_UNplugged);
		}
	}
	mxt_patchevent_unset(PATCH_EVENT_KNOCKON);

}

static void mxt_gesture_mode_start(struct mxt2954_ts_data *ts, u32 value)
{
	struct mxt2954_object *object;
	int error = 0;
	u16 reg = 0;
	u8 val = 0;

	object = mxt_get_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (!object)
		return;

	mxt_patchevent_set(PATCH_EVENT_KNOCKON);

	if (value == STATE_KNOCK_ON_ONLY) {
		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			TOUCH_LOG("Suspend : Double tap Enabled(TA)\n");
			mxt_patch_event(ts, CHARGER_KNOCKON_SLEEP);
		} else {
			TOUCH_LOG("Suspend : Double tap Enabled\n");
			mxt_patch_event(ts, NOCHARGER_KNOCKON_SLEEP);
		}
	} else if (value == STATE_KNOCK_ON_CODE) {
		if (mxt_patchevent_get(PATCH_EVENT_TA)) {
			TOUCH_LOG("Suspend : Multi tap Enabled(TA)\n");
			mxt_patch_event(ts, CHARGER_KNOCKON_SLEEP + PATCH_EVENT_PAIR_NUM);
		} else {
			TOUCH_LOG("Suspend : Multi tap Enabled\n");
			mxt_patch_event(ts, NOCHARGER_KNOCKON_SLEEP + PATCH_EVENT_PAIR_NUM);
		}
		/* Write TCHCNTTHR(Touch Count Threshold) for LPWG_MULTI_TAP */
		error = Mxt2954_I2C_Write(ts->client, object->start_address+17, &(ts->g_tap_cnt), 1);
		if (error)
			TOUCH_LOG("Object Write Fail\n");
	}

	/* Slimport check */
	if (slimport_is_check()) {
		reg = ts->T72_address + 0 ;
		val = 11;
		TOUCH_LOG("slimport is connected. T72 enable\n");
		error = Mxt2954_I2C_Write(ts->client, reg, &val, 1);
		if (error)
			TOUCH_LOG("T72 object write fail\n");
	}

	/* Interrupt control for LPWG Fail Reason */
	if (ts->use_debug_reason)
		mxt_lpwg_debug_interrupt_control(ts, ts->use_debug_reason, value);

}

static void mxt_lpwg_enable(struct mxt2954_ts_data *ts, u32 value)
{
	struct mxt2954_object *object;
	int error = 0;
	int knockOn_delay = 0; /* 16ms/unit */

	object = mxt_get_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93);
	if (!object)
		return;

	if (value == STATE_KNOCK_ON_ONLY) {
		ts->mxt_knock_on_enable= true;
		ts->mxt_multi_tap_enable= false;
		TOUCH_LOG("Knock On Enable\n");
	} else if (value == STATE_KNOCK_ON_CODE) {
		ts->mxt_knock_on_enable= false;
		ts->mxt_multi_tap_enable= true;
		TOUCH_LOG("Multi Tap Enable\n");
	} else {
		TOUCH_LOG("Unknown Value. Not Setting\n");
		return;
	}

	TOUCH_LOG("Double Tap check. First TwoTap Same [%d]\n", ts->lpwgSetting.isFirstTwoTapSame);
	if (ts->lpwgSetting.isFirstTwoTapSame == 1) {
		ts->is_knockCodeDelay = true;
		knockOn_delay = 43;
	} else {
		ts->is_knockCodeDelay = false;
		knockOn_delay = 0;
	}

	error = mxt_write_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 19, knockOn_delay);
	TOUCH_LOG("Set Knock ON delay (%d)\n", knockOn_delay);
	if (error) {
		TOUCH_ERR("T93 waited(%d) knock On write fail\n", knockOn_delay);
	}

	if (value == STATE_KNOCK_ON_ONLY) {
		error = mxt_write_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 22, 85);
		TOUCH_LOG("Set Knock ON range (10mm)\n");
	} else if (value == STATE_KNOCK_ON_CODE) {
		error = mxt_write_object(ts, MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93, 22, 63);
		TOUCH_LOG("Set Knock ON range (7mm)\n");
	}
}

static int mXT2954_lpwg_control(struct i2c_client *client, TouchState newState)
{

	TOUCH_FUNC();
	TOUCH_LOG("newState %d\n", newState);

	/* backup interrupt enable register */

	/* disable interrupt */
	//TouchDisableIrq();
	ts->g_tap_cnt = ts->lpwgSetting.tapCount;

	switch (newState) {
	case STATE_NORMAL:
		TOUCH_LOG("Normal State - TapCount[%d]\n", ts->g_tap_cnt);
		ts->mxt_knock_on_enable = false;
		ts->mxt_multi_tap_enable = false;
		/* Case : Disable Knock-On Mode setting in HiddenMenu */
		if (!ts->lpwgSetting.mode || ts->currState == STATE_OFF)
			mxt_regulator_enable(client);
		mxt_active_mode_start(ts);
		break;

	case STATE_KNOCK_ON_ONLY:
		TOUCH_LOG("Only Knock On - TapCount[%d]\n", ts->g_tap_cnt);
		if (ts->currState == STATE_OFF)
			mxt_regulator_enable(client);
		mxt_lpwg_enable(ts, newState);
		mxt_gesture_mode_start(ts, newState);
		mxt_reset_slots(ts);
		break;

	case STATE_KNOCK_ON_CODE:
		TOUCH_LOG("Knock On/Code - TapCount[%d]\n", ts->g_tap_cnt);
		if (ts->currState == STATE_OFF)
			mxt_regulator_enable(client);
		mxt_lpwg_enable(ts, newState);
		mxt_gesture_mode_start(ts, newState);
		mxt_reset_slots(ts);
		break;

	case STATE_OFF:
		TOUCH_LOG("State Off - TapCount[%d]\n", ts->g_tap_cnt);
		ts->mxt_knock_on_enable = false;
		ts->mxt_multi_tap_enable = false;
		TOUCH_LOG("KnockOn/Multitap Gesture Disable\n");
		/* Case : Disable Knock-On Mode setting in HiddenMenu */
		mxt_regulator_disable(client);
		mxt_reset_slots(ts);
		break;

	default:
		TOUCH_ERR("invalid touch state ( %d )\n", newState);
		break;
	}

	/* restore interrupt enable register */
	//TouchEnableIrq();

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : Dummy_ReadIcFirmwareInfo
// Description
//   - Read firmware information from touch IC
//   - will be called at boot time or after writing new firmware
//====================================================================
static int mXT2954_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	TOUCH_FUNC();

	/* below value is same as default firmware image ( dummy_firmware.img ) */
	pFwInfo->moduleMakerID = 2;
	pFwInfo->moduleVersion = 0;
	pFwInfo->modelID = 3;
	pFwInfo->isOfficial = 1;
	pFwInfo->version = 4;

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : Dummy_GetBinFirmwareInfo
// Description
//   - parse and return firmware information from firmware image
//   - if filename is NULL, return information of default firmware image
//   - will be called at boot time or needed
//====================================================================
static int mXT2954_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{

	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);

	/* parse and get firmware information function */
	pFwInfo->moduleMakerID = ( *(pBin) >> 4 ) & 0xF;
	pFwInfo->moduleVersion = *(pBin) & 0xF;
	pFwInfo->modelID = *(pBin+1);
	pFwInfo->isOfficial = ( *(pBin+3) >> 7 ) & 0x1;
	pFwInfo->version = *(pBin+3) & 0x7F;

	/* Free firmware image buffer */
	release_firmware(fw);

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : Dummy_UpdateFirmware
// Description
//   - Write firmware to touch IC
//   - if filename is NULL, use default firmware image
//   - common driver will call Reset(), InitRegister() and ReadIcFirmwareInfo() one by one after writing
//====================================================================
static int mXT2954_UpdateFirmware(struct i2c_client *client, char *pFilename)
{

	int ret = 0;
	int error = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;

	TOUCH_FUNC();

	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret ) {
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);

	ts->fw_info.ts = ts;

	/* IMPLEMENT : firmware update function */
	error = mxt_verify_fw(&ts->fw_info, fw);
	if (error) {
		TOUCH_ERR("Failed to mxt_verify_fw\n");
		return TOUCH_FAIL;
	}

	error = mxt_flash_fw_on_probe(&ts->fw_info);
	if (error) {
		TOUCH_ERR("Failed to mxt_table_initialize\n");
		return TOUCH_FAIL;
	}

	error = mxt_table_initialize(ts);
	if (error) {
		TOUCH_ERR("Failed to mxt_table_initialize\n");
		return TOUCH_FAIL;
	}

	error = mxt_init_t93_tab_count(ts);
	if (error) {
		TOUCH_ERR("Failed to get T93 tab count\n");
	}

	/* mxt_acquire_irq(data); */
	error = mxt_config_initialize(&ts->fw_info);
	if (error) {
		TOUCH_ERR("Failed to rest initialize\n");
		return TOUCH_FAIL;
	}

	/* Free firmware image buffer */
	release_firmware(fw);

	error = mxt_read_info_block(ts);
	if (error) {
		TOUCH_ERR("error mxt_read_info_block\n");
		return TOUCH_FAIL;
	}

	error = mxt_init_t7_power_cfg(ts);
	if (error) {
		TOUCH_ERR("error mxt_init_t7_power_cfg\n");
		return TOUCH_FAIL;
	}

	mxt_read_fw_version(ts);

	if (ts->T100_reportid_min) {
		error = mxt_initialize_t100_input_device(ts);
		if (error){
			TOUCH_ERR("Failed to init t100\n");
			return TOUCH_FAIL;
		}
	} else {
		TOUCH_ERR("Failed to read touch object\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}

//====================================================================
// Function : Dummy_SetLpwgMode
// Description
//   - Set device to requested state
//====================================================================
static int mXT2954_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;

	TOUCH_FUNC();

#if 0 /* 2015.06.16 For Test */
	if( ts->currState == newState ) {
		TOUCH_LOG("device state is same as driver requested\n");
		return TOUCH_SUCCESS;
	}
#endif

	if ((newState < STATE_NORMAL) && (newState > STATE_KNOCK_ON_CODE)) {
		TOUCH_LOG("invalid request state ( state = %d )\n", newState);
		return TOUCH_FAIL;
	}

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

	ret = mXT2954_lpwg_control(client, newState);
	if (ret == TOUCH_FAIL) {
		TOUCH_ERR("failed to set lpwg mode in device\n");
		return TOUCH_FAIL;
	}

	if (ret == TOUCH_SUCCESS)
		ts->currState = newState;

	switch( newState )
	{
		case STATE_NORMAL:
			TOUCH_LOG("device was set to NORMAL\n");
			break;
		case STATE_OFF:
			TOUCH_LOG("device was set to OFF\n");
			break;
		case STATE_KNOCK_ON_ONLY:
			TOUCH_LOG("device was set to KNOCK_ON_ONLY\n");
			break;
		case STATE_KNOCK_ON_CODE:
			TOUCH_LOG("device was set to KNOCK_ON_CODE\n");
			break;
		case STATE_NORMAL_HOVER:
			TOUCH_LOG("device was set to NORMAL_HOVER\n");
			break;
		case STATE_HOVER:
			TOUCH_LOG("device was set to HOVER\n");
			break;
		default:
			TOUCH_LOG("invalid state ( state = %d )\n", newState);
			ret = TOUCH_FAIL;
			break;

	}


	return TOUCH_SUCCESS;

}

//====================================================================
// Function : Dummy_DoSelfDiagnosis
// Description
//   - diagnose touch pannel and return result
//   - can use pBuf to give more information ( be careful not to exceed buffer size )
//   - should create a file of result ( TBD : consider file saving on common driver side )
//====================================================================
static int mXT2954_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int dataLen = 0;
	int i = 0;
	int len = 0;
	int ref_len = 0;
	char *ref_buf = NULL;
	bool chstatus_result = 1;
	bool rawdata_result = 1;

	int write_page = 1 << 14;

	TOUCH_FUNC();
	mxt_power_block(POWERLOCK_SYSFS);

	ts->self_test_result_status = SELF_DIAGNOSTIC_STATUS_RUNNING;

	if (pDriverData->lpwgSetting.lcdState == POWER_OFF) {
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "************************\n");
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "*** LCD STATUS : OFF ***\n");
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "************************\n");
		mxt_power_unblock(POWERLOCK_SYSFS);
		ts->self_test_result_status = SELF_DIAGNOSTIC_STATUS_COMPLETE;
		*pDataLen = dataLen;
		return TOUCH_SUCCESS;
	}

	TOUCH_LOG("Force calibrate for Self-Diagnostic Test\n");
	mxt_t6_command(ts, MXT_COMMAND_CALIBRATE, 1, false);
	msleep(30);

	ref_buf = kzalloc(write_page, GFP_KERNEL);
	if (!ref_buf) {
		TOUCH_ERR("Failed to allocate memory\n");
		mxt_power_unblock(POWERLOCK_SYSFS);
		ts->self_test_result_status = SELF_DIAGNOSTIC_STATUS_COMPLETE;
		return TOUCH_FAIL;
	}

	/* allocation of full_cap */
	ts->full_cap = NULL;

	ts->full_cap = (int **)kzalloc(ts->channel_size.size_x * sizeof(int *), GFP_KERNEL);
	if(!ts->full_cap)
	{
		TOUCH_ERR("full_cap data allocation error\n");
		return -1;
	}
	memset(ts->full_cap, 0, ts->channel_size.size_x * sizeof(int *));

	for(i = 0; i < ts->channel_size.size_x; i++) {
		ts->full_cap[i] = kzalloc(ts->channel_size.size_y * sizeof(int), GFP_KERNEL);
	}

	write_file(SELF_DIAGNOSTIC_FILE_PATH, pBuf, 1);
	msleep(30);
	len += show_mxt_info(client, pBuf);
	len = mxt_selftest(ts, pBuf, len);
	write_file(SELF_DIAGNOSTIC_FILE_PATH, pBuf, 0);
	msleep(30);
	run_reference_read(ts, ref_buf, &ref_len);
	write_file(SELF_DIAGNOSTIC_FILE_PATH, ref_buf, 0);
	msleep(30);
	dataLen = len;

//	Do not use this function to avoid Watch dog.
//	mxt_get_cap_diff(data);

	kfree(ref_buf);

	if (ts->full_cap != NULL) {
		for(i = 0; i < ts->channel_size.size_x; i++) {
			if(ts->full_cap[i] != NULL)
				kfree(ts->full_cap[i]);
		}
		kfree((int*)ts->full_cap);
		ts->full_cap = NULL;
	}

	if ((ts->self_test_status[0] == 0x01) || (ts->self_test_status[0] == 0x12))
		chstatus_result = 0;

	if (ts->self_test_status[0] == 0x17)
		rawdata_result = 0;

	dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "=================================\n");
	if (ts->self_test_status[0] == 0) {
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "Need more time. Try Again\n");
	} else if (ts->self_test_status[0] == 0xFD) {
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "Invalid Test Code. Try Again.\n");
	} else if (ts->self_test_status[0] == 0xFC) {
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "The test could not be completed. Try Again.\n");
	} else {
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "Channel Status : %s\n", chstatus_result == 1 ? "PASS" : "FAIL");
		dataLen += snprintf(pBuf + dataLen, PAGE_SIZE - dataLen, "Raw Data : %s\n", rawdata_result == 1 ? "PASS" : "FAIL");
	}
	*pRawStatus = (rawdata_result == 1 ? TOUCH_SUCCESS : TOUCH_FAIL);
	*pChannelStatus = (chstatus_result == 1 ? TOUCH_SUCCESS : TOUCH_FAIL);

	mxt_power_unblock(POWERLOCK_SYSFS);

	ts->self_test_result_status = SELF_DIAGNOSTIC_STATUS_COMPLETE;

	*pDataLen = dataLen;

	return TOUCH_SUCCESS;
}

//====================================================================
// Function : Dummy_AccessRegister
// Description
//   - read from or write to touch IC
//====================================================================
static int mXT2954_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	TouchDriverData *pDriverData = i2c_get_clientdata(client);
	int error = 0;

	TOUCH_FUNC();

	if (pDriverData->mfts_enable) {
		TOUCH_DBG("in MFTS\n");
		if (cmd == 0) {
			TOUCH_LOG("MXT Regulator Disable in MFTS\n");
			gpio_set_value(TOUCH_GPIO_RESET, 0);
			/* vdd_ana disable */
			gpio_set_value(TOUCH_LDO_AVDD, 0);

			if (ts->vcc_i2c) {
				error = regulator_disable(ts->vcc_i2c);
				if (error < 0) {
					TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
					return TOUCH_FAIL;
				}
			}
			/* vcc_dig disable */
			error = regulator_disable(ts->vcc_dig);
			if (error < 0) {
				TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
				return TOUCH_FAIL;
			}
			TOUCH_LOG("Complete - Power Off Touch IC in MFTS\n");
			return TOUCH_SUCCESS;
		} else if (cmd == 1) {
			TOUCH_LOG("MXT Regulator Enable in MFTS\n");
			gpio_set_value(TOUCH_GPIO_RESET, 0);
			/* vdd_ana enable */
			gpio_set_value(TOUCH_LDO_AVDD, 1);

			if (ts->vcc_i2c) {
				error = regulator_enable(ts->vcc_i2c);
				if (error < 0) {
					TOUCH_ERR("failed to disable regulator ( error = %d )\n", error);
					return TOUCH_FAIL;
				}
			}
			/* vcc_dig enable */
			error = regulator_enable(ts->vcc_dig);
			if (error < 0) {
				TOUCH_ERR("failed to enable regulator ( error = %d )\n", error);
				return TOUCH_FAIL;
			}
			msleep(1);
			gpio_set_value(TOUCH_GPIO_RESET, 1);
			msleep(200);
			TOUCH_LOG("Complete - Power On Touch IC in MFTS\n");

			mXT2954_ClearInterrupt(client);
			mxt_read_fw_version(ts);
		} else {
			if (cmd ==2) {
				TOUCH_LOG("Send patch-event after Self-Diagnostic in MFTS\n");
				mxt_patch_event(ts, PATCH_EVENT_AAT);
			}
		}
	}

	if (pDriverData->currState != STATE_SELF_DIAGNOSIS) {
		TOUCH_LOG("Send patch-event in MiniOs Bootmode[%d]\n", pDriverData->currState);
		mxt_patch_event(ts, PATCH_EVENT_AAT);
	} else {
		if (!pDriverData->mfts_enable) {
			TOUCH_LOG("Send patch-event after Self-Diagnostic in AAT[%d]\n", selfd_check_usb_type);
			mxt_patch_event(ts, PATCH_EVENT_AAT);
			trigger_usb_state_from_otg(selfd_check_usb_type);
		}
	}
 	return TOUCH_SUCCESS;

}

static void mXT2954_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	switch (notify) {
	case NOTIFY_CALL:
		TOUCH_LOG("Call was notified ( data = %d )\n", data);
		break;

	case NOTIFY_Q_COVER:
		TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
		break;

	default:
		TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
		break;
	}

	return;

}
TouchDeviceSpecificFunction mXT2954_Func = {

	.Initialize = mXT2954_Initialize,
	.Reset = mXT2954_Reset,
	.Connect = mXT2954_Connect,
	.InitRegister = mXT2954_InitRegister,
	.ClearInterrupt = mXT2954_ClearInterrupt,
	.InterruptHandler = mXT2954_InterruptHandler,
	.ReadIcFirmwareInfo = mXT2954_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = mXT2954_GetBinFirmwareInfo,
	.UpdateFirmware = mXT2954_UpdateFirmware,
	.SetLpwgMode = mXT2954_SetLpwgMode,
	.DoSelfDiagnosis = mXT2954_DoSelfDiagnosis,
	.AccessRegister = mXT2954_AccessRegister,
	.NotifyHandler = mXT2954_NotifyHandler,
	.device_attribute_list = mXT2954_attribute_list,

};


/* End Of File */


