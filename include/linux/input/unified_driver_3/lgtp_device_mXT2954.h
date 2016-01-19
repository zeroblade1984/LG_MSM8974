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
 *    File  	: lgtp_device_mXT2954.h
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/

#if !defined ( _LGTP_DEVICE_MXT2954_H_ )
#define _LGTP_DEVICE_MXT2954_H_

/****************************************************************************
* Nested Include Files
****************************************************************************/


/****************************************************************************
* Mainfest Constants / Defines
****************************************************************************/
#define MXT_FW_MAGIC		0x4D3C2B1A
#define MXT_PATCH_MAGIC		0x52296416

#define REF_OFFSET_VALUE		16384
#define REF_MIN_VALUE		(19744 - REF_OFFSET_VALUE)
#define REF_MAX_VALUE		(28884 - REF_OFFSET_VALUE)

#define NODE_PER_PAGE		64
#define DATA_PER_NODE		2

/* Diagnostic command defines  */
#define MXT_DIAG_PAGE_UP			0x01
#define MXT_DIAG_PAGE_DOWN		0x02
#define MXT_DIAG_DELTA_MODE		0x10
#define MXT_DIAG_REFERENCE_MODE	0x11
#define MXT_DIAG_CTE_MODE		0x31
#define MXT_DIAG_IDENTIFICATION_MODE	0x80
#define MXT_DIAG_TOCH_THRESHOLD_MODE	0xF4
#define MXT_DIAG_MODE_MASK		0xFC
#define MXT_DIAGNOSTIC_MODE		0
#define MXT_DIAGNOSTIC_PAGE		1
#define MXT_CONFIG_VERSION_LENGTH	30

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

#define MXT_MAX_FINGER			10
#define MXT_DISALEEVT_VALUE		0x33

/* Registers */
#define MXT_OBJECT_NUM_MAX		200
#define MXT_OBJECT_START			0x07
#define MXT_OBJECT_SIZE			6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE		256

#define MXT_INFOMATION_BLOCK_SIZE		7
#define MXT_OBJECT_TABLE_ELEMENT_SIZE	6
#define MXT_OBJECT_TABLE_START_ADDRESS	7

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE			0xa5
#define MXT_RESET_VALUE			0x01
#define MXT_BACKUP_VALUE			0x55
#define MXT_STOP_DYNAMIC_CONFIG	0x33

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET		0
#define MXT_COMMAND_BACKUPNV		1
#define MXT_COMMAND_CALIBRATE		2
#define MXT_COMMAND_REPORTALL		3
#define MXT_COMMAND_DIAGNOSTIC	5

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP	1

#define PATCH_EVENT_KNOCKON	(0x01 << 1)
#define PATCH_EVENT_TA		(0x01 << 2)
#define POWERLOCK_FW_UP		(0x01 << 1)
#define POWERLOCK_SYSFS		(0x01 << 2)

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP			(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE			(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37			37
#define MXT_GEN_MESSAGE_T5				5
#define MXT_GEN_COMMAND_T6				6
#define MXT_GEN_POWER_T7					7
#define MXT_GEN_ACQUIRE_T8				8
#define MXT_TOUCH_MULTI_T9				9
#define MXT_TOUCH_KEYARRAY_T15			15
#define MXT_TOUCH_PROXIMITY_T23			23
#define MXT_TOUCH_MULTITOUCHSCREEN_T100		100
#define MXT_PROCI_GRIPSUPPRESSION_T40		40
#define MXT_PROCI_TOUCHSUPPRESSION_T42		42
#define MXT_PROCI_STYLUS_T47				47
#define MXT_PROCG_NOISESUPPRESSION_T48		48
#define MXT_PROCI_ADAPTIVETHRESHOLD_T55		55
#define MXT_PROCI_SHIELDLESS_T56			56
#define MXT_PROCI_EXTRATOUCHSCREENDATA_T57	57
#define MXT_PROCI_LENSBENDING_T65			65
#define MXT_PROCI_PALMGESTUREPROCESSOR_T69	69
#define MXT_PROCG_NOISESUPPRESSION_T72		72
#define MXT_GLOVEDETECTION_T78			78
#define MXT_RETRANSMISSIONCOMPENSATION_T80	80
#define MXT_PROCI_GESTUREPROCESSOR_T84		84
#define MXT_PROCI_TOUCH_SEQUENCE_LOGGER_T93	93
#define MXT_PROCI_SCHNOISESUPPRESSION_T103	103
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19			19
#define MXT_SPT_SELFTEST_T25			25
#define MXT_SPT_USERDATA_T38			38
#define MXT_SPT_MESSAGECOUNT_T44		44
#define MXT_SPT_CTECONFIG_T46			46
#define MXT_SPT_NOISESUPPRESSION_T48	48
#define MXT_SPT_TIMER_T61			61
#define MXT_SPT_GOLDENREFERENCES_T66	66
#define MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70	70
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71		71
#define MXT_SPT_SELFCAPCBCRCONFIG_T102		102
#define MXT_SPT_AUXTOUCHCONFIG_T104		104
#define MXT_SPT_TOUCHSCREENHOVER_T101		101
#define MXT_SPT_DRIVENPLATEHOVERCONFIG_T105	105
#define MXT_PROCI_ONETOUCH_T24	24
#define MXT_SPT_PROTOTYPE_T35		35

#define UDF_MESSAGE_COMMAND 		50
#define MAX_POINT_SIZE_FOR_LPWG	10
#define MAX_T37_MSG_SIZE			9

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG			0xff

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)
#define MXT_T100_TCHAUX_RESV	(1 << 3)
#define MXT_T100_TCHAUX_PEAK	(1 << 4)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_FRIST_ID_SUPPRESSION	(1 << 6)
#define MXT_T100_TYPE_MASK		0x70
#define MXT_T100_TYPE_FINGER		0x10
#define MXT_T100_TYPE_STYLUS		0x20
#define MXT_T100_TYPE_GLOVE		0x50
#define MXT_T100_TYPE_PALM		0x60
#define MXT_T100_STATUS_MASK		0x0F
#define MXT_T100_PRESS			0x04
#define MXT_T100_RELEASE			0x05
#define MXT_T100_MOVE			0x01
#define MXT_T100_SUPPRESSION		0x03

/* Message type of T100 object */
#define MXT_T100_SCREEN_MSG_FIRST_RPT_ID	0
#define MXT_T100_SCREEN_MSG_SECOND_RPT_ID	1
#define MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID	2

/* Event Types of T100 object */
#define MXT_T100_DETECT_MSG_MASK			7

/* Tool types of T100 object */
#define MXT_T100_TYPE_RESERVED			0
#define MXT_T100_TYPE_PATCH_FINGER			1
#define MXT_T100_TYPE_PASSIVE_STYLUS		2
#define MXT_T100_TYPE_ACTIVE_STYLUS		3
#define MXT_T100_TYPE_HOVERING_FINGER		4
#define MXT_T100_TYPE_GLOVE_TOUCH			5
#define MXT_T100_TYPE_LARGE_TOUCH			6

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* Delay times */
#define MXT_BACKUP_TIME		20		/* msec */
#define MXT_RESET_TIME		200		/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	1000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300		/* msec */
#define MXT_WAKEUP_TIME		25		/* msec */
#define MXT_REGULATOR_DELAY	150		/* msec */
#define MXT_POWERON_DELAY	150		/* msec */
#define MXT_SELFTEST_TIME	3000	/* msec */
#define MXT_WAITED_UDF_TIME	200 		/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0		/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80		/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK		0x02
#define MXT_FRAME_CRC_FAIL		0x03
#define MXT_FRAME_CRC_PASS		0x04
#define MXT_APP_CRC_FAIL			0x40		/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK		0x3f
#define MXT_BOOT_EXTENDED_ID		(1 << 5)
#define MXT_BOOT_ID_MASK			0x1f

#define MXT_STATE_INACTIVE		0
#define MXT_STATE_RELEASE		1
#define MXT_STATE_PRESS			2
#define MXT_STATE_MOVE			3

#define TIME_WRAP_AROUND(x, y)		(((y) > (x)) ? (y) - (x) : (0 - (x)) + (y))

#define MS_TO_NS(x)				(x * 1E6L)

#define MXT_PATCH_MAGIC			0x52296416
#define MXT_PATCH_VERSION		1
#define MXT_PATCH_MAX_STAGE		255
#define MXT_PATCH_MAX_TLINE		255
#define MXT_PATCH_MAX_TRIG		255
#define MXT_PATCH_MAX_ITEM		255
#define MXT_PATCH_MAX_TYPE		255
#define MXT_PATCH_MAX_CON		255
#define MXT_PATCH_MAX_EVENT		255
#define MXT_PATCH_MAX_MSG_SIZE	10
#define MXT_PATCH_T71_DATA_MAX	112
#define MXT_PATCH_T71_PTN_OPT		1
#define MXT_PATCH_T71_PTN_CAL		2
#define MXT_PATCH_LOCK_CHECK		1
#define MXT_PATCH_SUPP_CHECK		1
#define MXT_PATCH_T9STATUS_CHK		1
#define MXT_PATCH_START_FROM_NORMAL	1
#define MXT_PATCH_STAGE_RESET		1
#define MXT_PATCH_USER_DATA		1
#define MXT_PATCH_USER_DATA_MAX	32
#define MXT_PATCH_TRIGGER_MOD		1

#define MXT_XML_CON_NONE		"0"
#define MXT_XML_CON_EQUAL	"="
#define MXT_XML_CON_BELOW	"<"
#define MXT_XML_CON_ABOVE	">"
#define MXT_XML_CON_PLUS		"+"
#define MXT_XML_CON_MINUS	"-"
#define MXT_XML_CON_MUL		"*"
#define MXT_XML_CON_DIV		"/"
#define MXT_XML_CON_MASK		"&"
#define MXT_XML_SRC_NONE		"0"
#define MXT_XML_SRC_CHRG		"TA"
#define MXT_XML_SRC_FCNT		"FCNT"
#define MXT_XML_SRC_AREA		"AREA"
#define MXT_XML_SRC_AMP		"AMP"
#define MXT_XML_SRC_SUM		"SUM"
#define MXT_XML_SRC_TCH		"TCH"
#define MXT_XML_SRC_ATCH		"ATCH"
#define MXT_XML_SRC_KCNT		"KCNT"
#define MXT_XML_SRC_KVAL		"KVAL"
#define MXT_XML_SRC_T9STATUS	"T9STATUS"
#define MXT_XML_SRC_USER1	"USR1"
#define MXT_XML_SRC_USER2	"USR2"
#define MXT_XML_SRC_USER3	"USR3"
#define MXT_XML_SRC_USER4	"USR4"
#define MXT_XML_SRC_USER5	"USR5"
#define MXT_XML_ACT_NONE		"0"
#define MXT_XML_ACT_CAL		"CAL"
#define MXT_XML_ACT_EXTMR	"EXTMR"

#define MXT_SUPPRESS_MSG_MASK		(1 << 1)
#define MXT_AMPLITUDE_MSG_MASK	(1 << 2)
#define MXT_VECTOR_MSG_MASK		(1 << 3)
#define MXT_MOVE_MSG_MASK		(1 << 4)
#define MXT_RELEASE_MSG_MASK		(1 << 5)
#define MXT_PRESS_MSG_MASK		(1 << 6)
#define MXT_DETECT_MSG_MASK		(1 << 7)

#define MXT_MSG_T15_STATUS		0x00
#define MXT_MSG_T15_KEYSTATE		0x01
#define MXT_MSGB_T15_DETECT		0x80

#define CHARGER_PLUGGED			0
#define CHARGER_UNplugged		1
#define DEEP_SLEEP_WAKEUP		2
#define CHARGER_KNOCKON_SLEEP		3
#define CHARGER_KNOCKON_WAKEUP	4
#define NOCHARGER_KNOCKON_SLEEP	5
#define NOCHARGER_KNOCKON_WAKEUP	6

#define PATCH_EVENT_PAIR_NUM		4
#define PATCH_EVENT_AAT			11
#define CHARGER_PLUGGED_AAT		12
#define PATCH_LOW_TEMP			13
#define PATCH_NOT_LOW_TEMP		14
#define IME_MODE_ENABLE			15
#define IME_MODE_DISABLE			16
#define SELF_CAP_OFF_NOISE_SUPPRESSION	17
#define SELF_CAP_ON_NOISE_RECOVER	18

#define RETRY_CNT	2

#define SELF_DIAGNOSTIC_FILE_PATH "/mnt/sdcard/touch_self_test.txt"
#define DELTA_FILE_PATH "/mnt/sdcard/touch_delta.txt"
#define SELF_DIAGNOSTIC_STATUS_COMPLETE	0
#define SELF_DIAGNOSTIC_STATUS_RUNNING	1

enum {
	MXT_PATCH_CON_NONE = 0,
	MXT_PATCH_CON_EQUAL,
	MXT_PATCH_CON_BELOW,
	MXT_PATCH_CON_ABOVE,
	MXT_PATCH_CON_PLUS,
	MXT_PATCH_CON_MINUS,
	MXT_PATCH_CON_MUL,
	MXT_PATCH_CON_DIV,
	MXT_PATCH_CON_MASK,
	/*... */
	MXT_PATCH_CON_END
};

enum {
	MXT_PATCH_ITEM_NONE = 0,
	MXT_PATCH_ITEM_CHARG,
	MXT_PATCH_ITEM_FCNT,
	MXT_PATCH_ITEM_AREA,
	MXT_PATCH_ITEM_AMP,
	MXT_PATCH_ITEM_SUM,
	MXT_PATCH_ITEM_TCH,
	MXT_PATCH_ITEM_ATCH,
	MXT_PATCH_ITEM_KCNT,
	MXT_PATCH_ITEM_KVAL,
	MXT_PATCH_ITEM_T9STATUS,
	MXT_PATCH_ITEM_USER1,
	MXT_PATCH_ITEM_USER2,
	MXT_PATCH_ITEM_USER3,
	MXT_PATCH_ITEM_USER4,
	MXT_PATCH_ITEM_USER5,
	MXT_PATCH_ITEM_USER6,
	MXT_PATCH_ITEM_USER7,
	/*... */
	MXT_PATCH_ITEM_END
};

enum {
	MXT_PATCH_ACTION_NONE = 0,
	MXT_PATCH_ACTION_CAL,
	MXT_PATCH_ACTION_EXTEND_TIMER,
	MXT_PATCH_ACTION_GOTO_STAGE,
	MXT_PATCH_ACTION_CHANGE_START,
	/*... */
	MXT_PATCH_ACTION_END
};

enum {
	POWER_OFF = 0,
	POWER_ON
};

enum{
	FINGER_INACTIVE,
	FINGER_RELEASED,
	FINGER_PRESSED,
	FINGER_MOVED
};

struct patch_header { /* 32b */
	u32	magic;
	u32	size;
	u32	date;
	u16	version;
	u8	option;
	u8	debug;
	u8	timer_id;
	u8	stage_cnt;
	u8	trigger_cnt;
	u8	event_cnt;
	u8	reserved[12];
};

struct stage_def {	/* 8b */
	u8	stage_id;
	u8	option;
	u16	stage_period;
	u8	cfg_cnt;
	u8	test_cnt;
	u16	reset_period;
};

struct stage_cfg {	/* 4b */
	u8	obj_type;
	u8	option;
	u8	offset;
	u8	val;
};

struct test_line {	/* 12b */
	u8	test_id;
	u8	item_cnt;
	u8	cfg_cnt;
	u8	act_id;
	u16	act_val;
	u16	option;
	u16	check_cnt;
	u8	reserved[2];
};

struct action_cfg {	/* 4b */
	u8	obj_type;
	u8	option;
	u8	offset;
	u8	val;
};

struct item_val {	/* 4b */
	u8	val_id;
	u8	val_eq;
	u16	val;
};

struct test_item {	/* 8b */
	u8	src_id;
	u8	cond;
	u8	reserved[2];
	struct item_val ival;
};

/* Message Trigger */
struct trigger {		/* 12b */
	u8	tid;
	u8	option;
	u8	object;
	u8	index;
	u8	match_cnt;
	u8	cfg_cnt;
	u8	reserved[3];
	u8	act_id;
	u16	act_val;
};

struct match {		/*8b */
	u8	offset;
	u8	cond;
	u16	mask;
	u8	reserved[2];
	u16	val;
};

struct trigger_cfg {	/* 4b */
	u8	obj_type;
	u8	reserved;
	u8	offset;
	u8	val;
};

/* Event */
struct user_event {	/* 8b */
	u8	eid;
	u8	option;
	u8	cfg_cnt;
	u8	reserved[5];
};

struct event_cfg {	/* 4b */
	u8	obj_type;
	u8	reserved;
	u8	offset;
	u8	val;
};

struct test_src {
	int	charger;
	int	finger_cnt;
	int	area;
	int 	area2;
	int	amp;
	int	sum_size;
	int	tch_ch;
	int	atch_ch;
	int	key_cnt;
	int	key_val;
	int	status;
	int	user1;
	int	user2;
	int	user3;
	int	user4;
	int	user5;
	int	user6;
	int	user7;
};

struct touch_pos {
	u8	tcount[MXT_MAX_FINGER];
	u16	initx[MXT_MAX_FINGER];
	u16	inity[MXT_MAX_FINGER];
	u16	oldx[MXT_MAX_FINGER];
	u16	oldy[MXT_MAX_FINGER];
	u8	locked_id;
	u8	moved_cnt;
	u8	option;
	u8	cal_enable;
	u8	reset_cnt;
	u8	distance;
	u8	maxdiff;
	u8	locked_cnt;
	u8	jitter;
	u8	area;
	u8	amp;
	u8	sum_size_t57;
	u8	tch_count_t57;
	u8	atch_count_t57;
	u8	amp_2finger_min;
	u8	area_2finger_min;
	u8	sum_size_t57_2finger_min;
	u8	tch_count_t57_2finger_min;
	u8	atch_count_t57_2finger_min;
	u8	amp_2finger_max;
	u8	area_2finger_max;
	u8	sum_size_t57_2finger_max;
	u8	tch_count_t57_2finger_max;
	u8	atch_count_t57_2finger_max;
	u8	amp_3finger_min;
	u8	area_3finger_min;
	u8	sum_size_t57_3finger_min;
	u8	tch_count_t57_3finger_min;
	u8	atch_count_t57_3finger_min;
	u8	amp_3finger_max;
	u8	area_3finger_max;
	u8	sum_size_t57_3finger_max;
	u8	tch_count_t57_3finger_max;
	u8	atch_count_t57_3finger_max;
	u8	amp_mfinger_min;
	u8	area_mfinger_min;
	u8	sum_size_t57_mfinger_min;
	u8	tch_count_t57_mfinger_min;
	u8	atch_count_t57_mfinger_min;
	u8	amp_mfinger_max;
	u8	area_mfinger_max;
	u8	sum_size_t57_mfinger_max;
	u8	tch_count_t57_mfinger_max;
	u8	atch_count_t57_mfinger_max;
	u8	xlo_limit;
	u16	xhi_limit;
	u8	ylo_limit;
	u16	yhi_limit;
};

struct touch_supp {
	u32	old_time;
	u8	repeat_cnt;
	u8	time_gap;
	u8	repeat_max;
};

/*Reference Check*/
struct mxt2954_channel_size{
	u8 start_x;
	u8 start_y;
	u8 size_x;
	u8 size_y;
};

/****************************************************************************
* Type Definitions
****************************************************************************/
/**
 * struct mxt_fw_image - Represents a firmware file.
 * @ magic_code: Identifier of file type.
 * @ hdr_len: Size of file header (struct mxt_fw_image).
 * @ cfg_len: Size of configuration data.
 * @ fw_len: Size of firmware data.
 * @ cfg_crc: CRC of configuration settings.
 * @ bin_ver: Version of binary firmware.
 * @ build_ver: Build version of firmware.
 * @ data: Configuration data followed by firmware image.
 */
struct mxt2954_fw_image {
	__le32 magic_code;
	__le32 hdr_len;
	__le32 cfg_len;
	__le32 fw_len;
	__le32 cfg_crc;
	u8 bin_ver;
	u8 build_ver;
	u8 extra_info[32];
	u8 data[0];
} __packed;

struct mxt2954_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt2954_fw_info {
	u8 bin_ver;
	u8 build_ver;
	u32 hdr_len;
	u32 cfg_len;
	u32 fw_len;
	u32 cfg_crc;
	const u8 *cfg_raw_data;	/* start address of configuration data */
	const u8 *fw_raw_data;	/* start address of firmware data */
	struct mxt2954_ts_data *ts;
};

struct mxt2954_finger {
	u16 x;
	u16 y;
	u16 w;
	u16 z;
	u16 component;
	u8 state;
	u8 type;
	u8 event;
	u16 mcount;
};

struct mxt2954_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

struct mxt2954_raw_data {
	u8 num_xnode;
	u8 num_ynode;
	u16 num_nodes;
	u16 *reference;
	s16 *delta;
};

/**
 * struct mxt_cfg_data - Represents a configuration data item.
 * @ type: Type of object.
 * @ instance: Instance number of object.
 * @ size: Size of object.
 * @ register_val: Series of register values for object.
 */
struct mxt2954_cfg_data {
	u8 type;
	u8 instance;
	u8 size;
	u8 register_val[0];
} __packed;

struct mxt2954_reportid {
	u8 type;
	u8 index;
};

struct mxt2954_message {
	u8 reportid;
	u8 message[8];
};

struct mxt2954_patch {
	u8 *patch;
	u16 *stage_addr;
	u16 *tline_addr;
	u16 *trigger_addr;
	u16 *event_addr;
	u16 *src_item;
	u16 *check_cnt;
	u16 period;
	u8 stage_cnt;
	u8 tline_cnt;
	u8 trigger_cnt;
	u8 event_cnt;
	u8 option;
	u8 debug;
	u8 timer_id;
	u8 cur_stage;
	u8 cur_stage_opt;
	u8 run_stage;
	u8 start;
	u8 finger_cnt;
	u8 start_stage;
	u8 skip_test;
	u8 cal_flag;
	u32 stage_timestamp;
};

struct t_data
{
	u16	id;
	u16	x_position;
	u16	y_position;
	u16	width_major;
	u16	width_minor;
	u16	width_orientation;
	u16	pressure;
	u8	status;
	u8	touch_major;
	u8	touch_minor;
	int tool;
	bool is_pen;
	bool is_palm;
	bool skip_report;
};

struct b_data
{
	u16	key_code;
	u16	state;
};

struct touch_data
{
	u8		total_num;
	u8		prev_total_num;
	u8		touch_count;
	u8		state;
	u8		palm;
	u8		prev_palm;
	struct t_data	curr_data[MXT_MAX_FINGER];
	struct t_data	prev_data[MXT_MAX_FINGER];
	struct b_data	curr_button;
	struct b_data	prev_button;
};

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

struct mxt2954_ts_data {
	struct i2c_client	*client;
	struct mxt2954_info	*info;
	struct mxt2954_fw_info	fw_info;
	struct mxt2954_object	*object_table;
	struct mxt2954_object *global_object;
	struct mxt2954_reportid		*reportids;
	struct mxt2954_patch		patch;
	struct mxt2954_finger		fingers[MXT_MAX_FINGER];
	struct mxt2954_channel_size 	channel_size;
	struct mxt2954_raw_data *rawdata;
	struct input_dev *input_dev;
	struct touch_data	ts_data;
	struct bin_attribute mem_access_attr;
	struct regulator *vdd_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;

	void *raw_info_block;
	bool update_input;
	bool	button_lock;
	bool driver_debug_enabled;
	bool debug_enabled;
	bool t57_debug_enabled;
	bool use_debug_reason;
	bool show_delta;
	bool use_mfts;
	bool minios;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int diff_scaling;
	unsigned int irq;
	int t15_num_keys;
	char phys[64];		/* device physical location */
	u32 config_crc;
	u16 mem_size;
	u8 num_touchids;
	u8 max_reportid;
	u8 total_num;
	u8 prev_total_num;
	u8 *msg_buf;
	u8 ref_chk;
	u8 last_message_count;
	u8 fw_ver[2];
	u8 product[10];
	u8 error_check_count[5];
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	u8 t100_aux_resv;
	u8 bootloader_addr;
	u8 finger_count;
	struct t7_config t7_cfg;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;

	u8 T6_reportid;
	u8 t6_status;
	u16 T6_address;

	u16 T7_address;
	u16 T8_address;

	u16 T9_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;

	u8 T15_reportid_min;
	u8 T15_reportid_max;

	u16 T18_address;
	u8 T24_reportid;
	u8 T35_reportid;

	u8 T25_reportid;
	u16 T25_address;

	u16 T42_address;
	u8 T42_reportid_min;
	u8 T42_reportid_max;

	u16 T44_address;

	u16 T46_address;
	u8 T48_reportid;
	u16 T56_address;
	u16 T61_address;
	u8 T61_reportid_min;
	u8 T61_reportid_max;
	u16 T65_address;
	u16 T71_address;
	u16 T72_address;
	u16 T93_address;
	u8 T93_reportid;
	u8 g_tap_cnt;

	u16 T100_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;

	/* LPWG Mode Set */
	struct completion lpwgmode_completion;

	u8 mxt_multi_tap_enable;

	/* Enable reporting of input events */
	bool enable_reporting;

	bool mxt_knock_on_enable;
	bool is_knockCodeDelay;

	u8 lpwg_mode;
	TouchState currState;
	LpwgSetting lpwgSetting;

	/* ATMEL SELF DELTA CHECK */
	u8 self_delta_chk[23];

	/* T71 tmp using before firmware change */
	u8 t71_diff_using;
	u8 t71_diff_val[32];
	u8 charging_mode; /*Charger mode in patch*/

	bool self_test_result;
	u8 self_test_status[4];
	u8 self_test_result_status;
	int **full_cap;
};

/****************************************************************************
* Exported Variables
****************************************************************************/


/****************************************************************************
* Macros
****************************************************************************/
#define TOUCH_PATCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[TOUCH PATCH] " fmt, ##args)

/****************************************************************************
* Global Function Prototypes
****************************************************************************/
struct mxt2954_object *mxt_get_object(struct mxt2954_ts_data *ts, u8 type);
int mxt_read_object(struct mxt2954_ts_data *ts, u8 type, u8 offset, u8 *val);
int mxt_write_object(struct mxt2954_ts_data *ts, u8 type, u8 offset, u8 val);
int mxt_patch_init(struct mxt2954_ts_data *data, u8 *ppatch);
int mxt_patch_event(struct mxt2954_ts_data *data, u8 event_id);
void mxt_patch_message(struct mxt2954_ts_data *data, struct mxt2954_message *message);
void mxt_patch_dump_source(struct mxt2954_ts_data *data, bool do_action);
int mxt_patch_run_stage(struct mxt2954_ts_data *data);

#endif /* _LGTP_DEVICE_MXT2954_H_ */

/* End Of File */

