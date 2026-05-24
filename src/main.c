/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB HID Mouse + Keyboard + BLE HID Client (HOGP)
 *
 * - USB HID Mouse: 转发蓝牙鼠标数据到电脑
 * - USB HID Keyboard: 自动打字演示(A-Z循环)
 * - BLE Central: 扫描并连接蓝牙鼠标(HID Service), 通过HOGP Client接收鼠标报告
 */

#include <sample_usbd.h>

#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/gatt_dm.h>

#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ============================================================
 *  RSSI 阈值：信号强度 >= 此值时尝试连接
 *  -30 dBm = 紧贴设备, -55 dBm ≈ 1米, -70 dBm ≈ 5-10米
 *  设为 -55 只扫描 1 米以内的设备
 * ============================================================ */
#define BLE_HID_RSSI_THRESHOLD  (-55)

/* 调试阶段：直接指定鼠标 MAC 地址 */
#define BLE_MOUSE_TARGET_MAC   "FB:DE:BD:46:FF:0B"

/* ============================================================
 *  鼠标相关 (USB HID)
 * ============================================================ */
static const uint8_t hid_mouse_report_desc[] = HID_MOUSE_REPORT_DESC(2);

/* USB HID Mouse Report 格式: [按钮, X(int8), Y(int8), 滚轮(int8)] */
enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

K_MSGQ_DEFINE(mouse_msgq, MOUSE_REPORT_COUNT, 10, 1);
static bool mouse_ready;

/* ============================================================
 *  Accept List（白名单）自动重连
 *  ============================================================
 *  绑定成功后，鼠标地址加入 Filter Accept List，
 *  断连后调用 bt_conn_le_create_auto() 启动硬件级自动重连。
 *  BLE 控制器自动扫描定向广播，发现鼠标后自动发起连接请求。
 *  断电重启后，bt_foreach_bond() 从 Flash 恢复绑定地址。
 *  ============================================================ */
static bt_addr_le_t mouse_bond_addr;
static bool mouse_bond_addr_valid;

/* 从 7 字节鼠标报告中提取 12-bit 有符号 X/Y 值，缩放到 [-127, 127]
 *
 * HID Report Map (Report ID 2) 位布局:
 *   Byte 0-1: buttons[15:0]  (16 × 1 bit)
 *   Byte 2:   X[7:0]         (低 8 位)
 *   Byte 3:   X[11:8] | Y[3:0]<<4  (X 高 4 位 + Y 低 4 位打包在同一字节)
 *   Byte 4:   Y[11:4]        (Y 高 8 位)
 *   Byte 5:   Wheel (signed 8-bit)
 *   Byte 6:   AC Pan (signed 8-bit, 水平滚轮)
 *
 * 12-bit 有符号值范围: -2047 ~ 2047
 * USB HID 8-bit 范围: -127 ~ 127
 */
static void unpack_mouse_report_7byte(const uint8_t *data,
				      uint8_t *report)
{
	/* 7 字节报告布局 (HID Report ID 2):
	 *   data[0] = buttons byte 0 (bit0=左键, bit1=右键, bit2=中键)
	 *   data[1] = buttons byte 1 (保留)
	 *   data[2] = X[7:0]       (低 8 位)
	 *   data[3] = X[11:8] | Y[3:0]<<4  (X 高 4 位 + Y 低 4 位)
	 *   data[4] = Y[11:4]      (Y 高 8 位)
	 *   data[5] = Wheel (signed 8-bit)
	 *   data[6] = AC Pan (水平滚轮, signed 8-bit)
	 */

	/* 提取 X: 12-bit 无符号 */
	uint16_t x_raw = (uint16_t)(data[2] | ((data[3] & 0x0F) << 8));
	/* 提取 Y: 12-bit 无符号 */
	uint16_t y_raw = (uint16_t)(((data[3] >> 4) & 0x0F) | (data[4] << 4));

	/* 转换为 12-bit 有符号 (二进制补码, 范围 -2047~2047) */
	int16_t x_val = (x_raw >= 0x0800) ? (int16_t)(x_raw - 4096) : (int16_t)x_raw;
	int16_t y_val = (y_raw >= 0x0800) ? (int16_t)(y_raw - 4096) : (int16_t)y_raw;

	/* 直接截断到 USB HID 8-bit 范围 [-127, 127]
	 * 不缩放: 12-bit 值的 ±2047 范围很少用到,
	 * 实际移动幅度通常 <127, 缩放反而导致小移动被舍入为 0
	 */
	if (x_val > 127) x_val = 127;
	if (x_val < -127) x_val = -127;
	if (y_val > 127) y_val = 127;
	if (y_val < -127) y_val = -127;

	/* 按钮: data[0] 低 3 位 = 左键/右键/中键 */
	report[MOUSE_BTN_REPORT_IDX] = data[0] & 0x07;
	report[MOUSE_X_REPORT_IDX] = (uint8_t)x_val;
	report[MOUSE_Y_REPORT_IDX] = (uint8_t)y_val;

	/* 滚轮 */
	report[MOUSE_WHEEL_REPORT_IDX] = data[5];
}

/* BLE 鼠标通知回调 - 根据 HID Report Map 解析鼠标报告
 *
 * M720 通过多个特征值发送数据:
 *   [1] Boot Mouse (0x2A4B) 3 字节: [buttons, X(int8), Y(int8)]
 *   [2] Report (0x2A4D) handle 0x0030 7 字节: HID Report ID 2 鼠标报告
 *       (见 unpack_mouse_report_7byte 的位布局)
 *   [3] Report (0x2A4D) handle 0x0034 19 字节: Report ID 0x11 (17),
 *       Logitech HID++ 厂商报告, data[0]!=0xFF 但通过 len==19 识别
 *
 * 处理策略:
 *   - len == 19 → Logitech HID++ 报告, 跳过
 *   - len <= 3  → Boot Mouse 格式 [buttons, X(int8), Y(int8)]
 *   - len >= 7  → Report ID 2 格式, 用 unpack_mouse_report_7byte 解析
 */
static void ble_mouse_report_forward(const uint8_t *data, uint16_t len)
{
	uint8_t report[MOUSE_REPORT_COUNT] = {0};

	if (len < 3) {
		LOG_WRN("Short mouse report: %u bytes", len);
		return;
	}

	/* 打印原始数据前 7 字节用于调试 */
	LOG_INF("RAW[%u]: %02x %02x %02x %02x %02x %02x %02x",
		len,
		data[0], data[1], data[2], data[3], data[4], data[5], data[6]);

	/* 跳过 19 字节 Logitech HID++ 报告 (Report ID 0x11) */
	if (len == 19) {
		LOG_DBG("Skipping HID++ report (19 bytes)");
		return;
	}

	if (len <= 3) {
		/* Boot Mouse 格式: [buttons(1B), X(int8), Y(int8)] */
		report[MOUSE_BTN_REPORT_IDX] = data[0];
		report[MOUSE_X_REPORT_IDX] = data[1];
		report[MOUSE_Y_REPORT_IDX] = data[2];
	} else {
		/* 7 字节 Report ID 2 格式: 用 HID Report Map 定义的位布局解析 */
		unpack_mouse_report_7byte(data, report);
	}

	if (k_msgq_put(&mouse_msgq, report, K_NO_WAIT) != 0) {
		LOG_DBG("Mouse msgq full, dropping report");
	}
}

static void mouse_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("Mouse HID ready: %s", ready ? "yes" : "no");
	mouse_ready = ready;
}

static int mouse_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
{
	return 0;
}

struct hid_device_ops mouse_ops = {
	.iface_ready = mouse_iface_ready,
	.get_report = mouse_get_report,
};

/* ============================================================
 *  键盘相关 (USB HID) - 自动打字演示
 * ============================================================ */
static const uint8_t hid_keyboard_report_desc[] = HID_KEYBOARD_REPORT_DESC();

enum kb_report_idx {
	KB_MOD_KEY = 0,
	KB_RESERVED,
	KB_KEY_CODE1,
	KB_KEY_CODE2,
	KB_KEY_CODE3,
	KB_KEY_CODE4,
	KB_KEY_CODE5,
	KB_KEY_CODE6,
	KB_REPORT_COUNT,
};

/* HID 键盘码：A=0x04, B=0x05, ..., Z=0x1D */
static const uint8_t hid_keycodes[] __maybe_unused = {
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
	0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
	0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
	0x1C, 0x1D
};

K_MSGQ_DEFINE(kb_msgq, KB_REPORT_COUNT, 10, 1);
static bool kb_ready;

#if 0
/* 键盘自动打字任务：每1秒按一个字母，A-Z循环（调试用，暂时关闭） */
void auto_keyboard_task(void *arg1, void *arg2, void *arg3)
{
	uint8_t index = 0;
	const char chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

	while (1) {
		/* ---- 按下字母 ---- */
		uint8_t report[KB_REPORT_COUNT] = {0};
		report[KB_KEY_CODE1] = hid_keycodes[index];

		if (k_msgq_put(&kb_msgq, report, K_NO_WAIT) != 0) {
			LOG_ERR("KB put press failed");
		}
		LOG_INF("KB press: %c", chars[index]);

		k_msleep(150);

		/* ---- 松开字母 ---- */
		uint8_t report_release[KB_REPORT_COUNT] = {0};
		if (k_msgq_put(&kb_msgq, report_release, K_NO_WAIT) != 0) {
			LOG_ERR("KB put release failed");
		}
		LOG_INF("KB release: %c", chars[index]);

		/* 下一个字母 */
		index++;
		if (index >= 26) {
			index = 0;
			LOG_INF("=== KB cycle complete ===");
		}

		k_msleep(850);
	}
}

K_THREAD_STACK_DEFINE(auto_keyboard_stack, 2048);
static struct k_thread auto_keyboard_thread;
#endif /* 0 - auto keyboard task disabled */

static void kb_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("Keyboard HID ready: %s", ready ? "yes" : "no");
	kb_ready = ready;
}

static int kb_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
{
	return 0;
}

struct hid_device_ops kb_ops = {
	.iface_ready = kb_iface_ready,
	.get_report = kb_get_report,
};

/* ============================================================
 *  WebHID 配置接口 (hid_dev_2)
 *
 *  通过 USB HID Feature 报告实现浏览器 WebHID API 通信，
 *  用于读取/保存设备配置参数。
 *
 *  协议格式 (64 字节 Feature Report, Report ID 1):
 *
 *   主机 → 设备 (Set_Report):
 *     [0]  = Report ID (0x01)
 *     [1]  = 命令 (CMD_xxx)
 *     [2]  = 参数 ID
 *     [3]  = 数据长度 (仅 WRITE_PARAM)
 *     [4..63] = 数据 (仅 WRITE_PARAM)
 *
 *   设备 → 主机 (Get_Report):
 *     [0]  = Report ID (0x01)
 *     [1]  = 状态 (STATUS_xxx)
 *     [2]  = 参数 ID (回显)
 *     [3]  = 数据长度
 *     [4..63] = 数据
 *
 *  使用 Zephyr settings 子系统持久化保存参数到 Flash。
 * ============================================================ */

/* ---- 命令定义 ---- */
#define CFG_CMD_PING           0x01  /* 测试连接，设备回复 PONG */
#define CFG_CMD_READ_PARAM     0x02  /* 读取参数值 */
#define CFG_CMD_WRITE_PARAM    0x03  /* 写入参数值 */
#define CFG_CMD_SAVE_ALL       0x04  /* 保存所有参数到 Flash */
#define CFG_CMD_LOAD_ALL       0x05  /* 从 Flash 重新加载所有参数 */
#define CFG_CMD_LIST_PARAMS    0x06  /* 列出所有可用的参数 ID */

/* ---- 状态码 ---- */
#define CFG_STATUS_OK               0x00
#define CFG_STATUS_BUSY             0x01
#define CFG_STATUS_UNKNOWN_CMD      0x02
#define CFG_STATUS_INVALID_PARAM    0x03
#define CFG_STATUS_INVALID_LEN      0x04
#define CFG_STATUS_ERROR            0xFF

/* ---- 参数 ID 定义 ---- */
#define CFG_PARAM_RSSI_THRESHOLD  1  /* int8_t, 范围 -127~0, 默认 -90 */
#define CFG_PARAM_AUTO_RECONNECT  2  /* uint8_t, 0/1, 默认 1 */
#define CFG_PARAM_MOUSE_SPEED     3  /* uint8_t, 1~10, 默认 5 */
#define CFG_PARAM_DEMO_INT        4  /* int32_t, 演示用整数参数 */
#define CFG_PARAM_DEMO_STRING     5  /* char[16], 演示用字符串参数 */

#define CFG_PARAM_COUNT           5
#define CFG_REPORT_SIZE           64
#define CFG_REPORT_ID             1

/* ---- 参数名称表（用于 LIST_PARAMS 响应） ---- */
struct cfg_param_info {
	uint8_t id;
	uint8_t size;   /* 参数数据大小（字节） */
	const char *name;
};

static const struct cfg_param_info cfg_params[] = {
	{ CFG_PARAM_RSSI_THRESHOLD, 1, "rssi_threshold" },
	{ CFG_PARAM_AUTO_RECONNECT, 1, "auto_reconnect" },
	{ CFG_PARAM_MOUSE_SPEED,    1, "mouse_speed" },
	{ CFG_PARAM_DEMO_INT,       4, "demo_int" },
	{ CFG_PARAM_DEMO_STRING,   16, "demo_string" },
};

/* ---- 参数存储（运行时值） ---- */
static struct {
	int8_t  rssi_threshold;
	uint8_t auto_reconnect;
	uint8_t mouse_speed;
	int32_t demo_int;
	char    demo_string[16];
} cfg_data = {
	.rssi_threshold = -90,
	.auto_reconnect = 1,
	.mouse_speed    = 5,
	.demo_int       = 12345,
	.demo_string    = "Hello WebHID",
};

/* ---- 打印所有参数当前值 ---- */
static void cfg_dump_values(void)
{
	LOG_INF("=== CFG Parameter Dump ===");
	LOG_INF("  rssi_threshold = %d (int8)", cfg_data.rssi_threshold);
	LOG_INF("  auto_reconnect = %u (uint8)", cfg_data.auto_reconnect);
	LOG_INF("  mouse_speed    = %u (uint8)", cfg_data.mouse_speed);
	LOG_INF("  demo_int       = %d (int32)", cfg_data.demo_int);
	LOG_INF("  demo_string    = '%s' (char[16])", cfg_data.demo_string);
	LOG_INF("=== End of Dump ===");
}

/* ---- 配置状态 ---- */
static bool cfg_ready;
/* 用于 Get_Report 响应的缓冲区（USB 栈线程上下文无法阻塞） */
static uint8_t cfg_response_buf[CFG_REPORT_SIZE];
/* 标记是否有待读取的响应数据 */
static bool cfg_response_pending;

/* ---- 通过参数 ID 获取参数值和大小 ---- */
static int cfg_get_value(uint8_t param_id, uint8_t *out_buf, uint8_t buf_size)
{
	switch (param_id) {
	case CFG_PARAM_RSSI_THRESHOLD:
		if (buf_size < 1) return -ENOBUFS;
		out_buf[0] = (uint8_t)(cfg_data.rssi_threshold);
		return 1;
	case CFG_PARAM_AUTO_RECONNECT:
		if (buf_size < 1) return -ENOBUFS;
		out_buf[0] = cfg_data.auto_reconnect;
		return 1;
	case CFG_PARAM_MOUSE_SPEED:
		if (buf_size < 1) return -ENOBUFS;
		out_buf[0] = cfg_data.mouse_speed;
		return 1;
	case CFG_PARAM_DEMO_INT:
		if (buf_size < 4) return -ENOBUFS;
		sys_put_le32(cfg_data.demo_int, out_buf);
		return 4;
	case CFG_PARAM_DEMO_STRING:
	{
		size_t len = strlen(cfg_data.demo_string);
		if (len > buf_size) len = buf_size;
		memcpy(out_buf, cfg_data.demo_string, len);
		return len;
	}
	default:
		return -EINVAL;
	}
}

/* ---- 通过参数 ID 设置参数值 ---- */
static int cfg_set_value(uint8_t param_id, const uint8_t *data, uint8_t len)
{
	switch (param_id) {
	case CFG_PARAM_RSSI_THRESHOLD:
		if (len < 1) return -EINVAL;
		cfg_data.rssi_threshold = (int8_t)data[0];
		LOG_INF("CFG: rssi_threshold set to %d", cfg_data.rssi_threshold);
		return 0;
	case CFG_PARAM_AUTO_RECONNECT:
		if (len < 1) return -EINVAL;
		cfg_data.auto_reconnect = data[0] ? 1 : 0;
		LOG_INF("CFG: auto_reconnect set to %u", cfg_data.auto_reconnect);
		return 0;
	case CFG_PARAM_MOUSE_SPEED:
		if (len < 1) return -EINVAL;
		cfg_data.mouse_speed = CLAMP(data[0], 1, 10);
		LOG_INF("CFG: mouse_speed set to %u", cfg_data.mouse_speed);
		return 0;
	case CFG_PARAM_DEMO_INT:
		if (len < 4) return -EINVAL;
		cfg_data.demo_int = (int32_t)sys_get_le32(data);
		LOG_INF("CFG: demo_int set to %d", cfg_data.demo_int);
		return 0;
	case CFG_PARAM_DEMO_STRING:
	{
		size_t copy_len = MIN(len, sizeof(cfg_data.demo_string) - 1);
		memcpy(cfg_data.demo_string, data, copy_len);
		cfg_data.demo_string[copy_len] = '\0';
		LOG_INF("CFG: demo_string set to '%s'", cfg_data.demo_string);
		return 0;
	}
	default:
		return -EINVAL;
	}
}

/* ---- Settings 回调：从 Flash 加载参数 ---- */
static int cfg_settings_set(const char *key, size_t len,
			    settings_read_cb read_cb, void *cb_arg)
{
	if (!key) {
		return 0;
	}

	if (strcmp(key, "rssi_threshold") == 0) {
		if (len != sizeof(cfg_data.rssi_threshold)) return 0;
		read_cb(cb_arg, &cfg_data.rssi_threshold, len);
	} else if (strcmp(key, "auto_reconnect") == 0) {
		if (len != sizeof(cfg_data.auto_reconnect)) return 0;
		read_cb(cb_arg, &cfg_data.auto_reconnect, len);
	} else if (strcmp(key, "mouse_speed") == 0) {
		if (len != sizeof(cfg_data.mouse_speed)) return 0;
		read_cb(cb_arg, &cfg_data.mouse_speed, len);
	} else if (strcmp(key, "demo_int") == 0) {
		if (len != sizeof(cfg_data.demo_int)) return 0;
		read_cb(cb_arg, &cfg_data.demo_int, len);
	} else if (strcmp(key, "demo_string") == 0) {
		size_t str_len = MIN(len, sizeof(cfg_data.demo_string) - 1);
		read_cb(cb_arg, cfg_data.demo_string, str_len);
		cfg_data.demo_string[str_len] = '\0';
	}

	return 0;
}

/* 注册 settings 处理器 */
SETTINGS_STATIC_HANDLER_DEFINE(cfg, "cfg", NULL, cfg_settings_set, NULL, NULL);

/* ---- 保存单个参数到 Flash ---- */
static int cfg_save_param(uint8_t param_id)
{
	const char *key = NULL;
	const void *data = NULL;
	size_t size = 0;

	switch (param_id) {
	case CFG_PARAM_RSSI_THRESHOLD:
		key = "cfg/rssi_threshold";
		data = &cfg_data.rssi_threshold;
		size = sizeof(cfg_data.rssi_threshold);
		break;
	case CFG_PARAM_AUTO_RECONNECT:
		key = "cfg/auto_reconnect";
		data = &cfg_data.auto_reconnect;
		size = sizeof(cfg_data.auto_reconnect);
		break;
	case CFG_PARAM_MOUSE_SPEED:
		key = "cfg/mouse_speed";
		data = &cfg_data.mouse_speed;
		size = sizeof(cfg_data.mouse_speed);
		break;
	case CFG_PARAM_DEMO_INT:
		key = "cfg/demo_int";
		data = &cfg_data.demo_int;
		size = sizeof(cfg_data.demo_int);
		break;
	case CFG_PARAM_DEMO_STRING:
		key = "cfg/demo_string";
		data = cfg_data.demo_string;
		size = strlen(cfg_data.demo_string) + 1;
		break;
	default:
		return -EINVAL;
	}

	int err = settings_save_one(key, data, size);
	if (err) {
		LOG_ERR("CFG: Failed to save %s: %d", key, err);
	} else {
		LOG_INF("CFG: Saved %s", key);
	}
	return err;
}

/* ---- 保存所有参数到 Flash ---- */
static int cfg_save_all(void)
{
	LOG_INF("CFG: Saving all %d parameters to flash...", CFG_PARAM_COUNT);
	cfg_dump_values();
	for (int i = 0; i < CFG_PARAM_COUNT; i++) {
		int err = cfg_save_param(cfg_params[i].id);
		if (err) {
			LOG_ERR("CFG: Save all FAILED at param #%d: %d", cfg_params[i].id, err);
			return err;
		}
	}
	LOG_INF("CFG: All parameters saved to flash OK");
	/* 验证：重新加载并打印 */
	LOG_INF("CFG: Verifying by reloading from flash...");
	settings_load_subtree("cfg");
	cfg_dump_values();
	return 0;
}

/* ---- 构建 List Params 响应 ---- */
static int cfg_build_list_response(uint8_t *buf, uint8_t buf_size)
{
	uint8_t pos = 0;
	for (int i = 0; i < CFG_PARAM_COUNT && pos < buf_size - 4; i++) {
		/* 格式: [ID(1), Size(1), Name(null-terminated)] */
		buf[pos++] = cfg_params[i].id;
		buf[pos++] = cfg_params[i].size;
		size_t name_len = strlen(cfg_params[i].name);
		size_t copy = MIN(name_len, buf_size - pos - 1);
		memcpy(&buf[pos], cfg_params[i].name, copy);
		pos += copy;
		buf[pos++] = '\0'; /* null 终止 */
	}
	return pos;
}

/* ---- HID Report Descriptor for WebHIC Config (Vendor-Defined) ---- */
static const uint8_t hid_config_report_desc[] = {
	/* Usage Page (Vendor Defined 0xFF00) */
	0x06, 0x00, 0xFF,       /* Usage Page (Vendor Defined 0xFF00) */
	0x09, 0x01,             /* Usage (Vendor Usage 1 - Config) */
	0xA1, 0x01,             /* Collection (Application) */

	/* Report ID 1: Config Feature Report (64 bytes) */
	0x85, CFG_REPORT_ID,    /*   Report ID (1) */
	0x09, 0x01,             /*   Usage (Config Data) */
	0x15, 0x00,             /*   Logical Minimum (0) */
	0x26, 0xFF, 0x00,       /*   Logical Maximum (255) */
	0x75, 0x08,             /*   Report Size (8 bits) */
	0x95, CFG_REPORT_SIZE,  /*   Report Count (64) */
	0xB1, 0x02,             /*   Feature (Data,Var,Abs) */

	0xC0                    /* End Collection */
};

/* 配置 HID 就绪回调 */
static void cfg_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("Config HID ready: %s", ready ? "yes" : "no");
	cfg_ready = ready;

	if (ready) {
		/* 加载配置参数 */
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			LOG_INF("CFG: Loading settings from flash...");
			settings_load_subtree("cfg");
			cfg_dump_values();
		}
	}
}

/* 配置 HID Get_Report 回调：主机读取 Feature 报告 */
static int cfg_get_report(const struct device *dev,
			  const uint8_t type, const uint8_t id,
			  const uint16_t len, uint8_t *const buf)
{
	if (type != HID_REPORT_TYPE_FEATURE || id != CFG_REPORT_ID) {
		return -EINVAL;
	}

	/* 检查是否有待发送的响应数据 */
	if (!cfg_response_pending) {
		/* 没有待发送的数据，返回空报告 */
		memset(buf, 0, MIN(len, CFG_REPORT_SIZE));
		buf[0] = CFG_REPORT_ID;
		buf[1] = CFG_STATUS_BUSY;
		return MIN(len, CFG_REPORT_SIZE);
	}

	uint16_t copy_len = MIN(len, CFG_REPORT_SIZE);
	memcpy(buf, cfg_response_buf, copy_len);
	cfg_response_pending = false;

	LOG_DBG("CFG: Get_Report returning %u bytes, status=%02x",
		copy_len, buf[1]);

	return copy_len;
}

/* 配置 HID Set_Report 回调：主机发送 Feature 报告（命令）
 *
 * USB 数据格式（Chrome WebHID 自动在数据前追加 Report ID）：
 *   buf[0] = Report ID (0x01)
 *   buf[1] = 命令 (CMD_xxx)
 *   buf[2] = 参数 ID
 *   buf[3] = 数据长度 (仅 WRITE_PARAM)
 *   buf[4..] = 数据
 */
static int cfg_set_report(const struct device *dev,
			  const uint8_t type, const uint8_t id,
			  const uint16_t len, const uint8_t *const buf)
{
	if (type != HID_REPORT_TYPE_FEATURE || id != CFG_REPORT_ID) {
		return -EINVAL;
	}

	/* 打印接收到的原始数据（前16字节）用于调试 */
	LOG_INF("CFG: Set_Report received: len=%u, type=0x%02x, id=%u", len, type, id);
	LOG_HEXDUMP_INF(buf, MIN(len, 16), "CFG: Set_Report raw");

	if (len < 2 || buf[0] != CFG_REPORT_ID) {
		LOG_ERR("CFG: Invalid header: len=%u, buf[0]=0x%02x (expected 0x%02x)",
			len, buf[0], CFG_REPORT_ID);
		return -EINVAL;
	}

	uint8_t cmd = buf[1];
	uint8_t param_id = (len > 2) ? buf[2] : 0;
	uint8_t data_len = (len > 3) ? buf[3] : 0;
	const uint8_t *data = (len > 4) ? &buf[4] : NULL;

	LOG_INF("CFG: Command: cmd=0x%02x, param=%u, data_len=%u", cmd, param_id, data_len);

	/* 构建响应（存入 cfg_response_buf 供 Get_Report 读取） */
	memset(cfg_response_buf, 0, CFG_REPORT_SIZE);
	cfg_response_buf[0] = CFG_REPORT_ID;
	cfg_response_buf[1] = CFG_STATUS_OK;

	switch (cmd) {
	case CFG_CMD_PING:
		LOG_INF("CFG: PING command");
		/* 返回 PONG + 固件版本信息 */
		cfg_response_buf[2] = 0;
		cfg_response_buf[3] = 6;
		memcpy(&cfg_response_buf[4], "PONG!", 5);
		cfg_response_buf[9] = 0; /* 版本号: 主版本 */
		cfg_response_buf[10] = 1; /* 次版本 */
		break;

	case CFG_CMD_READ_PARAM: {
		LOG_INF("CFG: READ_PARAM #%u", param_id);
		uint8_t value_buf[CFG_REPORT_SIZE - 4];
		int ret = cfg_get_value(param_id, value_buf, sizeof(value_buf));
		if (ret < 0) {
			LOG_ERR("CFG: READ_PARAM #%u failed: %d", param_id, ret);
			cfg_response_buf[1] = CFG_STATUS_INVALID_PARAM;
		} else {
			cfg_response_buf[2] = param_id;
			cfg_response_buf[3] = (uint8_t)ret;
			memcpy(&cfg_response_buf[4], value_buf, ret);
			LOG_HEXDUMP_INF(value_buf, ret, "CFG: READ_PARAM value");
		}
		break;
	}

	case CFG_CMD_WRITE_PARAM: {
		if (data == NULL || data_len == 0) {
			LOG_ERR("CFG: WRITE_PARAM invalid: data=%p, data_len=%u", data, data_len);
			cfg_response_buf[1] = CFG_STATUS_INVALID_LEN;
		} else {
			LOG_HEXDUMP_INF(data, MIN(data_len, 16), "CFG: WRITE_PARAM data");
			int ret = cfg_set_value(param_id, data, data_len);
			if (ret < 0) {
				LOG_ERR("CFG: WRITE_PARAM #%u failed: %d", param_id, ret);
				cfg_response_buf[1] = CFG_STATUS_INVALID_PARAM;
			} else {
				cfg_response_buf[2] = param_id;
				LOG_INF("CFG: Param #%u updated OK", param_id);
				cfg_dump_values();
			}
		}
		break;
	}

	case CFG_CMD_SAVE_ALL: {
		LOG_INF("CFG: SAVE_ALL command");
		int ret = cfg_save_all();
		if (ret < 0) {
			LOG_ERR("CFG: SAVE_ALL failed: %d", ret);
			cfg_response_buf[1] = CFG_STATUS_ERROR;
		}
		break;
	}

	case CFG_CMD_LOAD_ALL:
		LOG_INF("CFG: LOAD_ALL command");
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			settings_load_subtree("cfg");
			LOG_INF("CFG: Parameters reloaded from flash");
			cfg_dump_values();
		}
		break;

	case CFG_CMD_LIST_PARAMS: {
		LOG_INF("CFG: LIST_PARAMS command");
		int ret = cfg_build_list_response(&cfg_response_buf[4],
						  CFG_REPORT_SIZE - 4);
		cfg_response_buf[3] = (uint8_t)ret;
		LOG_INF("CFG: LIST_PARAMS returning %u bytes", ret);
		break;
	}

	default:
		cfg_response_buf[1] = CFG_STATUS_UNKNOWN_CMD;
		LOG_WRN("CFG: Unknown command: 0x%02x", cmd);
		break;
	}

	cfg_response_pending = true;
	return 0;
}

struct hid_device_ops cfg_ops = {
	.iface_ready = cfg_iface_ready,
	.get_report  = cfg_get_report,
	.set_report  = cfg_set_report,
};

/* ============================================================
 *  BLE Central - HID 设备发现与连接
 * ============================================================ */

/* BLE 连接对象 */
static struct bt_conn *ble_conn;
/* 是否已订阅鼠标通知 */
static bool mouse_subscribed;
/* HID Control Point 句柄（用于 Exit Suspend） */
static uint16_t control_point_handle;
/* 扫描超时计时器 */
static struct k_work_delayable scan_timeout_work;
/* 自动重连超时计时器：Auto Connect 15秒内未连上则回退到普通扫描 */
static struct k_work_delayable auto_connect_timeout_work;
/* 延迟自动重连工作项：断连后延迟500ms启动，给HCI堆栈时间清理资源 */
static struct k_work_delayable delayed_auto_connect_work;
/* 连接状态监测定时器 */
static struct k_work_delayable conn_status_work;

/* 最多支持的 Report 订阅数量 */
#define MAX_REPORT_SUBS 4

/* 手动 GATT 订阅参数（必须保持有效直到取消订阅） */
static struct bt_gatt_subscribe_params mouse_sub_params;
/* 备用订阅参数数组：用于普通 Report (0x2A4D) 特征值 */
static struct bt_gatt_subscribe_params report_sub_params[MAX_REPORT_SUBS];
/* 当前使用的 report_sub_params 索引 */
static int report_sub_idx;

/* 普通 Report (0x2A4D) 句柄记录 */
static struct {
	uint16_t value_handle;
	uint16_t ccc_handle;
} report_handles[MAX_REPORT_SUBS];
static int report_handles_count;

/* HID Report Map 读取 */
#define HID_REPORT_MAP_MAX_SIZE 256
static uint8_t report_map_data[HID_REPORT_MAP_MAX_SIZE];
static uint16_t report_map_len;
static struct bt_gatt_read_params report_map_read_params;

/* ---------- 前向声明 ---------- */
static void start_ble_scan(void);

/* ---------- HID Report Map 读取 ---------- */

/* Report Map 读取完成回调 */
static uint8_t report_map_read_cb(struct bt_conn *conn, uint8_t err,
				  struct bt_gatt_read_params *params,
				  const void *data, uint16_t length)
{
	if (err) {
		LOG_WRN("Report Map read error: %d", err);
		return BT_GATT_ITER_STOP;
	}

	/* 累积数据（可能分多次回调） */
	if (data && length > 0) {
		if (report_map_len + length > HID_REPORT_MAP_MAX_SIZE) {
			LOG_WRN("Report Map too large, truncating");
			length = HID_REPORT_MAP_MAX_SIZE - report_map_len;
		}
		memcpy(report_map_data + report_map_len, data, length);
		report_map_len += length;
		LOG_INF("Report Map chunk: %u bytes (total %u)", length, report_map_len);
		return BT_GATT_ITER_CONTINUE;
	}

	/* 读取完成，打印 Report Map */
	LOG_INF("=== HID Report Map (%u bytes) ===", report_map_len);
	for (uint16_t i = 0; i < report_map_len; i += 16) {
		char hex[64], ascii[20];
		int pos = 0, apos = 0;
		for (uint16_t j = i; j < i + 16 && j < report_map_len; j++) {
			pos += snprintf(hex + pos, sizeof(hex) - pos,
					"%02x ", report_map_data[j]);
			apos += snprintf(ascii + apos, sizeof(ascii) - apos,
					 "%c", (report_map_data[j] >= 32 && report_map_data[j] < 127)
					       ? report_map_data[j] : '.');
		}
		LOG_INF("  %04x: %-48s %s", i, hex, ascii);
	}

	/* TODO: 解析 Report Map 中 Input Report 格式，自动确定 X/Y 偏移 */

	return BT_GATT_ITER_STOP;
}

/* ---------- GATT 订阅通知回调 ---------- */

static uint8_t mouse_notify_cb(struct bt_conn *conn,
			       struct bt_gatt_subscribe_params *params,
			       const void *data, uint16_t length)
{
	if (data == NULL) {
		LOG_INF("Mouse unsubscribed (handle 0x%04x)",
			params->value_handle);
		mouse_subscribed = false;
		return BT_GATT_ITER_STOP;
	}

	LOG_INF("BLE mouse report received: %u bytes from handle 0x%04x",
		length, params->value_handle);

	/* 将 BLE 鼠标数据转发到 USB HID */
	ble_mouse_report_forward(data, length);
	return BT_GATT_ITER_CONTINUE;
}

/* 连接状态周期性检查：每5秒打印一次连接状态 */
static void conn_status_handler(struct k_work *work)
{
	if (ble_conn) {
		struct bt_conn_info info;
		int err = bt_conn_get_info(ble_conn, &info);
		if (err == 0) {
			LOG_INF("--- Connection alive: state=%d, sec_level=%d, "
				"subscribed=%d ---",
				info.state, info.security.level,
				mouse_subscribed);
		}
		/* 继续每5秒检查一次 */
		k_work_schedule(&conn_status_work, K_SECONDS(5));
	}
}

/* ---------- GATT Discovery Manager 回调 ---------- */

/* 打印发现的属性 - 调试用 */
static void print_discovered_attrs(struct bt_gatt_dm *dm)
{
	const struct bt_gatt_dm_attr *attr = NULL;

	LOG_INF("--- Discovered HID Service attributes ---");
	/* 先打印度服务 */
	attr = bt_gatt_dm_service_get(dm);
	if (attr) {
		char uuid_str[BT_UUID_STR_LEN];
		bt_uuid_to_str(attr->uuid, uuid_str, sizeof(uuid_str));
		LOG_INF("  Service: %s handle=0x%04x", uuid_str, attr->handle);
	}

	/* 遍历所有特征值和描述符 */
	attr = NULL;
	while ((attr = bt_gatt_dm_attr_next(dm, attr)) != NULL) {
		char uuid_str[BT_UUID_STR_LEN];
		bt_uuid_to_str(attr->uuid, uuid_str, sizeof(uuid_str));
		LOG_INF("  Attr: %s handle=0x%04x perm=0x%02x",
			uuid_str, attr->handle, attr->perm);
	}
	LOG_INF("--- End of attributes ---");
}

/* 通过 WWoR 写入 CCC 并调用 bt_gatt_resubscribe 订阅
 *
 * bt_gatt_subscribe() 内部使用 Write Request（带响应），
 * 鼠标会返回 "Insufficient Authentication" 错误，
 * 导致 Zephyr 自动触发安全升级 -> 配对失败 -> 断开连接。
 *
 * 解决方案：
 * 1. bt_gatt_write_without_response() 写 CCC 0x0001
 *    (Write Command，无响应，不触发安全升级)
 * 2. bt_gatt_resubscribe() 注册通知回调
 *    (只添加到订阅列表，不写 CCC)
 */
static void subscribe_char_via_wwor(uint16_t value_handle, uint16_t ccc_handle,
				    struct bt_gatt_subscribe_params *params,
				    const char *desc)
{
	int err;
	uint16_t ccc_value = BT_GATT_CCC_NOTIFY;

	LOG_INF("Subscribing to %s: value=0x%04x ccc=0x%04x",
		desc, value_handle, ccc_handle);

	/* 第1步：Write Without Response 写 CCC */
	err = bt_gatt_write_without_response(ble_conn, ccc_handle,
					    &ccc_value, sizeof(ccc_value),
					    false);
	if (err) {
		LOG_ERR("  CCC WWoR failed for %s: %d", desc, err);
	} else {
		LOG_INF("  CCC WWoR queued for %s", desc);
	}

	/* 第2步：bt_gatt_resubscribe 注册回调
	 *
	 * 注意：params 必须是静态或长期有效的变量（不能是栈变量），
	 * 且每个不同的订阅必须使用不同的 params 指针。
	 * 这里不调用 memset，因为 params 是静态变量，初始为全零。
	 */
	const bt_addr_le_t *peer = bt_conn_get_dst(ble_conn);
	params->notify = mouse_notify_cb;
	params->value = BT_GATT_CCC_NOTIFY;
	params->value_handle = value_handle;
	params->ccc_handle = ccc_handle;
	params->min_security = BT_SECURITY_L1;

	err = bt_gatt_resubscribe(BT_ID_DEFAULT, peer, params);
	if (err) {
		LOG_ERR("  bt_gatt_resubscribe failed for %s: %d", desc, err);
	} else {
		mouse_subscribed = true;
		LOG_INF("  Subscribed to %s (WWoR + resubscribe)", desc);
	}
}

static void discovery_completed(struct bt_gatt_dm *dm, void *context)
{
	LOG_INF("GATT discovery completed");

	/* 打印发现的所有属性 */
	print_discovered_attrs(dm);

	/* 清除计数 */
	report_handles_count = 0;

	/* ---- 第1步：订阅 Boot Mouse Input Report (0x2A4B) ---- */
	const struct bt_gatt_dm_attr *boot_mouse_chrc;
	boot_mouse_chrc = bt_gatt_dm_char_by_uuid(dm,
						BT_UUID_HIDS_BOOT_MOUSE_IN_REPORT);
	if (boot_mouse_chrc) {
		uint16_t value_handle =
			bt_gatt_dm_attr_chrc_val(boot_mouse_chrc)->value_handle;
		LOG_INF("Boot Mouse Input Report FOUND: value_handle=0x%04x",
			value_handle);

		const struct bt_gatt_dm_attr *ccc =
			bt_gatt_dm_desc_by_uuid(dm, boot_mouse_chrc,
						BT_UUID_GATT_CCC);
		if (ccc) {
			subscribe_char_via_wwor(value_handle, ccc->handle,
						&mouse_sub_params,
						"Boot Mouse (0x2A4B)");
		} else {
			LOG_WRN("No CCC for Boot Mouse");
		}
	} else {
		LOG_WRN("Boot Mouse Input Report NOT FOUND");
	}

	/* ---- 第2步：遍历所有 Report (0x2A4D) 特征值并订阅 ----
	 * 罗技鼠标可能通过普通 Report 而非 Boot Mouse 发送数据
	 */
	report_sub_idx = 0;
	const struct bt_gatt_dm_attr *attr = NULL;
	while ((attr = bt_gatt_dm_attr_next(dm, attr)) != NULL) {
		/* 查找 Characteristic 声明 (2803) */
		if (attr->uuid->type != BT_UUID_TYPE_16) {
			continue;
		}
		const struct bt_uuid_16 *u16 = CONTAINER_OF(
			attr->uuid, const struct bt_uuid_16, uuid);
		if (u16->val != BT_UUID_GATT_CHRC_VAL) {
			continue;
		}

		/* 获取特征值 UUID */
		struct bt_gatt_chrc *chrc_val =
			bt_gatt_dm_attr_chrc_val(attr);
		if (chrc_val->uuid->type != BT_UUID_TYPE_16) {
			continue;
		}
		const struct bt_uuid_16 *chrc_u16 = CONTAINER_OF(
			chrc_val->uuid, const struct bt_uuid_16, uuid);

		/* 检查是否为 HID Control Point (0x2A4C) */
		if (chrc_u16->val == BT_UUID_HIDS_CTRL_POINT_VAL) {
			control_point_handle = chrc_val->value_handle;
			LOG_INF("  Control Point (0x2A4C) FOUND: value_handle=0x%04x",
				control_point_handle);
			continue;
		}

		/* 只处理 Report 特征值 (0x2A4D) */
		if (chrc_u16->val != BT_UUID_HIDS_REPORT_VAL) {
			continue;
		}

		uint16_t value_handle = chrc_val->value_handle;

		/* 查找该特征值对应的 CCC */
		const struct bt_gatt_dm_attr *ccc =
			bt_gatt_dm_desc_by_uuid(dm, attr, BT_UUID_GATT_CCC);
		if (!ccc) {
			LOG_INF("  Report (0x2A4D) value=0x%04x: no CCC, skip",
				value_handle);
			continue;
		}

		/* 保存句柄供后续使用 */
		if (report_handles_count < MAX_REPORT_SUBS) {
			report_handles[report_handles_count].value_handle =
				value_handle;
			report_handles[report_handles_count].ccc_handle =
				ccc->handle;
			report_handles_count++;
		}

		/* 只订阅 Report 特征值（不是 Boot Mouse 也不是 Control Point）*/
		if (report_sub_idx < MAX_REPORT_SUBS) {
			char desc[32];
			snprintf(desc, sizeof(desc),
				 "Report %d (0x2A4D)", report_sub_idx + 1);
			subscribe_char_via_wwor(value_handle, ccc->handle,
						&report_sub_params[report_sub_idx],
						desc);
			report_sub_idx++;
		}
	}

	/* ---- 第3步：写入 HID Control Point "Exit Suspend" (0x01) via WWoR ----
		* 如果鼠标之前处于 Suspend 状态，写 Exit Suspend 可以唤醒它
		* 使鼠标开始发送 HID 报告
		*/
	if (control_point_handle) {
		uint8_t cp_value = 0x01; /* Exit Suspend */
		LOG_INF("Writing Control Point (0x2A4C) Exit Suspend=0x%02x "
			"via WWoR...", cp_value);
		int err = bt_gatt_write_without_response(ble_conn,
					control_point_handle,
					&cp_value, sizeof(cp_value), false);
		if (err) {
			LOG_ERR("  Control Point WWoR failed: %d", err);
		} else {
			LOG_INF("  Control Point Exit Suspend queued");
		}
	} else {
		LOG_INF("No Control Point found, skip");
	}

	/* ---- 第4步：读取 HID Report Map (0x2A4B) ----
	 * Report Map 包含 HID Report Descriptor，定义了鼠标报告的精确格式
	 * （如 X/Y 是 8-bit 还是 16-bit，字节偏移等）
	 */
	const struct bt_gatt_dm_attr *report_map_attr;
	report_map_attr = bt_gatt_dm_char_by_uuid(dm, BT_UUID_HIDS_REPORT_MAP);
	if (report_map_attr) {
		uint16_t report_map_handle =
			bt_gatt_dm_attr_chrc_val(report_map_attr)->value_handle;
		LOG_INF("Report Map (0x2A4B) FOUND: value_handle=0x%04x",
			report_map_handle);

		/* 准备读取参数 */
		report_map_len = 0;
		report_map_read_params.func = report_map_read_cb;
		report_map_read_params.handle_count = 1;
		report_map_read_params.single.handle = report_map_handle;
		report_map_read_params.single.offset = 0;

		int err = bt_gatt_read(ble_conn, &report_map_read_params);
		if (err) {
			LOG_ERR("  Report Map read failed: %d", err);
		} else {
			LOG_INF("  Report Map read initiated...");
		}
	} else {
		LOG_WRN("Report Map (0x2A4B) NOT FOUND");
	}

	/* 释放发现数据 */
	bt_gatt_dm_data_release(dm);

	/* 启动连接状态监测 */
	k_work_schedule(&conn_status_work, K_SECONDS(5));
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	LOG_WRN("HID Service not found on this device, disconnecting...");
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("GATT discovery error: %d", err);
	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

/* GATT DM 回调结构体 */
static const struct bt_gatt_dm_cb dm_cb = {
	.completed = discovery_completed,
	.service_not_found = discovery_service_not_found,
	.error_found = discovery_error,
};

/* ---------- 配对/认证回调 ---------- */

static void pairing_confirm(struct bt_conn *conn)
{
	LOG_INF("Pairing confirm - auto accepting");
	bt_conn_auth_pairing_confirm(conn);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing complete: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Pairing FAILED: %s, reason %d %s", addr, reason,
		bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb auth_callbacks = {
	.pairing_confirm = pairing_confirm,
	.cancel = auth_cancel,
};

/* ---------- Accept List（白名单）回调 ---------- */

/* 从 Flash 恢复绑定设备到 Accept List */
static void bond_restore_cb(const struct bt_bond_info *info, void *user_data)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(&info->addr, addr, sizeof(addr));

	LOG_INF("Restoring bonded device: %s", addr);

	/* 保存地址 */
	bt_addr_le_copy(&mouse_bond_addr, &info->addr);
	mouse_bond_addr_valid = true;

	/* 加入 Accept List */
	int err = bt_le_filter_accept_list_add(&info->addr);
	if (err) {
		LOG_ERR("  accept list add FAILED: %d", err);
	} else {
		LOG_INF("  Added to filter accept list");
	}
}

static struct bt_conn_auth_info_cb auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* ---------- BLE 安全/加密状态变化 ---------- */

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("=== Security: %s, level %u, FAILED: err %d %s ===\n",
		       addr, level, err, bt_security_err_to_str(err));
	} else {
		printk("=== Security changed: %s, level %u ===\n", addr, level);
	}

	/* 无论安全是否成功，都尝试 GATT 发现（同 central_hids 官方做法） */
	printk("=== Starting GATT discovery ===\n");
	int ret = bt_gatt_dm_start(conn, BT_UUID_HIDS, &dm_cb, NULL);
	if (ret) {
		LOG_ERR("GATT DM start failed: %d", ret);
	}
}

/* ---------- BLE 连接回调 ---------- */

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_ERR("Failed to connect to %s: %u", addr, conn_err);
		bt_conn_unref(ble_conn);
		ble_conn = NULL;
		/* 连接失败，如果有绑定则启动自动重连，否则回退到扫描 */
		if (mouse_bond_addr_valid) {
			LOG_INF("Connection failed, starting auto reconnect...");
			bt_conn_le_create_auto(BT_CONN_LE_CREATE_CONN_AUTO,
					       BT_LE_CONN_PARAM_DEFAULT);
		} else {
			k_work_schedule(&scan_timeout_work, K_SECONDS(3));
		}
		return;
	}

	printk("=== Connected: %s ===\n", addr);
	control_point_handle = 0;

	/* 停止自动重连（已建立连接，不再需要硬件自动扫描） */
	bt_conn_create_auto_stop();
	/* 取消所有自动重连相关的定时器 */
	k_work_cancel_delayable(&auto_connect_timeout_work);
	k_work_cancel_delayable(&delayed_auto_connect_work);

	/* 将已连接的鼠标地址加入 Accept List（以备下次自动重连）*/
	if (!mouse_bond_addr_valid) {
		bt_addr_le_copy(&mouse_bond_addr, bt_conn_get_dst(conn));
		mouse_bond_addr_valid = true;
	}
	int al_err = bt_le_filter_accept_list_add(&mouse_bond_addr);
	if (al_err) {
		LOG_ERR("Accept List add failed in connected(): %d", al_err);
	} else {
		LOG_INF("Added %s to Accept List in connected()", addr);
	}

	/* 直接调用 bt_conn_set_security，同 central_hids 官方做法 */
	int err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		printk("=== bt_conn_set_security failed: %d, starting GATT DM directly ===\n",
		       err);
		int ret = bt_gatt_dm_start(conn, BT_UUID_HIDS, &dm_cb, NULL);
		if (ret) {
			LOG_ERR("GATT DM start failed: %d", ret);
		}
	} else {
		printk("=== bt_conn_set_security OK, waiting for security_changed ===\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_WRN("Disconnected: %s, reason 0x%02x %s",
		addr, reason, bt_hci_err_to_str(reason));

	if (ble_conn != conn) {
		return;
	}

	bt_conn_unref(ble_conn);
	ble_conn = NULL;
	mouse_subscribed = false;

	/* 取消所有定时器 */
	k_work_cancel_delayable(&conn_status_work);
	k_work_cancel_delayable(&delayed_auto_connect_work);

	/* 处理连接建立失败（reason 0x3e）：
	 *
	 * M720 每次切换到同一设备槽时，可能使用不同的随机地址重连
	 * （如 FF:1D → FF:07），但在同一会话中地址保持不变。
	 *
	 * M720 自身存储了设备槽 3 的绑定信息（来自第一次 FF:1D 配对），
	 * 重连时 M720 可能立即用存储的 LTK 发起加密请求。
	 * 但我们的主机没有新地址（FF:07）的绑定（绑定的是旧地址 FF:1D），
	 * 控制器找不到 LTK → 0x3e（连接建立失败）。
	 *
	 * 注意：不要在这里调用 bt_unpair()！M720 自身仍有绑定，
	 * 删除我们的绑定只会让 M720 拒绝新配对（err 9 = Pair Not Supported）。
	 * 正确的做法是启用 RPA 解析（CONFIG_BT_PRIVACY），
	 * 让 nRF 能将不同随机地址解析到同一身份地址。
	 *
	 * 这里只清除 Accept List（对 M720 的随机地址无意义），
	 * 然后重新扫描，MAC 前缀匹配会再次发现 M720。
	 */
	if (reason == BT_HCI_ERR_CONN_FAIL_TO_ESTAB) {
		LOG_WRN("0x3e: clearing Accept List, keeping bond (M720 has stored bond)");
		bt_le_filter_accept_list_clear();
		/* 不调用 bt_unpair()！保留绑定，让 RPA 解析能找到 LTK */
	}

	/* M720 使用不同的随机地址（每次连接地址都不同），
	 * Accept List + Auto Connect 无法匹配新地址。
	 * 因此直接回退到普通扫描（MAC 前缀匹配），能发现任何 M720 地址。
	 *
	 * 延迟 500ms 是必要的：disconnected() 在 HCI 事件上下文中调用，
	 * HCI 命令缓冲池还未释放完毕，立即启动扫描可能失败。
	 */
	if (mouse_bond_addr_valid) {
		LOG_INF("Disconnected, restarting BLE scan in 500ms...");
		k_work_schedule(&scan_timeout_work, K_MSEC(500));
	} else {
		/* 第一次使用，没有绑定，回退到普通扫描 */
		LOG_INF("No bonded device or MIC failure, restarting BLE scan in 500ms...");
		k_work_schedule(&scan_timeout_work, K_MSEC(500));
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

/* ---------- 扫描超时/重启 ---------- */

static void scan_timeout_handler(struct k_work *work)
{
	start_ble_scan();
}

/* 自动重连超时处理：如果在规定时间内未连接成功，停止自动重连并回退到普通扫描 */
static void auto_connect_timeout_handler(struct k_work *work)
{
	LOG_INF("Auto connect timeout, stopping and falling back to BLE scan...");
	bt_conn_create_auto_stop();
	start_ble_scan();
}

/* 延迟自动重连处理：断连后等待 HCI 资源清理完毕，再启动硬件自动重连 */
static void delayed_auto_connect_handler(struct k_work *work)
{
	LOG_INF("Auto connect starting now...");
	int err = bt_conn_le_create_auto(BT_CONN_LE_CREATE_CONN_AUTO,
					BT_LE_CONN_PARAM_DEFAULT);
	if (err) {
		LOG_ERR("Auto connect failed: %d, fallback to scan", err);
		start_ble_scan();
	} else {
		/* 自动重连启动成功，设置 15 秒后备超时 */
		k_work_schedule(&auto_connect_timeout_work, K_SECONDS(15));
	}
}

/* ---------- BLE 扫描 ---------- */
/*
 * 注意：许多 BLE 鼠标在配对模式下不广播 HID Service UUID (0x1812)，
 * 只广播设备名称和 Appearance(外观)。
 * 因此我们不能只靠 UUID 过滤，也需要检查 Appearance。
 */


static bool eir_check_hid_mouse(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;

	switch (data->type) {
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		/* 检查是否包含 HID Service UUID (0x1812) */
		if (data->data_len % sizeof(uint16_t) != 0U) {
			return true;
		}
		for (i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			uint16_t u16;
			memcpy(&u16, &data->data[i], sizeof(u16));
			if (sys_le16_to_cpu(u16) == BT_UUID_HIDS_VAL) {
				LOG_INF("HID Service UUID found, connecting...");
				goto do_connect;
			}
		}
		break;

	case BT_DATA_GAP_APPEARANCE:
		/* 检查 Appearance 是否为鼠标 (0x03C2=962) */
		if (data->data_len >= sizeof(uint16_t)) {
			uint16_t appearance;
			memcpy(&appearance, data->data, sizeof(uint16_t));
			if (sys_le16_to_cpu(appearance) == BT_APPEARANCE_HID_MOUSE) {
				LOG_INF("Mouse appearance found, connecting...");
				goto do_connect;
			}
		}
		break;

	default:
		break;
	}

	return true;

do_connect:
	bt_le_scan_stop();

	int err = bt_conn_le_create(addr,
		BT_CONN_LE_CREATE_CONN,
		BT_LE_CONN_PARAM_DEFAULT, &ble_conn);
	if (err) {
		LOG_ERR("Create conn failed: %d", err);
		/* 3秒后重试扫描 */
		k_work_schedule(&scan_timeout_work, K_SECONDS(3));
	}
	return false;
}

/* M720 的 MAC 前缀（BT 地址存储为小端序，val[0]=最低字节）
 * MAC 字符串: FB:DE:BD:46:FF:0C
 * val[]     : { 0x0C, 0xFF, 0x46, 0xBD, 0xDE, 0xFB }
 * 前5字节前缀: FB:DE:BD:46:FF 对应 val[5..1]
 */
static const uint8_t target_mac_prefix[5] = { 0xFF, 0x46, 0xBD, 0xDE, 0xFB };

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));

	/* RSSI 过滤：信号不够强就不处理 */
	if (rssi < BLE_HID_RSSI_THRESHOLD) {
		return;
	}

	/* 信号够强才打印日志 */
	LOG_INF("DEVICE: %s, RSSI %d, type %u", dev, rssi, type);

	/* 只处理可连接广播 */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
	    type != BT_GAP_ADV_TYPE_EXT_ADV) {
		LOG_DBG("Not connectable type: %u", type);
		return;
	}

	/* 检查是否已经连接或正在连接 */
	if (ble_conn != NULL) {
		LOG_DBG("Already connected, skip");
		return;
	}

	/* 已有绑定地址时：只精确匹配绑定的地址，防止连到其他设备槽。
	 *
	 * M720 有 3 个设备槽，每个槽有独立的随机地址和绑定信息。
	 * 例如：设备槽2(FF:07)→电脑，设备槽3(FF:1D)→开发板。
	 * 断开后如果扫描到设备2的广播（FF:07），MAC前缀也匹配，
	 * 但连接过去会因密钥不匹配导致 0x3e。
	 *
	 * 精确匹配确保只连回之前绑定的那个设备槽。
	 */
	if (mouse_bond_addr_valid) {
		/* 只连接已绑定的精确地址 */
		if (bt_addr_le_cmp(addr, &mouse_bond_addr) == 0) {
			LOG_INF("*** Bonded device found: %s! ***", dev);
			bt_data_parse(ad, eir_check_hid_mouse, (void *)addr);
		} else {
			LOG_DBG("Not bonded address (%s), skip", dev);
		}
	} else {
		/* 首次连接：使用 MAC 前缀匹配发现 M720 */
		if (memcmp(&addr->a.val[1], target_mac_prefix, 5) == 0) {
			LOG_INF("*** MAC prefix matched %s! Checking EIR data... ***", dev);
			bt_data_parse(ad, eir_check_hid_mouse, (void *)addr);
		} else {
			LOG_DBG("MAC prefix mismatch");
		}
	}
}

static void start_ble_scan(void)
{
	int err;

	/* 如果已连接，不扫描 */
	if (ble_conn != NULL) {
		return;
	}

	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		LOG_ERR("BLE scan start failed: %d", err);
		return;
	}

	LOG_INF("BLE scanning for HID mouse (RSSI >= %d dBm)...",
		BLE_HID_RSSI_THRESHOLD);
}

/* ---------- USB 消息回调 ---------- */

static void usbd_msg_cb(struct usbd_context *const usbd_ctx,
			const struct usbd_msg *const msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (msg->type == USBD_MSG_CONFIGURATION) {
		LOG_INF("\tConfiguration value %d", msg->status);
	}

	if (usbd_can_detect_vbus(usbd_ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			int ret = usbd_enable(usbd_ctx);
			if (ret) {
				LOG_ERR("Failed to enable USB device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(usbd_ctx)) {
				LOG_ERR("Failed to disable USB device support");
			}
		}
	}
}

/* ============================================================
 *  主函数
 * ============================================================ */
int main(void)
{
	struct usbd_context *sample_usbd;
	const struct device *mouse_dev;
	const struct device *kb_dev;
	const struct device *cfg_dev;
	int ret;

	/* ---- 第1步：获取 HID 设备 ---- */
	mouse_dev = DEVICE_DT_GET(DT_NODELABEL(hid_dev_0));
	if (!device_is_ready(mouse_dev)) {
		LOG_ERR("Mouse HID device not ready");
		return -EIO;
	}

	kb_dev = DEVICE_DT_GET(DT_NODELABEL(hid_dev_1));
	if (!device_is_ready(kb_dev)) {
		LOG_ERR("Keyboard HID device not ready");
		return -EIO;
	}

	cfg_dev = DEVICE_DT_GET(DT_NODELABEL(hid_dev_2));
	if (!device_is_ready(cfg_dev)) {
		LOG_ERR("Config HID device not ready");
		return -EIO;
	}

	/* ---- 第2步：注册 HID 设备 ---- */
	ret = hid_device_register(mouse_dev,
				  hid_mouse_report_desc,
				  sizeof(hid_mouse_report_desc),
				  &mouse_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register mouse HID, %d", ret);
		return ret;
	}

	ret = hid_device_register(kb_dev,
				  hid_keyboard_report_desc,
				  sizeof(hid_keyboard_report_desc),
				  &kb_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register keyboard HID, %d", ret);
		return ret;
	}

	ret = hid_device_register(cfg_dev,
				  hid_config_report_desc,
				  sizeof(hid_config_report_desc),
				  &cfg_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register config HID, %d", ret);
		return ret;
	}

	/* ---- 第3步：初始化 USB 设备栈（注册所有类、分配描述符等） ---- */
	sample_usbd = sample_usbd_init_device(usbd_msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	/* ---- 第4步：启用 USB ---- */
	if (!usbd_can_detect_vbus(sample_usbd)) {
		ret = usbd_enable(sample_usbd);
		if (ret) {
			LOG_ERR("Failed to enable USB device support");
			return ret;
		}
	}
	/* 如果硬件支持 VBUS 检测，usbd_enable() 将在 VBUS 就绪后由回调调用 */

	LOG_DBG("USB initialized");

#if 0
	/* ---- 第5步：创建键盘自动打字任务（调试用，暂时关闭） ---- */
	k_thread_create(&auto_keyboard_thread, auto_keyboard_stack,
			K_THREAD_STACK_SIZEOF(auto_keyboard_stack),
			auto_keyboard_task, NULL, NULL, NULL,
			5, 0, K_NO_WAIT);
#endif

	/* ---- 第6步：初始化工作队列 ---- */
	k_work_init_delayable(&scan_timeout_work, scan_timeout_handler);
	k_work_init_delayable(&conn_status_work, conn_status_handler);
	k_work_init_delayable(&auto_connect_timeout_work, auto_connect_timeout_handler);
	k_work_init_delayable(&delayed_auto_connect_work, delayed_auto_connect_handler);

	/* ---- 第7步：初始化 BLE（失败不影响 USB HID 功能） ---- */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("BLE init failed: %d, USB HID will still work", ret);
	} else {
		LOG_INF("BLE initialized");

		/* 加载持久化设置（备用） */
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			settings_load();
		}

		/* 注册配对/认证回调 */
		{
			int cb_err = bt_conn_auth_cb_register(&auth_callbacks);
			if (cb_err) {
				LOG_ERR("Failed to register auth callbacks: %d", cb_err);
			}
		}
		{
			int cb_err = bt_conn_auth_info_cb_register(&auth_info_callbacks);
			if (cb_err) {
				LOG_ERR("Failed to register auth info callbacks: %d", cb_err);
			}
		}

		LOG_INF("Pairing callbacks registered (NoInputNoOutput + Just Works). "
			"Will try security L2, fallback to WWoR");

		/* 从 Flash 恢复绑定设备到 Accept List（白名单）*/
		mouse_bond_addr_valid = false;
		bt_foreach_bond(BT_ID_DEFAULT, bond_restore_cb, NULL);

		if (mouse_bond_addr_valid) {
			/* 有绑定设备，启动硬件级自动重连（Accept List + Auto Connect）
			 * BLE 控制器自动扫描定向广播（ADV_DIRECT_IND），
			 * 发现鼠标后自动发起连接，不需要软件扫描
			 *
			 * 同时设置 3 秒超时：如果自动重连超时未连上，
			 * 停止自动重连并回退到普通扫描模式。
			 * Auto Connect 只对使用相同地址的定向广播有效，
			 * M720 每次地址不同，大概率会超时回退到扫描。
			 */
			LOG_INF("Bonded mouse found, starting auto connect (3s timeout)...");
			int err = bt_conn_le_create_auto(BT_CONN_LE_CREATE_CONN_AUTO,
			 			BT_LE_CONN_PARAM_DEFAULT);
			if (err) {
			 LOG_ERR("Auto connect failed: %d, fallback to scan", err);
			 start_ble_scan();
			} else {
			 /* 自动重连启动成功，设置 3 秒后备超时 */
			 k_work_schedule(&auto_connect_timeout_work, K_SECONDS(3));
			}
		} else {
			/* 没有绑定设备（第一次使用），启动普通 BLE 扫描 */
			LOG_INF("No bonded device, starting BLE scan...");
			start_ble_scan();
		}
	}

	/* ---- 第8步：主循环，处理鼠标和键盘事件 ---- */
	while (true) {
		bool has_data = false;

		/* 检查鼠标事件 */
		UDC_STATIC_BUF_DEFINE(mouse_report, MOUSE_REPORT_COUNT);
		if (k_msgq_get(&mouse_msgq, &mouse_report, K_NO_WAIT) == 0) {
			if (mouse_ready) {
				ret = hid_device_submit_report(mouse_dev,
					MOUSE_REPORT_COUNT, mouse_report);
				if (ret) {
					LOG_ERR("Mouse report error, %d", ret);
				}
			} else {
				LOG_DBG("Mouse USB not ready, drop report");
			}
			has_data = true;
		}

		/* 检查键盘事件 */
		UDC_STATIC_BUF_DEFINE(kb_report, KB_REPORT_COUNT);
		if (k_msgq_get(&kb_msgq, &kb_report, K_NO_WAIT) == 0) {
			if (kb_ready) {
				ret = hid_device_submit_report(kb_dev,
					KB_REPORT_COUNT, kb_report);
				if (ret) {
					LOG_ERR("KB report error, %d", ret);
				}
			} else {
				LOG_DBG("KB USB not ready, drop report");
			}
			has_data = true;
		}

		/* 如果没有数据，等10ms再检查 */
		if (!has_data) {
			k_msleep(10);
		}
	}

	return 0;
}
