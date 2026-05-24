/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB HID Mouse + Keyboard 示例
 * - 鼠标每 0.1 秒向右移动 10 像素
 * - 键盘每 1 秒按一个字母，从 A 到 Z 循环
 */

#include <sample_usbd.h>

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ============================================================
 *  鼠标相关
 * ============================================================ */
static const uint8_t hid_mouse_report_desc[] = HID_MOUSE_REPORT_DESC(2);

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

K_MSGQ_DEFINE(mouse_msgq, MOUSE_REPORT_COUNT, 10, 1);
static bool mouse_ready;

/* 鼠标自动移动任务：每0.1秒向右移动10像素 */
void auto_mouse_task(void *arg1, void *arg2, void *arg3)
{
	static uint8_t report[MOUSE_REPORT_COUNT] = {0};

	while (1) {
		report[MOUSE_X_REPORT_IDX] = 10;
		report[MOUSE_Y_REPORT_IDX] = 0;

		if (k_msgq_put(&mouse_msgq, report, K_NO_WAIT) != 0) {
			LOG_ERR("Failed to put mouse move event");
		} else {
			LOG_INF("Auto mouse: X+10");
		}

		k_msleep(100);
	}
}

K_THREAD_STACK_DEFINE(auto_mouse_stack, 1024);
static struct k_thread auto_mouse_thread;

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
 *  键盘相关
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
static const uint8_t hid_keycodes[] = {
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
	0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
	0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
	0x1C, 0x1D
};

K_MSGQ_DEFINE(kb_msgq, KB_REPORT_COUNT, 10, 1);
static bool kb_ready;

/* 键盘自动打字任务：每1秒按一个字母，A-Z循环 */
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
 *  主函数
 * ============================================================ */
int main(void)
{
	struct usbd_context *sample_usbd;
	const struct device *mouse_dev;
	const struct device *kb_dev;
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

	/* ---- 第3步：初始化并启用 USB ---- */
	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	usbd_enable(sample_usbd);
	LOG_DBG("USB device support enabled");

	/* ---- 第4步：创建任务线程 ---- */
	k_thread_create(&auto_mouse_thread, auto_mouse_stack,
			K_THREAD_STACK_SIZEOF(auto_mouse_stack),
			auto_mouse_task, NULL, NULL, NULL,
			5, 0, K_NO_WAIT);

	k_thread_create(&auto_keyboard_thread, auto_keyboard_stack,
			K_THREAD_STACK_SIZEOF(auto_keyboard_stack),
			auto_keyboard_task, NULL, NULL, NULL,
			5, 0, K_NO_WAIT);

	/* ---- 第5步：主循环，处理鼠标和键盘事件 ---- */
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
