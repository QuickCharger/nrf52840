/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sample_usbd.h>

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

#define MOUSE_BTN_LEFT		0
#define MOUSE_BTN_RIGHT		1

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

K_MSGQ_DEFINE(mouse_msgq, MOUSE_REPORT_COUNT, 10, 1);
static bool mouse_ready;

/* 自动移动鼠标的任务：每0.1秒向右移动10像素 */
void auto_mouse_task(void *arg1, void *arg2, void *arg3)
{
	static uint8_t report[MOUSE_REPORT_COUNT] = {0};

	while (1) {
		/* 设置移动量：X轴移动10像素 */
		report[MOUSE_X_REPORT_IDX] = 10;
		report[MOUSE_Y_REPORT_IDX] = 0;

		/* 发送报告到消息队列 */
		if (k_msgq_put(&mouse_msgq, report, K_NO_WAIT) != 0) {
			LOG_ERR("Failed to put mouse move event");
		} else {
			LOG_INF("Auto mouse move: X+10");
		}

		/* 每0.1秒执行一次 */
		k_msleep(100);
	}
}

K_THREAD_STACK_DEFINE(auto_mouse_stack, 1024);
static struct k_thread auto_mouse_thread;

static void mouse_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s",
		dev->name, ready ? "ready" : "not ready");
	mouse_ready = ready;
}

static int mouse_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
{
	LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);

	return 0;
}

struct hid_device_ops mouse_ops = {
	.iface_ready = mouse_iface_ready,
	.get_report = mouse_get_report,
};

int main(void)
{
	struct usbd_context *sample_usbd;
	const struct device *hid_dev;
	int ret;

	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	if (!device_is_ready(hid_dev)) {
		LOG_ERR("HID Device is not ready");
		return -EIO;
	}

	ret = hid_device_register(hid_dev,
				  hid_report_desc, sizeof(hid_report_desc),
				  &mouse_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	usbd_enable(sample_usbd);

	LOG_DBG("USB device support enabled");

	/* 创建自动移动鼠标的任务 */
	k_thread_create(&auto_mouse_thread, auto_mouse_stack,
			K_THREAD_STACK_SIZEOF(auto_mouse_stack),
			auto_mouse_task, NULL, NULL, NULL,
			5, 0, K_NO_WAIT);

	while (true) {
		UDC_STATIC_BUF_DEFINE(report, MOUSE_REPORT_COUNT);

		k_msgq_get(&mouse_msgq, &report, K_FOREVER);

		if (!mouse_ready) {
			LOG_INF("USB HID device is not ready");
			continue;
		}

		ret = hid_device_submit_report(hid_dev, MOUSE_REPORT_COUNT, report);
		if (ret) {
			LOG_ERR("HID submit report error, %d", ret);
		}
	}

	return 0;
}
