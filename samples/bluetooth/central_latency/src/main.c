/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Central that connects to the minimal latency peripheral.
 *
 *  Scans for a peripheral advertising the name PERIPHERAL_NAME and connects to
 *  it with a fixed 15 ms connection interval. After staying connected for 5
 *  seconds, it requests a peripheral latency of 199 (keeping the same interval
 *  and a 6080 ms supervision timeout). The link is encrypted automatically;
 *  subrating is not used.
 */

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/scan.h>

/* Advertised name of the peripheral to connect to. Must match the
 * peripheral's CONFIG_BT_DEVICE_NAME (the minimal power peripheral).
 */
#define PERIPHERAL_NAME "nordic_latency_test"

/* Connection interval in 1.25 ms units: 12 * 1.25 ms = 15 ms. */
#define CONN_INTERVAL 12

/* Supervision timeout in 10 ms units: 608 * 10 ms = 6080 ms. */
#define CONN_SUP_TIMEOUT 608

/* Peripheral latency used for the initial connection. */
#define INITIAL_LATENCY 0

/* Peripheral latency requested after the initial connected period.
 * Constraint check: (1 + 199) * 15 ms * 2 = 6000 ms < 6080 ms timeout. OK.
 */
#define TARGET_LATENCY 199

/* How long to stay connected before requesting the target peripheral latency. */
#define LATENCY_SWITCH_DELAY K_SECONDS(5)

static struct bt_conn *default_conn;
static struct k_work_delayable latency_update_work;

/* Initial connection parameters: 15 ms interval, no latency, 6080 ms timeout. */
static const struct bt_le_conn_param *init_conn_param =
	BT_LE_CONN_PARAM(CONN_INTERVAL, CONN_INTERVAL, INITIAL_LATENCY, CONN_SUP_TIMEOUT);

static void scan_start(void)
{
	int err;

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void latency_update_work_handler(struct k_work *work)
{
	int err;
	const struct bt_le_conn_param param =
		BT_LE_CONN_PARAM_INIT(CONN_INTERVAL, CONN_INTERVAL, TARGET_LATENCY,
				      CONN_SUP_TIMEOUT);

	ARG_UNUSED(work);

	if (!default_conn) {
		return;
	}

	err = bt_conn_le_param_update(default_conn, &param);
	if (err) {
		printk("Requesting peripheral latency %d failed (err %d)\n", TARGET_LATENCY, err);
	} else {
		printk("Requested peripheral latency %d\n", TARGET_LATENCY);
	}
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filter matched. Address: %s connectable: %d\n", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, NULL);

static void scan_init(void)
{
	int err;
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = 0x0010,
		.window = 0x0010,
	};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = true,
		.scan_param = &scan_param,
		.conn_param = init_conn_param,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, PERIPHERAL_NAME);
	if (err) {
		printk("Scanning filter cannot be set (err %d)\n", err);
		return;
	}

	err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Connection failed: %s, err 0x%02x %s\n", addr, err, bt_hci_err_to_str(err));
		scan_start();
		return;
	}

	default_conn = bt_conn_ref(conn);

	printk("Connected: %s (15 ms interval, latency %d, 6080 ms timeout)\n", addr,
	       INITIAL_LATENCY);

	/* Encrypt the link automatically. The peripheral also requests security,
	 * but driving it from the central makes the behavior deterministic.
	 */
	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		printk("Failed to set security (err %d)\n", err);
	}

	printk("Staying connected for 5 s before requesting peripheral latency %d\n",
	       TARGET_LATENCY);
	k_work_schedule(&latency_update_work, LATENCY_SWITCH_DELAY);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	k_work_cancel_delayable(&latency_update_work);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	scan_start();
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout)
{
	ARG_UNUSED(conn);

	printk("Conn params updated: interval %u.%02u ms, latency %u, timeout %u ms\n",
	       (interval * 5) / 4, ((interval * 5) % 4) * 25, latency, timeout * 10);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	ARG_UNUSED(conn);

	if (err) {
		printk("Security failed: level %u err %d %s\n", level, err,
		       bt_security_err_to_str(err));
	} else {
		printk("Security changed: level %u\n", level);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
	.security_changed = security_changed,
};

static void pairing_confirm(struct bt_conn *conn)
{
	(void)bt_conn_auth_pairing_confirm(conn);
}

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
}

/* The cancel callback is mandatory whenever an interactive auth callback
 * (here, pairing_confirm) is provided.
 */
static struct bt_conn_auth_cb auth_cb = {
	.pairing_confirm = pairing_confirm,
	.cancel = auth_cancel,
};

int main(void)
{
	int err;

	printk("Starting Central Latency sample\n");

	err = bt_conn_auth_cb_register(&auth_cb);
	if (err) {
		printk("Failed to register auth callbacks (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	k_work_init_delayable(&latency_update_work, latency_update_work_handler);

	scan_init();
	scan_start();

	return 0;
}
