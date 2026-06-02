/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Minimal low-power connectable BLE peripheral.
 *
 *  Advertises connectably and keeps the link up with no application-level
 *  data. No UART, NUS, logging, or DK peripherals. Connection parameters,
 *  peripheral latency and subrating are driven by the peer (central). The
 *  link is encrypted automatically (Just Works, no bonding).
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static struct k_work adv_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void adv_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

#if defined(CONFIG_BT_SMP)
static void pairing_confirm(struct bt_conn *conn)
{
	(void)bt_conn_auth_pairing_confirm(conn);
}

static void auth_cancel(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
}

/* The cancel callback is mandatory when any interactive auth callback
 * (here, pairing_confirm) is provided; otherwise bt_conn_auth_cb_register()
 * returns -EINVAL.
 */
static struct bt_conn_auth_cb auth_cb = {
	.pairing_confirm = pairing_confirm,
	.cancel = auth_cancel,
};

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	ARG_UNUSED(level);

	if (err) {
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}
#endif /* CONFIG_BT_SMP */

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		advertising_start();
		return;
	}

#if defined(CONFIG_BT_SMP)
	(void)bt_conn_set_security(conn, BT_SECURITY_L2);
#endif
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(reason);
}

static void recycled_cb(void)
{
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled_cb,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed,
#endif
};

int main(void)
{
	int err;

#if defined(CONFIG_BT_SMP)
	err = bt_conn_auth_cb_register(&auth_cb);
	if (err) {
		return 0;
	}
#endif

	err = bt_enable(NULL);
	if (err) {
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	for (;;) {
		k_sleep(K_FOREVER);
	}
}
