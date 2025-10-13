/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Peripheral Heart Rate over LE Coded PHY sample
 */
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <dk_buttons_and_leds.h>

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000
#define NOTIFY_INTERVAL         1000

static void start_advertising_coded(struct k_work *work);
static void notify_work_handler(struct k_work *work);

static K_WORK_DEFINE(start_advertising_worker, start_advertising_coded);
static K_WORK_DELAYABLE_DEFINE(notify_work, notify_work_handler);

static struct bt_le_ext_adv *adv_conn;
static struct bt_le_ext_adv *adv_large;

/*
 * Build a 1650-byte extended advertising payload using multiple
 * Manufacturer Specific Data AD structures (type 0xFF).
 * Each full AD structure contributes 256 bytes total to the HCI
 * Advertising_Data_Length: 1 (Length) + 1 (Type) + 254 (Data).
 * 1650 = 6 * 256 + 114, so we use 6 full entries (254 data bytes)
 * and one final entry contributing 114 bytes total => 112 data bytes.
 */
#define ADV_TARGET_TOTAL_LEN               1650
#define MFG_AD_TYPE                        BT_DATA_MANUFACTURER_DATA
#define MFG_DATA_BYTES_PER_FULL_ENTRY      254
#define FULL_ENTRY_TOTAL_BYTES             (1 /*len*/ + 1 /*type*/ + MFG_DATA_BYTES_PER_FULL_ENTRY)
#define FULL_ENTRY_COUNT                   6
#define LAST_ENTRY_TOTAL_BYTES             (ADV_TARGET_TOTAL_LEN - (FULL_ENTRY_COUNT * FULL_ENTRY_TOTAL_BYTES))
#define LAST_ENTRY_DATA_LEN                (LAST_ENTRY_TOTAL_BYTES - 2 /*len+type*/)

BUILD_ASSERT(FULL_ENTRY_TOTAL_BYTES == 256, "Unexpected full entry sizing");
BUILD_ASSERT(LAST_ENTRY_DATA_LEN > 0, "Last AD entry must have positive data length");

/* Company Identifier for Nordic Semiconductor ASA (0x0059) */
#define MFG_COMPANY_ID_LSB                 0x59
#define MFG_COMPANY_ID_MSB                 0x00

/* Static buffers to persist after set_data() */
static uint8_t adv_mfg_full[FULL_ENTRY_COUNT][MFG_DATA_BYTES_PER_FULL_ENTRY];
static uint8_t adv_mfg_last[LAST_ENTRY_DATA_LEN];
static struct bt_data ad_large[FULL_ENTRY_COUNT + 1];

static void build_large_adv_payload(void)
{
	/* Initialize full-size manufacturer data entries */
	for (int i = 0; i < FULL_ENTRY_COUNT; i++) {
		uint8_t *buf = adv_mfg_full[i];
		/* First two bytes: Company ID (LSB first) */
		buf[0] = MFG_COMPANY_ID_LSB;
		buf[1] = MFG_COMPANY_ID_MSB;
		/* Fill remaining bytes with a simple pattern for visibility */
		for (size_t j = 2; j < MFG_DATA_BYTES_PER_FULL_ENTRY; j++) {
			buf[j] = (uint8_t)(i + 1);
		}

		ad_large[i].type = MFG_AD_TYPE;
		ad_large[i].data_len = MFG_DATA_BYTES_PER_FULL_ENTRY;
		ad_large[i].data = buf;
	}

	/* Initialize the last (partial) entry */
	adv_mfg_last[0] = MFG_COMPANY_ID_LSB;
	adv_mfg_last[1] = MFG_COMPANY_ID_MSB;
	for (size_t j = 2; j < LAST_ENTRY_DATA_LEN; j++) {
		adv_mfg_last[j] = 0xEE;
	}

	ad_large[FULL_ENTRY_COUNT].type = MFG_AD_TYPE;
	ad_large[FULL_ENTRY_COUNT].data_len = LAST_ENTRY_DATA_LEN;
	ad_large[FULL_ENTRY_COUNT].data = adv_mfg_last;
}


static const char *phy_to_str(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_NONE:
		return "No packets";
	case BT_GAP_LE_PHY_1M:
		return "LE 1M";
	case BT_GAP_LE_PHY_2M:
		return "LE 2M";
	case BT_GAP_LE_PHY_CODED:
		return "LE Coded";
	case BT_GAP_LE_PHY_CODED_S8:
		return "S=8 Coded";
	case BT_GAP_LE_PHY_CODED_S2:
		return "S=2 Coded";
	default: return "Unknown";
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	int err;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Connection failed, err 0x%02x %s\n", conn_err, bt_hci_err_to_str(conn_err));
		return;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (err %d)\n", err);
	} else {
		const struct bt_conn_le_phy_info *phy_info;
		phy_info = info.le.phy;

		printk("Connected: %s, tx_phy %s, rx_phy %s\n",
		       addr, phy_to_str(phy_info->tx_phy), phy_to_str(phy_info->rx_phy));
	}

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	k_work_submit(&start_advertising_worker);

	dk_set_led_off(CON_STATUS_LED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static const struct bt_data ad_conn[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

static int create_advertising_coded(void)
{
	int err;
    struct bt_le_adv_param param_conn =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN |
				     BT_LE_ADV_OPT_EXT_ADV |
				     BT_LE_ADV_OPT_CODED |
				     BT_LE_ADV_OPT_REQUIRE_S8_CODING,
				     BT_GAP_ADV_FAST_INT_MIN_2,
				     BT_GAP_ADV_FAST_INT_MAX_2,
				     NULL);

	/* Non-connectable large extended advertising at slower interval */
	struct bt_le_adv_param param_large =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_EXT_ADV |
				     BT_LE_ADV_OPT_CODED |
				     BT_LE_ADV_OPT_REQUIRE_S8_CODING,
				     BT_GAP_ADV_SLOW_INT_MIN,
				     BT_GAP_ADV_SLOW_INT_MAX,
				     NULL);

	err = bt_le_ext_adv_create(&param_conn, NULL, &adv_conn);
	if (err) {
		printk("Failed to create connectable advertiser (err %d)\n", err);
		return err;
	}

	printk("Created connectable adv: %p\n", adv_conn);

	/* Small AD set for connectable advertiser */
	err = bt_le_ext_adv_set_data(adv_conn, ad_conn, ARRAY_SIZE(ad_conn), NULL, 0);
	if (err) {
		printk("Failed to set connectable adv data (err %d)\n", err);
		return err;
	}

	/* Create large non-connectable advertiser */
	err = bt_le_ext_adv_create(&param_large, NULL, &adv_large);
	if (err) {
		printk("Failed to create large advertiser (err %d)\n", err);
		return err;
	}

	printk("Created large adv: %p\n", adv_large);

	build_large_adv_payload();
	err = bt_le_ext_adv_set_data(adv_large, ad_large, ARRAY_SIZE(ad_large), NULL, 0);
	if (err) {
		printk("Failed to set large adv data (err %d)\n", err);
		return err;
	}

	return 0;
}

static void start_advertising_coded(struct k_work *work)
{
	int err;

    err = bt_le_ext_adv_start(adv_conn, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start connectable advertiser (err %d)\n", err);
		return;
	}

	printk("Connectable advertiser %p started\n", adv_conn);

	err = bt_le_ext_adv_start(adv_large, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start large advertiser (err %d)\n", err);
		return;
	}

	printk("Large advertiser %p started\n", adv_large);
}

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	__ASSERT_NO_MSG(battery_level > 0);

	battery_level--;

	if (!battery_level) {
		battery_level = 100;
	}

	bt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
	static uint8_t heartrate = 100;

	heartrate++;
	if (heartrate == 160) {
		heartrate = 100;
	}

	bt_hrs_notify(heartrate);
}

static void notify_work_handler(struct k_work *work)
{
	/* Services data simulation. */
	hrs_notify();
	bas_notify();

	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(NOTIFY_INTERVAL));
}

int main(void)
{
	uint32_t led_status = 0;
	int err;

	printk("Starting Bluetooth Peripheral HR coded sample\n");

	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	err = create_advertising_coded();
	if (err) {
		printk("Advertising failed to create (err %d)\n", err);
		return 0;
	}

	k_work_submit(&start_advertising_worker);
	k_work_schedule(&notify_work, K_NO_WAIT);

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++led_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
