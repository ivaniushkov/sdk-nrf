/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Reflector with Ranging Responder sample
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/settings/settings.h>
#include <bluetooth/services/ras.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1

static K_SEM_DEFINE(sem_connected, 0, 1);

static struct bt_conn *connection;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected to %s (err 0x%02X)", addr, err);

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
	}

	connection = bt_conn_ref(conn);

	k_sem_give(&sem_connected);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02X)", reason);

	bt_conn_unref(conn);
	connection = NULL;

	dk_set_led_off(CON_STATUS_LED);
}

static void remote_capabilities_cb(struct bt_conn *conn, struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	LOG_INF("CS capability exchange completed.");
	LOG_INF("CS capabilities:");
	LOG_INF("  Number of CS configurations: %d", params->num_config_supported);
	LOG_INF("  Max consecutive procedures supported: %d",
		params->max_consecutive_procedures_supported);
	LOG_INF("  Number of antennas supported: %d", params->num_antennas_supported);
	LOG_INF("  Max antenna paths supported: %d", params->max_antenna_paths_supported);
	LOG_INF("  Initiator supported: %d", params->initiator_supported);
	LOG_INF("  Reflector supported: %d", params->reflector_supported);
	LOG_INF("  Mode-3 supported: %d", params->mode_3_supported);
	LOG_INF("  RTT AA-Only precision: %d", params->rtt_aa_only_precision);
	LOG_INF("  RTT Sounding precision: %d", params->rtt_sounding_precision);
	LOG_INF("  RTT Random Payload precision: %d", params->rtt_random_payload_precision);
	LOG_INF("  RTT AA-Only steps: %d", params->rtt_aa_only_n);
	LOG_INF("  RTT Sounding steps: %d", params->rtt_sounding_n);
	LOG_INF("  RTT Random Payload steps: %d", params->rtt_random_payload_n);
	LOG_INF("  Phase-based NADM Sounding supported: %d",
		params->phase_based_nadm_sounding_supported);
	LOG_INF("  Phase-based NADM Random supported: %d",
		params->phase_based_nadm_random_supported);
	LOG_INF("  CS_SYNC LE 2M PHY supported: %d", params->cs_sync_2m_phy_supported);
	LOG_INF("  CS_SYNC LE 2M 2BT PHY supported: %d", params->cs_sync_2m_2bt_phy_supported);
	LOG_INF("  CS without FAE supported: %d", params->cs_without_fae_supported);
	LOG_INF("  Channel Selection Algorithm #3c supported: %d", params->chsel_alg_3c_supported);
	LOG_INF("  PBR from RTT Sounding Sequence supported: %d",
		params->pbr_from_rtt_sounding_seq_supported);
	LOG_INF("  T_IP1 times supported: %d", params->t_ip1_times_supported);
	LOG_INF("  T_IP2 times supported: %d", params->t_ip2_times_supported);
	LOG_INF("  T_FCS times supported: %d", params->t_fcs_times_supported);
	LOG_INF("  T_PM times supported: %d", params->t_pm_times_supported);
	LOG_INF("  Antenna switch period time: %d", params->t_sw_time);
	LOG_INF("  TX SNR capability: %d", params->tx_snr_capability);
}

static void config_created_cb(struct bt_conn *conn, struct bt_conn_le_cs_config *config)
{
	ARG_UNUSED(conn);
	LOG_INF("CS config creation complete. ID: %d", config->id);
	LOG_INF("Config:");
	LOG_INF("  ID: %d", config->id);
	LOG_INF("  Main mode type: %d", config->main_mode_type);
	LOG_INF("  Sub mode type: %d", config->sub_mode_type);
	LOG_INF("  Min main mode steps: %d", config->min_main_mode_steps);
	LOG_INF("  Max main mode steps: %d", config->max_main_mode_steps);
	LOG_INF("  Main mode repetition: %d", config->main_mode_repetition);
	LOG_INF("  Mode 0 steps: %d", config->mode_0_steps);
	LOG_INF("  Role: %d", config->role);
	LOG_INF("  RTT type: %d", config->rtt_type);
	LOG_INF("  CS Sync PHY: %d", config->cs_sync_phy);
	LOG_INF("  Channel map repetition: %d", config->channel_map_repetition);
	LOG_INF("  Channel selection type: %d", config->channel_selection_type);
	LOG_INF("  CH3C shape: %d", config->ch3c_shape);
	LOG_INF("  CH3C jump: %d", config->ch3c_jump);
	LOG_INF("  T IP1 time (us): %d", config->t_ip1_time_us);
	LOG_INF("  T IP2 time (us): %d", config->t_ip2_time_us);
	LOG_INF("  T FCS time (us): %d", config->t_fcs_time_us);
	LOG_INF("  T PM time (us): %d", config->t_pm_time_us);
	LOG_INF("  Channel map: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		config->channel_map[0], config->channel_map[1], config->channel_map[2], config->channel_map[3],
		config->channel_map[4], config->channel_map[5], config->channel_map[6], config->channel_map[7],
		config->channel_map[8], config->channel_map[9]);
}

static void security_enabled_cb(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	LOG_INF("CS security enabled.");
}

static void procedure_enabled_cb(struct bt_conn *conn,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	ARG_UNUSED(conn);
	if (params->state == 1) {
		LOG_INF("CS procedures enabled.");
	} else {
		LOG_INF("CS procedures disabled.");
	}
	LOG_INF("Procedure enabled complete:");
	LOG_INF("  Config ID: %d", params->config_id);
	LOG_INF("  State: %d", params->state);
	LOG_INF("  Tone antenna config selection: %d", params->tone_antenna_config_selection);
	LOG_INF("  Selected TX power: %d dB", params->selected_tx_power);
	LOG_INF("  Subevent length: %d us", params->subevent_len);
	LOG_INF("  Subevents per event: %d", params->subevents_per_event);
	LOG_INF("  Subevent interval: %d units of 0.625 ms", params->subevent_interval);
	LOG_INF("  Event interval: %d", params->event_interval);
	LOG_INF("  Procedure interval: %d", params->procedure_interval);
	LOG_INF("  Procedure count: %d", params->procedure_count);
	LOG_INF("  Max procedure length: %d units of 0.625 ms", params->max_procedure_len);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_cs_remote_capabilities_available = remote_capabilities_cb,
	.le_cs_config_created = config_created_cb,
	.le_cs_security_enabled = security_enabled_cb,
	.le_cs_procedure_enabled = procedure_enabled_cb,
};

int main(void)
{
	int err;

	LOG_INF("Starting Channel Sounding Reflector Sample");

	dk_leds_init();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	while (true) {
		k_sem_take(&sem_connected, K_FOREVER);

		const struct bt_le_cs_set_default_settings_param default_settings = {
			.enable_initiator_role = false,
			.enable_reflector_role = true,
			.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
			.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
		};

		err = bt_le_cs_set_default_settings(connection, &default_settings);
		if (err) {
			LOG_ERR("Failed to configure default CS settings (err %d)", err);
		}
	}

	return 0;
}
