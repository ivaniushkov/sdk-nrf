/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Ranging Requestor sample
 */

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/settings/settings.h>
#include "distance_estimation.h"

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#define CON_STATUS_LED DK_LED1

#define CS_CONFIG_ID	       0
#define NUM_MODE_0_STEPS       3
#define PROCEDURE_COUNTER_NONE (-1)

#define LOCAL_PROCEDURE_MEM                                                                        \
	((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) +                \
	 (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_done, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_rd_ready, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_rd_complete, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);

static struct bt_conn *connection;
static uint8_t n_ap;
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_peer_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	LOG_INF("Subevent result callback %d", result->header.procedure_counter);

	if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
		/* If this subevent was aborted, drop the entire procedure for now. */
		LOG_WRN("Subevent aborted");
		dropped_ranging_counter = result->header.procedure_counter;
		net_buf_simple_reset(&latest_local_steps);
		return;
	}

	if (dropped_ranging_counter == result->header.procedure_counter) {
		return;
	}

	if (result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);

			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			LOG_ERR("Not enough memory to store step data. (%d > %d)",
				latest_local_steps.len + result->step_data_buf->len,
				latest_local_steps.size);
			net_buf_simple_reset(&latest_local_steps);
			dropped_ranging_counter = result->header.procedure_counter;
			return;
		}
	}

	dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
	n_ap = result->header.num_antenna_paths;

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		most_recent_local_ranging_counter = result->header.procedure_counter;
		k_sem_give(&sem_procedure_done);
	} else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
		LOG_WRN("Procedure aborted");
		net_buf_simple_reset(&latest_local_steps);
	}
}

static void ranging_data_get_complete_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
	ARG_UNUSED(conn);

	if (err) {
		LOG_ERR("Error when getting ranging data with ranging counter %d (err %d)",
			ranging_counter, err);
		return;
	}

	LOG_INF("Ranging data get completed for ranging counter %d", ranging_counter);
	k_sem_give(&sem_rd_complete);
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_INF("Ranging data ready %i", ranging_counter);
	most_recent_peer_ranging_counter = ranging_counter;
	k_sem_give(&sem_rd_ready);
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_INF("Ranging data overwritten %i", ranging_counter);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
		return;
	}

	LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
	k_sem_give(&sem_mtu_exchange_done);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	LOG_INF("The discovery procedure succeeded");

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

	bt_gatt_dm_data_print(dm);

	err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
	if (err) {
		LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		LOG_ERR("Could not release the discovery data (err %d)", err);
	}

	k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	LOG_INF("The service could not be found during the discovery, disconnecting");
	bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	LOG_INF("The discovery procedure failed (err %d)", err);
	bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Security failed: %s level %u err %d %s", addr, level, err,
			bt_security_err_to_str(err));
		return;
	}

	LOG_INF("Security changed: %s level %u", addr, level);
	k_sem_give(&sem_security);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	/* Ignore peer parameter preferences. */
	return false;
}

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
	k_sem_give(&sem_remote_capabilities_obtained);
}

static void config_created_cb(struct bt_conn *conn, struct bt_conn_le_cs_config *config)
{
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
	k_sem_give(&sem_config_created);
}

static void security_enabled_cb(struct bt_conn *conn)
{
	LOG_INF("CS security enabled.");
	k_sem_give(&sem_cs_security_enabled);
}

static void procedure_enabled_cb(struct bt_conn *conn,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
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

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	int err;

	LOG_INF("Connecting failed, restarting scanning");

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Failed to restart scanning (err %i)", err);
		return;
	}
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	LOG_INF("Connecting");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;

	struct bt_scan_init_param param = {
		.scan_param = NULL, .conn_param = BT_LE_CONN_PARAM_DEFAULT, .connect_if_match = 1};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		return err;
	}

	return 0;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_req = le_param_req,
	.security_changed = security_changed,
	.le_cs_remote_capabilities_available = remote_capabilities_cb,
	.le_cs_config_created = config_created_cb,
	.le_cs_security_enabled = security_enabled_cb,
	.le_cs_procedure_enabled = procedure_enabled_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

int main(void)
{
	int err;

	LOG_INF("Starting Channel Sounding Initiator Sample");

	dk_leds_init();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = scan_init();
	if (err) {
		LOG_ERR("Scan init failed (err %d)", err);
		return 0;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		LOG_ERR("Scanning failed to start (err %i)", err);
		return 0;
	}

	k_sem_take(&sem_connected, K_FOREVER);

	err = bt_conn_set_security(connection, BT_SECURITY_L2);
	if (err) {
		LOG_ERR("Failed to encrypt connection (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_security, K_FOREVER);

	static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

	bt_gatt_exchange_mtu(connection, &mtu_exchange_params);

	k_sem_take(&sem_mtu_exchange_done, K_FOREVER);

	err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		LOG_ERR("Discovery failed (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_discovery_done, K_FOREVER);

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(connection, &default_settings);
	if (err) {
		LOG_ERR("Failed to configure default CS settings (err %d)", err);
		return 0;
	}

	err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
	if (err) {
		LOG_ERR("RAS RREQ ranging data overwritten subscribe failed (err %d)", err);
		return 0;
	}

	err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
	if (err) {
		LOG_ERR("RAS RREQ ranging data ready subscribe failed (err %d)", err);
		return 0;
	}

	err = bt_ras_rreq_on_demand_rd_subscribe(connection);
	if (err) {
		LOG_ERR("RAS RREQ On-demand ranging data subscribe failed (err %d)", err);
		return 0;
	}

	err = bt_ras_rreq_cp_subscribe(connection);
	if (err) {
		LOG_ERR("RAS RREQ CP subscribe failed (err %d)", err);
		return 0;
	}

	err = bt_le_cs_read_remote_supported_capabilities(connection);
	if (err) {
		LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

	struct bt_le_cs_create_config_params config_params = {
		.id = CS_CONFIG_ID,
		.main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
		.sub_mode_type = BT_CONN_LE_CS_SUB_MODE_1,
		.min_main_mode_steps = 10,
		.max_main_mode_steps = 20,
		.main_mode_repetition = 0,
		.mode_0_steps = NUM_MODE_0_STEPS,
		.role = BT_CONN_LE_CS_ROLE_INITIATOR,
		.rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
		.cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
		.channel_map_repetition = 5,
		.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
		.ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
		.ch3c_jump = 2,
	};

	bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

	err = bt_le_cs_create_config(connection, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
	if (err) {
		LOG_ERR("Failed to create CS config (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_config_created, K_FOREVER);

	err = bt_le_cs_security_enable(connection);
	if (err) {
		LOG_ERR("Failed to start CS Security (err %d)", err);
		return 0;
	}

	k_sem_take(&sem_cs_security_enabled, K_FOREVER);

	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = 100,
		.min_procedure_interval = 100,
		.max_procedure_interval = 100,
		.max_procedure_count = 1,
		.min_subevent_len = 60000,
		.max_subevent_len = 60000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_INDEX_ONE,
		.phy = BT_LE_CS_PROCEDURE_PHY_1M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
	if (err) {
		LOG_ERR("Failed to set procedure parameters (err %d)", err);
		return 0;
	}

	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = 1,
	};

	while (true) {
		err = bt_le_cs_procedure_enable(connection, &params);
		if (err) {
			LOG_ERR("Failed to enable CS procedures (err %d)", err);
			return 0;
		}

		err = k_sem_take(&sem_procedure_done, K_SECONDS(1));
		if (err) {
			LOG_WRN("Timeout waiting for local procedure done (err %d)", err);

			/* Check if remote has rd ready to align counters. */
			k_sem_take(&sem_rd_ready, K_SECONDS(1));

			goto retry;
		}

		err = k_sem_take(&sem_rd_ready, K_SECONDS(1));
		if (err) {
			LOG_WRN("Timeout waiting for ranging data ready (err %d)", err);
			goto retry;
		}

		if (most_recent_peer_ranging_counter != most_recent_local_ranging_counter) {
			LOG_WRN("Mismatch of local and peer ranging counters (%d != %d)",
				most_recent_peer_ranging_counter,
				most_recent_local_ranging_counter);
			goto retry;
		}

		err = bt_ras_rreq_cp_get_ranging_data(connection, &latest_peer_steps,
						      most_recent_peer_ranging_counter,
						      ranging_data_get_complete_cb);
		if (err) {
			LOG_ERR("Get ranging data failed (err %d)", err);
			goto retry;
		}

		err = k_sem_take(&sem_rd_complete, K_SECONDS(5));
		if (err) {
			LOG_ERR("Timeout waiting for ranging data complete (err %d)", err);
			goto retry;
		}

		estimate_distance(&latest_local_steps, &latest_peer_steps, n_ap,
				  BT_CONN_LE_CS_ROLE_INITIATOR);

retry:
		net_buf_simple_reset(&latest_local_steps);
		net_buf_simple_reset(&latest_peer_steps);
	}

	return 0;
}
