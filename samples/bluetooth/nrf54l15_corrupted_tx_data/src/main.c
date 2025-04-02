/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <nrfx_timer.h>
#include <nrfx_egu.h>
#include "hal/nrf_clock.h"
#include "hal/nrf_radio.h"

#include <stdbool.h>

/* Sample application to transmit dummy data in BLE format using nrf54l15 */

/* Configs */
#define BLE_FREQ (2402) // adv channel 37
#define ACCESS_ADDRESS (0x8E89BED6)

/* Internal defines */

#define CRC_POLY                 (0x0000065B)
#define CRC_INIT                 (0x00555555)

static const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(10);
static atomic_t send_packet;

static uint8_t pdu_buffer[17];

static int clock_init(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		printk("Unable to get the Clock manager\n");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		printk("Clock request failed: %d\n", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			printk("Clock could not be started: %d\n", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	// nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	return err;
}

static void timer_handler(nrf_timer_event_t event_type, void *context){
	/* do nothing */
	atomic_set(&send_packet, true);
}

static int timer_init()
{
	nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = {
		.frequency = NRFX_MHZ_TO_HZ(1),
		.mode      = NRF_TIMER_MODE_TIMER,
		.bit_width = NRF_TIMER_BIT_WIDTH_32,
	};

	err = nrfx_timer_init(&timer_instance, &timer_cfg, timer_handler);
	if (err != NRFX_SUCCESS) {
		printk("nrfx_timer_init failed with: %d\n", err);
		return -EAGAIN;
	}

	IRQ_CONNECT(TIMER10_IRQn, 3,
		nrfx_timer_10_irq_handler, NULL, 0);

	return 0;
}

static int radio_prepare(bool tx)
{
	if (tx)
	{
		pdu_buffer[0]  = 0x42;
		pdu_buffer[1]  = 0x0e; // len
		pdu_buffer[2]  = 0x00; // s1
		pdu_buffer[3]  = 0x86; // bd addr
		pdu_buffer[4]  = 0x85; // bd addr
		pdu_buffer[5]  = 0x84; // bd addr
		pdu_buffer[6]  = 0x83; // bd addr
		pdu_buffer[7]  = 0x82; // bd addr
		pdu_buffer[8]  = 0x81; // bd addr
		pdu_buffer[9]  = 0x07; // dummy data
		pdu_buffer[10] = 0xff; // dummy data
		pdu_buffer[11] = 0x4c; // dummy data
		pdu_buffer[12] = 0x00; // dummy data
		pdu_buffer[13] = 0x12; // dummy data
		pdu_buffer[14] = 0x01; // dummy data
		pdu_buffer[15] = 0x02; // dummy data
		pdu_buffer[16] = 0x03; // dummy data
	}
	nrf_radio_frequency_set(NRF_RADIO, BLE_FREQ);

	/* Setting packet pointer will start the radio */
	nrf_radio_packetptr_set(NRF_RADIO, pdu_buffer);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);

	/* Set shortcuts:
	 * between READY event and START task and
	 * between END event and DISABLE task
	 */
	nrf_radio_shorts_set(NRF_RADIO,
			     NRF_RADIO_SHORT_READY_START_MASK |
			     NRF_RADIO_SHORT_PHYEND_DISABLE_MASK);

	NVIC_ClearPendingIRQ(RADIO_0_IRQn);

	if (!tx) {
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	}
	return 0;
}

static int timer_set(void)
{
	nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0,
		20000,
		NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	nrfx_timer_enable(&timer_instance);
	return 0;
}

static void radio_reset(void)
{
	nrf_radio_shorts_set(NRF_RADIO, 0);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
		/* Do nothing */
	}
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	irq_disable(RADIO_0_IRQn);
	nrf_radio_int_disable(NRF_RADIO,
			NRF_RADIO_INT_READY_MASK |
			NRF_RADIO_INT_ADDRESS_MASK |
			NRF_RADIO_INT_END_MASK);
}

static int radio_init(void)
{
	nrf_radio_packet_conf_t packet_conf;

	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, 1);

	/* Turn off radio before configuring it */
	// nrf_radio_txpower_set(NRF_RADIO, 0);
	nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_BLE_1MBIT);

	/* Set the access address, address0/prefix0 used for both Rx and Tx
	 * address.
	 */
	nrf_radio_prefix0_set(NRF_RADIO, ACCESS_ADDRESS >> 24);
	nrf_radio_base0_set(NRF_RADIO, ACCESS_ADDRESS << 8);
	// nrf_radio_rxaddresses_set(NRF_RADIO, RADIO_RXADDRESSES_ADDR0_Enabled);
	nrf_radio_txaddress_set(NRF_RADIO, 0x00);

	/* Configure CRC calculation. */
	nrf_radio_crcinit_set(NRF_RADIO, CRC_INIT);
	nrf_radio_crc_configure(NRF_RADIO, RADIO_CRCCNF_LEN_Three,
				NRF_RADIO_CRC_ADDR_SKIP, CRC_POLY);

	memset(&packet_conf, 0, sizeof(packet_conf));
	packet_conf.s0len = 1;
	packet_conf.s1len = 0;
	packet_conf.s1incl = 1;
	packet_conf.lflen = 8;
	packet_conf.plen = 0; //need to change to 1 if we use 2M; 0 for 1M
	packet_conf.whiteen = true;
	packet_conf.big_endian = false;
	packet_conf.balen = 3;
	packet_conf.statlen = 0;
	packet_conf.maxlen = 37; // maybe need to change
	packet_conf.termlen = 0;
	packet_conf.cilen = 0;

	nrf_radio_packet_configure(NRF_RADIO, &packet_conf);

	return 0;
}

static int radio_start(bool tx)
{
	if (tx) {
		/* Shorts will start radio in RX mode when it is ready */
		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	}
	return 0;
}

int main(void)
{
	printk("starting\n");
	bool tx = true;

	int ret = clock_init();
	if (ret) {
		printk("Clock init failed: %d\n", ret);
		return ret;
	}
	ret = timer_init();
	if (ret) {
		printk("Timer init failed: %d\n", ret);
		return ret;
	}
	radio_reset();
	ret = radio_prepare(tx);
	if (ret) {
		printk("Radio prepare failed: %d\n", ret);
		return ret;
	}

	ret = radio_init();
	if (ret) {
		printk("Radio init failed: %d\n", ret);
		return ret;
	}

	ret = timer_set();
	if (ret) {
		printk("Timer set failed: %d\n", ret);
		return ret;
	}
	while (true) {
		if (atomic_get(&send_packet)) {
			static uint32_t i;
			printk("sending packet %d\n", i++);
			atomic_set(&send_packet, false);


			ret = radio_start(tx);
			if (ret) {
				printk("Radio start failed: %d\n", ret);
				return ret;
			}
			volatile uint32_t cnt = 0;
			while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_END)) {
				cnt++;
			}
			nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
			printk("cnt: %d\n", cnt);
		}
	}
	printk("ok\n");
	return 0;
}
