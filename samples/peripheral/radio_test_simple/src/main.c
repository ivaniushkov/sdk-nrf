/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Transmits bursts of CS mode-2 like tones (unmodulated carrier for T_PM)
 * on a fixed frequency. Within a burst timing is hardware driven:
 *
 *   TIMER COMPARE[PERIOD]   -> clear TIMER + RADIO TXEN
 *   TIMER COMPARE[TONE_END] -> RADIO DISABLE
 *   RADIO DISABLED IRQ      -> count tones, stop TIMER after the last one
 *
 * TX_PIN is high while the carrier is on air: RADIO READY sets it, TONE_END clears it.
 * Between bursts HFXO is released and constant latency mode is turned off.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <soc.h>
#include <nrfx_gpiote.h>
#include <gpiote_nrfx.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_dppi.h>
#include <hal/nrf_power.h>
#if NRF_ERRATA_STATIC_CHECK(54L, 39)
#include <hal/nrf_clock.h>
#endif

/* Gap between consecutive tones and tone (phase measurement) duration. */
#define TONE_GAP_US 80
#define CS_T_PM_US  10

/* TXEN to TXIDLE with fast ramp-up enabled. */
#define TX_RAMPUP_US 41

/* Time between consecutive TXEN triggers (tone starts). */
#define TONE_INTERVAL_US (TONE_GAP_US + CS_T_PM_US)
#define TONE_END_US      (TX_RAMPUP_US + CS_T_PM_US)

/* Number of tones per burst, followed by a pause with HFXO off. */
#define BURST_TONES    75
#define BURST_PAUSE_MS 10

#define RADIO_FREQ_MHZ 2440
#define RADIO_MODE     NRF_RADIO_MODE_BLE_1MBIT
#define RADIO_TX_POWER NRF_RADIO_TXPOWER_0DBM
#define RADIO_IRQN     DT_IRQN(DT_NODELABEL(radio))
#define RADIO_IRQ_PRIO DT_IRQ(DT_NODELABEL(radio), priority)

#define TIMER              NRF_TIMER10
#define TIMER_FREQ_HZ      USEC_PER_SEC
#define TIMER_CC_TONE_END  NRF_TIMER_CC_CHANNEL0
#define TIMER_CC_PERIOD    NRF_TIMER_CC_CHANNEL1
#define TIMER_EVT_TONE_END NRF_TIMER_EVENT_COMPARE0
#define TIMER_EVT_PERIOD   NRF_TIMER_EVENT_COMPARE1
#define TIMER_SHORTS       NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK

#define DPPIC           NRF_DPPIC10
#define DPPI_CH_TXEN    0
#define DPPI_CH_DISABLE 1
#define DPPI_CH_READY   2
#define DPPI_CH_MASK    (BIT(DPPI_CH_TXEN) | BIT(DPPI_CH_DISABLE) | BIT(DPPI_CH_READY))

/* Pin indicating TX activity, LED1 (P1.10) on the nRF54L15 DK. */
#define TX_PIN_NODE   DT_ALIAS(led1)
#define TX_PIN        NRF_DT_GPIOS_TO_PSEL(TX_PIN_NODE, gpios)
#define TX_PIN_GPIOTE (&GPIOTE_NRFX_INST_BY_NODE(NRF_DT_GPIOTE_NODE(TX_PIN_NODE, gpios)))

static const struct device *const hfxo = DEVICE_DT_GET_ONE(nordic_nrf_clock_xo);
static K_SEM_DEFINE(burst_done, 0, 1);
static uint32_t tones_sent;

static void radio_isr(const void *arg)
{
	ARG_UNUSED(arg);

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	if (++tones_sent == BURST_TONES) {
		nrf_timer_task_trigger(TIMER, NRF_TIMER_TASK_STOP);
		k_sem_give(&burst_done);
	}
}

static int clock_on(void)
{
	int err = nrf_clock_control_request_sync(hfxo, NULL, K_FOREVER);

	if (err < 0) {
		return err;
	}

	nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
#if NRF_ERRATA_STATIC_CHECK(54L, 39)
	if (NRF_ERRATA_DYNAMIC_CHECK(54L, 39)) {
		nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
	}
#endif
	return 0;
}

static void clock_off(void)
{
#if NRF_ERRATA_STATIC_CHECK(54L, 39)
	if (NRF_ERRATA_DYNAMIC_CHECK(54L, 39)) {
		nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTOP);
	}
#endif
	nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_LOWPWR);
	(void)nrf_clock_control_release(hfxo, NULL);
}

/* Radio and TIMER are reconfigured before every burst as their domain may be powered off. */
static void radio_config(void)
{
	nrf_radio_mode_set(NRF_RADIO, RADIO_MODE);
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, true);
	nrf_radio_txpower_set(NRF_RADIO, RADIO_TX_POWER);
	nrf_radio_shorts_set(NRF_RADIO, 0);
	nrf_radio_frequency_set(NRF_RADIO, RADIO_FREQ_MHZ);

	nrf_radio_subscribe_set(NRF_RADIO, NRF_RADIO_TASK_TXEN, DPPI_CH_TXEN);
	nrf_radio_subscribe_set(NRF_RADIO, NRF_RADIO_TASK_DISABLE, DPPI_CH_DISABLE);
	nrf_radio_publish_set(NRF_RADIO, NRF_RADIO_EVENT_READY, DPPI_CH_READY);

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
}

static void timer_config(void)
{
	nrf_timer_task_trigger(TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_mode_set(TIMER, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(TIMER, NRF_TIMER_PRESCALER_CALCULATE(
					       NRF_TIMER_BASE_FREQUENCY_GET(TIMER), TIMER_FREQ_HZ));
	nrf_timer_cc_set(TIMER, TIMER_CC_TONE_END, TONE_END_US);
	nrf_timer_cc_set(TIMER, TIMER_CC_PERIOD, TONE_INTERVAL_US);
	nrf_timer_shorts_enable(TIMER, TIMER_SHORTS);

	nrf_timer_publish_set(TIMER, TIMER_EVT_PERIOD, DPPI_CH_TXEN);
	nrf_timer_publish_set(TIMER, TIMER_EVT_TONE_END, DPPI_CH_DISABLE);
}

/* Drives TX_PIN from radio domain DPPI channels, bridged to the pin's GPIOTE domain. */
static int tx_pin_init(void)
{
	static const uint8_t radio_ch[] = {DPPI_CH_READY, DPPI_CH_DISABLE};
	const nrfx_gpiote_output_config_t out_cfg = NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG;
	nrfx_gpiote_task_config_t task_cfg = {
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
	};
	nrfx_gppi_resource_t res = {.domain_id = nrfx_gppi_domain_id_get((uint32_t)DPPIC)};
	nrfx_gppi_handle_t handle;

	if (nrfx_gpiote_channel_alloc(TX_PIN_GPIOTE, &task_cfg.task_ch) != 0 ||
	    nrfx_gpiote_output_configure(TX_PIN_GPIOTE, TX_PIN, &out_cfg, &task_cfg) != 0) {
		return -EIO;
	}

	const uint32_t gpiote_task[] = {
		nrfx_gpiote_set_task_address_get(TX_PIN_GPIOTE, TX_PIN),
		nrfx_gpiote_clr_task_address_get(TX_PIN_GPIOTE, TX_PIN),
	};

	for (size_t i = 0; i < ARRAY_SIZE(radio_ch); i++) {
		res.channel = radio_ch[i];
		if (nrfx_gppi_ext_conn_alloc(res.domain_id, nrfx_gppi_domain_id_get(gpiote_task[i]),
					     &handle, &res) < 0) {
			return -ENOMEM;
		}
		nrfx_gppi_ep_attach(gpiote_task[i], handle);
		nrfx_gppi_conn_enable(handle);
	}

	nrfx_gpiote_out_task_enable(TX_PIN_GPIOTE, TX_PIN);
	return 0;
}

static void burst_send(void)
{
	tones_sent = 0;
	radio_config();
	timer_config();

	/* The first tone is started by software, the following ones by TIMER. */
	nrf_timer_task_trigger(TIMER, NRF_TIMER_TASK_START);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);

	k_sem_take(&burst_done, K_FOREVER);
}

int main(void)
{
	IRQ_CONNECT(RADIO_IRQN, RADIO_IRQ_PRIO, radio_isr, NULL, 0);
	irq_enable(RADIO_IRQN);
	nrf_dppi_channels_enable(DPPIC, DPPI_CH_MASK);

	if (tx_pin_init() != 0) {
		return 0;
	}

	while (clock_on() == 0) {
		burst_send();
		clock_off();
		k_msleep(BURST_PAUSE_MS);
	}

	return 0;
}
