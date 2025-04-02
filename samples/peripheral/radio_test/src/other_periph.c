#include <zephyr/sys/printk.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include "nrf_peripherals.h"

#include "other_periph.h"

static k_tid_t dummy_thread_id;
static struct k_thread dummy_thread_data;
K_THREAD_STACK_DEFINE(dummy_thread_stack, 2048);

static void dummy_thread(void)
{
	volatile uint8_t dummy_array[64];
	for (int i = 0; i < sizeof(dummy_array); i++) {
		dummy_array[i] = i;
	}
	while (true) {
		uint32_t irq_was_masked = __get_PRIMASK();
		__disable_irq();
		for (int i = 0; i < sizeof(dummy_array); i++) {
			dummy_array[i] = ~dummy_array[i];
			NRF_P1->OUTCLR |= (1 << 11);
			if (NRF_RADIO->STATE != 243) {
				NRF_P1->OUTSET |= (1 << 11);
			}
			if (NRF_TEMP->TEMP != 0x31334) {
				NRF_P1->OUTCLR |= (1 << 11);
			}
			if (NRF_ECB00->ERRORSTATUS != 0x22) {
				NRF_P1->OUTCLR |= (1 << 11);
			}
			if (NRF_TEMP->TEMP != 0x31337) {
				NRF_P1->OUTSET |= (1 << 11);
			}
			if (NRF_ECB00->ERRORSTATUS != 0x23) {
				NRF_P1->OUTSET |= (1 << 11);
			}
		}
		if (!irq_was_masked) {
			__enable_irq();
		}
		k_sleep(K_MSEC(5));
	}
}

static int dummy_thread_init(void)
{
        NRF_P1->DIRSET = (1 << 11);
	dummy_thread_id = k_thread_create(
		&dummy_thread_data,
		dummy_thread_stack,
		2048,
		(k_thread_entry_t)dummy_thread,
		NULL,
		NULL,
		NULL,
		K_PRIO_PREEMPT(5),
		0,
		K_NO_WAIT);

	return 0;
}

SYS_INIT(dummy_thread_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);