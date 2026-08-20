/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <sdc.h>
#include "sdc_log_config.h"
#include "sdc_log_msg.h"

LOG_MODULE_REGISTER(sdc_log, CONFIG_BT_CTLR_SDC_LOG_PRINT_LEVEL);

#define SDC_LOG_FIFO_SIZE CONFIG_BT_CTLR_SDC_LOG_FIFO_SIZE
#define SDC_LOG_MAX_ARGS  3U /* must match SDC_LOG_MAX_ARGS in sdc_log.h */
#define SDC_LOG_LINE_BUF_SIZE 128U /* max formatted line length; format strings are build-time bounded */

/* Unused argv slots are zero; the format string reads only as many as it needs. */
struct sdc_log_entry {
	uint32_t id;
	uint32_t argv[SDC_LOG_MAX_ARGS];
};

K_MSGQ_DEFINE(sdc_log_msgq, sizeof(struct sdc_log_entry), SDC_LOG_FIFO_SIZE, 4);

static int sdc_log_id_cmp(const void *key, const void *elem)
{
	uint32_t id = *(const uint32_t *)key;
	const struct sdc_log_msg_entry *entry = elem;

	if (id < entry->id) {
		return -1;
	}
	if (id > entry->id) {
		return 1;
	}
	return 0;
}

static const struct sdc_log_msg_entry *sdc_log_msg_lookup(uint32_t id)
{
	return bsearch(&id, sdc_log_msgs, ARRAY_SIZE(sdc_log_msgs),
		       sizeof(sdc_log_msgs[0]), sdc_log_id_cmp);
}

static void sdc_log_print(const struct sdc_log_entry *entry)
{
	const struct sdc_log_msg_entry *msg = sdc_log_msg_lookup(entry->id);
	char line[SDC_LOG_LINE_BUF_SIZE];

	if (msg == NULL) {
		/* Unknown id, or the entry was gated out at the configured log level. */
		return;
	}

	/* fmt is a trusted, build-time-generated string. The log-collection script
	 * rejects %s/%n and non-word specifiers so this stays memory-safe. Unused
	 * argument slots are zero and ignored by the format string.
	 */
	(void)snprintf(line, sizeof(line), msg->fmt,
		       entry->argv[0], entry->argv[1], entry->argv[2]);

	switch (msg->level) {
	case LOG_LEVEL_ERR:
		LOG_ERR("%s", line);
		break;
	default:
		LOG_INF("%s", line);
		break;
	}
}

static void sdc_log_thread_fn(void *p1, void *p2, void *p3)
{
	struct sdc_log_entry entry;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		if (k_msgq_get(&sdc_log_msgq, &entry, K_FOREVER) == 0) {
			sdc_log_print(&entry);
		}
	}
}

/* Registered via sdc_log_handlers_set(); run in the SDC calling context, so they must
 * only enqueue and defer printing to the log thread. */
static void sdc_log_enqueue0(uint32_t id)
{
	struct sdc_log_entry entry = { .id = id };

	(void)k_msgq_put(&sdc_log_msgq, &entry, K_NO_WAIT);
}

static void sdc_log_enqueue1(uint32_t id, uint32_t a0)
{
	struct sdc_log_entry entry = { .id = id, .argv = { a0 } };

	(void)k_msgq_put(&sdc_log_msgq, &entry, K_NO_WAIT);
}

static void sdc_log_enqueue2(uint32_t id, uint32_t a0, uint32_t a1)
{
	struct sdc_log_entry entry = { .id = id, .argv = { a0, a1 } };

	(void)k_msgq_put(&sdc_log_msgq, &entry, K_NO_WAIT);
}

static void sdc_log_enqueue3(uint32_t id, uint32_t a0, uint32_t a1, uint32_t a2)
{
	struct sdc_log_entry entry = { .id = id, .argv = { a0, a1, a2 } };

	(void)k_msgq_put(&sdc_log_msgq, &entry, K_NO_WAIT);
}

static const sdc_log_handlers_t sdc_log_handlers = {
	.log0 = sdc_log_enqueue0,
	.log1 = sdc_log_enqueue1,
	.log2 = sdc_log_enqueue2,
	.log3 = sdc_log_enqueue3,
};

/* Static drain thread; blocks on the FIFO until messages arrive. */
K_THREAD_DEFINE(sdc_log_thread, CONFIG_BT_CTLR_SDC_LOG_THREAD_STACK_SIZE,
		sdc_log_thread_fn, NULL, NULL, NULL,
		K_PRIO_PREEMPT(CONFIG_SYSTEM_WORKQUEUE_PRIORITY), 0, 0);

/* Registering the handler is only a pointer write, so do it at the earliest init level.
 * This is before any POST_KERNEL controller init, so the first sdc_init() log is captured
 * without depending on any specific init-priority relationship. */
static int sdc_log_register(void)
{
	return sdc_log_handlers_set(&sdc_log_handlers);
}

SYS_INIT(sdc_log_register, PRE_KERNEL_1, 0);
