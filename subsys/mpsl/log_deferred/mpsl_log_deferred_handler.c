/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(mpsl_log_def, CONFIG_MPSL_LOG_LEVEL);

#define MPSL_LOG_SEVERITY_SHIFT  30
#define MPSL_LOG_SEVERITY_MASK   0x3U
#define MPSL_LOG_GROUP_SHIFT     29
#define MPSL_LOG_GROUP_MASK     1U
#define MPSL_LOG_FILE_ID_SHIFT  16
#define MPSL_LOG_FILE_ID_MASK   0x1FFFU
#define MPSL_LOG_COUNTER_MASK   0xFFFFU

#define MPSL_LOG_FIFO_SIZE CONFIG_MPSL_LOG_FIFO_SIZE

K_MSGQ_DEFINE(mpsl_log_msgq, sizeof(uint32_t), MPSL_LOG_FIFO_SIZE, 4);
static K_THREAD_STACK_DEFINE(mpsl_log_thread_stack, CONFIG_MPSL_LOG_THREAD_STACK_SIZE);
static struct k_thread mpsl_log_thread;

#include "mpsl_log_msg.h"
#include "sdc_log_msg.h"

static const char *mpsl_log_msg_lookup(uint32_t msg_id)
{
	uint32_t group = (msg_id >> MPSL_LOG_GROUP_SHIFT) & MPSL_LOG_GROUP_MASK;
	uint32_t log_file_id = (msg_id >> MPSL_LOG_FILE_ID_SHIFT) & MPSL_LOG_FILE_ID_MASK;
	uint32_t counter = msg_id & MPSL_LOG_COUNTER_MASK;

	if (group == 0) {
		if (log_file_id < MPSL_LOG_FILE_COUNT &&
		    counter < mpsl_log_msg_count[log_file_id]) {
			return mpsl_log_msgs[log_file_id][counter];
		}
	} else {
		if (log_file_id < SDC_LOG_FILE_COUNT &&
		    counter < sdc_log_msg_count[log_file_id]) {
			return sdc_log_msgs[log_file_id][counter];
		}
	}
	return "(unknown)";
}

static void mpsl_log_thread_fn(void *p1, void *p2, void *p3)
{
	uint32_t msg_id;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		if (k_msgq_get(&mpsl_log_msgq, &msg_id, K_FOREVER) == 0) {
			const char *msg = mpsl_log_msg_lookup(msg_id);
			uint32_t severity = (msg_id >> MPSL_LOG_SEVERITY_SHIFT) & MPSL_LOG_SEVERITY_MASK;

			switch (severity) {
			case 0:
				LOG_INF("%s", msg);
				break;
			case 1:
				LOG_ERR("%s", msg);
				break;
			default:
				LOG_INF("%s", msg);
				break;
			}
		}
	}
}

void mpsl_log_handler(uint32_t msg_id)
{
	(void)k_msgq_put(&mpsl_log_msgq, &msg_id, K_NO_WAIT);
}

static int mpsl_log_deferred_init(void)
{
	k_tid_t tid = k_thread_create(&mpsl_log_thread,
				      mpsl_log_thread_stack,
				      K_THREAD_STACK_SIZEOF(mpsl_log_thread_stack),
				      mpsl_log_thread_fn,
				      NULL, NULL, NULL,
				      K_PRIO_PREEMPT(CONFIG_SYSTEM_WORKQUEUE_PRIORITY), /* TODO: find correct priority*/
				      0, K_NO_WAIT);
	k_thread_name_set(tid, "mpsl_log");

	return 0;
}

SYS_INIT(mpsl_log_deferred_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
