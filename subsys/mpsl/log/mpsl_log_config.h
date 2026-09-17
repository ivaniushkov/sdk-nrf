/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Maps the RTOS-agnostic settings used by the generated mpsl_log_msg.h / sdc_log_msg.h
 * tables onto Zephyr Kconfig. Included before those headers. A single print level
 * gates the rows of both tables.
 */

#ifndef MPSL_LOG_CONFIG_H__
#define MPSL_LOG_CONFIG_H__

#include <zephyr/autoconf.h>

#define MPSL_LOG_PRINT_LEVEL          CONFIG_MPSL_LOG_PRINT_LEVEL
#define SDC_LOG_PRINT_LEVEL           CONFIG_MPSL_LOG_PRINT_LEVEL

#endif /* MPSL_LOG_CONFIG_H__ */
