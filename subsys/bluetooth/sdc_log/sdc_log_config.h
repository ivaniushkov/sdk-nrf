/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Maps the RTOS-agnostic settings used by the generated sdc_log_msg.h onto Zephyr
 * Kconfig. Included before sdc_log_msg.h.
 */

#ifndef SDC_LOG_CONFIG_H__
#define SDC_LOG_CONFIG_H__

#include <zephyr/autoconf.h>

#define SDC_LOG_PRINT_LEVEL           CONFIG_BT_CTLR_SDC_LOG_PRINT_LEVEL

#endif /* SDC_LOG_CONFIG_H__ */
