/**
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief        Register network media source for network play
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to register
 */
int media_src_register_network();

#ifdef __cplusplus
}
#endif
