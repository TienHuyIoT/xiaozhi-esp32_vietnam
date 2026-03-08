/**
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */

#pragma once

#include "media_src.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief        Register storage media source for local play
 *
 * @return
 *               - 0: On success
 *               - Others: Fail to register
 */
int media_src_register_storage();

#ifdef __cplusplus
}
#endif
