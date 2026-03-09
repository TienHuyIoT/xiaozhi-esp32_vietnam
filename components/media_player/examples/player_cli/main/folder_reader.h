/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

int folder_init(char *folder, const char *ext);
int folder_get_file_count(char *s);
char *folder_get_file(char *s, int n);
int folder_deinit();
