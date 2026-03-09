/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint16_t folder_id;
    char    *name;
} folder_name_t;

typedef struct {
    uint16_t folder_id;
    char    *name;
} file_name_t;

typedef struct {
    folder_name_t *folders;
    file_name_t   *files;
    uint16_t       folder_count;
    uint16_t       file_count;
    uint16_t       folder_alloc;
    uint16_t       file_alloc;
    const char    *ext;
} folder_info_t;

static char path[256];
static folder_info_t folder_info;

int folder_deinit()
{
    int i;
    for (i = 0; i < folder_info.file_count; i++) {
        if (folder_info.files[i].name) {
            free(folder_info.files[i].name);
        }
    }
    for (i = 0; i < folder_info.folder_count; i++) {
        if (folder_info.folders[i].name) {
            free(folder_info.folders[i].name);
        }
    }
    if (folder_info.files) {
        free(folder_info.files);
    }
    if (folder_info.folders) {
        free(folder_info.folders);
    }
    memset(&folder_info, 0, sizeof(folder_info_t));
    return 0;
}

static void d_alloc(void **arr, uint16_t *n, uint16_t *t, int size)
{
    if (*t == 0) {
        *arr = malloc(64 * size);
        if (*arr) {
            *t = 64;
        } else {
            return;
        }
    }
    if ((*n + 1) > *t) {
        *arr = realloc(*arr, (*t << 1) * size);
        if (*arr) {
            *t <<= 1;
        } else {
            *n = *t = 0;
        }
    }
}

static void add_file(uint16_t folder_id, char *name)
{
    d_alloc((void **) &folder_info.files, &folder_info.file_count, &folder_info.file_alloc, sizeof(file_name_t));
    if (folder_info.files) {
        folder_info.files[folder_info.file_count].folder_id = folder_id;
        folder_info.files[folder_info.file_count].name = strdup(name);
        folder_info.file_count++;
    }
}

static void add_folder(uint16_t folder_id, char *name)
{
    d_alloc((void **) &folder_info.folders, &folder_info.folder_count, &folder_info.folder_alloc,
            sizeof(folder_name_t));
    if (folder_info.folders) {
        folder_info.folders[folder_info.folder_count].folder_id = folder_id;
        folder_info.folders[folder_info.folder_count].name = strdup(name);
        folder_info.folder_count++;
    }
}

static char *reverse_add(char *s, char *d, char *l)
{
    if (d == NULL) {
        return NULL;
    }
    int len = strlen(s);
    char *f = d - len;
    if (f > l) {
        memcpy(f, s, len);
        return f;
    }
    return NULL;
}

static char *get_path(uint16_t id, char *file)
{
    char *e = &path[sizeof(path) - 1];
    *(e--) = 0;
    char *s = e;
    if (file) {
        e = reverse_add(file, e, path);
    }
    while (id > 0) {
        id--;
        if (e != s) {
            e = reverse_add("/", e, path);
        }
        e = reverse_add(folder_info.folders[id].name, e, path);
        if (e == NULL) {
            return NULL;
        }
        id = folder_info.folders[id].folder_id;
    }
    return e;
}

static char *lc(char *n)
{
    char *s = path;
    while (*n) {
        *s = *n;
        if (*s >= 'A' && *s <= 'Z')
            *s += 'a' - 'A';
        s++;
        n++;
    }
    *s = 0;
    return path;
}

static int match_ext(const char *s, char *e)
{
    int len = strlen(e);
    char *p = strstr(s, lc(e));
    return (p != NULL && p[len] == ';');
}

static void folder_reader(int folder_id, int depth)
{
    char *folder = get_path(folder_id, NULL);
    printf("Start to read folder %s id:%x\n", folder, folder_id);
    DIR *dir = opendir(folder);
    if (dir == NULL) {
        return;
    }
    struct dirent *c;
    int folder_start = folder_info.folder_count;
    int folder_count = 0;
    while ((c = readdir(dir)) != NULL) {
        if (c->d_type != DT_DIR) {
            char *ext = strchr(c->d_name, '.');
            if (ext && match_ext(folder_info.ext, ext + 1)) {
                add_file(folder_id, c->d_name);
                // printf("file: %s\n", c->d_name);
            }
        } else {
            if (c->d_name[0] == '.') {
                continue;
            }
            add_folder(folder_id, c->d_name);
            // printf("folder: %d %s\n", folder_info.folder_count, c->d_name);
            folder_count++;
        }
    }
    closedir(dir);
    depth++;
    while (folder_count-- > 0) {
        folder_start++;
        folder_reader(folder_start, depth);
    }
}

int folder_init(char *folder, const char *ext)
{
    folder_deinit();
    folder_info.ext = ext;
    add_folder(0, folder);
    folder_reader(1, 0);
    printf("Find total folder:%d file:%d\n", folder_info.folder_count, folder_info.file_count);
    return 0;
}

int folder_get_file_count(char *s)
{
    int match = 0;
    for (int i = 0; i < folder_info.file_count; i++) {
        char *m = get_path(folder_info.files[i].folder_id, folder_info.files[i].name);
        if (strstr(m, s)) {
            printf("%d: %s\n", match, m);
            match++;
        }
    }
    return match;
}

char *folder_get_file(char *s, int n)
{
    int match = 0;
    for (int i = 0; i < folder_info.file_count; i++) {
        char *m = get_path(folder_info.files[i].folder_id, folder_info.files[i].name);
        if (strstr(m, s)) {
            if (match == n) {
                return m;
            }
            match++;
        }
    }
    return NULL;
}
