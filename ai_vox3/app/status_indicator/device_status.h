/****************************************************************************
 * device_status.h — 统一状态入口（无 LVGL 依赖版）
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __DEVICE_STATUS_H
#define __DEVICE_STATUS_H

#include "status_bar.h"

void device_status_init(void *unused);
void device_status_set(enum device_status s);
enum device_status device_status_get(void);

#endif
