/****************************************************************************
 * clock_widget.h — 时钟组件（RTC 实时更新）
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __CLOCK_WIDGET_H
#define __CLOCK_WIDGET_H

#include <lvgl.h>

/**
 * 创建时钟组件
 * @param parent 父对象
 * @return 时钟容器对象
 *
 * 显示：时间（大字）+ 日期 + 星期
 * 每秒自动更新
 */

lv_obj_t *clock_widget_create(lv_obj_t *parent);

#endif
