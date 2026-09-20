/****************************************************************************
 * reminder_page.h — 全屏提醒页面
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __REMINDER_PAGE_H
#define __REMINDER_PAGE_H

#include <lvgl.h>

/**
 * 创建全屏课程提醒页面
 * @param parent   父对象（屏幕）
 * @param course   课程名称
 * @param location 地点
 * @param time_str 时间字符串（如 "14:00"）
 * @return 页面对象
 *
 * 显示：大字课程名 + 地点 + 时间 + "确认"按钮
 * 点击确认后自动关闭
 */

lv_obj_t *reminder_page_create(lv_obj_t *parent, const char *course,
                                const char *location, const char *time_str);

#endif
