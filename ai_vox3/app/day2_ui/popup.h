/****************************************************************************
 * popup.h — 弹窗组件
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __POPUP_H
#define __POPUP_H

#include <lvgl.h>

/**
 * 弹窗确认回调
 */

typedef void (*popup_cb_t)(void *user_data);

/**
 * 创建弹窗
 * @param parent    父对象（通常是屏幕）
 * @param title     标题文字
 * @param content   内容文字
 * @param btn_text  按钮文字（NULL 则默认"确认"）
 * @param cb        确认回调（可为 NULL）
 * @param user_data 回调用户数据
 * @return 弹窗对象（可用 lv_obj_del 删除）
 */

lv_obj_t *popup_create(lv_obj_t *parent, const char *title,
                       const char *content, const char *btn_text,
                       popup_cb_t cb, void *user_data);

/**
 * 关闭弹窗（带动画）
 */

void popup_close(lv_obj_t *popup);

#endif
