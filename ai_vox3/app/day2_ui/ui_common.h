/****************************************************************************
 * ui_common.h — UI 公共定义与工具
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __UI_COMMON_H
#define __UI_COMMON_H

#include <lvgl.h>

/****************************************************************************
 * 屏幕参数（黄山派 390×450 AMOLED）
 ****************************************************************************/

#define SCREEN_WIDTH    390
#define SCREEN_HEIGHT   450

/****************************************************************************
 * 颜色定义（RGB565 兼容）
 ****************************************************************************/

#define COLOR_BG_DARK     lv_color_hex(0x1A1A2E)   /* 深色背景 */
#define COLOR_BG_CARD     lv_color_hex(0x16213E)   /* 卡片背景 */
#define COLOR_TEXT_WHITE   lv_color_hex(0xFFFFFF)   /* 白色文字 */
#define COLOR_TEXT_GRAY    lv_color_hex(0xAAAAAA)   /* 灰色文字 */
#define COLOR_TEXT_GREEN   lv_color_hex(0x00E676)   /* 绿色强调 */
#define COLOR_TEXT_BLUE    lv_color_hex(0x2979FF)   /* 蓝色强调 */
#define COLOR_TEXT_RED     lv_color_hex(0xFF1744)   /* 红色警告 */
#define COLOR_ACCENT       lv_color_hex(0x00E676)   /* 主题色 */

/****************************************************************************
 * 字体路径（FreeType 中文字体）
 *
 * 字体文件已打包到 ROM 文件系统的 /etc/data/font/ 目录
 ****************************************************************************/

#define FONT_PATH_MISANS  "/etc/data/font/MiSans-Regular.ttf"

/****************************************************************************
 * 全局字体指针（由 ui_font_init 初始化）
 ****************************************************************************/

extern const lv_font_t *font_cn_large;    /* 28px */
extern const lv_font_t *font_cn_medium;   /* 20px */
extern const lv_font_t *font_cn_small;    /* 16px */
extern const lv_font_t *font_cn_tiny;     /* 14px */

/****************************************************************************
 * 兼容旧代码的字体宏
 ****************************************************************************/

#define FONT_LARGE    font_cn_large
#define FONT_MEDIUM   font_cn_medium
#define FONT_SMALL    font_cn_small
#define FONT_TINY     font_cn_tiny

/****************************************************************************
 * 公共工具函数
 ****************************************************************************/

/**
 * 初始化 FreeType 中文字体（必须在 LVGL 初始化后调用）
 * 返回 0 成功，负数失败
 */

int ui_font_init(void);

/**
 * 释放 FreeType 字体资源
 */

void ui_font_deinit(void);

/**
 * 创建一个带圆角的卡片容器
 */

lv_obj_t *ui_create_card(lv_obj_t *parent, lv_coord_t w, lv_coord_t h);

/**
 * 创建一个标签
 */

lv_obj_t *ui_create_label(lv_obj_t *parent, const char *text,
                          const lv_font_t *font, lv_color_t color);

/**
 * 创建一个按钮
 */

lv_obj_t *ui_create_button(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
                           const char *text, lv_event_cb_t cb);

#endif
