/****************************************************************************
 * status_bar.h — 顶部 4px 状态色带组件（framebuffer 直绘版）
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#ifndef __STATUS_BAR_H
#define __STATUS_BAR_H

#include <stdint.h>

enum device_status
{
  STATUS_NORMAL,  /* 绿色，静态 */
  STATUS_NOTIFY,  /* 蓝色，闪烁 3 次后保持 */
  STATUS_ALERT,   /* 红色，呼吸循环 */
  STATUS_SLEEP    /* 隐藏（息屏） */
};

/**
 * 初始化色带（framebuffer 模式）
 * @param fbmem   mmap 后的 framebuffer 地址
 * @param xres    屏幕宽度
 * @param yres    屏幕高度
 * @param stride  行字节数
 * @param fb_fd   framebuffer 设备 fd（用于 FBIO_UPDATE）
 */

void status_bar_init_fb(uint16_t *fbmem, uint32_t xres, uint32_t yres,
                        uint32_t stride, int fb_fd);

void status_bar_set(enum device_status s);
void status_bar_acknowledge(void);
void status_bar_hide(void);
void status_bar_show(void);
enum device_status status_bar_get(void);

#endif
