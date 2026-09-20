/****************************************************************************
 * popup.c — 弹窗组件
 *
 * 结构：半透明遮罩 + 白色卡片 + 标题 + 内容 + 确认按钮
 * 支持：点击按钮关闭，回调通知
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include "ui_common.h"
#include "popup.h"

/****************************************************************************
 * 弹窗上下文
 ****************************************************************************/

struct popup_ctx
{
  popup_cb_t  cb;
  void       *user_data;
  lv_obj_t   *popup;      /* 最外层遮罩 */
};

/****************************************************************************
 * 按钮回调
 ****************************************************************************/

static void btn_confirm_cb(lv_event_t *e)
{
  struct popup_ctx *ctx = (struct popup_ctx *)lv_event_get_user_data(e);
  if (ctx->cb)
    ctx->cb(ctx->user_data);
  popup_close(ctx->popup);
  lv_free(ctx);
}

static void overlay_click_cb(lv_event_t *e)
{
  /* 点击遮罩空白区域也关闭 */

  struct popup_ctx *ctx = (struct popup_ctx *)lv_event_get_user_data(e);
  popup_close(ctx->popup);
  lv_free(ctx);
}

/****************************************************************************
 * 公开接口
 ****************************************************************************/

lv_obj_t *popup_create(lv_obj_t *parent, const char *title,
                       const char *content, const char *btn_text,
                       popup_cb_t cb, void *user_data)
{
  struct popup_ctx *ctx = lv_malloc(sizeof(struct popup_ctx));
  if (!ctx) return NULL;
  ctx->cb       = cb;
  ctx->user_data = user_data;

  /* 全屏半透明背景（简化版，无 card 嵌套） */

  lv_obj_t *overlay = lv_obj_create(parent);
  lv_obj_set_size(overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(overlay, LV_OPA_80, 0);
  lv_obj_set_style_border_width(overlay, 0, 0);
  lv_obj_set_style_radius(overlay, 0, 0);
  lv_obj_remove_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(overlay, LV_ALIGN_TOP_LEFT, 0, 0);

  /* Flex 布局：垂直居中 */

  lv_obj_set_flex_flow(overlay, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(overlay, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(overlay, 16, 0);

  ctx->popup = overlay;

  /* 点击任意位置关闭 */

  lv_obj_add_event_cb(overlay, overlay_click_cb, LV_EVENT_CLICKED, ctx);

  /* 标题 */

  ui_create_label(overlay, title, FONT_LARGE, COLOR_TEXT_WHITE);

  /* 内容 */

  ui_create_label(overlay, content, FONT_MEDIUM, COLOR_TEXT_GRAY);

  /* 确认按钮 */

  const char *text = btn_text ? btn_text : "确认";
  ui_create_button(overlay, 200, 44, text, btn_confirm_cb);

  return overlay;
}

void popup_close(lv_obj_t *popup)
{
  if (popup)
    lv_obj_del(popup);
}
