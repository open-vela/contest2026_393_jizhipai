/****************************************************************************
 * ui_common.c — UI 公共工具函数
 * Contest 2026 Team 307
 ****************************************************************************/

#include <stdio.h>
#include "ui_common.h"

/****************************************************************************
 * 全局字体指针
 ****************************************************************************/

const lv_font_t *font_cn_large  = NULL;
const lv_font_t *font_cn_medium = NULL;
const lv_font_t *font_cn_small  = NULL;
const lv_font_t *font_cn_tiny   = NULL;

/****************************************************************************
 * FreeType 字体初始化
 ****************************************************************************/

#if LV_USE_FREETYPE

/* FreeType 字体对象（需要保持存活） */

static lv_font_t *g_ft_font_28 = NULL;
static lv_font_t *g_ft_font_20 = NULL;
static lv_font_t *g_ft_font_16 = NULL;
static lv_font_t *g_ft_font_14 = NULL;

static lv_font_t *create_ft_font(const char *path, uint16_t size)
{
  lv_font_t *font = lv_freetype_font_create(path,
                                            LV_FREETYPE_FONT_RENDER_MODE_BITMAP,
                                            size,
                                            LV_FREETYPE_FONT_STYLE_NORMAL);
  if (font == NULL)
    {
      printf("ERROR: Failed to create FreeType font %s at %dpx\n", path, size);
    }
  else
    {
      printf("FreeType font loaded: %s at %dpx\n", path, size);
    }

  return font;
}

#endif /* LV_USE_FREETYPE */

int ui_font_init(void)
{
#if LV_USE_FREETYPE
  lv_result_t res;

  printf("Initializing FreeType library...\n");

  /* 初始化 FreeType 库（如果已初始化则跳过） */

  res = lv_freetype_init(256);  /* 缓存 256 个字形 */
  if (res != LV_RESULT_OK)
    {
      printf("FreeType already initialized, reusing existing context.\n");
    }

  printf("FreeType library initialized. Loading fonts from %s...\n",
         FONT_PATH_MISANS);

  /* 检查字体文件是否存在 */

  FILE *f = fopen(FONT_PATH_MISANS, "rb");
  if (f == NULL)
    {
      printf("ERROR: Font file not found: %s\n", FONT_PATH_MISANS);
      printf("Please ensure the font file is in the ROM filesystem.\n");
      goto use_builtin_fonts;
    }
  fclose(f);
  printf("Font file found: %s\n", FONT_PATH_MISANS);

  g_ft_font_28 = create_ft_font(FONT_PATH_MISANS, 28);
  g_ft_font_20 = create_ft_font(FONT_PATH_MISANS, 20);
  g_ft_font_16 = create_ft_font(FONT_PATH_MISANS, 16);
  g_ft_font_14 = create_ft_font(FONT_PATH_MISANS, 14);

  /* 设置全局字体指针 */

  font_cn_large  = g_ft_font_28 ? g_ft_font_28 : &lv_font_montserrat_28;
  font_cn_medium = g_ft_font_20 ? g_ft_font_20 : &lv_font_montserrat_20;
  font_cn_small  = g_ft_font_16 ? g_ft_font_16 : &lv_font_montserrat_16;
  font_cn_tiny   = g_ft_font_14 ? g_ft_font_14 : &lv_font_montserrat_14;

  if (g_ft_font_28 && g_ft_font_20 && g_ft_font_16 && g_ft_font_14)
    {
      printf("All FreeType fonts loaded successfully.\n");
      return 0;
    }

  printf("WARNING: Some fonts failed to load, using fallback.\n");
  return -1;

use_builtin_fonts:
#else
  printf("FreeType not enabled, using built-in fonts (no Chinese support).\n");
#endif

  font_cn_large  = &lv_font_montserrat_28;
  font_cn_medium = &lv_font_montserrat_20;
  font_cn_small  = &lv_font_montserrat_16;
  font_cn_tiny   = &lv_font_montserrat_14;
  return 0;
}

void ui_font_deinit(void)
{
#if LV_USE_FREETYPE
  if (g_ft_font_28) lv_freetype_font_delete(g_ft_font_28);
  if (g_ft_font_20) lv_freetype_font_delete(g_ft_font_20);
  if (g_ft_font_16) lv_freetype_font_delete(g_ft_font_16);
  if (g_ft_font_14) lv_freetype_font_delete(g_ft_font_14);

  g_ft_font_28 = NULL;
  g_ft_font_20 = NULL;
  g_ft_font_16 = NULL;
  g_ft_font_14 = NULL;
  font_cn_large  = NULL;
  font_cn_medium = NULL;
  font_cn_small  = NULL;
  font_cn_tiny   = NULL;

  lv_freetype_uninit();
#endif
}

/****************************************************************************
 * UI 工具函数
 ****************************************************************************/

lv_obj_t *ui_create_card(lv_obj_t *parent, lv_coord_t w, lv_coord_t h)
{
  lv_obj_t *card = lv_obj_create(parent);
  lv_obj_set_size(card, w, h);
  lv_obj_set_style_bg_color(card, COLOR_BG_CARD, 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_style_pad_all(card, 16, 0);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  return card;
}

lv_obj_t *ui_create_label(lv_obj_t *parent, const char *text,
                          const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  return label;
}

lv_obj_t *ui_create_button(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
                           const char *text, lv_event_cb_t cb)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_style_bg_color(btn, COLOR_ACCENT, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(btn, 8, 0);
  lv_obj_set_style_border_width(btn, 0, 0);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, FONT_MEDIUM, 0);
  lv_obj_set_style_text_color(label, COLOR_BG_DARK, 0);
  lv_obj_center(label);

  if (cb)
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  return btn;
}
