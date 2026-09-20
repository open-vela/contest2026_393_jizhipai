/****************************************************************************
 * day2_ui_main.c — Day 2 UI 主入口
 *
 * 用法：nsh> day2_ui
 *
 * 功能：
 *   - 初始化 LVGL + 显示
 *   - 初始化 FreeType 中文字体
 *   - 创建主界面（时钟 + 测试按钮）
 *   - 测试弹窗和提醒页面
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>

#include <lvgl/lvgl.h>

#include "ui_common.h"
#include "clock_widget.h"
#include "popup.h"
#include "reminder_page.h"

/****************************************************************************
 * 按钮回调
 ****************************************************************************/

static void btn_popup_cb(lv_event_t *e)
{
  lv_obj_t *scr = lv_screen_active();
  popup_create(scr, "提示", "这是一条测试弹窗消息",
               "我知道了", NULL, NULL);
}

static void btn_reminder_cb(lv_event_t *e)
{
  lv_obj_t *scr = lv_screen_active();
  reminder_page_create(scr, "高等数学", "教学楼 A302", "14:00 - 15:35");
}

static void btn_reminder2_cb(lv_event_t *e)
{
  lv_obj_t *scr = lv_screen_active();
  reminder_page_create(scr, "嵌入式系统", "实验楼 B201", "16:00 - 17:35");
}

/****************************************************************************
 * 创建主界面
 ****************************************************************************/

static void create_main_ui(void)
{
  lv_obj_t *scr = lv_screen_active();

  /* 深色背景 */

  lv_obj_set_style_bg_color(scr, COLOR_BG_DARK, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  /* 时钟组件 */

  clock_widget_create(scr);

  /* 测试按钮区域 */

  lv_obj_t *btn_cont = lv_obj_create(scr);
  lv_obj_set_size(btn_cont, 350, 240);
  lv_obj_align(btn_cont, LV_ALIGN_TOP_MID, 0, 210);
  lv_obj_set_style_bg_opa(btn_cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(btn_cont, 0, 0);
  lv_obj_set_style_pad_all(btn_cont, 0, 0);
  lv_obj_remove_flag(btn_cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(btn_cont, 12, 0);

  /* 测试按钮 */

  ui_create_button(btn_cont, 280, 46, "课程提醒：高数", btn_reminder_cb);
  ui_create_button(btn_cont, 280, 46, "课程提醒：嵌入式", btn_reminder2_cb);

  /* 底部版本信息 */

  lv_obj_t *ver = ui_create_label(scr, "Day 2 UI Demo — Team 307",
                                  FONT_TINY, COLOR_TEXT_GRAY);
  lv_obj_align(ver, LV_ALIGN_BOTTOM_MID, 0, -16);
}

/****************************************************************************
 * 主函数
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  lv_nuttx_dsc_t    dsc;
  lv_nuttx_result_t result;
  uint32_t          idle;

  printf("=== Day 2 UI Demo (Team 307) ===\n");

  /* 检查 LVGL 是否已初始化 */

  if (lv_is_initialized())
    {
      printf("ERROR: LVGL already initialized by another app.\n");
      printf("Please stop the other app first, or disable CONFIG_QUICKAPP.\n");
      return -1;
    }

  /* LVGL 初始化 */

  lv_init();
  lv_nuttx_dsc_init(&dsc);

  dsc.fb_path    = "/dev/lcd0";
  dsc.input_path = "/dev/input0";

  lv_nuttx_init(&dsc, &result);

  if (result.disp == NULL)
    {
      printf("ERROR: display init failed\n");
      return -1;
    }

  printf("Display: %dx%d\n", SCREEN_WIDTH, SCREEN_HEIGHT);

  /* 初始化中文字体 */

  if (ui_font_init() < 0)
    {
      printf("WARNING: Chinese font init failed, some text may not display.\n");
    }

  /* 创建主界面 */

  create_main_ui();

  printf("UI ready. Main loop running.\n");

  /* 主循环 */

  while (1)
    {
      idle = lv_timer_handler();
      idle = idle ? idle : 1;
      usleep(idle * 1000);
    }

  /* 清理（实际上不会执行到） */

  ui_font_deinit();
  lv_nuttx_deinit(&result);
  lv_deinit();

  return 0;
}
