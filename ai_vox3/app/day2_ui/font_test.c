/****************************************************************************
 * font_test.c — 中文字体测试
 *
 * 用法：nsh> font_test
 *
 * 功能：
 *   - 初始化 LVGL + 显示
 *   - 测试 FreeType 中文字体加载
 *   - 显示中文测试文字
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>

#include <lvgl/lvgl.h>

#include "ui_common.h"

/****************************************************************************
 * 主函数
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  lv_nuttx_dsc_t    dsc;
  lv_nuttx_result_t result;
  uint32_t          idle;

  printf("=== Chinese Font Test (Team 307) ===\n");

  /* 检查 LVGL 是否已初始化 */

  if (lv_is_initialized())
    {
      printf("ERROR: LVGL already initialized by another app.\n");
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

  printf("Initializing Chinese fonts...\n");
  if (ui_font_init() < 0)
    {
      printf("ERROR: Chinese font init failed!\n");
    }
  else
    {
      printf("Chinese fonts initialized successfully.\n");
    }

  /* 创建测试界面 */

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, COLOR_BG_DARK, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  /* 标题 */

  lv_obj_t *title = ui_create_label(scr, "中文字体测试",
                                    FONT_LARGE, COLOR_TEXT_WHITE);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

  /* 测试文字 */

  lv_obj_t *test1 = ui_create_label(scr, "课程提醒：高等数学",
                                    FONT_MEDIUM, COLOR_TEXT_GREEN);
  lv_obj_align(test1, LV_ALIGN_TOP_MID, 0, 80);

  lv_obj_t *test2 = ui_create_label(scr, "地点：教学楼 A302",
                                    FONT_MEDIUM, COLOR_TEXT_BLUE);
  lv_obj_align(test2, LV_ALIGN_TOP_MID, 0, 120);

  lv_obj_t *test3 = ui_create_label(scr, "时间：14:00 - 15:35",
                                    FONT_MEDIUM, COLOR_TEXT_GRAY);
  lv_obj_align(test3, LV_ALIGN_TOP_MID, 0, 160);

  lv_obj_t *test4 = ui_create_label(scr, "这是一条测试弹窗消息",
                                    FONT_SMALL, COLOR_TEXT_WHITE);
  lv_obj_align(test4, LV_ALIGN_TOP_MID, 0, 200);

  lv_obj_t *test5 = ui_create_label(scr, "久坐提醒：请起身活动",
                                    FONT_SMALL, COLOR_TEXT_RED);
  lv_obj_align(test5, LV_ALIGN_TOP_MID, 0, 240);

  /* 底部提示 */

  lv_obj_t *hint = ui_create_label(scr, "如果显示方框，请检查字体文件",
                                   FONT_TINY, COLOR_TEXT_GRAY);
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);

  printf("Test UI created. Main loop running.\n");

  /* 主循环 */

  while (1)
    {
      idle = lv_timer_handler();
      idle = idle ? idle : 1;
      usleep(idle * 1000);
    }

  /* 清理 */

  ui_font_deinit();
  lv_nuttx_deinit(&result);
  lv_deinit();

  return 0;
}
