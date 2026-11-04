############################################################################
# XiaoZhi-style LVGL UI - Contest 2026 team 307
############################################################################

include $(APPDIR)/Make.defs

# [V581] LVGL app: the whole widget tree lives in one C file for now.

PROGNAME  = $(CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI_PROGNAME)
PRIORITY  = $(CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI_PRIORITY)
STACKSIZE = $(CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI_STACKSIZE)
MODULE    = $(CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI)

MAINSRC   = xiaozhi_ui_main.c

# [V583] original xiaozhi subset fonts (LVGL 9 format C files)
CSRCS     = font_desk_20.c font_desk_14.c font_clock_70.c font_num_20.c \
            font_sec_30.c

# [V675] Full GB2312 CJK font (7469 glyphs, 16 px, 2 bpp, ~416 KB of bitmap)
# for the top status strip and the chat subtitle.  The LVGL stock font
# lv_font_simsun_16_cjk carries only 1166 glyphs from a Japanese/traditional
# demo list, so common simplified characters such as 聆/听/说/话 were missing.
CSRCS     += font_cn_16.c

# [V1222] bottom-right astronaut deco (A8 alpha map)
CSRCS     += astronaut_map.c

include $(APPDIR)/Application.mk
