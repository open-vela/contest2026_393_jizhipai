############################################################################
# Day 2 UI Demo - Contest 2026 team 307
############################################################################

include $(APPDIR)/Make.defs

# 主程序：day2_ui

PROGNAME  = $(CONFIG_LVX_CONTEST2026_307_DAY2_UI_PROGNAME) touch_diag
PRIORITY  = $(CONFIG_LVX_CONTEST2026_307_DAY2_UI_PRIORITY) 100
STACKSIZE = $(CONFIG_LVX_CONTEST2026_307_DAY2_UI_STACKSIZE) 4096
MODULE    = $(CONFIG_LVX_CONTEST2026_307_DAY2_UI)

MAINSRC   = day2_ui_main.c touch_diag.c
CSRCS     = ui_common.c \
            clock_widget.c \
            popup.c \
            reminder_page.c

include $(APPDIR)/Application.mk
