# ##############################################################################
# Day 2 UI Demo - Contest 2026 team 307
# ##############################################################################

if(CONFIG_LVX_CONTEST2026_307_DAY2_UI)
  nuttx_add_application(
    NAME
    ${CONFIG_LVX_CONTEST2026_307_DAY2_UI_PROGNAME}
    PRIORITY
    ${CONFIG_LVX_CONTEST2026_307_DAY2_UI_PRIORITY}
    STACKSIZE
    ${CONFIG_LVX_CONTEST2026_307_DAY2_UI_STACKSIZE}
    MODULE
    ${CONFIG_LVX_CONTEST2026_307_DAY2_UI}
    SRCS
    day2_ui_main.c
    ui_common.c
    clock_widget.c
    popup.c
    reminder_page.c)

  nuttx_add_application(
    NAME
    touch_diag
    PRIORITY
    100
    STACKSIZE
    4096
    MODULE
    ${CONFIG_LVX_CONTEST2026_307_DAY2_UI}
    SRCS
    touch_diag.c)
endif()
