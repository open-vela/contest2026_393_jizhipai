# ##############################################################################
# Status Indicator - Contest 2026 team 307
# ##############################################################################

if(CONFIG_LVX_CONTEST2026_307_STATUS_INDICATOR)
  nuttx_add_application(
    NAME
    ${CONFIG_LVX_CONTEST2026_307_STATUS_INDICATOR_PROGNAME}
    PRIORITY
    ${CONFIG_LVX_CONTEST2026_307_STATUS_INDICATOR_PRIORITY}
    STACKSIZE
    ${CONFIG_LVX_CONTEST2026_307_STATUS_INDICATOR_STACKSIZE}
    MODULE
    ${CONFIG_LVX_CONTEST2026_307_STATUS_INDICATOR}
    SRCS
    status_indicator_main.c
    status_bar.c
    motor.c
    device_status.c
    test_status_page.c)
endif()
