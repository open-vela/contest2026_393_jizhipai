# ##############################################################################
# Info Screen - Contest 2026 team 307
# ##############################################################################

if(CONFIG_LVX_CONTEST2026_307_INFO_SCREEN)
  nuttx_add_application(
    NAME
    ${CONFIG_LVX_CONTEST2026_307_INFO_SCREEN_PROGNAME}
    PRIORITY
    ${CONFIG_LVX_CONTEST2026_307_INFO_SCREEN_PRIORITY}
    STACKSIZE
    ${CONFIG_LVX_CONTEST2026_307_INFO_SCREEN_STACKSIZE}
    MODULE
    ${CONFIG_LVX_CONTEST2026_307_INFO_SCREEN}
    SRCS
    info_screen_main.c
    font16.c)
endif()
