# My AI Tool - Contest 2026 team 307
if(CONFIG_LVX_CONTEST2026_307_MY_AI_TOOL)
  nuttx_add_application(
    NAME my_ai_tool
    SRCS my_ai_tool_main.c
    STACKSIZE 4096)
endif()
