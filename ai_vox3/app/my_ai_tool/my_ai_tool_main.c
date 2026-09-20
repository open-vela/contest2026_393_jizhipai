/****************************************************************************
 * Contest 2026 team 307 - My AI Tool
 ****************************************************************************/

#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
  printf("=== My AI Tool (team 307) ===\n");

  if (argc < 2)
    {
      printf("Usage: my_ai_tool <command>\n");
      printf("  hello   - Say hello\n");
      printf("  info    - Show system info\n");
      return 0;
    }

  if (strcmp(argv[1], "hello") == 0)
    {
      printf("Hello from team 307!\n");
    }
  else if (strcmp(argv[1], "info") == 0)
    {
      printf("Board: lckfb_huangshan_pi (SF32LB52)\n");
      printf("Team: 307 justdoit\n");
    }
  else
    {
      printf("Unknown command: %s\n", argv[1]);
    }

  return 0;
}
