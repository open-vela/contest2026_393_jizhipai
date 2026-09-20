/****************************************************************************
 * apps/packages/ai_agent/src/tools/tool_weather.c
 *
 * Desktop companion "refresh_weather" agent tool implementation.
 *
 * Fetches current weather from Open-Meteo (https://api.open-meteo.com,
 * no API key required) for the configured city, maps the WMO weather code
 * to a short Chinese label, and writes /tmp/weather.txt as:
 *
 *     <weather_text> <temp_celsius>\n
 *
 * e.g.  "晴 25"
 *
 * On any failure the file is intentionally NOT overwritten, so a previously
 * valid reading (if present) keeps displaying instead of flipping to "--".
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pthread.h>

/* vela HTTPS helper (mbedTLS backed, no CONFIG_NET_TLS needed) */
#include "infra/vela_tls.h"

/* cJSON for parsing the Open-Meteo response */
#include "cJSON.h"

#include "tool_weather.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifndef CONFIG_AI_AGENT_WEATHER_LAT
#  define WEATHER_LAT "39.9042"   /* Beijing, fallback */
#else
#  define WEATHER_LAT CONFIG_AI_AGENT_WEATHER_LAT
#endif

#ifndef CONFIG_AI_AGENT_WEATHER_LON
#  define WEATHER_LON "116.4074"  /* Beijing, fallback */
#else
#  define WEATHER_LON CONFIG_AI_AGENT_WEATHER_LON
#endif

#define OPEN_METEO_HOST "api.open-meteo.com"
#define OPEN_METEO_PORT "443"
#define RESP_BUF_LEN    2048

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Map a WMO weather interpretation code to a short Chinese label.
 * Reference: https://open-meteo.com/en/docs (WMO Weather interpretation codes)
 */
static const char *wmo_text(int code)
{
  switch (code)
    {
      case 0:
        return "晴";
      case 1:
      case 2:
        return "多云";
      case 3:
        return "阴";
      case 45:
      case 48:
        return "雾";
      case 51:
      case 53:
      case 55:
      case 56:
      case 57:
      case 61:
      case 63:
      case 65:
      case 66:
      case 67:
      case 80:
      case 81:
      case 82:
      case 95:
      case 96:
      case 99:
        return "雨";
      case 71:
      case 73:
      case 75:
      case 77:
      case 85:
      case 86:
        return "雪";
      default:
        return "未知";
    }
}

/* Fetch weather from Open-Meteo and write /tmp/weather.txt.
 * Returns 0 on success, -1 on failure (file NOT written on failure).
 */
static int weather_fetch_and_write(void)
{
  char path[256];
  char buf[RESP_BUF_LEN];
  int ret = -1;

  snprintf(path, sizeof(path),
           "/v1/forecast?latitude=%s&longitude=%s"
           "&current=temperature_2m,weather_code",
           WEATHER_LAT, WEATHER_LON);

  memset(buf, 0, sizeof(buf));
  int http_ret = vela_https_get(OPEN_METEO_HOST, OPEN_METEO_PORT,
                                path, buf, sizeof(buf) - 1);
  if (http_ret < 0)
    {
      fprintf(stderr, "weather: https get failed (%d)\n", http_ret);
      return -1;
    }

  cJSON *root = cJSON_Parse(buf);
  if (root == NULL)
    {
      fprintf(stderr, "weather: json parse failed\n");
      return -1;
    }

  cJSON *current = cJSON_GetObjectItem(root, "current");
  if (current == NULL)
    {
      fprintf(stderr, "weather: no 'current' in response\n");
      cJSON_Delete(root);
      return -1;
    }

  cJSON *temp = cJSON_GetObjectItem(current, "temperature_2m");
  cJSON *code = cJSON_GetObjectItem(current, "weather_code");
  if (temp == NULL || code == NULL ||
      !cJSON_IsNumber(temp) || !cJSON_IsNumber(code))
    {
      fprintf(stderr, "weather: missing temperature_2m/weather_code\n");
      cJSON_Delete(root);
      return -1;
    }

  const char *label = wmo_text(code->valueint);
  int t = (int)(temp->valuedouble);

  FILE *f = fopen(WEATHER_FILE_PATH, "w");
  if (f == NULL)
    {
      fprintf(stderr, "weather: cannot open %s for write\n", WEATHER_FILE_PATH);
      cJSON_Delete(root);
      return -1;
    }

  fprintf(f, "%s %d\n", label, t);
  fclose(f);

  cJSON_Delete(root);
  ret = 0;
  fprintf(stderr, "weather: updated -> %s %d\n", label, t);
  return ret;
}

/* Background refresh thread entry point. */
static void *weather_thread(void *arg)
{
  UNUSED(arg);

  sleep(WEATHER_FIRST_DELAY_SEC);
  for (;;)
    {
      weather_fetch_and_write();
      sleep(WEATHER_REFRESH_INTERVAL_SEC);
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tool_refresh_weather_execute(const char *input_json,
                                 char *output, size_t output_size)
{
  int ret = weather_fetch_and_write();

  if (ret == 0)
    {
      if (output && output_size > 0)
        {
          snprintf(output, output_size,
                   "{\"status\":\"ok\",\"msg\":\"weather refreshed\"}");
        }
      return 0;
    }
  else
    {
      if (output && output_size > 0)
        {
          snprintf(output, output_size,
                   "{\"status\":\"error\",\"msg\":\"weather fetch failed\"}");
        }
      return -1;
    }
}

void weather_tool_init(void)
{
  pthread_t tid;
  pthread_attr_t attr;

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 2048);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  if (pthread_create(&tid, &attr, weather_thread, NULL) != 0)
    {
      fprintf(stderr, "weather: failed to start refresh thread\n");
    }

  pthread_attr_destroy(&attr);
}
