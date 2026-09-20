/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_lcd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <unistd.h>      /* [V326] usleep */
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7789.h>

#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "esp32s3_spi.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include "esp32s3-eye.h"

/* [LCDDIAG] Board-level LCD / backlight / SPI pin tracing (black-screen hunt).
 * Prints the physical pin numbers plus their read-back levels so we can tell
 * whether the backlight (GPIO16) is really driven high, whether the shared
 * DC/MISO pin (GPIO14) ended up as a plain GPIO output, and whether the SPI3
 * bus pins were (re)configured successfully.
 */

#define LCDDIAG(fmt, ...) \
  syslog(LOG_ERR, "[LCDDIAG] board: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd;

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name:  lcd_selftest
 *
 * Description:
 *   AI-VOX3 boot self-test: power on the panel (DISPON) and sweep
 *   full-screen RED -> GREEN -> BLUE -> WHITE, then leave BLUE.
 *   Without setpower() the ST7789 stays in display-off state (black
 *   screen even though the driver initialized fine).
 *
 ****************************************************************************/

#ifdef AI_VOX3_LCD_SELFTEST

static FAR void *lcd_demo_thread(FAR void *arg)
{
  static const uint16_t colors[3] = { 0x001f, 0xf800, 0x07e0 }; /* B R G */
  struct lcd_planeinfo_s pinfo;
  uint16_t linebuf[240];
  int i;
  int row;
  int c = 0;

  if (!g_lcd)
    {
      return NULL;
    }

  if (g_lcd->getplaneinfo(g_lcd, 0, &pinfo) != OK || pinfo.putrun == NULL)
    {
      return NULL;
    }

  for (; ; )
    {
      for (i = 0; i < 240; i++)
        {
          linebuf[i] = colors[c % 3];
        }

      for (row = 0; row < 240; row++)
        {
          pinfo.putrun(g_lcd, row, 0, (FAR const uint8_t *)linebuf, 240);
        }

      c++;
      nxsig_sleep(2);
    }

  return NULL; /* not reached */
}

static void lcd_selftest(void)
{
  if (!g_lcd)
    {
      return;
    }

  /* Power on the display: st7789_setpower() issues DISPON (0x29). */

  g_lcd->setpower(g_lcd, CONFIG_LCD_MAXPOWER);

  kthread_create("lcddemo", SCHED_PRIORITY_DEFAULT, 2048,
                 (main_t)lcd_demo_thread, NULL);
}

#endif /* AI_VOX3_LCD_SELFTEST */

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

/* Re-assert the LCD backlight (GPIO16, active HIGH per AI-VOX3 ref). */
/* [V326] 保活任务：启动期间反复重夺背光脚。
 * 只运行 5 秒（10 次 x 500ms）后自行退出，不是常驻任务。 */
static void board_lcd_bckl_keeper(void)
{
  int i;

  for (i = 0; i < 10; i++)
    {
      esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, OUTPUT | INPUT);
      esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, true);
      usleep(500000);
    }

  LCDDIAG("[V326] bckl keeper done: BCKL(%d) level=%d",
          ESP32S3_EYE_DISPLAY_BCKL,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));
}

void board_lcd_backlight_on(void)
{
  /* [V326] 配成 OUTPUT|INPUT：只配 OUTPUT 时输入缓冲关闭，gpioread 读不到
   * 真实电平（会把"其实已经拉高"误报成 0）。 */
  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, OUTPUT | INPUT);
  esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, true);
}

int board_lcd_initialize(void)
{
  /* Initialize non-SPI GPIOs */

  LCDDIAG("pins: DC=%d BCKL=%d SPI=%d | SPI3 cs=%d clk=%d mosi=%d miso(=DC)=%d",
          ESP32S3_EYE_DISPLAY_DC, ESP32S3_EYE_DISPLAY_BCKL,
          ESP32S3_EYE_DISPLAY_SPI,
          CONFIG_ESP32S3_SPI3_CSPIN, CONFIG_ESP32S3_SPI3_CLKPIN,
          CONFIG_ESP32S3_SPI3_MOSIPIN, CONFIG_ESP32S3_SPI3_MISOPIN);

  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_DC, OUTPUT);
  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_BCKL, OUTPUT | INPUT);  /* [V326] */
  LCDDIAG("[V326] BCKL attr = OUTPUT|INPUT (input buffer on -> read is real)");

  LCDDIAG("after configgpio: DC(%d) level=%d, BCKL(%d) level=%d",
          ESP32S3_EYE_DISPLAY_DC,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_DC),
          ESP32S3_EYE_DISPLAY_BCKL,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));

  /* Turn on LCD backlight */

  esp32s3_gpiowrite(ESP32S3_EYE_DISPLAY_BCKL, true); /* AI-VOX3: active HIGH */

  LCDDIAG("after BCKL=1: BCKL(%d) level=%d (expect 1 = backlight ON)",
          ESP32S3_EYE_DISPLAY_BCKL,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));

  g_spidev = esp32s3_spibus_initialize(ESP32S3_EYE_DISPLAY_SPI);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n",
             ESP32S3_EYE_DISPLAY_SPI);
      return -ENODEV;
    }

  LCDDIAG("spibus_initialize(%d)=%p", ESP32S3_EYE_DISPLAY_SPI,
          (FAR void *)g_spidev);

  /* Re-claim DC: SPI bus init reconfigures GPIO14 as MISO (LCD needs DC). */
  esp32s3_configgpio(ESP32S3_EYE_DISPLAY_DC, OUTPUT);

  LCDDIAG("after re-claim DC: DC(%d) level=%d", ESP32S3_EYE_DISPLAY_DC,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_DC));

  g_lcd = st7789_lcdinitialize(g_spidev);
  if (!g_lcd)
    {
      lcderr("ERROR: st7789_lcdinitialize() failed\n");
      return -ENODEV;
    }

  LCDDIAG("st7789_lcdinitialize()=%p", (FAR void *)g_lcd);

  board_lcd_backlight_on();

  /* [V326] 真正测一次背光电平（此刻才是最终状态） */
  LCDDIAG("[V326] after board_lcd_backlight_on: BCKL(%d) level=%d (expect 1)",
          ESP32S3_EYE_DISPLAY_BCKL,
          (int)esp32s3_gpioread(ESP32S3_EYE_DISPLAY_BCKL));

  {
    int kpid;

    /* [V326] 保活：启动期反复重夺背光脚（5 秒后自行退出） */
    kpid = kthread_create("bcklkeep", 100, 2048,
                          (main_t)board_lcd_bckl_keeper, NULL);
    LCDDIAG("[V326] bckl keeper pid=%d", kpid);
  }

#ifdef AI_VOX3_LCD_SELFTEST
  lcd_selftest();
#endif

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 * Input Parameters:
 *   devno - LCD device nmber
 *
 * Returned Value:
 *   LCD device pointer if success or NULL if failed.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n",
             ESP32S3_EYE_DISPLAY_SPI, devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n",
              ESP32S3_EYE_DISPLAY_SPI,
              devno);
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the display off */

  g_lcd->setpower(g_lcd, 0);
}
