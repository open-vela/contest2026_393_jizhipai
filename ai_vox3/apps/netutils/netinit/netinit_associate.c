/****************************************************************************
 * apps/netutils/netinit/netinit_associate.c
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

/* [V100] SSID pinned in source: .config keeps getting reverted externally
 * (observed 31241 / 888 with stale netinit.o), so force the working
 * credentials regardless of what the generated config says. */
#ifdef CONFIG_NETINIT_WAPI_SSID
#  undef CONFIG_NETINIT_WAPI_SSID
#endif
#define CONFIG_NETINIT_WAPI_SSID "3124"
#ifdef CONFIG_NETINIT_WAPI_PASSPHRASE
#  undef CONFIG_NETINIT_WAPI_PASSPHRASE
#endif
#define CONFIG_NETINIT_WAPI_PASSPHRASE "rczzzhl312"

#include <syslog.h>
#include <stdio.h>

#include <unistd.h>
#include <string.h>
#include <errno.h>

/* [V375] link-already-up guard */
#include <netinet/in.h>
#include <arpa/inet.h>
#include "netutils/netlib.h"

/* [V740] clock() for the DHCP quiet window */
#include <time.h>

extern int esp_wifi_sta_is_connected(void);

#include <nuttx/wireless/wireless.h>

#include "wireless/wapi.h"
#include "netutils/netinit.h"

#ifdef CONFIG_WIRELESS_WAPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netinit_associate
 ****************************************************************************/

/* [WIFI-DIAG 2026-09-14] facts about the LAST association attempt; the panel
 * status line in netinit.c renders them as "... L<load> K<psk-match> P<len>".
 */

volatile int g_assoc_load;
volatile int g_assoc_psk_match;
volatile int g_assoc_saved_phraselen;

/* [V740] How long a freshly associated station is left completely alone so
 * the DHCP exchange can finish (milliseconds).
 *
 * A DHCP lease costs a few hundred milliseconds when nothing interrupts it
 * (measured: associate at 11.16 s -> "Network connected" at 11.29 s).  The
 * boot used to spend ~8 s between association and the address because the
 * diag thread re-issued esp_wifi_connect() every ~5 s, and every connect
 * tears the link down and restarts DHCP.  12 s is long enough for a slow
 * DHCP server and short enough that a genuinely dead link still gets the
 * reconnect it needs. */

#define V740_DHCP_QUIET_MS 12000

int netinit_associate(FAR const char *ifname)
{
  struct wpa_wconfig_s conf;
  int ret = -EINVAL;
  FAR void *load;

  syslog(LOG_ERR, "[DIAG] netinit_associate enter ifname=%s\n", ifname);

  /* [V375] Never tear down a link that is already working.
   *
   * See the module comment: rssi=-41 and a first-try DHCP lease prove the
   * air link is healthy, yet something re-issues the association every few
   * seconds and each attempt drops it (reason=2 / reason=205), after which
   * every TCP probe reports -101 ENETUNREACH and TTS/ASR/LLM cannot run.
   *
   * Gating here covers every caller at once.  The check is deliberately
   * conservative: it only short-circuits when the station is associated AND
   * the interface holds a real address.  Lose either and the next call
   * proceeds exactly as before, so a genuinely dead link still recovers. */

  {
    struct in_addr v375_addr;

    memset(&v375_addr, 0, sizeof(v375_addr));

    if (esp_wifi_sta_is_connected() == 1 &&
        netlib_get_ipv4addr(ifname, &v375_addr) == 0 &&
        v375_addr.s_addr != 0)
      {
        
        return 0;
      }
  }

  /* [V740] *** Do not re-connect while DHCP is still in flight. ***
   *
   * The v375 guard above only short-circuits when the station is
   * associated AND already holds an address - but "has an address" is
   * exactly the thing we are waiting for, so inside the DHCP window it
   * never fires and every call falls through to
   * wpa_driver_wext_associate() -> esp_wifi_connect(), which drops the
   * current association and restarts DHCP from zero.  With the diag
   * thread's ~5 s cadence the lease never got a full window; the boot
   * measured association at 3.12 s and the address at 11.29 s.
   *
   * Bounded: once the station has been associated for
   * V740_DHCP_QUIET_MS without an address, this falls through and the
   * reconnect happens exactly as before, so recovery is never lost. */

  {
    static uint32_t s_v740_assoc_ms;
    uint32_t v740_now;

    v740_now = (uint32_t)(((uint64_t)clock() * 1000u) /
                          (uint64_t)CLOCKS_PER_SEC);

    if (esp_wifi_sta_is_connected() == 1)
      {
        if (s_v740_assoc_ms == 0)
          {
            s_v740_assoc_ms = v740_now ? v740_now : 1u;

            syslog(LOG_ERR, "[V740] associated at t=%u ms: holding the link "
                   "for %d ms so DHCP can finish\n",
                   (unsigned)v740_now, (int)V740_DHCP_QUIET_MS);
          }

        if (v740_now - s_v740_assoc_ms < (uint32_t)V740_DHCP_QUIET_MS)
          {
            return 0;
          }
      }
    else
      {
        s_v740_assoc_ms = 0;
      }
  }

  load = wapi_load_config(ifname, NULL, &conf);
  g_assoc_load = (load != NULL) ? 1 : 0;
  if (load != NULL)
    {
      g_assoc_psk_match       = (conf.passphrase != NULL &&
                                 strcmp(conf.passphrase,
                                        CONFIG_NETINIT_WAPI_PASSPHRASE) == 0)
                                ? 1 : 0;
      g_assoc_saved_phraselen = (int)conf.phraselen;
    }
  else
    {
      g_assoc_psk_match       = 0;
      g_assoc_saved_phraselen = 0;
    }
  if (!load)
    {
      conf.ifname      = ifname;
      conf.sta_mode    = CONFIG_NETINIT_WAPI_STAMODE;
      conf.auth_wpa    = CONFIG_NETINIT_WAPI_AUTHWPA;
      conf.cipher_mode = CONFIG_NETINIT_WAPI_CIPHERMODE;
      conf.alg         = CONFIG_NETINIT_WAPI_ALG;
      conf.ssid        = CONFIG_NETINIT_WAPI_SSID;
      conf.passphrase  = CONFIG_NETINIT_WAPI_PASSPHRASE;
      conf.ssidlen     = strlen(conf.ssid);
      conf.phraselen   = strlen(conf.passphrase);
      conf.bssid       = NULL;
      /* [WIFI-FIX] no channel lock: the ch6 lock made the station auth on
       * a channel the AP is not on -> reason 2 AUTH_EXPIRE. The real fix
       * is the driver-side country CN. freq 0 => scan channels 1-13. */
      conf.freq        = 0;
      conf.flag        = WAPI_FREQ_AUTO;
    }

  /* [WIFI-FIX 2026-09-14] Enforce the real WPA2-PSK/CCMP parameters no
   * matter where conf came from.  The previous [GW-TEST] block forced
   * auth_wpa = cipher_mode = alg = 0, i.e. "open network".  With
   * alg == IW_ENCODE_ALG_NONE the esp32s3 wifi adapter deliberately does
   * NOT copy the passphrase into wifi_config_t.sta.password (see
   * esp_wifi_sta_password()), so esp_wifi was left with an EMPTY PSK.
   * The station could still associate - 802.11 association does not check
   * the key - but the WPA2 4-way handshake could never complete, so the
   * driver reported disconnect reason 15 (4WAY_HANDSHAKE_TIMEOUT) forever:
   * the permanent "NET 02 r15 a-1" on the panel.  Restoring WPA2 / CCMP /
   * CCMP lets the blob derive the PMK and finish the handshake.
   */
  conf.ifname      = ifname;
  conf.sta_mode    = CONFIG_NETINIT_WAPI_STAMODE;
  conf.auth_wpa    = CONFIG_NETINIT_WAPI_AUTHWPA;
  conf.cipher_mode = CONFIG_NETINIT_WAPI_CIPHERMODE;
  conf.alg         = CONFIG_NETINIT_WAPI_ALG;
  conf.freq        = 0;
  conf.flag        = WAPI_FREQ_AUTO;

  /* [WIFI-FIX 2026-09-14] The compiled-in AP always wins.
   *
   * /data/wapi.conf - left behind by any earlier `wapi save_config wlan0`
   * (CONFIG_WIRELESS_WAPI_CONFIG_PATH) - used to take precedence, because
   * ssid/passphrase only fell back to the CONFIG_* values when they were
   * EMPTY.  A stale file therefore silently overrode the intended
   * credentials: the station authenticated with the WRONG key, the AP
   * dropped the attempt and the blob reported reason 2 (AUTH_EXPIRE) -
   * the permanent "NET ... r2 a-1" on the panel.  The two constants below
   * describe how this device is actually provisioned, so nothing may
   * override them.
   */

  conf.ssid       = CONFIG_NETINIT_WAPI_SSID;
  conf.ssidlen    = (int)strlen(CONFIG_NETINIT_WAPI_SSID);
  conf.passphrase = CONFIG_NETINIT_WAPI_PASSPHRASE;
  conf.phraselen  = (int)strlen(CONFIG_NETINIT_WAPI_PASSPHRASE);
  conf.bssid      = NULL;

  syslog(LOG_ERR, "[DIAG] assoc cfg load=%p ssid=%s ssidlen=%d "
         "passlen=%d auth=%08x cipher=%08x alg=%d freq=%d flag=%d "
         "L%d K%d P%d\n",
         load, conf.ssid ? conf.ssid : "(null)", (int)conf.ssidlen,
         (int)conf.phraselen, (unsigned)conf.auth_wpa,
         (unsigned)conf.cipher_mode, (int)conf.alg,
         (int)conf.freq, (int)conf.flag,
         (int)g_assoc_load, (int)g_assoc_psk_match,
         (int)g_assoc_saved_phraselen);
  /* [V101] multi-SSID fallback: campus APs reject ESP intermittently.
   * Try each candidate (same passphrase) until one associates. */
  /* [V103] "3124" REMOVED.  It is not our network: it is a different
   * device (BSSID 1e:36:dc:4f:3e:75, locally-administered MAC, WPA2-only
   * where our AP is WPA/WPA2 mixed) sitting on the same channel with a far
   * stronger signal.  802.11 association does NOT verify the key, so the
   * station could "connect" to it and then sit there with no traffic at
   * all, which is exactly the "associated but never gets a DHCP lease"
   * symptom.  One candidate, the real one, only. */
  /* [V604] Campus AP first again, now that the real blocker is gone.
   *
   * v603 proved the link comes up (LINKED / connected=1 / zero disconnects)
   * as soon as pmf_cfg.capable is false - which is exactly what the working
   * xiaozhi firmware does: it bzero()s the whole wifi_config_t and never
   * touches pmf_cfg, so its PMF is off.  With that fixed, 3124 is tried
   * first again, on the clean driver. */
  /* [V618] 3124 first again: only the FIRST connect attempt of a boot
   * can complete, so a candidate that is merely absent burns the boot. */
  static const char *g_ssid_candidates[] = {"3124", "31246"};
  int g_ncand = (int)(sizeof(g_ssid_candidates) / sizeof(g_ssid_candidates[0]));
  ret = -ENOENT;
  for (int ci = 0; ci < g_ncand; ci++)
    {
      conf.ssid = g_ssid_candidates[ci];
      conf.ssidlen = (int)strlen(conf.ssid);
      conf.bssid = NULL;
      if (conf.ssidlen <= 0)
        continue;
      /* [SCAN-DIAG] Show what the radio can actually hear, on the console. */
  if (ci == 0)
    {
      FILE *saf = fopen("/tmp/aplist.txt", "r");
      printf("[SCAN] ---- visible APs (ssid|auth,ch,rssi) ----\n");
      fflush(stdout);
      if (saf == NULL)
        {
          printf("[SCAN] no /tmp/aplist.txt -> the boot scan did not run\n");
          fflush(stdout);
        }
      else
        {
          char slb[128];
          int sn = 0;
          while (fgets(slb, sizeof(slb), saf) != NULL && sn < 30)
            {
              printf("[SCAN] %s", slb);
              sn++;
            }
          printf("[SCAN] ---- end (%d entries) ----\n", sn);
          fflush(stdout);
          fclose(saf);
        }
    }

  syslog(LOG_ERR, "[DIAG] assoc try #%d ssid=%s\n", ci, conf.ssid);
  printf("[ASSOC] try#%d ssid=%s\n", ci, conf.ssid);
  fflush(stdout);
      int r = wpa_driver_wext_associate(&conf);
      syslog(LOG_ERR, "[DIAG] assoc try #%d ret=%d errno=%d\n", ci, r, errno);
      printf("[ASSOC] try#%d ret=%d errno=%d\n", ci, r, errno);
      fflush(stdout);
      {
        FILE *wrf = fopen("/tmp/wr.txt", "r");
        if (wrf != NULL)
          {
            char wrb[64];
            if (fgets(wrb, sizeof(wrb), wrf) != NULL)
              {
                printf("[ASSOC] disconnect reason=%s", wrb);
                fflush(stdout);
              }
            fclose(wrf);
          }
      }
      if (r == 0)
        {
          ret = 0;
          break;
        }
      /* give the radio a moment before the next attempt */
      usleep(500000);
    }
  syslog(LOG_ERR, "[DIAG] netinit_associate ret=%d errno=%d\n", ret, errno);

  wapi_unload_config(load);

  return ret;
}

#endif /* CONFIG_WIRELESS_WAPI */
