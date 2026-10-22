/****************************************************************************
 * apps/netutils/netinit/netinit.c
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

#include <errno.h>
#include <stdio.h>
#include <syslog.h>
#include <nuttx/clock.h>   /* [V613] clock_systime_ticks */

/* Is network initialization debug forced on? */

#ifdef CONFIG_NETINIT_DEBUG
#  undef  CONFIG_DEBUG_INFO
#  define CONFIG_DEBUG_INFO 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <arpa/inet.h>
#include <debug.h>
#include <net/if.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/net/mii.h>
#include <sys/boardctl.h>

#include "netutils/netlib.h"

#ifdef CONFIG_NET_6LOWPAN
#  include <nuttx/net/sixlowpan.h>
#endif

#ifdef CONFIG_NET_IEEE802154
#  include <nuttx/net/ieee802154.h>
#endif

#ifdef CONFIG_NETUTILS_NTPCLIENT
#  include "netutils/ntpclient.h"
#endif

#if defined(CONFIG_FSUTILS_IPCFG)
#  include "fsutils/ipcfg.h"
#endif

#include "netutils/netinit.h"

#ifdef CONFIG_NETUTILS_NETINIT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pick one and at most one supported link layer so that all decisions are
 * made consistently.
 *
 * NOTE: Ethernet should always be selected with IEEE 802.11
 */

#if defined(CONFIG_NET_ETHERNET)
#  undef CONFIG_NET_6LOWPAN
#  undef CONFIG_NET_SLIP
#  undef CONFIG_NET_TUN
#  undef CONFIG_NET_LOCAL
#  undef CONFIG_NET_USRSOCK
#  undef CONFIG_NET_IEEE802154
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_6LOWPAN)
#  undef CONFIG_NET_SLIP
#  undef CONFIG_NET_TUN
#  undef CONFIG_NET_LOCAL
#  undef CONFIG_NET_USRSOCK
#  undef CONFIG_NET_IEEE802154
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_SLIP)
#  undef CONFIG_NET_TUN
#  undef CONFIG_NET_LOCAL
#  undef CONFIG_NET_USRSOCK
#  undef CONFIG_NET_IEEE802154
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_TUN)
#  undef CONFIG_NET_LOCAL
#  undef CONFIG_NET_USRSOCK
#  undef CONFIG_NET_IEEE802154
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_LOCAL)
#  undef CONFIG_NET_USRSOCK
#  undef CONFIG_NET_IEEE802154
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_USRSOCK)
#  undef CONFIG_NET_IEEE802154
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_IEEE802154)
#  undef CONFIG_NET_CAN
#  undef CONFIG_NET_LOOPBACK
#elif defined(CONFIG_NET_CAN)
#  undef CONFIG_NET_LOOPBACK
#endif

/* Only Ethernet and 6LoWPAN have MAC layer addresses */

#undef HAVE_MAC
#if defined(CONFIG_NET_ETHERNET) || defined(CONFIG_NET_6LOWPAN)
#  define HAVE_MAC 1
#endif

/* Currently there is only logic in 6LoWPAN configurations to
 * set the IEEE 802.15.4 addresses.
 */

#undef HAVE_EADDR

#if defined(CONFIG_NET_6LOWPAN)
#  if defined(CONFIG_WIRELESS_IEEE802154)
#    define HAVE_EADDR 1
#  elif defined(CONFIG_WIRELESS_PKTRADIO)
#    warning Missing logic
#  endif
#endif

/* Provide a default DNS address */

#if defined(CONFIG_NETINIT_DRIPADDR) && !defined(CONFIG_NETINIT_DNSIPADDR)
#  define CONFIG_NETINIT_DNSIPADDR CONFIG_NETINIT_DRIPADDR
#endif

/* Select the single network device name supported this this network
 * initialization logic.  If multiple interfaces are present with different
 * link types, the order of definition in the following conditional
 * logic will select the one interface that will be used (which might
 * not be the one that you want).
 */

#undef NETINIT_HAVE_NETDEV
#if defined(CONFIG_DRIVERS_IEEE80211) /* Usually also has CONFIG_NET_ETHERNET */
#  define NET_DEVNAME "wlan0"
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_ETHERNET)
#  define NET_DEVNAME "eth0"
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)
#  define NET_DEVNAME "wpan0"
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_BLUETOOTH)
#  define NET_DEVNAME "bnep0"
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_SLIP)
#  define NET_DEVNAME "sl0"
#  ifndef CONFIG_NETINIT_NOMAC
#    error "CONFIG_NETINIT_NOMAC must be defined for SLIP"
#  endif
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_TUN)
#  define NET_DEVNAME "tun0"
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_LOOPBACK)
#  define NET_DEVNAME "lo"
#  define NETINIT_HAVE_NETDEV
#elif defined(CONFIG_NET_CAN)
#  define NET_DEVNAME "can0"
#  define NETINIT_HAVE_NETDEV
#endif

/* If we have no network device (perhaps only USRSOCK, local loopback, or
 * Netlink sockets), then we cannot support the network monitor.
 */

#ifndef NETINIT_HAVE_NETDEV
#  undef CONFIG_NETINIT_MONITOR
#endif

/* While the network is up, the network monitor really does nothing.  It
 * will wait for a very long time while waiting, it can be awakened by a
 * signal indicating a change in network status.
 */

#ifdef CONFIG_SYSTEM_TIME64
#  define LONG_TIME_SEC    (60*60)   /* One hour in seconds */
#else
#  define LONG_TIME_SEC    (5*60)    /* Five minutes in seconds */
#endif

#define SHORT_TIME_SEC     (2)       /* 2 seconds */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NETINIT_MONITOR
static sem_t g_notify_sem;
#endif

#ifdef CONFIG_NETUTILS_DHCPC
bool g_use_dhcpc;
#endif

#if defined(CONFIG_NET_IPv6) && !defined(CONFIG_NET_ICMPv6_AUTOCONF) && \
   !defined(CONFIG_NET_6LOWPAN)
/* Host IPv6 address */

static const uint16_t g_ipv6_hostaddr[8] =
{
  HTONS(CONFIG_NETINIT_IPv6ADDR_1),
  HTONS(CONFIG_NETINIT_IPv6ADDR_2),
  HTONS(CONFIG_NETINIT_IPv6ADDR_3),
  HTONS(CONFIG_NETINIT_IPv6ADDR_4),
  HTONS(CONFIG_NETINIT_IPv6ADDR_5),
  HTONS(CONFIG_NETINIT_IPv6ADDR_6),
  HTONS(CONFIG_NETINIT_IPv6ADDR_7),
  HTONS(CONFIG_NETINIT_IPv6ADDR_8),
};

/* Default routine IPv6 address */

static const uint16_t g_ipv6_draddr[8] =
{
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_1),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_2),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_3),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_4),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_5),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_6),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_7),
  HTONS(CONFIG_NETINIT_DRIPv6ADDR_8),
};

/* IPv6 netmask */

static const uint16_t g_ipv6_netmask[8] =
{
  HTONS(CONFIG_NETINIT_IPv6NETMASK_1),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_2),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_3),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_4),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_5),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_6),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_7),
  HTONS(CONFIG_NETINIT_IPv6NETMASK_8),
};
#endif /* CONFIG_NET_IPv6 && !CONFIG_NET_ICMPv6_AUTOCONF && !CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netinit_set_macaddr
 *
 * Description:
 *   Set the hardware MAC address if the hardware is not capable of doing
 *   that for itself.
 *
 ****************************************************************************/

#if defined(NETINIT_HAVE_NETDEV) && defined(CONFIG_NETINIT_NOMAC) && \
    defined(HAVE_MAC)
static void netinit_set_macaddr(void)
{
#if defined(CONFIG_NETINIT_WIFIMAC)
  char macstr[20];
  uint8_t wifi_mac[IFHWADDRLEN];
#elif defined(CONFIG_NETINIT_UIDMAC)
  uint8_t uid[CONFIG_BOARDCTL_UNIQUEID_SIZE];
#elif defined(CONFIG_NET_ETHERNET)
  uint8_t mac[IFHWADDRLEN];
#elif defined(HAVE_EADDR)
  uint8_t eaddr[8];
#endif

  /* Many embedded network interfaces must have a software assigned MAC */

#if defined(CONFIG_NETINIT_WIFIMAC)
  #define BOARDIOC_USER_MAC_WIFI (BOARDIOC_USER + 3)

  boardctl(BOARDIOC_USER_MAC_WIFI, (uintptr_t)macstr);
  if (netlib_ethaddrconv(macstr, wifi_mac))
    {
      netlib_setmacaddr(NET_DEVNAME, wifi_mac);
    }
#elif defined(CONFIG_NETINIT_UIDMAC)
  boardctl(BOARDIOC_UNIQUEID, (uintptr_t)&uid);
  uid[0] = (uid[0] & 0b11110000) | 2; /* Locally Administered MAC */
  netlib_setmacaddr(NET_DEVNAME, uid);

#elif defined(CONFIG_NET_ETHERNET)
  /* Use the configured, fixed MAC address */

  mac[0] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 1)) & 0xff;
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;

  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  /* Set the MAC address */

  netlib_setmacaddr(NET_DEVNAME, mac);

#elif defined(HAVE_EADDR)
  /* Use the configured, fixed extended address */

  eaddr[0] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 3)) & 0xff;
  eaddr[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 2)) & 0xff;
  eaddr[2] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 1)) & 0xff;
  eaddr[3] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;

  eaddr[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  eaddr[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  eaddr[6] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  eaddr[7] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;

  /* Set the 6LoWPAN extended address */

  netlib_seteaddr(NET_DEVNAME, eaddr);
#endif /* CONFIG_NET_ETHERNET or HAVE_EADDR */
}
#else
#  define netinit_set_macaddr()
#endif

#if defined(CONFIG_NETINIT_THREAD) && CONFIG_NETINIT_RETRY_MOUNTPATH > 0
static inline void netinit_checkpath(void)
{
  int retries = CONFIG_NETINIT_RETRY_MOUNTPATH;
  while (retries > 0)
    {
      DIR * dir = opendir(CONFIG_IPCFG_PATH);
      if (dir)
        {
          /* Directory exists. */

          closedir(dir);
          break;
        }
      else
        {
        usleep(100000);
        }

      retries--;
    }
}
#endif

/****************************************************************************
 * Name: netinit_set_ipv4addrs
 *
 * Description:
 *   Setup IPv4 addresses.
 *
 ****************************************************************************/

#if defined(NETINIT_HAVE_NETDEV) && !defined(CONFIG_NET_6LOWPAN) && ! \
    defined(CONFIG_NET_IEEE802154) && defined(CONFIG_NET_IPv4)
static inline void netinit_set_ipv4addrs(void)
{
  struct in_addr addr;
#ifdef CONFIG_FSUTILS_IPCFG
  struct ipv4cfg_s ipv4cfg;
  int ret;

  /* Attempt to obtain IPv4 address configuration from the IP configuration
   * file.
   */

#if defined(CONFIG_NETINIT_THREAD) && CONFIG_NETINIT_RETRY_MOUNTPATH > 0
  netinit_checkpath();
#endif

  ret = ipcfg_read(NET_DEVNAME, (FAR struct ipcfg_s *)&ipv4cfg, AF_INET);
#ifdef CONFIG_NETUTILS_DHCPC
  if (ret >= 0 && ipv4cfg.proto != IPv4PROTO_NONE)
#else
  if (ret >= 0 && IPCFG_HAVE_STATIC(ipv4cfg.proto))
#endif
    {
      /* Check if we are using DHCPC */

#ifdef CONFIG_NETUTILS_DHCPC
      if (IPCFG_USE_DHCP(ipv4cfg.proto))
        {
          g_use_dhcpc = true;
          addr.s_addr = 0;
        }
      else
#endif
        {
          /* We are not using DHCPC.  We need an IP address */

#ifdef CONFIG_NETINIT_IPADDR
          /* Check if we have a static IP address in the configuration file */

          if (IPCFG_HAVE_STATIC(ipv4cfg.proto))
            {
              addr.s_addr = ipv4cfg.ipaddr;
            }
          else
            {
              /* This is not a good option, but in this case what else can
               * we do?
               */

              addr.s_addr = HTONL(CONFIG_NETINIT_IPADDR);
            }
#else
          /* Use whatever was provided in the file (might be zero) */

          addr.s_addr = ipv4cfg.ipaddr;
#endif
        }

      netlib_set_ipv4addr(NET_DEVNAME, &addr);

      /* Set up the remaining addresses */

      if (IPCFG_HAVE_STATIC(ipv4cfg.proto))
        {
          /* Set up the default router address */

          addr.s_addr = ipv4cfg.router;
          netlib_set_dripv4addr(NET_DEVNAME, &addr);

          /* Setup the subnet mask */

          addr.s_addr = ipv4cfg.netmask;
          netlib_set_ipv4netmask(NET_DEVNAME, &addr);
        }

#ifdef CONFIG_NETUTILS_DHCPC
      /* No static addresses?  That is fine if we are have addresses
       * provided by the configuration, or if we are using DHCP.
       */

      else if (g_use_dhcpc)
        {
          /* Set up the default router address and sub-net mask */

          addr.s_addr = 0;
          netlib_set_dripv4addr(NET_DEVNAME, &addr);
          netlib_set_ipv4netmask(NET_DEVNAME, &addr);
        }
#endif
      else
        {
          /* Otherwise, set up the configured default router address */

          addr.s_addr = HTONL(CONFIG_NETINIT_DRIPADDR);
          netlib_set_dripv4addr(NET_DEVNAME, &addr);

          /* Setup the subnet mask */

          addr.s_addr = HTONL(CONFIG_NETINIT_NETMASK);
          netlib_set_ipv4netmask(NET_DEVNAME, &addr);
        }

#ifdef CONFIG_NETINIT_DNS
      /* Set up the DNS address.  Was one provided in the configuration? */

      if (ipv4cfg.dnsaddr == 0)
        {
          /* No, use the configured default */

          addr.s_addr = HTONL(CONFIG_NETINIT_DNSIPADDR);
        }
      else
        {
          addr.s_addr = ipv4cfg.dnsaddr;
        }

      netlib_set_ipv4dnsaddr(&addr);
#endif
    }
  else
#endif
    {
      /* Set up our host address */

#ifdef CONFIG_NETINIT_DHCPC
      g_use_dhcpc = true;
      addr.s_addr = 0;
#else
      addr.s_addr = HTONL(CONFIG_NETINIT_IPADDR);
#endif
      netlib_set_ipv4addr(NET_DEVNAME, &addr);

      /* Set up the default router address */

      addr.s_addr = HTONL(CONFIG_NETINIT_DRIPADDR);
      netlib_set_dripv4addr(NET_DEVNAME, &addr);

      /* Setup the subnet mask */

      addr.s_addr = HTONL(CONFIG_NETINIT_NETMASK);
      netlib_set_ipv4netmask(NET_DEVNAME, &addr);

#ifdef CONFIG_NETINIT_DNS
      addr.s_addr = HTONL(CONFIG_NETINIT_DNSIPADDR);
      netlib_set_ipv4dnsaddr(&addr);
#endif
    }
}
#endif

/****************************************************************************
 * Name: netinit_set_ipv6addrs
 *
 * Description:
 *   Setup IPv6 addresses.
 *
 ****************************************************************************/

#if defined(NETINIT_HAVE_NETDEV) && !defined(CONFIG_NET_6LOWPAN) && ! \
    defined(CONFIG_NET_IEEE802154) && defined(CONFIG_NET_IPv6)
static inline void netinit_set_ipv6addrs(void)
{
#ifndef CONFIG_NET_ICMPv6_AUTOCONF
#ifdef CONFIG_FSUTILS_IPCFG
  struct ipv6cfg_s ipv6cfg;
  int ret;
#endif

#ifdef CONFIG_FSUTILS_IPCFG
  /* Attempt to obtain IPv6 address configuration from the IP configuration
   * file.
   */

#if defined(CONFIG_NETINIT_THREAD) && CONFIG_NETINIT_RETRY_MOUNTPATH > 0
  netinit_checkpath();
#endif

  ret = ipcfg_read(NET_DEVNAME, (FAR struct ipcfg_s *)&ipv6cfg, AF_INET6);
  if (ret >= 0 && IPCFG_HAVE_STATIC(ipv6cfg.proto))
    {
      /* Set up our fixed host address */

      netlib_set_ipv6addr(NET_DEVNAME, &ipv6cfg.ipaddr);

      /* Set up the default router address */

      netlib_set_dripv6addr(NET_DEVNAME, &ipv6cfg.router);

      /* Setup the subnet mask */

      netlib_set_ipv6netmask(NET_DEVNAME, &ipv6cfg.netmask);
    }
  else
#endif
    {
      /* Set up our fixed host address */

      netlib_set_ipv6addr(NET_DEVNAME,
                          (FAR const struct in6_addr *)g_ipv6_hostaddr);

      /* Set up the default router address */

      netlib_set_dripv6addr(NET_DEVNAME,
                            (FAR const struct in6_addr *)g_ipv6_draddr);

      /* Setup the subnet mask */

      netlib_set_ipv6netmask(NET_DEVNAME,
                            (FAR const struct in6_addr *)g_ipv6_netmask);
    }
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
}
#endif

/****************************************************************************
 * Name: netinit_set_ipaddrs
 *
 * Description:
 *   Setup IP addresses.
 *
 *   For 6LoWPAN, the IP address derives from the MAC address.  Setting it
 *   to any user provided value is asking for trouble.
 *
 ****************************************************************************/

#if defined(NETINIT_HAVE_NETDEV) && !defined(CONFIG_NET_6LOWPAN) && ! \
    defined(CONFIG_NET_IEEE802154)
static void netinit_set_ipaddrs(void)
{
#ifdef CONFIG_NET_IPv4
  netinit_set_ipv4addrs();
#endif

#ifdef CONFIG_NET_IPv6
  netinit_set_ipv6addrs();
#endif
}
#else
#  define netinit_set_ipaddrs()
#endif

/****************************************************************************
 * Name: netinit_net_bringup()
 *
 * Description:
 *   Bring up the configured network
 *
 ****************************************************************************/

#if defined(NETINIT_HAVE_NETDEV) && !defined(CONFIG_NETINIT_NETLOCAL)
static void netinit_net_bringup(void)
{
  int _diag_ret;

  /* [V741] *** One bring-up per boot. ***
   *
   * Measured on v740 (_work\_v740_chk.log): this function was entered
   * THREE times in the first 13 seconds (0.87 / 7.39 / 12.31 s).  Its very
   * first act is netlib_ifup() -> esp_wifi_start(), which fires
   * evt STA_START and tears down whatever association the previous pass had
   * just built, and each pass also pays for a ~2.5 s blocking scan.  The
   * boot therefore went associate -> dropped -> associate -> dropped, and
   * the DHCP wait of the first pass was restarted before it could ever
   * finish.  The address only appeared at 12.75 s.
   *
   * Recovery is NOT lost: the association observer
   * (netinit_wifi_diag_thread) keeps calling netinit_associate() whenever
   * the link really is down, and netinit_associate()'s v375 guard (plus the
   * v740 quiet window) leaves a healthy link alone. */

  {
    static int s_v741_entered;

    if (s_v741_entered)
      {
        syslog(LOG_ERR, "[V741] netinit_net_bringup skipped: pass %d "
               "already ran (another pass would restart the station)\n",
               s_v741_entered);
        return;
      }

    s_v741_entered = 1;
  }

  syslog(LOG_ERR, "[DIAG] netinit_net_bringup enter dev=%s\n", NET_DEVNAME);
  /* Bring the network up. */

  _diag_ret = netlib_ifup(NET_DEVNAME);
  syslog(LOG_ERR, "[DIAG] netlib_ifup ret=%d\n", _diag_ret);
  if (_diag_ret < 0)
    {
      return;
    }

#if defined(CONFIG_WIRELESS_WAPI) && defined(CONFIG_DRIVERS_IEEE80211)
  /* Associate the wlan with an access point. */

  if (netinit_associate(NET_DEVNAME) < 0)
    {
      return;
    }
#endif

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Perform ICMPv6 auto-configuration */

  netlib_icmpv6_autoconfiguration(NET_DEVNAME);
#endif

#ifdef CONFIG_NETUTILS_DHCPC
  if (g_use_dhcpc)
    {
      /* [V113] The first DHCP attempt always fails because the WiFi driver
       * needs ~8 seconds to fully settle after association.  Wait before
       * trying.  This eliminates the 140-second delay caused by the keeper's
       * slow retry cadence. */
      /* [V618] Retry until a lease arrives.
       *
       * The ESP32-S3 blob refuses every LARGE frame (>= ~96 bytes) with
       * ESP_ERR_NO_MEM for roughly the first 30 s after association, while
       * small frames still go out.  The DHCP DISCOVER is ~307 bytes, so a
       * short two-shot schedule always loses the race.  Keep asking; the
       * window opens by itself and the first attempt after that succeeds. */

      /* [V741] Poll for the lease instead of blind-sleeping 8 s.
       *
       * The [V113] sleep was written on the assumption that the driver
       * needs a fixed 8 s to settle, so even when the window opened at 4 s
       * we still waited until 8 s before asking.  Ask every second instead:
       * as soon as the blob stops refusing the 307-byte DISCOVER, the lease
       * lands.  The original "sleep(5) + second attempt" fallback below is
       * left exactly as it was, so the worst case is no worse than before. */

      {
        int _r1 = -1;
        int _v741_try;

        for (_v741_try = 0; _v741_try < 8 && _r1 < 0; _v741_try++)
          {
            if (_v741_try > 0)
              {
                sleep(1);
              }

            _r1 = netlib_obtain_ipv4addr(NET_DEVNAME);

            syslog(LOG_ERR, "[V741] dhcp try %d -> %d\n", _v741_try, _r1);
          }


        

        if (_r1 < 0)
          {
            
            sleep(5);

            {
              int _r2 = netlib_obtain_ipv4addr(NET_DEVNAME);

              

              if (_r2 < 0)
                {
                  
                  return;
                }
            }
          }
      }
    }
#endif

#ifdef CONFIG_NETUTILS_NTPCLIENT
  /* Start the NTP client */

  ntpc_start();
#endif
}
#else
#  define netinit_net_bringup()
#endif

/****************************************************************************
 * Name: netinit_configure
 *
 * Description:
 *   Initialize the network per the selected NuttX configuration
 *
 ****************************************************************************/

/****************************************************************************
 * [DIAG] Temporary: periodic WiFi association retry / observer
 *
 * netinit performs the primary association exactly once at boot.  Because the
 * USB-Serial-JTAG console is not ready in time to capture that single attempt,
 * this task re-runs the *same* association every 10s and logs the result with
 * syslog(LOG_ERR) so the outcome can be observed (live and via 'dmesg') at any
 * time.  It stops retrying as soon as one attempt reports success.
 ****************************************************************************/

#if defined(CONFIG_WIRELESS_WAPI) && defined(CONFIG_DRIVERS_IEEE80211)

/* [WIFI-DIAG 2026-09-14] produced by netinit_associate() */
extern volatile int g_assoc_load;
/* [WDIAG-R10] defined in esp32s3_wifi_adapter.c */
extern int g_sta_thr_override;
/* [WDIAG-R12] tight one-shot knob sweep (esp32s3_wifi_adapter.c) */
int esp_wifi_sta_try_cfg(int thr, int pmf_cap, int pin, int *out_reason);
/* [WDIAG-R15] direct SSID/key connect + radio ownership flag */
int esp_wifi_sta_try_ssid(const char *ssid, const char *psk, int thr,
                          int pmf_cap, int *out_reason);
extern volatile int g_sweep_busy;
void esp_wifi_sta_lowrate(void);
extern volatile int g_assoc_psk_match;
extern volatile int g_assoc_saved_phraselen;

static volatile int g_wifi_diag_ok;

/* [UI-FIX-R35] Publish the network state for the info panel.
 *
 * /tmp/net.txt is the panel's contract and this thread is its only writer:
 *
 *   line 1   the IPv4 address, or "NO NET"
 *   line 2   the association diagnostics ("r2 A4C11"), which the panel shows
 *            only while there is no address
 *
 * Line 1 used to carry this thread's internal counters ("o0 r2 a-1") and the
 * panel dutifully displayed those instead of an address.  The address is now
 * read straight from the interface, so it stays right regardless of how the
 * association bookkeeping is doing.
 */

static void netinit_publish_net_state(void)
{
  static char last[64];
  char wrb[16] = "";
  char apb[16] = "";
  char ipb[24] = "";
  char line[64];
  struct in_addr addr;
  FILE *f;
  FILE *nf;

  f = fopen("/tmp/wr.txt", "r");
  if (f != NULL)
    {
      if (fgets(wrb, sizeof(wrb), f) == NULL)
        {
          wrb[0] = '\0';
        }
      fclose(f);
    }

  f = fopen("/tmp/wap.txt", "r");
  if (f != NULL)
    {
      if (fgets(apb, sizeof(apb), f) == NULL)
        {
          apb[0] = '\0';
        }
      fclose(f);
    }

  addr.s_addr = 0;
  if (netlib_get_ipv4addr(NET_DEVNAME, &addr) == OK && addr.s_addr != 0)
    {
      inet_ntop(AF_INET, &addr, ipb, sizeof(ipb));
    }

  snprintf(line, sizeof(line), "%s|%s %s",
           ipb[0] ? ipb : "NO NET",
           wrb[0] ? wrb : "-", apb[0] ? apb : "-");

  nf = fopen("/tmp/net.txt", "w");
  if (nf != NULL)
    {
      fprintf(nf, "%s\n", ipb[0] ? ipb : "NO NET");
      fprintf(nf, "%s %s", wrb[0] ? wrb : "-", apb[0] ? apb : "-");
      fclose(nf);
    }

  /* Console trace, but only when it changes - this runs every pass. */

  if (strcmp(line, last) != 0)
    {
      strncpy(last, line, sizeof(last) - 1);
      last[sizeof(last) - 1] = '\0';
      printf("[NETUI] %s\n", line);
      fflush(stdout);
    }
}

static FAR void *netinit_wifi_diag_thread(FAR void *arg)
{
  int ret;
  int n = 0;

  (void)arg;

  for (;;)
    {
      if (!g_wifi_diag_ok)
        {
          /* [WDIAG-R12] before the slow netinit retry loop, run one tight
           * sweep over the three knobs left in play and print the disconnect
           * reason of every combination.  thr: 0=OPEN 3=WPA2 4=WPA/WPA2
           * 6=WPA3(SAE) 7=WPA2/WPA3, pmf: pmf_cfg.capable, pin: lock onto
           * the scanned BSSID + channel (0 = let the blob scan itself). */
          {
            static int swept;

            /* [WIFI-FIX-R31] Experiment table switched off for good.
             *
             * It served its purpose (it is what exposed the channel and the
             * camclient interference), but its rows connect and release in
             * turn and leave the driver mid-flight - after which netinit's own
             * attempts fail with -1 and no address is ever obtained.  netinit
             * now performs the association by itself, which is the real
             * product configuration. */

            if (0)
              {
                static const struct
                {
                  const char *ssid;
                  const char *psk;
                  int thr;
                  int pmf;
                } tab[] =
                {
                  /* Strong AP under our own control first - the previous
                   * control (open AP at -88 dBm) was too weak to be
                   * conclusive.  This one is next to the board. */

                  { "OU 5519", "45120012",   4, 0 },
                  { "OU 5519", "45120012",   4, 0 },

                  /* reference: the router that keeps failing */

                  { "31241",   "rczzzhl312", 5, 0 },

                  /* differential: a wrong key can only fail at the 4-way
                   * handshake (reason 15).  If it reports reason 2 like the
                   * correct key does, reason 2 is not about credentials. */

                  { "31241",   "rczzzhl312",  5, 0 }
                };

                int k;
                int rc;
                int reason;

                swept = 1;
                g_sweep_busy = 1;

                printf("[SWEEP] start %d combos (radio owned)\n",
                       (int)(sizeof(tab) / sizeof(tab[0])));
                fflush(stdout);

                for (k = 0; k < (int)(sizeof(tab) / sizeof(tab[0])); k++)
                  {
                    reason = 0;
                    esp_wifi_sta_lowrate();
                    rc = esp_wifi_sta_try_ssid(tab[k].ssid, tab[k].psk,
                                               tab[k].thr, tab[k].pmf,
                                               &reason);
                    printf("[SWEEP] #%d ssid=%s thr=%d pmf=%d rc=%d "
                           "connected=%d reason=%d\n",
                           k, tab[k].ssid, tab[k].thr, tab[k].pmf, rc,
                           rc == 0 ? 1 : 0, reason);
                    fflush(stdout);
                    sleep(1);
                  }

                g_sweep_busy = 0;
                printf("[SWEEP] done (radio released)\n");
                fflush(stdout);
              }
          }

          /* [WDIAG-R10] sweep threshold.authmode across attempts:
           * 0=OPEN 2=WPA 3=WPA2 4=WPA/WPA2 6=WPA3 7=WPA2/WPA3 */
          {
            static const int thr_tab[6] = { 0, 2, 3, 4, 6, 7 };
            g_sta_thr_override = thr_tab[n % 6];
          }

          errno = 0;
          /* a stale reason from the previous attempt must never be shown */
          unlink("/tmp/wr.txt");

          /* [WDIAG-R9] print the full scan list once, so we can see whether a
           * control AP (e.g. an open hotspot) exists nearby. */
          {
            static int aplist_done;
            if (!aplist_done)
              {
                FILE *lf = fopen("/tmp/aplist.txt", "r");
                if (lf != NULL)
                  {
                    char lb[96];
                    aplist_done = 1;
                    printf("[APLIST]\n");
                    while (fgets(lb, sizeof(lb), lf) != NULL)
                      {
                        printf("[AP] %s", lb);
                      }
                    printf("[/APLIST]\n");
                    fflush(stdout);
                    fclose(lf);
                  }
              }
          }
          ret = netinit_associate(NET_DEVNAME);
          syslog(LOG_ERR, "[DIAG] wifidiag #%d associate ret=%d errno=%d\n",
                 n, ret, errno);

          /* [WDIAG-R7] Also put the verdict on the CONSOLE (printf -> COM3).
           * syslog only reaches /dev/kmsg, which needs an interactive shell
           * to read; the USB-CDC input endpoint stalls once the panel starts
           * repainting, so a read-only console line is the reliable channel.
           * Wait briefly so the asynchronous STA_DISCONNECT event lands in
           * /tmp/wr.txt before we report it. */
          {
            char wrb2[24] = "";
            char apb2[24] = "";
            FILE *f2;

            sleep(2);
            f2 = fopen("/tmp/wr.txt", "r");
            if (f2 != NULL)
              {
                if (fgets(wrb2, sizeof(wrb2), f2) == NULL)
                  {
                    wrb2[0] = '\0';
                  }
                fclose(f2);
              }

            f2 = fopen("/tmp/wap.txt", "r");
            if (f2 != NULL)
              {
                if (fgets(apb2, sizeof(apb2), f2) == NULL)
                  {
                    apb2[0] = '\0';
                  }
                fclose(f2);
              }

            {
              char ccb[40] = "";
              FILE *f3 = fopen("/tmp/wconn.txt", "r");
              if (f3 != NULL)
                {
                  if (fgets(ccb, sizeof(ccb), f3) == NULL)
                    {
                      ccb[0] = '\0';
                    }
                  fclose(f3);
                }

              printf("[WDIAG] n=%d ret=%d load=%d ssd=%s wr=%s ap=%s cfg=%s\n",
                     n, ret, (int)g_assoc_load,
                     CONFIG_NETINIT_WAPI_SSID,
                     wrb2[0] ? wrb2 : "-",
                     apb2[0] ? apb2 : "-",
                     ccb[0] ? ccb : "-");
            }
            fflush(stdout);
          }
          /* [UI-FIX-R35] Superseded by netinit_publish_net_state(), which runs
           * unconditionally at the bottom of this loop.  Kept compiled but
           * disabled so this thread stops writing its counters into the
           * panel's file. */

          if (0)
          {
            FILE *nf = fopen("/tmp/net.txt", "w");
            if (nf != NULL)
              {
                char wcb[16] = "";
                char wrb[16] = "";
                FILE *cf = fopen("/tmp/wc.txt", "r");
                FILE *rf;
                if (cf != NULL)
                  {
                    if (fgets(wcb, sizeof(wcb), cf) == NULL)
                      {
                        wcb[0] = '\0';
                      }
                    fclose(cf);
                  }
                rf = fopen("/tmp/wr.txt", "r");
                if (rf != NULL)
                  {
                    if (fgets(wrb, sizeof(wrb), rf) == NULL)
                      {
                        wrb[0] = '\0';
                      }
                    fclose(rf);
                  }
                char ob[16] = "";
                FILE *of = fopen("/tmp/wnv.txt", "r");
                if (of != NULL)
                  {
                    if (fgets(ob, sizeof(ob), of) == NULL)
                      {
                        ob[0] = '\0';
                      }
                    fclose(of);
                  }
                fprintf(nf, "%s %s%s a%d\n", ob, wcb, wrb, ret);
                {
                  /* line 2: credential facts + the scanned AP security.
                   * A<authmode>C<channel>, A9C9 = the SSID was not in the
                   * scan result at all.  Kept <= 15 chars so the 16px font
                   * still fits one 240px line. */
                  char ab[24] = "";
                  FILE *af = fopen("/tmp/wap.txt", "r");
                  if (af != NULL)
                    {
                      if (fgets(ab, sizeof(ab), af) == NULL)
                        {
                          ab[0] = '\0';
                        }
                      fclose(af);
                    }

                  fprintf(nf, "L%d %s", (int)g_assoc_load, ab);
                }
                fclose(nf);
              }
          }
          if (ret >= 0)
            {
              g_wifi_diag_ok = 1;
              syslog(LOG_ERR, "[DIAG] wifidiag: association OK\n");
            }
        }

      /* [UI-FIX-R35] Publish on EVERY pass, not only while the association is
       * still being worked on.
       *
       * This write used to sit inside the block above, which is skipped once
       * g_wifi_diag_ok is set - so the moment the link came up the file
       * stopped being refreshed and, 30 s later, the panel treated it as
       * stale and showed "NO NET" while the station was connected. */

      netinit_publish_net_state();

      n++;
      sleep(3);
    }

  return NULL;
}

/* [WDIAG-R11] Drain the syslog ring (/dev/kmsg) to the console.
 *
 * The Wi-Fi adapter reports every step with syslog(LOG_ERR, "[DIAG] ..."),
 * which only lands in the RAMLOG ring buffer.  Reading that normally needs an
 * interactive NSH, and on this board the USB-CDC input endpoint stalls as soon
 * as the panel starts repainting.  This task closes that gap: it owns the
 * /dev/kmsg file descriptor for the rest of the uptime and simply copies
 * whatever arrives to stdout (COM3), which is a proven read-only channel.
 *
 * It is deliberately independent of netinit_wifi_diag_thread(): that thread
 * may block inside netinit_associate() for a long time, and we must still be
 * able to tell a stuck thread apart from a stalled console.
 */

static FAR void *netinit_kmsg_dump_thread(FAR void *arg)
{
  static const char *names[3] =
  {
    "/dev/kmsg", "/dev/ramlog", "/dev/syslog"
  };

  char buf[1024];
  int fd = -1;
  int ni;
  int iter = 0;

  (void)arg;

  for (ni = 0; ni < 3 && fd < 0; ni++)
    {
      fd = open(names[ni], O_RDONLY);
      if (fd >= 0)
        {
          printf("[KMSG] drain device %s\n", names[ni]);
          fflush(stdout);
        }
    }

  if (fd < 0)
    {
      printf("[KMSG] no syslog device found, errno=%d\n", errno);
      fflush(stdout);
      return NULL;
    }

  for (;;)
    {
      int n = 0;
      int total = 0;

      iter++;

      /* Heartbeat: proves this task is alive and hands us the latest
       * disconnect reason without waiting for netinit's diag thread. */

      printf("\n[KMSG-%d] hb wr=", iter);

      {
        FILE *f = fopen("/tmp/wr.txt", "r");
        if (f != NULL)
          {
            if (fgets(buf, sizeof(buf), f) == NULL)
              {
                printf("-");
              }
            else
              {
                printf("%s", buf);
              }

            fclose(f);
          }
        else
          {
            printf("(none)");
          }
      }

      printf("\n");

      for (;;)
        {
          n = read(fd, buf, sizeof(buf) - 1);
          if (n <= 0)
            {
              break;
            }

          buf[n] = '\0';
          printf("%s", buf);
          total += n;

          if (total > 10000)
            {
              break;
            }
        }

      printf("\n[KMSG-%d] end n=%d errno=%d got=%d\n", iter, n, errno, total);
      fflush(stdout);

      sleep(4);
    }

  return NULL;
}

static void netinit_wifi_diag_start(void)
{
  static int started;
  pthread_attr_t attr;
  struct sched_param sparam;
  pthread_t tid;
  int ret;

  if (started)
    {
      return;
    }

  started = 1;

  pthread_attr_init(&attr);
  sparam.sched_priority = 90;
  pthread_attr_setschedparam(&attr, &sparam);
  /* [WIFI-FIX-R16] 4 KB was not enough: this thread runs printf plus the
   * adapter call chain (esp_wifi_sta_try_ssid -> esp_wifi_set_config) and it
   * was silently dying - which is why every earlier round only ever printed
   * its n=0 line and round 15 stopped after sweep row #1. */

  pthread_attr_setstacksize(&attr, 32768);

  ret = pthread_create(&tid, &attr, netinit_wifi_diag_thread, NULL);
  if (ret == OK)
    {
      pthread_detach(tid);
    }
  else
    {
      syslog(LOG_ERR, "[DIAG] wifidiag: pthread_create failed: %d\n", ret);
    }

  /* [WDIAG-R11] second, independent task: drain /dev/kmsg to the console.
   * Its own stack because it formats 1 KB chunks. */

  pthread_attr_setstacksize(&attr, 8192);
  ret = pthread_create(&tid, &attr, netinit_kmsg_dump_thread, NULL);
  if (ret == OK)
    {
      pthread_detach(tid);
    }
  else
    {
      syslog(LOG_ERR, "[DIAG] kmsgdump: pthread_create failed: %d\n", ret);
    }
}

#else
#  define netinit_wifi_diag_start()
#endif

static void netinit_configure(void)
{
#ifdef NETINIT_HAVE_NETDEV
  /* Many embedded network interfaces must have a software assigned MAC */

  netinit_set_macaddr();

  /* Set up IP addresses */

  netinit_set_ipaddrs();

  /* That completes the 'local' initialization of the network device. */

#ifndef CONFIG_NETINIT_NETLOCAL
  /* Bring the network up. */

  netinit_net_bringup();
#endif
#endif /* NETINIT_HAVE_NETDEV */

  /* [DIAG] start the periodic WiFi association retry/observer task */

  netinit_wifi_diag_start();
}

/****************************************************************************
 * Name: netinit_signal
 *
 * Description:
 *   This signal handler responds to changes in PHY status.
 *
 ****************************************************************************/

#ifdef CONFIG_NETINIT_MONITOR
static void netinit_signal(int signo, FAR siginfo_t *siginfo,
                           FAR void *context)
{
  int semcount;
  int ret;

  /* What is the count on the semaphore?  Don't over-post */

  ret = sem_getvalue(&g_notify_sem, &semcount);
  ninfo("Entry: semcount=%d\n", semcount);

  if (ret == OK && semcount <= 0)
    {
      sem_post(&g_notify_sem);
    }

  ninfo("Exit\n");
}
#endif

/****************************************************************************
 * Name: netinit_monitor
 *
 * Description:
 *   Monitor link status, gracefully taking the link up and down as the
 *   link becomes available or as the link is lost.
 *
 ****************************************************************************/

#ifdef CONFIG_NETINIT_MONITOR
static int netinit_monitor(void)
{
  struct timespec abstime;
  struct timespec reltime;
  struct ifreq ifr;
  struct sigaction act;
  struct sigaction oact;
  bool devup;
  int ret;
  int sd;

  ninfo("Entry\n");

  /* Initialize the notification semaphore */

  DEBUGVERIFY(sem_init(&g_notify_sem, 0, 0));

  /* Get a socket descriptor that we can use to communicate with the network
   * interface driver.
   */

  sd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);
  if (sd < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      nerr("ERROR: Failed to create a socket: %d\n", ret);
      goto errout;
    }

  /* Attach a signal handler so that we do not lose PHY events */

  act.sa_sigaction = netinit_signal;
  act.sa_flags = SA_SIGINFO;

  ret = sigaction(CONFIG_NETINIT_SIGNO, &act, &oact);
  if (ret < 0)
    {
      ret = -errno;
      DEBUGASSERT(ret < 0);

      nerr("ERROR: sigaction() failed: %d\n", ret);
      goto errout_with_socket;
    }

  /* Now loop, waiting for changes in link status */

  for (; ; )
    {
      /* Configure to receive a signal on changes in link status */

      memset(&ifr, 0, sizeof(struct ifreq));
      strlcpy(ifr.ifr_name, NET_DEVNAME, IFNAMSIZ);

      ifr.ifr_mii_notify_event.sigev_notify = SIGEV_SIGNAL;
      ifr.ifr_mii_notify_event.sigev_signo  = CONFIG_NETINIT_SIGNO;

      ret = ioctl(sd, SIOCMIINOTIFY, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCMIINOTIFY) failed: %d\n", ret);
          goto errout_with_sigaction;
        }

      /* Does the driver think that the link is up or down? */

      ret = ioctl(sd, SIOCGIFFLAGS, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCGIFFLAGS) failed: %d\n", ret);
          goto errout_with_notification;
        }

      devup = ((ifr.ifr_flags & IFF_UP) != 0);

      /* Get the current PHY address in use.  This probably does not change,
       * but just in case...
       *
       * NOTE: We are assuming that the network device name is preserved in
       * the ifr structure.
       */

      ret = ioctl(sd, SIOCGMIIPHY, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCGMIIPHY) failed: %d\n", ret);
          goto errout_with_notification;
        }

      /* Read the PHY status register */

      ifr.ifr_mii_reg_num = MII_MSR;

      ret = ioctl(sd, SIOCGMIIREG, (unsigned long)&ifr);
      if (ret < 0)
        {
          ret = -errno;
          DEBUGASSERT(ret < 0);

          nerr("ERROR: ioctl(SIOCGMIIREG) failed: %d\n", ret);
          goto errout_with_notification;
        }

      ninfo("%s: devup=%d PHY address=%02x MSR=%04x\n",
            ifr.ifr_name, devup, ifr.ifr_mii_phy_id, ifr.ifr_mii_val_out);

      /* Check for link up or down */

      if ((ifr.ifr_mii_val_out & MII_MSR_LINKSTATUS) != 0)
        {
          /* Link up... does the drive think that the link is up? */

          if (!devup)
            {
              /* No... We just transitioned from link down to link up.
               * Bring the link up.
               */

              ninfo("Bringing the link up\n");

              ifr.ifr_flags = IFF_UP;
              ret = ioctl(sd, SIOCSIFFLAGS, (unsigned long)&ifr);
              if (ret < 0)
                {
                  ret = -errno;
                  DEBUGASSERT(ret < 0);

                  nerr("ERROR: ioctl(SIOCSIFFLAGS) failed: %d\n", ret);
                  goto errout_with_notification;
                }

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
              /* Perform ICMPv6 auto-configuration */

              netlib_icmpv6_autoconfiguration(ifr.ifr_name);
#endif
              /* And wait for a short delay.  We will want to recheck the
               * link status again soon.
               */

              reltime.tv_sec  = SHORT_TIME_SEC;
              reltime.tv_nsec = 0;
            }
          else
            {
              /* The link is still up.  Take a long, well-deserved rest */

              reltime.tv_sec  = LONG_TIME_SEC;
              reltime.tv_nsec = 0;
            }
        }
      else
        {
          /* Link down... Was the driver link state already down? */

          if (devup)
            {
              /* No... we just transitioned from link up to link down.  Take
               * the link down.
               */

              ninfo("Taking the link down\n");

              ifr.ifr_flags = 0;
              ret = ioctl(sd, SIOCSIFFLAGS, (unsigned long)&ifr);
              if (ret < 0)
                {
                  ret = -errno;
                  DEBUGASSERT(ret < 0);

                  nerr("ERROR: ioctl(SIOCSIFFLAGS) failed: %d\n", ret);
                  goto errout_with_notification;
                }
            }

          /* In either case, wait for the short, configurable delay */

          reltime.tv_sec  = CONFIG_NETINIT_RETRYMSEC / 1000;
          reltime.tv_nsec = (CONFIG_NETINIT_RETRYMSEC % 1000) * 1000000;
        }

      /* Now wait for either the semaphore to be posted for a timed-out to
       * occur.
       */

      sched_lock();
      DEBUGVERIFY(clock_gettime(CLOCK_REALTIME, &abstime));

      abstime.tv_sec  += reltime.tv_sec;
      abstime.tv_nsec += reltime.tv_nsec;
      if (abstime.tv_nsec >= 1000000000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000000000;
        }

      sem_timedwait(&g_notify_sem, &abstime);
      sched_unlock();
    }

  /* TODO: Stop the PHY notifications and remove the signal handler. */

errout_with_notification:
  ifr.ifr_mii_notify_event.sigev_notify = SIGEV_NONE;
  ioctl(sd, SIOCMIINOTIFY, (unsigned long)&ifr);
errout_with_sigaction:
  sigaction(CONFIG_NETINIT_SIGNO, &oact, NULL);
errout_with_socket:
  close(sd);
errout:
  nerr("ERROR: Aborting\n");
  return ret;
}
#endif

/****************************************************************************
 * Name: netinit_thread
 *
 * Description:
 *   Initialize the network per the selected NuttX configuration
 *
 ****************************************************************************/

#ifdef CONFIG_NETINIT_THREAD
static pthread_addr_t netinit_thread(pthread_addr_t arg)
{
  ninfo("Entry\n");

  /* Configure the network */

  netinit_configure();

#ifdef CONFIG_NETINIT_MONITOR
  /* Monitor the network status */

  netinit_monitor();
#endif

  ninfo("Exit\n");
  return NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netinit_bringup
 *
 * Description:
 *   Initialize the network per the selected NuttX configuration
 *
 ****************************************************************************/

int netinit_bringup(void)
{
#ifdef CONFIG_NETINIT_THREAD
  struct sched_param sparam;
  pthread_attr_t     attr;
  pthread_t          tid;
  int                ret;

  /* Start the network initialization thread to perform the network bring-up
   * asynchronously.
   */

  pthread_attr_init(&attr);
  sparam.sched_priority = CONFIG_NETINIT_THREAD_PRIORITY;
  pthread_attr_setschedparam(&attr, &sparam);
  pthread_attr_setstacksize(&attr, CONFIG_NETINIT_THREAD_STACKSIZE);

  ninfo("Starting netinit thread\n");
  ret = pthread_create(&tid, &attr, netinit_thread, NULL);
  if (ret != OK)
    {
      nerr("ERROR: Failed to create netinit thread: %d\n", ret);
      netinit_thread(NULL);
    }
  else
    {
      /* Detach the thread because we will not be joining to it */

      pthread_detach(tid);

      /* Name the thread */

      pthread_setname_np(tid, "netinit");
    }

  return OK;

#else
  /* Perform network initialization sequentially */

  netinit_configure();
  return OK;
#endif
}

#endif /* CONFIG_NETUTILS_NETINIT */
