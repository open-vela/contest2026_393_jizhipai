/*
 * companion_face.c — K230 人脸跟踪云台 (openvela / NuttX, AI-VOX3)
 *
 * 协议（与小智固件已验证的实现完全一致，K230 侧脚本零改动）
 * ------------------------------------------------------------
 *   主控 bind 0.0.0.0:3333；任何同网段主机往里发 UDP 即可。
 *   报文按行解析：
 *     "F,<cx>,<cy>,<w>,<h>,<conf>\n"   人脸，5 个数字，千分比 0..1000
 *     "N\n"                            没有脸
 *     其它以 '#' 开头的行             调试行，忽略
 *
 *   ★ got<5 整行丢弃 —— K230 的 sscanf 行为，不能只取前两个数。
 *
 * 控制律（摄像头固定在桌面，绝对映射）
 * ------------------------------------
 * 摄像头不动，头转不会改变人脸在画面里的位置，拿不到负反馈，所以必须
 * "看到人在哪就直接算出该指多少度"：
 *
 *   pan  = 90 + (cx - 500) * (HFOV/2)/500
 *   tilt = 90 - (cy - 500) * (VFOV/2)/500     y 轴向下，所以减号
 *
 * 若把摄像头装到云台上（COMPANION_FACE_ON_GIMBAL=1），改用比例累积律：
 *
 *   pan_target += -(cx-500) * kp
 *
 * 踩过的坑（都保留在这里，别删）
 * ------------------------------
 * 1. 副控模型会在多个疑似目标之间跳（实测相邻两帧 cx 从 730 跳到 79），
 *    真人不可能一帧移动 65% 画面宽。所以做**整帧拒绝**，不是限变化率。
 * 2. 跳变阈值要跟脸的大小走：脸越大同样的物理位移在画面里位移越大，
 *    固定阈值会把正常移动误判成跳变，攒够 kRejectMax 次后一次性放行
 *    —— 表现就是"猛甩一下"。脸宽 >150 时把阈值放宽到脸宽（上限 600）。
 * 3. 被拒的帧**必须刷新时间戳**，否则主控以为跟丢了，转去左右扫视找人，
 *    那种扫描看起来就像舵机在乱摆。
 * 4. 丢失时**不要持续补发 F**（实测 5~8s 就崩）。静默即可。
 * 5. 到位后停止 PWM 并按 kIdleRefreshMs 补一次，省得廉价舵机嗡嗡响。
 *    注意这和 servo.c 的保活线程有冲突：保活每 200ms 会重新 START。
 *    所以这里不 STOP，改成"到位后降频刷新"（kIdleRefreshMs）。
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <nuttx/net/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <syslog.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h>

#include "companion_face.h"
#include "servo.h"

static const char *TAG = "companion_face";

/* ------------------------------------------------------------ 参数 */

/* 副控以 8~10Hz 发送，主控 20Hz 轮询足够。 */
#define CF_LOOP_PERIOD_MS      50

/* [W962] Coordinate low-pass.
 *
 * The K230 detector jitters a few per-mille frame to frame even when the
 * head is still.  The loop already had a spatial deadband and a slew cap,
 * which stops the SERVO chasing it, but the target angle still twitched
 * and any twitch crossing the deadband became a real move.  Smoothing the
 * coordinate removes the jitter at the source instead of absorbing it
 * downstream.
 *
 * Integer EMA:  y += (x - y) * CF_EMA_NUM / CF_EMA_DEN
 * 3/8 puts -3 dB near 2.3 frames (~0.15 s at 15 fps), well below the
 * ~0.5 s a move needs to pay off. */
#ifndef CF_EMA_NUM
#  define CF_EMA_NUM 6
#endif
#ifndef CF_EMA_DEN
#  define CF_EMA_DEN 8
#endif
#ifndef CF_EMA_MAX_JUMP
/* A new face entering frame must not drag the estimate across the whole
 * picture through the control loop, so single-frame jumps are bounded. */
#  define CF_EMA_MAX_JUMP 250
#endif

/* 1.5s 没人脸就算跟丢。 */
#define CF_LOST_FACE_MS        1500

/* 长时间没人就慢慢回中位的间隔。 */
#define CF_RECENTER_MS         900

/* 跳变拒绝：千分比阈值与连续允许次数。 */
#define CF_REJECT_JUMP         150
#define CF_REJECT_MAX          5

/* 死区，误差小于这个不动，避免一直微调。 */
#define CF_DEADBAND            10

/* [V1043] Smallest servo-angle correction worth making, in
 * degrees.  This is the term that actually gates motion: the
 * permille CF_DEADBAND above is looser than the truncating
 * (int)(dx * gain) that follows.  Kept at 2 deg to match
 * SERVO_DEFAULT_DEADBAND in servo.c, which drops anything
 * smaller anyway. */
#define CF_MIN_DELTA_DEG      1

/* 限位。机械上 PAN 可以到 150，但留余量。 */
#define CF_PAN_MIN             10
#define CF_PAN_MAX             170
#define CF_TILT_MIN            20
#define CF_TILT_MAX            160

/* 每拍最大转角，限速。 */
#define CF_SLEW_PER_TICK       12

/* 到位后的刷新间隔（ms）。到位就不动，但保活线程会重发，
 * 所以这里只是"不再更新目标角度"，不主动停 PWM。 */
#define CF_IDLE_REFRESH_MS     1500

/* 视角：与 config.h 的 COMPANION_CAM_*_FOV_DEG 保持一致即可。 */
#ifndef COMPANION_FACE_HFOV_DEG
#  define COMPANION_FACE_HFOV_DEG   65.0f
#endif
#ifndef COMPANION_FACE_VFOV_DEG
#  define COMPANION_FACE_VFOV_DEG   51.0f
#endif

/* 绝对映射增益：偏离画面中心 500‰ 时，头应转 (FOV/2) 度。 */
/* [V1044] Aim gain: degrees of servo per permille of image offset.
 *
 * The physically correct value is (FOV/2)/500 = 0.065 deg/permille:
 * a face at the very edge turns the head by exactly the half-FOV.
 * Correct, but it reads as "barely moves" - the measured pan throw
 * was only a quarter of the clamp.  Deliberately larger now:
 *   0.16 -> +-80 deg at the edge, exactly CF_PAN_MIN/MAX
 *   0.14 -> +-70 deg at the edge, exactly CF_TILT_MIN/MAX
 * Safe to raise: COMPANION_FACE_ON_GIMBAL is 0 (camera fixed on the
 * desk), so this loop is open - moving the head cannot change what
 * the camera sees, so there is no feedback path to oscillate.
 */
#define CF_ABS_PAN_GAIN        0.16f
#define CF_ABS_TILT_GAIN       0.14f

/* 比例累积律的增益（摄像头在云台上时才用）。 */
#ifndef CF_KP
#  define CF_KP                0.06f
#endif

/* ------------------------------------------------------------ 状态 */

struct cf_face_s
{
  int x;                    /* 千分比，-1 表示无 */
  int y;
  int w;
  int h;
  int conf;
  int64_t stamp_ms;
};

static struct cf_face_s g_face = { -1, 0, 0, 0, 0, 0 };
static int g_reject_cnt;
static int g_rx_lines;
static int g_face_frames;

static int g_pan_target  = 90;
static int g_tilt_target = 90;
static int g_pan_cur     = 90;
static int g_tilt_cur    = 90;
static int64_t g_last_seen_ms;
static int64_t g_last_sweep_ms;

static int g_track_enabled = 1;
static int g_running;
static int g_sock = -1;
static uint32_t g_peer_ip;          /* 最近一个发包者的 IP */
static int g_have_peer;

static volatile int g_manual_until_ms;

/* [V1032] converted to syslog by V1032. */
/* [V1030] Observability counters for the K230 -> gimbal path.
 *
 * Why: the tracker produced no output for 240 s and there was no way to
 * tell (a) no datagram ever arrived, (b) datagrams arrived but failed to
 * parse, or (c) they parsed but never became a servo command.  rx /
 * parsed / moved separates the three.
 *
 * Bare printf(), not syslog(): the ramlog mover dies early on this board
 * and syslog never reaches the console.  File scope on purpose - two
 * function-local statics with one name are two different variables.
 */
static unsigned int g_v1030_rx_pkts;    /* datagrams recvfrom() gave us */
static unsigned int g_v1030_rx_err;     /* real socket errors, not EAGAIN */
static unsigned int g_v1030_parsed;     /* lines with all 5 fields      */
static unsigned int g_v1030_short;      /* lines dropped: < 5 fields    */
static unsigned int g_v1030_moved;      /* servo targets pushed         */

/* ------------------------------------------------------------ 工具 */

static int64_t now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int clampi(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int absi(int v)
{
  return v < 0 ? -v : v;
}

/* ------------------------------------------------------- 报文解析 */

/*
 * 处理一行报文。line 不含结尾的 \n。
 */
static void cf_handle_line(const char *line, int len)
{
  int cx = -1, cy = -1, w = 0, h = 0, conf = 0;
  int got;
  int nx, ny;

  g_rx_lines++;

  if (len < 1)
    {
      return;
    }

  /* N = 没有脸。 */
  if (line[0] == 'N' && (len == 1 || line[1] == ','))
    {
      g_face.x = -1;
      g_face.stamp_ms = now_ms();
      return;
    }

  /* 只认 F 开头的行，K230 的调试输出直接忽略。 */
  if (line[0] != 'F')
    {
      return;
    }

  /* ★ 必须 5 个数字全拿到。小智侧 got<5 整行丢弃，
   *   这里保持一致，否则会拿半截数据驱动舵机。 */
  got = sscanf(line, "F,%d,%d,%d,%d,%d", &cx, &cy, &w, &h, &conf);
  if (got < 5)
    {
      /* [V1030] A short line is a formatting problem on the K230 side.
       * Counted so the heartbeat shows it separately from "nothing
       * arrived at all". */
      g_v1030_short++;
      syslog(LOG_DEBUG, "[%s] short line (%d fields), dropped\n", TAG, got);
      return;
    }

  /* [V1030] Parsed OK.  Counted before jump rejection, so parsed
   * outruns g_face_frames whenever the frame-reject filter is what is
   * swallowing the data. */
  /* [V1044] A coordinate outside 0..1000 cannot come from the
   * detector - it is a corrupt line (UART glitches produced
   * cx=3077 in the field).  clampi() below would fold it into a
   * legitimate-looking edge value, and jump rejection does not
   * reliably catch it: jump_limit rises with the face width, up to
   * 600.  With the gain now 2.5x larger, one corrupt line would
   * slam a servo to its limit.  Drop the frame instead. */
  if (cx < 0 || cx > 1000 || cy < 0 || cy > 1000)
    {
      static unsigned int s_v1044_oob;

      s_v1044_oob++;
      if (s_v1044_oob <= 5u)
        {
          syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "oob", cx);
        }

      return;
    }

  g_v1030_parsed++;

  nx = clampi(cx, 0, 1000);
  ny = clampi(cy, 0, 1000);

  /* ---- 跳变拒绝 ---- */
  {
    int jump_limit = CF_REJECT_JUMP;

    if (w > 150)
      {
        /* 脸越近，同样的物理移动在画面里位移越大，阈值跟着放宽。 */
        jump_limit = w;
        if (jump_limit > 600)
          {
            jump_limit = 600;
          }
      }

    if (jump_limit > 0 && g_face.x >= 0)
      {
        if (absi(nx - g_face.x) > jump_limit ||
            absi(ny - g_face.y) > jump_limit)
          {
            g_reject_cnt++;
            if (g_reject_cnt <= CF_REJECT_MAX)
              {
                /*
                 * ★ 必须刷新时间戳：否则主控以为跟丢了，转去扫视找人，
                 *   那种扫描看起来就像舵机在乱摆。
                 */
                g_face.stamp_ms = now_ms();
                g_last_seen_ms = g_face.stamp_ms;
                return;
              }
            /* 连着这么多次都跳，目标大概真动了，认下这一帧。 */
          }
      }

    g_reject_cnt = 0;
  }


  /* [W962] Low-pass before the control law sees it.  The first sample
   * after a loss (g_face.x < 0) is taken raw, otherwise a face coming
   * back would ease in from where it used to be. */
  if (g_face.x >= 0)
    {
      int sx = g_face.x;
      int sy = g_face.y;
      int dx = nx - sx;
      int dy = ny - sy;

      if (dx > CF_EMA_MAX_JUMP)  dx = CF_EMA_MAX_JUMP;
      if (dx < -CF_EMA_MAX_JUMP) dx = -CF_EMA_MAX_JUMP;
      if (dy > CF_EMA_MAX_JUMP)  dy = CF_EMA_MAX_JUMP;
      if (dy < -CF_EMA_MAX_JUMP) dy = -CF_EMA_MAX_JUMP;

      nx = sx + (dx * CF_EMA_NUM) / CF_EMA_DEN;
      ny = sy + (dy * CF_EMA_NUM) / CF_EMA_DEN;
    }
  g_face.x = nx;
  g_face.y = ny;
  g_face.w = clampi(w, 0, 1000);
  g_face.h = clampi(h, 0, 1000);
  g_face.conf = clampi(conf, 0, 100);
  g_face.stamp_ms = now_ms();
  g_last_seen_ms = g_face.stamp_ms;
  g_face_frames++;
}

/* ------------------------------------------------- UDP 接收 */

/* [V1036] This board's own wlan0 address, in network byte order.
 *
 * Needed to tell our own self-test packets apart from a real K230.
 * Resolved lazily and then cached: companion_face_start() runs in the
 * OFFLINE-FIRST block, before the netinit gate, so wlan0 has no
 * address yet at bind time.  0 means "not known yet", in which case
 * every packet is reported as external rather than silently dropped.
 */
static uint32_t g_v1036_local_ip;

static uint32_t cf_local_ip(void)
{
  if (g_v1036_local_ip == 0)
    {
      struct ifreq req;
      int fd = socket(AF_INET, SOCK_DGRAM, 0);

      if (fd >= 0)
        {
          memset(&req, 0, sizeof(req));
          strncpy(req.ifr_name, "wlan0", IFNAMSIZ - 1);
          if (ioctl(fd, SIOCGIFADDR, &req) >= 0)
            {
              struct sockaddr_in *si = (struct sockaddr_in *)&req.ifr_addr;
              g_v1036_local_ip = si->sin_addr.s_addr;
            }
          close(fd);
        }
    }

  return g_v1036_local_ip;
}

static int cf_open_socket(void)
{
  struct sockaddr_in a;
  int fd;
  int opt = 1;

  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0)
    {
      syslog(LOG_ERR, "[%s] socket failed errno=%d\n", TAG, errno);
      return -errno;
    }

  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  /* [V1030] Non-blocking RX.
   *
   * cf_task() is a 20 Hz poll loop, but a blocking recvfrom() parks the
   * thread inside cf_poll_socket() whenever the queue is empty, so the
   * control update, the servo publish and the 10 s health report never
   * run.  Empty queue now returns -1/EAGAIN, which cf_poll_socket()
   * already handles by breaking out of its drain loop.
   *
   * F_SETFL, not SO_RCVTIMEO: fcntl.h is already included here. */
  {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl >= 0)
      {
        (void)fcntl(fd, F_SETFL, fl | O_NONBLOCK);
      }
  }

  memset(&a, 0, sizeof(a));
  a.sin_family      = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port        = htons(COMPANION_FACE_PORT);

  if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0)
    {
      syslog(LOG_ERR, "[%s] bind %d failed errno=%d\n",
             TAG, COMPANION_FACE_PORT, errno);
      close(fd);
      return -errno;
    }

  syslog(LOG_INFO, "[%s] listening on udp/%d\n", TAG, COMPANION_FACE_PORT);
  /* [V1030] Same fact on the console: syslog() lands in the ramlog,
   * whose mover dies early here, so this is often the only proof the
   * socket bound at all. */
  syslog(LOG_INFO, "[V1030] face listening on udp/%d\n", COMPANION_FACE_PORT);
  return fd;
}

/* 收一批数据并按行分发。 */
static void cf_poll_socket(void)
{
  char buf[256];
  static char acc[512];
  static int acc_len;
  struct sockaddr_in from;
  socklen_t fromlen;
  int n;
  int i;

  for (;;)
    {
      fromlen = sizeof(from);
      n = recvfrom(g_sock, buf, sizeof(buf) - 1, 0,
                   (struct sockaddr *)&from, &fromlen);
      if (n < 0)
        {
          /* [V1030] EAGAIN/EWOULDBLOCK is the normal empty-queue return
           * of a non-blocking socket and must stay quiet; anything else
           * is a real error.  Only the first few are printed so a
           * permanently broken socket cannot flood the console. */
          if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
              g_v1030_rx_err++;
              if (g_v1030_rx_err <= 5u)
                {
                  syslog(LOG_INFO, "[V1030] face recvfrom errno=%d (rx=%u)\n",
                         errno, g_v1030_rx_pkts);
                }
            }
          break;
        }

      if (!g_have_peer)
        {
          g_peer_ip = from.sin_addr.s_addr;
          g_have_peer = 1;
          syslog(LOG_INFO, "[%s] first vision packet from %s\n",
                  TAG, inet_ntoa(from.sin_addr));
        }

      buf[n] = '\0';

      /* [V1030] One line for each of the first 20 datagrams, then 1 in
       * 32.  %.24s shows the payload head, so a format mismatch (wrong
       * delimiter, missing fields, stray CRLF, binary) is visible at a
       * glance instead of being inferred from a counter. */
      g_v1030_rx_pkts++;
      if (g_v1030_rx_pkts <= 20u || (g_v1030_rx_pkts & 31u) == 1u)
        {
          syslog(LOG_INFO, "[V1030] face rx#%u len=%d from=%s : %.24s\n",
                 g_v1030_rx_pkts, n, inet_ntoa(from.sin_addr), buf);
        }

      /* [V1036] Anything that is not this board is news.
       *
       * The self-test proves we can send to ourselves and receive it;
       * it says nothing about whether an outside host can reach us.
       * Until the K230 is configured this is the only line that can
       * ever show it trying, so it is deliberately NOT rate limited:
       * a K230 sending to the wrong port never shows up here, and that
       * silence is itself the diagnosis.
       *
       * Full payload (%.*s), not a %.24s prefix: if it does arrive we
       * need to see the exact bytes to fix the format in one pass. */
      if (from.sin_addr.s_addr != cf_local_ip())
        {
          syslog(LOG_INFO, "[V1036] EXTERNAL from=%s : %.*s\n",
                 inet_ntoa(from.sin_addr), n, buf);
        }

      for (i = 0; i < n; i++)
        {
          if (buf[i] == '\n' || buf[i] == '\r')
            {
              if (acc_len > 0)
                {
                  acc[acc_len] = '\0';
                  cf_handle_line(acc, acc_len);
                  acc_len = 0;
                }
            }
          else if (acc_len < (int)sizeof(acc) - 1)
            {
              acc[acc_len++] = buf[i];
            }
          else
            {
              /* 溢出，整段丢弃。 */
              acc_len = 0;
            }
        }
    }
}

/* ------------------------------------------------- 控制律 */

/*
 * 根据最新人脸更新 pan_target / tilt_target。
 */
static void cf_update_targets(void)
{
  int64_t t = now_ms();
  int fresh = (g_face.x >= 0) && ((t - g_face.stamp_ms) < CF_LOST_FACE_MS);

#if COMPANION_FACE_ON_GIMBAL
  if (fresh)
    {
      /* 比例累积律：摄像头在云台上，头转会改变画面，所以有负反馈。 */
      float gain = CF_KP;
      int ex, ey;
      int too_close;

      if (g_face.w > 150)
        {
          /* 增益自适应：人越近，同样转头角度在画面里扫过越远，
           * 固定增益会转过头然后来回过冲。 */
          gain = CF_KP * (150.0f / (float)g_face.w);
          if (gain < CF_KP * 0.3f)
            {
              gain = CF_KP * 0.3f;
            }
        }

      ex = (g_face.x - 500) * CAM_PAN_SIGN;
      ey = (g_face.y - 500) * CAM_TILT_SIGN;

      /* 贴脸时再转也没意义，只会猛甩。 */
      too_close = (g_face.w > 400 || g_face.h > 400);

      if (absi(ex) > CF_DEADBAND && !too_close)
        {
          g_pan_target = clampi(g_pan_target + (int)(-ex * gain),
                                CF_PAN_MIN, CF_PAN_MAX);
        }

      if (absi(ey) > CF_DEADBAND && !too_close)
        {
          g_tilt_target = clampi(g_tilt_target + (int)(-ey * gain),
                                 CF_TILT_MIN, CF_TILT_MAX);
        }

      /* 远处抬一点头，做出"注视"的拟人感。 */
      if (g_face.w > 0 && g_face.w < 120)
        {
          g_tilt_target = clampi(g_tilt_target - 1,
                                 CF_TILT_MIN, CF_TILT_MAX);
        }

      g_last_sweep_ms = t;
    }
  else if (t - g_last_sweep_ms > CF_LOST_FACE_MS)
    {
      /* 跟丢了：扫视找人（摄像头在云台上，扫视真的能扫到人）。 */
      if (t - g_last_sweep_ms > 400)
        {
          g_pan_target = clampi(g_pan_target + 6, CF_PAN_MIN, CF_PAN_MAX);
          if (g_pan_target >= CF_PAN_MAX)
            {
              g_pan_target = CF_PAN_MAX - 6;
            }

          g_tilt_target = 90;
          g_last_sweep_ms = t;
        }
    }

#else
  /* 绝对映射律（本项目默认：摄像头固定在桌面）。 */
  if (fresh)
    {
      int dx = g_face.x - 500;
      int dy = g_face.y - 500;

      if (absi(dx) >= CF_DEADBAND)
        {
          int delta = (int)((float)dx * CF_ABS_PAN_GAIN);
          if (delta >= 0 ? delta > CF_MIN_DELTA_DEG
                         : -delta > CF_MIN_DELTA_DEG)
            {
              g_pan_target = clampi(90 + delta * CAM_ABS_PAN_SIGN,
                                    CF_PAN_MIN, CF_PAN_MAX);
            }
        }

      if (absi(dy) >= CF_DEADBAND)
        {
          int delta = (int)((float)dy * CF_ABS_TILT_GAIN);
          if (delta >= 0 ? delta > CF_MIN_DELTA_DEG
                         : -delta > CF_MIN_DELTA_DEG)
            {
              /* y 轴向下，所以减号：人在画面上方 -> tilt 变大 -> 抬头。 */
              g_tilt_target = clampi(90 - delta * CAM_ABS_TILT_SIGN,
                                     CF_TILT_MIN, CF_TILT_MAX);
            }
        }

      g_last_sweep_ms = t;
    }
  else if (t - g_last_sweep_ms > CF_RECENTER_MS)
    {
      /*
       * 长时间没人：缓慢回中位。这里**不扫视** —— 摄像头固定时，
       * 头左右转并不会让画面变，扫视找不到人只是白耗舵机寿命。
       */
      g_pan_target  = 90;
      g_tilt_target = 90;
    }
#endif
}

/* -------------------------------------------------------- 任务 */

static void *cf_task(void *arg)
{
  int64_t last_report = 0;
  int manual_guard = 0;

  syslog(LOG_INFO, "[%s] task started (port=%d gimbal=%d)\n",
          TAG, COMPANION_FACE_PORT, COMPANION_FACE_ON_GIMBAL);

  while (g_running)
    {
      cf_poll_socket();

      /* 手动模式保持人工指定角度，别被自动跟随覆盖。 */
      if (now_ms() < g_manual_until_ms)
        {
          manual_guard = 1;
        }
      else if (manual_guard)
        {
          manual_guard = 0;
        }

      if (g_track_enabled && !manual_guard)
        {
          cf_update_targets();
        }

      /* 平滑逼近，限制每拍最大转角。 */
      {
        int dp = g_pan_target - g_pan_cur;
        int dt = g_tilt_target - g_tilt_cur;

        if (dp > CF_SLEW_PER_TICK)  dp = CF_SLEW_PER_TICK;
        if (dp < -CF_SLEW_PER_TICK) dp = -CF_SLEW_PER_TICK;
        if (dt > CF_SLEW_PER_TICK)  dt = CF_SLEW_PER_TICK;
        if (dt < -CF_SLEW_PER_TICK) dt = -CF_SLEW_PER_TICK;

        g_pan_cur  += dp;
        g_tilt_cur += dt;

        if (dp != 0 || dt != 0)
          {
            /* [W962] Publish the angle as a TARGET and let the servo module
             * walk there on its own tick.
             *
             * The writes used to go straight to PWM.  That works, but it
             * bypasses the servo module, so the tracker and the servo module
             * each kept their own idea of the current angle and the two
             * drifted apart whenever the module clamped a write (a limit hit,
             * or a sub-debounce correction it dropped).
             *
             * servo_move_*() also gives a second layer of debounce: the module
             * refuses target changes smaller than its own deadband, so the
             * residual detector jitter that survives the EMA above never
             * reaches the hardware at all. */
            /* [V1030] What the control law actually asks the hardware
             * for, with the raw detector coordinate beside it.  1 in 8
             * (~2.5 Hz) so a moving head stays readable. */
            g_v1030_moved++;
            if ((g_v1030_moved & 7u) == 1u)
              {
                syslog(LOG_INFO, "[V1030] face -> pan=%d tilt=%d "
                       "(cx=%d cy=%d)\n",
                       g_pan_cur, g_tilt_cur, g_face.x, g_face.y);
              }
            servo_move_pan(g_pan_cur);
            servo_move_tilt(g_tilt_cur);
          }
      }

      /* 每 10 秒报一次健康度，不然出问题只能靠猜。 */
      if (now_ms() - last_report > 10000)
        {
          last_report = now_ms();

          /* [V1030] Unconditional heartbeat: printed whether or not any
           * datagram arrived, which is the only way to tell "nothing is
           * being sent" from "it is being sent but we drop it".
           * Reachable at all only because the socket is non-blocking
           * now. */
          syslog(LOG_INFO, "[V1030] face idle: rx=%u parsed=%u moved=%u "
                 "short=%u err=%u lines=%d faces=%d "
                 "cur=%d,%d tgt=%d,%d peer=%d\n",
                 g_v1030_rx_pkts, g_v1030_parsed, g_v1030_moved,
                 g_v1030_short, g_v1030_rx_err,
                 g_rx_lines, g_face_frames,
                 g_pan_cur, g_tilt_cur, g_pan_target, g_tilt_target,
                 g_have_peer);
          syslog(LOG_INFO,
                 "[%s] lines=%d faces=%d cur=(%d,%d) target=(%d,%d) "
                 "reject=%d peer=%d\n",
                 TAG, g_rx_lines, g_face_frames,
                 g_pan_cur, g_tilt_cur, g_pan_target, g_tilt_target,
                 g_reject_cnt, g_have_peer);
        }

      usleep(CF_LOOP_PERIOD_MS * 1000);
    }

  syslog(LOG_INFO, "[%s] task exiting\n", TAG);
  return NULL;
}

/* -------------------------------------------------------- API */

int companion_face_start(void)
{
  pthread_attr_t attr;
  pthread_t th;

  if (g_running)
    {
      return OK;
    }

  g_sock = cf_open_socket();
  if (g_sock < 0)
    {
      return g_sock;
    }

  g_running = 1;
  g_pan_target = g_pan_cur = 90;
  g_tilt_target = g_tilt_cur = 90;
  g_face.x = -1;
  g_reject_cnt = 0;
  g_rx_lines = 0;
  g_face_frames = 0;
  g_last_sweep_ms = now_ms();

  /* [V1043] Responsiveness tuning.  One line per constant so a
   * captured log states the running values instead of leaving them
   * to be guessed.  A single format string is reused on purpose:
   * DRAM has only ~112 bytes left. */
  syslog(LOG_INFO, "[V1043] tune %s=%d\n", "slew", CF_SLEW_PER_TICK);
  syslog(LOG_INFO, "[V1043] tune %s=%d\n", "dband", CF_DEADBAND);
  syslog(LOG_INFO, "[V1043] tune %s=%d\n", "ddeg", CF_MIN_DELTA_DEG);
  syslog(LOG_INFO, "[V1043] tune %s=%d\n", "rctr", CF_RECENTER_MS);
  syslog(LOG_INFO, "[V1043] tune %s=%d\n", "ema", CF_EMA_NUM);

  /* [V1044] Same shared-format-string trick as [V1043]: DRAM has
   * only ~112 bytes left, so these reuse one format string.  Gains
   * are printed x1000 to stay integer. */
  syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "gainp",
         (int)(CF_ABS_PAN_GAIN * 1000.0f));
  syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "gaint",
         (int)(CF_ABS_TILT_GAIN * 1000.0f));
  syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "pmin", CF_PAN_MIN);
  syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "pmax", CF_PAN_MAX);
  syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "tmin", CF_TILT_MIN);
  syslog(LOG_INFO, "[V1044] cfg %s=%d\n", "tmax", CF_TILT_MAX);

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 4096);

  if (pthread_create(&th, &attr, cf_task, NULL) != 0)
    {
      int e = errno;
      syslog(LOG_ERR, "[%s] pthread_create failed errno=%d\n", TAG, e);
      g_running = 0;
      close(g_sock);
      g_sock = -1;
      return -e;
    }

  pthread_attr_destroy(&attr);

  syslog(LOG_INFO, "[%s] thread started\n", TAG);

  return OK;
}

void companion_face_stop(void)
{
  g_running = 0;
  if (g_sock >= 0)
    {
      close(g_sock);
      g_sock = -1;
    }
}

void companion_face_set_track(int on)
{
  g_track_enabled = on ? 1 : 0;
  syslog(LOG_INFO, "[%s] track=%d\n", TAG, g_track_enabled);
}

int companion_face_get_track(void)
{
  return g_track_enabled;
}

/* 语音工具用：临时接管云台 angle 秒。 */
void companion_face_hold(int angle, int seconds)
{
  if (angle >= CF_PAN_MIN && angle <= CF_PAN_MAX)
    {
      g_pan_target = g_pan_cur = angle;
      servo_pan(angle);
    }

  g_manual_until_ms = now_ms() + (int64_t)seconds * 1000;
  syslog(LOG_INFO, "[%s] hold pan=%d for %ds\n", TAG, angle, seconds);
}

void companion_face_center(void)
{
  g_manual_until_ms = now_ms() + 3000;
  g_pan_target = g_pan_cur = 90;
  g_tilt_target = g_tilt_cur = 90;
  servo_center();
}

void companion_face_stats(int *lines, int *faces, int *pan,
                         int *tilt, int *have_peer)
{
  if (lines)     *lines = g_rx_lines;
  if (faces)     *faces = g_face_frames;
  if (pan)       *pan = g_pan_cur;
  if (tilt)      *tilt = g_tilt_cur;
  if (have_peer) *have_peer = g_have_peer;
}

int companion_face_peer(char *buf, size_t cap)
{
  if (!g_have_peer || cap < 16)
    {
      return -1;
    }

  /* g_peer_ip is already in network byte order, so inet_ntoa() takes it
   * as-is; do not shift it again. */
  snprintf(buf, cap, "%s", inet_ntoa(*(struct in_addr *)&g_peer_ip));
  return 0;
}