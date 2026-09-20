/*
 * Copyright (C) 2026 Xiaomi Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* audio_capture_native.c - Streaming capture through the standard NuttX
 * audio character device, bypassing the Vela media framework.
 *
 * Data flow (ASR voice path):
 *   ES8311 ADC -> I2S0 RX -> es8311 lower half fills apbs
 *   -> AUDIO_MSG_DEQUEUE on our private mq -> native_capture_read()
 *   copies s16le PCM out and re-arms the buffer
 *   -> audio_capture_read() applies capture gain
 *   -> voice_channel recording thread -> voice_asr (16 kHz mono).
 *
 * start() primes the pipeline with empty buffers before AUDIOIOC_START.
 * read() drains filled buffers (partial-buffer state is kept across
 * calls) and immediately re-enqueues drained ones to avoid overruns.
 */

#include "audio_native.h"

#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <pthread.h>
#include <sched.h>
#include <nuttx/audio/audio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

/* [V85] Cross-module diagnostic; diag_report() lives in voice/http_tts.c.
 * Rate-limited because a 1 Hz poll would otherwise fill the 2301 channel. */

extern int diag_report(const char *msg);



/* [V94] Counters published by arch/xtensa/src/esp32s3/esp32s3_i2s.c. */

extern volatile unsigned int g_v94_recv;
extern volatile int          g_v94_setupret;
extern volatile unsigned int g_v94_start;
extern volatile unsigned int g_v94_isr;
extern volatile unsigned int g_v94_eof;
extern volatile unsigned int g_v94_pb;
extern volatile int          g_v94_i2sret;
extern volatile unsigned int g_v94_done;
extern volatile int          g_v94_rxonly;

/* [V380] See the note at the top of patch_v380.py: phase 1 captures the
 * transmitter through the internal loopback, phase 2 captures the real
 * ES8311 with the transmitter supplying BCK/WS.
 */

extern volatile int          g_v380_loopback;
extern volatile int          g_v380_txmaster;
static int                   s_v380_phase = 1;
static void nc_probe(const char *tag)
{
    static int s_n;

    s_n++;
    if (s_n <= 12 || (s_n % 20) == 0) {
        diag_report(tag);
    }
}

static const char* TAG = "audio_cap_nat";

/* [V88] Clock keep-alive.
 *
 * RX is a slave on this port: i2s_configure() forces I2S_RX_SLAVE_MOD
 * whenever TX is also enabled, and sets I2S_SIG_LOOPBACK so RX takes BCLK
 * and WS from the transmitter.  The ESP32-S3 transmitter only drives them
 * while it has DMA descriptors to shift out - which is why playback is
 * audible and capture is silent.
 *
 * Rather than fake a transmitter in the driver, feed the real one: while a
 * capture runs, stream zeroed PCM to /dev/audio/pcm0p.  Zero samples are
 * inaudible but keep BCLK and WS alive, so the RX DMA sees frames and the
 * lower half starts dequeuing.
 *
 * Same primitives the TTS path uses, so nothing here is untested code. */

#define CAP_CLOCK_DEV       "/dev/audio/pcm0p"
#define CAP_CLOCK_RATE      16000
#define CAP_CLOCK_CHAN      1
#define CAP_CLOCK_BITS      16
#define CAP_CLOCK_STACK     16384

static pthread_t   s_clktid;
static volatile int s_clkrun;
static int         s_clkstarted;

static void* cap_clock_thread(void* arg)
{
    native_pb_t* pb = NULL;
    static uint16_t silence[512];      /* 1 KB of quiet PCM, in .bss */
    int rc;
    int k;

    /* [V380] Not silence any more: a DC value of 8 LSB (-72 dBFS,
     * inaudible on the speaker) makes the stream recognisable in the
     * capture dump, so "the RX path works" can be told apart from
     * "the RX path returns whatever the codec sends".
     */

    for (k = 0; k < 512; k++)
      {
        silence[k] = 0x0008;
      }

    rc = native_playback_open(&pb, CAP_CLOCK_DEV, CAP_CLOCK_RATE,
                              CAP_CLOCK_CHAN, CAP_CLOCK_BITS);
    if (rc != 0 || pb == NULL)
    {
        nc_probe("NC_CLK_OPENFAIL");
        return NULL;
    }

    nc_probe("NC_CLK_OPEN");

    while (s_clkrun)
    {
        int n = native_playback_write(pb, silence, sizeof(silence));

        if (n < 0)
        {
            nc_probe("NC_CLK_WRFAIL");
            break;
        }
    }

    native_playback_close(pb);
    nc_probe("NC_CLK_STOP");
    return NULL;
}

static void cap_clock_start(void)
{
    pthread_attr_t attr;
    struct sched_param sp;

    if (s_clkstarted)
    {
        return;
    }

    s_clkrun = 1;

    pthread_attr_init(&attr);
    /* Deliberately below the codec worker: this thread feeds it,
     * it must not starve it. */
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO) - 12;
    pthread_attr_setschedparam(&attr, &sp);
    pthread_attr_setstacksize(&attr, CAP_CLOCK_STACK);

    if (pthread_create(&s_clktid, &attr, cap_clock_thread, NULL) != 0)
    {
        s_clkrun = 0;
        nc_probe("NC_CLK_TFAIL");
        return;
    }

    pthread_detach(s_clktid);
    s_clkstarted = 1;
    nc_probe("NC_CLK_TSTART");
}

static void cap_clock_stop(void)
{
    if (!s_clkstarted)
    {
        return;
    }

    s_clkrun = 0;
    s_clkstarted = 0;
}

/* Pipeline geometry when the driver does not report its own (es8311 in
 * this build does not).  4096 B @ 16 kHz mono s16le = 128 ms per buffer,
 * 4 buffers deep (~512 ms of protection against read-side hiccups). */

#define CAP_FALLBACK_NBUFFERS  2   /* [V312] 2 slots: shares the I2S driver's
                                     * 2-slot internal-SRAM bounce pool with
                                     * TX (they never run concurrently, V89).
                                     * 4 would need +16KB static BSS, which
                                     * shifts .bss and breaks WiFi assoc. */
#define CAP_FALLBACK_BUFSIZE   4096

#define CAP_MAX_NBUFFERS       2   /* [V312] see CAP_FALLBACK_NBUFFERS */
#define CAP_MAX_BUFSIZE        16384

/* read() wait per mq_receive attempt (retried until stopped). */

#define CAP_MSG_TIMEOUT_SEC    1

/* [V85] How many consecutive empty waits before read() gives up and lets the
 * caller restart.  Without this the retry loop below never exits. */

#define CAP_MQ_MAX_TIMEOUTS    6

struct native_cap_s {
    int fd;
    mqd_t mq;
    char mqname[NATIVE_MQ_NAME_LEN];

    struct ap_buffer_s** apbs;    /* all allocated apbs */
    int nbuffers;
    int outstanding;              /* armed/enqueued at the driver */

    struct ap_buffer_s** freeq;   /* apbs drained and waiting re-arm */
    int nfree;

    struct ap_buffer_s* pending;  /* filled apb being consumed */
    size_t pending_off;

    volatile int started;
    volatile int stopped;
    int complete;
};

/* Fetch the next filled buffer from the mq (or keep draining stale
 * empties).  Returns 1 with cap->pending set, 0 on timeout, -1 when the
 * stream has ended (stopped/completed) or on error. */

static int cap_next_filled(struct native_cap_s* cap)
{
    struct audio_msg_s msg;
    int ret;

    while (1) {
        ret = native_mq_receive_timed(cap->mq, &msg, CAP_MSG_TIMEOUT_SEC);
        if (ret <= 0) {
            static int s_to;

            if (cap->stopped) {
                nc_probe("NC_MQ_STOP");
                return -1;
            }

            /* [V85] This used to be a bare continue, which made the timeout
             * meaningless: with no filled buffer ever arriving, read() sat
             * here for good and took the whole listen thread with it.
             * Count the empty waits and give up, so the caller sees -EIO
             * and can restart the pipeline instead of hanging. */

            s_to++;
            if ((s_to % 3) == 0) {
                char tb[48];

                snprintf(tb, sizeof(tb),
                         "V94_R%u_S%d_T%u_I%u_E%u_P%u_X%d_D%u_O%d",
                         (unsigned)g_v94_recv, (int)g_v94_setupret,
                         (unsigned)g_v94_start, (unsigned)g_v94_isr,
                         (unsigned)g_v94_eof, (unsigned)g_v94_pb,
                         (int)g_v94_i2sret, (unsigned)g_v94_done,
                         (int)g_v94_rxonly);
                diag_report(tb);
            }

            if (s_to >= CAP_MQ_MAX_TIMEOUTS) {
                nc_probe("NC_MQ_GIVEUP");
                return -1;
            }

            continue;
        }

        switch (msg.msg_id) {
        case AUDIO_MSG_DEQUEUE:
            {
                static int s_dq;

                s_dq++;
                if (s_dq <= 8 || (s_dq % 25) == 0) {
                    char db[32];

                    snprintf(db, sizeof(db), "NC_DQ_%d", s_dq);
                    nc_probe(db);
                }
            }
            if (!msg.u.ptr || cap->outstanding <= 0) {
                break;
            }

            cap->outstanding--;
            if (((struct ap_buffer_s*)msg.u.ptr)->nbytes > 0) {
                cap->pending = (struct ap_buffer_s*)msg.u.ptr;
                cap->pending_off = 0;
                return 1;
            }

            /* Empty buffer (returned on stop / spurious): keep it for
             * re-arming instead of dropping it. */

            cap->freeq[cap->nfree++] = (struct ap_buffer_s*)msg.u.ptr;
            break;

        case AUDIO_MSG_COMPLETE:
            cap->complete = 1;
            syslog(LOG_INFO, "[%s] driver completed capture\n", TAG);
            if (cap->outstanding <= 0 && !cap->pending) {
                return -1;
            }
            break;

        case AUDIO_MSG_STOP:
            cap->stopped = 1;
            return -1;

        default:
            break;
        }
    }
}

/* Re-arm a drained buffer: enqueue it back as empty so the driver can
 * fill it again.  After stop/abort the buffer just goes to the free
 * pool.  Returns 0 or -errno. */

static int cap_rearm(struct native_cap_s* cap, struct ap_buffer_s* apb)
{
    int ret;

    if (cap->stopped || !cap->started) {
        cap->freeq[cap->nfree++] = apb;
        return 0;
    }

    apb->nbytes = apb->nmaxbytes;   /* [V86] see native_capture_start() */
    apb->curbyte = 0;

    ret = native_enqueue_buffer(cap->fd, apb);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] re-arm enqueue failed: %d\n", TAG, ret);
        cap->freeq[cap->nfree++] = apb;
        return ret;
    }

    cap->outstanding++;
    return 0;
}

int native_capture_open(native_cap_t** out_cap, const char* dev_path,
    unsigned int sample_rate, unsigned int channels,
    unsigned int bits_per_sample)
{
    struct native_cap_s* cap;
    struct ap_buffer_info_s bufinfo;
    size_t buf_size;
    int ret;
    int i;

    if (!out_cap || !dev_path || channels == 0 || channels > 2) {
        return -EINVAL;
    }

    *out_cap = NULL;

    cap = calloc(1, sizeof(*cap));
    if (!cap) {
        return -ENOMEM;
    }

    cap->fd = -1;
    cap->mq = -1;

    /* Open the raw audio device (e.g. /dev/audio/pcm0c). */

    cap->fd = open(dev_path, O_RDWR);
    if (cap->fd < 0) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] open(%s) failed: %d\n", TAG, dev_path, ret);
        goto err_out;
    }

    if (ioctl(cap->fd, AUDIOIOC_RESERVE, 0) != OK) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] AUDIOIOC_RESERVE failed: %d\n", TAG, ret);
        goto err_out;
    }

    ret = native_mq_create(cap->mqname, sizeof(cap->mqname), &cap->mq);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] mq_create failed: %d\n", TAG, ret);
        goto err_out;
    }

    if (ioctl(cap->fd, AUDIOIOC_REGISTERMQ, (unsigned long)cap->mq) != OK) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] REGISTERMQ failed: %d\n", TAG, ret);
        goto err_out;
    }

    ret = native_check_caps(cap->fd, AUDIO_TYPE_INPUT);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] %s is not an input audio device\n",
            TAG, dev_path);
        goto err_out;
    }

    ret = native_configure(cap->fd, AUDIO_TYPE_INPUT, sample_rate,
        channels, bits_per_sample);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] configure %uHz %uch %ubit failed: %d\n",
            TAG, sample_rate, channels, bits_per_sample, ret);
        goto err_out;
    }

    /* Pipeline geometry: prefer the driver's own numbers. */

    cap->nbuffers = CAP_FALLBACK_NBUFFERS;
    buf_size = CAP_FALLBACK_BUFSIZE;
    if (ioctl(cap->fd, AUDIOIOC_GETBUFFERINFO,
              (unsigned long)&bufinfo) == OK &&
        bufinfo.nbuffers > 0 && bufinfo.buffer_size > 0) {
        cap->nbuffers = (bufinfo.nbuffers > CAP_MAX_NBUFFERS)
            ? CAP_MAX_NBUFFERS : (int)bufinfo.nbuffers;
        buf_size = (bufinfo.buffer_size > CAP_MAX_BUFSIZE)
            ? CAP_MAX_BUFSIZE : (size_t)bufinfo.buffer_size;
    }

    cap->apbs = calloc(cap->nbuffers, sizeof(*cap->apbs));
    cap->freeq = calloc(cap->nbuffers, sizeof(*cap->freeq));
    if (!cap->apbs || !cap->freeq) {
        ret = -ENOMEM;
        goto err_out;
    }

    for (i = 0; i < cap->nbuffers; i++) {
        cap->apbs[i] = native_alloc_buffer(cap->fd, buf_size);
        if (!cap->apbs[i]) {
            ret = -ENOMEM;
            syslog(LOG_ERR, "[%s] alloc apb %d failed\n", TAG, i);
            goto err_out;
        }

        cap->freeq[cap->nfree++] = cap->apbs[i];
    }

    *out_cap = cap;
    syslog(LOG_INFO,
        "[%s] native capture ready (%s, %uHz %uch %ubit, %d x %u B)\n",
        TAG, dev_path, sample_rate, channels, bits_per_sample,
        cap->nbuffers, (unsigned)buf_size);
    return 0;

err_out:
    native_capture_close(cap);
    return ret;
}

int native_capture_start(struct native_cap_s* cap)
{
    int ret;
    int i;

    if (!cap) {
        return -EINVAL;
    }

    if (cap->started) {
        return 0;
    }

    if (cap->stopped) {
        return -ECANCELED;
    }

    /* [V85] Prime the pipeline: all empty buffers armed before START so the
     * ADC DMA has somewhere to put the first frames.  Each step is probed
     * because if start() never returns, AC_ST_BEG is the last thing the
     * board ever says and there is no way to tell enqueue from ioctl. */

    nc_probe("NC_ST_BEG");

    /* [V373] Re-enable the silence-playback clock keeper.
     * WHY: the ES8311 only shifts ADC samples out while it is being
     * clocked, and on this board BCLK/WS come from the ESP32-S3 TX
     * generator.  With TX stopped there is no frame clock on the pins,
     * so the codec never puts a single bit on DIN and the RX GDMA waits
     * forever:
     *   [RXDUMP] arm#1 ... isr=0 eof=0  ->  capture EIO, retry 1/5..5/5
     *   [voice] listen: no speech for 8000 ms
     * [V89] had disabled this because opening pcm0p WHILE pcm0c was
     * already streaming deadlocked the two es8311 instances.  Starting
     * the playback side BEFORE the capture is armed avoids that order. */
    /* [V379] cap_clock_start() is OFF again.
     *
     * It kept the I2S transmitter alive by streaming zeroed PCM to
     * /dev/audio/pcm0p during a capture, which is what made RX produce data
     * in the first place.  It is also what kills the board: every 1 KB write
     * restarts the TX DMA (captured live: "[V347] txdma_start #29" followed
     * immediately by "xtensa_user_panic: EXCCAUSE=001c task: es8311"), i.e.
     * the second es8311 instance running while the capture instance is
     * already streaming -- the exact hazard the [V89] note above describes.
     * Enlarging the worker stack to 4096 (V377) did not save it: the dump
     * showed both workers already at 4064 when it died.
     *
     * With V374 having removed the boot speaker test and the always-on
     * listen, nothing else opens pcm0p, so i2s_configure() no longer forces
     * I2S_RX_SLAVE_MOD and the capture side can clock itself. */

    /* [V380] Back ON: phase 1 needs a running transmitter to loop back,
     * phase 2 needs it to put BCK/WS on the pins for the ES8311.
     * The port ping-pong that used to crash here is gone because
     * g_v380_txmaster makes esp32s3_i2s_set_rxonly() a no-op.
     */

    /* [V382] OFF again.  Opening pcm0p while pcm0c is streaming is the
     * [V89] hazard, and v381 proved it still panics the board even with
     * the port ownership locked (EXCCAUSE=001c in the es8311 worker as
     * soon as the keeper's TX DMA starts).  The clock now comes from the
     * [V382] tx clock-only start inside i2s_rx_channel_start().
     */

    /* cap_clock_start(); */

    /* [V380] Phase 1 routes the transmitter back into the receiver; it
     * is off everywhere else so it cannot disturb TTS playback.
     */

    /* [V382] The loopback experiment is parked: with the clock keeper
     * gone the transmitter has no payload to loop back, so it could only
     * ever produce zeros and would tell us nothing.  Phase 1 now just
     * labels the boot self-test.
     */

    if (s_v380_phase == 1)
      {
        /* [V383] Loop the transmitter back into the receiver inside the
         * chip.  The transmitter is running clock-only, so what comes
         * back is zeros -- but the point is whether ANY buffer comes
         * back at all.  If it does, the RX GDMA + SUC_EOF path is fine
         * and the codec / pins are the remaining suspects; if it does
         * not, our receiver cannot move a frame even when one is handed
         * to it on chip.
         */

        g_v380_loopback = 1;
        syslog(LOG_INFO, "[V383] boot self-test: internal loopback armed\n");
      }

    for (i = 0; i < cap->nfree; i++) {
        struct ap_buffer_s* apb = cap->freeq[i];
        char eb[32];

        /* [V86] nbytes is what the driver is being asked to transfer, not
         * what it has already transferred.  es8311_processbegin() turns
         * (nbytes - curbyte) into the I2S timeout, so arming an empty
         * capture buffer with nbytes = 0 asks for zero bytes in zero
         * ticks: I2S_RECEIVE gives up at once, the lower half breaks out
         * of its loop, and no buffer is ever dequeued back to us.  Offer
         * the whole buffer instead. */

        apb->nbytes = apb->nmaxbytes;
        apb->curbyte = 0;

        snprintf(eb, sizeof(eb), "NC_ST_EQ%d_N%u", i,
                 (unsigned)apb->nmaxbytes);
        nc_probe(eb);

        ret = native_enqueue_buffer(cap->fd, apb);
        if (ret < 0) {
            syslog(LOG_ERR, "[%s] prime enqueue failed: %d\n", TAG, ret);
            nc_probe("NC_ST_EQFAIL");
            return ret;
        }

        cap->outstanding++;
    }
    cap->nfree = 0;

    nc_probe("NC_ST_IOCTL");

    if (ioctl(cap->fd, AUDIOIOC_START, 0) != OK) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] AUDIOIOC_START failed: %d\n", TAG, ret);
        nc_probe("NC_ST_IOFAIL");
        return ret;
    }

    cap->started = 1;
    syslog(LOG_INFO, "[%s] capture started (%d buffers armed)\n",
        TAG, cap->outstanding);

    {
        char ob[32];

        snprintf(ob, sizeof(ob), "NC_ST_OK_%d", cap->outstanding);
        nc_probe(ob);
    }
    return 0;
}

int native_capture_read(struct native_cap_s* cap, void* buf, size_t len)
{
    unsigned char* dst = buf;

    if (!cap || !buf || len == 0) {
        return -EINVAL;
    }

    if (cap->stopped) {
        return -ECANCELED;
    }

    while (1) {
        struct ap_buffer_s* apb;
        size_t avail;
        size_t n;

        if (cap->stopped) {
            return -ECANCELED;
        }

        /* Serve the partially-consumed filled buffer first. */

        apb = cap->pending;
        if (apb) {
            avail = (size_t)apb->nbytes - cap->pending_off;
            if (avail == 0) {
                if (cap_rearm(cap, apb) < 0) {
                    return -EIO;
                }
                cap->pending = NULL;
                continue;
            }

            n = (avail < len) ? avail : len;
            memcpy(dst, apb->samp + cap->pending_off, n);
            cap->pending_off += n;

            /* [V380] What did the port actually hand us?  A healthy
             * loopback shows bytes 08 00 08 00 ... with nz == n/2 and
             * peak == 8.  Real microphone audio shows a much larger
             * peak.  All-zero with nz == 0 is the starvation case.
             */

            {
              static int s_v380_buf;
              const unsigned char *vp = (const unsigned char *)apb->samp +
                                        cap->pending_off - n;
              size_t j;
              size_t nz = 0;
              long peak = 0;

              s_v380_buf++;

              for (j = 0; j + 1 < n; j += 2)
                {
                  long v = (long)(short)((unsigned)vp[j]
                                          | ((unsigned)vp[j + 1] << 8));

                  if (v < 0)
                    {
                      v = -v;
                    }

                  if (v > peak)
                    {
                      peak = v;
                    }

                  if (vp[j] != 0 || vp[j + 1] != 0)
                    {
                      nz++;
                    }
                }

              if ((s_v380_buf % 4) == 1 && n >= 8)
                {
                  syslog(LOG_INFO, "[V380] ph=%d buf#%d n=%u nz=%u "
                         "peak=%ld b=%02x%02x %02x%02x %02x%02x %02x%02x\n",
                         s_v380_phase, s_v380_buf, (unsigned)n, (unsigned)nz,
                         peak, vp[0], vp[1], vp[2], vp[3],
                         vp[4], vp[5], vp[6], vp[7]);
                }
            }

            if (cap->pending_off >= (size_t)apb->nbytes) {
                if (cap_rearm(cap, apb) < 0) {
                    return -EIO;
                }
                cap->pending = NULL;
            }

            return (int)n;
        }

        /* No pending data: wait for the driver to hand one back. */

        if (cap_next_filled(cap) <= 0) {
            if (cap->stopped) {
                return -ECANCELED;
            }
            if (cap->complete) {
                return -EPIPE;
            }
            return -EIO;
        }
    }
}

/* [V381] Boot-time capture probe.
 *
 * The V380 diagnostics only print while a capture session is open, and a
 * session used to need somebody to hold the BOOT key at exactly the right
 * moment -- v380run.log has BTN_PRESS = 0 for the whole window, so the one
 * measurement that matters was never taken.  This runs the same open/start/
 * read/abort sequence by itself, once, right after the boot greeting.
 *
 * native_capture_start() is what arms the phase-1 loopback and the clock
 * keeper, so the [V380] dump lines inside native_capture_read() tell us
 * whether the RX DMA path moves any bytes at all.  native_capture_abort()
 * then flips the board to phase 2, so a later real BOOT press measures the
 * true ES8311 path in the same firmware.
 */

/* [V401] 采集自检开关。
 * 1 = 开机跑（当前必崩，用于复现）；0 = 跳过（设备可用）。
 *
 * V398/V400 实测：探针跑起来后 RX 确实收到了第一帧 ——
 *   [RXEOF] first SUC_EOF! isr=1、[RXBOUNCE] 回灌到 PSRAM 缓冲、
 *   [AUDIO-R42] xfer ret=0
 * 但紧接着系统崩溃：EXCCAUSE=001c、VADDR=0x00000000，调用链为
 *   irq_dispatch -> wDev_ProcessFiq -> xtensa_user -> xtensa_user_panic
 *   -> _assert -> up_saveusercontext
 * 也就是崩溃转储时向空指针保存现场（tcb->xcp.regs 为 NULL 时
 * xtensa_user_panic 把 NULL 传了下去），真正的第一现场在 WiFi 闭源 FIQ
 * 与 RX 中断的并发里，被这次二次崩溃掩盖了。
 * 中断栈实测只用了 336 字节（4096 / 6144 / 8192 三档都一样），
 * 所以不是栈容量问题。今晚先把设备保住，置 1 即可继续排查。 */
#define V401_CAPTURE_PROBE 0

void native_capture_selftest(void)
{
#if V401_CAPTURE_PROBE == 0
    syslog(LOG_INFO, "[V401] capture probe parked "
                     "(V401_CAPTURE_PROBE=0)\n");
    return;
#else
    native_cap_t* cap = NULL;
    static unsigned char sbuf[4096];
    int rc;
    int i;
    int got = 0;

    syslog(LOG_INFO, "[V380] selftest begin (phase=%d)\n", s_v380_phase);

    rc = native_capture_open(&cap, "/dev/audio/pcm0c", 16000, 1, 16);
    if (rc != 0 || cap == NULL)
      {
        syslog(LOG_ERR, "[V380] selftest open failed: %d\n", rc);
        return;
      }

    rc = native_capture_start(cap);
    if (rc != 0)
      {
        syslog(LOG_ERR, "[V380] selftest start failed: %d\n", rc);
        native_capture_abort(cap);
        native_capture_close(cap);
        return;
      }

    for (i = 0; i < 8; i++)
      {
        int n = native_capture_read(cap, sbuf, sizeof(sbuf));

        if (n > 0)
          {
            got += n;
            if (got >= 8192)
              {
                break;
              }
          }
        else if (n == -EIO)
          {
            /* cap_next_filled() already waited CAP_MSG_TIMEOUT_SEC. */
            continue;
          }
        else
          {
            syslog(LOG_INFO, "[V380] selftest read rc=%d\n", n);
            break;
          }
      }

    syslog(LOG_INFO, "[V380] selftest end: %d bytes in %d reads\n", got, i);

    native_capture_abort(cap);
    native_capture_close(cap);
#endif
}

int native_capture_abort(struct native_cap_s* cap)
{
    struct audio_msg_s msg;

    if (!cap) {
        return -EINVAL;
    }

    /* [V88] Let the silence stream go: nothing is capturing any more. */

    cap_clock_stop();

    /* [V380] Leave the loopback behind and remember that the next
     * session listens to the real ES8311.
     */

    g_v380_loopback = 0;

    if (s_v380_phase == 1)
      {
        s_v380_phase = 2;
        syslog(LOG_INFO, "[V380] phase 2: loopback OFF, TX master kept, "
               "real codec from now on\n");
      }

    if (cap->started) {
        cap->started = 0;
        ioctl(cap->fd, AUDIOIOC_STOP, 0);
    }

    cap->stopped = 1;

    /* Drain completion messages so nothing stays queued on the mq. */

    while (native_mq_receive_timed(cap->mq, &msg, 1) > 0) {
        if (msg.msg_id == AUDIO_MSG_DEQUEUE && msg.u.ptr &&
            cap->outstanding > 0) {
            cap->outstanding--;
            cap->freeq[cap->nfree++] = (struct ap_buffer_s*)msg.u.ptr;
        }
    }

    return 0;
}

void native_capture_close(struct native_cap_s* cap)
{
    struct audio_msg_s msg;
    int i;

    if (!cap) {
        return;
    }

    /* Stop the stream so in-flight buffers come back before we free
     * them, then drain the completion queue. */

    if (cap->fd >= 0 && (cap->started || cap->outstanding > 0)) {
        cap->started = 0;
        ioctl(cap->fd, AUDIOIOC_STOP, 0);
    }

    cap->stopped = 1;

    if (cap->mq >= 0) {
        while (native_mq_receive_timed(cap->mq, &msg, 1) > 0) {
            if (msg.msg_id == AUDIO_MSG_DEQUEUE && msg.u.ptr &&
                cap->outstanding > 0) {
                cap->outstanding--;
                cap->freeq[cap->nfree++] =
                    (struct ap_buffer_s*)msg.u.ptr;
            }
        }

        ioctl(cap->fd, AUDIOIOC_UNREGISTERMQ, (unsigned long)cap->mq);
        mq_close(cap->mq);
        mq_unlink(cap->mqname);
        cap->mq = -1;
    }

    if (cap->fd >= 0) {
        ioctl(cap->fd, AUDIOIOC_RELEASE, 0);

        if (cap->apbs) {
            for (i = 0; i < cap->nbuffers; i++) {
                native_free_buffer(cap->fd, cap->apbs[i]);
            }
        }

        close(cap->fd);
        cap->fd = -1;
    }

    free(cap->apbs);
    free(cap->freeq);
    free(cap);

    syslog(LOG_INFO, "[%s] closed\n", TAG);
}
