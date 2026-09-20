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

/* audio_playback.c - Direct ES8311 + I2S0-TX playback backend for VOX3.
 *
 * The previous native backend drove the codec through the NuttX AUDIO
 * upper-half (AUDIOIOC_* ioctls on /dev/audio/pcm0p).  On this board that
 * path received PCM from the TTS server but never produced sound: the opaque
 * framework left the ES8311 DAC output path disabled.  This backend instead
 * programs the ES8311 DAC and the ESP32-S3 I2S0 TX DMA directly, using the
 * exact register sequence the proven Arduino firmware (voice_chat_src) uses to
 * make the same hardware speak.  It bypasses the AUDIO upper-half entirely.
 *
 * The public API (audio_playback_open/write/stop/close) is unchanged so
 * voice_channel.c and the Makefile need no edits.
 */

#include "voice/audio_playback.h"
#include "agent_config.h"
#include "voice/http_tts.h"

#include <nuttx/config.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

/* The ESP32-S3 bus-init entry points are provided by the arch layer and are
 * not always surfaced through the board header in package builds. */
struct i2c_master_s;
struct i2s_dev_s;
FAR struct i2c_master_s *esp32s3_i2cbus_initialize(int port);
FAR struct i2s_dev_s    *esp32s3_i2sbus_initialize(int port);

/* [V222] Self-contained, ESP-IDF-ordered I2S0 TX one-shot write implemented in
 * arch/xtensa/src/esp32s3/esp32s3_i2s.c.  Declared here directly, exactly like
 * the two bus initialisers above, so no shared header has to change. */


#include <errno.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

/* Wiring mirrors esp32s3_board_audio.c */
#define ES_I2C_PORT  0
#define ES_I2C_ADDR  0x18
#define ES_I2C_FREQ  400000
#define ES_I2S_PORT  0

#define ES_SAMPLE_RATE  16000
#define ES_BITS         16
#define ES_CHANNELS     1

/* ES8311 register addresses */
#define ES_REG_RESET    0x00
#define ES_REG_CLKPOW   0x01
#define ES_REG_DAC_EQ   0x37
#define ES_REG_ANA1     0x0D
#define ES_REG_ANA2     0x0E
#define ES_REG_PWR_DAC  0x12
#define ES_REG_OUT_EN   0x13
#define ES_REG_DAC_VOL  0x32

static const char *TAG = "audio_direct";
static int g_tx_timeouts;

static struct i2c_master_s *g_i2c;
static struct i2s_dev_s    *g_i2s;
static bool                 g_inited;

struct audio_playback
{
  bool stopped;
};

/* ---- ES8311 I2C helpers ---- */

static int g_i2c_errs;

static void es_writereg(uint8_t reg, uint8_t val)
{
  int rc;

  if (!g_i2c)
    {
      syslog(LOG_ERR, "[%s] es_writereg(0x%02x): no i2c bus\n", TAG, reg);
      return;
    }

  uint8_t buf[2] = { reg, val };
  struct i2c_config_s cfg =
    {
      .frequency = ES_I2C_FREQ,
      .address   = ES_I2C_ADDR,
      .addrlen   = 7
    };

  rc = i2c_write(g_i2c, &cfg, buf, 2);
  if (rc < 0)
    {
      if (g_i2c_errs < 6)
        {
          g_i2c_errs++;
          syslog(LOG_ERR, "[%s] es_writereg(0x%02x,0x%02x) FAILED rc=%d\n",
                 TAG, reg, val, rc);
        }
    }
}

static uint8_t es_readreg(uint8_t reg)
{
  if (!g_i2c)
    {
      return 0;
    }

  uint8_t w = reg;
  uint8_t r = 0;
  struct i2c_config_s cfg =
    {
      .frequency = ES_I2C_FREQ,
      .address   = ES_I2C_ADDR,
      .addrlen   = 7
    };

  i2c_write(g_i2c, &cfg, &w, 1);
  i2c_read(g_i2c, &cfg, &r, 1);
  return r;
}

/* Program ES8311 for DAC playback using the proven register sequence. */
static void es8311_dac_init(void)
{
  uint8_t rv;

  /* [V248] *** ES8311 initialisation rewritten to the official esp_codec_dev
   * sequence, taken from the reference source tree that ships with the working
   * firmware on this board
   *   <VOX3>/ai_vox_ref/ai_vox-main/src/core/espressif_esp_codec_dev/es8311.c
   * (es8311_open() + es8311_set_bits_per_sample() + es8311_config_fmt() +
   *  es8311_config_sample() + es8311_start()).
   *
   * The previous table was wrong in five places, two of which are fatal:
   *
   *   reg 0x07 (LRCK divider high)  was 0x3F, official 0x00
   *   reg 0x08 (LRCK divider low)   was 0x10, official 0xFF
   *   reg 0x04 (DAC OSR)            was 0x10, official 0x20
   *   reg 0x44 (internal reference) was 0x50, official 0x58
   *   reg 0x37 (DAC equaliser)      was 0x48, official 0x08
   *
   * 0x07/0x08 are the codec's LRCK division for MCLK = 4096 kHz / fs = 16 kHz
   * (coeff_div[] entry {4096000, 16000, ... lrck_h 0x00, lrck_l 0xff ...}).
   * With them wrong the codec's internal sample clock does not match the I2S
   * frame at all, so the DAC only ever emits its noise floor -- the hiss.
   */

  /* ---- es8311_open(): power-on sequence ---- */

  es_writereg(0x44, 0x08);            /* I2C noise immunity (twice)      */
  es_writereg(0x44, 0x08);

  es_writereg(0x01, 0x30);
  es_writereg(0x02, 0x00);
  es_writereg(0x03, 0x10);
  es_writereg(0x16, 0x24);
  es_writereg(0x04, 0x10);
  es_writereg(0x05, 0x00);
  es_writereg(0x0B, 0x00);
  es_writereg(0x0C, 0x00);
  es_writereg(0x10, 0x1F);
  es_writereg(0x11, 0x7F);
  es_writereg(0x00, 0x80);

  rv = es_readreg(0x00) & 0xBF;       /* slave mode                      */
  es_writereg(0x00, rv);

  es_writereg(0x01, 0x3F);            /* MCLK from pin, not inverted     */

  rv = es_readreg(0x06) & ~0x20;      /* SCLK not inverted               */
  es_writereg(0x06, rv);

  es_writereg(0x13, 0x10);
  es_writereg(0x1B, 0x0A);
  es_writereg(0x1C, 0x6A);

  es_writereg(0x44, 0x58);            /* internal reference ADCL + DACR  */

  /* ---- 16-bit word length ---- */

  rv = es_readreg(0x09) | 0x0C;
  es_writereg(0x09, rv);
  rv = es_readreg(0x0A) | 0x0C;
  es_writereg(0x0A, rv);

  /* ---- format: standard I2S (clear bit 6 on both directions) ---- */

  rv = es_readreg(0x09) & ~0x40;
  es_writereg(0x09, rv);
  rv = es_readreg(0x0A) & ~0x40;
  es_writereg(0x0A, rv);

  /* ---- clock: MCLK 4096 kHz, fs 16000 Hz (coeff_div entry) ---- */

  rv = es_readreg(0x02) & 0x07;       /* pre_div = 1, pre_multi = 1      */
  es_writereg(0x02, rv);

  es_writereg(0x05, 0x00);            /* adc_div = dac_div = 1           */

  rv = (es_readreg(0x03) & 0x80) | 0x10;
  es_writereg(0x03, rv);              /* fs_mode = 0, adc_osr = 0x10     */

  rv = (es_readreg(0x04) & 0x80) | 0x20;
  es_writereg(0x04, rv);              /* dac_osr = 0x20                  */

  rv = es_readreg(0x07) & 0xC0;       /* lrck_h = 0x00                   */
  es_writereg(0x07, rv);

  es_writereg(0x08, 0xFF);            /* lrck_l = 0xFF                   */

  rv = (es_readreg(0x06) & 0xE0) | (0x04 - 1);
  es_writereg(0x06, rv);              /* bclk_div = 4                    */

  /* ---- es8311_start(): enable the analogue path ---- */

  rv = es_readreg(0x09) & ~0x40;
  es_writereg(0x09, rv);
  rv = es_readreg(0x0A) & ~0x40;
  es_writereg(0x0A, rv);

  es_writereg(0x17, 0xBF);
  es_writereg(0x0E, 0x02);
  es_writereg(0x12, 0x00);
  es_writereg(0x14, 0x1A);

  rv = es_readreg(0x14) & ~0x40;      /* analogue mic, not digital       */
  es_writereg(0x14, rv);

  es_writereg(0x0D, 0x01);
  es_writereg(0x15, 0x40);
  es_writereg(0x37, 0x08);
  es_writereg(0x45, 0x00);

  /* ---- DAC volume ---- */

  es_writereg(ES_REG_DAC_VOL, 0xBF);
}

/* ---- I2S TX completion callback ---- */

static void direct_tx_done(FAR struct i2s_dev_s *dev,
                           FAR struct ap_buffer_s *apb,
                           FAR void *arg, int result)
{
  sem_t *done = (sem_t *)arg;
  if (done)
    {
      sem_post(done);
    }
}

/* [V207] CRITICAL FIX.
 *
 * This probe used to call http_tts_speak_stream(), i.e. it asked the TTS
 * server to synthesise speech *from inside audio_playback_open()*.  The open
 * path runs under voice_channel's speak_lock, so the first playback of every
 * boot paid four extra blocking HTTP round trips (4 attempts x 15 s connect +
 * 8 s recv budget) before a single sample reached the codec -- and when the
 * link was slow the whole speak path never returned, which is exactly the
 * "board goes silent ~10 s after boot" symptom.  The beacon also exercised
 * the very audio path it was meant to observe, so it could never have been a
 * valid probe.
 *
 * Probes now travel on the bridge's lightweight /diag endpoint (3 s connect
 * cap) and are always mirrored to syslog so they show up on the serial
 * console as well. */
static void ap_beacon(const char *tag)
{
  syslog(LOG_INFO, "[%s] probe %s\n", TAG, tag);
  diag_report(tag);
}

/* ---- public API ---- */

audio_playback_t *audio_playback_open(const char *dev_path,
    unsigned int sample_rate, unsigned int channels,
    unsigned int bits_per_sample)
{
  struct audio_playback *pb;

  if (!g_inited)
    {
      g_i2c = esp32s3_i2cbus_initialize(ES_I2C_PORT);
      g_i2s = esp32s3_i2sbus_initialize(ES_I2S_PORT);
      if (!g_i2c || !g_i2s)
        {
          syslog(LOG_ERR, "[%s] i2c/i2s init failed (i2c=%p i2s=%p)\n",
                 TAG, g_i2c, g_i2s);
          return NULL;
        }

      ap_beacon("B7_AP_BUSOK");
      g_inited = true;
    }

  /* Configure codec DAC + I2S0 TX for 16k/16bit/mono.  These ops also start
   * the TX channel, so playback begins as soon as we push PCM. */
  es8311_dac_init();
  ap_beacon("B7_AP_ESOK");

  /* [V208] Read the codec back over I2C.  If the bus is dead, every write
   * above was silently dropped and the DAC is mute no matter what the I2S
   * side does -- so prove the bus first.  Reg 0x00 should read 0x80 and the
   * DAC volume register 0x32 should read back the value we wrote. */
  {
    uint8_t rb0 = es_readreg(ES_REG_RESET);
    uint8_t rb32 = es_readreg(ES_REG_DAC_VOL);
    syslog(LOG_INFO,
           "[%s] codec readback: r00=0x%02x (want 0x80) r32=0x%02x (want 0x%02x) "
           "i2c_errs=%d\n", TAG, rb0, rb32, 0xCB, g_i2c_errs);
  }

  /* [V208] Every one of these returns a meaningful value: a 0 from
   * I2S_TXSAMPLERATE means i2s_check_mclkfrequency() rejected the rate, in
   * which case the TX channel is left STOPPED and I2S_SEND never completes
   * (the write then blocks forever, because the timeout we pass is 0). */
  {
    uint32_t dw = I2S_TXDATAWIDTH(g_i2s, ES_BITS);
    uint32_t mk = I2S_SETMCLKFREQUENCY(g_i2s, ES_SAMPLE_RATE * 256);
    int      ch = I2S_TXCHANNELS(g_i2s, ES_CHANNELS);
    uint32_t sr = I2S_TXSAMPLERATE(g_i2s, ES_SAMPLE_RATE);

    syslog(LOG_INFO,
           "[%s] i2s cfg: datawidth=%u(rc) mclk=%u rc_txchannels=%d "
           "samplerate_rc=%u\n", TAG, (unsigned)dw, (unsigned)mk, ch,
           (unsigned)sr);
  }
  ap_beacon("B7_AP_I2SOK");

  pb = calloc(1, sizeof(struct audio_playback));
  if (!pb)
    {
      return NULL;
    }

  pb->stopped = false;
  syslog(LOG_INFO,
         "[%s] direct playback opened (req rate=%u ch=%u bits=%u)\n",
         TAG, sample_rate, channels, bits_per_sample);
  ap_beacon("B7_AP_RET");
  return pb;
}

/* [V212] Upper bound for a single transfer.
 *
 * The I2S driver copies every transfer into an internal-SRAM DMA bounce
 * buffer (esp32s3_i2s.c, I2S_DMABUF_SLOT_SZ) because ESP32-S3 GDMA cannot
 * fetch from PSRAM, and that slot is deliberately small to keep .bss off the
 * already-tight internal heap.  Anything larger is refused by the driver, so
 * audio_playback_write() splits the caller's buffer into chunks of this
 * size. */
#define AP_WRITE_CHUNK_MAX  4064

/* Single DMA transfer.  Must never be called with len > AP_WRITE_CHUNK_MAX;
 * use audio_playback_write() instead. */
static int ap_write_once(audio_playback_t *pb, const void *buf, size_t len)
{
  struct audio_buf_desc_s desc;
  FAR struct ap_buffer_s *apb = NULL;
  sem_t done;
  int ret;

  if (!pb)
    {
      return -EINVAL;
    }

  if (pb->stopped)
    {
      return -ECANCELED;
    }

  if (!g_i2s)
    {
      return -ENODEV;
    }

  if (len == 0)
    {
      return 0;
    }


  desc.numbytes  = len;
  desc.u.pbuffer = &apb;
  ret = apb_alloc(&desc);
  if (ret < 0 || apb == NULL)
    {
      syslog(LOG_ERR, "[%s] apb_alloc(%u) failed: %d\n",
             TAG, (unsigned)len, ret);
      return ret < 0 ? ret : -ENOMEM;
    }

  memcpy(apb->samp, buf, len);
  apb->nbytes    = len;
  apb->curbyte   = 0;
  apb->nsamples  = len / (ES_BITS / 8 * ES_CHANNELS);

  sem_init(&done, 0, 0);

  /* [V208] The driver treats timeout==0 as "no timeout", so a TX channel that
   * never drains turned every write into a permanent block -- the board simply
   * stopped producing log output and no sound.  Give the transfer a real
   * deadline and a backstop wait so a dead DMA channel degrades into a logged
   * error the agent can survive. */
  ret = I2S_SEND(g_i2s, apb, direct_tx_done, &done, MSEC2TICK(500));
  if (ret < 0)
    {
      syslog(LOG_ERR, "[%s] i2s_send failed: %d\n", TAG, ret);
      apb_free(apb);
      sem_destroy(&done);
      return ret;
    }

  {
    struct timespec ts;
    int wrc;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += 1;
    ts.tv_nsec += 500000000;          /* +1.5 s */
    if (ts.tv_nsec >= 1000000000)
      {
        ts.tv_sec  += 1;
        ts.tv_nsec -= 1000000000;
      }

    wrc = sem_timedwait(&done, &ts);
    if (wrc < 0)
      {
        if (g_tx_timeouts < 4)
          {
            g_tx_timeouts++;
            syslog(LOG_ERR,
                   "[%s] I2S TX completion TIMEOUT after 1.5 s (n=%d) -- the "
                   "TX DMA never drained; speaker cannot sound\n",
                   TAG, g_tx_timeouts);
          }

        apb_free(apb);
        sem_destroy(&done);
        return -ETIMEDOUT;
      }
  }

  apb_free(apb);
  sem_destroy(&done);

  return pb->stopped ? -ECANCELED : (int)len;
}

/* [V212] Public entry point: splits arbitrarily large PCM buffers into
 * transfers the internal-SRAM DMA slot can hold, and reports partial progress
 * if a later chunk fails. */
int audio_playback_write(audio_playback_t *pb, const void *buf, size_t len)
{
  const unsigned char *p = (const unsigned char *)buf;
  size_t done = 0;

  if (pb == NULL)
    {
      return -EINVAL;
    }

  if (len == 0)
    {
      return 0;
    }

  while (done < len)
    {
      size_t n = len - done;
      int ret;

      if (n > AP_WRITE_CHUNK_MAX)
        {
          n = AP_WRITE_CHUNK_MAX;
        }

      ret = ap_write_once(pb, p + done, n);
      if (ret < 0)
        {
          return done > 0 ? (int)done : ret;
        }

      done += n;
    }

  return (int)done;
}

/* [V244] Read back every register the official sequence touches, so the codec
 * state can be verified against what was written instead of assumed. */
static const uint8_t g_es_dump[] =
{
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x1B, 0x1C, 0x32, 0x37, 0x44, 0x45
};

void audio_direct_dump_codec(void)
{
  char line[160];
  int n = 0;
  unsigned int k;

  line[0] = '\0';
  for (k = 0; k < sizeof(g_es_dump) / sizeof(g_es_dump[0]); k++)
    {
      n += snprintf(line + n, sizeof(line) - n, "%02x=%02x ",
                    g_es_dump[k], es_readreg(g_es_dump[k]));
      if (n > 110)
        {
          syslog(LOG_INFO, "[%s] codec regs: %s\n", TAG, line);
          n = 0;
          line[0] = '\0';
        }
    }

  if (n > 0)
    {
      syslog(LOG_INFO, "[%s] codec regs: %s\n", TAG, line);
    }
}

/* [V234] Expose the ES8311 DAC volume register so the boot smoke test can
 * sweep it and settle the direction of the scale empirically.  On several
 * codec families this register is an attenuation (0x00 = 0 dB, 0xFF = quietest)
 * rather than a gain, which would make our 0xFF write the *quietest* setting. */
void audio_direct_set_dac_volume(unsigned int v)
{
  es_writereg(ES_REG_DAC_VOL, (uint8_t)v);
  syslog(LOG_INFO, "[%s] ES8311 reg 0x32 (DAC vol) = 0x%02x\n", TAG,
         (unsigned)(uint8_t)v);
}

void audio_playback_stop(audio_playback_t *pb)
{
  if (!pb)
    {
      return;
    }

  pb->stopped = true;
}

void audio_playback_close(audio_playback_t *pb)
{
  if (pb)
    {
      free(pb);
    }
}
