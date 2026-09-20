/*
 * mimo_tts.c -- Xiaomi MiMo cloud text-to-speech backend.
 *
 * Real implementation, no mocks.  POSTs a chat-completions request to
 * https://api.xiaomimimo.com/v1/chat/completions with model mimo-v2.5-tts and
 * audio:{format:"wav"}.  The reply carries the synthesised clip as base64 WAV
 * in choices[0].message.audio.data.  We decode it, locate the WAV "data"
 * chunk, read the real sample rate (MiMo returns 24000 Hz) and resample to the
 * 16 kHz the board's I2S runs at, then hand the PCM back through the generic
 * voice_tts_ops_t synthesize() contract (16-bit LE, 16 kHz, mono).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <stdint.h>
#include <stdio.h>

#include "voice/voice_tts.h"
#include "voice/mimo_tts.h"
#include "infra/vela_tls.h"

#define MIMO_HOST        "124.251.34.64"
#define MIMO_PORT        "443"
#define MIMO_PATH        "/v1/chat/completions"
#define MIMO_TTS_MODEL   "mimo-v2.5-tts"

/* API key provisioned for this device. */
#define MIMO_API_KEY \
  "sk-c8xejq2zvdde4v7kjehwtpx6vipi0icdec9t7q4sjwhcqr6w"

#define MIMO_RESP_CAP    (256u * 1024u)
#define MIMO_TEXT_MAX    1024u
#define MIMO_PCM_RATE    16000u

static const char *TAG = "mimo_tts";

/* ── base64 decoding (self contained, no external dependency) ───────────── */

static int b64_val(int c)
{
  if (c >= 'A' && c <= 'Z') { return c - 'A'; }
  if (c >= 'a' && c <= 'z') { return c - 'a' + 26; }
  if (c >= '0' && c <= '9') { return c - '0' + 52; }
  if (c == '+') { return 62; }
  if (c == '/') { return 63; }
  return -1;
}

/* Returns 0 on success, negative errno on malformed input. */
static int b64_decode(const char *in, size_t in_len,
                      unsigned char *out, size_t *out_len)
{
  size_t i;
  size_t o = 0;
  int acc = 0;
  int bits = 0;

  if (in == NULL || out == NULL || out_len == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < in_len; i++)
    {
      int c = in[i];

      if (c == '=')
        {
          break;
        }

      if (c == '\r' || c == '\n' || c == ' ' || c == '\t')
        {
          continue;
        }

      {
        int v = b64_val(c);

        if (v < 0)
          {
            return -EINVAL;
          }

        acc = (acc << 6) | v;
        bits += 6;

        if (bits >= 8)
          {
            bits -= 8;

            out[o++] = (unsigned char)((acc >> bits) & 0xff);

            if (o + 1u > (size_t)0x7fffffffu)
              {
                return -EINVAL;
              }
          }
      }
    }

  *out_len = o;

  return 0;
}

/* ── JSON helpers ───────────────────────────────────────────────────────── */

/* Escape src into dst as a JSON string body.  dst_cap includes the NUL. */
static int json_escape(const char *src, char *dst, size_t dst_cap)
{
  size_t o = 0;
  size_t i;

  if (src == NULL || dst == NULL || dst_cap == 0u)
    {
      return -EINVAL;
    }

  for (i = 0; src[i] != '\0'; i++)
    {
      unsigned char c = (unsigned char)src[i];
      const char *repl = NULL;

      if (c == '"') { repl = "\\\""; }
      else if (c == '\\') { repl = "\\\\"; }
      else if (c == '\n') { repl = "\\n"; }
      else if (c == '\r') { repl = "\\r"; }
      else if (c == '\t') { repl = "\\t"; }
      else if (c < 0x20)
        {
          /* Control characters must be escaped; map them to a space. */
          repl = " ";
        }

      if (repl != NULL)
        {
          size_t rl = strlen(repl);

          if (o + rl + 1u > dst_cap)
            {
              return -ENAMETOOLONG;
            }

          memcpy(&dst[o], repl, rl);
          o += rl;
        }
      else
        {
          if (o + 2u > dst_cap)
            {
              return -ENAMETOOLONG;
            }

          dst[o++] = (char)c;
        }
    }

  dst[o] = '\0';

  return 0;
}

/* Locate the string value of a JSON member starting at p (p points just after
 * the member name).  Returns 0 and sets *vstart / *vlen on success. */
static int json_str_after(const char *p, const char **vstart, size_t *vlen)
{
  const char *d = p;
  const char *e;

  if (p == NULL || vstart == NULL || vlen == NULL)
    {
      return -EINVAL;
    }

  while (*d != '\0' && *d != ':') { d++; }

  if (*d != ':') { return -EINVAL; }

  d++;

  while (*d == ' ' || *d == '\t') { d++; }

  if (*d != '"') { return -EINVAL; }

  d++;
  e = strchr(d, '"');

  if (e == NULL) { return -EINVAL; }

  *vstart = d;
  *vlen = (size_t)(e - d);

  return 0;
}

/* ── WAV parsing ────────────────────────────────────────────────────────── */

static uint32_t rd32_le(const unsigned char *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t rd16_le(const unsigned char *p)
{
  return (uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
}

/* Walk the RIFF chunks and describe the PCM payload. */
static int wav_parse(const unsigned char *w, size_t len,
                     uint32_t *rate, uint16_t *bits, uint16_t *channels,
                     const unsigned char **data, size_t *dlen)
{
  size_t off = 12u;

  if (w == NULL || len < 44u || rate == NULL || bits == NULL ||
      channels == NULL || data == NULL || dlen == NULL)
    {
      return -EINVAL;
    }

  if (memcmp(w, "RIFF", 4) != 0 || memcmp(w + 8u, "WAVE", 4) != 0)
    {
      return -EINVAL;
    }

  while (off + 8u <= len)
    {
      uint32_t csz = rd32_le(w + off + 4u);
      size_t body = off + 8u;

      if (memcmp(w + off, "fmt ", 4) == 0)
        {
          if (body + 16u <= len)
            {
              *channels = rd16_le(w + body + 2u);
              *rate = rd32_le(w + body + 4u);
              *bits = rd16_le(w + body + 14u);
            }
        }
      else if (memcmp(w + off, "data", 4) == 0)
        {
          size_t avail = len - body;

          if (csz > avail) { csz = (uint32_t)avail; }

          *data = w + body;
            *dlen = (size_t)csz;

            if (*rate == 0u) { *rate = 24000u; }
            if (*bits == 0u) { *bits = 16u; }
            if (*channels == 0u) { *channels = 1u; }

            return 0;
        }

      if (csz == 0u) { break; }

      off = body + (size_t)csz;

      /* Chunk sizes are word aligned. */
      if ((csz & 1u) != 0u) { off++; }
    }

  return -ENOENT;
}

/* ── resampling 24000 -> 16000 (anti-aliased FIR + decimation) ──────────── */

/* n_in input samples at 24 kHz produce floor(n_in * 2 / 3) samples at 16 kHz.
 * Writes at most cap_out samples.  Returns the number of samples written.
 *
 * [RES-FIX] The input is low-passed in place with two cascaded [1 2 1]/4
 * passes before the decimation.  Plain decimation folds every component
 * between 8 kHz and 12 kHz of the 24 kHz source back into the 4..8 kHz band
 * of the 16 kHz output, which is the harsh / distorted colour heard on the
 * greeting.  Two [1 2 1] passes give roughly -24 dB at the 8 kHz band edge
 * and much more above, for two O(n) sweeps. */
static size_t resample_24k_to_16k(int16_t *in, size_t n_in,
                                  int16_t *out, size_t cap_out)
{
  size_t n_out = (n_in * 2u) / 3u;
  size_t i;
  int pass;

  if (in == NULL || out == NULL || n_in < 2u)
    {
      return 0u;
    }

  if (n_out > cap_out) { n_out = cap_out; }

  for (pass = 0; pass < 2; pass++)
    {
      int16_t prev = in[0];

      for (i = 1u; i + 1u < n_in; i++)
        {
          int16_t cur = in[i];

          in[i] = (int16_t)((prev + 2l * (long)cur + (long)in[i + 1u]) / 4l);
          prev = cur;
        }
    }

  for (i = 0u; i < n_out; i++)
    {
      /* Input position of output sample i. */
      unsigned long pos3 = 3ul * (unsigned long)i;
      size_t idx = (size_t)(pos3 / 2ul);
      int frac = (int)(pos3 % 2ul);

      if (idx + 1u >= n_in)
        {
          out[i] = in[idx < n_in ? idx : n_in - 1u];
          continue;
        }

      if (frac == 0)
        {
          out[i] = in[idx];
        }
      else
        {
          /* 0.5 -> average of neighbours. */
          long a = (long)in[idx];
          long b = (long)in[idx + 1u];
          out[i] = (int16_t)((a + b) / 2l);
        }
    }

  return n_out;
}

/* ── backend ops ───────────────────────────────────────────────────────── */

static __attribute__((used)) int mimo_tts_init(void)
{
  syslog(LOG_INFO, "[MIMO-TTS] backend ready host=%s model=%s\n",
         MIMO_HOST, MIMO_TTS_MODEL);

  return 0;
}

static __attribute__((used)) void mimo_tts_deinit(void)
{
  syslog(LOG_INFO, "[MIMO-TTS] backend released\n");
}

static __attribute__((used)) int mimo_tts_synthesize(const char *text,
                               unsigned char *pcm_out,
                               size_t pcm_cap,
                               size_t *pcm_len)
{
  char *body = NULL;
  char esc[MIMO_TEXT_MAX];
  char *resp = NULL;
  unsigned char *wav = NULL;
  const char *b64 = NULL;
  size_t b64len = 0u;
  const unsigned char *pcm24 = NULL;
  size_t pcm24_len = 0u;
  uint32_t rate = 0u;
  uint16_t bits = 0u;
  uint16_t ch = 0u;
  size_t wavlen = 0u;
  size_t frames;
  size_t wrote = 0u;
  int ret;

  if (text == NULL || pcm_out == NULL || pcm_len == NULL)
    {
      return -EINVAL;
    }

  *pcm_len = 0u;

  if (text[0] == '\0')
    {
      /* Nothing to say is not an error, but there is nothing to play. */
      return -EINVAL;
    }

  if (pcm_cap < 2u)
    {
      return -EINVAL;
    }

  if (json_escape(text, esc, sizeof(esc)) != 0)
    {
      syslog(LOG_ERR, "[MIMO-TTS] text too long or unescapable\n");

      return -ENAMETOOLONG;
    }

  body = (char *)malloc(MIMO_TEXT_MAX + 256u);
  resp = (char *)malloc(MIMO_RESP_CAP);

  if (body == NULL || resp == NULL)
    {
      ret = -ENOMEM;

      goto out;
    }

  snprintf(body, MIMO_TEXT_MAX + 256u,
           "{\"model\":\"%s\",\"messages\":[{\"role\":\"assistant\","
           "\"content\":\"%s\"}],\"audio\":{\"format\":\"wav\"}}",
           MIMO_TTS_MODEL, esc);

  /* Request. */
  {
    vela_header_t hdrs[2];

    hdrs[0].name = "Authorization";
    hdrs[0].value = "Bearer " MIMO_API_KEY;
    hdrs[1].name = NULL;
    hdrs[1].value = NULL;

    ret = vela_https_post_json(MIMO_HOST, MIMO_PORT, MIMO_PATH,
                               hdrs, body, resp, MIMO_RESP_CAP);
  }

  if (ret < 0)
    {
      syslog(LOG_ERR, "[MIMO-TTS] transport failed ret=%d\n", ret);

      goto out;
    }

  if (ret >= 400)
    {
      syslog(LOG_ERR, "[MIMO-TTS] HTTP %d\n", ret);

      ret = -EIO;

      goto out;
    }

  /* Locate choices[0].message.audio.data (base64 WAV). */
  {
    const char *p = strstr(resp, "\"audio\"");
    const char *d;

    if (p == NULL) { p = resp; }

    d = strstr(p, "\"data\"");

    if (d == NULL || json_str_after(d + 6u, &b64, &b64len) != 0)
      {
        syslog(LOG_ERR, "[MIMO-TTS] no audio.data in reply\n");
        ret = -ENOENT;

        goto out;
      }
  }

  wavlen = (b64len * 3u) / 4u + 8u;
  wav = (unsigned char *)malloc(wavlen);

  if (wav == NULL)
    {
      ret = -ENOMEM;

      goto out;
    }

  if (b64_decode(b64, b64len, wav, &wavlen) != 0)
    {
      syslog(LOG_ERR, "[MIMO-TTS] base64 decode failed\n");

      ret = -EINVAL;

      goto out;
    }

  if (wav_parse(wav, wavlen, &rate, &bits, &ch, &pcm24, &pcm24_len) != 0)
    {
      syslog(LOG_ERR, "[MIMO-TTS] bad WAV\n");

      ret = -EINVAL;

      goto out;
    }

  if (bits != 16u || ch != 1u)
    {
      syslog(LOG_ERR, "[MIMO-TTS] unsupported format bits=%u ch=%u\n",
             (unsigned)bits, (unsigned)ch);
      ret = -EINVAL;

      goto out;
    }

  frames = pcm24_len / 2u;

  if (rate == MIMO_PCM_RATE)
    {
      wrote = frames * 2u;

      if (wrote > pcm_cap) { wrote = pcm_cap - (pcm_cap % 2u); }

      memcpy(pcm_out, pcm24, wrote);
    }
  else
    {
      size_t n_out;
      size_t cap_frames = pcm_cap / 2u;
      size_t o_frames;

      wrote = 0u;

      n_out = (frames * (size_t)MIMO_PCM_RATE) / (size_t)rate;

      if (n_out > cap_frames) { n_out = cap_frames; }

      o_frames = resample_24k_to_16k((int16_t *)pcm24, frames,
                                     (int16_t *)pcm_out, n_out);

      wrote = o_frames * 2u;
    }

  *pcm_len = wrote;

  syslog(LOG_INFO,
         "[MIMO-TTS] synthesized %u bytes in_rate=%u out_rate=%u\n",
         (unsigned)wrote, (unsigned)rate, (unsigned)MIMO_PCM_RATE);

  ret = 0;

out:
  if (body != NULL) { free(body); }
  if (resp != NULL) { free(resp); }
  if (wav != NULL) { free(wav); }

  return ret;
}

/* ── streaming entry point ───────────────────────────────────────────────
 *
 * voice_channel speaks through the streaming API, so the one-shot
 * synthesize() above is not enough on its own: this wrapper renders the whole
 * clip into a heap buffer and pushes it to the callback in playback-sized
 * chunks, terminated by the is_last marker.
 */

#define MIMO_TTS_STREAM_PCM_CAP (192u * 1024u)  /* 6 s @ 16 kHz s16le mono */
#define MIMO_TTS_STREAM_CHUNK   2048u

int __attribute__((used)) mimo_tts_speak_stream(const char *text,
                          voice_tts_chunk_cb cb,
                          void *user_data)
{
  unsigned char *pcm = NULL;
  size_t pcm_len = 0u;
  size_t off = 0u;
  int ret;

  if (text == NULL || cb == NULL)
    {
      return -EINVAL;
    }

  pcm = (unsigned char *)malloc(MIMO_TTS_STREAM_PCM_CAP);

  if (pcm == NULL)
    {
      return -ENOMEM;
    }

  ret = mimo_tts_synthesize(text, pcm, MIMO_TTS_STREAM_PCM_CAP, &pcm_len);

  if (ret != 0)
    {
      free(pcm);
      return ret;
    }

  while (off < pcm_len)
    {
      size_t n = pcm_len - off;

      if (n > MIMO_TTS_STREAM_CHUNK) { n = MIMO_TTS_STREAM_CHUNK; }

      cb(pcm + off, n, 0, user_data);
      off += n;
    }

  cb(NULL, 0u, 1, user_data);

  syslog(LOG_INFO, "[MIMO-TTS] streamed %u bytes to playback\n",
         (unsigned)pcm_len);

  free(pcm);

  return 0;
}

static const voice_tts_ops_t s_mimo_tts_ops __attribute__((used)) =
{
  .name       = MIMO_TTS_BACKEND_NAME,
  .init       = mimo_tts_init,
  .synthesize = mimo_tts_synthesize,
  .deinit     = mimo_tts_deinit
};

int __attribute__((noinline, used)) mimo_tts_register(void)
{
  int ret = voice_tts_register(&s_mimo_tts_ops);

  if (ret == 0)
    {
      voice_tts_set_backend("mimo");
    }

  return ret;
}
