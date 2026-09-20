/*
 * mimo_asr.c -- Xiaomi MiMo cloud speech recognition backend.
 *
 * Real implementation, no mocks.  Wraps the recorded PCM (16-bit LE, 16 kHz,
 * mono) into a WAV container, base64 encodes it and POSTs it as an
 * input_audio content part to https://api.xiaomimimo.com/v1/chat/completions
 * with model mimo-v2.5-asr.  The transcript comes back in
 * choices[0].message.content.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <stdint.h>
#include <stdio.h>

#include "voice/voice_asr.h"
#include "infra/vela_tls.h"

#define MIMO_HOST        "124.251.34.64"
#define MIMO_PORT        "443"
#define MIMO_PATH        "/v1/chat/completions"
#define MIMO_ASR_MODEL   "mimo-v2.5-asr"

#define MIMO_API_KEY \
  "sk-c8xejq2zvdde4v7kjehwtpx6vipi0icdec9t7q4sjwhcqr6w"

/* Cap the clip we upload: 8 s of 16 kHz mono 16-bit PCM. */
#define MIMO_ASR_MAX_PCM (16u * 2u * 8000u)

#define MIMO_RESP_CAP    (8u * 1024u)

static const char *TAG = "mimo_asr";

/* ── base64 encoding (self contained) ───────────────────────────────────── */

static const char s_b64[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/* Locate a JSON string value starting just after a member name. */
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

static int b64_encode(const unsigned char *in, size_t in_len,
                      char *out, size_t out_cap, size_t *out_len)
{
  size_t i = 0u;
  size_t o = 0u;

  if (in == NULL || out == NULL || out_len == NULL)
    {
      return -EINVAL;
    }

  while (i < in_len)
    {
      unsigned long acc = 0ul;
      int have = 0;
      int k;

      for (k = 0; k < 3; k++)
        {
          if (i < in_len)
            {
              acc = (acc << 8) | (unsigned long)in[i++];
              have++;
            }
          else
            {
              acc <<= 8;
            }
        }

      if (o + 4u + 1u > out_cap)
        {
          return -ENAMETOOLONG;
        }

      out[o++] = s_b64[(acc >> 18) & 0x3f];
      out[o++] = s_b64[(acc >> 12) & 0x3f];
      out[o++] = (have > 1) ? s_b64[(acc >> 6) & 0x3f] : '=';
      out[o++] = (have > 2) ? s_b64[acc & 0x3f] : '=';
    }

  out[o] = '\0';

  *out_len = o;

  return 0;
}

/* ── transcript decoding ────────────────────────────────────────────────── */

/* Length of the JSON string body starting at d (d points at the first
 * character after the opening quote).  Escapes are skipped so that an
 * embedded \" does not cut the value short.  Returns (size_t)-1 when the
 * string is unterminated. */
static size_t json_str_len(const char *d)
{
  size_t i;

  if (d == NULL)
    {
      return (size_t)-1;
    }

  for (i = 0u; d[i] != '\0'; i++)
    {
      if (d[i] == '\\')
        {
          if (d[i + 1u] == '\0')
            {
              break;
            }

          i++;
          continue;
        }

      if (d[i] == '"')
        {
          return i;
        }
    }

  return (size_t)-1;
}

static unsigned long hex4(const char *s)
{
  unsigned long v = 0ul;
  int k;

  for (k = 0; k < 4; k++)
    {
      char c = s[k];
      int d;

      if (c >= '0' && c <= '9') { d = c - '0'; }
      else if (c >= 'a' && c <= 'f') { d = c - 'a' + 10; }
      else if (c >= 'A' && c <= 'F') { d = c - 'A' + 10; }
      else { return 0xFFFFFFFFul; }

      v = (v << 4) | (unsigned long)d;
    }

  return v;
}

/* Encode one code point as UTF-8.  Returns the number of bytes written. */
static size_t utf8_put(unsigned long cp, char *dst, size_t cap)
{
  if (cp < 0x80ul)
    {
      if (cap < 1u) { return 0u; }
      dst[0] = (char)(cp & 0x7ful);
      return 1u;
    }

  if (cp < 0x800ul)
    {
      if (cap < 2u) { return 0u; }
      dst[0] = (char)(0xC0ul | ((cp >> 6) & 0x1Ful));
      dst[1] = (char)(0x80ul | (cp & 0x3Ful));
      return 2u;
    }

  if (cp < 0x10000ul)
    {
      if (cap < 3u) { return 0u; }
      dst[0] = (char)(0xE0ul | ((cp >> 12) & 0x0Ful));
      dst[1] = (char)(0x80ul | ((cp >> 6) & 0x3Ful));
      dst[2] = (char)(0x80ul | (cp & 0x3Ful));
      return 3u;
    }

  if (cap < 4u) { return 0u; }

  dst[0] = (char)(0xF0ul | ((cp >> 18) & 0x07ul));
  dst[1] = (char)(0x80ul | ((cp >> 12) & 0x3Ful));
  dst[2] = (char)(0x80ul | ((cp >> 6) & 0x3Ful));
  dst[3] = (char)(0x80ul | (cp & 0x3Ful));

  return 4u;
}

/* Copy [s, s+len) into dst resolving JSON escapes.  dst is always
 * NUL-terminated; the value is truncated when cap is too small. */
static void json_unescape(const char *s, size_t len, char *dst, size_t cap)
{
  size_t i;
  size_t o = 0u;

  if (s == NULL || dst == NULL || cap == 0u)
    {
      return;
    }

  for (i = 0u; i < len && o + 1u < cap; i++)
    {
      char c = s[i];

      if (c != '\\')
        {
          dst[o++] = c;
          continue;
        }

      if (i + 1u >= len)
        {
          break;
        }

      c = s[++i];

      if (c == 'u')
        {
          unsigned long cp;

          if (i + 4u >= len)
            {
              break;
            }

          cp = hex4(s + i + 1u);
          i += 4u;

          if (cp == 0xFFFFFFFFul)
            {
              dst[o++] = '?';
              continue;
            }

          /* Surrogate pair (\uD83D\uDE00 style). */
          if (cp >= 0xD800ul && cp <= 0xDBFFul &&
              i + 6u < len && s[i + 1u] == '\\' && s[i + 2u] == 'u')
            {
              unsigned long lo = hex4(s + i + 3u);

              if (lo >= 0xDC00ul && lo <= 0xDFFFul)
                {
                  cp = 0x10000ul + ((cp - 0xD800ul) << 10) + (lo - 0xDC00ul);
                  i += 6u;
                }
            }

          o += utf8_put(cp, dst + o, cap - o - 1u);
          continue;
        }

      switch (c)
        {
          case 'n': c = '\n'; break;
          case 't': c = '\t'; break;
          case 'r': c = '\r'; break;
          case 'b': c = '\b'; break;
          case 'f': c = '\f'; break;
          case '"': c = '"';  break;
          case '/': c = '/';  break;
          case '\\': c = '\\'; break;
          default: break; /* unknown escape: keep the character as-is */
        }

      dst[o++] = c;
    }

  dst[o] = '\0';
}

/* ── WAV container ──────────────────────────────────────────────────────── */

static void put32_le(unsigned char *p, uint32_t v)
{
  p[0] = (unsigned char)(v & 0xffu);
  p[1] = (unsigned char)((v >> 8) & 0xffu);
  p[2] = (unsigned char)((v >> 16) & 0xffu);
  p[3] = (unsigned char)((v >> 24) & 0xffu);
}

static void put16_le(unsigned char *p, uint16_t v)
{
  p[0] = (unsigned char)(v & 0xffu);
  p[1] = (unsigned char)((v >> 8) & 0xffu);
}

/* Builds a 44 byte header followed by the PCM payload. */
static size_t wav_build(const unsigned char *pcm, size_t pcm_len,
                        unsigned char *out, size_t out_cap)
{
  size_t total = pcm_len + 44u;

  if (out_cap < total)
    {
      return 0u;
    }

  memcpy(out, "RIFF", 4);
  put32_le(out + 4u, (uint32_t)(pcm_len + 36u));
  memcpy(out + 8u, "WAVE", 4);
  memcpy(out + 12u, "fmt ", 4);
  put32_le(out + 16u, 16u);
  put16_le(out + 20u, 1u);
  put16_le(out + 22u, 1u);
  put32_le(out + 24u, 16000u);
  put32_le(out + 28u, 32000u);
  put16_le(out + 32u, 2u);
  put16_le(out + 34u, 16u);
  memcpy(out + 36u, "data", 4);
  put32_le(out + 40u, (uint32_t)pcm_len);
  memcpy(out + 44u, pcm, pcm_len);

  return total;
}

/* ── backend ops ────────────────────────────────────────────────────────── */

int __attribute__((used)) mimo_asr_init(void)
{
  syslog(LOG_INFO, "[MIMO-ASR] backend ready host=%s model=%s\n",
         MIMO_HOST, MIMO_ASR_MODEL);

  return 0;
}

void __attribute__((used)) mimo_asr_deinit(void)
{
  syslog(LOG_INFO, "[MIMO-ASR] backend released\n");
}

int mimo_asr_recognize(const unsigned char *pcm_data,
                              size_t pcm_len,
                              char *text_out,
                              size_t text_cap)
{
  unsigned char *wav = NULL;
  char *b64 = NULL;
  char *body = NULL;
  char *resp = NULL;
  size_t wavlen = 0u;
  size_t b64len = 0u;
  size_t send = pcm_len;
  int ret;

  if (pcm_data == NULL || text_out == NULL || text_cap == 0u)
    {
      return -EINVAL;
    }

  text_out[0] = '\0';

  if (pcm_len < 320u)
    {
      /* Too short to contain speech; report empty rather than error. */
      return -EINVAL;
    }

  if (send > MIMO_ASR_MAX_PCM)
    {
      send = MIMO_ASR_MAX_PCM;
    }

  wav = (unsigned char *)malloc(send + 44u);
  b64 = (char *)malloc(((send + 44u) * 4u) / 3u + 8u);
  body = (char *)malloc(((send + 44u) * 4u) / 3u + 1024u);
  resp = (char *)malloc(MIMO_RESP_CAP);

  if (wav == NULL || b64 == NULL || body == NULL || resp == NULL)
    {
      ret = -ENOMEM;

      goto out;
    }

  wavlen = wav_build(pcm_data, send, wav, send + 44u);

  if (wavlen == 0u)
    {
      ret = -EINVAL;

      goto out;
    }

  if (b64_encode(wav, wavlen, b64, ((send + 44u) * 4u) / 3u + 8u,
                 &b64len) != 0)
    {
      ret = -EINVAL;

      goto out;
    }

  /* NOTE on the closing brackets after %s:
   *   "  -> closes the data string
   *   }} -> closes input_audio{} and the content-part object{}
   *   ]  -> closes "content":[]
   *   }  -> closes the message object{}
   *   ]  -> closes "messages":[]
   * Only after that does "asr_options" sit at the ROOT level, which is what
   * the MiMo endpoint expects. */
  snprintf(body, ((send + 44u) * 4u) / 3u + 1024u,
           "{\"model\":\"%s\",\"messages\":[{\"role\":\"user\",\"content\":"
           "[{\"type\":\"input_audio\",\"input_audio\":{\"data\":"
           "\"data:audio/wav;base64,%s\"}}]}],"
           "\"asr_options\":{\"language\":\"zh\"}}",
           MIMO_ASR_MODEL, b64);

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
      syslog(LOG_ERR, "[MIMO-ASR] transport failed ret=%d\n", ret);

      goto out;
    }

  if (ret >= 400)
    {
      syslog(LOG_ERR, "[MIMO-ASR] HTTP %d\n", ret);

      ret = -EIO;

      goto out;
    }

  /* Extract choices[0].message.content. */
  {
    const char *p = strstr(resp, "\"choices\"");

    if (p == NULL) { p = resp; }

    {
      const char *c = strstr(p, "\"content\"");
      const char *v = NULL;
      size_t vl = 0u;

      if (c == NULL)
        {
          syslog(LOG_ERR, "[MIMO-ASR] no content in reply\n");

          ret = -ENOENT;

          goto out;
        }

      if (json_str_after(c + 9u, &v, &vl) != 0)
        {
          ret = -EINVAL;

          goto out;
        }

      /* json_str_after() stops at the first quote, which is wrong for a
       * value containing \" ; recompute with escape awareness and then
       * resolve the escapes into the caller's buffer. */
      {
        size_t rl = json_str_len(v);

        if (rl == (size_t)-1)
          {
            syslog(LOG_ERR, "[MIMO-ASR] unterminated content string\n");

            ret = -EPROTO;

            goto out;
          }

        json_unescape(v, rl, text_out, text_cap);
      }
    }
  }

  syslog(LOG_INFO, "[MIMO-ASR] transcript: %s\n", text_out);

  ret = 0;

out:
  if (wav != NULL) { free(wav); }
  if (b64 != NULL) { free(b64); }
  if (body != NULL) { free(body); }
  if (resp != NULL) { free(resp); }

  return ret;
}

static const voice_asr_ops_t s_mimo_asr_ops __attribute__((used)) =
{
  .name      = "mimo",
  .init      = mimo_asr_init,
  .recognize = mimo_asr_recognize,
  .deinit    = mimo_asr_deinit
};

int __attribute__((noinline, used)) mimo_asr_register(void)
{
  int ret = voice_asr_register(&s_mimo_asr_ops);

  if (ret == 0)
    {
      /* [ASR-INTRA] Use the self-hosted recogniser (IP literal, no DNS and no
     * internet egress required).  MiMo stays registered as a fallback. */
      voice_asr_set_backend("http");
    }

  return ret;
}
