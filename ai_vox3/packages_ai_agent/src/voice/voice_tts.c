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

#include "voice/voice_tts.h"
#include "voice/volc_tts.h"
#include "voice/mimo_tts.h"
#include "http_tts.h"

#include <errno.h>
#include <string.h>
#include <syslog.h>

#define VOICE_TTS_MAX_BACKENDS 4

/* [V359] 公网 TTS 降级开关。
 * 0（默认）= 关闭：自建 HTTP TTS 失败后直接放弃，不再去连 MiMo / volc。
 *   它们走 vela_https_post_json()，connect() 没有超时旋钮，被 PBR 黑洞时
 *   会把整句对话挂住（正是 [V357] 看到的现象）。
 * 1 = 启用公网降级（仅在确认公网可出时开）。
 */

#ifndef VOICE_TTS_PUBLIC_FALLBACK
#  define VOICE_TTS_PUBLIC_FALLBACK 0
#endif

static const char* TAG = "voice_tts";

static const voice_tts_ops_t* s_backends[VOICE_TTS_MAX_BACKENDS];
static int s_backend_count;
static const voice_tts_ops_t* s_active;

int voice_tts_register(const voice_tts_ops_t* ops)
{
    if (!ops || !ops->name || !ops->synthesize) {
        return -EINVAL;
    }

    if (s_backend_count >= VOICE_TTS_MAX_BACKENDS) {
        syslog(LOG_ERR, "[%s] Too many TTS backends\n", TAG);
        return -ENOMEM;
    }

    s_backends[s_backend_count++] = ops;
    syslog(LOG_INFO, "[%s] Registered backend: %s\n", TAG, ops->name);

    /* First registered backend becomes the default */
    if (!s_active) {
        s_active = ops;
        if (ops->init) {
            ops->init();
        }
    }

    return 0;
}

int voice_tts_set_backend(const char* name)
{
    if (!name) {
        return -EINVAL;
    }

    for (int i = 0; i < s_backend_count; i++) {
        if (strcmp(s_backends[i]->name, name) == 0) {
            if (s_active && s_active->deinit) {
                s_active->deinit();
            }

            s_active = s_backends[i];
            if (s_active->init) {
                s_active->init();
            }

            syslog(LOG_INFO, "[%s] Backend set to: %s\n",
                TAG, name);
            return 0;
        }
    }

    syslog(LOG_ERR, "[%s] Backend not found: %s\n", TAG, name);
    return -ENOENT;
}

const char* voice_tts_get_backend(void)
{
    return s_active ? s_active->name : NULL;
}

int voice_tts_speak(const char* text,
    unsigned char* pcm_out,
    size_t pcm_cap,
    size_t* pcm_len)
{
    if (!s_active) {
        syslog(LOG_ERR, "[%s] No TTS backend registered\n", TAG);
        return -ENODEV;
    }

    return s_active->synthesize(text, pcm_out, pcm_cap, pcm_len);
}

/* ── Streaming TTS (delegates to volc_tts) ───────────────────── */

int voice_tts_speak_stream(const char* text,
    voice_tts_chunk_cb cb,
    void* user_data)
{
    if (!s_active) {
        syslog(LOG_WARNING,
            "[%s] No TTS backend selected, using HTTP/volc fallback\n", TAG);
    }

    /* Try the self-hosted HTTP TTS first.
     *
     * It is reached through an IP literal, so it needs neither DNS nor
     * internet egress -- the two things that proved unreliable on both APs.
     * Cloud MiMo and then volc remain as fallbacks for when the device
     * happens to sit on a healthy internet link. */
    int ret = -ENOSYS;

    /* [V359] 自建 HTTP TTS 先上：IP 直连，不需要 DNS / 公网出口，且自带
     * 有界 connect 与每 recv 超时（见 http_tts.c）。 */

    ret = http_tts_speak_stream(text, cb, user_data);
    if (ret != 0) {
        syslog(LOG_WARNING,
            "[V359] [%s] self-hosted TTS failed (%d), public fallback=%d\n",
            TAG, ret, VOICE_TTS_PUBLIC_FALLBACK);
    }

#if VOICE_TTS_PUBLIC_FALLBACK
    if (ret != 0) {
        if (s_active && s_active->name &&
            strcmp(s_active->name, MIMO_TTS_BACKEND_NAME) == 0) {
            ret = mimo_tts_speak_stream(text, cb, user_data);
        }
    }

    if (ret != 0) {
        ret = http_tts_speak_stream(text, cb, user_data);
    }
    if (ret != 0) {
        ret = volc_tts_ws_synthesize_stream(text,
            (volc_tts_chunk_cb)cb, user_data);
    }
#endif
    return ret;
}
