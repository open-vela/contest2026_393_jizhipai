#include <stdint.h>
#include <nuttx/arch.h>
// [V76] one-shot listen-path beacons
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

#include "voice/voice_channel.h"
#ifdef CONFIG_AI_AGENT_AUDIO_PREPROCESS
#include <compexp.h>
#include <echo_canceller.h>
#include <heap_api.h>
#endif
#include "core/message_bus.h"
#include "infra/config_store.h"
#include "agent_compat.h"
#include "agent_config.h"
#include "servo.h"
#include "voice/audio_capture.h"
#include "voice/audio_playback.h"
#include "voice/voice_asr.h"
#include "voice/voice_tts.h"
#include "voice/volc_asr.h"
#include "voice/volc_tts.h"
#include "voice/http_tts.h"
#include "voice/http_asr.h"
#include "voice/mimo_tts.h"
#include "voice/mimo_asr.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <unistd.h>

static const char* TAG = "voice";
static int s_backends_registered;

/* ── Audio preprocessing (NS + AGC via BES ec2float) ────── */

#ifdef CONFIG_AI_AGENT_AUDIO_PREPROCESS

#define PREPROC_HEAP_SIZE (150 * 1024)
#define PREPROC_FRAME_MS 15
#define PREPROC_FRAME_SIZE \
    (AGENT_VOICE_SAMPLE_RATE / 1000 * PREPROC_FRAME_MS)

static const Ec2FloatConfig s_ns_cfg = {
    .bypass = 0,
    .hpf_enabled = 1,
    .af_enabled = 0, /* no AEC */
    .adprop_enabled = 0,
    .varistep_enabled = 0,
    .nlp_enabled = 0, /* no NLP */
    .clip_enabled = 0,
    .stsupp_enabled = 0,
    .hfsupp_enabled = 0,
    .constrain_enabled = 0,
    .ns_enabled = 1, /* noise suppression ON */
    .cng_enabled = 0,
    .blocks = 1,
    .delay = 0,
    .gamma = 0.9f,
    .echo_band_start = 300,
    .echo_band_end = 1800,
    .min_ovrd = 2,
    .target_supp = -40,
    .highfre_band_start = 4000,
    .highfre_supp = 8.f,
    .noise_supp = -15,
    .cng_type = 0,
    .cng_level = -60,
    .clip_threshold = -20.f,
    .banks = 64,
};

static const CompexpConfig s_agc_cfg = {
    .bypass = 0,
    .type = 0,
    .comp_threshold = -20.f,
    .comp_ratio = 2.f,
    .expand_threshold = -45.f,
    .expand_ratio = 0.5556f,
    .attack_time = 0.001f,
    .release_time = 0.006f,
    .makeup_gain = 6,
    .delay = 32,
    .tav = 0.2f,
};

typedef struct {
    void* heap;
    Ec2FloatState* ns;
    CompexpState* agc;
} audio_preproc_t;

static audio_preproc_t* preproc_create(void)
{
    audio_preproc_t* pp = calloc(1, sizeof(*pp));
    if (!pp) {
        return NULL;
    }

    pp->heap = malloc(PREPROC_HEAP_SIZE);
    if (!pp->heap) {
        free(pp);
        return NULL;
    }
    med_heap_init(pp->heap, PREPROC_HEAP_SIZE);

    pp->ns = ec2float_create(AGENT_VOICE_SAMPLE_RATE,
        PREPROC_FRAME_SIZE, 0, &s_ns_cfg);
    pp->agc = compexp_create(AGENT_VOICE_SAMPLE_RATE,
        PREPROC_FRAME_SIZE, &s_agc_cfg);

    syslog(LOG_INFO, "[%s] preproc: NS=%p AGC=%p heap=%dKB\n",
        TAG, (void*)pp->ns, (void*)pp->agc,
        PREPROC_HEAP_SIZE / 1024);
    return pp;
}

static void preproc_destroy(audio_preproc_t* pp)
{
    if (!pp) {
        return;
    }
    if (pp->agc) {
        compexp_destroy(pp->agc);
    }
    if (pp->ns) {
        ec2float_destroy(pp->ns);
    }
    free(pp->heap);
    free(pp);
}

static void preproc_chunk(audio_preproc_t* pp,
    unsigned char* buf, int bytes)
{
    if (!pp || (!pp->ns && !pp->agc)) {
        return;
    }

    int16_t* samples = (int16_t*)buf;
    int total = bytes / 2;
    int fsz = PREPROC_FRAME_SIZE;
    int32_t in32[PREPROC_FRAME_SIZE];
    int32_t ref32[PREPROC_FRAME_SIZE];
    int32_t out32[PREPROC_FRAME_SIZE];

    memset(ref32, 0, sizeof(ref32));

    for (int off = 0; off < total; off += fsz) {
        int n = (off + fsz <= total) ? fsz : (total - off);

        for (int i = 0; i < n; i++) {
            in32[i] = samples[off + i];
        }

        if (pp->ns) {
            ec2float_process(pp->ns, in32, ref32, n, out32);
        } else {
            memcpy(out32, in32, n * sizeof(int32_t));
        }

        if (pp->agc) {
            for (int i = 0; i < n; i++) {
                out32[i] <<= 8;
            }
            compexp_process_int24(pp->agc, out32, n);
            for (int i = 0; i < n; i++) {
                out32[i] >>= 8;
            }
        }

        for (int i = 0; i < n; i++) {
            int32_t v = out32[i];
            if (v > 32767) {
                v = 32767;
            } else if (v < -32768) {
                v = -32768;
            }
            samples[off + i] = (int16_t)v;
        }
    }
}

#endif /* CONFIG_AI_AGENT_AUDIO_PREPROCESS */

/* ── Recording state machine (PTT mode) ─────────────────── */

enum voice_state {
    VOICE_IDLE = 0,
    VOICE_RECORDING,
    VOICE_STOPPING
};

/* ── Always-on listening: local endpointing ──────────────────────────────
 *
 * Nothing in this build decides when the user stopped talking: there is no
 * wake-word engine, and the HTTP ASR endpoint is plain request/response.  The
 * decision is therefore made locally from the captured level.
 *
 * Peak (16-bit samples) against an open and a close threshold, with hysteresis,
 * so one click cannot open a session and a natural pause inside a sentence
 * cannot close one.
 */

/* [V65] Thresholds lowered from 700 / 500.
 *
 * 700 of 32767 is about -33 dBFS, which a MEMS microphone behind this codec
 * did not reach: the log was a wall of "no speech for 8000 ms" and not one
 * byte of audio ever reached the ASR endpoint.  The codec input volume is
 * already at maximum (CONFIG_ES8311_INPUT_INITVOLUME = 1000), so the ceiling
 * was on this side. */

#define VOICE_VAD_OPEN_PEAK     150    /* level that counts as speech */
#define VOICE_VAD_CLOSE_PEAK    100    /* below this we count silence */
#define VOICE_VAD_SILENCE_MS    900    /* trailing silence that ends it */
#define VOICE_VAD_MIN_SPEECH_MS 250    /* ignore blips shorter than this */
#define VOICE_VAD_MAX_MS        15000  /* hard cap on one utterance */

/* [V65] Defined further down; used here to ship the level report. */

static void vc_beacon_cb(const unsigned char *d, size_t l, int f, void *u);

static volatile int s_vad_started;     /* speech seen in this session */
static volatile int s_vad_silence_ms;  /* trailing silence, milliseconds */
static volatile int s_vad_speech_ms;   /* speech so far, milliseconds */

static struct {
    enum voice_state state;
    pthread_mutex_t lock;
    audio_capture_t* cap;
    unsigned char* pcm_buf; /* accumulated PCM data */
    size_t pcm_len; /* bytes recorded so far */
    size_t pcm_cap; /* buffer capacity */
    pthread_t rec_thread;
    voice_asr_stream_t* asr_stream; /* streaming ASR handle */
    sem_t rec_ready; /* posted by recording_thread when ready to consume */
    volatile int tts_abort; /* set to 1 to stop active TTS playback */
    audio_playback_t* tts_pb; /* active TTS playback handle (or NULL) */
    pthread_mutex_t speak_lock; /* serialize concurrent speak calls */
    volatile int vad_ended; /* endpointing closed this utterance */
} s_voice = {
    .state = VOICE_IDLE,
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .speak_lock = PTHREAD_MUTEX_INITIALIZER,
};

/* ── File-based helpers (kept for QEMU test commands) ──── */

static int read_pcm_file(const char* path,
    unsigned char** out, size_t* out_len)
{
    int fd = open(path, O_RDONLY);

    if (fd < 0) {
        syslog(LOG_ERR, "[%s] Cannot open %s: %d\n",
            TAG, path, errno);
        return -errno;
    }

    off_t fsize = lseek(fd, 0, SEEK_END);

    if (fsize <= 0 || (size_t)fsize > AGENT_VOICE_PCM_BUF_SIZE) {
        syslog(LOG_ERR, "[%s] File too large or empty: %ld\n",
            TAG, (long)fsize);
        close(fd);
        return -EFBIG;
    }

    lseek(fd, 0, SEEK_SET);

    unsigned char* buf = malloc((size_t)fsize);

    if (!buf) {
        close(fd);
        return -ENOMEM;
    }

    ssize_t nread = read(fd, buf, (size_t)fsize);

    close(fd);

    if (nread != (ssize_t)fsize) {
        free(buf);
        return -EIO;
    }

    *out = buf;
    *out_len = (size_t)fsize;
    return 0;
}

static int write_pcm_file(const char* path,
    const unsigned char* data, size_t len)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

    if (fd < 0) {
        syslog(LOG_ERR, "[%s] Cannot create %s: %d\n",
            TAG, path, errno);
        return -errno;
    }

    ssize_t nw = write(fd, data, len);

    close(fd);
    return (nw == (ssize_t)len) ? 0 : -EIO;
}

/* ── Recording cleanup helper ────────────────────────────── */

static void recording_cleanup_locked(voice_asr_stream_t* stream)
{
    /* Abort streaming ASR if active */
    if (stream) {
        voice_asr_stream_abort(stream);
        s_voice.asr_stream = NULL;
    }

    /* Close capture device */
    if (s_voice.cap) {
        audio_capture_close(s_voice.cap);
        s_voice.cap = NULL;
    }

    /* Free PCM buffer */
    free(s_voice.pcm_buf);
    s_voice.pcm_buf = NULL;
    s_voice.pcm_len = 0;

    /* Reset state to IDLE */
    s_voice.state = VOICE_IDLE;
}

/* Allocate fallback PCM buffer (must hold lock) */
static int alloc_fallback_buffer_locked(void)
{
    if (s_voice.pcm_buf) {
        return 0;
    }

    s_voice.pcm_buf = malloc(s_voice.pcm_cap);
    if (!s_voice.pcm_buf) {
        syslog(LOG_ERR, "[%s] fallback buffer alloc failed\n", TAG);
        return -ENOMEM;
    }

    s_voice.pcm_len = 0;
    return 0;
}

/* Accumulate PCM chunk to fallback buffer (must hold lock) */
static void accumulate_pcm_locked(const unsigned char* chunk, size_t len)
{
    if (s_voice.pcm_buf && s_voice.pcm_len + len <= s_voice.pcm_cap) {
        memcpy(s_voice.pcm_buf + s_voice.pcm_len, chunk, len);
        s_voice.pcm_len += len;
    }
}

/* Process one audio chunk: send to ASR or accumulate for fallback */
static int process_audio_chunk(voice_asr_stream_t** stream_ptr,
    const unsigned char* chunk, size_t len, int* need_fallback,
    size_t* bytes_sent)
{
    if (*need_fallback) {
        pthread_mutex_lock(&s_voice.lock);
        accumulate_pcm_locked(chunk, len);
        pthread_mutex_unlock(&s_voice.lock);
    }

    if (*stream_ptr) {
        int ret = voice_asr_stream_send(*stream_ptr, chunk, len);

        if (ret != 0) {
            syslog(LOG_ERR,
                "[%s] stream send failed: %d, batch fallback\n", TAG, ret);
            voice_asr_stream_abort(*stream_ptr);
            *stream_ptr = NULL;
            *need_fallback = 1;

            pthread_mutex_lock(&s_voice.lock);
            alloc_fallback_buffer_locked();
            pthread_mutex_unlock(&s_voice.lock);
        } else {
            *bytes_sent += len;
        }
    }

    return 0;
}

/* ── Recording thread (PTT mode — streaming ASR) ─────────── */

static void vc_beacon(const char *tag);   /* [V67] defined below */

static void vc_probe(const char *tag, long a, long b);  /* [V69] */

/* [V98] 语音情绪钩子：写 /tmp/emotion.txt，faced 读取并显示表情徽章 */
static void emotion_set(const char *name);

static void* recording_thread(void* arg)
{
    (void)arg;
    unsigned char chunk[AGENT_ASR_CHUNK_SIZE];
    int need_fallback = 0;
    // int dump_fd = -1;
    size_t total_bytes_read = 0;
    size_t total_bytes_sent = 0;
    int chunk_count = 0;
    int abnormal_exit = 0;

    syslog(LOG_INFO, "[%s] recording thread started (streaming)\n", TAG);

    /* Use pre-connected ASR stream if available (opened in
     * voice_channel_start to avoid TLS handshake delay during
     * recording).  Fall back to opening a new session if not. */
    pthread_mutex_lock(&s_voice.lock);
    voice_asr_stream_t* stream = s_voice.asr_stream;
    s_voice.asr_stream = NULL; /* thread now owns it */
    pthread_mutex_unlock(&s_voice.lock);

    if (!stream) {
        /* No pre-connected stream — open now (may lose first 300-800ms) */
        stream = voice_asr_stream_open();
        syslog(LOG_WARNING,
            "[%s] streaming ASR unavailable, batch fallback\n", TAG);
        need_fallback = 1;

        pthread_mutex_lock(&s_voice.lock);
        if (alloc_fallback_buffer_locked() != 0) {
            s_voice.state = VOICE_IDLE;
            pthread_mutex_unlock(&s_voice.lock);
            sem_post(&s_voice.rec_ready);
            return NULL;
        }
        pthread_mutex_unlock(&s_voice.lock);
    } else {
        syslog(LOG_INFO, "[%s] using pre-connected ASR stream\n", TAG);
    }

    /* Signal voice_channel_start() that we are ready to consume
     * audio frames. Without this, sem_wait in start() deadlocks. */
    sem_post(&s_voice.rec_ready);

    /* No flush needed: the new start sequence guarantees that
     * audio_capture_start() runs AFTER this thread is ready,
     * so there is no stale data in the queue. The old flush
     * was harmful — audio_capture_read() blocks, so it would
     * consume real audio frames instead of stale ones. */

    while (1) {
        pthread_mutex_lock(&s_voice.lock);
        enum voice_state st = s_voice.state;
        pthread_mutex_unlock(&s_voice.lock);

        if (st != VOICE_RECORDING) {
            break;
        }

        int n = audio_capture_read(s_voice.cap,
            chunk, sizeof(chunk));

        /* [V67] One-shot capture probes.  "B7_REC_FIRST_OK" proves audio is
         * being delivered at all; "B7_REC_ERR<n>" reports the first hard
         * error.  Both are visible in the TTS bridge log. */

        {
            static int s_v67_first;
            static int s_v67_err;

            if (n > 0 && !s_v67_first) {
                s_v67_first = 1;
                syslog(LOG_INFO,
                    "[%s] [V67] first capture chunk: %d bytes\n", TAG, n);
                vc_beacon("B7_REC_FIRST_OK");
            } else if (n < 0 && n != -EAGAIN && !s_v67_err) {
                char tb[32];

                s_v67_err = 1;
                snprintf(tb, sizeof(tb), "B7_REC_ERR%d", -n);
                vc_beacon(tb);
            }
        }

        if (n == -EAGAIN || n == -EWOULDBLOCK) {
            /* [V82] Count the empty polls.  "The microphone is silent" has to
             * be a measured fact, not an inference from a probe that never
             * fired - otherwise a missing B7_REC_FIRST_OK is unfalsifiable. */

            {
                static int s_sil;

                if ((++s_sil % 100) == 0) {
                    vc_probe("B7_RECZ", (long)total_bytes_read, (long)s_sil);
                }
            }

            /* Non-blocking capture socket has no data yet (common right
             * after start before the first frames arrive). Retry instead
             * of treating it as a fatal error. */
            usleep(10000);
            continue;
        }

        if (n <= 0) {
            if (s_voice.vad_ended) {
                /* The endpointing closed the stream on purpose.  Taking the
                 * abnormal path here would call recording_cleanup_locked(),
                 * which aborts the ASR stream - and with it the transcript we
                 * are waiting for. */

                syslog(LOG_INFO,
                    "[%s] capture closed by endpointing\n", TAG);
            } else {
                /* [V117] Retry on transient I2S bus errors instead of
                 * killing the thread permanently.  ES8311 returns -EIO
                 * when the bus is momentarily busy after a failed TTS
                 * playback attempt.  Up to 5 retries with 200ms gap. */

                static int s_eio_cnt;
                if (n == -EIO && s_eio_cnt < 5) {
                    s_eio_cnt++;
                    syslog(LOG_WARNING,
                        "[%s] capture EIO, retry %d/5\n", TAG, s_eio_cnt);
                    usleep(200000);
                    continue;   /* back to while(1) read loop */
                }
                s_eio_cnt = 0;
                syslog(LOG_WARNING,
                    "[%s] capture read returned %d, stopping\n", TAG, n);
                abnormal_exit = 1;
            }

            break;
        }

        total_bytes_read += (size_t)n;
        chunk_count++;

        /* Dump raw PCM to file */
        // if (dump_fd >= 0) {
        //     write(dump_fd, chunk, (size_t)n);
        // }

        /* Level of this chunk.  The peak used to be computed only for the
         * periodic log line; the endpointing below needs it for every chunk,
         * so it is measured unconditionally now. */

        int16_t* samples = (int16_t*)chunk;
        int sample_count = (int)((size_t)n / sizeof(int16_t));
        int32_t peak = 0;

        for (int i = 0; i < sample_count; i++) {
            int32_t v = samples[i] < 0 ? -samples[i] : samples[i];
            if (v > peak) {
                peak = v;
            }
        }

        if (chunk_count <= 3 || chunk_count % 50 == 0) {
            syslog(LOG_INFO,
                "[%s] chunk#%d: %d bytes, peak=%ld\n",
                TAG, chunk_count, n, (long)peak);
        }

#ifdef CONFIG_AI_AGENT_AUDIO_PREPROCESS
        preproc_chunk(pp, chunk, n);
#endif

        /* ── Endpointing ────────────────────────────────────────────────
         * Measured after preprocessing so the AGC gain counts towards the
         * decision (a quiet talker is boosted before it is judged). */

        {
            int16_t* vs = (int16_t*)chunk;
            int vcount = (int)((size_t)n / sizeof(int16_t));
            int32_t vpeak = 0;
            int chunk_ms = (vcount * 1000) / AGENT_VOICE_SAMPLE_RATE;

            if (chunk_ms <= 0) {
                chunk_ms = 1;
            }

            for (int i = 0; i < vcount; i++) {
                int32_t v = vs[i] < 0 ? -vs[i] : vs[i];
                if (v > vpeak) {
                    vpeak = v;
                }
            }

            /* [V64] Level forensics.
             *
             * The VAD opens at VOICE_VAD_OPEN_PEAK, and the log repeats
             * "no speech for 8000 ms" -- which is either a silent capture
             * path (I2S/ES8311 gain) or a threshold above whatever the
             * microphone really delivers.  Report the loudest chunk seen
             * each second so the two can be told apart. */

            {
                static int32_t s_v64_peak;
                static time_t s_v64_ts;
                time_t v64_now = time(NULL);

                if (vpeak > s_v64_peak) {
                    s_v64_peak = vpeak;
                }

                if (v64_now != s_v64_ts) {
                    s_v64_ts = v64_now;
                    syslog(LOG_ERR, "[V64] mic peak=%ld open=%d started=%d\n",
                           (long)s_v64_peak, VOICE_VAD_OPEN_PEAK,
                           s_vad_started);
                    s_v64_peak = 0;
                }
            }

            /* [V65] Ship the loudest chunk of the last 5 s to the server.
             *
             * The serial console is unreliable on this board and the kmsg
             * shipper loses /dev/kmsg to netinit's drain thread, so the TTS
             * request line -- the one path that demonstrably reaches the
             * server -- carries the measurement instead.  The bridge logs
             * every request, so "text=MICPK1234Q0" gives the peak and whether
             * the VAD ever opened. */

            {
                static time_t s_v65_last;
                static int32_t s_v65_peak;
                time_t v65_now = time(NULL);

                if (vpeak > s_v65_peak) {
                    s_v65_peak = vpeak;
                }

                if (s_v65_last == 0) {
                    s_v65_last = v65_now;
                } else if (v65_now - s_v65_last >= 5) {
                    char v65_tb[40];

                    snprintf(v65_tb, sizeof(v65_tb), "MICPK%ldQ%d",
                             (long)s_v65_peak, s_vad_started);
                    syslog(LOG_ERR, "[V65] mic report: %s\n", v65_tb);
                    http_tts_speak_stream(v65_tb, vc_beacon_cb, NULL);
                    s_v65_peak = 0;
                    s_v65_last = v65_now;
                }
            }

            if (vpeak >= VOICE_VAD_OPEN_PEAK) {
                if (!s_vad_started) {
                    s_vad_started = 1;
                    syslog(LOG_INFO,
                        "[%s] voice: speech detected (peak=%ld)\n",
                        TAG, (long)vpeak);
                }

                s_vad_speech_ms += chunk_ms;
                s_vad_silence_ms = 0;
            } else if (s_vad_started && vpeak < VOICE_VAD_CLOSE_PEAK) {
                s_vad_silence_ms += chunk_ms;
            }

            if (s_vad_started && s_vad_speech_ms >= VOICE_VAD_MIN_SPEECH_MS &&
                (s_vad_silence_ms >= VOICE_VAD_SILENCE_MS ||
                 s_vad_speech_ms + s_vad_silence_ms >= VOICE_VAD_MAX_MS)) {
                syslog(LOG_INFO,
                    "[%s] voice: end of utterance "
                    "(%d ms speech, %d ms silence)\n",
                    TAG, s_vad_speech_ms, s_vad_silence_ms);

                pthread_mutex_lock(&s_voice.lock);
                s_voice.vad_ended = 1;
                pthread_mutex_unlock(&s_voice.lock);

                /* Unblock this thread's read so the loop can wind down in an
                 * orderly way.  voice_channel_stop() then finalises the ASR
                 * and hands the transcript to the agent. */

                audio_capture_abort(s_voice.cap);
                break;
            }
        }

        process_audio_chunk(&stream, chunk, (size_t)n,
            &need_fallback, &total_bytes_sent);
    }

#ifdef CONFIG_AI_AGENT_AUDIO_PREPROCESS
    preproc_destroy(pp);
#endif

    /* Store stream handle for voice_channel_stop to finish */
    // if (dump_fd >= 0) {
    //     close(dump_fd);
    //     syslog(LOG_INFO, "[%s] PCM dump saved to /data/cap_dump.pcm\n", TAG);
    // }

    pthread_mutex_lock(&s_voice.lock);
    s_voice.asr_stream = stream;

    if (abnormal_exit && s_voice.state == VOICE_RECORDING) {
        syslog(LOG_WARNING, "[%s] abnormal exit, cleaning up\n", TAG);
        recording_cleanup_locked(stream);
    }
    pthread_mutex_unlock(&s_voice.lock);

    syslog(LOG_INFO,
        "[%s] recording thread exit: %d chunks, %zu read, %zu sent%s\n",
        TAG, chunk_count, total_bytes_read, total_bytes_sent,
        abnormal_exit ? " (abnormal)" : "");
    return NULL;
}


/* ── Wake-word gate ─────────────────────────────────────────────────────
 * This build has no on-device WakeNet, so "wake-up" is keyword matching on
 * the ASR transcript.  Once woken, the conversation stays open for
 * AGENT_VOICE_WAKE_ACTIVE_SEC so follow-up sentences need no wake word.
 * Side effect: it also drops ASR hallucinations caused by ambient noise.
 */
#ifndef AGENT_VOICE_WAKE_ENABLE
#define AGENT_VOICE_WAKE_ENABLE 1
#endif

#ifndef AGENT_VOICE_WAKE_ACTIVE_SEC
#define AGENT_VOICE_WAKE_ACTIVE_SEC 30
#endif

#ifndef AGENT_VOICE_WAKE_WORDS
/* 唤醒词：小沃优先（机器人人格名），保留小智作 ASR 容错（转写歧义）。 */
#define AGENT_VOICE_WAKE_WORDS "\u4f60\u597d\u5c0f\u6c83,\u5c0f\u6c83,\u4f60\u597d\u5c0f\u667a,\u5c0f\u667a"
#endif

static time_t s_wake_active_ts;

/* Returns 1 when the utterance should be dispatched to the agent. */
static int voice_wake_accepted(const char *text)
{
#if !AGENT_VOICE_WAKE_ENABLE
    (void)text;
    return 1;
#else
    char list[256];

    if (text == NULL || text[0] == '\0') {
        return 0;
    }

    /* Conversation already open -> accept without repeating the wake word. */
    if (s_wake_active_ts != 0 &&
        (time(NULL) - s_wake_active_ts) < AGENT_VOICE_WAKE_ACTIVE_SEC) {
        s_wake_active_ts = time(NULL);
        return 1;
    }

    strncpy(list, AGENT_VOICE_WAKE_WORDS, sizeof(list) - 1);
    list[sizeof(list) - 1] = '\0';

    for (char *w = strtok(list, ","); w != NULL; w = strtok(NULL, ",")) {
        if (*w != '\0' && strstr(text, w) != NULL) {
            s_wake_active_ts = time(NULL);
            syslog(LOG_INFO, "[%s] wake word matched: %s\n", TAG, w);
            diag_report("WAKE_HIT");
            return 1;
        }
    }

    return 0;
#endif
}

/* \u2500\u2500 Button (BOOT / GPIO0) wake \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
 * The AI-VOX3 board exposes a single tactile BOOT button wired to
 * GPIO0 (BUTTON_BOOT), configured INPUT+PULLUP by the board at boot.
 * A press reads as active-LOW; board_buttons() (standard NuttX board
 * API) returns bit 0x1 while held.  We poll it and, on a fresh press,
 * inject the *same* wake event the ASR path produces:
 *   - open the 30s wake window (s_wake_active_ts)
 *   - push an inbound voice message so the agent greets / opens the
 *     conversation.  Voice (ASR) wake below is kept fully intact.
 */
extern int esp32s3_gpioread(int pin);  /* arch GPIO read, defined in libarch.a */
extern int esp32s3_configgpio(uint32_t pin, uint16_t attr);  /* arch GPIO config */

static void button_wake_trigger(void)
{
    syslog(LOG_INFO, "[%s] BUTTON wake triggered\n", TAG);
    diag_report("BTN_WAKE");
    /* [V314] 用户要求 BOOT 只作唤醒、不要带动舵机。
     * 原来这里每次唤醒都 servo_shake() 摇头，看起来像"BOOT 绑定了舵机"。 */

    /* open the conversation window so follow-up speech bypasses the gate */
    s_wake_active_ts = time(NULL);

    agent_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    strncpy(msg.channel, AGENT_CHAN_VOICE, sizeof(msg.channel) - 1);
    strncpy(msg.chat_id, "voice", sizeof(msg.chat_id) - 1);
    msg.content = strdup("\u4f60\u597d\u5c0f\u6c83");
    if (msg.content != NULL) {
        message_bus_push_inbound(&msg);
    }
}

/* [V315] BOOT as a true push-to-talk key with on-screen status text.
 *   press   -> on-screen listen prompt + voice_channel_start()
 *   release -> on-screen thinking prompt + voice_channel_stop_with_text()
 * voice_channel_start fails -> fall back to the old wake injection so the
 * button always does something. */
/* [V315] 屏幕状态由 emotion_set() -> /tmp/emotion.txt -> info_screen 显示，
 * 不再需要 LVGL chat UI（本固件未启用 CONFIG_AI_AGENT_LVGL_UI）。 */

static bool s_ptt_recording = false;

/* [V369] The release edge is NOT reliable on this board: GPIO0's pad can
 * be re-routed by another driver, which freezes GPIO_IN_REG at whatever
 * level it held at the time -- so after a press it can read LOW forever
 * and button_ptt_release() is never called.  The session then hangs in
 * "listening" and no ASR ever runs.  Cap the session so it always ends. */
#define VOICE_PTT_MAX_SEC  8
/* [V370] minimum recording length.  One physical press produced THREE
 * press/release pairs (V369 log) because the pad level glitches; the
 * release fired ~0.3s in and killed the capture thread with
 * "0 chunks, 0 read".  A real utterance needs a couple of seconds. */
#define VOICE_PTT_MIN_SEC  3
static time_t s_ptt_start = 0;
static int    s_btn_level  = 0;   /* debounced level  */
static int    s_btn_stable = 0;   /* polls it has held */

static void button_ptt_press(void)
{
    syslog(LOG_INFO, "[%s] BUTTON PTT press -> listen\n", TAG);
    diag_report("BTN_PTT_PRESS");

    /* keep the wake-window semantics so follow-up speech bypasses the gate */
    s_wake_active_ts = time(NULL);

    emotion_set("listening");   /* 屏显 开始聆听... */

    int rc = voice_channel_start();
    if (rc != 0) {
        syslog(LOG_ERR,
               "[%s] PTT voice_channel_start failed: %d -- fallback to wake\n",
               TAG, rc);
        emotion_set("thinking");
        button_wake_trigger();
        return;
    }
    s_ptt_recording = true;
    s_ptt_start = time(NULL);
}

static void button_ptt_release(void)
{
    syslog(LOG_INFO, "[%s] BUTTON PTT release -> thinking\n", TAG);
    diag_report("BTN_PTT_RELEASE");

    if (!s_ptt_recording) {
        return;   /* fallback path already handled the press */
    }
    s_ptt_recording = false;

    emotion_set("thinking");   /* 屏显 正在思考... */

    char text[256];
    int rc = voice_channel_stop_with_text(text, sizeof(text));
    if (rc != 0) {
        syslog(LOG_ERR, "[%s] PTT stop_with_text failed: %d\n", TAG, rc);
        emotion_set("idle");
        return;
    }
    if (text[0] == '\0') {
        syslog(LOG_INFO, "[%s] PTT: no speech recognized\n", TAG);
        emotion_set("idle");
        return;
    }

    syslog(LOG_INFO, "[%s] PTT asr: \"%s\"\n", TAG, text);

    agent_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    strncpy(msg.channel, AGENT_CHAN_VOICE, sizeof(msg.channel) - 1);
    strncpy(msg.chat_id, "voice", sizeof(msg.chat_id) - 1);
    msg.content = strdup(text);
    if (msg.content != NULL) {
        message_bus_push_inbound(&msg);
    }
}

/* [V367] how many times GPIO0 has actually been seen LOW since boot */
static int s_boot_fall = 0;

static void* button_wake_task(void* arg)
{
    (void)arg;
    /* 0x8D decoded = INPUT|PULLUP|FUNCTION_2 -> the pin was routed to IO-MUX
     * function 2, so GPIO_IN_REG did NOT track the pad and the BOOT
     * press was never seen.  0x09 = INPUT|PULLUP (plain GPIO0). */
    esp32s3_configgpio(0, 0x09);  /* INPUT | PULLUP, IO-MUX function 0 */
    int pressed_prev = 0;

    syslog(LOG_INFO, "[%s] button wake task started (BOOT=GPIO0)\n", TAG);
    int bw_tick = 0;

    for (;;) {
        int raw = esp32s3_gpioread(0) ? 1 : 0;   /* BOOT=GPIO0, active LOW */
        int pressed = (raw == 0) ? 1 : 0;

        /* [V371] Asymmetric debounce.
         * V370 debounced BOTH edges and ended up detecting nothing at all
         * (BTN_PRESS=0 over a whole run): this pad glitches faster than
         * 60ms, so "level stable for 3 polls" is never satisfied.
         * Press therefore acts on the very first LOW sample -- it is the
         * edge we care about -- while the release must hold HIGH for 60ms.
         * A press while a session is already open is ignored, so the
         * glitching cannot stack up recordings. */
        int committed;
        if (pressed) {
            committed    = 1;
            s_btn_level  = 1;
            s_btn_stable = 0;
        } else {
            if (s_btn_level == 0) {
                if (s_btn_stable < 255) { ++s_btn_stable; }
            } else {
                s_btn_level  = 0;
                s_btn_stable = 0;
            }
            committed = (s_btn_stable >= 3) ? 0 : pressed_prev;
        }

        if (committed && !pressed_prev) {
            if (s_ptt_recording) {
                syslog(LOG_INFO, "[%s] BTN_PRESS ignored: already recording\n", TAG);
            } else {
                ++s_boot_fall;
                syslog(LOG_INFO, "[%s] BTN_PRESS raw=%d -> PTT\n", TAG, raw);
                button_ptt_press();
            }
        } else if (!committed && pressed_prev) {
            long el = s_ptt_start ? (long)(time(NULL) - s_ptt_start) : 0;
            if (s_ptt_recording && el < VOICE_PTT_MIN_SEC) {
                /* too short to hold any speech -- keep recording */
                syslog(LOG_INFO,
                       "[%s] release ignored: %lds recorded (min %ds)\n",
                       TAG, el, VOICE_PTT_MIN_SEC);
            } else {
                button_ptt_release();
            }
        }
        if (committed != pressed_prev) {
            syslog(LOG_INFO, "[%s] BTN_LEVEL committed=%d stable=%d\n",
                   TAG, committed, s_btn_stable);
        }
        /* [V369] never let a PTT session hang open waiting for an edge
         * that may never arrive. */
        if (s_ptt_recording && s_ptt_start != 0 &&
            (time(NULL) - s_ptt_start) >= VOICE_PTT_MAX_SEC) {
            syslog(LOG_INFO,
                   "[%s] PTT auto-stop after %lds (release edge never seen)\n",
                   TAG, (long)(time(NULL) - s_ptt_start));
            button_ptt_release();
        }
        pressed_prev = pressed;
        usleep(20000);
        ++bw_tick;
        /* [V369] re-arm the pad every 100ms: re-applying INPUT|PULLUP is
         * what made presses visible at all (V368), and doing it 10x more
         * often also unfreezes the level in time for the release. */
        if ((bw_tick % 5) == 0) {
            esp32s3_configgpio(0, 0x09);  /* INPUT | PULLUP */
        }
        /* [V368] 1s heartbeat.
         * low0/low1 = bitmap of every GPIO that currently reads LOW (bit i set
         * => GPIO i is LOW).  Whatever key the user actually presses shows up
         * here, so we learn the real pin instead of guessing.  We also
         * re-apply INPUT|PULLUP every second: if some driver clobbers GPIO0's
         * IO_MUX after this task starts, the pad stops tracking the button and
         * re-arming it self-heals that. */
        if ((bw_tick % 50) == 0) {
            uint32_t low0 = 0;
            uint32_t low1 = 0;
            for (int p = 0; p <= 48; p++) {
                if (!esp32s3_gpioread(p)) {
                    if (p < 32) {
                        low0 |= ((uint32_t)1u << p);
                    } else {
                        low1 |= ((uint32_t)1u << (p - 32));
                    }
                }
            }
            esp32s3_configgpio(0, 0x09);  /* INPUT | PULLUP -- self-heal */
            syslog(LOG_INFO, "[%s] BTN_TICK g0=%d fall=%d low0=%08lx low1=%08lx\n",
                   TAG, esp32s3_gpioread(0) ? 1 : 0, s_boot_fall,
                   (unsigned long)low0, (unsigned long)low1);
        }
        /* periodic liveness beacon (~30s) -- fires once network is up */
        if (bw_tick >= 1500) { bw_tick = 0; diag_report("BTNWK_TASK"); }
        /* periodic self-test (~10s) -- explicitly tagged so it is NOT confused
         * with a real key press */
        if ((bw_tick % 500) == 0) {
            syslog(LOG_INFO, "[%s] BTN_SELFTEST (periodic tick, no wake now)\n", TAG);
            /* [V206] wake disabled here: it shook the servo every 10s */
        }
    }
    return NULL;
}

/* ── ASR worker thread ──────────────────────────────────── */

#define ASR_THREAD_STACK (24 * 1024)

static void* asr_and_dispatch(void* arg)
{
    unsigned char* pcm = (unsigned char*)arg;
    size_t pcm_len;

    /* pcm_len is stored in the first sizeof(size_t) bytes */
    memcpy(&pcm_len, pcm, sizeof(size_t));
    const unsigned char* audio = pcm + sizeof(size_t);

    syslog(LOG_INFO, "[%s] ASR: recognizing %zu bytes\n",
        TAG, pcm_len);

    char text[512];
    int ret = voice_asr_recognize(audio, pcm_len,
        text, sizeof(text));

    if (ret == 0 && text[0] != '\0') {
        syslog(LOG_INFO, "[%s] ASR result: %s\n", TAG, text);
        diag_report("ASR_OK");
        printf("ASR: %s\n", text);

        /* Wake-word gate: drop anything that is not addressed to us. */
        if (!voice_wake_accepted(text)) {
            syslog(LOG_INFO, "[%s] wake word not found, dropped\n", TAG);
            diag_report("WAKE_MISS");
            printf("Wake: dropped\n");
            free(pcm);
            return NULL;
        }

        agent_msg_t msg;

        memset(&msg, 0, sizeof(msg));
        strncpy(msg.channel, AGENT_CHAN_VOICE,
            sizeof(msg.channel) - 1);
        strncpy(msg.chat_id, "voice",
            sizeof(msg.chat_id) - 1);
        msg.content = strdup(text);
        if (msg.content) {
            message_bus_push_inbound(&msg);
        }
    } else {
        syslog(LOG_WARNING, "[%s] ASR failed or empty: %d\n",
            TAG, ret);
    }

    free(pcm);
    return NULL;
}

/* ── TTS streaming callback ──────────────────────────────── */

static struct timespec s_tts_start;
static int s_tts_first_chunk;
static struct timespec s_asr_done_ts;

static void tts_stream_cb(const unsigned char* pcm_data,
    size_t pcm_len, int is_last, void* user_data)
{
    audio_playback_t* pb = (audio_playback_t*)user_data;

    /* Abort early if PTT recording started */
    if (s_voice.tts_abort) {
        return;
    }

    if (pcm_data && pcm_len > 0 && pb) {
        if (!s_tts_first_chunk) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long ms = (now.tv_sec - s_tts_start.tv_sec) * 1000
                + (now.tv_nsec - s_tts_start.tv_nsec) / 1000000;
            syslog(LOG_INFO, "[%s] TTS first chunk: %ldms\n",
                TAG, ms);
            s_tts_first_chunk = 1;
        }
        audio_playback_write(pb, pcm_data, pcm_len);
    }

    (void)is_last;
}

/* ── Public API ──────────────────────────────────────────── */

int voice_channel_init(void)
{
    diag_report("VC_INIT");
    syslog(LOG_INFO, "[%s] Voice channel initializing\n", TAG);

    /* Reset state on init — static vars may persist across process restarts
     * in NuttX depending on how the binary is loaded. */
    s_voice.state = VOICE_IDLE;
    s_voice.cap = NULL;
    s_voice.pcm_buf = NULL;
    s_voice.pcm_len = 0;
    s_voice.asr_stream = NULL;
    s_voice.tts_abort = 0;
    s_voice.tts_pb = NULL;
    sem_init(&s_voice.rec_ready, 0, 0);

    if (!s_backends_registered) {
        volc_tts_register();
        http_asr_register();
        volc_asr_register();

        /* Xiaomi MiMo cloud speech: registered last so it becomes the
         * default; the火山/self-hosted backends stay available as
         * fallbacks via the tts/asr shell command. */
        extern int mimo_tts_register(void);
        extern int mimo_asr_register(void);
        mimo_tts_register();
        mimo_asr_register();

        s_backends_registered = 1;
    }

    http_tts_init();

    /* start BOOT-button wake poller (independent of voice wake) */
    static int s_btn_started = 0;
    if (!s_btn_started) {
        s_btn_started = 1;
        pthread_t bt;
        if (pthread_create(&bt, NULL, button_wake_task, NULL) != 0) {
            syslog(LOG_ERR, "[%s] failed to start button wake task\n", TAG);
        } else {
            pthread_detach(bt);
        }
    }

    return 0;
}

int voice_channel_start(void)
{
    vc_beacon("B7_VC_ENTER");

    pthread_mutex_lock(&s_voice.lock);
    if (s_voice.state != VOICE_IDLE) {
        pthread_mutex_unlock(&s_voice.lock);
        syslog(LOG_WARNING, "[%s] already recording\n", TAG);
        return -EBUSY;
    }

    /* Drain stale sem posts from previous sessions to prevent
     * sem_wait returning immediately before thread is ready. */
    while (sem_trywait(&s_voice.rec_ready) == 0)
        ;

    /* ── AEC workaround: stop active TTS to prevent echo ── */
    if (s_voice.tts_pb) {
        syslog(LOG_INFO,
            "[%s] PTT: stopping active TTS to prevent echo\n", TAG);
        s_voice.tts_abort = 1;
        audio_playback_stop(s_voice.tts_pb);

        /* Release lock so the speak thread can finish closing the
         * playback device.  Without this, we hold the lock through
         * the entire ASR pre-connect (300-800ms), blocking the speak
         * thread from calling audio_playback_close().  The playback
         * device stays open while we open the capture device, and on
         * BES hardware the shared codec (ci2s) gets confused, causing
         * incomplete or corrupted recording. */
        pthread_mutex_unlock(&s_voice.lock);

        /* Wait for the speak thread to finish — speak_lock is held
         * for the entire duration of voice_channel_speak(). */
        pthread_mutex_lock(&s_voice.speak_lock);
        pthread_mutex_unlock(&s_voice.speak_lock);

        syslog(LOG_INFO,
            "[%s] PTT: TTS playback fully closed\n", TAG);

        /* Re-acquire state lock and re-check — another thread may
         * have started recording while we released the lock. */
        pthread_mutex_lock(&s_voice.lock);
        if (s_voice.state != VOICE_IDLE) {
            pthread_mutex_unlock(&s_voice.lock);
            syslog(LOG_WARNING, "[%s] state changed during TTS stop\n", TAG);
            return -EBUSY;
        }
    }

    emotion_set("listening");   /* 麦克风即将打开：显示“聆听”表情 */

    /* PCM buffer is lazy-allocated only when streaming ASR fails
     * (batch fallback). This saves 128KB RAM on the happy path. */
    s_voice.pcm_cap = AGENT_VOICE_PCM_BUF_SIZE;
    s_voice.pcm_buf = NULL;
    s_voice.pcm_len = 0;
    s_voice.asr_stream = NULL;
    s_voice.vad_ended = 0;
    s_vad_started = 0;
    s_vad_silence_ms = 0;
    s_vad_speech_ms = 0;

    /* Pre-open streaming ASR session HERE (before spawning the
     * recording thread) so that TLS handshake (~300-800ms) completes
     * before audio capture starts.  Without this, the handshake runs
     * inside recording_thread while audio is already flowing, causing
     * the first 300-800ms of speech to be lost.
     *
     * If the pre-open fails we fall back to batch mode as before. */
    voice_asr_stream_t* pre_stream = voice_asr_stream_open();
    if (!pre_stream) {
        syslog(LOG_WARNING,
            "[%s] pre-open ASR failed, will retry in thread\n", TAG);
    } else {
        syslog(LOG_INFO, "[%s] ASR pre-connected\n", TAG);
        s_voice.asr_stream = pre_stream;
    }

    /* Open capture device (but do NOT start yet — start after the
     * recording thread is spawned so the consumer is ready before
     * audio frames begin flowing, preventing media_recorder queue
     * overflow "data queue is more than max count(4)"). */
    vc_beacon("B7_VC_PRECAP");

    s_voice.cap = audio_capture_open(
        AGENT_AUDIO_CAPTURE_DEV,
        AGENT_VOICE_SAMPLE_RATE,
        AGENT_VOICE_CHANNELS,
        AGENT_VOICE_BITS);

    if (!s_voice.cap) {
        if (pre_stream) {
            voice_asr_stream_abort(pre_stream);
            s_voice.asr_stream = NULL;
        }
        pthread_mutex_unlock(&s_voice.lock);
        syslog(LOG_ERR, "[%s] capture open failed\n", TAG);
        return -EIO;
    }

    s_voice.state = VOICE_RECORDING;
    pthread_mutex_unlock(&s_voice.lock);

    /* Spawn recording thread BEFORE starting capture */
    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, AGENT_VOICE_STACK);

    int ret = pthread_create(&s_voice.rec_thread, &attr,
        recording_thread, NULL);

    pthread_attr_destroy(&attr);

    if (ret != 0) {
        pthread_mutex_lock(&s_voice.lock);
        s_voice.state = VOICE_IDLE;
        audio_capture_close(s_voice.cap);
        s_voice.cap = NULL;
        if (s_voice.asr_stream) {
            voice_asr_stream_abort(s_voice.asr_stream);
            s_voice.asr_stream = NULL;
        }
        free(s_voice.pcm_buf);
        s_voice.pcm_buf = NULL;
        pthread_mutex_unlock(&s_voice.lock);
        return -ret;
    }

    /* Now start capture — wait for recording thread to be ready first */
    sem_wait(&s_voice.rec_ready);

    vc_beacon("B7_VC_PRESTARTCAP");

    if (audio_capture_start(s_voice.cap) < 0) {
        pthread_mutex_lock(&s_voice.lock);
        s_voice.state = VOICE_IDLE;
        if (s_voice.asr_stream) {
            voice_asr_stream_abort(s_voice.asr_stream);
            s_voice.asr_stream = NULL;
        }
        pthread_mutex_unlock(&s_voice.lock);
        audio_capture_abort(s_voice.cap);
        pthread_join(s_voice.rec_thread, NULL);
        audio_capture_close(s_voice.cap);
        s_voice.cap = NULL;
        return -EIO;
    }

    syslog(LOG_INFO, "[%s] PTT recording started\n", TAG);
    printf("Voice: recording... (use voice_stop to finish)\n");
    return 0;
}

/* ── Always-on listening ────────────────────────────────────────────────────
 *
 * Loops the existing session machinery: start capture, let the endpointing
 * close the utterance, stop - which finalises the ASR and pushes the transcript
 * into the agent's message bus - then let the answer be spoken before listening
 * again.
 *
 * The last step is not optional: without echo cancellation, a session started
 * while TTS is playing records the answer and the assistant talks to itself.
 */

#define VOICE_LISTEN_NO_SPEECH_MS  8000  /* give up on a silent session */
#define VOICE_LISTEN_POLL_MS       20
#define VOICE_LISTEN_REPLY_WAIT_MS 20000 /* answer starts within this */  /* [V73] LLM round trips exceed 12 s on this link */
#define VOICE_LISTEN_REPLY_MAX_MS  30000 /* answer finishes within this */
#define VOICE_LISTEN_SETTLE_MS     800   /* quiet before listening again */

static volatile int s_listen_stop;
static volatile int s_listen_running;

/* [V67] Set while voice_channel_speak() is waiting for the channel.  The
 * listen loop watches it and winds its current session down immediately so
 * the speaker does not have to wait out the no-speech timeout. */

static volatile int s_speak_pending;

static void* voice_listen_task(void* arg)
{
    (void)arg;

    vc_beacon("B7_LST_TASK");

    syslog(LOG_INFO, "[%s] always-on listening active\n", TAG);

    while (!s_listen_stop) {
        /* [V73] Do not open a session while somebody is waiting to speak:
         * voice_channel_start() would move the channel straight back into
         * VOICE_RECORDING and the speaker's hand-over wait would expire
         * again, which is exactly what silenced the greeting and any reply
         * that landed while a session was being restarted. */

        if (s_speak_pending) {
            usleep(50000);
            continue;
        }

        if (s_voice.state != VOICE_IDLE || s_voice.tts_pb != NULL) {
            static int s_v76_waitb;
            if (!s_v76_waitb) { s_v76_waitb = 1; vc_beacon("B7_LST_WAITB"); }
            /* [V69] Report a one-shot probe the first time the loop finds
             * the channel busy, so a stuck state is visible immediately. */

            static int s_v69_waiting;
            static int s_v69_waiting_state;

            if (!s_v69_waiting_state) {
                s_v69_waiting_state = (int)s_voice.state + 1;
                vc_probe("B7_LST_WAIT", (long)s_voice.state,
                    (long)(s_voice.tts_pb != NULL));
            }

            if (s_v69_waiting < 40) {
                s_v69_waiting++;
            } else if (s_v69_waiting == 40) {
                s_v69_waiting = 41;
                vc_probe("B7_LST_STUCK", (long)s_voice.state, 0);
            }

            usleep(VOICE_LISTEN_POLL_MS * 1000);
            continue;
        }

        vc_probe("B7_LST_SESS", 0, 0);

        if (voice_channel_start() != 0) {
            syslog(LOG_WARNING, "[%s] listen: session start failed\n", TAG);
            sleep(1);
            continue;
        }

        /* Wait for the endpointing, but never hold a silent session open: the
         * streaming ASR session has its own server-side idle timeout. */

        int waited = 0;

        while (!s_listen_stop && !s_voice.vad_ended) {
            usleep(VOICE_LISTEN_POLL_MS * 1000);
            waited += VOICE_LISTEN_POLL_MS;

            /* [V67] Someone wants to speak: hand the channel over now. */

            if (s_speak_pending) {
                vc_probe("B7_LST_YIELD", (long)s_voice.state, waited);
                syslog(LOG_INFO,
                    "[%s] listen: yielding channel to speaker\n", TAG);
                break;
            }

            if (waited >= VOICE_VAD_MAX_MS + 2000) {
                break;
            }

            if (!s_vad_started && waited >= VOICE_LISTEN_NO_SPEECH_MS) {
                syslog(LOG_INFO,
                    "[%s] listen: no speech for %d ms, restarting session\n",
                    TAG, waited);
                break;
            }
        }

        vc_probe("B7_LST_STOPPRE", (long)s_voice.state, waited);
        voice_channel_stop(); /* ASR + message_bus_push_inbound */
        vc_probe("B7_LST_STOPDONE", (long)s_voice.state, 0);

        /* Let the answer arrive and finish playing before the microphone is
         * live again. */

        int waited2 = 0;

        while (!s_listen_stop && s_voice.tts_pb == NULL &&
            waited2 < VOICE_LISTEN_REPLY_WAIT_MS) {
            usleep(50000);
            waited2 += 50;
        }

        waited2 = 0;

        while (!s_listen_stop && s_voice.tts_pb != NULL &&
            waited2 < VOICE_LISTEN_REPLY_MAX_MS) {
            usleep(50000);
            waited2 += 50;
        }

        usleep(VOICE_LISTEN_SETTLE_MS * 1000);
    }

    s_listen_running = 0;
    syslog(LOG_INFO, "[%s] always-on listening stopped\n", TAG);
    return NULL;
}

int voice_channel_listen_start(void)
{
    if (s_listen_running) {
        return 0;
    }

    s_listen_stop = 0;

    pthread_attr_t attr;
    pthread_t tid;

    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 16384);

    int ret = pthread_create(&tid, &attr, voice_listen_task, NULL);

    diag_report("LISTEN_START");
    vc_beacon(ret == 0 ? "B7_LS_OK" : "B7_LS_FAIL");

    pthread_attr_destroy(&attr);

    if (ret != 0) {
        syslog(LOG_ERR, "[%s] listen task create failed: %d\n", TAG, ret);
        return -ret;
    }

    pthread_detach(tid);
    s_listen_running = 1;
    return 0;
}

void voice_channel_listen_stop(void)
{
    s_listen_stop = 1;
}

int voice_channel_stop(void)
{
    pthread_mutex_lock(&s_voice.lock);
    if (s_voice.state != VOICE_RECORDING) {
        pthread_mutex_unlock(&s_voice.lock);
        return -EINVAL;
    }
    s_voice.state = VOICE_STOPPING;
    pthread_mutex_unlock(&s_voice.lock);

    audio_capture_abort(s_voice.cap);

    /* Wait for recording thread to finish */
    pthread_join(s_voice.rec_thread, NULL);

    /* [V69] Release the channel as soon as the recording is over.  This used
     * to stay in VOICE_STOPPING until after audio_capture_close() and
     * voice_asr_stream_finish(), and that finish is a network round trip -
     * seconds on this link.  Everything that wanted the channel (the boot
     * greeting, an agent reply) burned its whole wait in the meantime and
     * gave up with -EBUSY, which is why a session that ended correctly still
     * left the device mute. */

    pthread_mutex_lock(&s_voice.lock);
    s_voice.state = VOICE_IDLE;
    pthread_mutex_unlock(&s_voice.lock);

    emotion_set("thinking");   /* 录音结束：进入识别/思考阶段 */

    vc_probe("B7_STOP_IDLE", (long)s_voice.state, 0);

    /* Close capture device */
    audio_capture_close(s_voice.cap);

    pthread_mutex_lock(&s_voice.lock);
    s_voice.cap = NULL;
    size_t pcm_len = s_voice.pcm_len;
    voice_asr_stream_t* stream = s_voice.asr_stream;
    s_voice.asr_stream = NULL;
    s_voice.state = VOICE_IDLE;
    pthread_mutex_unlock(&s_voice.lock);

    syslog(LOG_INFO, "[%s] recorded %zu bytes (stream=%p)\n",
        TAG, pcm_len, (void*)stream);

    /* If streaming ASR was active, finish it to get result.
     * Note: in streaming mode pcm_len is always 0 because audio
     * goes directly to the ASR server, not to pcm_buf. */
    if (stream) {
        printf("Voice: streaming ASR active, finishing...\n");
        struct timespec asr_t0;
        clock_gettime(CLOCK_MONOTONIC, &asr_t0);
        char text[512];
        int ret = voice_asr_stream_finish(stream, text,
            sizeof(text));
        struct timespec asr_t1;
        clock_gettime(CLOCK_MONOTONIC, &asr_t1);
        long asr_ms = (asr_t1.tv_sec - asr_t0.tv_sec) * 1000
            + (asr_t1.tv_nsec - asr_t0.tv_nsec) / 1000000;
        syslog(LOG_INFO, "[%s] ASR finish: %ldms\n", TAG, asr_ms);

        free(s_voice.pcm_buf); /* may be NULL */
        s_voice.pcm_buf = NULL;

        if (ret == 0 && text[0] != '\0') {
            syslog(LOG_INFO, "[%s] stream ASR: %s\n", TAG, text);
            printf("ASR: %s\n", text);
            clock_gettime(CLOCK_MONOTONIC, &s_asr_done_ts);

            agent_msg_t msg;

            memset(&msg, 0, sizeof(msg));
            strncpy(msg.channel, AGENT_CHAN_VOICE,
                sizeof(msg.channel) - 1);
            strncpy(msg.chat_id, "voice",
                sizeof(msg.chat_id) - 1);
            msg.content = strdup(text);
            if (msg.content) {
                message_bus_push_inbound(&msg);
            }
        } else {
            syslog(LOG_WARNING,
                "[%s] stream ASR failed: %d\n", TAG, ret);
        }
        return 0;
    }

    /* Batch fallback: pcm_buf must exist if we reach here */
    if (!s_voice.pcm_buf || pcm_len == 0) {
        free(s_voice.pcm_buf);
        s_voice.pcm_buf = NULL;
        syslog(LOG_ERR, "[%s] no PCM data for batch ASR\n", TAG);
        return -ENODATA;
    }

    /* Batch fallback: pack pcm_len + audio for ASR thread */
    size_t total = sizeof(size_t) + pcm_len;
    unsigned char* pkg = malloc(total);

    if (!pkg) {
        free(s_voice.pcm_buf);
        s_voice.pcm_buf = NULL;
        return -ENOMEM;
    }

    memcpy(pkg, &pcm_len, sizeof(size_t));
    memcpy(pkg + sizeof(size_t), s_voice.pcm_buf, pcm_len);
    free(s_voice.pcm_buf);
    s_voice.pcm_buf = NULL;

    /* Spawn ASR worker thread */
    int ret = agent_task_create(asr_and_dispatch, "asr_worker",
        ASR_THREAD_STACK, pkg, AGENT_VOICE_PRIO);

    if (ret != OK) {
        free(pkg);
        return -EIO;
    }

    return 0;
}

int voice_channel_stop_with_text(char* text_out, size_t text_cap)
{
    if (!text_out || text_cap == 0) {
        return -EINVAL;
    }

    text_out[0] = '\0';

    pthread_mutex_lock(&s_voice.lock);
    if (s_voice.state != VOICE_RECORDING) {
        pthread_mutex_unlock(&s_voice.lock);
        return -EINVAL;
    }
    s_voice.state = VOICE_STOPPING;
    pthread_mutex_unlock(&s_voice.lock);

    audio_capture_abort(s_voice.cap);

    /* Wait for recording thread to finish */
    pthread_join(s_voice.rec_thread, NULL);

    /* [V69] Release the channel as soon as the recording is over.  This used
     * to stay in VOICE_STOPPING until after audio_capture_close() and
     * voice_asr_stream_finish(), and that finish is a network round trip -
     * seconds on this link.  Everything that wanted the channel (the boot
     * greeting, an agent reply) burned its whole wait in the meantime and
     * gave up with -EBUSY, which is why a session that ended correctly still
     * left the device mute. */

    pthread_mutex_lock(&s_voice.lock);
    s_voice.state = VOICE_IDLE;
    pthread_mutex_unlock(&s_voice.lock);

    emotion_set("thinking");   /* 录音结束：进入识别/思考阶段 */

    vc_probe("B7_STOP_IDLE", (long)s_voice.state, 0);

    /* Close capture device */
    audio_capture_close(s_voice.cap);

    pthread_mutex_lock(&s_voice.lock);
    s_voice.cap = NULL;
    size_t pcm_len = s_voice.pcm_len;
    voice_asr_stream_t* stream = s_voice.asr_stream;
    s_voice.asr_stream = NULL;
    s_voice.state = VOICE_IDLE;
    pthread_mutex_unlock(&s_voice.lock);

    syslog(LOG_INFO, "[%s] recorded %zu bytes (with_text, stream=%p)\n",
        TAG, pcm_len, (void*)stream);

    /* Streaming ASR path: finish and copy text to caller.
     * In streaming mode audio goes directly to server, so
     * pcm_len may be 0 — that's OK, don't abort. */
    if (stream) {
        char text[512];
        int ret = voice_asr_stream_finish(stream, text,
            sizeof(text));

        free(s_voice.pcm_buf);
        s_voice.pcm_buf = NULL;

        if (ret == 0 && text[0] != '\0') {
            syslog(LOG_INFO, "[%s] stream ASR (with_text): %s\n",
                TAG, text);
            strncpy(text_out, text, text_cap - 1);
            text_out[text_cap - 1] = '\0';
        }
        return 0;
    }

    /* Batch fallback: synchronous ASR inline (no thread) */
    if (!s_voice.pcm_buf || pcm_len == 0) {
        free(s_voice.pcm_buf);
        s_voice.pcm_buf = NULL;
        syslog(LOG_ERR, "[%s] no PCM data for batch ASR\n", TAG);
        return -ENODATA;
    }

    syslog(LOG_INFO, "[%s] batch ASR (with_text): %zu bytes\n",
        TAG, pcm_len);

    char text[512];
    int ret = voice_asr_recognize(s_voice.pcm_buf, pcm_len,
        text, sizeof(text));

    free(s_voice.pcm_buf);
    s_voice.pcm_buf = NULL;

    if (ret == 0 && text[0] != '\0') {
        syslog(LOG_INFO, "[%s] batch ASR result: %s\n", TAG, text);
        strncpy(text_out, text, text_cap - 1);
        text_out[text_cap - 1] = '\0';
    } else if (ret != 0) {
        syslog(LOG_WARNING, "[%s] batch ASR failed: %d\n",
            TAG, ret);
    }

    return 0;
}

/* Strip Markdown formatting that TTS would read aloud.
 * Removes: **bold**, *italic*, # headings, - list bullets.
 * Operates in-place, result is always <= original length. */
static void tts_strip_markdown(char* s)
{
    char* r = s; /* read pointer */
    char* w = s; /* write pointer */

    while (*r) {
        /* Skip ** (bold markers) */
        if (r[0] == '*' && r[1] == '*') {
            r += 2;
            continue;
        }

        /* Skip lone * (italic markers) \xe2\x80\x94 but keep * in
         * contexts like "3*4" (digit before and after) */
        if (r[0] == '*') {
            if ((r == s || !isdigit((unsigned char)r[-1]))
                || !isdigit((unsigned char)r[1])) {
                r++;
                continue;
            }
        }

        /* Skip # at line start (headings) */
        if (r[0] == '#' && (r == s || r[-1] == '\n')) {
            while (*r == '#' || *r == ' ') {
                r++;
            }
            continue;
        }

        /* Skip "- " at line start (list bullets) */
        if (r[0] == '-' && r[1] == ' '
            && (r == s || r[-1] == '\n')) {
            r += 2;
            continue;
        }

        *w++ = *r++;
    }

    *w = '\0';
}

/* [DIAG] server-visible probe beacons (same trick as audio_playback.c):
 * each unique ping is logged by the TTS server, so we can see how far the
 * speak path got without a serial console. */
/* Still used by the V65 mic-level probe, which legitimately wants a /tts
 * request (that is how the peak value reaches the server). */
static void vc_beacon_cb(const unsigned char *d, size_t l, int f, void *u)
{
  (void)d; (void)l; (void)f; (void)u;
}
static void vc_beacon(const char *tag)
{
  /* [V68] Probes now travel on the bridge's lightweight /diag endpoint.
   * They used to call http_tts_speak_stream(), which made the server
   * synthesise speech and return a 30 kB PCM body for every probe tag: slow,
   * and it exercised the very audio path the probe was meant to observe. */
  diag_report(tag);
}

/* [V69] Probe carrying two numbers, e.g. "B7_SP_BUSY3_12000" = state 3 after
 * 12 s.  Enough to identify which state transition never happens, on a board
 * whose only usable output is the network. */

static void vc_probe(const char *tag, long a, long b)
{
  char tb[48];

  snprintf(tb, sizeof(tb), "%s%ld_%ld", tag, a, b);
  diag_report(tb);
}

/* ── 表情信号 ─────────────────────────────────────────────────────────
 * 把语音会话状态写入 /tmp/emotion.txt，faced 据此在屏幕左上角显示
 * 对应表情徽章（idle/happy/thinking/sad/listening/speaking/sleeping）。
 * 写文件开销极小（几十微秒），每状态切换一次即可。
 */

static void emotion_set(const char *name)
{
#ifdef CONFIG_LIBC_TMPDIR
  static const char emo_path[] = CONFIG_LIBC_TMPDIR "/emotion.txt";
#else
  static const char emo_path[] = "/tmp/emotion.txt";
#endif
  int fd;

  fd = open(emo_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (fd < 0)
    {
      return;
    }

  (void)write(fd, name, strlen(name));
  close(fd);
}

int voice_channel_speak(const char* text)
{
    if (!text || text[0] == '\0') {
        return -EINVAL;
    }

    vc_beacon("B7_SP_ENTER");

    /* ── Serialize concurrent speak calls ── */
    /* If another thread is already speaking, abort its TTS playback
     * so we don't overlap media_player sessions (which causes the
     * media framework to attempt a ~1.3GB allocation and crash). */
    pthread_mutex_lock(&s_voice.lock);
    if (s_voice.tts_pb) {
        syslog(LOG_INFO, "[%s] speak: aborting previous TTS\n", TAG);
        s_voice.tts_abort = 1;
        audio_playback_stop(s_voice.tts_pb);
    }
    pthread_mutex_unlock(&s_voice.lock);

    pthread_mutex_lock(&s_voice.speak_lock);

    /* ── [V67] Half-duplex hand-over instead of an instant -EBUSY ──
     *
     * The always-on listen loop keeps the channel in VOICE_RECORDING for
     * most of its cycle, so the old guard dropped the boot greeting every
     * time and also dropped replies that arrived while a session was open.
     * Raise s_speak_pending, which makes the listen loop finish the session
     * it is in, then wait for it to release the channel. */

    if (s_voice.state != VOICE_IDLE) {
        int waited_ms = 0;

        vc_probe("B7_SP_WAIT", (long)s_voice.state, 0);

        s_speak_pending = 1;

        while (s_voice.state != VOICE_IDLE && waited_ms < 12000) {
            usleep(100000);
            waited_ms += 100;
        }

        s_speak_pending = 0;

        if (s_voice.state != VOICE_IDLE) {
            pthread_mutex_unlock(&s_voice.speak_lock);
            syslog(LOG_WARNING,
                "[%s] speak: channel busy after %d ms\n", TAG, waited_ms);
            vc_probe("B7_SP_BUSY", (long)s_voice.state, (long)waited_ms);
            return -EBUSY;
        }

        vc_probe("B7_SP_GOT", (long)waited_ms, 0);

        syslog(LOG_INFO, "[%s] speak: channel acquired after %d ms\n",
            TAG, waited_ms);
    }

    /* Work on a mutable copy so we can strip markdown in-place */
    size_t text_len = strlen(text);
    char* clean = malloc(text_len + 1);

    if (!clean) {
        vc_beacon("B7_SP_NOMEM");
        pthread_mutex_unlock(&s_voice.speak_lock);
        return -ENOMEM;
    }

    memcpy(clean, text, text_len + 1);
    tts_strip_markdown(clean);

    /* If stripping left nothing, bail out */
    if (clean[0] == '\0') {
        vc_beacon("B7_SP_EMPTY");
        free(clean);
        pthread_mutex_unlock(&s_voice.speak_lock);
        return 0;
    }

    syslog(LOG_INFO, "[%s] speak: \"%.*s\" (%zu bytes)\n",
        TAG, 60, clean, strlen(clean));

    clock_gettime(CLOCK_MONOTONIC, &s_tts_start);
    s_tts_first_chunk = 0;

    if (s_asr_done_ts.tv_sec > 0) {
        long llm_ms = (s_tts_start.tv_sec - s_asr_done_ts.tv_sec) * 1000
            + (s_tts_start.tv_nsec - s_asr_done_ts.tv_nsec) / 1000000;
        syslog(LOG_INFO, "[%s] LLM latency: %ldms\n", TAG, llm_ms);
    }

    /* Try streaming TTS first (callback per chunk) */
    vc_beacon("B7_SP_PRE");
    audio_playback_t* pb = audio_playback_open(
        AGENT_AUDIO_PLAYBACK_DEV,
        AGENT_TTS_WS_SAMPLE_RATE,
        AGENT_VOICE_CHANNELS,
        AGENT_VOICE_BITS);
    vc_beacon(pb ? "B7_SP_POST_OK" : "B7_SP_POST_NULL");

    if (!pb) {
        syslog(LOG_ERR, "[%s] playback open failed; fetching TTS without "
            "playback (null-sink) so the request is still observable\n", TAG);
        /* Fall through with pb == NULL: tts_stream_cb is null-safe and drops
         * the PCM, but http_tts_speak_stream still issues the HTTP request,
         * letting us confirm the greeting path ran via the TTS server logs
         * even when the codec/playback device cannot be opened. */
    }

    /* Register active playback so voice_channel_start() can abort it */
    if (pb) {
        pthread_mutex_lock(&s_voice.lock);
        s_voice.tts_abort = 0;
        s_voice.tts_pb = pb;
        pthread_mutex_unlock(&s_voice.lock);

        emotion_set("speaking");   /* 开始说话：显示“说话”表情 */
    }

    /* Use the generic streaming entry point: it honours the selected
     * backend (MiMo) and falls back to the self-hosted HTTP service and
     * then to volc.  HTTP TTS/ASR need no paid credentials. */
    int ret = voice_tts_speak_stream(clean, tts_stream_cb, pb);
    if (ret != 0) {
        syslog(LOG_WARNING,
            "[%s] streaming TTS failed (%d)\n", TAG, ret);
    }

    struct timespec tts_net;
    clock_gettime(CLOCK_MONOTONIC, &tts_net);
    long net_ms = (tts_net.tv_sec - s_tts_start.tv_sec) * 1000
        + (tts_net.tv_nsec - s_tts_start.tv_nsec) / 1000000;
    syslog(LOG_INFO, "[%s] TTS network done: %ldms\n", TAG, net_ms);

    /* Unregister active playback */
    pthread_mutex_lock(&s_voice.lock);
    s_voice.tts_pb = NULL;
    s_voice.tts_abort = 0;
    pthread_mutex_unlock(&s_voice.lock);

    if (pb) {
        audio_playback_close(pb);
    }
    free(clean);

    if (ret != 0) {
        syslog(LOG_ERR, "[%s] TTS stream failed: %d\n", TAG, ret);
        emotion_set("sad");   /* 播放失败：显示“难过”表情 */
        pthread_mutex_unlock(&s_voice.speak_lock);
        return ret;
    }

    emotion_set("happy");   /* 回复说完：开心一下 */

    struct timespec tend;
    clock_gettime(CLOCK_MONOTONIC, &tend);
    long total_ms = (tend.tv_sec - s_tts_start.tv_sec) * 1000
        + (tend.tv_nsec - s_tts_start.tv_nsec) / 1000000;
    syslog(LOG_INFO, "[%s] speak done: total %ldms (play wait %ldms)\n",
        TAG, total_ms, total_ms - net_ms);
    pthread_mutex_unlock(&s_voice.speak_lock);
    return 0;
}

/* ── Test commands (file-based, for QEMU) ───────────────── */

int voice_channel_test_tts(const char* text, const char* out_path)
{
    if (!text || !out_path) {
        printf("Usage: voice_test_tts <text> [output_path]\n");
        return -EINVAL;
    }

    printf("TTS: synthesizing \"%s\" ...\n", text);

    unsigned char* pcm = malloc(AGENT_VOICE_PCM_BUF_SIZE);

    if (!pcm) {
        printf("Error: out of memory\n");
        return -ENOMEM;
    }

    size_t pcm_len;
    int ret = voice_tts_speak(text, pcm,
        AGENT_VOICE_PCM_BUF_SIZE, &pcm_len);

    if (ret != 0) {
        free(pcm);
        printf("TTS failed: %d\n", ret);
        return ret;
    }

    ret = write_pcm_file(out_path, pcm, pcm_len);
    free(pcm);

    if (ret == 0) {
        printf("TTS OK: %zu bytes -> %s\n", pcm_len, out_path);
    } else {
        printf("Failed to write %s: %d\n", out_path, ret);
    }

    return ret;
}

typedef struct {
    char pcm_path[256];
} asr_task_args_t;

static void* asr_file_worker(void* arg)
{
    asr_task_args_t* a = (asr_task_args_t*)arg;
    unsigned char* pcm = NULL;
    size_t pcm_len = 0;

    int ret = read_pcm_file(a->pcm_path, &pcm, &pcm_len);

    if (ret != 0) {
        printf("Failed to read %s: %d\n", a->pcm_path, ret);
        free(a);
        return NULL;
    }

    printf("ASR: recognizing %zu bytes from %s ...\n",
        pcm_len, a->pcm_path);

    char text[512];

    ret = voice_asr_recognize(pcm, pcm_len, text, sizeof(text));
    free(pcm);

    if (ret == 0) {
        printf("ASR result: %s\n", text);

        agent_msg_t msg;

        memset(&msg, 0, sizeof(msg));
        strncpy(msg.channel, AGENT_CHAN_VOICE,
            sizeof(msg.channel) - 1);
        strncpy(msg.chat_id, "voice",
            sizeof(msg.chat_id) - 1);
        msg.content = strdup(text);
        if (msg.content) {
            message_bus_push_inbound(&msg);
            printf("Sent to agent.\n");
        }
    } else {
        printf("ASR failed: %d\n", ret);
    }

    free(a);
    return NULL;
}

int voice_channel_test_asr(const char* pcm_path)
{
    if (!pcm_path) {
        printf("Usage: voice_test_asr <pcm_file>\n");
        return -EINVAL;
    }

    asr_task_args_t* args = malloc(sizeof(asr_task_args_t));

    if (!args) {
        printf("Error: out of memory\n");
        return -ENOMEM;
    }

    strncpy(args->pcm_path, pcm_path, sizeof(args->pcm_path) - 1);
    args->pcm_path[sizeof(args->pcm_path) - 1] = '\0';

    int ret = agent_task_create(asr_file_worker, "asr_test",
        ASR_THREAD_STACK, args, AGENT_VOICE_PRIO);

    if (ret != OK) {
        printf("Error: failed to create ASR thread\n");
        free(args);
        return -EIO;
    }

    printf("ASR test started (background thread).\n");
    return 0;
}

void voice_channel_cleanup(void)
{
    voice_channel_stop();
}

