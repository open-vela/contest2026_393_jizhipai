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

/* audio_playback_native.c - Streaming playback through the standard NuttX
 * audio character device, bypassing the Vela media framework.
 *
 * Data flow (TTS voice path):
 *   volc TTS WS chunks (base64 PCM, "format":"pcm") -> decoded s16le PCM
 *   -> voice_channel tts_stream_cb -> audio_playback_write()
 *   -> native_playback_write(): copy into apb, AUDIOIOC_ENQUEUEBUFFER
 *   -> es8311 lower half worker -> I2S0 TX -> ES8311 DAC -> speaker.
 *
 * Buffer lifecycle: nbuffers apbs are allocated via AUDIOIOC_ALLOCBUFFER.
 * write() fills free apbs and enqueues them; played-out buffers come back
 * as AUDIO_MSG_DEQUEUE messages on the private mq and return to the free
 * pool, so write() blocks like a pipe when the pipeline is full.
 */

#include "audio_native.h"

#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <nuttx/audio/audio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

static const char* TAG = "audio_pb_nat";

/* Pipeline geometry when the driver does not report its own
 * (GETBUFFERINFO fails on the es8311 lower half in this build).
 * 4096 B @ 24 kHz mono s16le = ~85 ms per buffer, 4 buffers deep. */

#define PB_FALLBACK_NBUFFERS  4
#define PB_FALLBACK_BUFSIZE   4096

#define PB_MAX_NBUFFERS       8
#define PB_MAX_BUFSIZE        16384

/* How long write() waits for a played-out buffer before giving up, and
 * how long close() waits for in-flight buffers during the drain. */

#define PB_MSG_TIMEOUT_SEC    10
#define PB_DRAIN_TIMEOUT_SEC  5

struct native_pb_s {
    int fd;
    mqd_t mq;
    char mqname[NATIVE_MQ_NAME_LEN];

    struct ap_buffer_s** apbs;    /* all allocated apbs */
    int nbuffers;

    struct ap_buffer_s** freeq;   /* apbs ready to be filled */
    int nfree;

    struct ap_buffer_s* cur;      /* partially filled, not yet enqueued */
    int outstanding;              /* enqueued, not yet returned */

    volatile int stopped;
    int complete;                 /* driver sent AUDIO_MSG_COMPLETE */
    size_t total_written;
};

/* Return the next free apb, reclaiming played-out buffers from the mq.
 * Returns NULL when the stream was stopped/completed or on timeout. */

static struct ap_buffer_s* pb_take_free(struct native_pb_s* pb)
{
    struct audio_msg_s msg;
    int ret;

    while (1) {
        if (pb->stopped) {
            return NULL;
        }

        if (pb->nfree > 0) {
            return pb->freeq[--pb->nfree];
        }

        ret = native_mq_receive_timed(pb->mq, &msg, PB_MSG_TIMEOUT_SEC);
        if (ret <= 0) {
            syslog(LOG_ERR, "[%s] no played-out buffer (ret=%d)\n",
                TAG, ret);
            return NULL;
        }

        switch (msg.msg_id) {
        case AUDIO_MSG_DEQUEUE:
            if (msg.u.ptr && pb->outstanding > 0) {
                pb->outstanding--;
                pb->freeq[pb->nfree++] = (struct ap_buffer_s*)msg.u.ptr;
            }
            break;

        case AUDIO_MSG_COMPLETE:
            pb->complete = 1;
            syslog(LOG_INFO, "[%s] driver completed stream\n", TAG);
            return NULL;

        case AUDIO_MSG_STOP:
            pb->stopped = 1;
            return NULL;

        default:
            break;
        }
    }
}

/* Enqueue the partially-filled current buffer.  Returns 0 or -errno. */

static int pb_flush_cur(struct native_pb_s* pb)
{
    int ret;

    if (!pb->cur || pb->cur->nbytes == 0) {
        if (pb->cur) {
            pb->freeq[pb->nfree++] = pb->cur;
            pb->cur = NULL;
        }
        return 0;
    }

    ret = native_enqueue_buffer(pb->fd, pb->cur);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] enqueue failed: %d\n", TAG, ret);
        pb->freeq[pb->nfree++] = pb->cur;
        pb->cur = NULL;
        return ret;
    }

    pb->outstanding++;
    pb->cur = NULL;
    return 0;
}

int native_playback_open(native_pb_t** out_pb, const char* dev_path,
    unsigned int sample_rate, unsigned int channels,
    unsigned int bits_per_sample)
{
    struct native_pb_s* pb;
    struct ap_buffer_info_s bufinfo;
    size_t buf_size;
    int ret;
    int i;

    if (!out_pb || !dev_path || channels == 0 || channels > 2) {
        return -EINVAL;
    }

    *out_pb = NULL;

    pb = calloc(1, sizeof(*pb));
    if (!pb) {
        return -ENOMEM;
    }

    /* Sentinels so the shared error/cleanup path is safe. */

    pb->fd = -1;
    pb->mq = -1;

    /* Open the raw audio device (e.g. /dev/audio/pcm0p). */

    pb->fd = open(dev_path, O_RDWR);
    if (pb->fd < 0) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] open(%s) failed: %d\n", TAG, dev_path, ret);
        goto err_out;
    }

    /* Reserve an exclusive session with the driver. */

    if (ioctl(pb->fd, AUDIOIOC_RESERVE, 0) != OK) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] AUDIOIOC_RESERVE failed: %d\n", TAG, ret);
        goto err_out;
    }

    /* Private mq for buffer-completion notifications. */

    ret = native_mq_create(pb->mqname, sizeof(pb->mqname), &pb->mq);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] mq_create failed: %d\n", TAG, ret);
        goto err_out;
    }

    if (ioctl(pb->fd, AUDIOIOC_REGISTERMQ, (unsigned long)pb->mq) != OK) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] REGISTERMQ failed: %d\n", TAG, ret);
        goto err_out;
    }

    /* Confirm this is an output-capable audio device. */

    ret = native_check_caps(pb->fd, AUDIO_TYPE_OUTPUT);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] %s is not an output audio device\n",
            TAG, dev_path);
        goto err_out;
    }

    /* Configure the stream format (es8311 reprograms the I2S clocks). */

    ret = native_configure(pb->fd, AUDIO_TYPE_OUTPUT, sample_rate,
        channels, bits_per_sample);
    if (ret < 0) {
        syslog(LOG_ERR, "[%s] configure %uHz %uch %ubit failed: %d\n",
            TAG, sample_rate, channels, bits_per_sample, ret);
        goto err_out;
    }

    /* Pipeline geometry: prefer the driver's own numbers, clamp to sane
     * streaming limits, otherwise use the fallback (4 x 4096 B). */

    pb->nbuffers = PB_FALLBACK_NBUFFERS;
    buf_size = PB_FALLBACK_BUFSIZE;
    if (ioctl(pb->fd, AUDIOIOC_GETBUFFERINFO,
              (unsigned long)&bufinfo) == OK &&
        bufinfo.nbuffers > 0 && bufinfo.buffer_size > 0) {
        pb->nbuffers = (bufinfo.nbuffers > PB_MAX_NBUFFERS)
            ? PB_MAX_NBUFFERS : (int)bufinfo.nbuffers;
        buf_size = (bufinfo.buffer_size > PB_MAX_BUFSIZE)
            ? PB_MAX_BUFSIZE : (size_t)bufinfo.buffer_size;
    }

    pb->apbs = calloc(pb->nbuffers, sizeof(*pb->apbs));
    pb->freeq = calloc(pb->nbuffers, sizeof(*pb->freeq));
    if (!pb->apbs || !pb->freeq) {
        ret = -ENOMEM;
        goto err_out;
    }

    for (i = 0; i < pb->nbuffers; i++) {
        pb->apbs[i] = native_alloc_buffer(pb->fd, buf_size);
        if (!pb->apbs[i]) {
            ret = -ENOMEM;
            syslog(LOG_ERR, "[%s] alloc apb %d failed\n", TAG, i);
            goto err_out;
        }

        pb->freeq[pb->nfree++] = pb->apbs[i];
    }

    /* Start the stream; data flows as write() enqueues buffers. */

    if (ioctl(pb->fd, AUDIOIOC_START, 0) != OK) {
        ret = -errno;
        syslog(LOG_ERR, "[%s] AUDIOIOC_START failed: %d\n", TAG, ret);
        goto err_out;
    }

    *out_pb = pb;
    syslog(LOG_INFO,
        "[%s] native playback ready (%s, %uHz %uch %ubit, %d x %u B)\n",
        TAG, dev_path, sample_rate, channels, bits_per_sample,
        pb->nbuffers,
        (unsigned)(pb->apbs[0] ? pb->apbs[0]->nmaxbytes : 0));
    return 0;

err_out:
    native_playback_close(pb);
    return ret;
}

int native_playback_write(native_pb_t* pb, const void* buf, size_t len)
{
    const unsigned char* src = buf;
    size_t remaining = len;

    if (!pb || !buf || len == 0) {
        return -EINVAL;
    }

    if (pb->stopped) {
        return -ECANCELED;
    }

    while (remaining > 0) {
        struct ap_buffer_s* apb;
        size_t space;
        size_t n;

        if (pb->stopped) {
            return -ECANCELED;
        }

        if (!pb->cur) {
            pb->cur = pb_take_free(pb);
            if (!pb->cur) {
                if (pb->stopped) {
                    return -ECANCELED;
                }
                if (pb->complete) {
                    return -EPIPE;
                }
                return -EIO;
            }
            pb->cur->nbytes = 0;
            pb->cur->curbyte = 0;
        }

        apb = pb->cur;
        space = apb->nmaxbytes - apb->nbytes;
        n = (space < remaining) ? space : remaining;

        memcpy(apb->samp + apb->nbytes, src, n);
        apb->nbytes += (apb_samp_t)n;
        src += n;
        remaining -= n;
        pb->total_written += n;

        if (apb->nbytes >= apb->nmaxbytes) {
            int ret = pb_flush_cur(pb);
            if (ret < 0) {
                return ret;
            }
        }
    }

    return (int)len;
}

void native_playback_stop(native_pb_t* pb)
{
    if (!pb || pb->stopped) {
        return;
    }

    pb->stopped = 1;

    /* Discard the partially-filled buffer; STOP makes the driver return
     * all in-flight buffers via AUDIO_MSG_DEQUEUE messages. */

    if (pb->cur) {
        pb->freeq[pb->nfree++] = pb->cur;
        pb->cur = NULL;
    }

    ioctl(pb->fd, AUDIOIOC_STOP, 0);
}

void native_playback_close(struct native_pb_s* pb)
{
    int ret;
    int i;

    if (!pb) {
        return;
    }

    if (!pb->stopped && pb->fd >= 0) {
        /* Graceful drain: flush the partial buffer, then wait until the
         * driver returned every enqueued buffer so nothing is cut off. */

        if (pb_flush_cur(pb) == 0) {
            struct audio_msg_s msg;

            while (pb->outstanding > 0) {
                ret = native_mq_receive_timed(pb->mq, &msg,
                    PB_DRAIN_TIMEOUT_SEC);
                if (ret <= 0) {
                    break;
                }

                if (msg.msg_id == AUDIO_MSG_DEQUEUE && msg.u.ptr &&
                    pb->outstanding > 0) {
                    pb->outstanding--;
                    pb->freeq[pb->nfree++] =
                        (struct ap_buffer_s*)msg.u.ptr;
                } else if (msg.msg_id == AUDIO_MSG_COMPLETE ||
                           msg.msg_id == AUDIO_MSG_STOP) {
                    break;
                }
            }
        }
    }

    if (pb->fd >= 0) {
        ioctl(pb->fd, AUDIOIOC_STOP, 0);
    }

    if (pb->mq >= 0) {
        ioctl(pb->fd, AUDIOIOC_UNREGISTERMQ, (unsigned long)pb->mq);
        mq_close(pb->mq);
        mq_unlink(pb->mqname);
        pb->mq = -1;
    }

    if (pb->fd >= 0) {
        ioctl(pb->fd, AUDIOIOC_RELEASE, 0);

        if (pb->apbs) {
            for (i = 0; i < pb->nbuffers; i++) {
                native_free_buffer(pb->fd, pb->apbs[i]);
            }
        }

        close(pb->fd);
        pb->fd = -1;
    }

    free(pb->apbs);
    free(pb->freeq);
    free(pb);

    syslog(LOG_INFO, "[%s] closed\n", TAG);
}
