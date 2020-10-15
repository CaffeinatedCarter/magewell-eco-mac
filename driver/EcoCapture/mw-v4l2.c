////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#include <linux/types.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/videodev2.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/pagemap.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>

#include "ospi/ospi.h"

#include "win-types.h"
#include "mw-fourcc.h"

#include "capture-channel.h"
#include "supports/deinterlace-blend.h"
#include "supports/image-copy.h"
#include "supports/math.h"

#include "mw-version.h"
#include "mw-v4l2.h"
#include "mw-stream.h"
#include "mw-driver-name.h"

#include "v4l2-sg-buf.h"

#define MIN_WIDTH  48
#define MIN_HEIGHT 32

/* ------------------------------------------------------------------
   Basic structures
   ------------------------------------------------------------------*/

struct v4l2_fmt {
    char  *name;
    u32    fourcc;          /* v4l2 format id */
    int    depth;
};

static struct v4l2_fmt g_formats[] = {
    {
        .name     = "4:2:2, packed, YUYV",
        .fourcc   = V4L2_PIX_FMT_YUYV,
        .depth    = 16,
    },
    {
        .name     = "4:2:2, packed, UYVY",
        .fourcc   = V4L2_PIX_FMT_UYVY,
        .depth    = 16,
    },
    {
        .name     = "4:2:0, NV12",
        .fourcc   = V4L2_PIX_FMT_NV12,
        .depth    = 12,
    },
    {
        .name     = "4:2:0, planar, YV12",
        .fourcc   = V4L2_PIX_FMT_YVU420,
        .depth    = 12,
    },
    {
        .name     = "4:2:0, planar, I420",
        .fourcc   = V4L2_PIX_FMT_YUV420,
        .depth    = 12,
    },
    {
        .name     = "RGB24",
        .fourcc   = V4L2_PIX_FMT_RGB24,
        .depth    = 24,
    },
    {
        .name     = "RGB32",
        .fourcc   = V4L2_PIX_FMT_RGB32,
        .depth    = 32,
    },
    {
        .name     = "BGR24",
        .fourcc   = V4L2_PIX_FMT_BGR24,
        .depth    = 24,
    },
    {
        .name     = "BGR32",
        .fourcc   = V4L2_PIX_FMT_BGR32,
        .depth    = 32,
    },
    {
        .name     = "Greyscale 8 bpp",
        .fourcc   = V4L2_PIX_FMT_GREY,
        .depth    = 8,
    },
    {
        .name     = "Greyscale 16 bpp",
        .fourcc   = V4L2_PIX_FMT_Y16,
        .depth    = 16,
    },
    {
        .name     = "32-bit, AYUV",
        .fourcc   = V4L2_PIX_FMT_YUV32,
        .depth    = 32,
    },
};

static struct v4l2_fmt *get_format(struct v4l2_format *f)
{
    struct v4l2_fmt *fmt;
    unsigned int k;

    for (k = 0; k < ARRAY_SIZE(g_formats); k++) {
        fmt = &g_formats[k];
        if (fmt->fourcc == f->fmt.pix.pixelformat)
            break;
    }

    if (k == ARRAY_SIZE(g_formats))
        return NULL;

    return &g_formats[k];
}

bool enable_raw_mode = false;

/* video info */
static struct v4l2_fmt          *g_def_fmt = &g_formats[0];
static unsigned int             g_def_width = 1920, g_def_height = 1080;
static unsigned int             g_def_frame_duration = 400000;

struct mw_v4l2_channel {
    /* video device */
    struct video_device *       vfd;
    struct device *             pci_dev;

    struct list_head            chnl_node;

    struct capture_channel *    capch; /* low level card driver */

    struct mw_device            mw_dev;

    timer_event_manager         *m_pTimerEventManager;
    notify_event_manager        *m_pNotifyEventManager;

    /* these settings are channel level */
    /* vidoe info */
    // TODO lock
    os_spinlock_t               format_lock;
    int                         ch_ref;
    struct v4l2_fmt             *fmt;
    unsigned int                width, height;
    unsigned int                stride;
    unsigned int                image_size;
};

struct mw_stream_v4l2 {
    struct mutex                v4l2_mutex;
    unsigned long               generating;

    struct v4l2_sg_buf_queue    vsq;
    struct list_head            active;
    unsigned int                sequence;
    /* thread for generating video stream*/
    struct task_struct          *kthread;
    long long                   llstart_time;
    timer_event_t               *timer;
    bool                        exit_capture_thread;
    notify_event_t              *notify;

    /* vidoe info */
    struct v4l2_fmt             *fmt;
    unsigned int                width, height;
    unsigned int                stride;
    unsigned int                capstride;
    unsigned int                image_size;
    unsigned int                frame_duration;
};

struct mw_stream_pipe {
    struct mw_v4l2_channel      *vch;

    /* sync for close */
    struct rw_semaphore         io_sem;

    struct mw_stream_v4l2       s_v4l2;
    struct mw_stream            s_mw;
};

static inline u32 fourcc_v4l2_to_mwcap(u32 fourcc)
{
    switch (fourcc) {
    case V4L2_PIX_FMT_NV12:
        return MWFOURCC_NV12;
    case V4L2_PIX_FMT_YVU420:
        return MWFOURCC_YV12;
    case V4L2_PIX_FMT_YUV420:
        return MWFOURCC_I420;
    case V4L2_PIX_FMT_UYVY:
        return MWFOURCC_UYVY;
    case V4L2_PIX_FMT_YUYV:
        return MWFOURCC_YUYV;
    case V4L2_PIX_FMT_RGB24:
        return MWFOURCC_RGB24;
    case V4L2_PIX_FMT_RGB32:
        return MWFOURCC_RGBA;
    case V4L2_PIX_FMT_BGR24:
        return MWFOURCC_BGR24;
    case V4L2_PIX_FMT_BGR32:
        return MWFOURCC_BGRA;
    case V4L2_PIX_FMT_GREY:
        return MWFOURCC_GREY;
    case V4L2_PIX_FMT_Y16:
        return MWFOURCC_Y16;
    case V4L2_PIX_FMT_YUV32:
        return MWFOURCC_AYUV;
    default:
        return MWFOURCC_NV12;
    }

    return MWFOURCC_NV12;
}

static inline bool v4l2_is_generating(struct mw_stream_v4l2 *s_v4l2)
{
    return test_bit(0, &s_v4l2->generating);
}

static void v4l2_process_one_frame(struct mw_stream_pipe *pipe, int iframe)
{
    struct mw_v4l2_channel *vch = pipe->vch;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    struct v4l2_sg_buf *vbuf;
    int ret = -1;
    bool bottom_up;

    vbuf = v4l2_sg_queue_get_activebuf(&s_v4l2->vsq);
    if (vbuf == NULL)
        return;

    bottom_up = false;

    ret = -1;
    if (vbuf->v4l2_buf.memory == V4L2_MEMORY_MMAP || vbuf->v4l2_buf.memory == V4L2_MEMORY_USERPTR) {
        MWCAP_VIDEO_SIGNAL_STATUS videoSignalStatus;
        void *pvFrame;

        capture_channel_GetVideoSignalStatus(vch->capch, &videoSignalStatus);

        vbuf->anc_packet_count = MWCAP_MAX_ANC_PACKETS_PER_FRAME;
        pvFrame = capture_channel_GetVideoCaptureFrame(vch->capch, NULL,
                                                       vbuf->anc_packets, &vbuf->anc_packet_count);

        if (NULL != pvFrame) {
            // TODO
            if (/* settings.process_settings.deinterlaceMode == MWCAP_VIDEO_DEINTERLACE_BLEND
                && */videoSignalStatus.bInterlaced
                && videoSignalStatus.state == MWCAP_VIDEO_SIGNAL_LOCKED) {
                ImageDeinterlace(fourcc_v4l2_to_mwcap(vch->fmt->fourcc),
                                 vch->width,
                                 vch->height,
                                 pvFrame,
                                 s_v4l2->capstride,
                                 v4l2_sg_get_vaddr(vbuf),
                                 vch->stride,
                                 bottom_up);
            } else {
                ImageCopy(fourcc_v4l2_to_mwcap(vch->fmt->fourcc),
                          vch->width,
                          vch->height,
                          pvFrame,
                          s_v4l2->capstride,
                          v4l2_sg_get_vaddr(vbuf),
                          vch->stride,
                          bottom_up);
            }

            ret = 0;

            capture_channel_ReleaseVideoCaptureFrame(vch->capch);
        }
    }

    vbuf->v4l2_buf.field = V4L2_FIELD_NONE;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0))
    {
        struct timespec ts;

        ktime_get_ts(&ts);
        vbuf->v4l2_buf.timestamp.tv_sec = ts.tv_sec;
        vbuf->v4l2_buf.timestamp.tv_usec = ts.tv_nsec / NSEC_PER_USEC;
    }
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5,1,0))
    v4l2_get_timestamp(&vbuf->v4l2_buf.timestamp);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0))
    vbuf->v4l2_buf.timestamp = ns_to_timeval(ktime_get_ns());
#else
    v4l2_buffer_set_timestamp(&vbuf->v4l2_buf, ktime_get_ns());
#endif
    vbuf->v4l2_buf.sequence = s_v4l2->sequence;
    if (ret == 0) {
        vbuf->state = V4L2_SG_BUF_STATE_DONE;
    } else { /* timeout */
        vbuf->state = V4L2_SG_BUF_STATE_ERROR;
    }

    v4l2_sg_queue_put_donebuf(&pipe->s_v4l2.vsq, vbuf);

    return;
}

static int mw_v4l2_thread_proc(void *data)
{
    struct mw_stream_pipe *pipe = data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    long long lltime;
    long ret;

    MWCAP_VIDEO_SIGNAL_STATUS signal_status;
    bool low_latency_mode = false;

    os_event_t wait_events[] = {
        s_v4l2->timer->event,
        s_v4l2->notify->event
    };

    mw_debug(1, "v4l2 thread started\n");

    set_freezable();

    notify_event_mgr_get_status_bits(pipe->vch->m_pNotifyEventManager, s_v4l2->notify);

    capture_channel_GetVideoSignalStatus(pipe->vch->capch, &signal_status);
    low_latency_mode = enable_raw_mode && signal_status.state == MWCAP_VIDEO_SIGNAL_LOCKED;

    lltime = s_v4l2->llstart_time + s_v4l2->frame_duration;
    timer_event_mgr_ScheduleTimer(pipe->vch->m_pTimerEventManager, lltime, s_v4l2->timer);

    for (;;) {
        ret = os_event_wait_for_multiple(
                    wait_events, ARRAY_SIZE(wait_events), 1000);
        if (s_v4l2->exit_capture_thread)
            break;

        if (ret == 0 || ret == -ERESTARTSYS) {
            mw_debug(0, "os_event_wait_for_multiple ret=%ld\n", ret);
            continue;
        }

        if (os_event_try_wait(s_v4l2->notify->event)) {
            u64 status_bits = notify_event_mgr_get_status_bits(pipe->vch->m_pNotifyEventManager, s_v4l2->notify);

            if (status_bits & MWCAP_NOTIFY_VIDEO_SIGNAL_CHANGE) {
                capture_channel_GetVideoSignalStatus(pipe->vch->capch, &signal_status);
                low_latency_mode = enable_raw_mode && signal_status.state == MWCAP_VIDEO_SIGNAL_LOCKED;
            }

            if (low_latency_mode && (status_bits & MWCAP_NOTIFY_VIDEO_FRAME_BUFFERED))
                v4l2_process_one_frame(pipe, -1);
        }

        if (os_event_try_wait(s_v4l2->timer->event)) {
            if (!low_latency_mode)
                v4l2_process_one_frame(pipe, -1);

            s_v4l2->sequence++;
            lltime += s_v4l2->frame_duration;
            timer_event_mgr_ScheduleTimer(pipe->vch->m_pTimerEventManager, lltime, s_v4l2->timer);
        }

        try_to_freeze();
    }

    timer_event_mgr_CancelTimer(pipe->vch->m_pTimerEventManager, s_v4l2->timer);

    /* sync with kthread_stop */
    for (;;) {
        if (kthread_should_stop())
            break;
        msleep(1);
    }

    mw_debug(1, "v4l2 thread: exit\n");
    return 0;
}

static int mw_v4l2_stop_thread(struct mw_stream_pipe *pipe)
{
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;

    s_v4l2->exit_capture_thread = true;
    if (s_v4l2->kthread) {
        timer_event_mgr_CancelTimer(pipe->vch->m_pTimerEventManager, s_v4l2->timer);
        os_event_set(s_v4l2->timer->event);
        kthread_stop(s_v4l2->kthread);
        s_v4l2->kthread = NULL;
    }

    if (s_v4l2->timer != NULL) {
        timer_event_mgr_DeleteTimer(pipe->vch->m_pTimerEventManager, s_v4l2->timer);
        s_v4l2->timer = NULL;
    }

    capture_channel_ReleaseVideoCapture(pipe->vch->capch);

    os_spin_lock(pipe->vch->format_lock);
    pipe->vch->ch_ref--;
    os_spin_unlock(pipe->vch->format_lock);

    return 0;
}

static int mw_v4l2_start_thread(struct mw_stream_pipe *pipe)
{
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    struct mw_v4l2_channel *vch = pipe->vch;
    int ret = -1;

    os_spin_lock(vch->format_lock);
    if (vch->ch_ref > 0) {
        if (vch->fmt != s_v4l2->fmt
            || vch->width != s_v4l2->width
            || vch->height != s_v4l2->height
            || vch->stride != s_v4l2->stride
            || vch->image_size != s_v4l2->image_size
            ) {
            os_spin_unlock(vch->format_lock);
            return -EBUSY;
        }
    } else {
        vch->fmt = s_v4l2->fmt;
        vch->width = s_v4l2->width;
        vch->height = s_v4l2->height;
        vch->stride = s_v4l2->stride;
        vch->image_size = s_v4l2->image_size;
    }
    vch->ch_ref++;
    os_spin_unlock(vch->format_lock);

    ret = capture_channel_AcquireVideoCapture(
                vch->capch,
                vch->width,
                vch->height,
                fourcc_v4l2_to_mwcap(vch->fmt->fourcc),
                &s_v4l2->capstride
                );
    if (ret != OS_RETURN_SUCCESS) {
        os_spin_lock(vch->format_lock);
        vch->ch_ref--;
        os_spin_unlock(vch->format_lock);
        return ret;
    }

    s_v4l2->timer = timer_event_mgr_NewTimer(vch->m_pTimerEventManager, NULL);
    if (s_v4l2->timer == NULL) {
        ret = -EFAULT;
        goto err;
    }

    os_event_clear(s_v4l2->timer->event);
    s_v4l2->llstart_time = capture_channel_GetTime(vch->capch);

    s_v4l2->notify = notify_event_mgr_add(vch->m_pNotifyEventManager,
                                          MWCAP_NOTIFY_VIDEO_SIGNAL_CHANGE
                                          | MWCAP_NOTIFY_VIDEO_FRAME_BUFFERED,
                                          NULL
                                          );
    if (s_v4l2->notify == NULL) {
        ret = -ENOMEM;
        goto err;
    }

    s_v4l2->sequence = 0;
    s_v4l2->exit_capture_thread = false;
    s_v4l2->kthread = kthread_run(mw_v4l2_thread_proc, pipe, "MW-V4L2");
    if (IS_ERR(s_v4l2->kthread)) {
        mw_debug(0, "Create kernel thread MW-V4L2 failed!\n");
        ret = PTR_ERR(s_v4l2->kthread);
        s_v4l2->kthread = NULL;
        goto err;
    }

    return 0;

err:
    mw_v4l2_stop_thread(pipe);
    return ret;
}

static int mw_v4l2_start_generating(struct mw_stream_pipe *pipe)
{
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    int ret = -1;

    mw_debug(1, "entering function %s\n", __func__);

    if (test_and_set_bit(0, &s_v4l2->generating))
        return -EBUSY;

    ret = v4l2_sg_queue_streamon(&s_v4l2->vsq);
    if (ret != 0) {
        return ret;
    }

    ret = mw_v4l2_start_thread(pipe);
    if (ret != 0) {
        goto thread_err;
    }

    mw_debug(1, "returning from %s\n", __func__);
    return 0;

thread_err:
    v4l2_sg_queue_streamoff(&s_v4l2->vsq);

    clear_bit(0, &s_v4l2->generating);
    return ret;
}

static void mw_v4l2_stop_generating(struct mw_stream_pipe *pipe)
{
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;

    mw_debug(1, "entering function %s\n", __func__);

    if (!test_and_clear_bit(0, &s_v4l2->generating))
        return;

    mw_v4l2_stop_thread(pipe);

    v4l2_sg_queue_streamoff(&s_v4l2->vsq);
}

/* ------------------------------------------------------------------
   IOCTL vidioc handling
   ------------------------------------------------------------------*/
static int vidioc_querycap(struct file *file, void  *priv, struct v4l2_capability *cap)
{
    struct mw_v4l2_channel *vch = video_drvdata(file);

    mw_debug(5, "entering function %s\n", __func__);

    strcpy(cap->driver, VIDEO_CAP_DRIVER_NAME);
    if (capture_channel_GetChannelName(vch->capch, cap->card, sizeof(cap->card)) <= 0) {
        strcpy(cap->card, VIDEO_CAP_DRIVER_NAME);
    }
    sprintf(cap->bus_info, "PCI:%s", dev_name(vch->pci_dev));

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0))
    cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
#else
    /* V4L2_CAP_DEVICE_CAPS must be set since linux kernle 3.17 */
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
#endif

    return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv, struct v4l2_fmtdesc *f)
{
    struct v4l2_fmt *fmt;
    mw_debug(5, "entering function %s\n", __func__);

    if (f->index >= ARRAY_SIZE(g_formats))
        return -EINVAL;

    fmt = &g_formats[f->index];

    strlcpy(f->description, fmt->name, sizeof(f->description));
    f->pixelformat = fmt->fourcc;

    return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;

    mw_debug(5, "entering function %s\n", __func__);

    f->fmt.pix.width        = s_v4l2->width;
    f->fmt.pix.height       = s_v4l2->height;
    f->fmt.pix.field        = s_v4l2->vsq.field;
    f->fmt.pix.pixelformat  = s_v4l2->fmt->fourcc;
    f->fmt.pix.bytesperline = s_v4l2->stride;
    f->fmt.pix.sizeimage    = s_v4l2->image_size;

    return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct mw_v4l2_channel *vch = video_drvdata(file);
    struct v4l2_fmt *fmt;
    enum v4l2_field field;
    MWCAP_VIDEO_CAPS video_caps;

    capture_channel_GetVideoCaps(vch->capch, &video_caps);

    mw_debug(5, "entering function %s\n", __func__);

    fmt = get_format(f);
    if (!fmt) {
        mw_debug(1, "Fourcc format (0x%08x) invalid.\n",
                 f->fmt.pix.pixelformat);
        return -EINVAL;
    }

    field = f->fmt.pix.field;

    if (field == V4L2_FIELD_ANY) {
        field = V4L2_FIELD_NONE;
    } else if (V4L2_FIELD_NONE != field) {
        mw_debug(1, "Field type invalid.\n");
        return -EINVAL;
    }

    f->fmt.pix.field = field;
    /* @walign @halign 2^align */
    if (fmt->fourcc == V4L2_PIX_FMT_YUYV
        || fmt->fourcc == V4L2_PIX_FMT_UYVY) {
        v4l_bound_align_image(&f->fmt.pix.width, MIN_WIDTH, video_caps.wMaxOutputWidth, 1,
                              &f->fmt.pix.height, MIN_HEIGHT, video_caps.wMaxOutputHeight, 0, 0);
    } else if (fmt->fourcc == V4L2_PIX_FMT_NV12
               || fmt->fourcc == V4L2_PIX_FMT_YVU420
               || fmt->fourcc == V4L2_PIX_FMT_YUV420) {
        v4l_bound_align_image(&f->fmt.pix.width, MIN_WIDTH, video_caps.wMaxOutputWidth, 1,
                              &f->fmt.pix.height, MIN_HEIGHT, video_caps.wMaxOutputHeight, 1, 0);
    } else {
        v4l_bound_align_image(&f->fmt.pix.width, MIN_WIDTH, video_caps.wMaxOutputWidth, 0,
                              &f->fmt.pix.height, MIN_HEIGHT, video_caps.wMaxOutputHeight, 0, 0);
    }

    /*
     * bytesperline不能接收用户设置的值，因为有些软件会传入不正确的值(如：mplayer的bytesperline
     * 是v4l2 G_FMT得到的，与现在要设置的分辨率不符).
     */
    if (fmt->fourcc == V4L2_PIX_FMT_NV12 ||
        fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
        fmt->fourcc == V4L2_PIX_FMT_YUV420) {
        f->fmt.pix.bytesperline = f->fmt.pix.width;

        f->fmt.pix.sizeimage =
                (f->fmt.pix.height * f->fmt.pix.bytesperline * fmt->depth + 7) >> 3;
    } else { // packed
        f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth + 7) >> 3;

        f->fmt.pix.sizeimage = f->fmt.pix.height * f->fmt.pix.bytesperline;
    }

    return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct mw_v4l2_channel *vch = video_drvdata(file);
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    int ret = 0;

    mw_debug(5, "entering function %s\n", __func__);

    if (v4l2_is_generating(s_v4l2)) {
        mw_debug(1, "%s device busy\n", __func__);
        return -EBUSY;
    }

    ret = vidioc_try_fmt_vid_cap(file, priv, f);
    if (ret < 0)
        return ret;

    os_spin_lock(vch->format_lock);
    if (vch->ch_ref > 0) {
        if (vch->fmt != get_format(f)
            || vch->width != f->fmt.pix.width
            || vch->height != f->fmt.pix.height
            || vch->stride != f->fmt.pix.bytesperline
            || vch->image_size != f->fmt.pix.sizeimage
            ) {
            os_spin_unlock(vch->format_lock);
            return -EBUSY;
        }
    } else {
        s_v4l2->fmt = get_format(f);
        s_v4l2->width = f->fmt.pix.width;
        s_v4l2->height = f->fmt.pix.height;
        s_v4l2->vsq.field = f->fmt.pix.field;
        s_v4l2->stride = f->fmt.pix.bytesperline;
        s_v4l2->image_size = f->fmt.pix.sizeimage;
    }
    os_spin_unlock(vch->format_lock);

    mw_info("ProCapture: Set v4l2 format to %dx%d(%s)\n",
           f->fmt.pix.width, f->fmt.pix.height,
           vch->fmt->name);

    return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *p)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct v4l2_sg_buf_queue *vsq = &pipe->s_v4l2.vsq;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;

    return v4l2_sg_queue_reqbufs(vsq, p, s_v4l2->image_size);
}

/*
 * user get the video buffer status
 * for mmap, get the map offset
 * */
static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct v4l2_sg_buf_queue *vsq = &pipe->s_v4l2.vsq;

    return v4l2_sg_queue_querybuf(vsq, p);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct v4l2_sg_buf_queue *vsq = &pipe->s_v4l2.vsq;

    return v4l2_sg_queue_qbuf(vsq, p);
}

/*
 * check if have available frame, if have no frame return <0;
 * if block, it will wait until there is frame available, (vb->done event).
 * if have frame, return 0 and transfer the buffer info to user.
 * */
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct v4l2_sg_buf_queue *vsq = &pipe->s_v4l2.vsq;

    return v4l2_sg_queue_dqbuf(vsq, p, file->f_flags & O_NONBLOCK);
}

static int vidioc_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
    struct mw_stream_pipe *pipe = file->private_data;
    const u32 *data;
    int count = 0;
    const char *name = "Auto";
    MWCAP_VIDEO_INPUT_TYPE input_type;
    typeof(inp->index) __index = inp->index;

    mw_debug(2, "entering function %s\n", __func__);

    memset(inp, 0, sizeof(*inp));
    inp->index = __index;

    data = capture_channel_GetSupportedVideoInputSources(pipe->vch->capch, &count);

    if (inp->index > count || inp->index > 6/* input type number */)
        return -EINVAL;

    inp->type = V4L2_INPUT_TYPE_CAMERA;
    if (inp->index == 0)
        input_type = MWCAP_VIDEO_INPUT_TYPE_NONE;
    else
        input_type = INPUT_TYPE(data[inp->index-1]);

    switch (input_type) {
    case MWCAP_VIDEO_INPUT_TYPE_NONE:
        name = "Auto";
        break;
    case MWCAP_VIDEO_INPUT_TYPE_HDMI:
        name = "HDMI";
        break;
    case MWCAP_VIDEO_INPUT_TYPE_VGA:
        name = "VGA";
        break;
    case MWCAP_VIDEO_INPUT_TYPE_SDI:
        name = "SDI";
        break;
    case MWCAP_VIDEO_INPUT_TYPE_COMPONENT:
        name = "COMPONENT";
        break;
    case MWCAP_VIDEO_INPUT_TYPE_CVBS:
        name = "CVBS";
        break;
    case MWCAP_VIDEO_INPUT_TYPE_YC:
        name = "YC";
        break;
    }
    sprintf(inp->name, "%s", name);

    return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
    struct mw_stream_pipe *pipe = file->private_data;

    const u32 *data;
    int count = 0;
    u32 current_input;
    int index;

    mw_debug(2, "entering function %s\n", __func__);

    *i = 0;
    if (capture_channel_GetInputSourceScanState(pipe->vch->capch)) {
        return 0;
    }

    data = capture_channel_GetSupportedVideoInputSources(pipe->vch->capch, &count);

    current_input = capture_channel_GetVideoInputSource(pipe->vch->capch);

    for (index = 0; index < count; index++) {
        if (data[index] == current_input) {
            *i = index + 1;
            break;
        }
    }

    return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
    struct mw_stream_pipe *pipe = file->private_data;

    const u32 *data;
    int count = 0;

    mw_debug(2, "entering function %s\n", __func__);

    data = capture_channel_GetSupportedVideoInputSources(pipe->vch->capch, &count);
    if (i > count)
        return -EINVAL;

    if (i == 0) {
        capture_channel_SetInputSourceScan(pipe->vch->capch, true);
        return 0;
    }

    capture_channel_SetInputSourceScan(pipe->vch->capch, false);
    capture_channel_SetVideoInputSource(pipe->vch->capch, data[i-1]);

    return 0;
}

static int vidioc_g_parm(struct file *file, void *__fh, struct v4l2_streamparm *sp)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    struct v4l2_captureparm *cp = &sp->parm.capture;

    cp->capability = V4L2_CAP_TIMEPERFRAME;
    cp->timeperframe.numerator = s_v4l2->frame_duration;
    cp->timeperframe.denominator = 10000000;

    return 0;
}

static int vidioc_s_parm(struct file *file, void *__fh, struct v4l2_streamparm *sp)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    struct v4l2_captureparm *cp = &sp->parm.capture;

    if (v4l2_is_generating(s_v4l2))
        return -EBUSY;

    if ((cp->timeperframe.numerator == 0) ||
        (cp->timeperframe.denominator == 0)) {
        /* reset framerate */
        s_v4l2->frame_duration = 333333;
    } else {
        s_v4l2->frame_duration = (unsigned int)div_u64(10000000LL * cp->timeperframe.numerator,
                                                       cp->timeperframe.denominator);
    }

    return 0;
}

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
    struct mw_stream_pipe *pipe = file->private_data;

    mw_debug(2, "entering function %s\n", __func__);

    if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    return mw_v4l2_start_generating(pipe);
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
    struct mw_stream_pipe *pipe = file->private_data;

    mw_debug(2, "entering function %s\n", __func__);

    if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    /* first kill thread, waiting for the delete queued buf */
    mw_v4l2_stop_generating(pipe);

    return 0;
}

/* --- controls ---------------------------------------------- */
static int vidioc_queryctrl(struct file *file, void *priv,
                            struct v4l2_queryctrl *qc)
{
    switch (qc->id) {
    case V4L2_CID_BRIGHTNESS:
        return v4l2_ctrl_query_fill(qc, -100, 100, 1, 0);
    case V4L2_CID_CONTRAST:
        return v4l2_ctrl_query_fill(qc, 50, 200, 1, 100);
    case V4L2_CID_SATURATION:
        return v4l2_ctrl_query_fill(qc, 0, 200, 1, 100);
    case V4L2_CID_HUE:
        return v4l2_ctrl_query_fill(qc, -90, 90, 1, 0);
    }
    return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
    struct mw_stream_pipe *pipe = file->private_data;
    MWCAP_VIDEO_ECO_CAPTURE_SETTINGS videoSettings;
    int ret = 0;

    os_spin_lock(pipe->vch->mw_dev.settings_lock);
    capture_channel_GetVideoSettings(pipe->vch->capch, &videoSettings);
    os_spin_unlock(pipe->vch->mw_dev.settings_lock);

    switch (ctrl->id) {
    case V4L2_CID_BRIGHTNESS:
        ctrl->value = videoSettings.sBrightness;
        break;
    case V4L2_CID_CONTRAST:
        ctrl->value = videoSettings.sContrast;
        break;
    case V4L2_CID_SATURATION:
        ctrl->value = videoSettings.sSaturation;
        break;
    case V4L2_CID_HUE:
        ctrl->value = videoSettings.sHue;
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int vidioc_s_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct v4l2_queryctrl qc;
    int err;
    int ret = 0;
    MWCAP_VIDEO_ECO_CAPTURE_SETTINGS videoSettings;

    qc.id = ctrl->id;
    err = vidioc_queryctrl(file, priv, &qc);
    if (err < 0)
        return err;
    if (ctrl->value < qc.minimum || ctrl->value > qc.maximum)
        return -ERANGE;

    os_spin_lock(pipe->vch->mw_dev.settings_lock);

    capture_channel_GetVideoSettings(pipe->vch->capch, &videoSettings);

    switch (ctrl->id) {
    case V4L2_CID_BRIGHTNESS:
        videoSettings.sBrightness = _limit(ctrl->value, -100, 100);
        break;
    case V4L2_CID_CONTRAST:
        videoSettings.sContrast = _limit(ctrl->value, 50, 200);
        break;
    case V4L2_CID_SATURATION:
        videoSettings.sSaturation = _limit(ctrl->value, 0, 200);
        break;
    case V4L2_CID_HUE:
        videoSettings.sHue = _limit(ctrl->value, -90, 90);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    capture_channel_SetVideoSettings(pipe->vch->capch, &videoSettings);

    os_spin_unlock(pipe->vch->mw_dev.settings_lock);

    return ret;
}

static struct v4l2_frmsize_discrete g_frmsize_array[] = {
    { 640, 360 },
    { 640, 480 },
    { 720, 480 },
    { 720, 576 },
    { 768, 576 },
    { 800, 600 },
    { 856, 480 },
    { 960, 540 },
    { 1024, 576 },
    { 1024, 768 },
    { 1280, 720 },
    { 1280, 800 },
    { 1280, 960 },
    { 1280, 1024 },
    { 1368, 768 },
    { 1440, 900 },
    { 1600, 1200 },
    { 1920, 1080 },
    { 1920, 1200 },
    { 2048, 1536 },

    { 2560, 1440 },
    { 3072, 768 },
    { 3840, 1024 },
    { 3840, 2160 },
    { 4096, 2160 },
};

extern int enum_framesizes_type;
static int vidioc_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
    int i;
    MWCAP_VIDEO_CAPS video_caps;
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_v4l2_channel *vch = pipe->vch;

    capture_channel_GetVideoCaps(vch->capch, &video_caps);

    for (i = 0; i < ARRAY_SIZE(g_formats); i++)
        if (g_formats[i].fourcc == fsize->pixel_format)
            break;
    if (i == ARRAY_SIZE(g_formats))
        return -EINVAL;

    switch (enum_framesizes_type) {
    case 1: /* V4L2_FRMSIZE_TYPE_DISCRETE */
        if (fsize->index < 0 || fsize->index >= ARRAY_SIZE(g_frmsize_array))
            return -EINVAL;
        if (g_frmsize_array[fsize->index].width > video_caps.wMaxOutputWidth
            || g_frmsize_array[fsize->index].height > video_caps.wMaxOutputHeight)
            return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
        fsize->discrete.width = g_frmsize_array[fsize->index].width;
        fsize->discrete.height = g_frmsize_array[fsize->index].height;
        break;
    case 2: /* V4L2_FRMSIZE_TYPE_STEPWISE */
        if (fsize->index != 0)
            return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
        fsize->stepwise.min_width = MIN_WIDTH;
        fsize->stepwise.min_height = MIN_HEIGHT;
        fsize->stepwise.max_width = video_caps.wMaxOutputWidth;
        fsize->stepwise.max_height = video_caps.wMaxOutputHeight;
        fsize->stepwise.step_width = 1;
        fsize->stepwise.step_height = 1;

        /* @walign @halign 2^align */
        if (fsize->pixel_format == V4L2_PIX_FMT_YUYV
            || fsize->pixel_format == V4L2_PIX_FMT_UYVY) {
            fsize->stepwise.step_width = 2;
            fsize->stepwise.step_height = 1;
        } else if (fsize->pixel_format == V4L2_PIX_FMT_NV12
                   || fsize->pixel_format == V4L2_PIX_FMT_YVU420
                   || fsize->pixel_format == V4L2_PIX_FMT_YUV420) {
            fsize->stepwise.step_width = 2;
            fsize->stepwise.step_height = 2;
        }
        break;
    default: /* V4L2_FRMSIZE_TYPE_CONTINUOUS */
        if (fsize->index != 0)
            return -EINVAL;
        fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
        fsize->stepwise.min_width = MIN_WIDTH;
        fsize->stepwise.min_height = MIN_HEIGHT;
        fsize->stepwise.max_width = video_caps.wMaxOutputWidth;
        fsize->stepwise.max_height = video_caps.wMaxOutputHeight;
        fsize->stepwise.step_width = 1;
        fsize->stepwise.step_height = 1;
        break;
    }

    return 0;
}

extern unsigned int enum_frameinterval_min;
static int vidioc_enum_frameintervals(struct file *file, void *fh, struct v4l2_frmivalenum *fival)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_v4l2_channel *vch = pipe->vch;
    MWCAP_VIDEO_CAPS video_caps;
    int i;

    if (fival->index != 0)
        return -EINVAL;

    for (i = 0; i < ARRAY_SIZE(g_formats); i++)
        if (g_formats[i].fourcc == fival->pixel_format)
            break;
    if (i == ARRAY_SIZE(g_formats))
        return -EINVAL;

    capture_channel_GetVideoCaps(vch->capch, &video_caps);

    switch (enum_framesizes_type) {
    case 1: /* V4L2_FRMSIZE_TYPE_DISCRETE */
        for (i = 0; i < ARRAY_SIZE(g_frmsize_array); i++)
            if (fival->width == g_frmsize_array[i].width
                && fival->height == g_frmsize_array[i].height)
                break;
        if (i == ARRAY_SIZE(g_frmsize_array))
            return -EINVAL;
        break;
    default: /* V4L2_FRMSIZE_TYPE_CONTINUOUS */
        if (fival->width < MIN_WIDTH
            || fival->width > video_caps.wMaxOutputWidth
            || fival->height < MIN_HEIGHT
            || fival->height > video_caps.wMaxOutputHeight)
            return -EINVAL;
        break;
    }

    fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
    fival->stepwise.min.denominator = 10000000;
    fival->stepwise.min.numerator =
            enum_frameinterval_min > 1000000 ? 1000000 : enum_frameinterval_min;
    fival->stepwise.max.denominator = 10000000;
    fival->stepwise.max.numerator = 10000000;
    fival->stepwise.step.denominator = 1;
    fival->stepwise.step.numerator = 1;

    return 0;
}

static int _stream_v4l2_init(struct mw_stream_pipe *pipe)
{
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    struct mw_v4l2_channel *vch = pipe->vch;

    mutex_init(&s_v4l2->v4l2_mutex);

    v4l2_sg_queue_init(&s_v4l2->vsq,
                       V4L2_BUF_TYPE_VIDEO_CAPTURE,
                       &s_v4l2->v4l2_mutex, // will unlock when dqbuf is waiting
                       vch->pci_dev,
                       V4L2_FIELD_NONE
                       );

    /* init video dma queues */
    INIT_LIST_HEAD(&s_v4l2->active);

    os_spin_lock(vch->format_lock);
    s_v4l2->fmt = vch->fmt;
    s_v4l2->width = vch->width;
    s_v4l2->height = vch->height;
    s_v4l2->stride = vch->stride;
    s_v4l2->image_size = vch->image_size;
    s_v4l2->capstride = 0;
    s_v4l2->frame_duration = g_def_frame_duration;
    os_spin_unlock(vch->format_lock);

    return 0;
}

/* ------------------------------------------------------------------
   File operations for the device
   ------------------------------------------------------------------*/
static int mw_v4l2_open(struct file *file)
{
    struct mw_v4l2_channel *vch = video_drvdata(file);
    struct mw_stream_pipe *pipe;
    int ret;

    mw_debug(5, "entering function %s\n", __func__);

    if (file->private_data != NULL)
        return -EFAULT;

    pipe = kzalloc(sizeof(*pipe), GFP_KERNEL);
    if (pipe == NULL)
        return -ENOMEM;

    pipe->vch = vch;

    init_rwsem(&pipe->io_sem);

    os_spin_lock(vch->format_lock);
    if (vch->ch_ref == 0) {
        vch->fmt = g_def_fmt;
        vch->width = g_def_width;
        vch->height = g_def_height;
        if (vch->fmt->fourcc == V4L2_PIX_FMT_NV12 ||
            vch->fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
            vch->fmt->fourcc == V4L2_PIX_FMT_YUV420) {
            vch->stride = vch->width;
            vch->image_size = (vch->width * vch->height * vch->fmt->depth + 7) >> 3;
        } else {
            vch->stride = (vch->width * vch->fmt->depth + 7) >> 3;
            vch->image_size = vch->stride * vch->height;
        }
    }
    os_spin_unlock(vch->format_lock);

    _stream_v4l2_init(pipe);

    ret = mw_stream_init(&pipe->s_mw, &pipe->vch->mw_dev);
    if (ret != 0) {
        kfree(pipe);
        return ret;
    }

    file->private_data = pipe;

    return 0;
}

static ssize_t
mw_v4l2_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
    mw_debug(1, VIDEO_CAP_DRIVER_NAME" capture card not support read!\n");

    return -1;
}

static unsigned int
mw_v4l2_poll(struct file *file, struct poll_table_struct *wait)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    unsigned int mask = 0;

    mw_debug(20, "entering function %s\n", __func__);

    down_read(&pipe->io_sem);

    mutex_lock(&s_v4l2->v4l2_mutex);
    if (v4l2_is_generating(s_v4l2))
        mask |= v4l2_sg_queue_poll(file, &s_v4l2->vsq, wait);
    mutex_unlock(&s_v4l2->v4l2_mutex);

    up_read(&pipe->io_sem);

    return mask;
}

static int mw_v4l2_close(struct file *file)
{
    struct video_device  *vdev = video_devdata(file);
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;

    mw_debug(5, "entering function %s\n", __func__);

    down_write(&pipe->io_sem);

    mutex_lock(&s_v4l2->v4l2_mutex);
    mw_v4l2_stop_generating(pipe);
    mutex_unlock(&s_v4l2->v4l2_mutex);

    mw_stream_deinit(&pipe->s_mw);

    v4l2_sg_queue_deinit(&s_v4l2->vsq);

    up_write(&pipe->io_sem);

    kfree(pipe);

    file->private_data = NULL;

    mw_debug(5, "close called (dev=%s)\n", video_device_node_name(vdev));
    return 0;
}

static int mw_v4l2_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct mw_stream_pipe *pipe = file->private_data;
    int ret = 0;

    mw_debug(5, "entering function %s\n", __func__);

    down_read(&pipe->io_sem);
    ret = v4l2_sg_buf_mmap(&pipe->s_v4l2.vsq, vma);
    up_read(&pipe->io_sem);

    return ret;
}

static long _mw_stream_ioctl(struct file *file, unsigned int cmd, void *arg)
{
    struct mw_stream_pipe *pipe = file->private_data;
    long ret = 0;

    ret = mw_stream_ioctl(&pipe->s_mw, cmd, arg);

    return ret;
}

typedef long (*__args_kioctl)(struct file *file, unsigned int cmd, void *arg);
static long __args_usercopy(struct file *file, unsigned int cmd, unsigned long arg,
                            __args_kioctl func)
{
    char    sbuf[128];
    void    *mbuf = NULL;
    void    *parg = (void *)arg;
    long    err  = -EINVAL;

    /*  Copy arguments into temp kernel buffer  */
    if (_IOC_DIR(cmd) != _IOC_NONE) {
        if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
            parg = sbuf;
        } else {
            /* too big to allocate from stack */
            mbuf = kmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
            if (NULL == mbuf)
                return -ENOMEM;
            parg = mbuf;
        }

        err = -EFAULT;
        if (_IOC_DIR(cmd) & _IOC_WRITE) {
            unsigned int n = _IOC_SIZE(cmd);

            if (copy_from_user(parg, (void __user *)arg, n))
                goto out;

            /* zero out anything we don't copy from userspace */
            if (n < _IOC_SIZE(cmd))
                memset((u8 *)parg + n, 0, _IOC_SIZE(cmd) - n);
        } else {
            /* read-only ioctl */
            memset(parg, 0, _IOC_SIZE(cmd));
        }
    }

    /* Handles IOCTL */
    err = func(file, cmd, parg);
    if (err == -ENOIOCTLCMD)
        err = -ENOTTY;

    if (err < 0)
        goto out;

    /*  Copy results into user buffer  */
    switch (_IOC_DIR(cmd)) {
    case _IOC_READ:
    case (_IOC_WRITE | _IOC_READ):
        if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd)))
            err = -EFAULT;
        break;
    }

out:
    kfree(mbuf);
    return err;
}

static long mw_v4l2_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct mw_stream_pipe *pipe = file->private_data;
    struct mw_stream_v4l2 *s_v4l2 = &pipe->s_v4l2;
    long ret;

    down_read(&pipe->io_sem);

    if (_IOC_TYPE(cmd) == _IOC_TYPE(VIDIOC_QUERYCAP)) {
        mutex_lock(&s_v4l2->v4l2_mutex);
        ret = video_ioctl2(file, cmd, arg);
        mutex_unlock(&s_v4l2->v4l2_mutex);
    } else {
        ret = __args_usercopy(file, cmd, arg, _mw_stream_ioctl);
    }

    /* the return value of ioctl for user is int  */
    if (ret > INT_MAX) {
        ret = 1;
    }
    up_read(&pipe->io_sem);

    return ret;
}

#ifdef CONFIG_COMPAT
static long mw_v4l2_compat_ioctl32(struct file *file, unsigned int cmd,
                                   unsigned long arg)
{
    long ret;

    ret = mw_v4l2_ioctl(file, cmd, arg);

    return ret;
}
#endif

static const struct v4l2_file_operations mw_v4l2_fops = {
    .owner          = THIS_MODULE,
    .open           = mw_v4l2_open,
    .read           = mw_v4l2_read,
    .release        = mw_v4l2_close,
    .poll           = mw_v4l2_poll,
    .unlocked_ioctl = mw_v4l2_ioctl,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
#ifdef CONFIG_COMPAT
    .compat_ioctl32 = mw_v4l2_compat_ioctl32,
#endif
#endif
    .mmap           = mw_v4l2_mmap,
};

static const struct v4l2_ioctl_ops mw_ioctl_ops = {
    .vidioc_querycap            = vidioc_querycap,
    .vidioc_enum_fmt_vid_cap    = vidioc_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap       = vidioc_g_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap     = vidioc_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap       = vidioc_s_fmt_vid_cap,
    .vidioc_reqbufs             = vidioc_reqbufs,
    .vidioc_querybuf            = vidioc_querybuf,
    .vidioc_qbuf                = vidioc_qbuf,
    .vidioc_dqbuf               = vidioc_dqbuf,
    .vidioc_enum_input          = vidioc_enum_input,
    .vidioc_g_input             = vidioc_g_input,
    .vidioc_s_input             = vidioc_s_input,
    .vidioc_g_parm              = vidioc_g_parm,
    .vidioc_s_parm              = vidioc_s_parm,
    .vidioc_streamon            = vidioc_streamon,
    .vidioc_streamoff           = vidioc_streamoff,
    .vidioc_queryctrl           = vidioc_queryctrl,
    .vidioc_g_ctrl              = vidioc_g_ctrl,
    .vidioc_s_ctrl              = vidioc_s_ctrl,
    .vidioc_enum_framesizes     = vidioc_enum_framesizes,
    .vidioc_enum_frameintervals = vidioc_enum_frameintervals,
};

static struct video_device mw_video_template = {
    .name           = VIDEO_CAP_DRIVER_NAME,
    /*
     * The device_caps was added in version 4.7 .
     * And has been checked since 5.4 in drivers/media/v4l2-core/v4l2-dev.c:863
     */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0))
    .device_caps    = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
#endif
    .fops           = &mw_v4l2_fops,
    .ioctl_ops      = &mw_ioctl_ops,
    .release        = video_device_release,
};

int mw_v4l2_init(struct mw_v4l2_dev *dev, struct device *pci_dev)
{
    int ret;

    dev->pci_dev = pci_dev;

    INIT_LIST_HEAD(&dev->chnl_list);

    snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
             "%s", VIDEO_CAP_DRIVER_NAME);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
    /*
     * v4l2_device_register @dev must be set to NULL
     * kernel version < 2.6.39, it will overwrite the parent_dev's drvdata:
     *
     *  if (dev_get_drvdata(dev))
     *      v4l2_warn(v4l2_dev, "Non-NULL drvdata on register\n");
     *  dev_set_drvdata(dev, v4l2_dev);
     *
     * */
    ret = v4l2_device_register(NULL, &dev->v4l2_dev);
    if (ret != 0)
        return ret;
    dev->v4l2_dev.dev = pci_dev;
#else
    ret = v4l2_device_register(pci_dev, &dev->v4l2_dev);
    if (ret != 0)
        return ret;
#endif

    return 0;
}

int mw_v4l2_register_channel(struct mw_v4l2_dev *dev, struct capture_channel *capch)
{
    struct mw_v4l2_channel *vch;
    struct video_device *vfd;
    int ret;

    vch = kzalloc(sizeof(*vch), GFP_KERNEL);
    if (vch == NULL)
        return -ENOMEM;

    vch->capch = capch;
    vch->pci_dev = dev->pci_dev;

    vch->m_pTimerEventManager = capture_channel_GetTimerEventManager(capch);
    vch->m_pNotifyEventManager = capture_channel_GetNotifyEventManager(capch);

    vch->ch_ref = 0;
    vch->format_lock = os_spin_lock_alloc();
    if (vch->format_lock == NULL) {
        ret = -ENOMEM;
        goto lock_alloc_err;
    }

    vch->mw_dev.capch = capch;
    vch->mw_dev.supported_multi_stream = capture_channel_IsSupportedMultiStream(capch);
    vch->mw_dev.dma_priv = dev->pci_dev;

    ret = mw_device_init(&vch->mw_dev);
    if (ret != OS_RETURN_SUCCESS)
        goto mw_dev_err;

    ret = -ENOMEM;
    vfd = video_device_alloc();
    if (!vfd)
        goto vdev_alloc_err;

    *vfd = mw_video_template;
    if (capture_channel_GetChannelName(capch, vfd->name, sizeof(vfd->name)) <= 0)
        snprintf(vfd->name, sizeof(vfd->name), "%s", VIDEO_CAP_DRIVER_NAME);

    vfd->v4l2_dev = &dev->v4l2_dev;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,7,0))
    ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
#else
    ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
#endif
    if (ret < 0)
        goto reg_vdev_err;

    vch->vfd = vfd;
    video_set_drvdata(vfd, vch);
    list_add_tail(&vch->chnl_node, &dev->chnl_list);

    v4l2_info(&dev->v4l2_dev, "V4L2 device registered as %s\n",
              video_device_node_name(vfd));

    return 0;

reg_vdev_err:
    video_device_release(vfd);
vdev_alloc_err:
    mw_device_deinit(&vch->mw_dev);
mw_dev_err:
    if (vch->format_lock != NULL) {
        os_spin_lock_free(vch->format_lock);
        vch->format_lock = NULL;
    }
lock_alloc_err:
    kfree(vch);
    return ret;
}

int mw_v4l2_release(struct mw_v4l2_dev *dev)
{
    struct mw_v4l2_channel *vch;
    struct list_head *node;

    if (dev == NULL)
        return 0;

    while (!list_empty(&dev->chnl_list)) {
        node = dev->chnl_list.next;
        vch = list_entry(node, struct mw_v4l2_channel, chnl_node);
        list_del(node);
        if (vch != NULL) {
            if (vch->vfd != NULL) {
                mw_debug(1, "unregistering %s\n", video_device_node_name(vch->vfd));

                video_unregister_device(vch->vfd);
                vch->vfd = NULL;
            }

            if (vch->format_lock != NULL)
                os_spin_lock_free(vch->format_lock);

            mw_device_deinit(&vch->mw_dev);

            kfree(vch);
        }
    }

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
    dev->v4l2_dev.dev = NULL;
#endif
    v4l2_device_unregister(&dev->v4l2_dev);

    return 0;
}

#ifdef CONFIG_PM
int mw_v4l2_suspend(struct mw_v4l2_dev *dev)
{
    return 0;
}

int mw_v4l2_resume(struct mw_v4l2_dev *dev)
{
    return 0;
}
#endif

int mw_v4l2_get_last_frame_sdianc_data(struct mw_stream *stream,
                                       MWCAP_SDI_ANC_PACKET **packets)
{
    struct mw_stream_pipe *pipe = container_of(stream, struct mw_stream_pipe, s_mw);

    return v4l2_sg_get_last_frame_sdianc_data(&pipe->s_v4l2.vsq, packets);
}
