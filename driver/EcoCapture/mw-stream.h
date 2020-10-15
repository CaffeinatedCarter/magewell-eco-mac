////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __MW_DEVICE_H__
#define __MW_DEVICE_H__

#include "ospi/ospi.h"
#include "mw-capture-impl.h"

#include "mw-procapture-extension-private.h"
#include "mw-linux.h"

struct frame_settings {
    short                                   brightness;
    short                                   contrast;
    short                                   hue;
    short                                   saturation;

    MWCAP_VIDEO_PROCESS_SETTINGS            process_settings;
};

struct mw_device {
    struct capture_channel *    capch; /* low level card driver */
    os_mutex_t                  dev_mutex; /* device global mutex */

    /* default settings */
    struct frame_settings       settings;
    os_spinlock_t               settings_lock;

    os_dma_par_t                dma_priv;

    bool                        supported_multi_stream;
};

struct mw_stream {
    struct mw_device                *mw_dev;

    struct mw_stream_cap            s_mwcap;
};

int mw_device_init(struct mw_device *mw_dev);

void mw_device_deinit(struct mw_device *mw_dev);

int mw_stream_init(struct mw_stream *stream, struct mw_device *mw_dev);
void mw_stream_deinit(struct mw_stream *stream);

long mw_stream_ioctl(struct mw_stream *stream,  os_iocmd_t cmd, void *arg);

int mw_v4l2_get_last_frame_sdianc_data(struct mw_stream *stream,
                                       MWCAP_SDI_ANC_PACKET **packets);

#endif /* __MW_DEVICE_H__ */
