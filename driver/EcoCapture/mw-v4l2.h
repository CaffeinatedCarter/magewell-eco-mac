////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __MW_V4L2_H__
#define __MW_V4L2_H__

#include <media/v4l2-device.h>

#include "capture-channel.h"

extern bool enable_raw_mode;

struct mw_v4l2_dev {
    struct v4l2_device          v4l2_dev;

    struct device *             pci_dev;

    struct list_head            chnl_list;
};

int mw_v4l2_init(struct mw_v4l2_dev *dev, struct device *pci_dev);

int mw_v4l2_register_channel(struct mw_v4l2_dev *dev, struct capture_channel *capch);

int mw_v4l2_release(struct mw_v4l2_dev *dev);

#ifdef CONFIG_PM
int mw_v4l2_suspend(struct mw_v4l2_dev *dev);

int mw_v4l2_resume(struct mw_v4l2_dev *dev);
#endif

#endif /* __MW_V4L2_H__ */
