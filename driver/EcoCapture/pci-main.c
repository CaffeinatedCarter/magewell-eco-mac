////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#include "mw-version.h"
#include "mw-driver-name.h"

#include "dma/mw-dma-mem-priv.h"

#include "ospi/linux-file.h"
#include "ospi/ospi.h"

#include "capture-device.h"
#include "capture-channel.h"

#include "mw-v4l2.h"
#include "mw-alsa.h"

extern void parse_internal_params(struct capture_device *capdev, const char *internal_params);

enum MW_CAP_PCI_ID {
    MW_CAP_PCI_VENDOR_ID                = 0x1CD7,

    MW_CAP_PCI_DEVICE_ID1               = 0x0050, /* EcoCapture OCTA_SDI */
    MW_CAP_PCI_DEVICE_ID2               = 0x0051, /* EcoCapture Dual HDMI M.2 */
    MW_CAP_PCI_DEVICE_ID3               = 0x0052, /* EcoCapture HDMI 4K M.2 */
    MW_CAP_PCI_DEVICE_ID4               = 0x0053, /* EcoCapture Dual SDI M.2 */
    MW_CAP_PCI_DEVICE_ID5               = 0x0054, /* EcoCapture Quad SDI M.2 */
    MW_CAP_PCI_DEVICE_ID6               = 0x0055, /* EcoCapture QL SDI 4K  M.2 */
    MW_CAP_PCI_DEVICE_ID7               = 0x0056, /* EcoCapture SDI 4K+ M.2 */
};

module_param(debug_level, uint, 00644);
MODULE_PARM_DESC(debug_level, "enable debug messages");

static char *nosignal_file = NULL;
module_param(nosignal_file, charp, 00644);
MODULE_PARM_DESC(nosignal_file, "Display this picture when there is no input signal");

static char *unsupported_file = NULL;
module_param(unsupported_file, charp, 00644);
MODULE_PARM_DESC(unsupported_file, "Display this picture when the input signal is not supported");

static char *locking_file = NULL;
module_param(locking_file, charp, 00644);
MODULE_PARM_DESC(locking_file, "Display this picture when the input signal is locking");

static char *limited_bandwidth_file = NULL;
module_param(limited_bandwidth_file, charp, 00644);
MODULE_PARM_DESC(limited_bandwidth_file, "Display this picture when the bandwidth is limited");

static bool disable_msi = false;
module_param(disable_msi, bool, 00644);
MODULE_PARM_DESC(disable_msi, "Disable or enable message signaled interrupts");

unsigned int enum_frameinterval_min = 166666;
module_param(enum_frameinterval_min, uint, 00644);
MODULE_PARM_DESC(enum_frameinterval_min, "Min frame interval for VIDIOC_ENUM_FRAMEINTERVALS (default: 166666(100ns))");

unsigned int enum_framesizes_type = 0;
module_param(enum_framesizes_type, uint, 00644);
MODULE_PARM_DESC(enum_framesizes_type, "VIDIOC_ENUM_FRAMESIZES type (1: DISCRETE; 2: STEPWISE; otherwise: CONTINUOUS )");

module_param(enable_raw_mode, bool, 00644);
MODULE_PARM_DESC(enable_raw_mode, "Enable raw mode of video for V4L2");

static char *internal_params = NULL;
module_param(internal_params, charp, 00644);
MODULE_PARM_DESC(internal_params, "Parameters for internal usage");

struct mw_cap_context {
    void __iomem *          reg_mem; /* pointer to mapped registers memory */
    struct capture_device   capdev;
    struct mw_v4l2_dev      v4l2;
    struct snd_card *       card[4];

    struct tasklet_struct   irq_tasklet;

    bool                    msi_enabled;
};

static const struct pci_device_id mw_cap_pci_tbl[] = {
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID1) },
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID2) },
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID3) },
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID4) },
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID5) },
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID6) },
    { PCI_DEVICE(MW_CAP_PCI_VENDOR_ID, MW_CAP_PCI_DEVICE_ID7) },
    { 0, }
};

static irqreturn_t mw_cap_irq_handler(int irq, void *data)
{
    struct mw_cap_context *ctx = data;

    if (capture_device_irq_process_top(&ctx->capdev) != 0)
        return IRQ_HANDLED;

    tasklet_schedule(&ctx->irq_tasklet);
    return IRQ_HANDLED;
}

static void mw_cap_irq_tasklet(unsigned long data)
{
    struct mw_cap_context *ctx = (struct mw_cap_context *)data;

    capture_device_irq_process_bottom(&ctx->capdev);
}

static int mw_cap_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int ret;
    struct mw_cap_context *ctx;
    int ch_count;
    int ch_index;
    struct capture_channel *pch;

    ctx = kzalloc(sizeof(struct mw_cap_context), GFP_KERNEL);
    if (ctx == NULL) {
        mw_err("Alloc memory for mw_cap_context failed!\n");
        ret = -ENOMEM;
        goto ctx_err;
    }

    tasklet_init(&ctx->irq_tasklet, mw_cap_irq_tasklet, (unsigned long)ctx);

    /* Enable PCI */
    ret = pci_enable_device(pdev);
    if (ret) {
        mw_err("Enable PCI Device failed!\n");
        goto enable_err;
    }

    pci_set_master(pdev);

    ret = pci_request_regions(pdev, VIDEO_CAP_DRIVER_NAME);
    if (ret < 0) {
        mw_err("Request memory region failed! ret=%d\n", ret);
        goto req_regions_err;
    }

    ctx->reg_mem = pci_iomap(pdev, 0, 0);
    if (NULL == ctx->reg_mem) {
        mw_err("pci_iomap failed\n");
        ret = -ENOMEM;
        goto iomap_err;
    }

    ctx->msi_enabled = false;
    if (!disable_msi && capture_device_is_support_msi(ctx->reg_mem)) {
        ret = pci_enable_msi(pdev);
        if (ret == 0) {
            ctx->msi_enabled = true;
            mw_debug(0, "MSI Enabled\n");
        }
    }

    if ((sizeof(dma_addr_t) > 4) &&
        !pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
        mw_debug(0, "dma 64 OK!\n");
    } else {
        mw_debug(0, "dma 64 not OK!\n");
        if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(64)) < 0) &&
            (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) < 0) {
            mw_err("DMA configuration failed\n");
            goto dma_mask_err;
        }
    }

    pci_set_drvdata(pdev, ctx);

    ret = request_irq(pdev->irq,
                      mw_cap_irq_handler,
                      IRQF_SHARED,
                      VIDEO_CAP_DRIVER_NAME,
                      ctx);
    if (ret) {
        mw_err("request_irq failed! ret=%d\n", ret);
        goto irq_err;
    }

    ret = capture_device_init(&ctx->capdev,
                              ctx->reg_mem,
                              NULL, //TODO
                              &pdev->dev,
                              nosignal_file,
                              unsupported_file,
                              locking_file,
                              limited_bandwidth_file);
    if (ret != 0) {
        mw_err("capture_device_init failed! ret=%d\n", ret);
        goto capdev_err;
    }

    parse_internal_params(&ctx->capdev, internal_params);

    ret = mw_v4l2_init(&ctx->v4l2, &pdev->dev);
    if (ret != 0)
        goto v4l2_err;

    ch_count = capture_device_GetChannelCount(&ctx->capdev);
    for (ch_index = 0; ch_index < ch_count; ch_index++) {
        pch = capture_device_GetChannel(&ctx->capdev, ch_index);
        ret = mw_v4l2_register_channel(&ctx->v4l2, pch);
        if (ret != 0)
            goto chnl_reg_err;

        ret = alsa_card_init(pch, &ctx->card[ch_index], &pdev->dev);
        if (ret != 0)
            goto alsa_err;
    }

    mw_debug(0, VIDEO_CAP_DRIVER_NAME " capture probe OK!\n");

    return 0;

alsa_err:
chnl_reg_err:
    ch_count = capture_device_GetChannelCount(&ctx->capdev);
    for (ch_index = 0; ch_index < ch_count; ch_index++) {
        if (ctx->card[ch_index] != NULL) {
            alsa_card_release(ctx->card[ch_index]);
            ctx->card[ch_index] = NULL;
        }
    }
    mw_v4l2_release(&ctx->v4l2);
v4l2_err:
    capture_device_deinit(&ctx->capdev);
capdev_err:
    free_irq(pdev->irq, ctx);
irq_err:
    pci_set_drvdata(pdev, NULL);
dma_mask_err:
    if (ctx->msi_enabled) {
        pci_disable_msi(pdev);
        ctx->msi_enabled = false;
    }
    pci_iounmap(pdev, ctx->reg_mem);
iomap_err:
    pci_release_regions(pdev);
req_regions_err:
    pci_disable_device(pdev);
enable_err:
    kfree(ctx);
ctx_err:
    return ret;
}

static void mw_cap_remove(struct pci_dev *pdev)
{
    struct mw_cap_context *ctx = pci_get_drvdata(pdev);
    int ch_count;
    int ch_index;

    mw_debug(0, "\n");

    if (ctx == NULL)
        return;

    ch_count = capture_device_GetChannelCount(&ctx->capdev);
    for (ch_index = 0; ch_index < ch_count; ch_index++) {
        if (ctx->card[ch_index] != NULL) {
            alsa_card_release(ctx->card[ch_index]);
            ctx->card[ch_index] = NULL;
        }
    }
    mw_v4l2_release(&ctx->v4l2);

    capture_device_deinit(&ctx->capdev);

    if (pdev->irq > 0) {
        free_irq(pdev->irq, ctx);
    }

    if (ctx->msi_enabled)
        pci_disable_msi(pdev);

    if (NULL != ctx->reg_mem)
        pci_iounmap(pdev, ctx->reg_mem);

    pci_release_regions(pdev);

    pci_set_drvdata(pdev, NULL);

    pci_disable_device(pdev);

    kfree(ctx);
}

static void mw_cap_shutdown(struct pci_dev *pdev)
{
    mw_debug(0, "\n");

    mw_cap_remove(pdev);
}

#ifdef CONFIG_PM
static int mw_cap_suspend(struct pci_dev *pdev, pm_message_t state)
{
    struct mw_cap_context *ctx = pci_get_drvdata(pdev);
    int ch_count;
    int ch_index;

    ch_count = capture_device_GetChannelCount(&ctx->capdev);
    for (ch_index = 0; ch_index < ch_count; ch_index++) {
        alsa_suspend(ctx->card[ch_index]);
    }
    mw_v4l2_suspend(&ctx->v4l2);

    capture_device_sleep(&ctx->capdev);

    if (pdev->irq > 0) {
        free_irq(pdev->irq, ctx);
    }

    if (ctx->msi_enabled)
        pci_disable_msi(pdev);
    ctx->msi_enabled = false;

    pci_disable_device(pdev);
    pci_save_state(pdev);

    pci_set_power_state(pdev, pci_choose_state(pdev, state));

    return 0;
}

static int mw_cap_resume(struct pci_dev *pdev)
{
    struct mw_cap_context *ctx = pci_get_drvdata(pdev);
    int ret;
    int ch_count;
    int ch_index;

    pci_set_power_state(pdev, PCI_D0);
    pci_restore_state(pdev);

    ret = pci_enable_device(pdev);
    if(ret) {
        mw_err("Enable PCI Device failed!\n");
        ret = -EIO;
        goto out;
    }

    pci_set_master(pdev);

    ctx->msi_enabled = false;
    if (!disable_msi && capture_device_is_support_msi(ctx->reg_mem)) {
        ret = pci_enable_msi(pdev);
        if (ret == 0) {
            ctx->msi_enabled = true;
            mw_debug(0, "MSI Enabled\n");
        }
    }

    ret = request_irq(pdev->irq,
                      mw_cap_irq_handler,
                      IRQF_SHARED,
                      VIDEO_CAP_DRIVER_NAME,
                      ctx);
    if (ret) {
        mw_err("request_irq failed\n");
        goto out;
    }

    capture_device_resume(&ctx->capdev);

    ch_count = capture_device_GetChannelCount(&ctx->capdev);
    for (ch_index = 0; ch_index < ch_count; ch_index++) {
        alsa_resume(ctx->card[ch_index]);
    }
    mw_v4l2_resume(&ctx->v4l2);

    return 0;

out: // TODO
    return ret;
}
#endif

static struct pci_driver mw_cap_pci_driver = {
    .name               = VIDEO_CAP_DRIVER_NAME,
    .probe              = mw_cap_probe,
    .remove             = mw_cap_remove,
    .id_table           = mw_cap_pci_tbl,
    .shutdown           = mw_cap_shutdown,
#ifdef CONFIG_PM
    .suspend            = mw_cap_suspend,
    .resume             = mw_cap_resume,
#endif
};

static int __init mw_cap_init(void)
{
    int ret;

    ret = os_init();
    if (ret != OS_RETURN_SUCCESS)
        return ret;

    ret = mw_dma_memory_init();
    if (ret != OS_RETURN_SUCCESS)
        goto dma_err;

    ret = pci_register_driver(&mw_cap_pci_driver);
    if (ret != 0)
        goto pci_err;

    return 0;

pci_err:
    mw_dma_memory_deinit();
dma_err:
    os_deinit();
    return ret;
}

static void __exit mw_cap_cleanup(void)
{
    pci_unregister_driver(&mw_cap_pci_driver);

    mw_dma_memory_deinit();
    os_deinit();
}

module_init(mw_cap_init);
module_exit(mw_cap_cleanup);

MODULE_DESCRIPTION("Magewell Electronics Co., Ltd. Eco Capture Driver");
MODULE_AUTHOR("Magewell Electronics Co., Ltd. <support@magewell.net>");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, mw_cap_pci_tbl);
MODULE_VERSION(VERSION_STRING);
