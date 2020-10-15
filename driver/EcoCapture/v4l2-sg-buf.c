////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////

#include <ospi/ospi.h>

#include "v4l2-sg-buf.h"

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>

void v4l2_sg_queue_init(struct v4l2_sg_buf_queue *queue,
                        enum v4l2_buf_type type,
                        struct mutex *ext_mutex,
                        struct device *parent_dev,
                        enum v4l2_field field)
{
    queue->ext_mutex = ext_mutex;
    queue->parent_dev = parent_dev;
    queue->field = field;
    queue->type = type;
    queue->num_buffers = 0;

    spin_lock_init(&queue->active_lock);
    spin_lock_init(&queue->done_lock);

    INIT_LIST_HEAD(&queue->queued_list);
    INIT_LIST_HEAD(&queue->active_list);
    INIT_LIST_HEAD(&queue->done_list);

    init_waitqueue_head(&queue->active_wait);
    init_waitqueue_head(&queue->done_wait);
}

static int _v4l2_sg_dma_map(struct v4l2_sg_buf *vbuf)
{
    if (vbuf->mwsg_len == 0 || vbuf->mwsg_list == NULL) {
        int sglen = 0;
        sglen = dma_map_sg(vbuf->queue->parent_dev, vbuf->dma_desc.sglist,
                           vbuf->dma_desc.num_pages, DMA_FROM_DEVICE);

        if (sglen <= 0) {
            printk(KERN_ERR "%s: dma_map_sg failed sglen=%d\n", __func__, sglen);
            return -ENOMEM;
        }

        vbuf->mwsg_list = os_malloc(sizeof(*vbuf->mwsg_list)
                                    * vbuf->dma_desc.num_pages);
        if (vbuf->mwsg_list != NULL) {
            int i;

            for (i = 0; i < vbuf->dma_desc.num_pages; i++) {
                mw_sg_dma_address(&vbuf->mwsg_list[i]) =
                        sg_dma_address(&vbuf->dma_desc.sglist[i]);
                mw_sg_dma_len(&vbuf->mwsg_list[i]) =
                        sg_dma_len(&vbuf->dma_desc.sglist[i]);
            }

            vbuf->mwsg_len = vbuf->dma_desc.num_pages;
        } else {
            dma_unmap_sg(vbuf->queue->parent_dev, vbuf->dma_desc.sglist,
                         vbuf->dma_desc.num_pages, DMA_FROM_DEVICE);
            return -ENOMEM;
        }

        if (v4l2_sg_get_vaddr(vbuf) == NULL) {
            os_free(vbuf->mwsg_list);
            dma_unmap_sg(vbuf->queue->parent_dev, vbuf->dma_desc.sglist,
                         vbuf->dma_desc.num_pages, DMA_FROM_DEVICE);
            return -ENOMEM;
        }
    }

    return 0;
}

static void _v4l2_sg_dma_unmap(struct v4l2_sg_buf *vbuf)
{
    if (vbuf->mwsg_list != NULL) {
        os_free(vbuf->mwsg_list);
        vbuf->mwsg_list = NULL;
        vbuf->mwsg_len = 0;

        dma_unmap_sg(vbuf->queue->parent_dev, vbuf->dma_desc.sglist,
                     vbuf->dma_desc.num_pages, DMA_FROM_DEVICE);
    }
}

static int _v4l2_sg_buf_mmap_alloc(struct v4l2_sg_buf *vbuf,
                                   unsigned int size)
{
    struct v4l2_sg_buf_dma_desc *dma_desc = &vbuf->dma_desc;
    int ret;
    int i;

    dma_desc->vaddr = NULL;
    dma_desc->write = 1;
    dma_desc->offset = 0;

    dma_desc->size = PAGE_ALIGN(size);
    dma_desc->num_pages = dma_desc->size >> PAGE_SHIFT;
    dma_desc->sglist = os_malloc(sizeof(*dma_desc->sglist) * dma_desc->num_pages);
    if (dma_desc->sglist == NULL) {
        ret = -ENOMEM;
        goto sglist_err;
    }
    sg_init_table(dma_desc->sglist, dma_desc->num_pages);

    dma_desc->pages = os_malloc(dma_desc->num_pages * sizeof(struct page *));
    if (dma_desc->pages == NULL) {
        ret = -ENOMEM;
        goto pages_array_err;
    }

    for (i = 0; i < dma_desc->num_pages; i++) {
        dma_desc->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO |  __GFP_NOWARN);
        if (dma_desc->pages[i] == NULL) {
            ret = -ENOMEM;
            goto pages_err;
        }
        sg_set_page(&dma_desc->sglist[i], dma_desc->pages[i], PAGE_SIZE, 0);
    }

    ret = _v4l2_sg_dma_map(vbuf);
    if (ret < 0)
        goto dma_map_err;

    atomic_set(&dma_desc->mmap_refcount, 1);

    mw_debug(50, "Alloc mmap buffer of %d pages\n", dma_desc->num_pages);

    return 0;

dma_map_err:
pages_err:
    while (--i >= 0)
        __free_page(dma_desc->pages[i]);
    os_free(dma_desc->pages);
pages_array_err:
    os_free(dma_desc->sglist);
sglist_err:
    memset(dma_desc, 0, sizeof(*dma_desc));
    return ret;
}

static void _v4l2_sg_buf_mmap_free(struct v4l2_sg_buf *vbuf)
{
    struct v4l2_sg_buf_dma_desc *dma_desc = &vbuf->dma_desc;

    if (atomic_dec_and_test(&dma_desc->mmap_refcount)) {
        int i = dma_desc->num_pages;
        mw_debug(50, "Freeing buffer of %d pages\n", dma_desc->num_pages);

        _v4l2_sg_dma_unmap(vbuf);

        if (dma_desc->vaddr)
            vm_unmap_ram(dma_desc->vaddr, dma_desc->num_pages);
        while (--i >= 0)
            __free_page(dma_desc->pages[i]);
        os_free(dma_desc->pages);
        os_free(dma_desc->sglist);
        memset(dma_desc, 0, sizeof(*dma_desc));
    }
}

static bool __mmap_mem_in_use(struct v4l2_sg_buf *vbuf)
{
    return (atomic_read(&vbuf->dma_desc.mmap_refcount) > 1);
}

static int _v4l2_sg_buf_userptr_get(struct v4l2_sg_buf *vbuf,
                                 unsigned long vaddr, unsigned long size)
{
    struct v4l2_sg_buf_dma_desc *dma_desc = &vbuf->dma_desc;
    unsigned long first, last;
    int user_pages;
    int ret = 0;
    int i;

    dma_desc->vaddr = NULL;
    dma_desc->write = 1;
    dma_desc->offset = vaddr & ~PAGE_MASK;
    dma_desc->size = size;

    first = (vaddr & PAGE_MASK) >> PAGE_SHIFT;
    last  = ((vaddr + size - 1) & PAGE_MASK) >> PAGE_SHIFT;
    dma_desc->num_pages = last - first + 1;

    dma_desc->sglist = os_malloc(sizeof(*dma_desc->sglist) * dma_desc->num_pages);
    if (dma_desc->sglist == NULL) {
        ret = -ENOMEM;
        goto sglist_err;
    }
    sg_init_table(dma_desc->sglist, dma_desc->num_pages);

    dma_desc->pages = os_malloc(dma_desc->num_pages * sizeof(struct page *));
    if (dma_desc->pages == NULL) {
        ret = -ENOMEM;
        goto pages_array_err;
    }

    user_pages = get_user_pages_fast(vaddr & PAGE_MASK,
                                dma_desc->num_pages,
                                dma_desc->write,
                                dma_desc->pages);
    if (user_pages != dma_desc->num_pages) {
        ret = -EFAULT;
        goto user_pages_err;
    }

    sg_set_page(&dma_desc->sglist[0], dma_desc->pages[0],
            PAGE_SIZE - dma_desc->offset, dma_desc->offset);
    size -= PAGE_SIZE - dma_desc->offset;
    for (i = 1; i < dma_desc->num_pages; ++i) {
        sg_set_page(&dma_desc->sglist[i], dma_desc->pages[i],
                    min_t(size_t, PAGE_SIZE, size), 0);
        size -= min_t(size_t, PAGE_SIZE, size);
    }

    ret = _v4l2_sg_dma_map(vbuf);
    if (ret < 0)
        goto dma_map_err;

    return 0;

dma_map_err:
user_pages_err:
    while (--user_pages >= 0)
            put_page(dma_desc->pages[user_pages]);
    os_free(dma_desc->pages);
pages_array_err:
    os_free(dma_desc->sglist);
sglist_err:
    memset(dma_desc, 0, sizeof(*dma_desc));
    return ret;
}

static void _v4l2_sg_buf_userptr_put(struct v4l2_sg_buf *vbuf)
{
    struct v4l2_sg_buf_dma_desc *dma_desc = &vbuf->dma_desc;
    int i = dma_desc->num_pages;

    _v4l2_sg_dma_unmap(vbuf);

    if (dma_desc->vaddr != NULL) {
        vm_unmap_ram(dma_desc->vaddr, dma_desc->num_pages);
        dma_desc->vaddr = NULL;
    }
    while (--i >= 0) {
        if (dma_desc->write)
            set_page_dirty_lock(dma_desc->pages[i]);
        put_page(dma_desc->pages[i]);
    }
    if (dma_desc->sglist != NULL)
        os_free(dma_desc->sglist);
    if (dma_desc->pages != NULL)
        os_free(dma_desc->pages);
    memset(dma_desc, 0, sizeof(*dma_desc));
}

static int _v4l2_sg_queue_alloc(struct v4l2_sg_buf_queue *queue,
                              unsigned int num_buffers)
{
    unsigned int index;
    int ret = 0;
    struct v4l2_sg_buf *vbuf;

    for (index = 0; index < num_buffers; index++) {
        vbuf = os_zalloc(sizeof(struct v4l2_sg_buf));
        if (vbuf == NULL) {
            mw_debug(0, "Memory alloc for v4l2_sg_buf struct failed!\n");
            break;
        }

        vbuf->state = V4L2_SG_BUF_STATE_DEQUEUED;
        vbuf->queue = queue;

        vbuf->v4l2_buf.index = index;
        vbuf->v4l2_buf.type = queue->type;
        vbuf->v4l2_buf.memory = queue->memory;

        if (vbuf->v4l2_buf.memory == V4L2_MEMORY_MMAP) {
            ret = _v4l2_sg_buf_mmap_alloc(vbuf, queue->buf_size);
            if (ret != 0) {
                mw_debug(0, "_v4l2_sg_buf_mmap_alloc error\n");
                goto out_err;
            }

            vbuf->v4l2_buf.m.offset = PAGE_ALIGN(queue->buf_size) * index;
            vbuf->v4l2_buf.length = queue->buf_size;
        }

        queue->bufs[vbuf->v4l2_buf.index] = vbuf;
    }

out_err:
    return index;
}

static int _v4l2_sg_queue_free(struct v4l2_sg_buf_queue *queue)
{
    int i;

    mw_debug(20, "%s\n", __func__);
    if (!queue)
        return 0;

    if (queue->streaming) {
        mw_debug(0, "Cannot free buffers when streaming!\n");
        return -EBUSY;
    }

    for (i = 0; i < VIDEO_MAX_FRAME; i++) {
        if (queue->bufs[i] &&
                __mmap_mem_in_use(queue->bufs[i])) {
            mw_debug(0, "Cannot free mmapped buffers\n");
            return -EBUSY;
        }
    }

    for (i = 0; i < VIDEO_MAX_FRAME; i++) {
        if (NULL == queue->bufs[i])
            continue;
        if (queue->bufs[i]->v4l2_buf.memory == V4L2_MEMORY_MMAP)
            _v4l2_sg_buf_mmap_free(queue->bufs[i]);
        else
            _v4l2_sg_buf_userptr_put(queue->bufs[i]);

        os_free(queue->bufs[i]);
        queue->bufs[i] = NULL;
    }

    queue->num_buffers = 0;

    return 0;
}

void v4l2_sg_queue_deinit(struct v4l2_sg_buf_queue *queue)
{
    v4l2_sg_queue_streamoff(queue);
    _v4l2_sg_queue_free(queue);
}

int v4l2_sg_queue_reqbufs(struct v4l2_sg_buf_queue *queue,
                          struct v4l2_requestbuffers *req,
                          unsigned int bufsize)
{
    unsigned int count;
    int ret = 0;

    if (req->memory != V4L2_MEMORY_MMAP
            && req->memory != V4L2_MEMORY_USERPTR
            ) {
        mw_debug(0, "buffer type or memory type invalid\n");
        return -EINVAL;
    }

    if (req->type != queue->type) {
        mw_debug(0, "queue type invalid\n");
        ret = -EINVAL;
        goto out;
    }

    if (queue->streaming) {
        mw_debug(0, "streaming already avtive\n");
        ret = -EBUSY;
        goto out;
    }

    if (req->count == 0 || queue->num_buffers != 0 || queue->memory != req->memory) {
        if (queue->memory == V4L2_MEMORY_MMAP) {
            unsigned int index;

            for (index = 0; index < queue->num_buffers; index++) {
                if (__mmap_mem_in_use(queue->bufs[index])) {
                    ret = -EBUSY;
                    goto out;
                }
            }
        }

        _v4l2_sg_queue_free(queue);

        /* just free buffers */
        if (req->count == 0) {
            ret = 0;
            goto out;
        }
    }

    count = min_t(unsigned int, req->count, VIDEO_MAX_FRAME);

    queue->memory = req->memory;
    queue->buf_size = bufsize;

    ret = _v4l2_sg_queue_alloc(queue, count);
    if (ret == 0) {
        mw_debug(0, "Memory buffer allocation failed\n");
        ret = -ENOMEM;
        goto out;
    }

    queue->num_buffers = ret;

    req->count = queue->num_buffers;

    ret = 0;

out:
    return ret;
}

static void __v4l2_sg_buf_status(struct v4l2_sg_buf *vbuf, struct v4l2_buffer *v4l2_buf)
{
    memcpy(v4l2_buf, &vbuf->v4l2_buf, offsetof(struct v4l2_buffer, m));

    v4l2_buf->memory   = vbuf->v4l2_buf.memory;
    switch (v4l2_buf->memory) {
    case V4L2_MEMORY_MMAP:
        v4l2_buf->m.offset  = vbuf->v4l2_buf.m.offset;
        v4l2_buf->length    = vbuf->v4l2_buf.length;
        break;
    case V4L2_MEMORY_USERPTR:
        v4l2_buf->m.userptr = vbuf->v4l2_buf.m.userptr;
        v4l2_buf->length    = vbuf->v4l2_buf.length;
        break;
    default:
        break;
    }
    v4l2_buf->bytesused = vbuf->v4l2_buf.length;

    v4l2_buf->flags = 0;

    switch (vbuf->state) {
    case V4L2_SG_BUF_STATE_QUEUED:
    case V4L2_SG_BUF_STATE_ACTIVE:
        v4l2_buf->flags |= V4L2_BUF_FLAG_QUEUED;
        break;
    case V4L2_SG_BUF_STATE_ERROR:
        v4l2_buf->flags |= V4L2_BUF_FLAG_ERROR;
        /* fall through */
    case V4L2_SG_BUF_STATE_DONE:
        v4l2_buf->flags |= V4L2_BUF_FLAG_DONE;
        break;
    case V4L2_SG_BUF_STATE_PREPARED:
        //v4l2_buf->flags |= V4L2_BUF_FLAG_PREPARED;
        break;
    case V4L2_SG_BUF_STATE_DEQUEUED:
        /* nothing */
        break;
    }

    if (__mmap_mem_in_use(vbuf))
        v4l2_buf->flags |= V4L2_BUF_FLAG_MAPPED;
}

int v4l2_sg_queue_querybuf(struct v4l2_sg_buf_queue *queue, struct v4l2_buffer *v4l2_buf)
{
    int ret = 0;
    struct v4l2_sg_buf *vbuf;

    if (v4l2_buf->type != queue->type) {
        mw_debug(0, "Wrong type.\n");
        ret = -EINVAL;
        goto done;
    }
    if (v4l2_buf->index >= queue->num_buffers) {
        mw_debug(1, "index out of range.\n");
        ret = -EINVAL;
        goto done;
    }

    vbuf = queue->bufs[v4l2_buf->index];

    __v4l2_sg_buf_status(vbuf, v4l2_buf);

    ret = 0;

done:
    return ret;
}

int v4l2_sg_queue_qbuf(struct v4l2_sg_buf_queue *queue, struct v4l2_buffer *v4l2_buf)
{
    struct v4l2_sg_buf *vbuf;
    int ret;

    if (v4l2_buf->type != queue->type) {
        mw_debug(0, "invalid buffer type\n");
        return -EINVAL;
    }
    if (v4l2_buf->index >= queue->num_buffers) {
        mw_debug(0, "buffer index out of range\n");
        return -EINVAL;
    }
    vbuf = queue->bufs[v4l2_buf->index];
    if (vbuf == NULL) {
        mw_debug(0, "buffer is NULL\n");
        return -EINVAL;
    }
    if (v4l2_buf->memory != queue->memory) {
        mw_debug(0, "invalid memory type");
        return -EINVAL;
    }
    if (vbuf->state != V4L2_SG_BUF_STATE_DEQUEUED) {
        mw_debug(0, "buffer is already queued.\n");
        return -EINVAL;
    }

    switch (v4l2_buf->memory) {
    case V4L2_MEMORY_MMAP:
        break;
    case V4L2_MEMORY_USERPTR:
        if (vbuf->queue->buf_size > v4l2_buf->length)
            return -EINVAL;

        if (vbuf->v4l2_buf.m.userptr != v4l2_buf->m.userptr ||
                vbuf->v4l2_buf.length != v4l2_buf->length) {
            _v4l2_sg_buf_userptr_put(vbuf);

            vbuf->v4l2_buf.m.userptr = 0;
            vbuf->v4l2_buf.length = 0;

            ret = _v4l2_sg_buf_userptr_get(vbuf,
                                     v4l2_buf->m.userptr,
                                     v4l2_buf->length);
            if (ret < 0) {
                mw_debug(0, "_v4l2_sg_buf_userptr_get err\n");
                return ret;
            }

            vbuf->v4l2_buf.m.userptr = v4l2_buf->m.userptr;
            vbuf->v4l2_buf.length = v4l2_buf->length;
        }
        break;
    default:
        mw_debug(1, "wrong memory type\n");
        return -EINVAL;
    }

    __v4l2_sg_buf_status(vbuf, v4l2_buf);

    list_add_tail(&vbuf->queued_node, &queue->queued_list);
    vbuf->state = V4L2_SG_BUF_STATE_QUEUED;

    if (queue->streaming) {
        spin_lock_bh(&queue->active_lock);
        list_add_tail(&vbuf->active_node, &queue->active_list);
        vbuf->state = V4L2_SG_BUF_STATE_ACTIVE;

        wake_up_interruptible_sync(&queue->active_wait);
        spin_unlock_bh(&queue->active_lock);
    }

    return 0;
}

int v4l2_sg_queue_dqbuf(struct v4l2_sg_buf_queue *queue,
                        struct v4l2_buffer *v4l2_buf, bool nonblocking)
{
    struct v4l2_sg_buf *vbuf;
    int ret;

    if (v4l2_buf->type != queue->type) {
        mw_debug(0, "invalid buffer type\n");
        return -EINVAL;
    }

    for (;;) {

        if (!queue->streaming) {
            mw_debug(5, "Streaming off, will not wait for buffers\n");
            return -EINVAL;
        }

        if (!list_empty(&queue->done_list)) {
            break;
        }

        if (nonblocking) {
            mw_debug(20, "Nonblocking and no buffers to dequeue, "
                       "will not wait\n");
            return -EAGAIN;
        }

        mw_debug(20, "Will sleep waiting for buffers\n");
        ret = wait_event_interruptible(queue->done_wait,
                                       !list_empty(&queue->done_list) || !queue->streaming);
        if (ret != 0) {
            mw_debug(1, "Sleep was interrupted\n");
            return ret;
        }
    }

    spin_lock_bh(&queue->done_lock);
    vbuf= list_first_entry(&queue->done_list, struct v4l2_sg_buf, done_node);
    list_del(&vbuf->done_node);
    spin_unlock_bh(&queue->done_lock);

    switch (vbuf->state) {
    case V4L2_SG_BUF_STATE_DONE:
        mw_debug(20, "Returning done buffer\n");
        break;
    case V4L2_SG_BUF_STATE_ERROR:
        mw_debug(20, "Returning done buffer with errors\n");
        break;
    default:
        mw_debug(1, "Invalid buffer state\n");
        return -EINVAL;
    }

    __v4l2_sg_buf_status(vbuf, v4l2_buf);

    dma_sync_sg_for_cpu(queue->parent_dev, vbuf->dma_desc.sglist,
                        vbuf->dma_desc.num_pages, DMA_FROM_DEVICE);

    list_del(&vbuf->queued_node);

    vbuf->state = V4L2_SG_BUF_STATE_DEQUEUED;

    queue->last_dqueue_vbuf = vbuf;

    return 0;
}

static void v4l2_sg_buf_vm_open(struct vm_area_struct *vma)
{
    struct v4l2_sg_buf *vbuf = vma->vm_private_data;

    mw_debug(50, "%s: %p, refcount: %d, vma: %08lx-%08lx\n",
           __func__, vbuf, atomic_read(&vbuf->dma_desc.mmap_refcount), vma->vm_start,
           vma->vm_end);

    atomic_inc(&vbuf->dma_desc.mmap_refcount);
}

static void v4l2_sg_buf_vm_close(struct vm_area_struct *vma)
{
    struct v4l2_sg_buf *vbuf = vma->vm_private_data;

    mw_debug(50, "%s: %p, refcount: %d, vma: %08lx-%08lx\n",
           __func__, vbuf, atomic_read(&vbuf->dma_desc.mmap_refcount), vma->vm_start,
           vma->vm_end);

    _v4l2_sg_buf_mmap_free(vbuf);
}

static struct vm_operations_struct v4l2_sg_buf_vm_ops = {
    .open       = v4l2_sg_buf_vm_open,
    .close      = v4l2_sg_buf_vm_close,
};


int v4l2_sg_buf_mmap(struct v4l2_sg_buf_queue *queue,
                     struct vm_area_struct *vma)
{
    struct v4l2_sg_buf *vbuf;
    unsigned long uaddr = vma->vm_start;
    unsigned long usize = vma->vm_end - vma->vm_start;
    int i;

    if (queue->memory != V4L2_MEMORY_MMAP) {
        mw_debug(0, "mmap for none V4L2_MEMORY_MMAP memory\n");
        return -EINVAL;
    }

    if (!(vma->vm_flags & VM_SHARED)) {
        mw_debug(0, "Invalid vma flags, VM_SHARED needed\n");
        return -EINVAL;
    }

    if (!(vma->vm_flags & VM_READ)) {
        mw_debug(0, "Invalid vma flags, VM_READ needed\n");
        return -EINVAL;
    }

    for (i = 0; i < queue->num_buffers; i++) {
        vbuf = queue->bufs[i];

        if (vbuf && vbuf->v4l2_buf.memory == V4L2_MEMORY_MMAP &&
                vbuf->v4l2_buf.m.offset == (vma->vm_pgoff << PAGE_SHIFT)) {
            break;
        }
    }
    if (i >= queue->num_buffers) {
        mw_debug(0, "MMAP invalid, as it would overflow buffer length\n");
        return -EINVAL;
    }

    if (PAGE_ALIGN(vbuf->v4l2_buf.length) < (vma->vm_end - vma->vm_start)) {
        mw_debug(0, "MMAP invalid, as it would overflow buffer length\n");
        return -EINVAL;
    }

    i = 0;
    do {
        int ret;

        ret = vm_insert_page(vma, uaddr, vbuf->dma_desc.pages[i++]);
        if (ret) {
            printk(KERN_ERR "Remapping memory, error: %d\n", ret);
            return ret;
        }

        uaddr += PAGE_SIZE;
        usize -= PAGE_SIZE;
    } while (usize > 0);

    vma->vm_private_data    = vbuf;
    vma->vm_ops             = &v4l2_sg_buf_vm_ops;

    vma->vm_ops->open(vma);

    return 0;
}

int v4l2_sg_queue_streamon(struct v4l2_sg_buf_queue *queue)
{
    struct v4l2_sg_buf *vbuf = NULL;

    if (queue->streaming) {
        mw_debug(0, "already streaming\n");
        return -EBUSY;
    }

    spin_lock_bh(&queue->active_lock);
    list_for_each_entry(vbuf, &queue->queued_list, queued_node)
        list_add_tail(&vbuf->active_node, &queue->active_list);
    spin_unlock_bh(&queue->active_lock);

    queue->streaming = 1;

    return 0;
}

int v4l2_sg_queue_streamoff(struct v4l2_sg_buf_queue *queue)
{
    int i;

    if (!queue->streaming) {
        mw_debug(0, "not streaming\n");
        return -EINVAL;
    }

    queue->streaming = 0;

    INIT_LIST_HEAD(&queue->queued_list);
    INIT_LIST_HEAD(&queue->done_list);
    INIT_LIST_HEAD(&queue->active_list);

    wake_up_all(&queue->done_wait);

    for (i = 0; i < queue->num_buffers; ++i) {
        queue->bufs[i]->state = V4L2_SG_BUF_STATE_DEQUEUED;
    }
    return 0;
}

struct v4l2_sg_buf *v4l2_sg_queue_get_activebuf(struct v4l2_sg_buf_queue *queue)
{
    struct v4l2_sg_buf *vbuf = NULL;

    spin_lock_bh(&queue->active_lock);

    if (list_empty(&queue->active_list)) {
        goto out;
    }

    vbuf = list_entry(queue->active_list.next,
                    struct v4l2_sg_buf, active_node);
    list_del(&vbuf->active_node);

out:
    spin_unlock_bh(&queue->active_lock);

    return vbuf;
}

void v4l2_sg_queue_put_donebuf(struct v4l2_sg_buf_queue *queue, struct v4l2_sg_buf *vbuf)
{
    spin_lock_bh(&queue->done_lock);

    list_add_tail(&vbuf->done_node, &queue->done_list);

    wake_up_interruptible_sync(&queue->done_wait);

    spin_unlock_bh(&queue->done_lock);
}

unsigned int v4l2_sg_queue_poll(struct file *file,
                                struct v4l2_sg_buf_queue *queue,
                                struct poll_table_struct *wait)
{
    unsigned int mask = 0;
    struct v4l2_sg_buf *vbuf = NULL;

    if (list_empty(&queue->queued_list))
        return mask | POLLERR;

    if (list_empty(&queue->done_list))
        poll_wait(file, &queue->done_wait, wait);

    spin_lock_bh(&queue->done_lock);
    if (!list_empty(&queue->done_list))
        vbuf= list_first_entry(&queue->done_list, struct v4l2_sg_buf, done_node);
    spin_unlock_bh(&queue->done_lock);
    if (vbuf != NULL && (vbuf->state == V4L2_SG_BUF_STATE_DONE
                         || vbuf->state == V4L2_SG_BUF_STATE_ERROR))
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

void* v4l2_sg_get_vaddr(struct v4l2_sg_buf* vbuf)
{
    BUG_ON(!vbuf);

    if (!vbuf->dma_desc.vaddr)
        vbuf->dma_desc.vaddr = vm_map_ram(
                    vbuf->dma_desc.pages,
                    vbuf->dma_desc.num_pages,
                    -1,
                    PAGE_KERNEL);

    /* add offset in case userptr is not page-aligned */
    return vbuf->dma_desc.vaddr + vbuf->dma_desc.offset;
}

int v4l2_sg_get_last_frame_sdianc_data(struct v4l2_sg_buf_queue *queue,
                                       MWCAP_SDI_ANC_PACKET **packets)
{
    struct v4l2_sg_buf *vbuf = NULL;

    if (packets == NULL || queue == NULL)
        return OS_RETURN_INVAL;

    vbuf = queue->last_dqueue_vbuf;

    if (vbuf == NULL)
        return OS_RETURN_NODATA;

    if (vbuf->anc_packet_count <= 0)
        return OS_RETURN_NODATA;

    *packets = vbuf->anc_packets;

    return vbuf->anc_packet_count;
}
