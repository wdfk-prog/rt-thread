/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_internal.h"

#define CAN_LOCK(can)   rt_mutex_take(&(can)->lock, RT_WAITING_FOREVER)
#define CAN_UNLOCK(can) rt_mutex_release(&(can)->lock)

static rt_err_t rt_can_init(struct rt_device *dev)
{
    struct rt_can_device *can = (struct rt_can_device *)dev;

    RT_ASSERT(can != RT_NULL);
    can->can_rx = RT_NULL;
    can->can_tx = RT_NULL;
#ifdef RT_CAN_USING_HDR
    can->hdr = RT_NULL;
#endif
    return can->ops->configure ? can->ops->configure(can, &can->config) : -RT_ENOSYS;
}

static rt_err_t _can_rx_open(struct rt_can_device *can)
{
    int i;
    struct rt_can_rx_fifo *rx_fifo;

    rx_fifo = (struct rt_can_rx_fifo *)rt_malloc(sizeof(*rx_fifo) +
              can->config.msgboxsz * sizeof(struct rt_can_msg_list));
    if (rx_fifo == RT_NULL)
        return -RT_ENOMEM;

    rx_fifo->buffer = (struct rt_can_msg_list *)(rx_fifo + 1);
    rt_memset(rx_fifo->buffer, 0,
              can->config.msgboxsz * sizeof(struct rt_can_msg_list));
    rt_list_init(&rx_fifo->freelist);
    rt_list_init(&rx_fifo->uselist);
    rx_fifo->freenumbers = can->config.msgboxsz;
    for (i = 0; i < can->config.msgboxsz; i++)
    {
        rt_list_insert_before(&rx_fifo->freelist, &rx_fifo->buffer[i].list);
#ifdef RT_CAN_USING_HDR
        rt_list_init(&rx_fifo->buffer[i].hdrlist);
        rx_fifo->buffer[i].owner = RT_NULL;
#endif
    }
    can->can_rx = rx_fifo;
    return RT_EOK;
}

static rt_err_t rt_can_open(struct rt_device *dev, rt_uint16_t oflag)
{
    struct rt_can_device *can = (struct rt_can_device *)dev;
    struct rt_can_runtime *runtime;
    rt_err_t ret = RT_EOK;

    RT_ASSERT(can != RT_NULL);
    CAN_LOCK(can);
    dev->open_flag = oflag & 0xff;

    if (can->can_tx == RT_NULL)
    {
        runtime = (struct rt_can_runtime *)rt_calloc(1, sizeof(*runtime));
        if (runtime == RT_NULL)
        {
            ret = -RT_ENOMEM;
            goto __exit;
        }
        ret = rt_can_tx_runtime_init(can, runtime);
        if (ret != RT_EOK)
        {
            rt_free(runtime);
            goto __exit;
        }
        can->can_tx = runtime;
    }

    if ((oflag & RT_DEVICE_FLAG_INT_RX) && can->can_rx == RT_NULL)
    {
        ret = _can_rx_open(can);
        if (ret != RT_EOK)
            goto __exit;
        dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
        can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }

    if (oflag & RT_DEVICE_FLAG_INT_TX)
    {
        dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
        can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_TX);
    }
    can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_CAN_INT_ERR);

#ifdef RT_CAN_USING_HDR
    if (can->hdr == RT_NULL)
    {
        int i;
        can->hdr = (struct rt_can_hdr *)rt_calloc(can->config.maxhdr, sizeof(struct rt_can_hdr));
        if (can->hdr == RT_NULL)
        {
            ret = -RT_ENOMEM;
            goto __exit;
        }
        for (i = 0; i < can->config.maxhdr; i++)
            rt_list_init(&can->hdr[i].list);
    }
#endif

    if (!can->timerinitflag)
    {
        can->timerinitflag = 1;
        rt_timer_start(&can->timer);
    }

__exit:
    CAN_UNLOCK(can);
    return ret;
}

static rt_err_t rt_can_close(struct rt_device *dev)
{
    struct rt_can_device *can = (struct rt_can_device *)dev;
    struct rt_can_runtime *runtime;

    RT_ASSERT(can != RT_NULL);
    CAN_LOCK(can);
    if (dev->ref_count > 1)
    {
        CAN_UNLOCK(can);
        return RT_EOK;
    }

    if (can->timerinitflag)
    {
        can->timerinitflag = 0;
        rt_timer_stop(&can->timer);
    }
    can->status_indicate.ind = RT_NULL;
    can->status_indicate.args = RT_NULL;

#ifdef RT_CAN_USING_HDR
    rt_free(can->hdr);
    can->hdr = RT_NULL;
#endif

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        rt_free(can->can_rx);
        can->can_rx = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
    }
    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_TX);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
    }
    can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_CAN_INT_ERR);
    can->ops->control(can, RT_CAN_CMD_START, RT_FALSE);

    runtime = rt_can_runtime_get(can);
    can->can_tx = RT_NULL;
    rt_can_tx_runtime_deinit(runtime);
    rt_free(runtime);

    CAN_UNLOCK(can);
    return RT_EOK;
}

static rt_ssize_t rt_can_read(struct rt_device *dev, rt_off_t pos,
                              void *buffer, rt_size_t size)
{
    RT_UNUSED(pos);
    if (size == 0)
        return -RT_EINVAL;
    if ((dev->open_flag & RT_DEVICE_FLAG_INT_RX) && dev->ref_count > 0)
        return rt_can_rx_read_core((struct rt_can_device *)dev,
                                   (struct rt_can_msg *)buffer, (rt_ssize_t)size);
    return -RT_ENOSYS;
}

static rt_ssize_t rt_can_write(struct rt_device *dev, rt_off_t pos,
                               const void *buffer, rt_size_t size)
{
    struct rt_can_device *can = (struct rt_can_device *)dev;
    const struct rt_can_msg *msg = (const struct rt_can_msg *)buffer;
    rt_bool_t blocking;

    RT_UNUSED(pos);
    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if (size == 0 || size % sizeof(struct rt_can_msg) != 0)
        return -RT_EINVAL;
    if (dev->ref_count == 0 || !(dev->open_flag & RT_DEVICE_FLAG_INT_TX))
        return -RT_ENOSYS;

    blocking = (rt_interrupt_get_nest() == 0 && !msg->nonblocking) ? RT_TRUE : RT_FALSE;
    return rt_can_tx_write_core(can, msg, size, blocking);
}

static rt_err_t rt_can_control(struct rt_device *dev, int cmd, void *args)
{
    struct rt_can_device *can = (struct rt_can_device *)dev;
    rt_err_t ret = RT_EOK;

    RT_ASSERT(can != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;
    case RT_DEVICE_CTRL_RESUME:
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;
    case RT_DEVICE_CTRL_CONFIG:
        ret = can->ops->configure(can, (struct can_configure *)args);
        break;
    case RT_CAN_CMD_SET_PRIV:
        if (can->ops->control == RT_NULL)
            return -RT_ENOSYS;
        ret = can->ops->control(can, cmd, args);
        if (ret == RT_EOK)
            can->config.privmode = (rt_uint32_t)(rt_ubase_t)args;
        break;
    case RT_CAN_CMD_SET_STATUS_IND:
        can->status_indicate.ind = ((rt_can_status_ind_type_t)args)->ind;
        can->status_indicate.args = ((rt_can_status_ind_type_t)args)->args;
        break;
    case RT_CAN_CMD_FLUSH_TX:
        ret = rt_can_quiesce(can, RT_CAN_QUIESCE_ABORT_ALL, RT_CANSND_MSG_TIMEOUT);
        break;
    case RT_CAN_CMD_RESUME_TX:
        ret = rt_can_tx_resume(can);
        break;
    case RT_CAN_CMD_SET_BAUD:
        ret = rt_can_quiesce(can, RT_CAN_QUIESCE_ABORT_ALL, RT_CANSND_MSG_TIMEOUT);
        if (ret == RT_EOK)
        {
            ret = can->ops->control ? can->ops->control(can, cmd, args) : -RT_ENOSYS;
            if (ret == RT_EOK)
                ret = rt_can_tx_resume(can);
        }
        break;
#ifdef RT_CAN_USING_HDR
    case RT_CAN_CMD_SET_FILTER:
        ret = can->ops->control(can, cmd, args);
        break;
#endif
#ifdef RT_CAN_USING_BUS_HOOK
    case RT_CAN_CMD_SET_BUS_HOOK:
        can->bus_hook = (rt_can_bus_hook)args;
        break;
#endif
    default:
        ret = can->ops->control ? can->ops->control(can, cmd, args) : -RT_ENOSYS;
        break;
    }
    return ret;
}

static void cantimeout(void *arg)
{
    rt_can_t can = (rt_can_t)arg;

    RT_ASSERT(can != RT_NULL);
    rt_device_control((rt_device_t)can, RT_CAN_CMD_GET_STATUS, &can->status);
    if (can->status_indicate.ind)
        can->status_indicate.ind(can, can->status_indicate.args);
#ifdef RT_CAN_USING_BUS_HOOK
    if (can->bus_hook)
        can->bus_hook(can);
#endif
    if (can->timerinitflag == 1)
        can->timerinitflag = 0xff;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops can_device_ops =
{
    rt_can_init, rt_can_open, rt_can_close,
    rt_can_read, rt_can_write, rt_can_control,
};
#endif

rt_err_t rt_hw_can_register(struct rt_can_device *can, const char *name,
                            const struct rt_can_ops *ops, void *data)
{
    struct rt_device *device;

    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(ops != RT_NULL);
    device = &can->parent;
    device->type = RT_Device_Class_CAN;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
#ifdef RT_CAN_USING_HDR
    can->hdr = RT_NULL;
#endif
    can->can_rx = RT_NULL;
    can->can_tx = RT_NULL;
    rt_mutex_init(&can->lock, "can", RT_IPC_FLAG_PRIO);
#ifdef RT_CAN_USING_BUS_HOOK
    can->bus_hook = RT_NULL;
#endif
#ifdef RT_CAN_MALLOC_NB_TX_BUFFER
    can->nb_tx_rb_pool = RT_NULL;
#endif
#ifdef RT_USING_DEVICE_OPS
    device->ops = &can_device_ops;
#else
    device->init = rt_can_init;
    device->open = rt_can_open;
    device->close = rt_can_close;
    device->read = rt_can_read;
    device->write = rt_can_write;
    device->control = rt_can_control;
#endif
    can->ops = ops;
    can->status_indicate.ind = RT_NULL;
    can->status_indicate.args = RT_NULL;
    rt_memset(&can->status, 0, sizeof(can->status));
    device->user_data = data;
    can->timerinitflag = 0;
    rt_timer_init(&can->timer, name, cantimeout, can, can->config.ticks,
                  RT_TIMER_FLAG_PERIODIC);
    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
}

void rt_hw_can_isr(struct rt_can_device *can, int event)
{
    switch (event & 0xff)
    {
    case RT_CAN_EVENT_RXOF_IND:
        can->status.dropedrcvpkg++;
        rt_can_rx_isr_core(can, event, RT_TRUE);
        break;
    case RT_CAN_EVENT_RX_IND:
        rt_can_rx_isr_core(can, event, RT_FALSE);
        break;
    case RT_CAN_EVENT_TX_DONE:
    case RT_CAN_EVENT_TX_FAIL:
        rt_can_tx_isr_core(can, event);
        break;
    default:
        break;
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
static int cmd_canstat(int argc, char **argv)
{
    struct rt_can_status status;
    rt_device_t dev;

    if (argc < 2)
        return -RT_EINVAL;
    dev = rt_device_find(argv[1]);
    if (dev == RT_NULL)
        return -RT_ERROR;
    rt_device_control(dev, RT_CAN_CMD_GET_STATUS, &status);
    rt_kprintf("RX=%lu drop=%lu TX=%lu drop=%lu err=%lu\n",
               status.rcvpkg, status.dropedrcvpkg,
               status.sndpkg, status.dropedsndpkg, status.errcode);
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(cmd_canstat, canstat, stat can device status);
#endif
