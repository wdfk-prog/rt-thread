/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_internal.h"

rt_ssize_t rt_can_rx_read_core(struct rt_can_device *can, struct rt_can_msg *data,
                               rt_ssize_t msgs)
{
    rt_ssize_t size = msgs;
    struct rt_can_rx_fifo *rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;

    RT_ASSERT(rx_fifo != RT_NULL);
    while (msgs / sizeof(struct rt_can_msg) > 0)
    {
        struct rt_can_msg_list *entry = RT_NULL;
        rt_base_t level = rt_hw_local_irq_disable();
#ifdef RT_CAN_USING_HDR
        rt_int8_t hdr = data->hdr_index;
        if (hdr >= 0 && can->hdr && hdr < can->config.maxhdr &&
            !rt_list_isempty(&can->hdr[hdr].list))
        {
            entry = rt_list_entry(can->hdr[hdr].list.next, struct rt_can_msg_list, hdrlist);
            rt_list_remove(&entry->list);
            rt_list_remove(&entry->hdrlist);
            if (can->hdr[hdr].msgs)
                can->hdr[hdr].msgs--;
            entry->owner = RT_NULL;
        }
        else if (hdr == -1)
#endif
        {
            if (!rt_list_isempty(&rx_fifo->uselist))
            {
                entry = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
                rt_list_remove(&entry->list);
#ifdef RT_CAN_USING_HDR
                rt_list_remove(&entry->hdrlist);
                if (entry->owner && entry->owner->msgs)
                    entry->owner->msgs--;
                entry->owner = RT_NULL;
#endif
            }
        }
        rt_hw_local_irq_enable(level);
        if (entry == RT_NULL)
            break;

        rt_memcpy(data, &entry->data, sizeof(*data));
        level = rt_hw_local_irq_disable();
        rt_list_insert_before(&rx_fifo->freelist, &entry->list);
        rx_fifo->freenumbers++;
        rt_hw_local_irq_enable(level);
        data++;
        msgs -= sizeof(struct rt_can_msg);
    }
    return size - msgs;
}

void rt_can_rx_isr_core(struct rt_can_device *can, int event, rt_bool_t overflow)
{
    struct rt_can_msg msg;
    struct rt_can_rx_fifo *rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;
    struct rt_can_msg_list *entry = RT_NULL;
    rt_base_t level;
    rt_uint32_t fifo = (rt_uint32_t)event >> 8;
#ifdef RT_CAN_USING_HDR
    rt_int8_t hdr = -1;
#endif

    if (rx_fifo == RT_NULL || can->ops->recvmsg(can, &msg, fifo) < 0)
        return;

    level = rt_hw_local_irq_disable();
    can->status.rcvpkg++;
    can->status.rcvchange = 1;
    if (!rt_list_isempty(&rx_fifo->freelist))
    {
        entry = rt_list_entry(rx_fifo->freelist.next, struct rt_can_msg_list, list);
        rt_list_remove(&entry->list);
        rx_fifo->freenumbers--;
    }
    else if (!rt_list_isempty(&rx_fifo->uselist))
    {
        entry = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
        rt_list_remove(&entry->list);
        if (!overflow)
            can->status.dropedrcvpkg++;
#ifdef RT_CAN_USING_HDR
        rt_list_remove(&entry->hdrlist);
        if (entry->owner && entry->owner->msgs)
            entry->owner->msgs--;
        entry->owner = RT_NULL;
#endif
    }
    rt_hw_local_irq_enable(level);

    if (entry == RT_NULL)
    {
        can->status.dropedrcvpkg++;
        return;
    }

    rt_memcpy(&entry->data, &msg, sizeof(msg));
    level = rt_hw_local_irq_disable();
    rt_list_insert_before(&rx_fifo->uselist, &entry->list);
#ifdef RT_CAN_USING_HDR
    hdr = msg.hdr_index;
    if (can->hdr && hdr >= 0 && hdr < can->config.maxhdr && can->hdr[hdr].connected)
    {
        rt_list_insert_before(&can->hdr[hdr].list, &entry->hdrlist);
        entry->owner = &can->hdr[hdr];
        can->hdr[hdr].msgs++;
    }
#endif
    rt_hw_local_irq_enable(level);

#ifdef RT_CAN_USING_HDR
    if (can->hdr && hdr >= 0 && hdr < can->config.maxhdr && can->hdr[hdr].connected &&
        can->hdr[hdr].filter.ind)
    {
        can->hdr[hdr].filter.ind(&can->parent, can->hdr[hdr].filter.args, hdr,
                                 can->hdr[hdr].msgs * sizeof(struct rt_can_msg));
        return;
    }
#endif
    if (can->parent.rx_indicate)
        can->parent.rx_indicate(&can->parent,
                                rt_list_len(&rx_fifo->uselist) * sizeof(struct rt_can_msg));
}
