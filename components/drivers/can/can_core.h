/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __RT_CAN_CORE_H__
#define __RT_CAN_CORE_H__
#include <rtthread.h>
#include <drivers/can_state.h>
struct rt_can_tx_request;
enum rt_can_lifecycle_state
{
    RT_CAN_LC_STOPPED = 0,
    RT_CAN_LC_RUNNING,
    RT_CAN_LC_QUIESCING,
    RT_CAN_LC_QUIESCED,
    RT_CAN_LC_RECONFIGURING,
    RT_CAN_LC_RECOVERING,
    RT_CAN_LC_CLOSING,
};
struct rt_can_tx_core
{
    struct rt_spinlock lock;
    struct rt_list_node free_list;
    struct rt_list_node pending_list;
    struct rt_can_tx_request *requests;
    struct rt_can_tx_request **mailbox_owner;
    rt_uint16_t request_count;
    rt_uint16_t mailbox_count;
    rt_bool_t scheduling;
};
struct rt_can_rx_core { struct rt_spinlock lock; };
struct rt_can_core
{
    struct rt_mutex lifecycle_lock;
    enum rt_can_lifecycle_state lifecycle_state;
    enum rt_can_bus_state bus_state;
    struct rt_can_tx_core tx;
    struct rt_can_rx_core rx;
};
static rt_inline rt_err_t rt_can_core_init(struct rt_can_core *core, const char *name)
{
    rt_err_t ret;
    ret = rt_mutex_init(&core->lifecycle_lock, name, RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK) return ret;
    rt_spin_lock_init(&core->tx.lock);
    rt_spin_lock_init(&core->rx.lock);
    rt_list_init(&core->tx.free_list);
    rt_list_init(&core->tx.pending_list);
    core->tx.requests = RT_NULL;
    core->tx.mailbox_owner = RT_NULL;
    core->tx.request_count = 0;
    core->tx.mailbox_count = 0;
    core->tx.scheduling = RT_FALSE;
    core->lifecycle_state = RT_CAN_LC_STOPPED;
    core->bus_state = RT_CAN_BUS_UNKNOWN;
    return RT_EOK;
}
#endif
