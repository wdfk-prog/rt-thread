/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RT_CAN_CORE_H__
#define __RT_CAN_CORE_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

struct rt_can_tx_request;

/**
 * @brief CAN framework lifecycle state.
 *
 * Lifecycle state describes whether the framework may accept new transactions
 * and whether controller resources are safe to reconfigure or release. It is
 * independent from the CAN protocol error state.
 */
enum rt_can_lifecycle_state
{
    RT_CAN_LC_STOPPED = 0,      /**< Controller/framework is stopped. */
    RT_CAN_LC_RUNNING,          /**< Normal RX/TX operation. */
    RT_CAN_LC_QUIESCING,        /**< New TX is blocked while accepted TX is being retired. */
    RT_CAN_LC_QUIESCED,         /**< TX is fully silent and no accepted TX is active. */
    RT_CAN_LC_RECONFIGURING,    /**< Controller configuration is being changed. */
    RT_CAN_LC_RECOVERING,       /**< Controller/bus recovery is in progress. */
    RT_CAN_LC_CLOSING,          /**< Device resources are being shut down. */
};

/**
 * @brief Internal TX synchronization and ownership domain.
 */
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

/**
 * @brief Internal RX synchronization domain.
 */
struct rt_can_rx_core
{
    struct rt_spinlock lock;
};

/**
 * @brief Common CAN framework core state.
 */
struct rt_can_core
{
    struct rt_mutex lifecycle_lock;
    enum rt_can_lifecycle_state lifecycle_state;
    struct rt_can_tx_core tx;
    struct rt_can_rx_core rx;
};

/**
 * @brief Initialize the common CAN core synchronization state.
 *
 * This helper only initializes framework-owned synchronization and list state.
 * It does not allocate TX requests, change controller configuration, or alter
 * existing CAN public behavior.
 *
 * @param core CAN core object to initialize.
 * @param name Mutex object name used for lifecycle serialization.
 * @return RT_EOK on success, or a negative RT-Thread error code.
 */
static rt_inline rt_err_t rt_can_core_init(struct rt_can_core *core, const char *name)
{
    rt_err_t ret;

    RT_ASSERT(core != RT_NULL);
    RT_ASSERT(name != RT_NULL);

    ret = rt_mutex_init(&core->lifecycle_lock, name, RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK)
    {
        return ret;
    }

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

    return RT_EOK;
}

#ifdef __cplusplus
}
#endif

#endif /* __RT_CAN_CORE_H__ */
