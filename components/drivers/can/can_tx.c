/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_internal.h"

static rt_bool_t _tx_terminal(enum rt_can_tx_req_state state)
{
    return state == RT_CAN_TX_REQ_DONE || state == RT_CAN_TX_REQ_ERROR;
}

static void _tx_release_locked(struct rt_can_tx_core *tx, struct rt_can_tx_request *req)
{
    req->state = RT_CAN_TX_REQ_FREE;
    req->result = RT_EOK;
    req->mailbox = -1;
    req->blocking = RT_FALSE;
    rt_list_insert_before(&tx->free_list, &req->node);
}

static struct rt_can_tx_request *_tx_alloc_locked(struct rt_can_tx_core *tx)
{
    struct rt_can_tx_request *req;

    if (rt_list_isempty(&tx->free_list))
        return RT_NULL;

    req = rt_list_entry(tx->free_list.next, struct rt_can_tx_request, node);
    rt_list_remove(&req->node);
    req->state = RT_CAN_TX_REQ_QUEUED;
    req->result = RT_EOK;
    req->mailbox = -1;
    return req;
}

static rt_int16_t _tx_find_mailbox_locked(struct rt_can_device *can,
                                          struct rt_can_tx_core *tx,
                                          const struct rt_can_tx_request *req)
{
    rt_uint16_t i;

    if (can->config.privmode)
    {
        if (req->msg.priv >= tx->mailbox_count)
            return -1;
        return tx->mailbox_owner[req->msg.priv] == RT_NULL ?
               (rt_int16_t)req->msg.priv : -1;
    }

    for (i = 0; i < tx->mailbox_count; i++)
    {
        if (tx->mailbox_owner[i] == RT_NULL)
            return (rt_int16_t)i;
    }
    return -1;
}

static void _tx_schedule(struct rt_can_device *can)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx;
    rt_base_t level;

    if (runtime == RT_NULL || can->ops->sendmsg == RT_NULL)
        return;

    tx = &runtime->core.tx;
    level = rt_spin_lock_irqsave(&tx->lock);
    if (tx->scheduling)
    {
        rt_spin_unlock_irqrestore(&tx->lock, level);
        return;
    }
    tx->scheduling = RT_TRUE;
    rt_spin_unlock_irqrestore(&tx->lock, level);

    for (;;)
    {
        struct rt_can_tx_request *req;
        rt_int16_t mailbox;
        rt_ssize_t ret;
        rt_bool_t wake = RT_FALSE;

        level = rt_spin_lock_irqsave(&tx->lock);
        if (runtime->core.lifecycle_state != RT_CAN_LC_RUNNING ||
            rt_list_isempty(&tx->pending_list))
        {
            tx->scheduling = RT_FALSE;
            rt_spin_unlock_irqrestore(&tx->lock, level);
            return;
        }

        req = rt_list_entry(tx->pending_list.next, struct rt_can_tx_request, node);
        mailbox = _tx_find_mailbox_locked(can, tx, req);
        if (mailbox < 0)
        {
            tx->scheduling = RT_FALSE;
            rt_spin_unlock_irqrestore(&tx->lock, level);
            return;
        }

        rt_list_remove(&req->node);
        req->mailbox = mailbox;
        req->state = RT_CAN_TX_REQ_SUBMITTING;
        tx->mailbox_owner[mailbox] = req;
        rt_spin_unlock_irqrestore(&tx->lock, level);

        /*
         * Unified driver contract: when non-blocking TX is enabled, sendmsg()
         * is an ISR-safe, non-sleeping hardware submit operation for the
         * framework-selected mailbox.
         */
        ret = can->ops->sendmsg(can, &req->msg, (rt_uint32_t)mailbox);

        level = rt_spin_lock_irqsave(&tx->lock);
        if (req->state != RT_CAN_TX_REQ_SUBMITTING)
        {
            rt_spin_unlock_irqrestore(&tx->lock, level);
            continue;
        }
        if (ret == RT_EOK)
        {
            req->state = RT_CAN_TX_REQ_HW_PENDING;
            rt_spin_unlock_irqrestore(&tx->lock, level);
            continue;
        }

        if (tx->mailbox_owner[mailbox] == req)
            tx->mailbox_owner[mailbox] = RT_NULL;
        req->mailbox = -1;
        req->result = (rt_err_t)ret;
        req->state = RT_CAN_TX_REQ_ERROR;
        if (req->blocking)
            wake = RT_TRUE;
        else
        {
            _tx_release_locked(tx, req);
            can->status.dropedsndpkg++;
        }
        rt_spin_unlock_irqrestore(&tx->lock, level);
        if (wake)
            rt_completion_done(&req->completion);
    }
}

static rt_ssize_t _tx_submit_one(struct rt_can_device *can,
                                 const struct rt_can_msg *msg,
                                 rt_bool_t blocking)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx;
    struct rt_can_tx_request *req;
    rt_base_t level;
    rt_err_t wait_ret;
    rt_err_t result;

    if (runtime == RT_NULL || runtime->core.lifecycle_state != RT_CAN_LC_RUNNING)
        return -RT_EBUSY;

    tx = &runtime->core.tx;
    level = rt_spin_lock_irqsave(&tx->lock);
    req = _tx_alloc_locked(tx);
    if (req == RT_NULL)
    {
        can->status.dropedsndpkg++;
        rt_spin_unlock_irqrestore(&tx->lock, level);
        return -RT_EFULL;
    }

    req->msg = *msg;
    req->blocking = blocking;
    rt_completion_init(&req->completion);
    rt_list_insert_before(&tx->pending_list, &req->node);
    rt_spin_unlock_irqrestore(&tx->lock, level);

    _tx_schedule(can);
    if (!blocking)
        return sizeof(struct rt_can_msg);

    wait_ret = rt_completion_wait(&req->completion, RT_CANSND_MSG_TIMEOUT);
    if (wait_ret != RT_EOK)
    {
        /* PR3 changes timeout to detach the waiter without retiring ownership. */
        level = rt_spin_lock_irqsave(&tx->lock);
        if (!_tx_terminal(req->state))
        {
            if (req->mailbox >= 0 && tx->mailbox_owner[req->mailbox] == req)
                tx->mailbox_owner[req->mailbox] = RT_NULL;
            if (!rt_list_isempty(&req->node))
                rt_list_remove(&req->node);
            _tx_release_locked(tx, req);
        }
        rt_spin_unlock_irqrestore(&tx->lock, level);
        can->status.dropedsndpkg++;
        return wait_ret;
    }

    level = rt_spin_lock_irqsave(&tx->lock);
    result = req->result;
    _tx_release_locked(tx, req);
    rt_spin_unlock_irqrestore(&tx->lock, level);

    if (result == RT_EOK)
    {
        can->status.sndpkg++;
        return sizeof(struct rt_can_msg);
    }
    can->status.dropedsndpkg++;
    return result;
}

rt_ssize_t rt_can_tx_write_core(struct rt_can_device *can, const struct rt_can_msg *msg,
                                rt_size_t size, rt_bool_t blocking)
{
    rt_ssize_t done = 0;

    while (done < (rt_ssize_t)size)
    {
        rt_ssize_t ret = _tx_submit_one(can, msg, blocking);
        if (ret != sizeof(struct rt_can_msg))
            break;
        done += ret;
        msg++;
    }
    return done;
}

rt_err_t rt_can_tx_runtime_init(struct rt_can_device *can, struct rt_can_runtime *runtime)
{
    struct rt_can_tx_core *tx = &runtime->core.tx;
    rt_uint16_t i;
    rt_err_t ret;

    ret = rt_can_core_init(&runtime->core, "canlc");
    if (ret != RT_EOK)
        return ret;

    tx->request_count = RT_CAN_TX_REQUEST_COUNT;
    tx->mailbox_count = (rt_uint16_t)can->config.sndboxnumber;
    if (tx->mailbox_count == 0)
        tx->mailbox_count = 1;

    tx->requests = (struct rt_can_tx_request *)rt_calloc(tx->request_count,
                                                         sizeof(struct rt_can_tx_request));
    if (tx->requests == RT_NULL)
        goto __nomem;

    tx->mailbox_owner = (struct rt_can_tx_request **)rt_calloc(tx->mailbox_count,
                                                              sizeof(struct rt_can_tx_request *));
    if (tx->mailbox_owner == RT_NULL)
    {
        rt_free(tx->requests);
        tx->requests = RT_NULL;
        goto __nomem;
    }

    for (i = 0; i < tx->request_count; i++)
    {
        tx->requests[i].state = RT_CAN_TX_REQ_FREE;
        tx->requests[i].mailbox = -1;
        rt_list_init(&tx->requests[i].node);
        rt_completion_init(&tx->requests[i].completion);
        rt_list_insert_before(&tx->free_list, &tx->requests[i].node);
    }
    runtime->core.lifecycle_state = RT_CAN_LC_RUNNING;
    return RT_EOK;

__nomem:
    rt_mutex_detach(&runtime->core.lifecycle_lock);
    return -RT_ENOMEM;
}

void rt_can_tx_runtime_deinit(struct rt_can_runtime *runtime)
{
    if (runtime == RT_NULL)
        return;

    runtime->core.lifecycle_state = RT_CAN_LC_STOPPED;
    rt_free(runtime->core.tx.mailbox_owner);
    rt_free(runtime->core.tx.requests);
    rt_mutex_detach(&runtime->core.lifecycle_lock);
}

void rt_can_tx_isr_core(struct rt_can_device *can, int event)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx;
    struct rt_can_tx_request *req;
    rt_uint32_t mailbox = (rt_uint32_t)event >> 8;
    rt_bool_t blocking;
    rt_base_t level;

    if (runtime == RT_NULL)
        return;
    tx = &runtime->core.tx;
    if (mailbox >= tx->mailbox_count)
        return;

    level = rt_spin_lock_irqsave(&tx->lock);
    req = tx->mailbox_owner[mailbox];
    if (req == RT_NULL)
    {
        rt_spin_unlock_irqrestore(&tx->lock, level);
        return;
    }

    tx->mailbox_owner[mailbox] = RT_NULL;
    req->mailbox = -1;
    req->result = ((event & 0xff) == RT_CAN_EVENT_TX_DONE) ? RT_EOK : -RT_ERROR;
    req->state = req->result == RT_EOK ? RT_CAN_TX_REQ_DONE : RT_CAN_TX_REQ_ERROR;
    blocking = req->blocking;

    if (!blocking)
    {
        if (req->result == RT_EOK)
            can->status.sndpkg++;
        else
            can->status.dropedsndpkg++;
        _tx_release_locked(tx, req);
    }
    rt_spin_unlock_irqrestore(&tx->lock, level);

    if (blocking)
        rt_completion_done(&req->completion);

    /* TX_DONE/TX_FAIL directly advances the queue in ISR context. */
    _tx_schedule(can);
}
