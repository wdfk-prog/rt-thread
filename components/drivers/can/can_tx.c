/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_internal.h"

struct tx_notify
{
    struct rt_can_tx_request *req;
    rt_bool_t wake_waiter;
    rt_bool_t auto_release;
    rt_can_tx_done_cb callback;
    void *callback_arg;
    struct rt_can_msg msg;
    rt_err_t result;
};

static rt_bool_t _tx_terminal(enum rt_can_tx_req_state state)
{
    return state == RT_CAN_TX_REQ_DONE || state == RT_CAN_TX_REQ_ERROR ||
           state == RT_CAN_TX_REQ_ABORTED || state == RT_CAN_TX_REQ_CANCELLED;
}

static void _tx_release_locked(struct rt_can_tx_core *tx, struct rt_can_tx_request *req)
{
    req->state = RT_CAN_TX_REQ_FREE;
    req->result = RT_EOK;
    req->mailbox = -1;
    req->blocking = RT_FALSE;
    req->waiter_attached = RT_FALSE;
    req->callback = RT_NULL;
    req->callback_arg = RT_NULL;
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
    req->callback = RT_NULL;
    req->callback_arg = RT_NULL;
    return req;
}

static rt_int16_t _tx_find_mailbox_locked(struct rt_can_device *can,
                                          struct rt_can_tx_core *tx,
                                          const struct rt_can_tx_request *req)
{
    rt_uint16_t i;

    /*
     * Strict wire order is the generic fallback. Unless the driver explicitly
     * reports ordered multi-mailbox transmission, keep at most one request in
     * hardware so a later CAN ID cannot overtake an earlier request locally.
     */
    if (!(rt_can_runtime_get(can)->capabilities & RT_CAN_CAP_TX_ORDERED_MAILBOX))
    {
        for (i = 0; i < tx->mailbox_count; i++)
        {
            if (tx->mailbox_owner[i] != RT_NULL)
                return -1;
        }
    }

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

static rt_bool_t _tx_mark_terminal_locked(struct rt_can_tx_core *tx,
                                          struct rt_can_tx_request *req,
                                          enum rt_can_tx_req_state terminal_state,
                                          rt_err_t result,
                                          struct tx_notify *notify)
{
    if (_tx_terminal(req->state) || req->state == RT_CAN_TX_REQ_FREE)
        return RT_FALSE;

    if (req->mailbox >= 0 && req->mailbox < tx->mailbox_count &&
        tx->mailbox_owner[req->mailbox] == req)
        tx->mailbox_owner[req->mailbox] = RT_NULL;

    if (!rt_list_isempty(&req->node))
        rt_list_remove(&req->node);
    req->mailbox = -1;
    req->result = result;
    req->state = terminal_state;

    notify->req = req;
    notify->result = result;
    notify->wake_waiter = req->blocking && req->waiter_attached;
    notify->auto_release = !req->blocking || !req->waiter_attached;
    notify->callback = req->callback;
    notify->callback_arg = req->callback_arg;
    notify->msg = req->msg;
    return RT_TRUE;
}

static void _tx_notify(struct rt_can_device *can, struct tx_notify *notify)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx;
    rt_base_t level;

    if (notify->req == RT_NULL || runtime == RT_NULL)
        return;
    tx = &runtime->core.tx;

    if (notify->wake_waiter)
        rt_completion_done(&notify->req->completion);
    if (notify->callback)
        notify->callback(can, &notify->msg, notify->result, notify->callback_arg);

    if (notify->auto_release)
    {
        level = rt_spin_lock_irqsave(&tx->lock);
        if (_tx_terminal(notify->req->state))
            _tx_release_locked(tx, notify->req);
        rt_spin_unlock_irqrestore(&tx->lock, level);
    }
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
        struct tx_notify notify = {0};
        rt_int16_t mailbox;
        rt_ssize_t ret;

        level = rt_spin_lock_irqsave(&tx->lock);
        if ((runtime->core.lifecycle_state != RT_CAN_LC_RUNNING &&
             !(runtime->core.lifecycle_state == RT_CAN_LC_QUIESCING &&
               runtime->quiesce_policy == RT_CAN_QUIESCE_DRAIN)) ||
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

        _tx_mark_terminal_locked(tx, req, RT_CAN_TX_REQ_ERROR, (rt_err_t)ret, &notify);
        can->status.dropedsndpkg++;
        rt_spin_unlock_irqrestore(&tx->lock, level);
        _tx_notify(can, &notify);
    }
}

static struct rt_can_tx_request *_tx_enqueue(struct rt_can_device *can,
                                             const struct rt_can_msg *msg,
                                             rt_bool_t blocking,
                                             rt_can_tx_done_cb callback,
                                             void *callback_arg)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx;
    struct rt_can_tx_request *req;
    rt_base_t level;

    if (runtime == RT_NULL || runtime->core.lifecycle_state != RT_CAN_LC_RUNNING)
        return RT_NULL;
    tx = &runtime->core.tx;

    level = rt_spin_lock_irqsave(&tx->lock);
    req = _tx_alloc_locked(tx);
    if (req)
    {
        req->msg = *msg;
        req->blocking = blocking;
        req->waiter_attached = blocking;
        req->callback = callback;
        req->callback_arg = callback_arg;
        rt_completion_init(&req->completion);
        rt_list_insert_before(&tx->pending_list, &req->node);
    }
    rt_spin_unlock_irqrestore(&tx->lock, level);
    return req;
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

    req = _tx_enqueue(can, msg, blocking, RT_NULL, RT_NULL);
    if (req == RT_NULL)
    {
        can->status.dropedsndpkg++;
        return -RT_EFULL;
    }
    tx = &runtime->core.tx;
    _tx_schedule(can);

    if (!blocking)
        return sizeof(struct rt_can_msg);

    wait_ret = rt_completion_wait(&req->completion, RT_CANSND_MSG_TIMEOUT);
    if (wait_ret != RT_EOK)
    {
        /* A caller timeout only detaches the waiter; hardware ownership stays. */
        level = rt_spin_lock_irqsave(&tx->lock);
        if (!_tx_terminal(req->state))
        {
            req->waiter_attached = RT_FALSE;
            rt_spin_unlock_irqrestore(&tx->lock, level);
            can->status.dropedsndpkg++;
            return wait_ret;
        }
        rt_spin_unlock_irqrestore(&tx->lock, level);
    }

    level = rt_spin_lock_irqsave(&tx->lock);
    if (!_tx_terminal(req->state))
    {
        rt_spin_unlock_irqrestore(&tx->lock, level);
        return wait_ret;
    }
    result = req->result;
    req->waiter_attached = RT_FALSE;
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

rt_err_t rt_can_tx_submit_async_core(struct rt_can_device *can,
                                     const struct rt_can_msg *msg,
                                     rt_can_tx_done_cb callback,
                                     void *arg)
{
    if (can == RT_NULL || msg == RT_NULL)
        return -RT_EINVAL;
    if (_tx_enqueue(can, msg, RT_FALSE, callback, arg) == RT_NULL)
        return -RT_EFULL;
    _tx_schedule(can);
    return RT_EOK;
}

rt_err_t rt_can_send_async(struct rt_can_device *can,
                           const struct rt_can_msg *msg,
                           rt_can_tx_done_cb callback,
                           void *arg)
{
    return rt_can_tx_submit_async_core(can, msg, callback, arg);
}

static rt_bool_t _tx_idle_locked(struct rt_can_tx_core *tx)
{
    rt_uint16_t i;

    if (!rt_list_isempty(&tx->pending_list) || tx->scheduling)
        return RT_FALSE;
    for (i = 0; i < tx->mailbox_count; i++)
    {
        if (tx->mailbox_owner[i] != RT_NULL)
            return RT_FALSE;
    }
    return RT_TRUE;
}

static void _tx_cancel_queued(struct rt_can_device *can)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx = &runtime->core.tx;

    for (;;)
    {
        struct rt_can_tx_request *req;
        struct tx_notify notify = {0};
        rt_base_t level = rt_spin_lock_irqsave(&tx->lock);

        if (rt_list_isempty(&tx->pending_list))
        {
            rt_spin_unlock_irqrestore(&tx->lock, level);
            return;
        }
        req = rt_list_entry(tx->pending_list.next, struct rt_can_tx_request, node);
        _tx_mark_terminal_locked(tx, req, RT_CAN_TX_REQ_CANCELLED, -RT_ERROR, &notify);
        rt_spin_unlock_irqrestore(&tx->lock, level);
        _tx_notify(can, &notify);
    }
}

static void _tx_abort_owned(struct rt_can_device *can)
{
    struct rt_can_runtime *runtime = rt_can_runtime_get(can);
    struct rt_can_tx_core *tx = &runtime->core.tx;
    rt_uint16_t i;

    if (!(runtime->capabilities & RT_CAN_CAP_TX_ABORT) || can->ops->control == RT_NULL)
        return;

    for (i = 0; i < tx->mailbox_count; i++)
    {
        struct rt_can_tx_request *req;
        rt_base_t level = rt_spin_lock_irqsave(&tx->lock);

        req = tx->mailbox_owner[i];
        if (req != RT_NULL && req->state == RT_CAN_TX_REQ_HW_PENDING)
            req->state = RT_CAN_TX_REQ_ABORTING;
        else
            req = RT_NULL;
        rt_spin_unlock_irqrestore(&tx->lock, level);

        if (req != RT_NULL)
        {
            /*
             * A successful abort request is not itself terminal. The driver
             * must later report TX_DONE/TX_FAIL for this mailbox; ABORTING is
             * retired exactly once by the common terminal path.
             */
            if (can->ops->control(can, RT_CAN_CMD_ABORT_TX,
                                  (void *)(rt_ubase_t)i) != RT_EOK)
            {
                level = rt_spin_lock_irqsave(&tx->lock);
                if (req->state == RT_CAN_TX_REQ_ABORTING)
                    req->state = RT_CAN_TX_REQ_HW_PENDING;
                rt_spin_unlock_irqrestore(&tx->lock, level);
            }
        }
    }
}

rt_err_t rt_can_quiesce(struct rt_can_device *can,
                        enum rt_can_quiesce_policy policy,
                        rt_tick_t timeout)
{
    struct rt_can_runtime *runtime;
    struct rt_can_tx_core *tx;
    rt_tick_t start;
    rt_base_t level;
    rt_err_t ret = RT_EOK;

    if (can == RT_NULL || rt_interrupt_get_nest() > 0)
        return -RT_EINVAL;
    runtime = rt_can_runtime_get(can);
    if (runtime == RT_NULL)
        return -RT_EINVAL;

    rt_mutex_take(&runtime->core.lifecycle_lock, RT_WAITING_FOREVER);
    tx = &runtime->core.tx;
    level = rt_spin_lock_irqsave(&tx->lock);
    if (runtime->core.lifecycle_state != RT_CAN_LC_RUNNING)
    {
        rt_spin_unlock_irqrestore(&tx->lock, level);
        rt_mutex_release(&runtime->core.lifecycle_lock);
        return -RT_EBUSY;
    }
    runtime->quiesce_policy = policy;
    runtime->core.lifecycle_state = RT_CAN_LC_QUIESCING;
    rt_spin_unlock_irqrestore(&tx->lock, level);

    if (policy == RT_CAN_QUIESCE_DRAIN)
        _tx_schedule(can);
    else
        _tx_cancel_queued(can);
    if (policy == RT_CAN_QUIESCE_ABORT_ALL)
        _tx_abort_owned(can);

    start = rt_tick_get();
    for (;;)
    {
        rt_bool_t idle;

        level = rt_spin_lock_irqsave(&tx->lock);
        idle = _tx_idle_locked(tx);
        if (idle)
            runtime->core.lifecycle_state = RT_CAN_LC_QUIESCED;
        rt_spin_unlock_irqrestore(&tx->lock, level);
        if (idle)
            break;
        if (timeout != RT_WAITING_FOREVER && (rt_tick_get() - start) >= timeout)
        {
            ret = -RT_ETIMEOUT;
            break;
        }
        rt_thread_mdelay(1);
    }

    rt_mutex_release(&runtime->core.lifecycle_lock);
    return ret;
}

rt_err_t rt_can_tx_resume(struct rt_can_device *can)
{
    struct rt_can_runtime *runtime;
    struct rt_can_tx_core *tx;
    rt_base_t level;

    if (can == RT_NULL || rt_interrupt_get_nest() > 0)
        return -RT_EINVAL;
    runtime = rt_can_runtime_get(can);
    if (runtime == RT_NULL)
        return -RT_EINVAL;

    rt_mutex_take(&runtime->core.lifecycle_lock, RT_WAITING_FOREVER);
    tx = &runtime->core.tx;
    level = rt_spin_lock_irqsave(&tx->lock);
    if (runtime->core.lifecycle_state != RT_CAN_LC_QUIESCED)
    {
        rt_spin_unlock_irqrestore(&tx->lock, level);
        rt_mutex_release(&runtime->core.lifecycle_lock);
        return -RT_EBUSY;
    }
    runtime->core.lifecycle_state = RT_CAN_LC_RUNNING;
    rt_spin_unlock_irqrestore(&tx->lock, level);
    rt_mutex_release(&runtime->core.lifecycle_lock);
    _tx_schedule(can);
    return RT_EOK;
}

void rt_can_tx_schedule_core(struct rt_can_device *can)
{
    _tx_schedule(can);
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
    runtime->capabilities = 0;
    runtime->quiesce_policy = RT_CAN_QUIESCE_DRAIN;
    if (can->ops->control != RT_NULL)
        can->ops->control(can, RT_CAN_CMD_GET_CAPABILITIES, &runtime->capabilities);

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
    struct tx_notify notify = {0};
    rt_uint32_t mailbox = (rt_uint32_t)event >> 8;
    rt_base_t level;
    rt_err_t result;

    if (runtime == RT_NULL)
        return;
    tx = &runtime->core.tx;
    if (mailbox >= tx->mailbox_count)
        return;

    level = rt_spin_lock_irqsave(&tx->lock);
    req = tx->mailbox_owner[mailbox];
    if (req == RT_NULL)
    {
        /* Stale or duplicate completion: it cannot affect a new request. */
        rt_spin_unlock_irqrestore(&tx->lock, level);
        return;
    }

    result = ((event & 0xff) == RT_CAN_EVENT_TX_DONE) ? RT_EOK : -RT_ERROR;
    if (!_tx_mark_terminal_locked(tx, req,
                                  req->state == RT_CAN_TX_REQ_ABORTING ? RT_CAN_TX_REQ_ABORTED :
                                  (result == RT_EOK ? RT_CAN_TX_REQ_DONE : RT_CAN_TX_REQ_ERROR),
                                  result, &notify))
    {
        rt_spin_unlock_irqrestore(&tx->lock, level);
        return;
    }
    if (!req->blocking)
    {
        if (result == RT_EOK)
            can->status.sndpkg++;
        else
            can->status.dropedsndpkg++;
    }
    rt_spin_unlock_irqrestore(&tx->lock, level);

    _tx_notify(can, &notify);
    _tx_schedule(can);
}
