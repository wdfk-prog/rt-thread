/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2015-05-14     aubrcool@qq.com   first version
 * 2015-07-06     Bernard           code cleanup and remove RT_CAN_USING_LED;
 * 2025-09-20     wdfk_prog         Implemented non-blocking, ISR-safe send logic unified under rt_device_write.
 * 2026-08-26     wdfk_prog         Add TX gate and full flush support.
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define CAN_LOCK(can)   rt_mutex_take(&(can->lock), RT_WAITING_FOREVER)
#define CAN_UNLOCK(can) rt_mutex_release(&(can->lock))

enum rt_can_tx_state
{
    RT_CAN_TX_STATE_DISABLED = 0, /**< New TX submissions are rejected. */
    RT_CAN_TX_STATE_ENABLED,      /**< New TX submissions are allowed. */
    RT_CAN_TX_STATE_FLUSHING,     /**< An exclusive TX quiesce operation is in progress; new submissions are rejected. */
};

/**
 * @brief Enter the TX submission critical section if the TX gate is open.
 *
 * The reference count is incremented before checking the gate. This ordering
 * ensures that a concurrent flush either observes this sender in the reference
 * count or causes the sender to observe the closed gate and back out without
 * submitting to hardware or the software TX queue.
 *
 * @param[in] can A pointer to the CAN device.
 * @return RT_TRUE if the caller may submit TX data, otherwise RT_FALSE.
 */
static rt_bool_t _can_tx_submit_begin(struct rt_can_device *can)
{
    rt_atomic_add(&can->tx_submit_refcnt, 1);

    if (rt_atomic_load(&can->tx_state) != RT_CAN_TX_STATE_ENABLED)
    {
        rt_atomic_sub(&can->tx_submit_refcnt, 1);
        return RT_FALSE;
    }

    return RT_TRUE;
}

/**
 * @brief Leave the TX submission critical section.
 *
 * @param[in] can A pointer to the CAN device.
 */
static void _can_tx_submit_end(struct rt_can_device *can)
{
    rt_atomic_sub(&can->tx_submit_refcnt, 1);
}

/**
 * @brief Open or close the framework TX admission gate.
 *
 * This function only controls admission of new TX submissions. It does not
 * wait for an already active submission and does not abort hardware TX.
 * Enabling TX is rejected while an exclusive TX quiesce operation is in progress.
 * The caller must provide any required device-lifecycle serialization.
 *
 * @param[in] can     A pointer to the CAN device.
 * @param[in] enabled RT_TRUE to enable new submissions, RT_FALSE to disable them.
 * @return RT_EOK on success, or -RT_EBUSY when enabling during a quiesce operation.
 */
static rt_err_t _can_set_tx_enabled(struct rt_can_device *can, rt_bool_t enabled)
{
    rt_base_t expected;
    rt_base_t desired = enabled ? RT_CAN_TX_STATE_ENABLED : RT_CAN_TX_STATE_DISABLED;

    while (RT_TRUE)
    {
        expected = rt_atomic_load(&can->tx_state);

        if (expected == RT_CAN_TX_STATE_FLUSHING)
        {
            return enabled ? -RT_EBUSY : RT_EOK;
        }

        if (expected == desired)
        {
            return RT_EOK;
        }

        if (rt_atomic_compare_exchange_strong(&can->tx_state, &expected, desired))
        {
            return RT_EOK;
        }
    }
}

/**
 * @brief Wait until all TX submission critical sections have exited.
 *
 * @param[in] can     A pointer to the CAN device.
 * @param[in] timeout Timeout in OS ticks, or RT_WAITING_FOREVER.
 * @return RT_EOK when the reference count reaches zero, otherwise -RT_ETIMEOUT.
 */
static rt_err_t _can_wait_tx_submit_refcnt(struct rt_can_device *can, rt_int32_t timeout)
{
    rt_tick_t start_tick = rt_tick_get();

    while (rt_atomic_load(&can->tx_submit_refcnt) != 0)
    {
        if (timeout == 0)
        {
            return -RT_ETIMEOUT;
        }

        if (timeout != RT_WAITING_FOREVER &&
            (rt_tick_t)(rt_tick_get() - start_tick) >= (rt_tick_t)timeout)
        {
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(1);
    }

    return RT_EOK;
}

/**
 * @brief Wait until all active CAN write paths have released TX runtime resources.
 *
 * Unlike `tx_submit_refcnt`, this reference count spans the complete public
 * write operation, including semaphore waits and blocking completion waits.
 *
 * @param[in] can     A pointer to the CAN device.
 * @param[in] timeout Timeout in OS ticks, or RT_WAITING_FOREVER.
 * @return RT_EOK when the reference count reaches zero, otherwise -RT_ETIMEOUT.
 */
static rt_err_t _can_wait_tx_active_refcnt(struct rt_can_device *can, rt_int32_t timeout)
{
    rt_tick_t start_tick = rt_tick_get();

    while (rt_atomic_load(&can->tx_active_refcnt) != 0)
    {
        if (timeout == 0)
        {
            return -RT_ETIMEOUT;
        }

        if (timeout != RT_WAITING_FOREVER &&
            (rt_tick_t)(rt_tick_get() - start_tick) >= (rt_tick_t)timeout)
        {
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(1);
    }

    return RT_EOK;
}

/**
 * @brief Retire framework blocking TX ownership after hardware TX is quiescent.
 *
 * Active blocking senders are completed with ABORT. A normal-mode sender which
 * previously timed out left ERR together with `sndchange` ownership and has no
 * thread left to return its slot, so that slot is reclaimed here instead.
 *
 * @param[in] can A pointer to the CAN device.
 */
static void _can_retire_blocking_tx(struct rt_can_device *can)
{
    struct rt_can_tx_fifo *tx_fifo;
    rt_uint32_t wake_mask = 0;
    rt_uint32_t release_mask = 0;
    rt_base_t level;
    rt_uint32_t i;

    tx_fifo = (struct rt_can_tx_fifo *)can->can_tx;
    if (tx_fifo == RT_NULL)
    {
        return;
    }

    level = rt_hw_local_irq_disable();
    for (i = 0; i < can->config.sndboxnumber && i < 32; i++)
    {
        if ((can->status.sndchange & (1UL << i)) == 0)
        {
            continue;
        }

        if (tx_fifo->buffer[i].result == RT_CAN_SND_RESULT_WAIT)
        {
            tx_fifo->buffer[i].result = RT_CAN_SND_RESULT_ABORT;
            can->status.sndchange &= ~(1UL << i);
            wake_mask |= (1UL << i);
        }
        else if (tx_fifo->buffer[i].result == RT_CAN_SND_RESULT_ERR)
        {
            tx_fifo->buffer[i].result = RT_CAN_SND_RESULT_ABORT;
            can->status.sndchange &= ~(1UL << i);
            if (rt_list_isempty(&tx_fifo->buffer[i].list))
            {
                rt_list_insert_before(&tx_fifo->freelist, &tx_fifo->buffer[i].list);
                release_mask |= (1UL << i);
            }
        }
    }
    rt_hw_local_irq_enable(level);

    for (i = 0; i < can->config.sndboxnumber && i < 32; i++)
    {
        if ((release_mask & (1UL << i)) != 0)
        {
            rt_sem_release(&tx_fifo->sem);
        }
        if ((wake_mask & (1UL << i)) != 0)
        {
            rt_completion_done(&tx_fifo->buffer[i].completion);
        }
    }
}

/**
 * @brief Fully quiesce CAN TX and leave the framework TX gate disabled.
 *
 * A full flush first closes the TX gate, waits for send paths that already
 * crossed the gate, clears the non-blocking software queue, synchronously
 * aborts pending hardware TX, and finally wakes any blocking senders whose
 * transactions were aborted. The timeout is treated as one budget shared by
 * submitter draining, hardware abort, and active writer draining.
 *
 * @note This function must be called from thread context.
 *
 * @param[in] can     A pointer to the CAN device.
 * @param[in] timeout Timeout in OS ticks, or RT_WAITING_FOREVER.
 * @return RT_EOK on success, or a negative error code on failure.
 */
static rt_err_t _can_flush_tx(struct rt_can_device *can, rt_int32_t timeout)
{
    struct rt_can_abort_tx_config abort_cfg;
    rt_uint32_t queued_msgs = 0;
    rt_base_t previous_state;
    rt_tick_t start_tick;
    rt_tick_t elapsed;
    rt_base_t level;
    rt_err_t result;
    rt_int32_t remaining_timeout;

    if (rt_interrupt_get_nest() > 0)
    {
        return -RT_EINVAL;
    }

    if (timeout < 0 && timeout != RT_WAITING_FOREVER)
    {
        return -RT_EINVAL;
    }

    previous_state = rt_atomic_exchange(&can->tx_state, RT_CAN_TX_STATE_FLUSHING);
    if (previous_state == RT_CAN_TX_STATE_FLUSHING)
    {
        return -RT_EBUSY;
    }

    if (can->ops->abort_tx == RT_NULL)
    {
        rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
        return -RT_ENOSYS;
    }

    start_tick = rt_tick_get();

    /*
     * A sender increments tx_submit_refcnt before its final gate check. Once the
     * state is FLUSHING, no new sender can submit, and waiting for this count
     * to reach zero closes the race with send paths that crossed the old gate.
     */
    result = _can_wait_tx_submit_refcnt(can, timeout);
    if (result != RT_EOK)
    {
        rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
        return result;
    }

    /* Drop all non-blocking messages which have not reached the controller. */
    level = rt_hw_local_irq_disable();
    queued_msgs = rt_ringbuffer_data_len(&can->nb_tx_rb) / sizeof(struct rt_can_msg);
    rt_ringbuffer_reset(&can->nb_tx_rb);
    can->status.dropedsndpkg += queued_msgs;
    rt_hw_local_irq_enable(level);

    abort_cfg.mailbox_mask = RT_CAN_TX_MAILBOX_ALL;
    abort_cfg.timeout = timeout;
    if (timeout != RT_WAITING_FOREVER)
    {
        elapsed = (rt_tick_t)(rt_tick_get() - start_tick);
        abort_cfg.timeout = elapsed >= (rt_tick_t)timeout ? 0 : timeout - (rt_int32_t)elapsed;
    }

    result = can->ops->abort_tx(can, &abort_cfg);
    if (result != RT_EOK)
    {
        /* Keep TX disabled after a partial flush. */
        rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
        return result;
    }

    /* Hardware is now idle; retire all remaining framework blocking ownership. */
    _can_retire_blocking_tx(can);

    remaining_timeout = timeout;
    if (timeout != RT_WAITING_FOREVER)
    {
        elapsed = (rt_tick_t)(rt_tick_get() - start_tick);
        remaining_timeout = elapsed >= (rt_tick_t)timeout ? 0 : timeout - (rt_int32_t)elapsed;
    }

    /*
     * Blocking writers may have been waiting on the mailbox semaphore before
     * they reached the submission gate. Retirement wakes/cascades those waiters;
     * keep the TX runtime alive until every public write path has returned.
     */
    result = _can_wait_tx_active_refcnt(can, remaining_timeout);
    if (result != RT_EOK)
    {
        rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
        return result;
    }

    /* A full flush intentionally leaves the device quiesced. */
    rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
    return RT_EOK;
}

static rt_err_t rt_can_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    /* initialize rx/tx */
    can->can_rx = RT_NULL;
    can->can_tx = RT_NULL;
#ifdef RT_CAN_USING_HDR
    can->hdr = RT_NULL;
#endif

    /* apply configuration */
    if (can->ops->configure)
        result = can->ops->configure(can, &can->config);
    else
        result = -RT_ENOSYS;

    return result;
}

/**
 * @internal
 * @brief Handles reading messages from the software RX FIFO into a user buffer.
 *
 * This function is called by the public `rt_can_read()` API when the device
 * is opened with interrupt-driven reception enabled. It safely transfers
 * messages from the internal `uselist` to the user-provided buffer.
 *
 * @param[in]  can   A pointer to the CAN device.
 * @param[out] data  A pointer to the destination buffer for the received messages.
 * @param[in]  msgs  The total size in bytes of the destination buffer.
 *
 * @return The number of bytes actually read from the FIFO.
 */
rt_inline rt_ssize_t _can_int_rx(struct rt_can_device *can, struct rt_can_msg *data, rt_ssize_t msgs)
{
    rt_ssize_t size;
    struct rt_can_rx_fifo *rx_fifo;
    RT_ASSERT(can != RT_NULL);
    size = msgs;

    rx_fifo = (struct rt_can_rx_fifo *) can->can_rx;
    RT_ASSERT(rx_fifo != RT_NULL);

    /* read from software FIFO */
    while (msgs / sizeof(struct rt_can_msg) > 0)
    {
        rt_base_t level;
#ifdef RT_CAN_USING_HDR
        rt_int8_t hdr;
#endif /*RT_CAN_USING_HDR*/
        struct rt_can_msg_list *listmsg = RT_NULL;

        /* disable interrupt */
        level = rt_hw_local_irq_disable();
#ifdef RT_CAN_USING_HDR
        hdr = data->hdr_index;

        if (hdr >= 0 && can->hdr && hdr < can->config.maxhdr && !rt_list_isempty(&can->hdr[hdr].list))
        {
            listmsg = rt_list_entry(can->hdr[hdr].list.next, struct rt_can_msg_list, hdrlist);
            rt_list_remove(&listmsg->list);
            rt_list_remove(&listmsg->hdrlist);
            if (can->hdr[hdr].msgs)
            {
                can->hdr[hdr].msgs--;
            }
            listmsg->owner = RT_NULL;
        }
        else if (hdr == -1)
#endif /*RT_CAN_USING_HDR*/
        {
            if (!rt_list_isempty(&rx_fifo->uselist))
            {
                listmsg = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
                rt_list_remove(&listmsg->list);
#ifdef RT_CAN_USING_HDR
                rt_list_remove(&listmsg->hdrlist);
                if (listmsg->owner != RT_NULL && listmsg->owner->msgs)
                {
                    listmsg->owner->msgs--;
                }
                listmsg->owner = RT_NULL;
#endif /*RT_CAN_USING_HDR*/
            }
            else
            {
                /* no data, enable interrupt and break out */
                rt_hw_local_irq_enable(level);
                break;
            }
        }

        /* enable interrupt */
        rt_hw_local_irq_enable(level);
        if (listmsg != RT_NULL)
        {
            rt_memcpy(data, &listmsg->data, sizeof(struct rt_can_msg));

            level = rt_hw_local_irq_disable();
            rt_list_insert_before(&rx_fifo->freelist, &listmsg->list);
            rx_fifo->freenumbers++;
            RT_ASSERT(rx_fifo->freenumbers <= can->config.msgboxsz);
            rt_hw_local_irq_enable(level);

            listmsg = RT_NULL;
        }
        else
        {
            break;
        }
        data ++;
        msgs -= sizeof(struct rt_can_msg);
    }

    return (size - msgs);
}

/**
 * @internal
 * @brief Handles the blocking (synchronous) transmission of CAN messages.
 *
 * This function is the core of the blocking send mechanism. It iterates through
 * a buffer of messages to be sent. For each message, it:
 * 1. Acquires a hardware mailbox resource using a semaphore.
 * 2. Submits the message to the low-level driver for transmission.
 * 3. Blocks the calling thread by waiting on a completion object.
 * 4. Is woken up by the TX complete ISR (`rt_hw_can_isr`) when the transmission is finished.
 *
 * @note This function will block the calling thread and must not be called from an ISR.
 *
 * @param[in] can   A pointer to the CAN device.
 * @param[in] data  A pointer to the source buffer of messages to be sent.
 * @param[in] msgs  The total size in bytes of the source buffer.
 *
 * @return The number of bytes successfully sent.
 */
rt_inline int _can_int_tx(struct rt_can_device *can, const struct rt_can_msg *data, int msgs)
{
    int size;
    struct rt_can_tx_fifo *tx_fifo;

    RT_ASSERT(can != RT_NULL);

    size = msgs;
    tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
    RT_ASSERT(tx_fifo != RT_NULL);

    while (msgs)
    {
        rt_base_t level;
        rt_uint32_t no;
        rt_uint32_t result;
        rt_err_t send_result;
        struct rt_can_sndbxinx_list *tx_tosnd = RT_NULL;

        if (rt_sem_take(&(tx_fifo->sem), RT_WAITING_FOREVER) != RT_EOK)
        {
            goto err_ret;
        }

        level = rt_hw_local_irq_disable();
        tx_tosnd = rt_list_entry(tx_fifo->freelist.next, struct rt_can_sndbxinx_list, list);
        RT_ASSERT(tx_tosnd != RT_NULL);
        rt_list_remove(&tx_tosnd->list);
        rt_hw_local_irq_enable(level);

        if (!_can_tx_submit_begin(can))
        {
            goto release_mailbox;
        }

        no = ((rt_ubase_t)tx_tosnd - (rt_ubase_t)tx_fifo->buffer) / sizeof(struct rt_can_sndbxinx_list);

        /* Publish the reusable completion, result and ownership bit as one ISR-visible transaction. */
        level = rt_hw_local_irq_disable();
        tx_tosnd->result = RT_CAN_SND_RESULT_WAIT;
        rt_completion_init(&tx_tosnd->completion);
        can->status.sndchange |= 1UL << no;
        rt_hw_local_irq_enable(level);

        send_result = can->ops->sendmsg(can, data, no);
        _can_tx_submit_end(can);

        if (send_result != RT_EOK)
        {
            goto abort_transaction;
        }

        if (rt_completion_wait(&(tx_tosnd->completion), RT_CANSND_MSG_TIMEOUT) != RT_EOK)
        {
            /*
             * A software timeout does not prove that hardware released the
             * mailbox. If ownership is still pending, detach this sender by
             * leaving sndchange set and marking ERR; the terminal ISR or full
             * flush will reclaim the slot and release the semaphore later.
             */
            level = rt_hw_local_irq_disable();
            if ((can->status.sndchange & (1UL << no)) != 0 &&
                tx_tosnd->result == RT_CAN_SND_RESULT_WAIT)
            {
                tx_tosnd->result = RT_CAN_SND_RESULT_ERR;
                rt_hw_local_irq_enable(level);
                goto err_ret;
            }

            /* A terminal IRQ or flush raced with the timeout; this sender still owns the software slot return. */
            if (!rt_list_isempty(&tx_tosnd->list))
            {
                rt_list_remove(&tx_tosnd->list);
            }
            rt_list_insert_before(&tx_fifo->freelist, &tx_tosnd->list);
            rt_hw_local_irq_enable(level);
            rt_sem_release(&(tx_fifo->sem));
            goto err_ret;
        }

        level = rt_hw_local_irq_disable();
        result = tx_tosnd->result;
        if (!rt_list_isempty(&tx_tosnd->list))
        {
            rt_list_remove(&tx_tosnd->list);
        }
        rt_list_insert_before(&tx_fifo->freelist, &tx_tosnd->list);
        rt_hw_local_irq_enable(level);
        rt_sem_release(&(tx_fifo->sem));

        if (result == RT_CAN_SND_RESULT_OK)
        {
            level = rt_hw_local_irq_disable();
            can->status.sndpkg++;
            rt_hw_local_irq_enable(level);

            data ++;
            msgs -= sizeof(struct rt_can_msg);
            if (!msgs) break;
            continue;
        }

        goto err_ret;

abort_transaction:
        level = rt_hw_local_irq_disable();
        can->status.sndchange &= ~(1UL << no);
        tx_tosnd->result = RT_CAN_SND_RESULT_ERR;
        rt_list_insert_before(&tx_fifo->freelist, &tx_tosnd->list);
        rt_hw_local_irq_enable(level);
        rt_sem_release(&(tx_fifo->sem));
        goto err_ret;

release_mailbox:
        level = rt_hw_local_irq_disable();
        rt_list_insert_before(&tx_fifo->freelist, &tx_tosnd->list);
        rt_hw_local_irq_enable(level);
        rt_sem_release(&(tx_fifo->sem));

err_ret:
        level = rt_hw_local_irq_disable();
        can->status.dropedsndpkg++;
        rt_hw_local_irq_enable(level);
        break;
    }

    return (size - msgs);
}

/**
 * @brief Reset private-mode software ownership after hardware has stopped using a mailbox.
 *
 * This helper is only used after the hardware transaction is known to be no
 * longer pending, such as a rejected send request or a terminal TX result.
 * It does not signal the transaction completion object.
 *
 * @param[in] can     A pointer to the CAN device.
 * @param[in] tx_fifo A pointer to the blocking TX FIFO.
 * @param[in] no      The private hardware mailbox index.
 */
static void _can_priv_tx_reset(struct rt_can_device *can,
                               struct rt_can_tx_fifo *tx_fifo,
                               rt_uint32_t no)
{
    rt_base_t level;

    level = rt_hw_local_irq_disable();
    can->status.sndchange &= ~(1UL << no);
    tx_fifo->buffer[no].result = RT_CAN_SND_RESULT_OK;
    rt_hw_local_irq_enable(level);
}

/**
 * @internal
 * @brief Handles blocking transmission in "private mode".
 *
 * This is a specialized version of `_can_int_tx` where the target hardware mailbox
 * for each message is specified by the user in the `priv` field of the `rt_can_msg`
 * structure, rather than being acquired dynamically from a pool.
 *
 * @note Private mode does not serialize concurrent senders targeting the same
 * hardware mailbox; callers must still serialize each mailbox. If a mailbox
 * remains owned by a prior transaction, this blocking path waits for that
 * transaction to retire and retries the gate/ownership checks. A software
 * timeout does not itself release hardware ownership.
 *
 * @param[in] can   A pointer to the CAN device.
 * @param[in] data  A pointer to the source buffer of messages.
 * @param[in] msgs  The total size in bytes of the source buffer.
 *
 * @return The number of bytes successfully sent.
 */
rt_inline int _can_int_tx_priv(struct rt_can_device *can, const struct rt_can_msg *data, int msgs)
{
    int size;
    rt_base_t level;
    rt_uint32_t no, result;
    struct rt_can_tx_fifo *tx_fifo;

    RT_ASSERT(can != RT_NULL);

    size = msgs;
    tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
    RT_ASSERT(tx_fifo != RT_NULL);

    while (msgs)
    {
        struct rt_can_sndbxinx_list *tx_slot;
        rt_err_t send_result;

        no = data->priv;
        if (no >= can->config.sndboxnumber)
        {
            break;
        }

        tx_slot = &tx_fifo->buffer[no];

        if (!_can_tx_submit_begin(can))
        {
            break;
        }

        /* Wait for a busy private mailbox to retire, then retry all gate and ownership checks. */
        level = rt_hw_local_irq_disable();
        result = tx_slot->result;
        if ((can->status.sndchange & (1UL << no)) != 0 ||
            result == RT_CAN_SND_RESULT_WAIT)
        {
            rt_hw_local_irq_enable(level);
            _can_tx_submit_end(can);
            if (rt_completion_wait(&(tx_slot->completion), RT_WAITING_FOREVER) != RT_EOK)
            {
                break;
            }
            continue;
        }

        /* Reinitialize the reusable completion before publishing this transaction to the ISR. */
        tx_slot->result = RT_CAN_SND_RESULT_WAIT;
        rt_completion_init(&(tx_slot->completion));
        can->status.sndchange |= 1UL << no;
        rt_hw_local_irq_enable(level);

        send_result = can->ops->sendmsg(can, data, no);
        _can_tx_submit_end(can);

        if (send_result != RT_EOK)
        {
            _can_priv_tx_reset(can, tx_fifo, no);
            break;
        }

        if (rt_completion_wait(&(tx_slot->completion), RT_CANSND_MSG_TIMEOUT) != RT_EOK)
        {
            /*
             * Hardware may still own this transaction. Preserve WAIT/sndchange so
             * a late IRQ or a full flush can retire it before the mailbox is reused.
             */
            level = rt_hw_local_irq_disable();
            can->status.dropedsndpkg++;
            rt_hw_local_irq_enable(level);
            break;
        }

        level = rt_hw_local_irq_disable();
        result = tx_slot->result;
        rt_hw_local_irq_enable(level);

        if (result == RT_CAN_SND_RESULT_OK)
        {
            level = rt_hw_local_irq_disable();
            can->status.sndpkg++;
            rt_hw_local_irq_enable(level);

            data ++;
            msgs -= sizeof(struct rt_can_msg);
            if (!msgs) break;
            continue;
        }

        if (result == RT_CAN_SND_RESULT_WAIT)
        {
            break;
        }

        _can_priv_tx_reset(can, tx_fifo, no);

        level = rt_hw_local_irq_disable();
        can->status.dropedsndpkg++;
        rt_hw_local_irq_enable(level);
        break;
    }

    return (size - msgs);
}

/**
 * @internal
 * @brief Internal implementation of non-blocking CAN transmission.
 *
 * This function iterates through a buffer of CAN messages and attempts to send each one
 * using a non-blocking strategy. It first tries to send directly via hardware (fast path).
 * If the hardware is busy, it enqueues the message into a software ring buffer (slow path).
 * This function is thread-safe and ISR-safe due to the use of critical sections for
 * accessing the shared ring buffer.
 *
 * @param[in] can   A pointer to the CAN device.
 * @param[in] pmsg  A pointer to the buffer of `rt_can_msg` structures.
 * @param[in] size  The total size of the buffer in bytes.
 *
 * @return The number of bytes successfully sent or enqueued for later transmission.
 */
static rt_ssize_t _can_nonblocking_tx(struct rt_can_device *can, const struct rt_can_msg *pmsg, rt_size_t size)
{
    rt_ssize_t sent_size = 0;
    rt_base_t level;

    if (can->ops->sendmsg_nonblocking == RT_NULL)
    {
        return -RT_EINVAL;
    }

    while (sent_size < size)
    {
        if (!_can_tx_submit_begin(can))
        {
            break;
        }

        if (can->ops->sendmsg_nonblocking(can, pmsg) == RT_EOK)
        {
            _can_tx_submit_end(can);
            pmsg++;
            sent_size += sizeof(struct rt_can_msg);
            continue;
        }

        level = rt_hw_local_irq_disable();
        if (rt_ringbuffer_space_len(&can->nb_tx_rb) >= sizeof(struct rt_can_msg))
        {
            rt_ringbuffer_put(&can->nb_tx_rb, (rt_uint8_t *)pmsg, sizeof(struct rt_can_msg));
            rt_hw_local_irq_enable(level);
            _can_tx_submit_end(can);

            pmsg++;
            sent_size += sizeof(struct rt_can_msg);
        }
        else
        {
            /* Buffer is full, cannot process this message or subsequent ones. */
            can->status.dropedsndpkg += (size - sent_size) / sizeof(struct rt_can_msg);
            rt_hw_local_irq_enable(level);
            _can_tx_submit_end(can);
            break;
        }
    }

    return sent_size;
}

/**
 * @internal
 * @brief Opens the CAN device and initializes its resources.
 *
 * This function is called when `rt_device_open()` is invoked on a CAN device.
 * It allocates and initializes software FIFOs for reception and transmission,
 * sets up semaphores for the blocking send mechanism, configures the non-blocking
 * send buffer, and starts the periodic status timer.
 *
 * @param[in] dev   A pointer to the device to be opened.
 * @param[in] oflag The open flags, e.g., `RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX`.
 *
 * @return `RT_EOK` on successful opening, or an error code on failure.
 */
static rt_err_t rt_can_open(struct rt_device *dev, rt_uint16_t oflag)
{
    struct rt_can_device *can;
    rt_bool_t first_open;
    char tmpname[16];
    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    CAN_LOCK(can);
    first_open = (dev->ref_count == 0);

    /* get open flags */
    dev->open_flag = oflag & 0xff;
    if (can->can_rx == RT_NULL)
    {
        if (oflag & RT_DEVICE_FLAG_INT_RX)
        {
            int i = 0;
            struct rt_can_rx_fifo *rx_fifo;

            rx_fifo = (struct rt_can_rx_fifo *) rt_malloc(sizeof(struct rt_can_rx_fifo) +
                      can->config.msgboxsz * sizeof(struct rt_can_msg_list));
            RT_ASSERT(rx_fifo != RT_NULL);

            rx_fifo->buffer = (struct rt_can_msg_list *)(rx_fifo + 1);
            rt_memset(rx_fifo->buffer, 0, can->config.msgboxsz * sizeof(struct rt_can_msg_list));
            rt_list_init(&rx_fifo->freelist);
            rt_list_init(&rx_fifo->uselist);
            rx_fifo->freenumbers = can->config.msgboxsz;
            for (i = 0;  i < can->config.msgboxsz; i++)
            {
                rt_list_insert_before(&rx_fifo->freelist, &rx_fifo->buffer[i].list);
#ifdef RT_CAN_USING_HDR
                rt_list_init(&rx_fifo->buffer[i].hdrlist);
                rx_fifo->buffer[i].owner = RT_NULL;
#endif
            }
            can->can_rx = rx_fifo;

            dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
            /* open can rx interrupt */
            can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        }
    }

    if (can->can_tx == RT_NULL)
    {
        if (oflag & RT_DEVICE_FLAG_INT_TX)
        {
            int i = 0;
            struct rt_can_tx_fifo *tx_fifo;

            tx_fifo = (struct rt_can_tx_fifo *) rt_malloc(sizeof(struct rt_can_tx_fifo) +
                      can->config.sndboxnumber * sizeof(struct rt_can_sndbxinx_list));
            RT_ASSERT(tx_fifo != RT_NULL);

            tx_fifo->buffer = (struct rt_can_sndbxinx_list *)(tx_fifo + 1);
            rt_memset(tx_fifo->buffer, 0,
                    can->config.sndboxnumber * sizeof(struct rt_can_sndbxinx_list));
            rt_list_init(&tx_fifo->freelist);
            for (i = 0;  i < can->config.sndboxnumber; i++)
            {
                rt_list_insert_before(&tx_fifo->freelist, &tx_fifo->buffer[i].list);
                rt_completion_init(&(tx_fifo->buffer[i].completion));
                tx_fifo->buffer[i].result = RT_CAN_SND_RESULT_OK;
            }

            rt_sprintf(tmpname, "%stl", dev->parent.name);
            rt_sem_init(&(tx_fifo->sem), tmpname, can->config.sndboxnumber, RT_IPC_FLAG_FIFO);
            can->can_tx = tx_fifo;

            dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
            /* open can tx interrupt */
            can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_TX);
        }
    }

    can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_CAN_INT_ERR);

#ifdef RT_CAN_USING_HDR
    if (can->hdr == RT_NULL)
    {
        int i = 0;
        struct rt_can_hdr *phdr;

        phdr = (struct rt_can_hdr *) rt_malloc(can->config.maxhdr * sizeof(struct rt_can_hdr));
        RT_ASSERT(phdr != RT_NULL);
        rt_memset(phdr, 0, can->config.maxhdr * sizeof(struct rt_can_hdr));
        for (i = 0;  i < can->config.maxhdr; i++)
        {
            rt_list_init(&phdr[i].list);
        }

        can->hdr = phdr;
    }
#endif

    if (first_open)
    {
#ifdef RT_CAN_MALLOC_NB_TX_BUFFER
        can->nb_tx_rb_pool = (rt_uint8_t *)rt_malloc(RT_CAN_NB_TX_FIFO_SIZE);
        RT_ASSERT(can->nb_tx_rb_pool != RT_NULL);
#endif /* RT_CAN_MALLOC_NB_TX_BUFFER  */
        rt_ringbuffer_init(&can->nb_tx_rb, can->nb_tx_rb_pool, RT_CAN_NB_TX_FIFO_SIZE);
        rt_atomic_store(&can->tx_submit_refcnt, 0);
        rt_atomic_store(&can->tx_active_refcnt, 0);
        rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_ENABLED);
    }

    if (!can->timerinitflag)
    {
        can->timerinitflag = 1;

        rt_timer_start(&can->timer);
    }

    CAN_UNLOCK(can);

    return RT_EOK;
}

static rt_err_t rt_can_close(struct rt_device *dev)
{
    struct rt_can_device *can;
    struct rt_can_abort_tx_config abort_cfg;
    rt_bool_t controller_stopped = RT_FALSE;
    rt_err_t result;

    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    CAN_LOCK(can);

    /* this device has more reference count */
    if (dev->ref_count > 1)
    {
        CAN_UNLOCK(can);
        return RT_EOK;
    }

    /* Stop admitting new TX work before touching any TX runtime resource. */
    rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);

    /* Wait until every sender that already crossed the gate has left low-level submission. */
    result = _can_wait_tx_submit_refcnt(can, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        CAN_UNLOCK(can);
        return result;
    }

    if (can->ops->abort_tx != RT_NULL)
    {
        abort_cfg.mailbox_mask = RT_CAN_TX_MAILBOX_ALL;
        abort_cfg.timeout = RT_CANSND_MSG_TIMEOUT;
        result = can->ops->abort_tx(can, &abort_cfg);
        if (result != RT_EOK)
        {
            /*
             * If synchronous abort cannot establish hardware idle, stop the
             * controller before retiring software ownership or freeing memory.
             */
            result = can->ops->control(can, RT_CAN_CMD_START, RT_FALSE);
            if (result != RT_EOK)
            {
                CAN_UNLOCK(can);
                return result;
            }
            controller_stopped = RT_TRUE;
        }
    }
    else
    {
        /* Drivers without abort support must stop hardware before software retirement. */
        result = can->ops->control(can, RT_CAN_CMD_START, RT_FALSE);
        if (result != RT_EOK)
        {
            CAN_UNLOCK(can);
            return result;
        }
        controller_stopped = RT_TRUE;
    }

    /* Hardware is idle/stopped, so blocking ownership can now be retired safely. */
    _can_retire_blocking_tx(can);

    /*
     * Completion and semaphore wakeups above may still be unwinding public
     * rt_device_write() calls. Keep all TX runtime resources alive until those
     * writers have returned.
     */
    result = _can_wait_tx_active_refcnt(can, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        CAN_UNLOCK(can);
        return result;
    }

    /*
     * Complete hardware shutdown before releasing software resources. If stop
     * fails, keep the TX runtime intact so the device is not left half-closed.
     */
    if (!controller_stopped)
    {
        result = can->ops->control(can, RT_CAN_CMD_START, RT_FALSE);
        if (result != RT_EOK)
        {
            CAN_UNLOCK(can);
            return result;
        }
    }

    if (can->timerinitflag)
    {
        can->timerinitflag = 0;

        rt_timer_stop(&can->timer);
    }

    can->status_indicate.ind = RT_NULL;
    can->status_indicate.args = RT_NULL;

#ifdef RT_CAN_USING_HDR
    if (can->hdr != RT_NULL)
    {
        rt_free(can->hdr);
        can->hdr = RT_NULL;
    }
#endif

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        struct rt_can_rx_fifo *rx_fifo;

        /* clear can rx interrupt */
        can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);

        rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        rt_free(rx_fifo);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
        can->can_rx = RT_NULL;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        struct rt_can_tx_fifo *tx_fifo;

        /* clear can tx interrupt before releasing TX runtime memory */
        can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_TX);

        tx_fifo = (struct rt_can_tx_fifo *)can->can_tx;
        RT_ASSERT(tx_fifo != RT_NULL);
        rt_sem_detach(&(tx_fifo->sem));
        rt_free(tx_fifo);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
        can->can_tx = RT_NULL;
    }

    can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_CAN_INT_ERR);

#ifdef RT_CAN_MALLOC_NB_TX_BUFFER
    if (can->nb_tx_rb_pool)
    {
        rt_free(can->nb_tx_rb_pool);
        can->nb_tx_rb_pool = RT_NULL;
    }
#endif

    CAN_UNLOCK(can);

    return RT_EOK;
}

static rt_ssize_t rt_can_read(struct rt_device *dev,
                             rt_off_t          pos,
                             void             *buffer,
                             rt_size_t         size)
{
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return -RT_EINVAL;

    can = (struct rt_can_device *)dev;

    if ((dev->open_flag & RT_DEVICE_FLAG_INT_RX) && (dev->ref_count > 0))
    {
        return _can_int_rx(can, buffer, size);
    }

    return -RT_ENOSYS;
}

/**
 * @brief Write data to the CAN device.
 *
 * This function serves as the unified entry point for sending CAN messages.
 * It intelligently routes the request to either a blocking or non-blocking
 * transmission function based on:
 * 1. The calling context (thread or ISR).
 * 2. A user-specified flag (`nonblocking`) in the message structure.
 *
 * @param[in] dev     A pointer to the device object.
 * @param[in] pos     This parameter is ignored for CAN devices.
 * @param[in] buffer  A pointer to the buffer containing one or more `rt_can_msg` structures.
 * @param[in] size    The total size of the buffer in bytes. Must be a multiple of `sizeof(struct rt_can_msg)`.
 *
 * @return The number of bytes successfully written. For non-blocking sends, this means
 *         the data was either sent directly or enqueued in the buffer.
 */
static rt_ssize_t rt_can_write(struct rt_device *dev,
                              rt_off_t          pos,
                              const void       *buffer,
                              rt_size_t         size)
{
    struct rt_can_device *can;
    const struct rt_can_msg *pmsg;
    rt_ssize_t result;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if (size == 0) return -RT_EINVAL;

    /* Ensure size is a multiple of the message size for buffer operations */
    if (size % sizeof(struct rt_can_msg) != 0)
    {
        return -RT_EINVAL;
    }

    can = (struct rt_can_device *)dev;
    pmsg = (const struct rt_can_msg *)buffer;

    if(dev->ref_count == 0)
    {
        return -RT_ENOSYS;
    }

    /*
     * Span the complete write lifetime so close/flush cannot release TX runtime
     * resources while a blocking writer is waiting on a semaphore/completion.
     */
    rt_atomic_add(&can->tx_active_refcnt, 1);

    if (rt_atomic_load(&can->tx_state) != RT_CAN_TX_STATE_ENABLED)
    {
        result = -RT_EBUSY;
        goto write_done;
    }

    /*
     * Routing to the non-blocking send scenario:
     * 1. Called from within an interrupt context.
     * 2. Called from a thread, but the user explicitly set the nonblocking flag on the first message.
     */
    if (rt_interrupt_get_nest() > 0 || pmsg->nonblocking)
    {
        result = _can_nonblocking_tx(can, pmsg, size);
        goto write_done;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        if (can->config.privmode)
        {
            result = _can_int_tx_priv(can, buffer, size);
        }
        else
        {
            result = _can_int_tx(can, buffer, size);
        }
    }
    else
    {
        result = -RT_ENOSYS;
    }

write_done:
    rt_atomic_sub(&can->tx_active_refcnt, 1);
    return result;
}

static rt_err_t rt_can_control(struct rt_device *dev,
                               int              cmd,
                               void             *args)
{
    struct rt_can_device *can;
    rt_err_t res;

    res = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* suspend device */
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_CONFIG:
        /* configure device */
        res = can->ops->configure(can, (struct can_configure *)args);
        break;

    case RT_CAN_CMD_SET_TX_ENABLE:
        if (rt_interrupt_get_nest() > 0)
        {
            return -RT_EINVAL;
        }

        /* Serialize the public gate change with close and controller teardown. */
        CAN_LOCK(can);
        if (dev->ref_count == 0)
        {
            res = -RT_EBUSY;
        }
        else
        {
            res = _can_set_tx_enabled(can, (rt_bool_t)((rt_ubase_t)args != 0));
        }
        CAN_UNLOCK(can);
        break;

    case RT_CAN_CMD_ABORT_TX:
    {
        struct rt_can_abort_tx_config default_cfg;
        struct rt_can_abort_tx_config abort_cfg;
        const struct rt_can_abort_tx_config *requested_cfg;
        rt_base_t expected;
        rt_tick_t start_tick;
        rt_tick_t elapsed;

        /* The abort operation is synchronous and may block while waiting for hardware idle. */
        if (rt_interrupt_get_nest() > 0)
        {
            return -RT_EINVAL;
        }

        if (can->ops->abort_tx == RT_NULL)
        {
            return -RT_ENOSYS;
        }

        if (args == RT_NULL)
        {
            default_cfg.mailbox_mask = RT_CAN_TX_MAILBOX_ALL;
            default_cfg.timeout = RT_CANSND_MSG_TIMEOUT;
            requested_cfg = &default_cfg;
        }
        else
        {
            requested_cfg = (const struct rt_can_abort_tx_config *)args;
        }

        if (requested_cfg->timeout < 0 && requested_cfg->timeout != RT_WAITING_FOREVER)
        {
            return -RT_EINVAL;
        }
        abort_cfg = *requested_cfg;

        /*
         * Direct hardware abort is intentionally not a full flush. Require the
         * caller to close the TX admission gate first, then reserve FLUSHING so
         * another thread cannot re-enable TX while submitters drain or hardware
         * abort is in progress. Software queues are left untouched and are not
         * restarted by this command; use a full flush when queued frames must not
         * survive the quiesce operation.
         */
        CAN_LOCK(can);
        if (dev->ref_count == 0)
        {
            CAN_UNLOCK(can);
            return -RT_EBUSY;
        }

        expected = RT_CAN_TX_STATE_DISABLED;
        if (!rt_atomic_compare_exchange_strong(&can->tx_state, &expected, RT_CAN_TX_STATE_FLUSHING))
        {
            CAN_UNLOCK(can);
            return -RT_EBUSY;
        }

        start_tick = rt_tick_get();
        res = _can_wait_tx_submit_refcnt(can, abort_cfg.timeout);
        if (res == RT_EOK)
        {
            if (abort_cfg.timeout != RT_WAITING_FOREVER)
            {
                elapsed = (rt_tick_t)(rt_tick_get() - start_tick);
                abort_cfg.timeout = elapsed >= (rt_tick_t)abort_cfg.timeout ?
                                    0 : abort_cfg.timeout - (rt_int32_t)elapsed;
            }
            res = can->ops->abort_tx(can, &abort_cfg);
        }

        rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
        CAN_UNLOCK(can);
        break;
    }

    case RT_CAN_CMD_FLUSH_TX:
    {
        rt_int32_t timeout = RT_CANSND_MSG_TIMEOUT;

        if (rt_interrupt_get_nest() > 0)
        {
            return -RT_EINVAL;
        }

        if (args != RT_NULL)
        {
            timeout = *(const rt_int32_t *)args;
        }

        /* Serialize the full quiesce sequence with close and controller deinitialization. */
        CAN_LOCK(can);
        if (dev->ref_count == 0)
        {
            CAN_UNLOCK(can);
            return -RT_EBUSY;
        }
        res = _can_flush_tx(can, timeout);
        CAN_UNLOCK(can);
        break;
    }

    case RT_CAN_CMD_SET_PRIV:
        /* configure device */
        if ((rt_uint32_t)(rt_ubase_t)args != can->config.privmode)
        {
            int i;
            rt_base_t level;
            struct rt_can_tx_fifo *tx_fifo;

            /*
             * Private-mode selection is a configuration-time operation. The
             * caller must serialize this command with all CAN TX users and invoke
             * it only when no TX path is active. sndchange is only a defensive
             * check for outstanding blocking ownership; it does not make live
             * mode conversion thread-safe.
             */
            level = rt_hw_local_irq_disable();
            if (can->status.sndchange != 0)
            {
                rt_hw_local_irq_enable(level);
                return -RT_EBUSY;
            }
            rt_hw_local_irq_enable(level);

            res = can->ops->control(can, cmd, args);
            if (res != RT_EOK) return res;
            tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
            if (can->config.privmode)
            {
                for (i = 0;  i < can->config.sndboxnumber; i++)
                {
                    level = rt_hw_local_irq_disable();
                    if(rt_list_isempty(&tx_fifo->buffer[i].list))
                    {
                        rt_sem_release(&(tx_fifo->sem));
                    }
                    else
                    {
                        rt_list_remove(&tx_fifo->buffer[i].list);
                    }
                    rt_hw_local_irq_enable(level);
                }

            }
            else
            {
                for (i = 0;  i < can->config.sndboxnumber; i++)
                {
                    level = rt_hw_local_irq_disable();
                    tx_fifo->buffer[i].result = RT_CAN_SND_RESULT_OK;
                    if (rt_list_isempty(&tx_fifo->buffer[i].list))
                    {
                        rt_list_insert_before(&tx_fifo->freelist, &tx_fifo->buffer[i].list);
                    }
                    rt_hw_local_irq_enable(level);
                }
            }
        }
        break;

    case RT_CAN_CMD_SET_STATUS_IND:
        can->status_indicate.ind = ((rt_can_status_ind_type_t)args)->ind;
        can->status_indicate.args = ((rt_can_status_ind_type_t)args)->args;
        break;

#ifdef RT_CAN_USING_HDR
    case RT_CAN_CMD_SET_FILTER:
        res = can->ops->control(can, cmd, args);
        if (res != RT_EOK || can->hdr == RT_NULL)
        {
            return res;
        }

        struct rt_can_filter_config *pfilter;
        struct rt_can_filter_item *pitem;
        rt_uint32_t count;
        rt_base_t level;

        pfilter = (struct rt_can_filter_config *)args;
        RT_ASSERT(pfilter);
        count = pfilter->count;
        pitem = pfilter->items;
        if (pfilter->actived)
        {
            while (count)
            {
                if (pitem->hdr_bank >= can->config.maxhdr || pitem->hdr_bank < 0)
                {
                    count--;
                    pitem++;
                    continue;
                }

                level = rt_hw_local_irq_disable();
                if (!can->hdr[pitem->hdr_bank].connected)
                {
                    rt_hw_local_irq_enable(level);
                    rt_memcpy(&can->hdr[pitem->hdr_bank].filter, pitem,
                              sizeof(struct rt_can_filter_item));
                    level = rt_hw_local_irq_disable();
                    can->hdr[pitem->hdr_bank].connected = 1;
                    can->hdr[pitem->hdr_bank].msgs = 0;
                    rt_list_init(&can->hdr[pitem->hdr_bank].list);
                }
                rt_hw_local_irq_enable(level);

                count--;
                pitem++;
            }
        }
        else
        {
            while (count)
            {
                if (pitem->hdr_bank >= can->config.maxhdr || pitem->hdr_bank < 0)
                {
                    count--;
                    pitem++;
                    continue;
                }
                level = rt_hw_local_irq_disable();

                if (can->hdr[pitem->hdr_bank].connected)
                {
                    can->hdr[pitem->hdr_bank].connected = 0;
                    can->hdr[pitem->hdr_bank].msgs = 0;
                    if (!rt_list_isempty(&can->hdr[pitem->hdr_bank].list))
                    {
                        rt_list_remove(can->hdr[pitem->hdr_bank].list.next);
                    }
                    rt_hw_local_irq_enable(level);
                    rt_memset(&can->hdr[pitem->hdr_bank].filter, 0,
                              sizeof(struct rt_can_filter_item));
                }
                else
                {
                    rt_hw_local_irq_enable(level);
                }
                count--;
                pitem++;
            }
        }
        break;
#endif /*RT_CAN_USING_HDR*/
#ifdef RT_CAN_USING_BUS_HOOK
    case RT_CAN_CMD_SET_BUS_HOOK:
        can->bus_hook = (rt_can_bus_hook) args;
        break;
#endif /*RT_CAN_USING_BUS_HOOK*/
    default :
        /* control device */
        if (can->ops->control != RT_NULL)
        {
            res = can->ops->control(can, cmd, args);
        }
        else
        {
            res = -RT_ENOSYS;
        }
        break;
    }

    return res;
}

/**
 * @internal
 * @brief Periodic timer callback for the CAN device.
 *
 * This function is executed periodically by a system timer. Its main purposes are:
 * 1. To query the current status of the CAN controller (e.g., error counters, bus state).
 * 2. To invoke a user-registered status indicator callback, if any.
 * 3. To call a user-registered bus hook function for periodic tasks, if any.
 *
 * @param[in] arg The argument passed to the callback, which is a pointer to the `rt_can_device`.
 * @return void
 */
static void cantimeout(void *arg)
{
    rt_can_t can;

    can = (rt_can_t)arg;
    RT_ASSERT(can);
    rt_device_control((rt_device_t)can, RT_CAN_CMD_GET_STATUS, (void *)&can->status);

    if (can->status_indicate.ind != RT_NULL)
    {
        can->status_indicate.ind(can, can->status_indicate.args);
    }
#ifdef RT_CAN_USING_BUS_HOOK
    if(can->bus_hook)
    {
        can->bus_hook(can);
    }
#endif /*RT_CAN_USING_BUS_HOOK*/
    if (can->timerinitflag == 1)
    {
        can->timerinitflag = 0xFF;
    }
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops can_device_ops =
{
    rt_can_init,
    rt_can_open,
    rt_can_close,
    rt_can_read,
    rt_can_write,
    rt_can_control
};
#endif

/*
 * can register
 */
rt_err_t rt_hw_can_register(struct rt_can_device    *can,
                            const char              *name,
                            const struct rt_can_ops *ops,
                            void                    *data)
{
    struct rt_device *device;
    RT_ASSERT(can != RT_NULL);

    device = &(can->parent);

    device->type        = RT_Device_Class_CAN;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
#ifdef RT_CAN_USING_HDR
    can->hdr            = RT_NULL;
#endif
    can->can_rx         = RT_NULL;
    can->can_tx         = RT_NULL;
    rt_mutex_init(&(can->lock), "can", RT_IPC_FLAG_PRIO);
#ifdef RT_CAN_USING_BUS_HOOK
    can->bus_hook       = RT_NULL;
#endif /*RT_CAN_USING_BUS_HOOK*/

#ifdef RT_CAN_MALLOC_NB_TX_BUFFER
    can->nb_tx_rb_pool = RT_NULL;
#endif

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &can_device_ops;
#else
    device->init        = rt_can_init;
    device->open        = rt_can_open;
    device->close       = rt_can_close;
    device->read        = rt_can_read;
    device->write       = rt_can_write;
    device->control     = rt_can_control;
#endif
    can->ops            = ops;

    can->status_indicate.ind  = RT_NULL;
    can->status_indicate.args = RT_NULL;
    rt_memset(&can->status, 0, sizeof(can->status));
    rt_atomic_store(&can->tx_state, RT_CAN_TX_STATE_DISABLED);
    rt_atomic_store(&can->tx_submit_refcnt, 0);
    rt_atomic_store(&can->tx_active_refcnt, 0);

    device->user_data   = data;

    can->timerinitflag  = 0;
    rt_timer_init(&can->timer,
                  name,
                  cantimeout,
                  (void *)can,
                  can->config.ticks,
                  RT_TIMER_FLAG_PERIODIC);
    /* register a character device */
    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
}

/* ISR for can interrupt */
/**
 * @brief The framework-level ISR handler for CAN devices.
 *
 * This function is called by the low-level BSP ISR and acts as the central
 * dispatcher for all CAN-related interrupt events. It handles both receive
 * events and transmission-complete events.
 *
 * @param[in] can    A pointer to the CAN device structure.
 * @param[in] event  The interrupt event mask, indicating the cause of the interrupt.
 * @return void
 */
void rt_hw_can_isr(struct rt_can_device *can, int event)
{
    rt_bool_t is_rxof_event = RT_FALSE;

    switch (event & 0xff)
    {
    case RT_CAN_EVENT_RXOF_IND:
    {
        rt_base_t level;
        is_rxof_event = RT_TRUE;
        level = rt_hw_local_irq_disable();
        can->status.dropedrcvpkg++;
        rt_hw_local_irq_enable(level);
    }
    /* FALLTHROUGH: RX overflow still tries to fetch one pending frame into software FIFO. */
    case RT_CAN_EVENT_RX_IND:
    {
        struct rt_can_msg tmpmsg;
        struct rt_can_rx_fifo *rx_fifo;
        struct rt_can_msg_list *listmsg = RT_NULL;
#ifdef RT_CAN_USING_HDR
        rt_int8_t hdr;
#endif
        int ch = -1;
        rt_base_t level;
        rt_uint32_t no;

        rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;
        RT_ASSERT(rx_fifo != RT_NULL);
        /* interrupt mode receive */
        RT_ASSERT(can->parent.open_flag & RT_DEVICE_FLAG_INT_RX);

        no = event >> 8;
        ch = can->ops->recvmsg(can, &tmpmsg, no);
        if (ch == -1) break;

        /* disable interrupt */
        level = rt_hw_local_irq_disable();
        can->status.rcvpkg++;
        can->status.rcvchange = 1;
        if (!rt_list_isempty(&rx_fifo->freelist))
        {
            listmsg = rt_list_entry(rx_fifo->freelist.next, struct rt_can_msg_list, list);
            rt_list_remove(&listmsg->list);
#ifdef RT_CAN_USING_HDR
            rt_list_remove(&listmsg->hdrlist);
            if (listmsg->owner != RT_NULL && listmsg->owner->msgs)
            {
                listmsg->owner->msgs--;
            }
            listmsg->owner = RT_NULL;
#endif /*RT_CAN_USING_HDR*/
            RT_ASSERT(rx_fifo->freenumbers > 0);
            rx_fifo->freenumbers--;
        }
        else if (!rt_list_isempty(&rx_fifo->uselist))
        {
            listmsg = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
            if (!is_rxof_event)
            {
                can->status.dropedrcvpkg++;
            }
            rt_list_remove(&listmsg->list);
#ifdef RT_CAN_USING_HDR
            rt_list_remove(&listmsg->hdrlist);
            if (listmsg->owner != RT_NULL && listmsg->owner->msgs)
            {
                listmsg->owner->msgs--;
            }
            listmsg->owner = RT_NULL;
#endif
        }
        /* enable interrupt */
        rt_hw_local_irq_enable(level);

        if (listmsg != RT_NULL)
        {
            rt_memcpy(&listmsg->data, &tmpmsg, sizeof(struct rt_can_msg));
            level = rt_hw_local_irq_disable();
            rt_list_insert_before(&rx_fifo->uselist, &listmsg->list);
#ifdef RT_CAN_USING_HDR
            hdr = tmpmsg.hdr_index;
            if (can->hdr != RT_NULL)
            {
                RT_ASSERT(hdr < can->config.maxhdr && hdr >= 0);
                if (can->hdr[hdr].connected)
                {
                    rt_list_insert_before(&can->hdr[hdr].list, &listmsg->hdrlist);
                    listmsg->owner = &can->hdr[hdr];
                    can->hdr[hdr].msgs++;
                }

            }
#endif
            rt_hw_local_irq_enable(level);
        }

        /* invoke callback */
#ifdef RT_CAN_USING_HDR
        if (can->hdr != RT_NULL && can->hdr[hdr].connected && can->hdr[hdr].filter.ind)
        {
            rt_size_t rx_length;
            RT_ASSERT(hdr < can->config.maxhdr && hdr >= 0);

            level = rt_hw_local_irq_disable();
            rx_length = can->hdr[hdr].msgs * sizeof(struct rt_can_msg);
            rt_hw_local_irq_enable(level);
            if (rx_length)
            {
                can->hdr[hdr].filter.ind(&can->parent, can->hdr[hdr].filter.args, hdr, rx_length);
            }
        }
        else
#endif
        {
            if (can->parent.rx_indicate != RT_NULL)
            {
                rt_size_t rx_length;

                level = rt_hw_local_irq_disable();
                /* get rx length */
                rx_length = rt_list_len(&rx_fifo->uselist)* sizeof(struct rt_can_msg);
                rt_hw_local_irq_enable(level);

                if (rx_length)
                {
                    can->parent.rx_indicate(&can->parent, rx_length);
                }
            }
        }
        break;
    }

    case RT_CAN_EVENT_TX_DONE:
    case RT_CAN_EVENT_TX_FAIL:
    {
        struct rt_can_tx_fifo *tx_fifo;
        rt_uint32_t no;
        rt_base_t level;
        rt_bool_t complete_blocking = RT_FALSE;
        rt_bool_t release_mailbox = RT_FALSE;

        no = event >> 8;
        tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;

        if (tx_fifo != RT_NULL && no < can->config.sndboxnumber)
        {
            level = rt_hw_local_irq_disable();
            if (can->status.sndchange & (1UL << no))
            {
                /*
                 * ERR together with sndchange denotes a normal blocking sender
                 * which timed out while hardware still owned this mailbox. That
                 * sender is gone, so the ISR must reclaim the software slot.
                 */
                if (tx_fifo->buffer[no].result == RT_CAN_SND_RESULT_ERR &&
                    rt_list_isempty(&tx_fifo->buffer[no].list))
                {
                    rt_list_insert_before(&tx_fifo->freelist, &tx_fifo->buffer[no].list);
                    release_mailbox = RT_TRUE;
                }
                else
                {
                    complete_blocking = RT_TRUE;
                }

                /* Consume ownership before wake-up/release so a late TX/abort IRQ cannot complete it twice. */
                can->status.sndchange &= ~(1UL << no);
                if ((event & 0xff) == RT_CAN_EVENT_TX_DONE)
                {
                    tx_fifo->buffer[no].result = RT_CAN_SND_RESULT_OK;
                }
                else
                {
                    tx_fifo->buffer[no].result = RT_CAN_SND_RESULT_ERR;
                }
            }
            rt_hw_local_irq_enable(level);

            if (release_mailbox)
            {
                rt_sem_release(&tx_fifo->sem);
            }
            else if (complete_blocking)
            {
                rt_completion_done(&(tx_fifo->buffer[no].completion));
            }
        }

        if (can->ops->sendmsg_nonblocking != RT_NULL)
        {
            while (RT_TRUE)
            {
                struct rt_can_msg msg_to_send;
                rt_bool_t msg_was_present = RT_FALSE;

                if (!_can_tx_submit_begin(can))
                {
                    break;
                }

                level = rt_hw_local_irq_disable();
                if (rt_ringbuffer_data_len(&can->nb_tx_rb) >= sizeof(struct rt_can_msg))
                {
                    rt_ringbuffer_get(&can->nb_tx_rb, (rt_uint8_t *)&msg_to_send, sizeof(struct rt_can_msg));
                    msg_was_present = RT_TRUE;
                }
                rt_hw_local_irq_enable(level);

                if (!msg_was_present)
                {
                    _can_tx_submit_end(can);
                    break;
                }

                if (can->ops->sendmsg_nonblocking(can, &msg_to_send) != RT_EOK)
                {
                    level = rt_hw_local_irq_disable();
                    rt_ringbuffer_put_force(&can->nb_tx_rb, (rt_uint8_t *)&msg_to_send, sizeof(struct rt_can_msg));
                    rt_hw_local_irq_enable(level);
                    _can_tx_submit_end(can);
                    break;
                }

                _can_tx_submit_end(can);
            }
        }
        break;
    }
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
int cmd_canstat(int argc, void **argv)
{
    static const char *ErrCode[] =
    {
        "No Error!",
        "Warning !",
        "Passive !",
        "Bus Off !"
    };

    if (argc >= 2)
    {
        struct rt_can_status status;
        rt_device_t candev = rt_device_find(argv[1]);
        if (!candev)
        {
            rt_kprintf(" Can't find can device %s\n", argv[1]);
            return -1;
        }
        rt_kprintf(" Found can device: %s...", argv[1]);

        rt_device_control(candev, RT_CAN_CMD_GET_STATUS, &status);
        rt_kprintf("\n Receive...error..count: %010ld. Send.....error....count: %010ld.",
                   status.rcverrcnt, status.snderrcnt);
        rt_kprintf("\n Bit..pad..error..count: %010ld. Format...error....count: %010ld",
                   status.bitpaderrcnt, status.formaterrcnt);
        rt_kprintf("\n Ack.......error..count: %010ld. Bit......error....count: %010ld.",
                   status.ackerrcnt, status.biterrcnt);
        rt_kprintf("\n CRC.......error..count: %010ld. Error.code.[%010ld]: ",
                   status.crcerrcnt, status.errcode);
        switch (status.errcode)
        {
        case 0:
            rt_kprintf("%s.", ErrCode[0]);
            break;
        case 1:
            rt_kprintf("%s.", ErrCode[1]);
            break;
        case 2:
        case 3:
            rt_kprintf("%s.", ErrCode[2]);
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            rt_kprintf("%s.", ErrCode[3]);
            break;
        }
        rt_kprintf("\n Total.receive.packages: %010ld. Dropped.receive.packages: %010ld.",
                   status.rcvpkg, status.dropedrcvpkg);
        rt_kprintf("\n Total..send...packages: %010ld. Dropped...send..packages: %010ld.\n",
                   status.sndpkg + status.dropedsndpkg, status.dropedsndpkg);
    }
    else
    {
        rt_kprintf(" Invalid Call %s\n", argv[0]);
        rt_kprintf(" Please using %s cannamex .Here canname is driver name and x is candrive number.\n", argv[0]);
    }
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_canstat, canstat, stat can device status);
#endif

