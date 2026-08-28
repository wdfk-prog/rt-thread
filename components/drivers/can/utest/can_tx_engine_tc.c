/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/can_tx.h>
#include <utest.h>

static struct rt_can_device _fake_can;
static rt_uint32_t _submit_count;
static rt_uint32_t _last_mailbox;
static rt_uint32_t _last_id;
static rt_uint32_t _callback_count;
static rt_err_t _callback_result;
static volatile rt_bool_t _blocking_done;

static rt_err_t _fake_configure(struct rt_can_device *can, struct can_configure *cfg)
{
    RT_UNUSED(can);
    RT_UNUSED(cfg);
    return RT_EOK;
}

static rt_err_t _fake_control(struct rt_can_device *can, int cmd, void *arg)
{
    RT_UNUSED(can);
    RT_UNUSED(cmd);
    RT_UNUSED(arg);
    return RT_EOK;
}

static rt_ssize_t _fake_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
    const struct rt_can_msg *msg = (const struct rt_can_msg *)buf;

    RT_UNUSED(can);
    _submit_count++;
    _last_mailbox = boxno;
    _last_id = msg->id;
    return RT_EOK;
}

static rt_ssize_t _fake_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    RT_UNUSED(can);
    RT_UNUSED(buf);
    RT_UNUSED(fifo);
    return -RT_ERROR;
}

static const struct rt_can_ops _fake_ops =
{
    _fake_configure,
    _fake_control,
    _fake_sendmsg,
    _fake_recvmsg,
    RT_NULL,
};

static void _done_cb(struct rt_can_device *can, const struct rt_can_msg *msg,
                     rt_err_t result, void *arg)
{
    RT_UNUSED(can);
    RT_UNUSED(msg);
    RT_UNUSED(arg);
    _callback_count++;
    _callback_result = result;
}

static void can_tx_fifo_schedule_tc(void)
{
    struct rt_can_msg msgs[2] = {0};
    rt_device_t dev = (rt_device_t)&_fake_can;

    msgs[0].id = 0x101;
    msgs[0].ide = RT_CAN_STDID;
    msgs[0].rtr = RT_CAN_DTR;
    msgs[0].len = 1;
    msgs[0].data[0] = 0x11;
    msgs[0].nonblocking = 1;
    msgs[1] = msgs[0];
    msgs[1].id = 0x102;

    uassert_int_equal(rt_device_write(dev, 0, msgs, sizeof(msgs)), sizeof(msgs));
    uassert_int_equal(_submit_count, 1);
    uassert_int_equal(_last_id, 0x101);

    rt_hw_can_isr(&_fake_can, RT_CAN_EVENT_TX_DONE | (_last_mailbox << 8));
    uassert_int_equal(_submit_count, 2);
    uassert_int_equal(_last_id, 0x102);
    rt_hw_can_isr(&_fake_can, RT_CAN_EVENT_TX_DONE | (_last_mailbox << 8));
}

static void can_tx_callback_once_tc(void)
{
    struct rt_can_msg msg = {0};
    rt_uint32_t mailbox;

    msg.id = 0x201;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 1;

    _callback_count = 0;
    uassert_int_equal(rt_can_send_async(&_fake_can, &msg, _done_cb, RT_NULL), RT_EOK);
    mailbox = _last_mailbox;
    rt_hw_can_isr(&_fake_can, RT_CAN_EVENT_TX_DONE | (mailbox << 8));
    rt_hw_can_isr(&_fake_can, RT_CAN_EVENT_TX_DONE | (mailbox << 8));
    uassert_int_equal(_callback_count, 1);
    uassert_int_equal(_callback_result, RT_EOK);
}

static void _blocking_writer(void *parameter)
{
    struct rt_can_msg msg = {0};

    RT_UNUSED(parameter);
    msg.id = 0x301;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 1;
    msg.nonblocking = 0;
    rt_device_write((rt_device_t)&_fake_can, 0, &msg, sizeof(msg));
    _blocking_done = RT_TRUE;
}

static void can_tx_timeout_keeps_owner_tc(void)
{
    struct rt_can_msg next = {0};
    rt_thread_t tid;
    rt_uint32_t first_mailbox;
    rt_uint32_t submit_before;

    _blocking_done = RT_FALSE;
    _submit_count = 0;
    tid = rt_thread_create("cantmo", _blocking_writer, RT_NULL,
                           UTEST_THR_STACK_SIZE, 20, 10);
    uassert_not_null(tid);
    rt_thread_startup(tid);

    while (_submit_count == 0)
        rt_thread_mdelay(1);
    first_mailbox = _last_mailbox;
    while (!_blocking_done)
        rt_thread_mdelay(1);

    submit_before = _submit_count;
    next.id = 0x302;
    next.ide = RT_CAN_STDID;
    next.rtr = RT_CAN_DTR;
    next.len = 1;
    next.nonblocking = 1;
    uassert_int_equal(rt_device_write((rt_device_t)&_fake_can, 0, &next, sizeof(next)), sizeof(next));
    uassert_int_equal(_submit_count, submit_before);

    rt_hw_can_isr(&_fake_can, RT_CAN_EVENT_TX_DONE | (first_mailbox << 8));
    uassert_int_equal(_submit_count, submit_before + 1);
    uassert_int_equal(_last_id, 0x302);
    rt_hw_can_isr(&_fake_can, RT_CAN_EVENT_TX_DONE | (_last_mailbox << 8));
    rt_thread_delete(tid);
}

static rt_err_t utest_tc_init(void)
{
    rt_err_t ret;

    rt_memset(&_fake_can, 0, sizeof(_fake_can));
    _fake_can.config = (struct can_configure)CANDEFAULTCONFIG;
    _fake_can.config.sndboxnumber = 1;
    _fake_can.config.msgboxsz = 4;
    _fake_can.config.ticks = 100;
    _submit_count = 0;
    _last_mailbox = 0;
    _last_id = 0;

    ret = rt_hw_can_register(&_fake_can, "cantx0", &_fake_ops, RT_NULL);
    if (ret != RT_EOK)
        return ret;
    return rt_device_open((rt_device_t)&_fake_can,
                          RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
}

static rt_err_t utest_tc_cleanup(void)
{
    rt_device_close((rt_device_t)&_fake_can);
    rt_device_unregister((rt_device_t)&_fake_can);
    return RT_EOK;
}

static void testcase(void)
{
    UTEST_UNIT_RUN(can_tx_fifo_schedule_tc);
    UTEST_UNIT_RUN(can_tx_callback_once_tc);
    UTEST_UNIT_RUN(can_tx_timeout_keeps_owner_tc);
}

UTEST_TC_EXPORT(testcase, "components.drivers.can.tx_engine",
                utest_tc_init, utest_tc_cleanup, 10);
