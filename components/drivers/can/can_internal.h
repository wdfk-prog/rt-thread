#ifndef __RT_CAN_INTERNAL_H__
#define __RT_CAN_INTERNAL_H__

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "can_core.h"

#ifndef RT_CAN_TX_REQUEST_COUNT
#define RT_CAN_TX_REQUEST_COUNT RT_CANMSG_BOX_SZ
#endif

enum rt_can_tx_req_state
{
    RT_CAN_TX_REQ_FREE = 0,
    RT_CAN_TX_REQ_QUEUED,
    RT_CAN_TX_REQ_SUBMITTING,
    RT_CAN_TX_REQ_HW_PENDING,
    RT_CAN_TX_REQ_DONE,
    RT_CAN_TX_REQ_ERROR,
};

struct rt_can_tx_request
{
    struct rt_list_node node;
    struct rt_can_msg msg;
    enum rt_can_tx_req_state state;
    rt_err_t result;
    rt_int16_t mailbox;
    rt_bool_t blocking;
    struct rt_completion completion;
};

struct rt_can_runtime
{
    struct rt_can_core core;
};

static rt_inline struct rt_can_runtime *rt_can_runtime_get(struct rt_can_device *can)
{
    return (struct rt_can_runtime *)can->can_tx;
}

rt_err_t rt_can_tx_runtime_init(struct rt_can_device *can, struct rt_can_runtime *runtime);
void rt_can_tx_runtime_deinit(struct rt_can_runtime *runtime);
rt_ssize_t rt_can_tx_write_core(struct rt_can_device *can, const struct rt_can_msg *msg,
                                rt_size_t size, rt_bool_t blocking);
void rt_can_tx_isr_core(struct rt_can_device *can, int event);

rt_ssize_t rt_can_rx_read_core(struct rt_can_device *can, struct rt_can_msg *data,
                               rt_ssize_t msgs);
void rt_can_rx_isr_core(struct rt_can_device *can, int event, rt_bool_t overflow);

#endif
