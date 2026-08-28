#ifndef __RT_CAN_TX_H__
#define __RT_CAN_TX_H__

#include <drivers/dev_can.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RT_CAN_CMD_GET_CAPABILITIES      0x1e

#define RT_CAN_CAP_TX_ORDERED_MAILBOX    (1UL << 0)

/**
 * @brief Asynchronous CAN TX terminal callback.
 *
 * The callback runs after the framework has released its internal TX lock. A
 * hardware completion normally originates from ISR context, so the callback
 * must not sleep or perform blocking operations.
 */
typedef void (*rt_can_tx_done_cb)(struct rt_can_device *can,
                                  const struct rt_can_msg *msg,
                                  rt_err_t result,
                                  void *arg);

/**
 * @brief Submit one CAN frame and report its terminal result asynchronously.
 *
 * @param can CAN device.
 * @param msg Frame to copy into the framework TX request pool.
 * @param callback Optional terminal callback.
 * @param arg User argument passed to callback.
 * @return RT_EOK when accepted by the framework, otherwise an error code.
 */
rt_err_t rt_can_send_async(struct rt_can_device *can,
                           const struct rt_can_msg *msg,
                           rt_can_tx_done_cb callback,
                           void *arg);

#ifdef __cplusplus
}
#endif

#endif
