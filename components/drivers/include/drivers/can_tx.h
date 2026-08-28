#ifndef __RT_CAN_TX_H__
#define __RT_CAN_TX_H__

#include <drivers/dev_can.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RT_CAN_CMD_GET_CAPABILITIES      0x1e

#define RT_CAN_CAP_TX_ORDERED_MAILBOX    (1UL << 0)
#define RT_CAN_CAP_TX_ABORT              (1UL << 1)

#define RT_CAN_CMD_ABORT_TX              0x1f
#define RT_CAN_CMD_FLUSH_TX              0x20
#define RT_CAN_CMD_RESUME_TX             0x21

/**
 * @brief CAN TX quiesce policy.
 */
enum rt_can_quiesce_policy
{
    RT_CAN_QUIESCE_DRAIN = 0,
    RT_CAN_QUIESCE_DROP_QUEUED,
    RT_CAN_QUIESCE_ABORT_ALL,
};

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

/**
 * @brief Quiesce CAN TX according to the requested policy.
 *
 * @param can CAN device.
 * @param policy Drain, drop queued requests, or abort all accepted TX.
 * @param timeout Maximum ticks to wait for the TX engine to become idle.
 * @return RT_EOK when QUIESCED, otherwise an error code.
 */
rt_err_t rt_can_quiesce(struct rt_can_device *can,
                        enum rt_can_quiesce_policy policy,
                        rt_tick_t timeout);

/**
 * @brief Resume TX admission from QUIESCED state.
 *
 * @param can CAN device.
 * @return RT_EOK on success, otherwise an error code.
 */
rt_err_t rt_can_tx_resume(struct rt_can_device *can);

#ifdef __cplusplus
}
#endif

#endif
