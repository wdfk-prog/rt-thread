#ifndef __RT_CAN_STATE_H__
#define __RT_CAN_STATE_H__

#include <drivers/dev_can.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RT_CAN_CMD_RECOVER                    0x22
#define RT_CAN_CAP_MANUAL_RECOVERY            (1UL << 2)

enum rt_can_bus_state
{
    RT_CAN_BUS_UNKNOWN = 0,
    RT_CAN_BUS_ERROR_ACTIVE,
    RT_CAN_BUS_ERROR_WARNING,
    RT_CAN_BUS_ERROR_PASSIVE,
    RT_CAN_BUS_OFF,
};

/**
 * @brief Report a controller bus-state transition to the CAN framework.
 *
 * This function is ISR-safe. Existing BSPs do not need to call it; drivers
 * that expose precise bus state may opt in without changing rt_hw_can_isr().
 *
 * @param can CAN device.
 * @param state New CAN bus state.
 */
void rt_hw_can_bus_state(struct rt_can_device *can, enum rt_can_bus_state state);

/**
 * @brief Request controller recovery after BUS_OFF.
 *
 * @param can CAN device.
 * @return RT_EOK when recovery succeeds, otherwise an error code.
 */
rt_err_t rt_can_recover(struct rt_can_device *can);

#ifdef __cplusplus
}
#endif

#endif
