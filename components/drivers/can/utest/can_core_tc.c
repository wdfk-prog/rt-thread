/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <utest.h>

#include "../can_core.h"

static void can_core_init_tc(void)
{
    struct rt_can_core core;

    rt_memset(&core, 0, sizeof(core));

    uassert_int_equal(rt_can_core_init(&core, "cantc"), RT_EOK);
    uassert_int_equal(core.lifecycle_state, RT_CAN_LC_STOPPED);
    uassert_true(rt_list_isempty(&core.tx.free_list));
    uassert_true(rt_list_isempty(&core.tx.pending_list));
    uassert_null(core.tx.requests);
    uassert_null(core.tx.mailbox_owner);
    uassert_int_equal(core.tx.request_count, 0);
    uassert_int_equal(core.tx.mailbox_count, 0);
    uassert_false(core.tx.scheduling);

    rt_mutex_detach(&core.lifecycle_lock);
}

static void testcase(void)
{
    UTEST_UNIT_RUN(can_core_init_tc);
}

UTEST_TC_EXPORT(testcase, "components.drivers.can.core", RT_NULL, RT_NULL, 10);
