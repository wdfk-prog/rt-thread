/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2012-10-27     heyuanjie87       first version.
 * 2013-05-17     aozima            initial alarm event & mutex in system init.
 * 2020-10-15     zhangsz           add alarm flags hour minute second.
 * 2020-11-09     zhangsz           fix alarm set when modify rtc time.
 * 2024-06-25     wdfk-prog         use timestamp list
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>

#if (defined(RT_USING_RTC) && defined(RT_USING_ALARM))

#define RT_RTC_YEARS_MAX         137
#ifdef RT_USING_SOFT_RTC
#define RT_ALARM_DELAY             0
#else
#define RT_ALARM_DELAY             2
#endif

#define ALARM_STACK_SIZE            2048
#define ALARM_PRIORITY              10
#define ALARM_TICKS                 5
struct rt_alarm_container
{
    rt_alarm_t head;
    struct rt_mutex mutex;
    struct rt_event event;

    rt_uint32_t flag;
    time_t context_timestamp;
};

static struct rt_alarm_container _container;

static rt_err_t alarm_set(rt_alarm_t alarm);

static time_t set_timer_context(void)
{
    get_timestamp(&_container.context_timestamp);
    return _container.context_timestamp;
}

static time_t get_timer_context(void)
{
    return _container.context_timestamp;
}

static time_t get_elapsed_time(void)
{
    time_t timestamp = 0;
    get_timestamp(&timestamp);

    return (timestamp - _container.context_timestamp);
}

static rt_err_t start_timer(rt_alarm_t alarm)
{
    /* start alarm */
    _container.flag |= RT_ALARM_STATE_START;

    return alarm_set(alarm);
}

static rt_err_t stop_timer(rt_alarm_t alarm)
{
    /* stop alarm */
    _container.flag &= ~RT_ALARM_STATE_START;

    return alarm_set(alarm);
}

/**
 * @brief Sets a timeout with the duration "timestamp"
 */
static void set_timeout(rt_alarm_t alarm)
{
    time_t timestamp = get_elapsed_time() + RT_ALARM_DELAY;
    /* In case deadline too soon */
    if(alarm->timestamp < timestamp)
    {
        alarm->timestamp = timestamp;
    }

    start_timer(alarm);
}

/**
 * @brief Adds a timer to the list.
 *
 * @remark The list is automatically sorted. The list head always contains the
 *     next timer to expire.
 */
static void insert_timer(rt_alarm_t node)
{
    rt_alarm_t cur = _container.head;
    rt_alarm_t next = _container.head->next;

    while (cur->next != RT_NULL)
    {  
        if(node->timestamp  > next->timestamp)
        {
            cur = next;
            next = next->next;
        }
        else
        {
            cur->next = node;
            node->next = next;
            return;
        }
    }
    cur->next = node;
    node->next = RT_NULL;
}

/**
 * @brief Adds or replace the head timer of the list.
 *
 * @remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 */
static void insert_new_head_timer(rt_alarm_t node)
{
    rt_alarm_t cur = _container.head;

    node->next = cur;
    _container.head = node;
    set_timeout(alarm);
}

static rt_err_t alarm_set(rt_alarm_t alarm)
{
    rt_device_t device = RT_NULL;
    struct rt_rtc_wkalarm wkalarm = {0};
    rt_err_t ret = -RT_ERROR;
    struct tm wktime = {0};

    device = rt_device_find("rtc");

    if (device == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (alarm->flag & RT_ALARM_STATE_START)
    {
        wkalarm.enable = RT_TRUE;
    }
    else
    {
        wkalarm.enable = RT_FALSE;
    }

    gmtime_r(&alarm->timestamp, &wktime);
    wkalarm.tm_sec  = wktime.tm_sec;
    wkalarm.tm_min  = wktime.tm_min;
    wkalarm.tm_hour = wktime.tm_hour;
    wkalarm.tm_mday = wktime.tm_mday;
    wkalarm.tm_mon  = wktime.tm_mon;
    wkalarm.tm_year = wktime.tm_year;

    ret = rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_ALARM, &wkalarm);
    if ((ret == RT_EOK) && wkalarm.enable)
    {
        ret = rt_device_control(device, RT_DEVICE_CTRL_RTC_GET_ALARM, &wkalarm);
        if (ret == RT_EOK)
        {
            /*
              some RTC device like RX8025,it's alarms precision is 1 minute.
              in this case,low level RTC driver should set wkalarm->tm_sec to 0.
            */
            alarm->wktime.tm_sec   = wkalarm.tm_sec;
            alarm->wktime.tm_min   = wkalarm.tm_min;
            alarm->wktime.tm_hour  = wkalarm.tm_hour;
            alarm->wktime.tm_mday  = wkalarm.tm_mday;
            alarm->wktime.tm_mon   = wkalarm.tm_mon;
            alarm->wktime.tm_year  = wkalarm.tm_year;
            alarm->timestamp = timegm(&wktime);
        }
    }

    return (ret);
}

static void alarm_update(rt_uint32_t event)
{
    rt_alarm_t cur = RT_NULL;
    rt_mutex_take(&_container.mutex, RT_WAITING_FOREVER);

    time_t old =  get_timer_context();
    time_t now =  set_timer_context();
    time_t delta = now  - old; /*intentional wrap around */
    
    /* update timeStamp based upon new Time Reference*/
    /* because delta context should never exceed 2^32*/
    if (_container.head != RT_NULL)
    {
        cur = _container.head;
        do {
            if (cur->timestamp > delta)
            {
                cur->timestamp -= delta;
            }
            else
            {
                cur->timestamp = 0;
            }
            cur = cur->next;
        } while(cur != RT_NULL);
    }

    /* Execute expired timer and update the list, prevent lost processing due to increased timestamps */
    while ((_container.head != RT_NULL) && ((_container.head->timestamp == 0U) || (_container.head->timestamp < get_elapsed_time())))
    {
        cur = _container.head;
        _container.head = _container.head->next;
        if (alarm->callback != RT_NULL)
        {
            alarm->callback(alarm, timestamp);
        }
        rt_alarm_start(cur);
    }

    /* start the next _container.head if it exists and it is not pending*/
    if(( _container.head != RT_NULL) && (_container.head->flag & RT_ALARM_STATE_START))
    {
        set_timeout(_container.head);
    }

    rt_mutex_release(&_container.mutex);
}

static int days_of_year_month(int tm_year, int tm_mon)
{
    int ret, year;

    year = tm_year + 1900;
    if (tm_mon == 1)
    {
        ret = 28 + ((!(year % 4) && (year % 100)) || !(year % 400));
    }
    else if (((tm_mon <= 6) && (tm_mon % 2 == 0)) || ((tm_mon > 6) && (tm_mon % 2 == 1)))
    {
        ret = 31;
    }
    else
    {
        ret = 30;
    }

    return (ret);
}

static rt_bool_t is_valid_date(struct tm *date)
{
    if ((date->tm_year < 0) || (date->tm_year > RT_RTC_YEARS_MAX))
    {
        return (RT_FALSE);
    }

    if ((date->tm_mon < 0) || (date->tm_mon > 11))
    {
        return (RT_FALSE);
    }

    if ((date->tm_mday < 1) || \
            (date->tm_mday > days_of_year_month(date->tm_year, date->tm_mon)))
    {
        return (RT_FALSE);
    }

    return (RT_TRUE);
}

static rt_err_t alarm_setup(rt_alarm_t alarm, struct tm *wktime)
{
    rt_err_t ret = -RT_ERROR;
    time_t timestamp = (time_t)0;
    struct tm *setup, now;

    setup = &alarm->wktime;
    *setup = *wktime;
    /* get time of now */
    get_timestamp(&timestamp);
    gmtime_r(&timestamp, &now);

    /* if these are a "don't care" value,we set them to now*/
    if ((setup->tm_sec > 59) || (setup->tm_sec < 0))
        setup->tm_sec = now.tm_sec;
    if ((setup->tm_min > 59) || (setup->tm_min < 0))
        setup->tm_min = now.tm_min;
    if ((setup->tm_hour > 23) || (setup->tm_hour < 0))
        setup->tm_hour = now.tm_hour;

    switch (alarm->flag & 0xFF00)
    {
    case RT_ALARM_SECOND:
    {
        alarm->wktime.tm_hour = now.tm_hour;
        alarm->wktime.tm_min = now.tm_min;
        alarm->wktime.tm_sec = now.tm_sec + 1;
        if (alarm->wktime.tm_sec > 59)
        {
            alarm->wktime.tm_sec = 0;
            alarm->wktime.tm_min = alarm->wktime.tm_min + 1;
            if (alarm->wktime.tm_min > 59)
            {
                alarm->wktime.tm_min = 0;
                alarm->wktime.tm_hour = alarm->wktime.tm_hour + 1;
                if (alarm->wktime.tm_hour > 23)
                {
                    alarm->wktime.tm_hour = 0;
                }
            }
        }
    }
    break;
    case RT_ALARM_MINUTE:
    {
        alarm->wktime.tm_hour = now.tm_hour;
        alarm->wktime.tm_min = now.tm_min + 1;
        if (alarm->wktime.tm_min > 59)
        {
            alarm->wktime.tm_min = 0;
            alarm->wktime.tm_hour = alarm->wktime.tm_hour + 1;
            if (alarm->wktime.tm_hour > 23)
            {
                alarm->wktime.tm_hour = 0;
            }
        }
    }
    break;
    case RT_ALARM_HOUR:
    {
        alarm->wktime.tm_hour = now.tm_hour + 1;
        if (alarm->wktime.tm_hour > 23)
        {
            alarm->wktime.tm_hour = 0;
        }
    }
    break;
    case RT_ALARM_DAILY:
    {
        /* do nothing but needed */
    }
    break;
    case RT_ALARM_ONESHOT:
    {
        /* if these are "don't care" value we set them to now */
        if (setup->tm_year == RT_ALARM_TM_NOW)
            setup->tm_year = now.tm_year;
        if (setup->tm_mon == RT_ALARM_TM_NOW)
            setup->tm_mon = now.tm_mon;
        if (setup->tm_mday == RT_ALARM_TM_NOW)
            setup->tm_mday = now.tm_mday;
        /* make sure the setup is valid */
        if (!is_valid_date(setup))
            goto _exit;
    }
    break;
    case RT_ALARM_WEEKLY:
    {
        /* if tm_wday is a "don't care" value we set it to now */
        if ((setup->tm_wday < 0) || (setup->tm_wday > 6))
            setup->tm_wday = now.tm_wday;
    }
    break;
    case RT_ALARM_MONTHLY:
    {
        /* if tm_mday is a "don't care" value we set it to now */
        if ((setup->tm_mday < 1) || (setup->tm_mday > 31))
            setup->tm_mday = now.tm_mday;
    }
    break;
    case RT_ALARM_YAERLY:
    {
        /* if tm_mon is a "don't care" value we set it to now */
        if ((setup->tm_mon < 0) || (setup->tm_mon > 11))
            setup->tm_mon = now.tm_mon;

        if (setup->tm_mon == 1)
        {
            /* tm_mon is February */

            /* tm_mday should be 1~29.otherwise,it's a "don't care" value */
            if ((setup->tm_mday < 1) || (setup->tm_mday > 29))
                setup->tm_mday = now.tm_mday;
        }
        else if (((setup->tm_mon <= 6) && (setup->tm_mon % 2 == 0)) || \
                 ((setup->tm_mon > 6) && (setup->tm_mon % 2 == 1)))
        {
            /* Jan,Mar,May,Jul,Aug,Oct,Dec */

            /* tm_mday should be 1~31.otherwise,it's a "don't care" value */
            if ((setup->tm_mday < 1) || (setup->tm_mday > 31))
                setup->tm_mday = now.tm_mday;
        }
        else
        {
            /* tm_mday should be 1~30.otherwise,it's a "don't care" value */
            if ((setup->tm_mday < 1) || (setup->tm_mday > 30))
                setup->tm_mday = now.tm_mday;
        }
    }
    break;
    default:
    {
        goto _exit;
    }
    }

    if ((setup->tm_hour == 23) && (setup->tm_min == 59) && (setup->tm_sec == 59))
    {
        /*
           for insurance purposes, we will generate an alarm
           signal two seconds ahead of.
        */
        setup->tm_sec = 60 - RT_ALARM_DELAY;
    }
    ret = RT_EOK;

_exit:

    return (ret);
}

/** \brief send a rtc alarm event
 *
 * \param dev pointer to RTC device(currently unused,you can ignore it)
 * \param event RTC event(currently unused)
 * \return none
 */
void rt_alarm_update(rt_device_t dev, rt_uint32_t event)
{
    rt_event_send(&_container.event, event);
}

/** \brief modify the alarm setup
 *
 * \param alarm pointer to alarm
 * \param cmd control command
 * \param arg argument
 */
rt_err_t rt_alarm_control(rt_alarm_t alarm, int cmd, void *arg)
{
    rt_err_t ret = -RT_ERROR;

    RT_ASSERT(alarm != RT_NULL);

    rt_mutex_take(&_container.mutex, RT_WAITING_FOREVER);
    switch (cmd)
    {
    case RT_ALARM_CTRL_MODIFY:
    {
        struct rt_alarm_setup *setup;

        RT_ASSERT(arg != RT_NULL);
        setup = arg;
        rt_alarm_stop(alarm);
        alarm->flag = setup->flag & 0xFF00;
        alarm->wktime = setup->wktime;
        ret = alarm_setup(alarm, &alarm->wktime);
    }
    break;
    }

    rt_mutex_release(&_container.mutex);

    return (ret);
}

/** \brief start an alarm
 *
 * \param alarm pointer to alarm
 * \return RT_EOK
 */
rt_err_t rt_alarm_start(rt_alarm_t alarm)
{
    rt_err_t ret = RT_EOK;

    if (alarm == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&_container.mutex, RT_WAITING_FOREVER);

    if (!(alarm->flag & RT_ALARM_STATE_START))
    {
        if (alarm_setup(alarm, &alarm->wktime) != RT_EOK)
        {
            ret = -RT_ERROR;
            goto _exit;
        }

        alarm->flag &= ~RT_ALARM_STATE_STOP;
        alarm->flag |= RT_ALARM_STATE_START;
        alarm->timestamp = timegm(setup.wktime);

        if(_container.head == RT_NULL)
        {
            set_timer_context();
            insert_new_head_timer(alarm);
        }
        else
        {
            alarm->timestamp += get_elapsed_time();
            if(alarm->timestamp < _container.head->setup.timestamp)
            {
                insert_new_head_timer(alarm);
            }
            else
            {
                insert_timer(alarm);
            }
        }
    }

_exit:
    rt_mutex_release(&_container.mutex);

    return ret;
}

/** \brief stop an alarm
 *
 * \param alarm pointer to alarm
 * \return RT_EOK
 */
rt_err_t rt_alarm_stop(rt_alarm_t alarm)
{
    rt_err_t ret = RT_EOK;

    if (alarm == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&_container.mutex, RT_WAITING_FOREVER);

    if (!(alarm->flag & RT_ALARM_STATE_START))
    {
        goto _exit;
    }

    /* stop alarm */
    alarm->flag &= ~RT_ALARM_STATE_START;
    alarm->flag |= RT_ALARM_STATE_STOP;

    rt_alarm_t prev = _container.head;
    rt_alarm_t cur  = _container.head;
    /* List is empty or the Obj to stop does not exist  */
    if(RT_NULL != _container.head)
    {
        if(_container.head == &alarm->setup) /* Stop the Head */
        {
            if(_container.head->next != RT_NULL)
            {
                _container.head = _container.head->next;
                set_timeout();
            }
            else
            {
                ret = stop_timer(alarm);
                _container.head = RT_NULL;
            }
        }
        else /* Stop an object within the list */
        {
            while(cur != RT_NULL)
            {
                if(cur == &alarm->setup)
                {
                    if( cur->next != RT_NULL )
                    {
                        cur = cur->next;
                        prev->next = cur;
                    }
                    else
                    {
                        cur = RT_NULL;
                        prev->next = cur;
                    }
                    break;
                }
                else
                {
                    prev = cur;
                    cur = cur->next;
                }
            }
        }
    }

_exit:
    rt_mutex_release(&_container.mutex);

    return ret;
}

/** \brief delete an alarm
 *
 * \param alarm pointer to alarm
 * \return RT_EOK
 */
rt_err_t rt_alarm_delete(rt_alarm_t alarm)
{
    rt_err_t ret = RT_EOK;

    if (alarm == RT_NULL)
    {
        return -RT_ERROR;
    }

    rt_mutex_take(&_container.mutex, RT_WAITING_FOREVER);

    ret = rt_alarm_stop(alarm);

    alarm->flag &= ~RT_ALARM_STATE_INITED;

    if(ret == RT_EOK)
    {
        rt_free(alarm);
    }

    rt_mutex_release(&_container.mutex);

    return ret;
}

/** \brief create an alarm
 *
 * \param callback user-defined function to handle alarm events
 * \param setup pointer to setup infomation
 */
rt_alarm_t rt_alarm_create(rt_alarm_callback_t callback, struct rt_alarm_setup *setup)
{
    if (setup == RT_NULL)
    {
        return RT_NULL;
    }

    rt_alarm_t alarm = rt_malloc(sizeof(struct rt_alarm));
    if (alarm == RT_NULL)
    {
        return RT_NULL;
    }

    alarm->wktime = setup->wktime;
    alarm->timestamp = timegm(setup.wktime);
    alarm->flag = setup->flag & 0xFF00;
    /* set initialized state */
    alarm->flag |= RT_ALARM_STATE_INITED;
    alarm->callback = callback;

    return alarm;
}

/** \brief rtc alarm service thread entry
 *
 */
static void rt_alarmsvc_thread(void *param)
{
    rt_uint32_t recv;

    _container.head = RT_NULL;

    while (1)
    {
        if (rt_event_recv(&_container.event, 0xFFFF,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &recv) == RT_EOK)
        {
            alarm_update(recv);
        }
    }
}

struct _alarm_flag
{
    const char* name;
    rt_uint32_t flag;
};

static const struct _alarm_flag _alarm_flag_tbl[] =
{
    {"N",        0xffff}, /* none */
    {"O",       RT_ALARM_ONESHOT}, /* only alarm once */
    {"D",       RT_ALARM_DAILY}, /* alarm everyday */
    {"W",       RT_ALARM_WEEKLY}, /* alarm weekly at Monday or Friday etc. */
    {"Mo",      RT_ALARM_MONTHLY}, /* alarm monthly at someday */
    {"Y",       RT_ALARM_YAERLY}, /* alarm yearly at a certain date */
    {"H",       RT_ALARM_HOUR}, /* alarm each hour at a certain min:second */
    {"M",       RT_ALARM_MINUTE}, /* alarm each minute at a certain second */
    {"S",       RT_ALARM_SECOND}, /* alarm each second */
};

static rt_uint8_t _alarm_flag_tbl_size = sizeof(_alarm_flag_tbl) / sizeof(_alarm_flag_tbl[0]);

static rt_uint8_t get_alarm_flag_index(rt_uint32_t alarm_flag)
{
    for (rt_uint8_t index = 0; index < _alarm_flag_tbl_size; index++)
    {
        alarm_flag &= 0xff00;
        if (alarm_flag == _alarm_flag_tbl[index].flag)
        {
            return index;
        }
    }

    return 0;
}

void rt_alarm_dump(void)
{
    rt_alarm_t node;

    rt_kprintf("| hh:mm:ss | week | flag | en | timestamp |\n");
    rt_kprintf("+----------+------+------+----+-----------+\n");
    for (node = _container.head.next; node != &_container.head; node = node->next)
    {
        rt_uint8_t flag_index = get_alarm_flag_index(node->flag);
        rt_kprintf("| %02d:%02d:%02d |  %2d  |  %2s  | %2d | %d |\n",
            alarm->wktime.tm_hour, alarm->wktime.tm_min, alarm->wktime.tm_sec,
            alarm->wktime.tm_wday, _alarm_flag_tbl[flag_index].name, alarm->flag & RT_ALARM_STATE_START, alarm->timestamp);
    }
    rt_kprintf("+----------+------+------+----+-----------+\n");
}

MSH_CMD_EXPORT_ALIAS(rt_alarm_dump, list_alarm, list alarm info);

/** \brief initialize alarm service system
 *
 * \param none
 * \return none
 */
int rt_alarm_system_init(void)
{
    rt_thread_t tid;

    rt_event_init(&_container.event, "alarmsvc", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&_container.mutex, "alarmsvc", RT_IPC_FLAG_PRIO);

    tid = rt_thread_create("alarmsvc",
                           rt_alarmsvc_thread, RT_NULL,
                           ALARM_STACK_SIZE, ALARM_PRIORITY, ALARM_TICKS);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }

    return 0;
}

INIT_PREV_EXPORT(rt_alarm_system_init);
#endif
