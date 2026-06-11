/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-17     supperthomas first version
 */


 #define LOG_TAG     "main"     
#define LOG_LVL     LOG_LVL_DBG   

#include <ulog.h>                

#include <rtthread.h>
#include <board.h>

/* defined the LED0 pin: PI8 */
#define LED_PIN_B    GET_PIN(E,4)
#define LED_PIN_G    GET_PIN(E,5)
#define LED_PIN_R    GET_PIN(E,6)
#define motor_clk    GET_PIN(A,0)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED_PIN_B, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_PIN_G, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_PIN_R, PIN_MODE_OUTPUT);

    while (1)
    {

        rt_pin_write(LED_PIN_B, PIN_HIGH);

        rt_thread_mdelay(500);
			  rt_pin_write(LED_PIN_G, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN_R, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN_R, PIN_LOW);
        rt_thread_mdelay(500);
			
			  rt_thread_mdelay(500);
        rt_pin_write(LED_PIN_B, PIN_LOW);
			  rt_thread_mdelay(500);
        rt_pin_write(LED_PIN_G, PIN_LOW);
			  rt_thread_mdelay(500);
//					rt_pin_write(motor_clk, PIN_LOW);
//					 rt_hw_us_delay(100) ;
//					rt_pin_write(motor_clk, PIN_HIGH);
//					 rt_hw_us_delay(100) ;
    }
}

/*
 * ????:???????????????
 * ????? pulse_encoder_sample ???????
 * ??????:pulse_encoder_sample
 * ????:?? 500 ms ???????????????,???????,?????????????
*/

#include <rtthread.h>
#include <rtdevice.h>

#define PULSE_ENCODER_DEV_NAME    "pulse3"    /* ??????? */

static int pulse_encoder_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_device_t pulse_encoder_dev = RT_NULL;   /* ????????? */
    rt_uint32_t index;
    rt_int32_t count;

    /* ????????? */
    pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
    if (pulse_encoder_dev == RT_NULL)
    {
        rt_kprintf("pulse encoder sample run failed! can't find %s device!\n", PULSE_ENCODER_DEV_NAME);
        return RT_ERROR;
    }

    /* ????????? */
    ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", PULSE_ENCODER_DEV_NAME);
        return ret;
    }

    for (index = 0; index <= 10; index ++)
    {
        rt_thread_mdelay(500);
        /* ?????????? */
        rt_device_read(pulse_encoder_dev, 0, &count, 1);
        /* ?????????? */
        rt_device_control(pulse_encoder_dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
        rt_kprintf("get count %d\n",count);
    }

    rt_device_close(pulse_encoder_dev);
    return ret;
}
/* ??? msh ????? */
MSH_CMD_EXPORT(pulse_encoder_sample, pulse encoder sample);

/*
 * ????:???? hwtimer ??????
 * ????? hwtimer_sample ???????
 * ??????:hwtimer_sample
 * ????:???????????????????tick?,2?tick?????????????????
*/

#include <rtthread.h>
#include <rtdevice.h>

#define HWTIMER_DEV_NAME   "timer4"     /* ????? */

/* ????????? */
static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("this is hwtimer timeout callback fucntion!\n");
    rt_kprintf("tick is :%d !\n", rt_tick_get());

    return 0;
}

static int hwtimer_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* ?????? */
    rt_device_t hw_dev = RT_NULL;   /* ??????? */
    rt_hwtimer_mode_t mode;         /* ????? */
    rt_uint32_t freq = 10000;               /* ???? */

    /* ??????? */
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* ????????? */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    /* ???????? */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    /* ??????(??????,???1Mhz ? ?????????) */
    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    /* ???????????(????,???HWTIMER_MODE_ONESHOT)*/
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    /* ?????????5s?????? */
    timeout_s.sec = 5;      /* ? */
    timeout_s.usec = 0;     /* ?? */
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    /* ??3500ms */
    rt_thread_mdelay(3500);

    /* ???????? */
    rt_device_read(hw_dev, 0, &timeout_s, sizeof(timeout_s));
    rt_kprintf("Read: Sec = %d, Usec = %d\n", timeout_s.sec, timeout_s.usec);

    return ret;
}
/* ??? msh ????? */
MSH_CMD_EXPORT(hwtimer_sample, hwtimer sample);

