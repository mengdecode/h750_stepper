/**
 * @file main.c
 * @brief 主程序入口
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          步进电机闭环控制
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

