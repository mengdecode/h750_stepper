#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

#define SOC_STM32H750XB
#define BOARD_STM32H750_ARTPI

/* RT-Thread Kernel */

/* klibc options */

/* rt_vsnprintf options */

#define RT_KLIBC_USING_VSNPRINTF_LONGLONG
#define RT_KLIBC_USING_VSNPRINTF_STANDARD
#define RT_KLIBC_USING_VSNPRINTF_DECIMAL_SPECIFIERS
#define RT_KLIBC_USING_VSNPRINTF_EXPONENTIAL_SPECIFIERS
#define RT_KLIBC_USING_VSNPRINTF_WRITEBACK_SPECIFIER
#define RT_KLIBC_USING_VSNPRINTF_CHECK_NUL_IN_FORMAT_SPECIFIER
#define RT_KLIBC_USING_VSNPRINTF_INTEGER_BUFFER_SIZE 32
#define RT_KLIBC_USING_VSNPRINTF_DECIMAL_BUFFER_SIZE 32
#define RT_KLIBC_USING_VSNPRINTF_FLOAT_PRECISION 6
#define RT_KLIBC_USING_VSNPRINTF_MAX_INTEGRAL_DIGITS_FOR_DECIMAL 9
#define RT_KLIBC_USING_VSNPRINTF_LOG10_TAYLOR_TERMS 4
/* end of rt_vsnprintf options */

/* rt_vsscanf options */

/* end of rt_vsscanf options */

/* rt_memset options */

/* end of rt_memset options */

/* rt_memcpy options */

/* end of rt_memcpy options */

/* rt_memmove options */

/* end of rt_memmove options */

/* rt_memcmp options */

/* end of rt_memcmp options */

/* rt_strstr options */

/* end of rt_strstr options */

/* rt_strcasecmp options */

/* end of rt_strcasecmp options */

/* rt_strncpy options */

/* end of rt_strncpy options */

/* rt_strcpy options */

/* end of rt_strcpy options */

/* rt_strncmp options */

/* end of rt_strncmp options */

/* rt_strcmp options */

/* end of rt_strcmp options */

/* rt_strlen options */

/* end of rt_strlen options */

/* rt_strnlen options */

/* end of rt_strnlen options */
/* end of klibc options */
#define RT_NAME_MAX 16
#define RT_CPUS_NR 1
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256

/* kservice options */

/* end of kservice options */
#define RT_USING_DEBUG
#define RT_DEBUGING_ASSERT
#define RT_DEBUGING_COLOR
#define RT_DEBUGING_CONTEXT

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
/* end of Inter-Thread communication */

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_MEMHEAP_FAST_MODE
#define RT_USING_MEMHEAP_AS_HEAP
#define RT_USING_MEMHEAP_AUTO_BINDING
#define RT_USING_HEAP
/* end of Memory Management */
#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 256
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x50201
#define RT_BACKTRACE_LEVEL_MAX_NR 32
/* end of RT-Thread Kernel */
#define RT_USING_CACHE
#define RT_USING_HW_ATOMIC
#define RT_USING_CPU_FFS
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 80
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10
#define FINSH_USING_OPTION_COMPLETION

/* DFS: device virtual file system */

/* end of DFS: device virtual file system */

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_UNAMED_PIPE_NUMBER 64
#define RT_USING_SYSTEM_WORKQUEUE
#define RT_SYSTEM_WORKQUEUE_STACKSIZE 2048
#define RT_SYSTEM_WORKQUEUE_PRIORITY 23
#define RT_USING_SERIAL
#define RT_USING_SERIAL_V2
#define RT_SERIAL_BUF_STRATEGY_OVERWRITE
#define RT_SERIAL_USING_DMA
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PWM
#define RT_USING_PULSE_ENCODER
#define RT_USING_SPI
#define RT_USING_TOUCH
#define RT_USING_PIN
#define RT_USING_HWTIMER
/* end of Device Drivers */

/* C/C++ and POSIX layer */

/* ISO-ANSI C layer */

/* Timezone and Daylight Saving Time */

#define RT_LIBC_USING_LIGHT_TZ_DST
#define RT_LIBC_TZ_DEFAULT_HOUR 8
#define RT_LIBC_TZ_DEFAULT_MIN 0
#define RT_LIBC_TZ_DEFAULT_SEC 0
/* end of Timezone and Daylight Saving Time */
/* end of ISO-ANSI C layer */

/* POSIX (Portable Operating System Interface) layer */


/* Interprocess Communication (IPC) */


/* Socket is in the 'Network' category */

/* end of Interprocess Communication (IPC) */
/* end of POSIX (Portable Operating System Interface) layer */
/* end of C/C++ and POSIX layer */

/* Network */

/* end of Network */

/* Memory protection */

/* end of Memory protection */

/* Utilities */

#define RT_USING_ULOG
#define ULOG_OUTPUT_LVL_D
#define ULOG_OUTPUT_LVL 7
#define ULOG_USING_ISR_LOG
#define ULOG_ASSERT_ENABLE
#define ULOG_LINE_BUF_SIZE 256

/* log format */

#define ULOG_USING_COLOR
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG
/* end of log format */
#define ULOG_BACKEND_USING_CONSOLE
/* end of Utilities */

/* Using USB legacy version */

/* end of Using USB legacy version */
/* end of RT-Thread Components */

/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */

/* end of Marvell WiFi */

/* Wiced WiFi */

/* end of Wiced WiFi */

/* CYW43012 WiFi */

/* end of CYW43012 WiFi */

/* BL808 WiFi */

/* end of BL808 WiFi */

/* CYW43439 WiFi */

/* end of CYW43439 WiFi */
/* end of Wi-Fi */

/* IoT Cloud */

/* end of IoT Cloud */
/* end of IoT - internet of things */

/* security packages */

/* end of security packages */

/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* end of JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* XML: Extensible Markup Language */

/* end of XML: Extensible Markup Language */
/* end of language packages */

/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */

/* end of LVGL: powerful and easy-to-use embedded GUI library */

/* u8g2: a monochrome graphic library */

/* end of u8g2: a monochrome graphic library */
/* end of multimedia packages */

/* tools packages */

#define PKG_USING_CMBACKTRACE
#define PKG_CMBACKTRACE_PLATFORM_M7
#define PKG_CMBACKTRACE_DUMP_STACK
#define PKG_CMBACKTRACE_PRINT_ENGLISH
#define PKG_USING_CMBACKTRACE_LATEST_VERSION
#define PKG_CMBACKTRACE_VER_NUM 0x99999
/* end of tools packages */

/* system packages */

/* enhanced kernel services */

/* end of enhanced kernel services */

/* acceleration: Assembly language or algorithmic acceleration packages */

/* end of acceleration: Assembly language or algorithmic acceleration packages */

/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

#define PKG_USING_CMSIS_CORE
#define PKG_USING_CMSIS_CORE_LATEST_VERSION
/* end of CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

/* Micrium: Micrium software products porting for RT-Thread */

/* end of Micrium: Micrium software products porting for RT-Thread */
/* end of system packages */

/* peripheral libraries and drivers */

/* HAL & SDK Drivers */

/* STM32 HAL & SDK Drivers */

#define PKG_USING_STM32H7_HAL_DRIVER
#define PKG_USING_STM32H7_HAL_DRIVER_LATEST_VERSION
#define PKG_USING_STM32H7_CMSIS_DRIVER
#define PKG_USING_STM32H7_CMSIS_DRIVER_LATEST_VERSION
/* end of STM32 HAL & SDK Drivers */

/* Infineon HAL Packages */

/* end of Infineon HAL Packages */

/* Kendryte SDK */

/* end of Kendryte SDK */

/* MM32 HAL & SDK Drivers */

/* end of MM32 HAL & SDK Drivers */

/* WCH HAL & SDK Drivers */

/* end of WCH HAL & SDK Drivers */

/* AT32 HAL & SDK Drivers */

/* end of AT32 HAL & SDK Drivers */

/* HC32 DDL Drivers */

/* end of HC32 DDL Drivers */

/* NXP HAL & SDK Drivers */

/* end of NXP HAL & SDK Drivers */

/* NUVOTON Drivers */

/* end of NUVOTON Drivers */

/* GD32 Drivers */

/* end of GD32 Drivers */

/* HPMicro SDK */

/* end of HPMicro SDK */

/* FT32 HAL & SDK Drivers */

/* end of FT32 HAL & SDK Drivers */

/* NOVOSNS Drivers */

/* end of NOVOSNS Drivers */
/* end of HAL & SDK Drivers */

/* sensors drivers */

/* end of sensors drivers */

/* touch drivers */

/* end of touch drivers */
/* end of peripheral libraries and drivers */

/* AI packages */

/* end of AI packages */

/* Signal Processing and Control Algorithm Packages */

/* end of Signal Processing and Control Algorithm Packages */

/* miscellaneous packages */

/* project laboratory */

/* end of project laboratory */

/* samples: kernel and components samples */

/* end of samples: kernel and components samples */

/* entertainment: terminal games and other interesting software packages */

/* end of entertainment: terminal games and other interesting software packages */
/* end of miscellaneous packages */

/* Arduino libraries */


/* Projects and Demos */

/* end of Projects and Demos */

/* Sensors */

/* end of Sensors */

/* Display */

/* end of Display */

/* Timing */

/* end of Timing */

/* Data Processing */

/* end of Data Processing */

/* Data Storage */

/* Communication */

/* end of Communication */

/* Device Control */

/* end of Device Control */

/* Other */

/* end of Other */

/* Signal IO */

/* end of Signal IO */

/* Uncategorized */

/* end of Arduino libraries */
/* end of RT-Thread online packages */
#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32H7

/* Hardware Drivers Config */

#define SOC_STM32H750_ARTPI

/* On-chip Peripheral Drivers */

#define BSP_SCB_ENABLE_I_CACHE
#define BSP_SCB_ENABLE_D_CACHE
#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_UART1_RX_BUFSIZE 256
#define BSP_UART1_TX_BUFSIZE 0
#define BSP_USING_TIM
#define BSP_USING_TIM4
#define BSP_USING_PWM
#define BSP_USING_PWM2
#define BSP_USING_PWM2_CH1
#define BSP_USING_PWM2_CH2
#define BSP_USING_PWM4
#define BSP_USING_PWM4_CH1
#define BSP_USING_PWM4_CH2
#define BSP_USING_PWM4_CH3
#define BSP_USING_PWM4_CH4
#define BSP_USING_PWM5
#define BSP_USING_PWM5_CH3
#define BSP_USING_PWM5_CH4
#define BSP_USING_PULSE_ENCODER
#define BSP_USING_PULSE_ENCODER3
#define BSP_USING_STEPPER_OPEN_LOOP
#define STEPPEROL_MOTOR_PWM_DEV "pwm2"
#define STEPPEROL_MOTOR_PWM_CH 1
#define STEPPEROL_MOTOR_DIR_PIN 33
#define STEPPEROL_MOTOR_EN_PIN 32
#define STEPPEROL_CTRL_TIMER_DEV "timer4"
#define STEPPEROL_CTRL_PERIOD_US 20000
#define STEPPEROL_MAX_ACCEL 500
#define BSP_USING_STEPPER_COMMON
#define BSP_USING_STEPPER_SPEED_LOOP
#define SPEED_LOOP_MOTOR_COUNT 1
#define SPEED_LOOP_CTRL_PERIOD_US 20000

/* ---- Motor 0 Configuration ---- */

/* Motor 0 */

#define SPEED_LOOP_M0_TIM_INDEX 2
#define SPEED_LOOP_M0_TIM_CH 1
#define SPEED_LOOP_M0_TIM_CLK_HZ 200000000
#define SPEED_LOOP_M0_PULSE_PORT_INDEX 0
#define SPEED_LOOP_M0_PULSE_PIN 0
#define SPEED_LOOP_M0_PULSE_AF 1
#define SPEED_LOOP_M0_DIR_PIN 33
#define SPEED_LOOP_M0_EN_PIN 32
#define SPEED_LOOP_M0_ENC_TIM_INDEX 3
#define SPEED_LOOP_M0_ENC_CH1_PORT_INDEX 2
#define SPEED_LOOP_M0_ENC_CH1_PIN 6
#define SPEED_LOOP_M0_ENC_CH1_AF 2
#define SPEED_LOOP_M0_ENC_CH2_PORT_INDEX 2
#define SPEED_LOOP_M0_ENC_CH2_PIN 7
#define SPEED_LOOP_M0_ENC_CH2_AF 2
#define SPEED_LOOP_M0_CTRL_TIMER "timer4"
#define SPEED_LOOP_M0_STEPS_PER_REV 6400
#define SPEED_LOOP_M0_ENC_COUNTS_PER_REV 4000
#define SPEED_LOOP_M0_ENC_INVERT 1
#define SPEED_LOOP_M0_SPEED_LIMIT 100000
#define SPEED_LOOP_M0_MOVE_THRESHOLD 50
#define SPEED_LOOP_M0_MAX_ACCEL 200
#define SPEED_LOOP_M0_KP 1000
#define SPEED_LOOP_M0_KI 200
#define SPEED_LOOP_M0_KD 50
/* end of Motor 0 */
#define BSP_USING_STEPPER_POS_LOOP
#define POS_LOOP_MOTOR_COUNT 1
#define POS_LOOP_CTRL_PERIOD_US 20000

/* ---- Motor 0 ---- */

/* Motor0-Pos */

#define POS_LOOP_M0_TIM_INDEX 2
#define POS_LOOP_M0_TIM_CH 1
#define POS_LOOP_M0_TIM_CLK_HZ 200000000
#define POS_LOOP_M0_PULSE_PORT_INDEX 0
#define POS_LOOP_M0_PULSE_PIN 0
#define POS_LOOP_M0_PULSE_AF 1
#define POS_LOOP_M0_DIR_PIN 33
#define POS_LOOP_M0_EN_PIN 32
#define POS_LOOP_M0_ENC_TIM_INDEX 3
#define POS_LOOP_M0_ENC_CH1_PORT_INDEX 2
#define POS_LOOP_M0_ENC_CH1_PIN 6
#define POS_LOOP_M0_ENC_CH1_AF 2
#define POS_LOOP_M0_ENC_CH2_PORT_INDEX 2
#define POS_LOOP_M0_ENC_CH2_PIN 7
#define POS_LOOP_M0_ENC_CH2_AF 2
#define POS_LOOP_M0_CTRL_TIMER "timer4"
#define POS_LOOP_M0_ENC_INVERT 1
#define POS_LOOP_M0_SPEED_LIMIT 50000
#define POS_LOOP_M0_MOVE_THRESHOLD 20
#define POS_LOOP_M0_MAX_ACCEL 200
#define POS_LOOP_M0_RAMP_MAX_SPEED 50000
#define POS_LOOP_M0_KP 5000
#define POS_LOOP_M0_KI 500
#define POS_LOOP_M0_KD 100
/* end of Motor0-Pos */
/* end of On-chip Peripheral Drivers */
/* end of Hardware Drivers Config */

#endif
