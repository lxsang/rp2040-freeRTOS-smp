/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
//#include "semphr.h"   /* Semaphore related API prototypes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
//#include "pico/sem.h"

/* WS specific header */
#include "DEV_Config.h"
#include "GUI_Paint.h"
#include "Debug.h"
#include "LCD_1in28.h"
#include "QMI8658.h"


/* Priorities at which the tasks are created */
#define DRAW_TASK_PRIORITY   ( configMAX_PRIORITIES - 1 )
#define DATA_TASK_PRIORITY   ( configMAX_PRIORITIES - 2 )
#define TIMER_TASK_PRIORITY   ( configMAX_PRIORITIES - 3 )

/* The rate at which task is called, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define TASK_PERIOD            pdMS_TO_TICKS( 100 )
#define TIMER_PERIOD           pdMS_TO_TICKS( 1000 )

/* The number of items the queue can hold.  This is 1 as the receive task (DRAW)
has a higher priority than the send task (DATA), so will remove items as they are added,
meaning the send task should always find the queue empty. */
#define DATA_QUEUE_LENGTH                    ( 1 )

#define VOLTAGE_FACTOR (3.3f / (1 << 12) * 2)

#define FB_SIZE (LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2)


typedef struct {
    float acc[3];
    float gyro[3];
    uint16_t adc;
} queue_data_t;

/*-----------------------------------------------------------*/

/*
 * Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void init_dev( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void draw_task( void *params );
static void data_task( void *params );

/*
 * The callback function assigned to the  software timer
 */
static void timer_callback( TimerHandle_t timer );

/*-----------------------------------------------------------*/

/* The queue used by the queue send and queue receive tasks. */
static QueueHandle_t g_queue = NULL;

/* The counters used by the various examples.  The usage is described in the
 * comments at the top of this file.
 */
static volatile uint32_t g_time_cb_count = 0;
static volatile uint32_t g_draw_count = 0;
static volatile uint32_t g_data_count = 0;
static volatile uint32_t g_mutex_enter_count = 0;

static semaphore_t g_sem;
/*-----------------------------------------------------------*/



#if configNUM_CORES > 1
#error Require one core configured for FreeRTOS
#endif

auto_init_mutex(g_mutex);

static void non_rtos_worker() {
    printf("Core %d: Doing Display rendering\n", get_core_num());
    UWORD * frame_buffer;
    if ((frame_buffer = (UWORD *)malloc(FB_SIZE)) == NULL)
    {
        printf("Error: Unable to create framebuffer memory \n");
        return;
    }
    printf("Created image buffer of size: %d\n", FB_SIZE);
    queue_data_t data;
    Paint_NewImage((UBYTE *)frame_buffer, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);
    for( ;; )
    {
        /** Wait for semafore permit*/
        sem_acquire_blocking(&g_sem);

        mutex_enter_blocking(&g_mutex);
        BaseType_t ret = xQueueReceive( g_queue, &data, 0 );
        mutex_exit(&g_mutex);

        if(ret == pdTRUE)
        {
            /*  To get here something must have been received from the queue, but
            is it the expected value?  If it is, increment the counter. */
            /* Count the number of items that have been received correctly. */
            mutex_enter_blocking(&g_mutex);
            g_draw_count++;
            mutex_exit(&g_mutex);
            Paint_Clear(WHITE);
            Paint_DrawString_EN(30, 50, "ACC_X = ", &Font16, BLACK, WHITE);
            Paint_DrawString_EN(30, 75, "ACC_Y = ", &Font16, BLACK, WHITE);
            Paint_DrawString_EN(30, 100, "ACC_Z = ", &Font16, BLACK, WHITE);
            Paint_DrawString_EN(30, 125, "GYR_X = ", &Font16, BLACK, WHITE);
            Paint_DrawString_EN(30, 150, "GYR_Y = ", &Font16, BLACK, WHITE);
            Paint_DrawString_EN(30, 175, "GYR_Z = ", &Font16, BLACK, WHITE);
            Paint_DrawString_EN(30, 175, "GYR_Z = ", &Font16, BLACK, WHITE);
            Paint_DrawNum(120, 50, data.acc[0], &Font16, 3, BLACK, WHITE);
            Paint_DrawNum(120, 75, data.acc[1], &Font16, 3, BLACK, WHITE);
            Paint_DrawNum(120, 100, data.acc[2], &Font16, 3, BLACK, WHITE);
            Paint_DrawNum(120, 50, data.acc[0], &Font16, 2, BLACK, WHITE);
            Paint_DrawNum(120, 75, data.acc[1], &Font16, 2, BLACK, WHITE);
            Paint_DrawNum(120, 100, data.acc[2], &Font16, 2, BLACK, WHITE);
            Paint_DrawNum(120, 125, data.gyro[0], &Font16, 2, BLACK, WHITE);
            Paint_DrawNum(120, 150, data.gyro[1], &Font16, 2, BLACK, WHITE);
            Paint_DrawNum(120, 175, data.gyro[2], &Font16, 2, BLACK, WHITE);
            Paint_DrawString_EN(50, 200, "BAT(V)=", &Font16, WHITE, BLACK);
            Paint_DrawNum(130, 200, data.adc * VOLTAGE_FACTOR, &Font16, 2, BLACK, WHITE);
            LCD_1IN28_Display(frame_buffer);
            printf("Core %d: Draw\n", get_core_num());
        }
        
    }
    free(frame_buffer);
}

int main(void) {

    TimerHandle_t timer = NULL;

    /* Configure the system.  The clock configuration
    can be done here if it was not done before main() was called. */
    init_dev();
    sem_init(&g_sem, 0, 1);
    /* Create the queue used by the data and draw tasks. */
    g_queue = xQueueCreate(
            /* The number of items the queue can hold. */
            DATA_QUEUE_LENGTH,
            /* The size of each item the queue holds. */
            sizeof(queue_data_t));


    /* Create the data task */
    xTaskCreate(data_task,
                "Data",
                configMINIMAL_STACK_SIZE,
                NULL,
                DATA_TASK_PRIORITY,
                NULL);

    /* Create the software timer as described in the comments at the top of
    this file. */
    timer = xTimerCreate(
            /* A text name, purely to help debugging. */
            (const char *) "LOGTimer",
            /* The timer period, in this case 1000ms (1s). */
            TIMER_PERIOD,
            /* This is a periodic timer, so xAutoReload is set to pdTRUE. */
            pdTRUE,
            /* The ID is not used, so can be set to anything. */
            (void *) 0,
            /* The callback function that switches the LED off. */
            timer_callback
    );

    /* Start the created timer.  A block time of zero is used as the timer
    command queue cannot possibly be full here (this is the first timer to
    be created, and it is not yet running). */
    xTimerStart(timer, 0);
    multicore_launch_core1(non_rtos_worker);

    printf("Core %d: Launching FreeRTOS scheduler\n", get_core_num());
    /* Start the scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
    /* should never reach here */
    DEV_Module_Exit();
    panic_unsupported();
}
/*-----------------------------------------------------------*/

static void timer_callback( TimerHandle_t timer )
{
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
    g_time_cb_count++;
    printf("TIME RUN ON CORE %d\n", get_core_num());
    printf("Time call back %d\n", g_time_cb_count);
    printf("Data read called %d\n", g_data_count);
    printf("Draw called %d\n", g_draw_count);
}
/*-----------------------------------------------------------*/

static void data_task( void *pvParameters )
{
    TickType_t next_wake_time;
    queue_data_t data;
    unsigned int tim_count;
    /* Initialise next_make_time - this only needs to be done once. */
    next_wake_time = xTaskGetTickCount();

    for( ;; )
    {
        /* Place this task in the blocked state until it is time to run again.
        The block time is specified in ticks, the constant used converts ticks
        to ms.  The task will not consume any CPU time while it is in the
        Blocked state. */
        vTaskDelayUntil( &next_wake_time, TASK_PERIOD);
        QMI8658_read_xyz(data.acc, data.gyro, &tim_count);
        data.adc = adc_read();
        mutex_enter_blocking(&g_mutex);
        g_data_count++;
        xQueueSend( g_queue, &data, 0 );
        mutex_exit(&g_mutex);
        /* Send to the queue - causing the queue receive task to unblock and
        increment its counter.  0 is used as the block time so the sending
        operation will not block - it shouldn't need to block as the queue
        should always be empty at this point in the code. */
        printf("Core %d - Thread '%s': send data\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()));
        sem_release(&g_sem);
    }
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    panic("malloc failed");
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) xTask;

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeStackSpace;

    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    xFreeStackSpace = xPortGetFreeHeapSize();

    if( xFreeStackSpace > 100 )
    {
        /* By now, the kernel has allocated everything it is going to, so
        if there is a lot of heap remaining unallocated then
        the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
        reduced accordingly. */
    }
}
/*-----------------------------------------------------------*/

static void init_dev( void )
{
    /* Want to be able to printf */
    DEV_Module_Init();
    printf("Init ADC...\n");
    adc_init();
    adc_gpio_init(29);
    adc_select_input(3);
    QMI8658_init();

    LCD_1IN28_Init(HORIZONTAL);
    LCD_1IN28_Clear(WHITE);
    DEV_SET_PWM(60);
    
}