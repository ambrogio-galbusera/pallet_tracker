/******************************************************************************
* File Name:   main.c
*
*******************************************************************************/

#include <tcp_client.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_device.h"
#include "cy_retarget_io.h"

#include "global.h"
#include "scan_task.h"
#include "tcp_client.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (9999)

#define COMMANDLINE_TASK_STACK_SIZE                    (4096u)
#define COMMANDLINE_TASK_PRIORITY                      (3u)

#define LPCOMP_TASK_STACK_SIZE  			(5 * 1024)
#define LPCOMP_TASK_PRIORITY    			(1)
#define LPCOMP_ULP_SETTLE        	(50u)
#define LPCOMP_OUTPUT_LOW        	(0u)
#define LPCOMP_OUTPUT_HIGH       	(1u)
#define TOGGLE_LED_PERIOD           (500u)
#define LED_ON_2S_BEFORE_HIB        (2000u)

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority ;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);

/* LPComp configuration structure */
const cy_stc_lpcomp_config_t myLPCompConfig =
{
    .outputMode = CY_LPCOMP_OUT_DIRECT,
    .hysteresis = CY_LPCOMP_HYST_DISABLE,
};

/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;
uint32_t lpCompOut;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;

/* Timer object used for blinking the LED */
cyhal_timer_t led_blink_timer;

static void LPComp_SetHibernateMode(cy_en_syspm_hib_wakeup_source_t MyLPComp_WakeUpSrc)
{
    /* Turn on LED for 2 seconds to indicate the hibernate mode. */
	cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
    Cy_SysLib_Delay(LED_ON_2S_BEFORE_HIB);
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

    /* Set the wake-up signal from Hibernate */
    Cy_SysPm_SetHibWakeupSource(MyLPComp_WakeUpSrc);

    /* Jump into Hibernate */
    Cy_SysPm_Hibernate();
}

TaskHandle_t commandline_task_handle;
void commandline_task(void* arg)
{
    for (;;)
    {
        /* Check if 'Enter' key was pressed */
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1)
             == CY_RSLT_SUCCESS)
        {
            if (uart_read_value == '\r')
            {
                /* Pause LED blinking by stopping the timer */
                if (led_blink_active_flag)
                {
                    cyhal_timer_stop(&led_blink_timer);

                    printf("LED blinking paused \r\n");
                }
                else /* Resume LED blinking by starting the timer */
                {
                    cyhal_timer_start(&led_blink_timer);

                    printf("LED blinking resumed\r\n");
                }

                /* Move cursor to previous line */
                printf("\x1b[1F");

                led_blink_active_flag ^= 1;
            }
        }

        /* Check if timer elapsed (interrupt fired) and toggle the LED */
        if (timer_interrupt_flag)
        {
            /* Clear the flag */
            timer_interrupt_flag = false;

            /* Invert the USER LED state */
            cyhal_gpio_toggle(CYBSP_USER_LED);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void hibernate_task(void* arg)
{
	xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

	cy_wcm_deinit();
	__NVIC_SystemReset();

	vTaskDelete(NULL);
}

/*******************************************************************************
* Function Name: main
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Check the IO status. If current status is frozen, unfreeze the system. */
    if(Cy_SysPm_GetIoFreezeStatus())
    {   /* Unfreeze the system */
        Cy_SysPm_IoUnfreeze();
    }
    else
    {
        /* Do nothing */
    }

    /* Enable the whole LPComp block */
    Cy_LPComp_GlobalEnable(LPCOMP);

    /* Configure LPComp output mode and hysteresis for channel 0 */
    Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_0, &myLPCompConfig);

    /* Enable the local reference voltage */
    Cy_LPComp_UlpReferenceEnable(LPCOMP);
    /* Set the local reference voltage to the negative terminal and set a GPIO input on the
       positive terminal for the wake up signal */
    Cy_LPComp_SetInputs(LPCOMP, CY_LPCOMP_CHANNEL_0, CY_LPCOMP_SW_GPIO, CY_LPCOMP_SW_LOCAL_VREF);

    /* Enable local reference for LPComp inputN */
     Cy_LPComp_ConnectULPReference(LPCOMP, CY_LPCOMP_CHANNEL_0);
     Cy_LPComp_UlpReferenceEnable(LPCOMP);

    /* Set channel 0 power mode - Ultra Low Power mode */
    Cy_LPComp_SetPower(LPCOMP, CY_LPCOMP_CHANNEL_0, CY_LPCOMP_MODE_ULP);

    /* It needs 50us start-up time to settle in ULP mode after the block is enabled */
    Cy_SysLib_DelayUs(LPCOMP_ULP_SETTLE);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "Pallet Tracker "
           "****************** \r\n\n");

    if (Cy_SysLib_GetResetReason() == CY_SYSLIB_RESET_HIB_WAKEUP)
    //if (1)
    {
		/* Initialize timer to toggle the LED */
		timer_init();

		xTaskCreate(commandline_task, "CommandLine task", COMMANDLINE_TASK_STACK_SIZE, NULL, COMMANDLINE_TASK_PRIORITY, &commandline_task_handle);
		xTaskCreate(hibernate_task, "Hibernate task", LPCOMP_TASK_STACK_SIZE, NULL, LPCOMP_TASK_PRIORITY, &global_data.hibernate_task_handle) ;
		xTaskCreate(tcp_secure_client_task, "Network task", TCP_SECURE_CLIENT_TASK_STACK_SIZE, &global_data, TCP_SECURE_CLIENT_TASK_PRIORITY, &global_data.tcp_client_task_handle) ;
		xTaskCreate(scan_task, "Scan task", SCAN_TASK_STACK_SIZE, &global_data, SCAN_TASK_PRIORITY, &scan_task_handle);

		/* Start the FreeRTOS scheduler. */
		vTaskStartScheduler();
    }
    else
    {
		/* store LPComp status */
		lpCompOut = Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_0);

    	if (lpCompOut == LPCOMP_OUTPUT_HIGH)
		{
			/* System wakes up when LPComp channel 0 output is low */
			LPComp_SetHibernateMode(CY_SYSPM_LPCOMP0_LOW);
		}
		else
		{
			/* System wakes up when LPComp channel 0 output is high */
			LPComp_SetHibernateMode(CY_SYSPM_LPCOMP0_HIGH);
		}
    }

#if 0
	/* If the comparison result is high, toggles LED every 500ms */
	if(Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_0) == LPCOMP_OUTPUT_HIGH)
	{
		/* InputP "floating" -> pallet on the floor
		/* Toggle LED every 500ms */
		/*
		cyhal_gpio_toggle(CYBSP_USER_LED);
		Cy_SysLib_Delay(TOGGLE_LED_PERIOD);
		*/
		for (int i=0; i<10; i++)
		{
			cyhal_gpio_toggle(CYBSP_USER_LED);
			Cy_SysLib_Delay(TOGGLE_LED_PERIOD);
		}
		MyLPComp_SetHibernateMode(CY_SYSPM_LPCOMP0_LOW);
	}
	/* If the comparison result is low, goes to the hibernate mode */
	else
	{
		/* InputP "grounded" -> pallet lifted up
		/* System wakes up when LPComp channel 0 output is high */
		MyLPComp_SetHibernateMode(CY_SYSPM_LPCOMP0_HIGH);
	}

    /* Initialize timer to toggle the LED */
    timer_init();
 
    printf("Press 'Enter' key to pause or "
           "resume blinking the user LED \r\n\r\n");

    /* Create the tasks. */
    //xTaskCreate(scan_task, "Scan task", SCAN_TASK_STACK_SIZE, NULL, SCAN_TASK_PRIORITY, &scan_task_handle);
    xTaskCreate(commandline_task, "CommandLine task", COMMANDLINE_TASK_STACK_SIZE, NULL, COMMANDLINE_TASK_PRIORITY, &commandline_task_handle);
    xTaskCreate(tcp_secure_client_task, "Network task", TCP_SECURE_CLIENT_TASK_STACK_SIZE, NULL, TCP_SECURE_CLIENT_TASK_PRIORITY, NULL) ;
    xTaskCreate(lpcomp_task, "LPCOMP task", LPCOMP_TASK_STACK_SIZE, NULL, LPCOMP_TASK_PRIORITY, &lpcomp_task_handle) ;
    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();
#endif
}


/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks 
* continuously and produces a periodic interrupt on every terminal count 
* event. The period is defined by the 'period' and 'compare_value' of the 
* timer configuration structure 'led_blink_timer_cfg'. Without any changes, 
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
 void timer_init(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg = 
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction, 
       duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&led_blink_timer);
 }


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
}

/* [] END OF FILE */
