/*******************************************************************************
* File Name: capsense_operation.c
*
* Description: This file contains threads, functions, and other resources related
* to CapSense operations.
*
********************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation.
********************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*****************************************​**************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "cycfg_capsense.h"
#include "capsense_operation.h"
#include "mqtt_operation.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
/* CSD EzI2C configuration resources */
#define CSD_COMM_HW                 (SCB3)
#define CSD_COMM_IRQ                (scb_3_interrupt_IRQn)
#define CSD_COMM_PCLK               (PCLK_SCB3_CLOCK)
#define CSD_COMM_CLK_DIV_HW         (CY_SYSCLK_DIV_8_BIT)
#define CSD_COMM_CLK_DIV_NUM        (1U)
#define CSD_COMM_CLK_DIV_VAL        (3U)
#define CSD_COMM_SCL_PORT           (GPIO_PRT6)
#define CSD_COMM_SCL_PIN            (0u)
#define CSD_COMM_SDA_PORT           (GPIO_PRT6)
#define CSD_COMM_SDA_PIN            (1u)
#define CSD_COMM_SCL_HSIOM_SEL      (P6_0_SCB3_I2C_SCL)
#define CSD_COMM_SDA_HSIOM_SEL      (P6_1_SCB3_I2C_SDA)
#define SLAVE_ADDRESS_1             (8)

/* CapSense interrupt priority */
#define CAPSENSE_INTERRUPT_PRIORITY (7u)

/* EZI2C interrupt priority must be higher than CapSense interrupt */
#define EZI2C_INTERRUPT_PRIORITY    (6u)

/* CapSense scan interval */
#define CAPSENSE_SCAN_INTERVAL_MS   (50u)

/* CapSense functions */
static cy_status capsense_init(void);
static void tuner_init(void);
static void process_touch(void);
static void capsense_isr(void);
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr);
static void capsense_timer_callback(TimerHandle_t xTimer);
static void ezi2c_isr(void);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;

/*************** CapSense Scan Thread ***************/
/*
 * Summary: Task that initializes the CapSense block and processes the touch input.
 *
 *  @param[in] arg argument for the thread
 *
 */
void task_capsense(void* param)
{
    cy_status status;
    capsense_command_t capsense_cmd;
    TimerHandle_t scan_timer_handle;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize timer for periodic CapSense scan */
    scan_timer_handle = xTimerCreate ("Scan Timer", CAPSENSE_SCAN_INTERVAL_MS,
                                      pdTRUE, NULL, capsense_timer_callback);

    /* Setup communication between Tuner GUI and PSoC 6 MCU */
    tuner_init();

    /* Initialize CapSense block */
    status = capsense_init();
    if(CY_RET_SUCCESS != status)
    {
        CY_ASSERT(0u);
    }

    /* Start the timer */
    xTimerStart(scan_timer_handle, 0u);

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a CapSense command has been received over queue */
        xQueueReceive(capsense_command_q, &capsense_cmd, portMAX_DELAY);

        /* Check if CapSense is busy with a previous scan */
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            switch(capsense_cmd)
            {
                case CAPSENSE_SCAN:
                {
                    /* Start scan */
                    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
                    break;
                }
                case CAPSENSE_PROCESS:
                {
                    /* Process all widgets */
                    Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
                    process_touch();

                    /* Establishes synchronized operation between the CapSense
                     * middleware and the CapSense Tuner tool.
                     */
                    Cy_CapSense_RunTuner(&cy_capsense_context);
                    break;
                }
                /* Invalid command */
                default:
                {
                    break;
                }
            }
        }
    }
}

/*************** Process Touch ***************/
/*
 * Summary: This function processes the touch input and sends command to MQTT task.
 *
 */
static void process_touch(void)
{
    /* Variables used to store touch information */
    uint32_t button0_status = 0;
    uint32_t button1_status = 0;
    uint16_t slider_pos = 0;
    uint8_t slider_touched = 0;
    cy_stc_capsense_touch_t *slider_touch;

    /* Variables used to store previous touch information */
    static uint32_t button0_status_prev = 0;
    static uint32_t button1_status_prev = 0;
    static uint16_t slider_pos_perv = 0;

    /* Variables used for storing command and data for LED Task */
    mqtt_data_t bowl_cmd_data;
    bool send_bowl_command = false;

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
                        CY_CAPSENSE_BUTTON0_WDGT_ID,
                        CY_CAPSENSE_BUTTON0_SNS0_ID,
                        &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
                        CY_CAPSENSE_BUTTON1_WDGT_ID,
                        CY_CAPSENSE_BUTTON1_SNS0_ID,
                        &cy_capsense_context);

    /* Get slider status */
    slider_touch = Cy_CapSense_GetTouchInfo(
                        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
                        &cy_capsense_context);
    slider_pos = slider_touch->ptrPosition->x;
    slider_touched = slider_touch->numPosition;

    /* Detect new touch on Button0 */
    if((0u != button0_status) && (0u == button0_status_prev))
    {
        bowl_cmd_data.command = BOWL_TURN_ON;
        send_bowl_command = true;
        button0_status_prev = button0_status;
        button1_status_prev = 0;
    	configPRINTF(("Touch detected on BTN0\r\n"));
    }

    /* Detect new touch on Button1 */
    if((0u != button1_status) && (0u == button1_status_prev))
    {
        bowl_cmd_data.command = BOWL_TURN_OFF;
        send_bowl_command = true;
        button1_status_prev = button1_status;
        button0_status_prev = 0;
    	configPRINTF(("Touch detected on BTN1\r\n"));
    }

    /* Detect new touch on slider */
    if((0u != slider_touched) && (slider_pos_perv != slider_pos ))
    {
    	bowl_cmd_data.command = BOWL_UPDATE_SPEED;

        /* brightness in percentage */
        bowl_cmd_data.speed = (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        send_bowl_command = true;
        slider_pos_perv = slider_pos;
        configPRINTF(("Touch detected on Slider at position (range: 0 to 100): %u\r\n", bowl_cmd_data.speed));
    }

    /* Send command to update LED state */
    if(send_bowl_command)
    {
        xQueueOverwrite(bowl_command_data_q, &bowl_cmd_data);
    }
}

/*************** Initialize CapSense ***************/
/*
 * Summary: This function initializes the CSD HW block, and configures the CapSense
 *  interrupt.
 *
 */
static cy_status capsense_init(void)
{
    cy_status status;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTERRUPT_PRIORITY,
    };

    /*Initialize CapSense Data structures */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if(CY_RET_SUCCESS == status)
    {
        /* Initialize CapSense interrupt */
        Cy_SysInt_Init(&capSense_intr_config, &capsense_isr);
        NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
        NVIC_EnableIRQ(capSense_intr_config.intrSrc);

        /* Register end of scan callback */
        status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
                capsense_end_of_scan_callback, &cy_capsense_context);

        if(CY_RET_SUCCESS == status)
        {
            /* Initialize the CapSense firmware modules. */
            status = Cy_CapSense_Enable(&cy_capsense_context);
        }
    }

    return status;
}

/*************** CapSense End of Scan Callback ***************/
/*
 * Summary: CapSense end of scan callback function. This function sends a command to
 *  CapSense task to process scan.
 *
 *  @param[in] active_scan_sns_ptr
 *
 */
static void capsense_end_of_scan_callback(cy_stc_active_scan_sns_t* active_scan_sns_ptr)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    (void)active_scan_sns_ptr;

    /* Send command to process CapSense data */
    capsense_command_t commmand = CAPSENSE_PROCESS;
    xQueueOverwriteFromISR(capsense_command_q, &commmand, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*************** Timer Callback ***************/
/*
 * Summary: CapSense timer callback. This function sends a command to start CapSense
 * scan.
 *
 *  @param[in] xTimer timer handle
 *
 */
static void capsense_timer_callback(TimerHandle_t xTimer)
{
    capsense_command_t command = CAPSENSE_SCAN;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    (void)xTimer;

    /* Send command to start CapSense scan */
    xQueueOverwriteFromISR(capsense_command_q, &command, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*************** CapSense ISR ***************/
/*
 * Summary: Wrapper function for handling interrupts from CSD block.
 *
 */
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*************** EzI2C ISR ***************/
/*
 * Summary: Wrapper function for handling interrupts from EZI2C block.
 *
 */
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CSD_COMM_HW, &ezi2c_context);
}

/*************** Initialize tuner ***************/
/*
 * Summary: Initializes communication between Tuner GUI and PSoC 6 MCU.
 *
 */
static void tuner_init(void)
{
    /* EZI2C configuration structure */
    const cy_stc_scb_ezi2c_config_t csd_comm_config =
    {
        .numberOfAddresses = CY_SCB_EZI2C_ONE_ADDRESS,
        .slaveAddress1 = SLAVE_ADDRESS_1,
        .slaveAddress2 = 0U,
        .subAddressSize = CY_SCB_EZI2C_SUB_ADDR16_BITS,
        .enableWakeFromSleep = false,
    };

    /* EZI2C interrupt configuration structure */
    static const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CSD_COMM_IRQ,
        .intrPriority = EZI2C_INTERRUPT_PRIORITY,
    };

    /* Initialize EZI2C pins */
    Cy_GPIO_Pin_FastInit(CSD_COMM_SCL_PORT, CSD_COMM_SCL_PIN,
                         CY_GPIO_DM_OD_DRIVESLOW, 1, CSD_COMM_SCL_HSIOM_SEL);
    Cy_GPIO_Pin_FastInit(CSD_COMM_SDA_PORT, CSD_COMM_SDA_PIN,
                         CY_GPIO_DM_OD_DRIVESLOW, 1, CSD_COMM_SDA_HSIOM_SEL);

    /* Configure EZI2C clock */
    Cy_SysClk_PeriphDisableDivider(CSD_COMM_CLK_DIV_HW, CSD_COMM_CLK_DIV_NUM);
    Cy_SysClk_PeriphAssignDivider(CSD_COMM_PCLK, CSD_COMM_CLK_DIV_HW,
                                  CSD_COMM_CLK_DIV_NUM);
    Cy_SysClk_PeriphSetDivider(CSD_COMM_CLK_DIV_HW, CSD_COMM_CLK_DIV_NUM,
                                   CSD_COMM_CLK_DIV_VAL);
    Cy_SysClk_PeriphEnableDivider(CSD_COMM_CLK_DIV_HW, CSD_COMM_CLK_DIV_NUM);


    /* Initialize EZI2C */
    Cy_SCB_EZI2C_Init(CSD_COMM_HW, &csd_comm_config, &ezi2c_context);

    /* Initialize and enable EZI2C interrupts */
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set up communication data buffer to CapSense data structure to be exposed
     * to I2C master at primary slave address request.
     */
    Cy_SCB_EZI2C_SetBuffer1(CSD_COMM_HW, (uint8 *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &ezi2c_context);

    /* Enable EZI2C block */
    Cy_SCB_EZI2C_Enable(CSD_COMM_HW);
}
