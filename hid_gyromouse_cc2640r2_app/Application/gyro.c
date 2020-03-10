/*
 * gyro.c
 *      Author: henorvell
 */

#include "gyro.h"

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include <icall.h>
#include <stdint.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>

static const uint8_t EXPECTED_WHOAMI[] = { 0x95, 0x98, 0x02, 0x03 };

/*
 * Just a handy variable to handle the icm207xx object
 */
static inv_icm207xx_t icm_device;

/*
 * Variable to keep track of the expected period common for all sensors
 */
//static uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

/*
 * Variable to drop the first timestamp(s) after a sensor start catched by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
static uint8_t chip_info[3];

static Task_Struct gyroTask;
static Char gyroTaskStackSize[644];

static void Gyro_init(void);
static void Gyro_taskFxn(UArg a0, UArg a1);
//static uint8_t Gyro_enqueueMsg(uint8_t x, uint8_t y);

void Gyro_createTask(void)
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = gyroTaskStackSize;
    taskParams.stackSize = 644;
    taskParams.priority = 1;
    Task_construct(&gyroTask, Gyro_taskFxn, &taskParams, NULL);
}

void Gyro_init(void)
{
    SPI_init();
    //ICall_registerApp(&gyroEntity, &gyroEvent);
    //appMsgQueue = Util_constructQueue(&appMsg);
}

void Gyro_taskFxn(UArg a0, UArg a1)
{
    // Initialize the application.
    Gyro_init();
    // Application main loop.
    for (;;)
    {
        //Semaphore_pend(semHandle, BIOS_WAIT_FORERVER);
    }
}
