/*
 * gyro.c
 *      Author: henorvell
 */

#include "gyro.h"
#include "Board.h"
#include "SensorTypes.h"
#include "util.h"

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
//#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>

static bool Spi_Read(uint8_t *buf, size_t len);
static void init_spi(uint32_t bitRate);
static void gyro_SPICallback(SPI_Handle handle, SPI_Transaction *transaction);
//static void Gyro_taskFxn(UArg a0, UArg a1);

static UART_Handle huart;
static SPI_Handle spiHandle;
static xSensorEvent gyroevent;
static uint8_t* rxBuf;

//// Entity ID globally used to check for source and/or destination of messages
//static ICall_EntityID selfEntity;
//
//// Event globally used to post local events and pend on system and
//// local events.
//static ICall_SyncHandle syncEvent;

//static Clock_Struct GyroUpdateClock;

//static bool reading = false;

// Queue object used for app messages
//static Queue_Struct appMsg;
//static Queue_Handle appMsgQueue;

static GyroCB_t appGyroChangeHandler = NULL;
//Event_Handle gyro_Event;
//
//#define GYRO_TASK_STACK_SIZE 1000
//
//Task_Struct gyroTask;
//Char gyroTaskStack[GYRO_TASK_STACK_SIZE];

//void Gyro_createTask(void)
//{
//  Task_Params taskParams;
//
//  // Configure task
//  Task_Params_init(&taskParams);
//  taskParams.stack = gyroTaskStack;
//  taskParams.stackSize = GYRO_TASK_STACK_SIZE;
//  taskParams.priority = 1;
//
//  //Task_construct(&gyroTask, Gyro_taskFxn, &taskParams, NULL);
//}

//void Gyro_taskFxn(UArg a0, UArg a1)
//{
//    Gyro_init(NULL);
//    for (;;) {
//        uint32_t events;
//        events = Event_pend(syncEvent, Event_Id_NONE, GYRO_CTL_EVT, ICALL_TIMEOUT_FOREVER);
//        if(events & GYRO_CTL_EVT) {
//            //Util_startClock(&GyroUpdateClock);
//            reading = true;
//        }
////        if(reading)
////            poll_SPI();
//    }
//}

void Gyro_init(GyroCB_t appGyroCB)
{
//    ICall_registerApp(&selfEntity, &syncEvent);
    //appMsgQueue = Util_constructQueue(&appMsg);
    //Util_constructClock(&GyroUpdateClock, GyroUpdateHandler, 100, 0, false, 0);

    rxBuf = ICall_malloc(sizeof(xSensorEvent));
    //txBuf = malloc(sizeof(xSensorEvent));
    if(!rxBuf)
        while (1);
    memset(rxBuf, 0, sizeof(xSensorEvent));
    //memset(txBuf, 0, sizeof(xSensorEvent));
    init_spi(3125000);

    UART_Params uartParams;
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    huart = UART_open(CC2640R2_LAUNCHXL_UART0, &uartParams);

    if (huart == NULL)
        while (1);
    appGyroChangeHandler = appGyroCB;
    //ICall_registerApp(&gyroEntity, &gyroEvent);
    //appMsgQueue = Util_constructQueue(&appMsg);
}

void init_spi(uint32_t bitRate)
{
    SPI_Params      spiParams;
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.dataSize = 8;       // 8-bit data size
    spiParams.bitRate = bitRate;
    spiParams.mode = SPI_SLAVE;
    //spiParams.transferTimeout = 20;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = gyro_SPICallback;
    spiHandle = SPI_open(CC2640R2_LAUNCHXL_SPI0, &spiParams);
    if (spiHandle == NULL) {
        while (1);  // SPI_open() failed
    }
}

//static bool Spi_Write(const uint8_t *buf, size_t len)
//{
//    SPI_Transaction Transaction;
//
//    Transaction.count  = len;
//    Transaction.txBuf  = (void*)buf;
//    Transaction.rxBuf  = rxBuf;
//    Transaction.arg    = NULL;
//
//    return SPI_transfer(spiHandle, &Transaction) ? 1 : 0;
//}

static bool Spi_Read(uint8_t *buf, size_t len)
{
    SPI_Transaction Transaction;

    Transaction.count = len;
    Transaction.rxBuf = buf;
    Transaction.txBuf = NULL;
    Transaction.arg = NULL;

    return SPI_transfer(spiHandle, &Transaction) ? 1 : 0;
}

static void gyro_SPICallback(SPI_Handle handle, SPI_Transaction *transaction)
{
    if (appGyroChangeHandler != NULL)
    {
        // Notify the application
        //gyroevent = *(xSensorEvent*)rxBuf;
        memcpy(&gyroevent, rxBuf, sizeof(xSensorEvent));
        UART_write(huart, &gyroevent, sizeof(xSensorEvent));
        memset(rxBuf, 0, sizeof(xSensorEvent));
        (*appGyroChangeHandler)(gyroevent);
    }
}

void poll_SPI()
{
    //ICall_CSState key = ICall_enterCriticalSection();
    //SPI_transferCancel(spiHandle);
    //RxActive = true;
    if(Spi_Read(rxBuf, sizeof(xSensorEvent))){
        memcpy(&gyroevent, rxBuf, sizeof(xSensorEvent));
        UART_write(huart, &gyroevent, sizeof(xSensorEvent));
        memset(rxBuf, 0, sizeof(xSensorEvent));
    }
//        memcpy(&gyroevent, rxBuf, sizeof(xSensorEvent));
//        memset(rxBuf, 0, sizeof(xSensorEvent));
//        char* out_str = malloc(50);
//        unsigned char idx = 0;
//        idx += snprintf(&out_str[idx], 50 - idx, "%u,", gyroevent.sensor);
//        idx += snprintf(&out_str[idx], 50 - idx, "%d,", gyroevent.status);
//        idx += snprintf(&out_str[idx], 50 - idx, "%f,", gyroevent.data.acc.vect[0]);
//        idx += snprintf(&out_str[idx], 50 - idx, "%f,", gyroevent.data.acc.vect[1]);
//        idx += snprintf(&out_str[idx], 50 - idx, "%f", gyroevent.data.acc.vect[2]);
//        idx += snprintf(&out_str[idx], 50 - idx, "\r\n");
//        UART_write(huart, out_str, idx);
//        free(out_str);
//    }
//    ICall_leaveCriticalSection(key);
    //Semaphore_post(gyro_sem);
}

//void Gyro_start_poll()
//{
//    Event_post(syncEvent, GYRO_CTL_EVT);
//}
