/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
* Copyright 1989-2008 ARC International
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*
*
*
*END************************************************************************/

#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <mutex.h>
#include <sem.h>
#include <event.h>
#include <log.h>
#if MQX_KERNEL_LOGGING
#include <klog.h>
#endif
#include "demo.h"

#if MQX_COMMON_CONFIG != MQX_LITE_CONFIG
    #error "MQX_COMMON_CONFIG must be set to MQX_LITE_CONFIG in mqx_sdk_config.h for this demo."
#endif

#if ! MQX_HAS_TIME_SLICE
#error This application requires MQX_HAS_TIME_SLICE defined non-zero in mqx_sdk_config.h. Please recompile mqx lib with this option.
#endif

#if !MQX_HAS_DYNAMIC_PRIORITIES
#error This apllication requires MQX_HAS_DYNAMIC_PRIORITIES defined non-zero in mqx_sdk_config.h Please recompile mqx lib with this option.
#endif

#if MQXCFG_STATIC_LWLOG
#error This apllication requires MQXCFG_STATIC_LWLOG defined zero in mqx_sdk_config.h Please recompile mqx lib with this option.
#endif

/* User stack size */
uint8_t  Main_task_task_stack[MAIN_TASK_STACK_SIZE];
uint8_t  MutexA_task_stack[MUTEXA_STACK_SIZE];
uint8_t  MutexB_task_stack[MUTEXB_STACK_SIZE];
uint8_t  SemA_task_stack[SEMA_STACK_SIZE];
uint8_t  SemB_task_stack[SEMB_STACK_SIZE];
uint8_t  EventA_task_stack[EVENTA_STACK_SIZE];
uint8_t  EventB_task_stack[EVENTB_STACK_SIZE];
uint8_t  ATimeSliceTask_task_stack[ATIMESLICETASK_STACK_SIZE];
uint8_t  BTimeSliceTask_task_stack[BTIMESLICETASK_STACK_SIZE];
uint8_t  Sender_task_stack[SENDER_STACK_SIZE];

/* Global task init structures */
const taskinit_t task_Main_task = {
    .stacksize  = MAIN_TASK_STACK_SIZE,
    .stackaddr  = Main_task_task_stack,
    .exec       = main_task,
    .priority   = 11,
    .name       = "main_task",
};

const taskinit_t task_MutexA_task = {
    .stacksize  = MUTEXA_STACK_SIZE,
    .stackaddr  = MutexA_task_stack,
    .exec       = MutexA,
    .priority   = 9,
    .name       = "MutexA",
};

const taskinit_t task_MutexB_task = {
    .stacksize  = MUTEXB_STACK_SIZE,
    .stackaddr  = MutexB_task_stack,
    .exec       = MutexB,
    .priority   = 9,
    .name       = "MutexB",
};

const taskinit_t task_SemA_task = {
    .stacksize  = SEMA_STACK_SIZE,
    .stackaddr  = SemA_task_stack,
    .exec       = SemA,
    .priority   = 9,
    .name       = "SemA",
};

const taskinit_t task_SemB_task = {
    .stacksize  = SEMB_STACK_SIZE,
    .stackaddr  = SemB_task_stack,
    .exec       = SemB,
    .priority   = 10,
    .name       = "SemB",
};

const taskinit_t task_EventA_task = {
    .stacksize  = EVENTA_STACK_SIZE,
    .stackaddr  = EventA_task_stack,
    .exec       = EventA,
    .priority   = 9,
    .name       = "EventA",
};

const taskinit_t task_EventB_task = {
    .stacksize  = EVENTB_STACK_SIZE,
    .stackaddr  = EventB_task_stack,
    .exec       = EventB,
    .priority   = 9,
    .name       = "EventB",
};

const taskinit_t task_ATimeSliceTask_task = {
    .stacksize  = ATIMESLICETASK_STACK_SIZE,
    .stackaddr  = ATimeSliceTask_task_stack,
    .exec       = ATimeSliceTask,
    .priority   = 9,
    .name       = "ATimeSliceTask",
    .attributes = MQX_TIME_SLICE_TASK,
    .time_slice = 0,
};

const taskinit_t task_BTimeSliceTask_task = {
    .stacksize  = BTIMESLICETASK_STACK_SIZE,
    .stackaddr  = BTimeSliceTask_task_stack,
    .exec       = BTimeSliceTask,
    .priority   = 9,
    .name       = "BTimeSliceTask",
};

const taskinit_t task_Sender_task = {
    .stacksize  = SENDER_STACK_SIZE,
    .stackaddr  = Sender_task_stack,
    .exec       = Sender,
    .priority   = 10,
    .name       = "Sender",
};

/*FUNCTION-----------------------------------------------------------
*
* Function Name   :  Main
* Comments    : create main_task, BTimeSliceTask, and ATimeSliceTask
*
*END*--------------------------------------------------------------*/
int main(void)
{
    _task_id   created_task;
extern MQX_INITIALIZATION_STRUCT MQX_init_struct;
#if defined(__ICCARM__) /* IAR */
/* Search for last block placed by IAR linker to set heap as big as possible.
** See mqx_main.c for details.*/
    if (NULL == MQX_init_struct.START_OF_HEAP)
    {
        MQX_init_struct.START_OF_HEAP = (__RW_END > __ZI_END) ? __RW_END : __ZI_END;
    }
#endif  /* defined(__ICCARM__) */
    mqx_init((MQX_INITIALIZATION_STRUCT_PTR) &MQX_init_struct);
    created_task = create_task(&task_Main_task);
    if (created_task == MQX_NULL_TASK_ID)
       return -1;
    create_task(&task_BTimeSliceTask_task);
    if (created_task == MQX_NULL_TASK_ID)
       return -1;
    create_task(&task_ATimeSliceTask_task);
    if (created_task == MQX_NULL_TASK_ID)
       return -1;
    mqx_start();
}

/*   Task Code -  Main     */

/*TASK------------------------------------------------------------------
*
* Task Name   :  main_task
* Comments    :  create event component, semaphore component, log
*                component, mutex component, message queue and message
*                pool, and other tasks.
*
*END*------------------------------------------------------------------*/

void main_task
   (
      uint32_t   parameter
   )
{
   MESSAGE_HEADER_STRUCT_PTR msg_ptr;
   _task_id                  created_task;
   _mqx_uint                 log_result;
   _mqx_uint                 event_result;
   _mqx_uint                 sem_result;

_int_install_unexpected_isr();
   PRINTF("\nMQX %s\n",_mqx_version);
   PRINTF("Hello from main_task().\n");

   /* create the log component */
   log_result = _log_create_component();
   if (log_result != MQX_OK) {
      /* log component could not be created */
   } /* endif */
   /* Create the mutex component */
   if (_mutex_create_component() != MQX_OK) {
      /* an error has been detected */
   }

   /* create the event component */
   event_result = _event_create_component(EVENT_INITIAL_NUMBER, EVENT_GROWTH,
      EVENT_MAXIMUM);
   if (event_result != MQX_OK) {
      /* event component could not be created */
      PRINTF("Error: Cannot create event component\n");
      _task_block();
   } /* endif */

   /* create the semaphore component */
   sem_result = _sem_create_component(SEM_INITIAL_NUMBER, SEM_GROWTH,
      SEM_MAXIMUM);
   if (sem_result != MQX_OK) {
      /* semaphore component could not be created */
      PRINTF("Error: Cannot create semaphore component\n");
      _task_block();
   } /* endif */
   MsgPool_pool_id = _msgpool_create ( 8, 10, 0, 0);
   if (MsgPool_pool_id == MSGPOOL_NULL_POOL_ID) {
      /* _msgpool_create did not succeed */
      PRINTF("Error: Cannot create message pool\n");
      _task_block();
   }
   Main_Queue_qid = _msgq_open( MSGQ_FREE_QUEUE, SIZE_UNLIMITED);
   if (Main_Queue_qid == (_queue_id)0){
         /* queue could not be opened */
      PRINTF("Error: Cannot open message pool\n");
      _task_block();
   }
   created_task = create_task(&task_Sender_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }
   created_task = create_task(&task_MutexA_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }
   created_task = create_task(&task_MutexB_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }
   created_task = create_task(&task_SemA_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }
   created_task = create_task(&task_SemB_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }
   created_task = create_task(&task_EventA_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }
   created_task = create_task(&task_EventB_task);
   if (created_task == MQX_NULL_TASK_ID) {
      /* task creation failed */
      PRINTF("Error: Cannot create task\n");
      _task_block();
   }

#if MQX_KERNEL_LOGGING
   /* create log number 0 */
   log_result = _klog_create(200, 0);
   if (log_result != MQX_OK) {
      /* log 0 could not be created */
      PRINTF("Error: Cannot create kernel log\n");
      _task_block();
   } /* endif */

   /* define kernel logging */
   _klog_control(0xFFFFFFFF, FALSE);
   _klog_control(
      KLOG_ENABLED                  |
      KLOG_FUNCTIONS_ENABLED        |
      KLOG_INTERRUPTS_ENABLED       |
      KLOG_SYSTEM_CLOCK_INT_ENABLED |
      KLOG_CONTEXT_ENABLED          |
      KLOG_TASKING_FUNCTIONS        |
      KLOG_ERROR_FUNCTIONS          |
      KLOG_MESSAGE_FUNCTIONS        |
      KLOG_INTERRUPT_FUNCTIONS      |
      KLOG_MEMORY_FUNCTIONS         |
      KLOG_TIME_FUNCTIONS           |
      KLOG_EVENT_FUNCTIONS          |
      KLOG_NAME_FUNCTIONS           |
      KLOG_MUTEX_FUNCTIONS          |
      KLOG_SEMAPHORE_FUNCTIONS      |
      KLOG_WATCHDOG_FUNCTIONS,
      TRUE
      );
#endif

   /*
   ** LOOP -
   */
   while ( TRUE ) {
      msg_ptr = _msg_alloc((_pool_id) MsgPool_pool_id );
      if (msg_ptr == NULL) {
         /* No available message buffer */
      }
      msg_ptr->SIZE = sizeof(MESSAGE_HEADER_STRUCT);
      msg_ptr->SOURCE_QID = msg_ptr->TARGET_QID;
      msg_ptr->TARGET_QID = Sender_Queue_qid;
      _msgq_send(msg_ptr);

      /*
      * Service the message queue - Main_Queue
      */
      msg_ptr = _msgq_receive_ticks(Main_Queue_qid, NO_TIMEOUT);
      /* process message End_msg */
      _msg_free(msg_ptr);

   } /* endwhile */
} /*end of task*/

/* End of File */
