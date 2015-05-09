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
* Comments: This file contains global definitions for task stack size,
*           semaphore, event, message queues.
*
*
*
*END************************************************************************/

#include <task.h>
#include <fsl_debug_console.h>

/*   Defines and data structures for application       */

#define NO_TIMEOUT        0
#define SIZE_UNLIMITED    0

/* Defines for Semaphore Component */
#define SEM_INITIAL_NUMBER    10
#define SEM_GROWTH            10
#define SEM_MAXIMUM           20

/* Defines for Event Component */
#define EVENT_INITIAL_NUMBER  10
#define EVENT_GROWTH          10
#define EVENT_MAXIMUM         20

#if PSP_MQX_CPU_IS_MCF51
   #define MAIN_STACK 900
   #define STACK_SIZE 590
   #define SEMA_STACK 380
#else
   #define STACK_SIZE 1024
   #define MAIN_STACK STACK_SIZE
   #define SEMA_STACK STACK_SIZE
#endif

#define MAIN_TASK_STACK_SIZE             MQX_REQUIRED_STACK_SIZE(MAIN_STACK)
#define MUTEXA_STACK_SIZE                MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define MUTEXB_STACK_SIZE                MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define SEMB_STACK_SIZE                  MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define SEMA_STACK_SIZE                  MQX_REQUIRED_STACK_SIZE(SEMA_STACK)
#define EVENTA_STACK_SIZE                MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define EVENTB_STACK_SIZE                MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define ATIMESLICETASK_STACK_SIZE        MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define BTIMESLICETASK_STACK_SIZE        MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define SENDER_STACK_SIZE                MQX_REQUIRED_STACK_SIZE(STACK_SIZE)
#define RESPONDER_STACK_SIZE             MQX_REQUIRED_STACK_SIZE(STACK_SIZE)

/*
**   Externs for global data
*/
/*  Message Queue Ids */
extern   _queue_id     Sender_Queue_qid;

/*  Message Queue Ids */
extern   _queue_id     Responder_Queue_qid;

/*  Message Queue Ids */
extern   _queue_id     Main_Queue_qid;

/*  Message Pool Ids */
extern   _pool_id      MsgPool_pool_id;

/*  Mutex Definitions */
extern   MUTEX_STRUCT  Mutex1;

/* Externs for global task init structures */
extern const taskinit_t task_Main_task;
extern const taskinit_t task_MutexA_task;
extern const taskinit_t task_MutexB_task;
extern const taskinit_t task_SemA_task;
extern const taskinit_t task_SemB_task;
extern const taskinit_t task_EventA_task;
extern const taskinit_t task_EventB_task;
extern const taskinit_t task_BTimeSliceTask_task;
extern const taskinit_t task_ATimeSliceTask_task;
extern const taskinit_t task_Sender_task;
extern const taskinit_t task_Responder_task;

/*
** Externs for Tasks and ISRs
*/
extern void MutexB(uint32_t);
extern void MutexA(uint32_t);
extern void SemB(uint32_t);
extern void SemA(uint32_t);
extern void EventB(uint32_t);
extern void EventA(uint32_t);
extern void BTimeSliceTask(uint32_t);
extern void ATimeSliceTask(uint32_t);
extern void Sender(uint32_t);
extern void Responder(uint32_t);
extern void main_task(uint32_t);

/* EOF */
