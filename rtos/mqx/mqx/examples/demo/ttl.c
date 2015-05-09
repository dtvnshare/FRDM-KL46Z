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

#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <mutex.h>
#include <sem.h>
#include <event.h>
#include <log.h>
#include "demo.h"

#define STACK_SIZE 1024
#define MAIN_STACK STACK_SIZE
#define SEMA_STACK STACK_SIZE  


/*   Task Templates  */ 
const TASK_TEMPLATE_STRUCT  MQX_template_list[] = 
{   
  /* Task Index,    Function,       Stack,       Priority, Name,             Attributes,                                Param,  Time Slice */
   { MUTEXB,        MutexB,         STACK_SIZE,   9,      "MutexB",         0,                                         0,      0 },
   { MUTEXA,        MutexA,         STACK_SIZE,   9,      "MutexA",         0,                                         0,      0 },
   { SEMB,          SemB,           STACK_SIZE,  10,      "SemB",           0,                                         0,      0 },
   { SEMA,          SemA,           SEMA_STACK,   9,      "SemA",           0,                                         0,      0 },
   { EVENTB,        EventB,         STACK_SIZE,   9,      "EventB",         0,                                         0,      0 },
   { EVENTA,        EventA,         STACK_SIZE,   9,      "EventA",         0,                                         0,      0 },
   { BTIMESLICETASK,BTimeSliceTask, STACK_SIZE,   9,      "BTimeSliceTask", MQX_AUTO_START_TASK,                       0,      0 },
   { ATIMESLICETASK,ATimeSliceTask, STACK_SIZE,   9,      "ATimeSliceTask", (MQX_TIME_SLICE_TASK|MQX_AUTO_START_TASK), 0,      0 },
   { SENDER,        Sender,         STACK_SIZE,  10,      "Sender",         0,                                         0,      0 },
   { RESPONDER,     Responder,      STACK_SIZE,   9,      "Responder",      0,                                         0,      0 },
   { MAIN_TASK,     main_task,      MAIN_STACK,  11,      "main_task",      MQX_AUTO_START_TASK,                       0,      0 },
   { 0 }
};

/* End of File */
