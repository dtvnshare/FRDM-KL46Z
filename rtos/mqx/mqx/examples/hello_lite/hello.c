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
*   This file contains the source for the hello2 example program.
*
*
*END************************************************************************/

#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <task.h>
#include <fsl_debug_console.h>

#define HELLO_STACK_SIZE MQX_REQUIRED_STACK_SIZE(500)
#define WORLD_STACK_SIZE MQX_REQUIRED_STACK_SIZE(500)

extern void hello_task(uint32_t);
extern void world_task(uint32_t);

#if MQX_COMMON_CONFIG != MQX_LITE_CONFIG
    #error "MQX_COMMON_CONFIG must be set to MQX_LITE_CONFIG in mqx_sdk_config.h for this demo."
#endif

/* User stack size */
uint8_t  Hello_task_stack[HELLO_STACK_SIZE];
uint8_t  World_task_stack[WORLD_STACK_SIZE];

/* Global parameter structures */
const taskinit_t task_Hello = {
    .stacksize  = HELLO_STACK_SIZE,
    .stackaddr  = Hello_task_stack,
    .priority   = 8,
    .name       = "hello_task",
    .exec       = hello_task,
};

const taskinit_t task_World = {
    .stacksize  = WORLD_STACK_SIZE,
    .stackaddr  = World_task_stack,
    .priority   = 9,
    .name       = "world_task",
    .exec       = world_task,
};

/*FUNCTION-----------------------------------------------------------
*
* Function Name   :  Main
* Comments    :
*
*END*--------------------------------------------------------------*/
int main(void)
{
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
    create_task(&task_World);
    mqx_start();
}

/*TASK*-----------------------------------------------------
*
* Task Name    : world_task
* Comments     :
*    This task creates hello_task and then prints " World ".
*
*END*-----------------------------------------------------*/

void world_task
   (
      uint32_t initial_data
   )
{
   _task_id   created_task;

   created_task = create_task(&task_Hello);
   if (created_task == MQX_NULL_TASK_ID) {
      PRINTF("Error: SENDER task creation failed\n");
   } else {
      PRINTF(" World \n");
   }

   _task_block();
}


/*TASK*-----------------------------------------------------
*
* Task Name    : hello_task
* Comments     :
*    This task prints " Hello".
*
*END*-----------------------------------------------------*/

void hello_task
   (
      uint32_t initial_data
   )
{

   PRINTF("\n Hello\n");
   _task_block();
}

/* EOF */
