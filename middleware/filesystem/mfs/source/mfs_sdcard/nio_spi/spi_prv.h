#ifndef __spi_prv_h__
#define __spi_prv_h__
/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
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
*   This file contains definitions private to the SPI driver.
*
*
*END************************************************************************/

#include <stdint.h>

#include "spi.h"
#include "lwevent.h"
//! #include "bsp.h"
//! #include "bsp_config.h"


/*--------------------------------------------------------------------------*/
/*
**                    CONSTANT DEFINITIONS
*/

#define IO_SPI_ATTRIBS (IO_DEV_ATTR_READ | IO_DEV_ATTR_REMOVE | IO_DEV_ATTR_SEEK | IO_DEV_ATTR_WRITE | IO_DEV_ATTR_BLOCK_MODE)

#define SPI_PROCTL_EMODE_BIG                0x00
#define SPI_PROCTL_EMODE_LITTLE             0x02

#define SPI_PROCTL_DTW_1BIT                 0b00
#define SPI_PROCTL_DTW_4BIT                 0b01
#define SPI_PROCTL_DTW_8BIT                 0b10

#define SPI_ADMA2_FLAG_VALID                0x01
#define SPI_ADMA2_FLAG_END                  0x02
#define SPI_ADMA2_FLAG_INT                  0x04
#define SPI_ADMA2_FLAG_TRAN                 0x20
#define SPI_ADMA2_FLAG_LINK                 0x30

#define SPI_CMD_TICK_TIMEOUT                20  // 40ms?
#define SPI_CMD12_TICK_TIMEOUT              200 //500ms
#define SPI_TRANSFER_TIMEOUT_MS             750 //750ms

#define SPI_LWEVENT_CMD_DONE                0x00000001
#define SPI_LWEVENT_CMD_ERROR               0x00000002
#define SPI_LWEVENT_CMD_TIMEOUT             0x00000004
#define SPI_LWEVENT_TRANSFER_DONE           0x00000008
#define SPI_LWEVENT_TRANSFER_ERROR          0x00000010
#define SPI_LWEVENT_TRANSFER_TIMEOUT        0x00000020

#define SPI_MAX_BAUDRATE_25MHz              (25000000)
#define SPI_MAX_BAUDRATE_50MHz              (50000000)


#ifndef SPI_IS_HANDLING_CACHE
  #define SPI_IS_HANDLING_CACHE  PSP_HAS_DATA_CACHE
#endif

#ifndef BSP_SPI_INT_LEVEL
  #define BSP_SPI_INT_LEVEL 4 // default value to achieve compatibility in older BSPs
#endif

#ifndef SPI_CARD_DETECTION_SUPPORT
  #define SPI_CARD_DETECTION_SUPPORT       (0)
#endif

#ifndef SPI_AUTO_CLK_GATING
  #define SPI_AUTO_CLK_GATING              (1)
#endif

/*--------------------------------------------------------------------------*/
/*
**                    DATATYPE DECLARATIONS
*/

/*
** SPI_DEVICE_STRUCT
*/

typedef struct spi_adma2_desc
{
    uint32_t LEN_ATTR;
    uint32_t DATA_ADDR;
} SPI_ADMA2_DESC;


typedef struct spi_adma2_data
{
    uint8_t HEAD_BUF[PSP_MEMORY_ALIGNMENT+1];
    uint8_t TAIL_BUF[PSP_MEMORY_ALIGNMENT+1];
    SPI_ADMA2_DESC DESC[3];
} SPI_ADMA2_DATA;


typedef struct spi_device_struct
{
    /* The number of opened file descriptors */
    uint32_t                     COUNT;

    /* The actual card status */
    uint32_t                     CARD;

    /* Interrupt vector of the SPI controller */
    uint32_t                     VECTOR;

    /* SPI registers (base address) */
    uint32_t                     DEV_BASE;
    
    /* The communication board allowed maximal baud rate */
    uint32_t                     MAX_BAUD_RATE;

    /* The pointer to callback and data for IO card interrupt*/
    SPI_IO_INT_CALLBACK_STRUCT         IO_CALLBACK_STR;

#if SPI_CARD_DETECTION_SUPPORT
    /* The pointer to callback and data for card presence change*/
    SPI_CARD_PRESENCE_CALLBACK_STRUCT  CARD_PRESENCE_CALLBACK_STR;
#endif

    /* Buffers and descriptors for ADMA2 are dynamically allocated to be properly aligned */
    SPI_ADMA2_DATA             *ADMA2_DATA;

    /* The buffered command for data operations */
    SPI_COMMAND_STRUCT         BUFFERED_CMD;

    /* Semaphore signalled from ISR when to notify about job done */
    LWSEM_STRUCT                 EVENT_IO_FINISHED;

    /* LightWeight Events to manage the interrupt & DMA style of driver */
    LWEVENT_STRUCT               LWEVENT;
    
    int                          FLAGS;

} SPI_DEVICE_STRUCT, * SPI_DEVICE_STRUCT_PTR;


/*--------------------------------------------------------------------------*/
/*
**                        FUNCTION PROTOTYPES
*/

#ifdef __cplusplus
extern "C" {
#endif

int _spi_install(char *identifier, SPI_INIT_STRUCT_CPTR spi_init_ptr);

#ifdef __cplusplus
}
#endif

#endif
/* EOF */
