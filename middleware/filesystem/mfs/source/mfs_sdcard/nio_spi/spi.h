#ifndef __spi_h__
#define __spi_h__
/*HEADER**********************************************************************
*
* Copyright 2009 Freescale Semiconductor, Inc.
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
*   This file contains the definitions of constants and structures
*   required for the SPI driver
*
*
*END************************************************************************/

#include <stdint.h>


/*--------------------------------------------------------------------------*/
/*
**                    CONSTANT DEFINITIONS
*/

/*
** IOCTL calls specific to SPI
*/
#define IOCTL_BASE_SPI                      0x100
#define IO_IOCTL_SPI_INIT                  (IOCTL_BASE_SPI+0x01)
#define IO_IOCTL_SPI_SEND_COMMAND          (IOCTL_BASE_SPI+0x02)
#define IO_IOCTL_SPI_GET_CARD              (IOCTL_BASE_SPI+0x03)
#define IO_IOCTL_SPI_GET_BAUDRATE          (IOCTL_BASE_SPI+0x04)
#define IO_IOCTL_SPI_SET_BAUDRATE          (IOCTL_BASE_SPI+0x05)
#define IO_IOCTL_SPI_GET_BUS_WIDTH         (IOCTL_BASE_SPI+0x06)
#define IO_IOCTL_SPI_SET_BUS_WIDTH         (IOCTL_BASE_SPI+0x07)
#define IO_IOCTL_SPI_GET_BAUDRATE_MAX      (IOCTL_BASE_SPI+0x08)
#define IO_IOCTL_SPI_SET_IO_CALLBACK       (IOCTL_BASE_SPI+0x09)
#define IO_IOCTL_SPI_SET_CARD_CALLBACK     (IOCTL_BASE_SPI+0x0A)
#define IO_IOCTL_SPI_GET_CARD_PRESENCE     (IOCTL_BASE_SPI+0x0B)


/* SPI error codes */
#define SPI_OK                             (0x00)
#define SPI_ERR                            (-1)
#define SPI_ERROR_INIT_FAILED              (SPI_ERROR_BASE | 0x01)
#define SPI_ERROR_COMMAND_FAILED           (SPI_ERROR_BASE | 0x02)
#define SPI_ERROR_COMMAND_TIMEOUT          (SPI_ERROR_BASE | 0x03)
#define SPI_ERROR_DATA_TRANSFER            (SPI_ERROR_BASE | 0x04)
#define SPI_ERROR_INVALID_BUS_WIDTH        (SPI_ERROR_BASE | 0x05)


/* SPI bus widths */
#define SPI_BUS_WIDTH_1BIT                 (0x00)
#define SPI_BUS_WIDTH_4BIT                 (0x01)
#define SPI_BUS_WIDTH_8BIT                 (0x02)


/* SPI card types */
#define SPI_CARD_NONE                      (0x00)
#define SPI_CARD_UNKNOWN                   (0x01)
#define SPI_CARD_SD                        (0x02)
#define SPI_CARD_SDHC                      (0x03)
#define SPI_CARD_SDIO                      (0x04)
#define SPI_CARD_SDCOMBO                   (0x05)
#define SPI_CARD_SDHCCOMBO                 (0x06)
#define SPI_CARD_MMC                       (0x07)
#define SPI_CARD_CEATA                     (0x08)


/* SPI standard baud rates */
#define SPI_DEFAULT_BAUDRATE               (25000000)
#define SPI_INIT_BAUDRATE                  (400000)


/*--------------------------------------------------------------------------*/
/*
**                    DATATYPE DECLARATIONS
*/

/*
** SPI_IO_INT_CALLBACK
**
** This callback function is used to notify that IO card provide interrupt
*/
typedef void (_CODE_PTR_ SPI_IO_INT_CALLBACK)(void *context_data);

/*
** SPI_IO_INT_CALLBACK_STRUCT
**
** This structure defines the parameters of the IO card interrupt callback
** when passed to IO_IOCTL_SPI_SET_IO_CALLBACK.
*/

typedef struct spi_io_int_callback_struct
{
   /* The spi callback itself */
   SPI_IO_INT_CALLBACK  CALLBACK;

   /* User data */
   void            *USERDATA;

} SPI_IO_INT_CALLBACK_STRUCT, * SPI_IO_INT_CALLBACK_STRUCT_PTR;

/*
** spi_CARD_PRESENCE_CALLBACK
**
** This callback function is used to notify that card presence changed
*/
typedef void (_CODE_PTR_ SPI_CARD_PRESENCE_CALLBACK)(void *context_data, bool presence);

/*
** SPI_CARD_PRESENCE_CALLBACK_STRUCT
**
** This structure defines the parameters of the card presence change callback
** when passed to IO_IOCTL_SPI_SET_CARD_CALLBACK.
*/

typedef struct spi_card_presence_callback_struct
{
   /* The SPI callback itself */
   SPI_CARD_PRESENCE_CALLBACK  CALLBACK;

   /* User data */
   void            *USERDATA;

} SPI_CARD_PRESENCE_CALLBACK_STRUCT, * SPI_CARD_PRESENCE_CALLBACK_STRUCT_PTR;

#define SPI_COMMAND_CMDIX_MASK        0x0000003F
#define SPI_COMMAND_CMDTYPE_MASK      0x00000F00
#define SPI_COMMAND_CMDRESPONSE_MASK  0x00FFF000
#define SPI_COMMAND_FLAGS_MASK        0xFF000000

#define SPI_COMMAND_CMDIX_SHIFT       0
#define SPI_COMMAND_CMDTYPE_SHIFT     8
#define SPI_COMMAND_CMDRESPONSE_SHIFT 12
#define SPI_COMMAND_FLAGS_SHIFT       24

#define SPI_COMMAND_TYPE_NORMAL       (((0x0) << SPI_COMMAND_CMDTYPE_SHIFT) & SPI_COMMAND_CMDTYPE_MASK)
#define SPI_COMMAND_TYPE_SUSPEND      (((0x1) << SPI_COMMAND_CMDTYPE_SHIFT) & SPI_COMMAND_CMDTYPE_MASK)
#define SPI_COMMAND_TYPE_RESUME       (((0x2) << SPI_COMMAND_CMDTYPE_SHIFT) & SPI_COMMAND_CMDTYPE_MASK)
#define SPI_COMMAND_TYPE_ABORT        (((0x3) << SPI_COMMAND_CMDTYPE_SHIFT) & SPI_COMMAND_CMDTYPE_MASK)

#define SPI_COMMAND_RESPONSE_NO       (((0x000) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R1       (((0x001) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R1b      (((0x002) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R2       (((0x004) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R3       (((0x008) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R4       (((0x010) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R5       (((0x020) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R5b      (((0x040) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R6       (((0x080) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)
#define SPI_COMMAND_RESPONSE_R7       (((0x100) << SPI_COMMAND_CMDRESPONSE_SHIFT) & SPI_COMMAND_CMDRESPONSE_MASK)

#define SPI_COMMAND_NONE_FLAG         (((0x000) << SPI_COMMAND_FLAGS_SHIFT) & SPI_COMMAND_FLAGS_MASK)
#define SPI_COMMAND_ACMD_FLAG         (((0x001) << SPI_COMMAND_FLAGS_SHIFT) & SPI_COMMAND_FLAGS_MASK)
#define SPI_COMMAND_DATACMD_FLAG      (((0x002) << SPI_COMMAND_FLAGS_SHIFT) & SPI_COMMAND_FLAGS_MASK)
#define SPI_COMMAND_DATA_READ_FLAG    (((0x004) << SPI_COMMAND_FLAGS_SHIFT) & SPI_COMMAND_FLAGS_MASK)


#define SPI_CREATE_CMD(cmdIx, cmdType, cmdResponse, cmdFlags) ((cmdFlags) | (cmdResponse) | (cmdType) | ((cmdIx) & SPI_COMMAND_CMDIX_MASK))


typedef struct spi_command_struct
{
    uint32_t COMMAND;
    uint32_t ARGUMENT;
    uint32_t BLOCKS;
    uint32_t BLOCKSIZE;
    uint32_t RESPONSE[4];
} SPI_COMMAND_STRUCT, * SPI_COMMAND_STRUCT_PTR;


/*
** SPI_INIT_STRUCT
**
** This structure defines the initialization parameters to be used
** when a spi driver is initialized.
*/
typedef struct spi_init_struct
{
    /* Interrupt vector of the SPI controller */
    uint32_t                     VECTOR;

    /* SPI registers (base address) */
    uint32_t                     DEV_BASE;
    
    /* The communication board allowed maximal baud rate */
    uint32_t                     MAX_BAUD_RATE;

} SPI_INIT_STRUCT, * SPI_INIT_STRUCT_PTR;

typedef const SPI_INIT_STRUCT * SPI_INIT_STRUCT_CPTR;
extern const SPI_INIT_STRUCT _bsp_spi_init;

/*--------------------------------------------------------------------------*/
/*
**                        FUNCTION PROTOTYPES
*/

#ifdef __cplusplus
extern "C" {
#endif

extern const NIO_DEV_FN_STRUCT nio_spi_dev_fn;

#ifdef __cplusplus
}
#endif


#endif

/* EOF */
