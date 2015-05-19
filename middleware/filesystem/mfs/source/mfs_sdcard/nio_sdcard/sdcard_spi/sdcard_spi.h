#ifndef __sdcard_spi_h__
#define __sdcard_spi_h__
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
*   The file contains functions prototypes, defines, structure 
*   definitions private to the SD spi communication.
*
*
*END************************************************************************/

#include <stdint.h>
#include "sdcard.h"
#include "sdcard_spi.h"

/*----------------------------------------------------------------------*/
/*
**                          CONSTANT DEFINITIONS
*/


/*----------------------------------------------------------------------*/
/*
**                    DATATYPE DEFINITIONS
*/


/* Internal functions to SD spi communication */

#ifdef __cplusplus
extern "C" {
#endif


extern  int _io_sdcard_spi_init(void*);
extern int _io_sdcard_spi_read_block(void*, unsigned char *, uint32_t, uint32_t);
extern int _io_sdcard_spi_write_block(void*, unsigned char *, uint32_t, uint32_t);

extern const SDCARD_CONST_INIT_STRUCT sdcard_spi_init_data;

#ifdef __cplusplus
}
#endif


#endif
