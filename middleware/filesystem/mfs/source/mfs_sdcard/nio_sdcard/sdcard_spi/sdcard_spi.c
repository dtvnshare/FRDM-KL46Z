/*HEADER**********************************************************************
*
* Copyright 2010 Freescale Semiconductor, Inc.
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
*   This file contains the SD card driver functions.
*
*
*END************************************************************************/


#include <mqx.h>

#include <stdio.h>
#include <stdint.h>
#include <nio.h>
#include <ioctl.h>
#include <fs_supp.h>
#include <unistd.h>

#include <sdcard.h>
#include <sdcard_prv.h>
#include <sdcard_spi.h>
#include <spi.h>

const SDCARD_CONST_INIT_STRUCT sdcard_spi_init_data = { _io_sdcard_spi_init, _io_sdcard_spi_read_block, _io_sdcard_spi_write_block, SPI_BUS_WIDTH_4BIT }; //---------


/*!
 * \brief Reformats R2 (CID,CSD) as read from spi registers to a byte array.
 *
 * \param[in] r Registers
 * \param[out] r2 Byte array
 *
 * \return None
 */
static void _io_sdcard_spi_r2_format(uint32_t r[4], uint8_t r2[16])
{
    int i;
    uint32_t tmp = 0; /* initialization required to avoid compilation warning */

    i = 15;
    while (i)
    {
        if ((i % 4) == 3)
        {
            tmp = r[3 - i / 4];
        }
        r2[--i] = tmp & 0xff;
        tmp >>= 8;
    }
    r2[15] = 0;
}

/*!
 * \brief Configures SDCARD for high speed mode
 *
 * \param[in] sdcard_ptr SDCard descriptor structure
 * \param[in] baudrate Desired baudrate in Hz
 *
 * \return SDCARD_OK on success, SDCARD_ERR on error, -NIO_ENODEV if device does not support high speed mode
 */
static int32_t _set_sd_high_speed_mode(SDCARD_STRUCT_PTR sdcard_ptr, uint32_t baudrate)
{
    SPI_COMMAND_STRUCT command;
    uint32_t param;
    uint8_t cmd6_data[64];

    return SDCARD_OK;
}


/*!
 * \brief Initializes spi communication, SD card itself and reads its parameters.
 *
 * \param[in, out] desc SDCARD_STRUCT_PTR, SDCard descriptor structure
 *
 * \return SDCARD_OK on success, SDCARD_ERR on error
 */
int _io_sdcard_spi_init(
    /* [IN/OUT] SD card descriptor */
    void *desc)
{
    uint32_t baudrate, param;
    SPI_COMMAND_STRUCT command;
    uint8_t csd[16];
    SDCARD_STRUCT_PTR sdcard_ptr = (SDCARD_STRUCT_PTR)desc;

    /* Check parameters */
    if ((NULL == sdcard_ptr) || (-1 == sdcard_ptr->COM_DEVICE))
    {
        return SDCARD_ERR;
    }

    return SDCARD_OK;
}


/*!
 * \brief Reads blocks from SD card starting with given index into given buffer.
 *
 * \param[in] desc SDCARD_STRUCT_PTR, SDCard descriptor structure
 * \param[out] buffer Output buffer for the data
 * \param[in] index Index of first block to read
 * \param[in] num Number of blocks to read
 *
 * \return Number of read bytes on success, SDCARD_ERR or -NIO_ETIMEDOUT on error
 */
int _io_sdcard_spi_read_block(
    /* [IN] SD card info */
    void *desc,

    /* [OUT] Buffer to fill with read 512*num bytes */
    unsigned char *buffer,

    /* [IN] Index of first block to read */
    uint32_t index,

    /* [IN] Number of blocks to read */
    uint32_t num)
{
    SDCARD_STRUCT_PTR sdcard_ptr = (SDCARD_STRUCT_PTR)desc;
    SPI_COMMAND_STRUCT command;
    int result;

    /* Check parameters */
    if ((NULL == sdcard_ptr) || (-1 == sdcard_ptr->COM_DEVICE) || (NULL == buffer))
    {
        return SDCARD_ERR;
    }

    return result;
}


/*!
 * \brief Writes blocks starting with given index to SD card from given buffer
 *
 * \param[in] desc SDCARD_STRUCT_PTR, SDCard descriptor structure
 * \param[in] buffer Input buffer for the data
 * \param[in] index Index of first block to write
 * \param[in] num Number of blocks to write
 *
 * \return Number of written bytes on success, SDCARD_ERR or -NIO_ETIMEDOUT on error
 */
int _io_sdcard_spi_write_block(
    /* [IN] SD card descriptor */
    void *desc,

    /* [IN] Buffer with data to write */
    unsigned char *buffer,

    /* [IN] Index of first block to write */
    uint32_t index,

    /* [IN] Number of blocks to be written */
    uint32_t num)
{
    SDCARD_STRUCT_PTR sdcard_ptr = (SDCARD_STRUCT_PTR)desc;
    SPI_COMMAND_STRUCT command;
    uint8_t tmp[4];
    int count;

    /* Check parameters */
    if ((NULL == sdcard_ptr) || (-1 == sdcard_ptr->COM_DEVICE) || (NULL == buffer))
    {
        return SDCARD_ERR;
    }

    return count;
}

/* EOF */
