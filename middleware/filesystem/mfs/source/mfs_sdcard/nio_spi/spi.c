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
*   The file contains low level SPI driver functions.
*
*
*END************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <mqx.h>
#include <nio.h>

#include <ioctl.h>
#include <fcntl.h>
#include <fs_supp.h>

#include <fsl_device_registers.h>
#include <fsl_port_hal.h>
#include <fsl_sim_hal.h>
#include <fsl_clock_manager.h>


#include <spi.h>
#include <spi_prv.h>


static int nio_spi_open(void *dev_context, const char *dev_name, int flags, void **fp_context, int *error);
static int nio_spi_close(void *dev_context, void *fp_context, int *error);
static int nio_spi_ioctl(void *dev_context, void *fp_context, int *error, unsigned long int cmd, va_list ap);
static int nio_spi_read(void *dev_context, void *fp_context, void *data_ptr, size_t n, int *error);
static int nio_spi_write(void *dev_context, void *fp_context, const void *data_ptr, size_t n, int *error);
static int nio_spi_deinit(void *dev_context, int *error);
static int nio_spi_init(void *init_data, void **dev_context, int *error);

static void _spi_isr(void *parameter);

const NIO_DEV_FN_STRUCT nio_spi_dev_fn = {
    .OPEN = nio_spi_open, .READ = nio_spi_read, .WRITE = nio_spi_write, .LSEEK = NULL, .IOCTL = nio_spi_ioctl, .CLOSE = nio_spi_close, .INIT = nio_spi_init, .DEINIT = nio_spi_deinit,
};

const SPI_INIT_STRUCT _bsp_spi_init = {
    SPI1_IRQn, /* SPI IRQ number      */
    SPI1_BASE, /* Peripheral base addr */
    SPI_MAX_BAUDRATE_25MHz, /* SPI baudrate       */
};

#if 0

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_spi_io_init
* Returned Value   : MQX_OK or -1
* Comments         :
*    This function performs BSP-specific initialization related to spi
*
*END*----------------------------------------------------------------------*/


_mqx_int _bsp_spi_io_init
(
    uint32_t base,
    uint16_t value
)
{
    /* spi.D1  */
    /* spi.D0  */
    /* spi.CLK */
    /* spi.CMD */
    /* spi.D3  */
    /* spi.D2  */

    uint32_t spi_port_base = PORTE_BASE;

    /* Enable clock gate to SPI module */
    SIM_HAL_EnableClock(SIM_BASE, kSimClockGateSpi0);

    return MQX_OK;
}

#endif


/*!
 * \brief Find and set closest divider values for given baudrate.
 *
 * \param[in] spi_device_ptr Device runtime information
 * \param[in] baudrate Desired baudrate in Hz
 *
 * \return spi_OK on success
 */
static int32_t _spi_set_baudrate(
    /* [IN] Device runtime information */
    SPI_DEVICE_STRUCT_PTR spi_device_ptr,

    /* [IN] Desired baudrate in Hz */
    uint32_t baudrate)
{
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;
    uint32_t clock;
    uint32_t pres, div, min, minpres = 0x80, mindiv = 0x0F;
    int32_t val;

    // TODO: fix harcoded instance 0
    clock = CLOCK_SYS_GetSpiFreq(0);

    return SPI_OK;
}


/*!
 * \brief Get current baudrate of spi peripheral.
 *
 * \param[in] spi_device_ptr Device runtime information
 *
 * \return Current baudrate set in the peripheral in Hz
 */
static uint32_t _spi_get_baudrate(
    /* [IN] Device runtime information */
    SPI_DEVICE_STRUCT_PTR spi_device_ptr)
{
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;
    uint32_t clock;
    int32_t div;

    CLOCK_SYS_GetFreq(kSystemClock, &clock);

    return (clock / div);
}

/*!
 * \brief Checks whether write/read operation is in progress. (Device busy)
 *
 * \param[in] spi_base Module base pointer
 *
 * \return true if busy, false if ready
 */
static bool _spi_is_running(
    /* [IN/OUT] Module base pointer */
    uint32_t spi_base)
{
    return 0;//(0 != (SPI_RD_PRSSTAT((SPI_Type *)spi_base) & (SPI_PRSSTAT_RTA_MASK | SPI_PRSSTAT_WTA_MASK | SPI_PRSSTAT_DLA_MASK | SPI_PRSSTAT_CDIHB_MASK | SPI_PRSSTAT_CIHB_MASK)));
}

/*!
 * \brief Internal callback of NIO, initialization of the device and allocation of device context
 *
 * \param[in] init_data spi_INIT_STRUCT_CPTR init structure
 * \param[out] dev_context  spi_DEVICE_STRUCT containing spi context variables
 *
 * \return spi_OK on success, spi_ERR on error
 */
static int nio_spi_init(

    void *init_data, void **dev_context, int *error)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr;

    /* Create device context */
    spi_device_ptr = _mem_alloc_system_zero(sizeof(SPI_DEVICE_STRUCT));
    if (NULL == spi_device_ptr)
    {
        if (error)
        {
            *error = SPI_ERR;
        }
        return -1;
    }
    //_mem_set_type(spi_device_ptr, MEM_TYPE_IO_SPI_DEVICE_STRUCT);

    return 0;
}

/*!
 * \brief Internal callback of NIO, deinitialization of spi and its device context
 *
 * \param[in] dev_context  SPI_DEVICE_STRUCT containing spi context variables
 *
 * \return SPI_OK on success, SPI_ERR on error
 */
static int nio_spi_deinit(void *dev_context, int *error)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)dev_context;

    if (NULL == spi_device_ptr)
    {
        if (error)
        {
            *error = SPI_ERR;
        }
        return -1;
    }

    /* This should not happen, nio assures deinit is called after
       all files are closed! */
    if (spi_device_ptr->COUNT)
    {
        if (error)
        {
            *error = SPI_ERR;
        }
        return -1;
    }

    /* Cleanup */
    _nvic_int_disable(spi_device_ptr->VECTOR);
    _int_install_isr(spi_device_ptr->VECTOR, _int_get_default_isr(), NULL);
    _lwevent_destroy(&spi_device_ptr->LWEVENT);
#if SPI_IS_HANDLING_CACHE
    _mem_free(spi_device_ptr->ADMA2_DATA);
#endif
    _mem_free((void *)spi_device_ptr);

    return 0;
}

/*!
 * \brief SPI registers initialization and card detection.
 *
 * \param[in] SPI_device_ptr  Device runtime information
 *
 * \return SPI_OK on success, SPI_ERR on error
 */
static int32_t nio_spi_init_module(
    /* [IN/OUT] Device runtime information */
    SPI_DEVICE_STRUCT_PTR spi_device_ptr)
{
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;

    spi_device_ptr->CARD = SPI_CARD_NONE;

    _nvic_int_init(spi_device_ptr->VECTOR, BSP_SPI_INT_LEVEL, TRUE);

    return SPI_OK;
}


/*!
 * \brief Function fills in the CARD in the device description
 *
 * \param[out] spi_device_ptr  Device runtime information
 *
 * \return spi_OK on success, SPI_ERROR_x on failure
 */
static _mqx_int _spi_get_card_type(
    /* [IN] The I/O init data pointer */
    SPI_DEVICE_STRUCT_PTR spi_device_ptr)
{
    uint32_t card_type;
    int32_t val;
    SPI_COMMAND_STRUCT command;

    card_type = 0;

#define CARD_MEM_MASK 0x01
#define CARD_IO_MASK 0x02
#define CARD_MMC_MASK 0x04
#define CARD_CEATA_MASK 0x08
#define CARD_HC_MASK 0x10
#define CARD_MP_MASK 0x20

#define GET_CARD_TIMEOUT (100)

    return SPI_OK;
}

/*!
 * \brief Internal callback of NIO, This function opens the SPI device.
 *
 * \param[in] dev_context  SPI_DEVICE_STRUCT containing spi context variables
 * \param[in] dev_name  Name of the device
 * \param[in] flags  The flags to be used during operation
 * \param[out] fp_context  The file context, unused by this driver
 *
 * \return SPI_OK on success, SPI_ERR on error
 */
static int nio_spi_open(
    /* Device context */
    void *dev_context,

    /* Name of the device */
    const char *dev_name,

    /* The flags to be used during operation */
    int flags,

    /* The file context, unused in this driver */
    void **fp_context,

    int *error)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)dev_context;

    /* Exclusive access till close */
    _int_disable();


    return 0;
}


/*!
 * \brief Internal callback of NIO, This function closes an opened SPI device.
 *
 * \param[in] dev_context  SPI_DEVICE_STRUCT containing spi context variables
 * \param[in] fp_context  The file context, unused by this driver
 *
 * \return SPI_OK on success, SPI_ERR on error
 */
static int nio_spi_close(
    /* Device context */
    void *dev_context,

    /* File context*/
    void *fp_context,

    int *error)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)dev_context;
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;

    /* Disable SPI device */
    _int_disable();

    _int_enable();

    return 0;
}


/*!
 * \brief Internal callback of NIO. This function performs miscellaneous services for the SPI I/O device.
 *
 * \param[in] dev_context  SPI_DEVICE_STRUCT containing spi context variables
 * \param[in] fp_context  The file context, unused by this driver
 * \param[in] cmd  The command to perform
 * \param[out] ap  Parameters for the command
 *
 * \return SPI_OK on success, SPI_ERR on error
 */
static int nio_spi_ioctl(
    /* Device context */
    void *dev_context,

    /* File context*/
    void *fp_context,

    /* Error value to be filled */
    int *error,

    /* The command to perform */
    unsigned long int cmd,

    /* Parameters for the command */
    va_list ap)
{
    int32_t val;
    int result = 0;
    uint32_t *param32_ptr;
    void *param_ptr;

    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)dev_context;
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;

    param_ptr = va_arg(ap, void *);
    param32_ptr = param_ptr;

    return 0;
}

/*!
 * \brief Internal callback of NIO. Reads the data into provided buffer.
 *
 * \param[in] dev_context  SPI_DEVICE_STRUCT containing SPI context variables
 * \param[in] fp_context  The file context, unused by this driver
 * \param[out] data_ptr  Where the characters are to be stored
 * \param[in] n  The number of bytes to read
 *
 * \return Returns the number of bytes received on success, SPI_ERR on error (negative)
 */
static int nio_spi_read(
    /* Device context */
    void *dev_context,

    /* File context */
    void *fp_context,

    /* [OUT] Where the characters are to be stored */
    void *data_ptr,

    /* [IN] The number of bytes to read */
    size_t n,

    int *error)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)dev_context;
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;

#if SPI_IS_HANDLING_CACHE
    SPI_ADMA2_DESC *adma2_desc = spi_device_ptr->ADMA2_DATA->DESC;
    int adma2_desc_idx;

    uint8_t *head;
    uint32_t head_len;

    uint8_t *body;
    uint32_t body_len;

    uint8_t *tail;
    uint32_t tail_len;
#else
    SPI_ADMA2_DESC adma2_desc[1];
#endif

    return (spi_device_ptr->BUFFERED_CMD.BLOCKS * spi_device_ptr->BUFFERED_CMD.BLOCKSIZE);
}

/*!
 * \brief Internal callback of NIO. Writes the provided data buffer to the device.
 *
 * \param[in] dev_context  SPI_DEVICE_STRUCT containing SPI context variables
 * \param[in] fp_context  The file context, unused by this driver
 * \param[in] data_ptr  Where the characters are to be taken from
 * \param[in] n  The number of bytes to write
 *
 * \return Returns number of bytes transmitted on success, SPI_ERR on error (negative)
 */
static int nio_spi_write(
    /* Device context */
    void *dev_context,

    /* File context */
    void *fp_context,

    /* [OUT] Where the characters are to be taken from */
    const void *data_ptr,

    /* [IN] The number of bytes to write */
    size_t n,

    int *error)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)dev_context;
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;

    return (spi_device_ptr->BUFFERED_CMD.BLOCKS * spi_device_ptr->BUFFERED_CMD.BLOCKSIZE);
}

/*!
 * \brief SPI ISR. Used for event rising.
 *
 * \param[in] parameter  SPI_DEVICE_STRUCT_PTR Device runtime information
 *
 * \return None
 */
static void _spi_isr(
    /* [IN] The address of the device specific information */
    void *parameter)
{
    SPI_DEVICE_STRUCT_PTR spi_device_ptr = (SPI_DEVICE_STRUCT_PTR)parameter;
    SPI_Type *spi_base = (SPI_Type *)spi_device_ptr->DEV_BASE;
    uint32_t spi_irqstat;
    uint32_t spi_irqsigen;

    /* Back up the IRQ status */
    spi_irqstat = SPI_RD_IRQSTAT(spi_base);
    spi_irqsigen = SPI_RD_IRQSIGEN(spi_base);
    // Clear the all sets IRQ status bits
    SPI_WR_IRQSTAT(spi_base, spi_irqstat);

}

/* EOF */
