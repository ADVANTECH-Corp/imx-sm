/*
** ###################################################################
**
** Copyright 2023-2026 NXP
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**
** o Redistributions of source code must retain the above copyright notice, this list
**   of conditions and the following disclaimer.
**
** o Redistributions in binary form must reproduce the above copyright notice, this
**   list of conditions and the following disclaimer in the documentation and/or
**   other materials provided with the distribution.
**
** o Neither the name of the copyright holder nor the names of its
**   contributors may be used to endorse or promote products derived from this
**   software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
** ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
** ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/* File containing the implementation of the handlers for the board.        */
/*==========================================================================*/

/* Includes */

#include "sm.h"
#include "brd_sm.h"
#include "dev_sm.h"
#include "fsl_lpi2c.h"
#include "fsl_rgpio.h"
#include "pin_mux.h"

/* Local defines */

/* I2C device addresses */
#define BOARD_PF09_DEV_ADDR         0x08U
#define BOARD_PCAL6408A_DEV_ADDR    0x20U
#define BOARD_PF5301_DEV_ADDR       0x2AU
#define BOARD_PF5302_DEV_ADDR       0x29U
#define BOARD_PCA2131_DEV_ADDR      0x53U

#define PCAL6408A_INPUT_PF53_ARM_PG  1U
#define PCAL6408A_INPUT_PF53_SOC_PG  2U
#define PCAL6408A_INPUT_PF09_INT     3U
#define PCAL6408A_INPUT_PCA2131_INT  6U

/* Local types */

/* Local variables */

/* Global variables */

PCAL6408A_Type g_pcal6408aDev;
PF09_Type g_pf09Dev;
PF53_Type g_pf5301Dev;
PF53_Type g_pf5302Dev;
PCA2131_Type g_pca2131Dev;

irq_prio_info_t g_brdIrqPrioInfo[BOARD_NUM_IRQ_PRIO_IDX] =
{
    [BOARD_IRQ_PRIO_IDX_GPIO1_0] =
    {
        .irqId = GPIO1_0_IRQn,
        .irqCntr = 0U,
        .basePrio = 0U,
        .dynPrioEn = false
    }
};

bool g_pca2131Used = false;

uint32_t g_pmicFaultFlags = 0U;

/* Local functions */

static void BRD_SM_Pf09Handler(void);

#if (BOARD_I2C_INSTANCE == 1U)
#define BOARD_I2C_RGPIO              GPIO1
#define BOARD_I2C_SDA_RGPIO_PIN      1U
#define BOARD_I2C_SCL_RGPIO_PIN      0U
#elif (BOARD_I2C_INSTANCE == 2U)
#define BOARD_I2C_RGPIO              GPIO1
#define BOARD_I2C_SDA_RGPIO_PIN      3U
#define BOARD_I2C_SCL_RGPIO_PIN      2U
#endif

void BOARD_I2C_Recovery(void)
{
    static LPI2C_Type *const s_i2cBases[] = LPI2C_BASE_PTRS;
    LPI2C_Type *base = s_i2cBases[BOARD_I2C_INSTANCE];
    lpi2c_master_config_t lpi2cConfig = {0};
    int i;
    static uint32_t const s_i2cClks[] =
    {
        0U,
        CLOCK_ROOT_LPI2C1,
        CLOCK_ROOT_LPI2C2
    };
    uint32_t clockId = s_i2cClks[BOARD_I2C_INSTANCE];
    uint32_t rate;

    rgpio_pin_config_t output_config = {
        kRGPIO_DigitalOutput,
        0,
    };
    rgpio_pin_config_t input_config = {
        kRGPIO_DigitalInput,
        0,
    };

    LPI2C_MasterDeinit(base);

#if (BOARD_I2C_INSTANCE == 1U)
    IOMUXC_SetPinMux(IOMUXC_PAD_I2C1_SCL__GPIO1_IO_BIT0, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C1_SCL__GPIO1_IO_BIT0, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));

    IOMUXC_SetPinMux(IOMUXC_PAD_I2C1_SDA__GPIO1_IO_BIT1, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C1_SDA__GPIO1_IO_BIT1, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));
#elif (BOARD_I2C_INSTANCE == 2U)
    IOMUXC_SetPinMux(IOMUXC_PAD_I2C2_SCL__GPIO1_IO_BIT2, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C2_SCL__GPIO1_IO_BIT2, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));

    IOMUXC_SetPinMux(IOMUXC_PAD_I2C2_SDA__GPIO1_IO_BIT3, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C2_SDA__GPIO1_IO_BIT3, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));
#endif

    /* Set PCNS register value to 0x0 to prepare the RGPIO initialization */
    BOARD_I2C_RGPIO->PCNS = 0x0;

    /* SDA input */
    RGPIO_PinInit(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN, &input_config);

    if ((RGPIO_PinRead(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN) & 0x1) == 0)
    {
        printf("BOARD_I2C_INSTANCE: SDA is low, start I2C recovery.\r\n");

        RGPIO_PinInit(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, &output_config);
		RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, 1);
		SystemTimeDelay(10000);

        for (i = 0; i < 9; i++)
        {
            RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, 1);
            SystemTimeDelay(5);
            RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, 0);
            SystemTimeDelay(5);
        }

        /* 9th clock here, the slave should already
            release the SDA, we can set SDA as high to
            a NAK.*/
        RGPIO_PinInit(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN, &output_config);
        RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN, 1);
        SystemTimeDelay(1); /* Pull up SDA first */
        RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, 1);
        SystemTimeDelay(5); /* plus pervious 1 us */
        RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, 0);
        SystemTimeDelay(5);
        RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN, 0);
        SystemTimeDelay(5);
        RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SCL_RGPIO_PIN, 1);
        SystemTimeDelay(5);
        /* Here: SCL is high, and SDA from low to high, it's a
          * stop condition */
        RGPIO_WritePinOutput(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN, 1);
        SystemTimeDelay(5);

        RGPIO_PinInit(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN, &input_config);
        if ((RGPIO_PinRead(BOARD_I2C_RGPIO, BOARD_I2C_SDA_RGPIO_PIN) & 0x1) == 1)
            printf("BOARD_I2C_INSTANCE: I2C recovery success.\r\n");
        else
            printf("BOARD_I2C_INSTANCE Recovery failed, I2C1 SDA still low!!!\n");
    }

#if (BOARD_I2C_INSTANCE == 1U)
    /* Configure LPI2C 1 */
    IOMUXC_SetPinMux(IOMUXC_PAD_I2C1_SCL__LPI2C1_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C1_SCL__LPI2C1_SCL, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));

    IOMUXC_SetPinMux(IOMUXC_PAD_I2C1_SDA__LPI2C1_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C1_SDA__LPI2C1_SDA, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));
#elif (BOARD_I2C_INSTANCE == 2U)
    /* Configure LPI2C 2 */
    IOMUXC_SetPinMux(IOMUXC_PAD_I2C2_SCL__LPI2C2_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C2_SCL__LPI2C2_SCL, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));

    IOMUXC_SetPinMux(IOMUXC_PAD_I2C2_SDA__LPI2C2_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_PAD_I2C2_SDA__LPI2C2_SDA, IOMUXC_PAD_DSE(0xFU)
        | IOMUXC_PAD_FSEL1(0x3U) | IOMUXC_PAD_PU(0x1U) | IOMUXC_PAD_OD(0x1U));
#endif

    rate = (uint32_t) CCM_RootGetRate(clockId);

    LPI2C_MasterGetDefaultConfig(&lpi2cConfig);

    lpi2cConfig.baudRate_Hz = BOARD_I2C_BAUDRATE; //BOARD_I2C_BAUDRATE_GPIO;
    lpi2cConfig.enableDoze = false;

    LPI2C_MasterInit(base, &lpi2cConfig, rate);
}

/*--------------------------------------------------------------------------*/
/* Init serial devices                                                      */
/*--------------------------------------------------------------------------*/
int32_t BRD_SM_SerialDevicesInit(void)
{
    int32_t status = SM_ERR_SUCCESS;
    LPI2C_Type *const s_i2cBases[] = LPI2C_BASE_PTRS;

    BOARD_I2C_Recovery();

#if 0
    pcal6408a_config_t pcal6408Config;

    /* Fill in PCAL6408A dev */
    g_pcal6408aDev.i2cBase = s_i2cBases[BOARD_I2C_INSTANCE];
    g_pcal6408aDev.devAddr = BOARD_PCAL6408A_DEV_ADDR;

    /* Init the bus expander */
    PCAL6408A_GetDefaultConfig(&pcal6408Config);
    pcal6408Config.inputLatch = 0xFFU;
    if (!PCAL6408A_Init(&g_pcal6408aDev, &pcal6408Config))
    {
        status = SM_ERR_HARDWARE_ERROR;
    }
    else
    {
        if (!PCAL6408A_IntMaskSet(&g_pcal6408aDev, PCAL6408A_INITIAL_MASK))
        {
            status = SM_ERR_HARDWARE_ERROR;
        }
    }
#endif

    if (status == SM_ERR_SUCCESS)
    {
        /* Fill in PF09 PMIC handle */
        g_pf09Dev.i2cBase = s_i2cBases[BOARD_I2C_INSTANCE];
        g_pf09Dev.devAddr = BOARD_PF09_DEV_ADDR;
        g_pf09Dev.crcEn = true;

        /* Initialize PF09 PMIC */
        if (!PF09_Init(&g_pf09Dev))
        {
            status = SM_ERR_HARDWARE_ERROR;
        }

        /* Disable voltage monitor 1 */
        if (status == SM_ERR_SUCCESS)
        {
            if (!PF09_MonitorEnable(&g_pf09Dev, PF09_VMON1, false))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Disable voltage monitor 2 */
        if (status == SM_ERR_SUCCESS)
        {
            if (!PF09_MonitorEnable(&g_pf09Dev, PF09_VMON2, false))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Disable the PWRUP interrupt */
        if (status == SM_ERR_SUCCESS)
        {
            const uint8_t mask[PF09_MASK_LEN] =
            {
                [PF09_MASK_IDX_STATUS1] = 0x08U
            };

            if (!PF09_IntEnable(&g_pf09Dev, mask, PF09_MASK_LEN, false))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Change the LDO3 sequence */
        if (status == SM_ERR_SUCCESS)
        {
            if (!PF09_PmicWrite(&g_pf09Dev, PF09_REG_LDO3_PWRUP,
                0x1EU, 0xFFU))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Set the LDO3 OV bypass */
        if (status == SM_ERR_SUCCESS)
        {
            if (!PF09_PmicWrite(&g_pf09Dev, PF09_REG_LDO3_CFG2,
                0xFCU, 0xFFU))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Enable the LDO3 in RUN mode */
        if (status == SM_ERR_SUCCESS)
        {
            if (!PF09_PmicWrite(&g_pf09Dev, PF09_REG_LDO3_RUN,
                0x20U, 0xFFU))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Set the OV debounce to 50us due to errata ER011/12
           for rev B0 */
        if (g_pf09Dev.id[PF09_REG_REV_ID] == PF09_REV_B0)
        {
            if (status == SM_ERR_SUCCESS)
            {
                if (!PF09_PmicWrite(&g_pf09Dev, PF09_REG_VMON_CFG1,
                    0x94U, 0xFFU))
                {
                    status = SM_ERR_HARDWARE_ERROR;
                }
            }
        }

        /* Save and clear any fault flags */
        if (status == SM_ERR_SUCCESS)
        {
            if (!PF09_FaultFlags(&g_pf09Dev, &g_pmicFaultFlags, true))
            {
                status = SM_ERR_HARDWARE_ERROR;
            }
        }

        /* Handle any already pending PF09 interrupts */
        if (status == SM_ERR_SUCCESS)
        {
            BRD_SM_Pf09Handler();
        }
    }

    if (status == SM_ERR_SUCCESS)
    {
        /* Fill in PF5301 PMIC handle */
        g_pf5301Dev.i2cBase = s_i2cBases[BOARD_I2C_INSTANCE];
        g_pf5301Dev.devAddr = BOARD_PF5301_DEV_ADDR;

        /* Initialize PF5301 PMIC */
        if (!PF53_Init(&g_pf5301Dev))
        {
            status = SM_ERR_HARDWARE_ERROR;
        }

    }

    if (status == SM_ERR_SUCCESS)
    {
        /* Fill in PF5302 PMIC handle */
        g_pf5302Dev.i2cBase = s_i2cBases[BOARD_I2C_INSTANCE];
        g_pf5302Dev.devAddr = BOARD_PF5302_DEV_ADDR;

        /* Initialize PF5302 PMIC */
        if (!PF53_Init(&g_pf5302Dev))
        {
            status = SM_ERR_HARDWARE_ERROR;
        }
    }

    if (status == SM_ERR_SUCCESS)
    {
        /* Fill in PCA2131 RTC handle */
        g_pca2131Dev.i2cBase = s_i2cBases[BOARD_I2C_INSTANCE];
        g_pca2131Dev.devAddr = BOARD_PCA2131_DEV_ADDR;

        /* Initialize PCA2131 RTC */
        if (!PCA2131_Init(&g_pca2131Dev))
        {
            status = SM_ERR_HARDWARE_ERROR;
        }
    }

    if (status == SM_ERR_SUCCESS)
    {
        rgpio_pin_config_t gpioConfig =
        {
            kRGPIO_DigitalInput,
            0U
        };

        /* Init GPIO1-10 */
        RGPIO_PinInit(GPIO1, 10U, &gpioConfig);
        RGPIO_SetPinInterruptConfig(GPIO1, 10U, kRGPIO_InterruptOutput0,
            kRGPIO_InterruptLogicZero);
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set bus expander interrupt mask                                          */
/*--------------------------------------------------------------------------*/
int32_t BRD_SM_BusExpMaskSet(uint8_t val, uint8_t mask)
{
    int32_t status = SM_ERR_SUCCESS;
    static uint8_t cachedMask = PCAL6408A_INITIAL_MASK;
    uint8_t newMask = (cachedMask & ~mask);

    newMask |= val;

    /* Mask changed? */
    if (cachedMask != newMask)
    {
        if (PCAL6408A_IntMaskSet(&g_pcal6408aDev, newMask))
        {
            cachedMask = newMask;
        }
        else
        {
            status = SM_ERR_HARDWARE_ERROR;
        }
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* GPIO1 handler                                                            */
/*--------------------------------------------------------------------------*/
void GPIO1_0_IRQHandler(void)
{
    uint32_t flags;
    //uint8_t status, val;

    /* Get GPIO status */
    flags = RGPIO_GetPinsInterruptFlags(GPIO1, kRGPIO_InterruptOutput0);

#if 0
    /* Get PCAL6408A status */
    (void) PCAL6408A_IntStatusGet(&g_pcal6408aDev, &status);

    /* Get value and Clear PCAL6408A interrupts */
    (void) PCAL6408A_InputGet(&g_pcal6408aDev, &val);

    /* Clear GPIO interrupts */
    RGPIO_ClearPinsInterruptFlags(GPIO1, kRGPIO_InterruptOutput0, flags);

    /* Handle PF09 interrupt */
    if ((status & BIT8(PCAL6408A_INPUT_PF09_INT)) != 0U)
    {
        /* Asserts low */
        /* gcov_excl_ntbr_nextline - rising edge not seen */
        if ((val & BIT8(PCAL6408A_INPUT_PF09_INT)) == 0U)
        {
            BRD_SM_Pf09Handler();
        }
    }

    /* Handle PCA2131 interrupt */
    if (g_pca2131Used && ((status & BIT8(PCAL6408A_INPUT_PCA2131_INT))
        != 0U))
    {
        /* Asserts low */
        /* gcov_excl_ntbr_nextline - rising edge not seen */
        if ((val & BIT8(PCAL6408A_INPUT_PCA2131_INT)) == 0U)
        {
            BRD_SM_BbmHandler();
        }
    }

    /* Handle controls interrupts */
    BRD_SM_ControlHandler(status, val);
#else
    /* Clear GPIO interrupts */
    RGPIO_ClearPinsInterruptFlags(GPIO1, kRGPIO_InterruptOutput0, flags);

    BRD_SM_Pf09Handler();

    if (g_pca2131Used)
        BRD_SM_BbmHandler();
#endif

    /* Adjust dynamic IRQ priority */
    (void) DEV_SM_IrqPrioUpdate();
}

/*==========================================================================*/

/*--------------------------------------------------------------------------*/
/* PF09 handler                                                             */
/*--------------------------------------------------------------------------*/
static void BRD_SM_Pf09Handler(void)
{
    uint8_t stat[PF09_MASK_LEN] = { 0 };

    /* Read status of interrupts */
    (void) PF09_IntStatus(&g_pf09Dev, stat, PF09_MASK_LEN);

    /* Clear pending */
    (void) PF09_IntClear(&g_pf09Dev, stat, PF09_MASK_LEN);

    /* Handle pending temp interrupts */
    if ((stat[PF09_MASK_IDX_STATUS2] & 0x0FU) != 0U)
    {
        BRD_SM_SensorHandler();
    }
}

