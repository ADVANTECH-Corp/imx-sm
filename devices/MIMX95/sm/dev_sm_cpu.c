/*
** ###################################################################
**
**     Copyright 2023 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/* File containing the implementation of the device CPUs.                   */
/*==========================================================================*/

/* Includes */

#include "sm.h"
#include "dev_sm.h"

/* Local defines */

/* Local types */

/* Local variables */

/*--------------------------------------------------------------------------*/
/* Initialize CPUs                                                          */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuInit(void)
{
    int32_t status = SM_ERR_SUCCESS;

    for (uint32_t cpuId = 0U; cpuId < DEV_SM_NUM_CPU; cpuId++)
    {
        /* Initialize CMC interfaces */
        if (!CPU_Init(cpuId))
        {
            status = SM_ERR_HARDWARE_ERROR;
        }
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Return CPU name                                                          */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuNameGet(uint32_t cpuId, string *cpuNameAddr,
    int32_t *len)
{
    int32_t status = SM_ERR_SUCCESS;
    static int32_t maxLen = 0;

    static string const name[DEV_SM_NUM_CPU] =
    {
        [DEV_SM_CPU_M33P] =  "M33P",
        [DEV_SM_CPU_M7P] =   "M7P",
        [DEV_SM_CPU_A55C0] = "A55C0",
        [DEV_SM_CPU_A55C1] = "A55C1",
        [DEV_SM_CPU_A55C2] = "A55C2",
        [DEV_SM_CPU_A55C3] = "A55C3",
        [DEV_SM_CPU_A55C4] = "A55C4",
        [DEV_SM_CPU_A55C5] = "A55C5",
        [DEV_SM_CPU_A55P] =  "A55P"
    };

    /* Get max string width */
    DEV_SM_MaxStringGet(len, &maxLen, name, DEV_SM_NUM_CPU);

    /* Check CPU */
    if (cpuId >= DEV_SM_NUM_CPU)
    {
        status = SM_ERR_NOT_FOUND;
    }
    else
    {
        /* Return pointer to name */
        *cpuNameAddr = name[cpuId];
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Get CPU info                                                             */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuInfoGet(uint32_t cpuId, uint32_t *runMode,
    uint32_t *sleepMode, uint64_t *vector)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Get run mode */
    if (!CPU_RunModeGet(cpuId, runMode))
    {
        status = SM_ERR_INVALID_PARAMETERS;
    }

    /* Get sleep mode */
    if (status == SM_ERR_SUCCESS)
    {
        if (!CPU_SleepModeGet(cpuId, sleepMode))
        {
            status = SM_ERR_INVALID_PARAMETERS;
        }
    }

    /* Get vector */
    if (status == SM_ERR_SUCCESS)
    {
        if (!CPU_ResetVectorGet(cpuId, vector))
        {
            *vector = 0ULL;
        }
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* CPU start                                                                */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuStart(uint32_t cpuId)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Set CPU run mode to START */
    if (!CPU_RunModeSet(cpuId, CPU_RUN_MODE_START))
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* CPU hold                                                                 */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuHold(uint32_t cpuId)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Set CPU run mode to HOLD */
    if (!CPU_RunModeSet(cpuId, CPU_RUN_MODE_HOLD))
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* CPU stop                                                                 */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuStop(uint32_t cpuId)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Set CPU run mode to STOP */
    if (!CPU_RunModeSet(cpuId, CPU_RUN_MODE_STOP))
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Check reset vector                                                       */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuResetVectorCheck(uint32_t cpuId, uint64_t resetVector,
    bool table)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Check CPU */
    if (cpuId >= DEV_SM_NUM_CPU)
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set reset vector                                                         */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuResetVectorSet(uint32_t cpuId, uint64_t resetVector)
{
    int32_t status = SM_ERR_SUCCESS;

    if (!CPU_ResetVectorSet(cpuId, resetVector))
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set CPU sleep mode                                                       */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuSleepModeSet(uint32_t cpuId, uint32_t sleepMode,
    bool irqMuxGic)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Set CPU target sleep mode on next WFI entry */
    if (!(CPU_SleepModeSet(cpuId, sleepMode)))
    {
        status = SM_ERR_NOT_FOUND;
    }
    else
    {
        /* Set wake mux to GPC/GIC */
        if (!CPU_WakeMuxSet(cpuId, irqMuxGic))
        {
            status = SM_ERR_NOT_FOUND;
        }
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set CPU IRQ wake mask                                                    */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuIrqWakeSet(uint32_t cpuId, uint32_t maskIdx,
    uint32_t maskVal)
{
    int32_t status = SM_ERR_SUCCESS;

    if (maskIdx >= GPC_CPU_CTRL_CMC_IRQ_WAKEUP_MASK_COUNT)
    {
        status = SM_ERR_INVALID_PARAMETERS;
    }
    else
    {
        if (!CPU_IrqWakeSet(cpuId, maskIdx, maskVal))
        {
            status = SM_ERR_NOT_FOUND;
        }
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set CPU non-IRQ wake mask                                                */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuNonIrqWakeSet(uint32_t cpuId, uint32_t maskIdx,
    uint32_t maskVal)
{
    int32_t status = SM_ERR_SUCCESS;

    if (maskIdx >= 1U)
    {
        status = SM_ERR_INVALID_PARAMETERS;
    }
    else
    {
        if (!CPU_NonIrqWakeSet(cpuId, maskVal))
        {
            status = SM_ERR_NOT_FOUND;
        }
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set CPU power domain LPM config                                          */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuPdLpmConfigSet(uint32_t cpuId, uint32_t domainId,
    uint32_t lpmSetting, uint32_t retMask)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Configure CPU LPM response for specified power domain */
    if (!CPU_LpmConfigSet(cpuId, domainId, lpmSetting, retMask))
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Set CPU clock LPM config                                                 */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_CpuClkLpmConfigSet(uint32_t cpuId, uint32_t clockId,
    uint32_t lpmSetting)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Return status */
    return status;
}

