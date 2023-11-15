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
/* File containing the implementation of the device controls.               */
/*==========================================================================*/

/* Includes */

#include "sm.h"
#include "dev_sm.h"

/* Local defines */

/* Local types */

/* Local variables */

static uint32_t s_ctrl[DEV_SM_NUM_CTRL];

/*--------------------------------------------------------------------------*/
/* Set a control value                                                      */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_ControlSet(uint32_t ctrlId, uint32_t numVal,
    const uint32_t *val)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Check control */
    if (ctrlId < DEV_SM_NUM_CTRL)
    {
        if (numVal == 1U)
        {
            s_ctrl[ctrlId] = val[0];
        }
        else
        {
            status = SM_ERR_INVALID_PARAMETERS;
        }
    }
    else
    {
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Get a control value                                                      */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_ControlGet(uint32_t ctrlId, uint32_t *numRtn, uint32_t *rtn)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Check control */
    if (ctrlId < DEV_SM_NUM_CTRL)
    {
        *numRtn = 1U;
        rtn[0] = s_ctrl[ctrlId];
    }
    else
    {
        *numRtn = 0U;
        status = SM_ERR_NOT_FOUND;
    }

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Do a control action                                                      */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_ControlAction(uint32_t ctrlId, uint32_t action,
    uint32_t numArg, const uint32_t *arg, uint32_t *numRtn, uint32_t *rtn)
{
    int32_t status = SM_ERR_SUCCESS;

    /* Nothing to return */
    *numRtn = 0U;

    /* Return status */
    return status;
}

/*--------------------------------------------------------------------------*/
/* Configure notification flags                                             */
/*--------------------------------------------------------------------------*/
int32_t DEV_SM_ControlFlagsSet(uint32_t ctrlId, uint32_t flags)
{
    return SM_ERR_NOT_SUPPORTED;
}

