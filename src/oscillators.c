/***************************************************************************//**
 * @file
 * @brief Oscillator configuration file.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * Editor: Add your name and email address here:
 * @student    Sonal Tamrakar, sonal.tamrakar@Colorado.edu
 *
 *
 ******************************************************************************/

#include "oscillators.h"
#include "em_cmu.h"
#include "app.h"


/*
 * Depending on the EM defined
 * by the define LOWEST_ENERGY_MODE, this function selects
 * the clock + the oscillator. For EM0-EM2,
 * LFXO oscillator is selected within the LFA
 * clock branch. For EM3, the ULFRCO oscillator is
 * selected also within the LFA clock branch. Appropriate
 * clock dividers are also selected and the LETIMER0
 * is enabled at the end.
 *
 * Parameters:
 *   None
 *
 * Returns:
 *   None
 */
void cmuConfig(void)
{


  if (LOWEST_ENERGY_MODE == 3) // EM3
    {
        CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
        CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
        CMU_ClockEnable(cmuClock_LFA, true);
        CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4); //cmuClkDiv_1
        CMU_ClockEnable(cmuClock_LETIMER0, true);
    }

  else                        // EM0, EM1, EM2
    {
        CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
        CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
        CMU_ClockEnable(cmuClock_LFA, true);
        CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4);
        CMU_ClockEnable(cmuClock_LETIMER0, true);
    }


    //Debug Help
   // uint32_t frequency = CMU_ClockFreqGet (cmuClock_LFA);
   // frequency = CMU_ClockFreqGet (cmuClock_LETIMER0);
}

