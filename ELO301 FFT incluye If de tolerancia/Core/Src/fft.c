/**
 ******************************************************************************
 * @file    fft.c
 * @author  SIANA Systems
 * @date    MM/YY (ex: 11/23)
 * @brief   Defines the API for the <TBD> module.
 *			This module is responsible for:
 *			  - <TBD>
 *
 * @note
 * @warning
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2024 SIANA Systems </center></h2>
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential.
 *
 * Written by SIANA Systems, 2024
 *
 ******************************************************************************
 */
#ifndef _SRC_FFT_C_
#define _SRC_FFT_C_

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup SIANA
 * @{
 */
/** @addtogroup Drivers
 * @{
 */
/** @addtogroup fft
 * @{
 */

#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------*/
/** @addtogroup PUBLIC_Definitions                                            */
/**@{                                                                         */
/*----------------------------------------------------------------------------*/

/** @} */
/*----------------------------------------------------------------------------*/
/** @addtogroup PUBLIC_Types                                                  */
/**@{                                                                         */
/*----------------------------------------------------------------------------*/
typedef struct
{
} t_fft;

/** @} */
/*----------------------------------------------------------------------------*/
/** @addtogroup PUBLIC_Data                                                  */
/**@{                                                                         */
/*----------------------------------------------------------------------------*/

/** @} */
/*----------------------------------------------------------------------------*/
/** @addtogroup PUBLIC_API                                                    */
/**@{                                                                         */
/*----------------------------------------------------------------------------*/

/**
 * @brief Init fft data structure
 * @param fft Pointer to fft data structure
 * @return true on success, false otherwise
 */
bool fft_Init(t_fft *fft);

/**
 * @brief Open the fft driver
 * @param fft Pointer to fft data structure
 * @return true on success, false otherwise
 */
bool fft_Open(t_fft *fft);

/** @} */
/*----------------------------------------------------------------------------*/
/** @addtogroup PUBLIC_WEAK                                                   */
/**@{                                                                         */
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/** @} */
/*--->> END: PUBLIC API <<----------------------------------------------------*/

/** @} */
/** @} */
/** @} */

#ifdef __cplusplus
}
#endif

#endif   // _SRC_FFT_C_

