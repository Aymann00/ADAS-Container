/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/**
 * @enum : @BlindSpotDirection_t
 */
typedef enum{
	BlindSpotDirection_Right = 0x77 ,/**< BlindSpotDirection_Right */
	BlindSpotDirection_Left  = 0x88 ,
	BlindSpodDirection_Both  = 0x99/**< BlindSpotDirection_Left */
}BlindSpotDirection_t;

/**
 * @enum : @DontPassWarningDirection_t
 */
typedef enum{
	DontPassWarningDirection_Right = 0xAA,
	DontPassWarningDirection_Left  = 0xBB
}DontPassWarningDirection_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
 * @fn 		: _vSSD1306_ForwardCollisionWarning(void)
 * @brief   : For Forward Collision Warning Visualization on SSD1306
 * @param   : void
 * @note 	: After Calling This Function ,
 * 			 The Function ( SSD1306_UpdateScreen ) Should Be Called To Update the Display
 */
void _vSSD1306_ForwardCollisionWarning(void) ;

/**
 * @fn  	: _vSSD1306_BlindSpotWarning(BlindSpotDirection_t)
 * @brief 	: For Blind Spot Warning Visualization on SSD1306
 * @param 	: Copy_u8Direction => Direction of BlindSpot Detected By Algorithm ,
 * 								For Options Go To ( @BlindSpotDirection_t )
 * @note 	: After Calling This Function ,
 * 			 The Function ( SSD1306_UpdateScreen ) Should Be Called To Update the Display
 */
void _vSSD1306_BlindSpotWarning( BlindSpotDirection_t Copy_u8Direction ) ;

/**
 * @fn 		: _vSSD1306_EmergencyElectroniBrake(void)
 * @brief	: For EEBL Visualization on SSD1306
 * @param 	: void
 * @note 	: After Calling This Function ,
 * 			 The Function ( SSD1306_UpdateScreen ) Should Be Called To Update the Display
 */
void _vSSD1306_EmergencyElectronicBrake(void) ;

/**
 * @fn  	: _vSSD1306_DontPassWarning(DontPassWarningDirection_t)
 * @brief 	: For Don't Pass Warning Visualization on SSD1306
 * @param 	: Copy_u8Direction => Direction of Don't Pass Detected By Algorithm ,
 * 								For Options Go To ( @DontPassWarningDirection_t )
 * @note 	: After Calling This Function ,
 * 			 The Function ( SSD1306_UpdateScreen ) Should Be Called To Update the Display
 */
void _vSSD1306_DontPassWarning(DontPassWarningDirection_t Copy_u8Direction) ;


/**
 * @fn  	: _vSSD1306_SafeToPass(DontPassWarningDirection_t)
 * @brief 	: For Safe To Pass Visualization on SSD1306
 * @param 	: Copy_u8Direction => Direction of Safe To Pass Detected By Algorithm ,
 * 								For Options Go To ( @DontPassWarningDirection_t )
 * @note 	: After Calling This Function ,
 * 			 The Function ( SSD1306_UpdateScreen ) Should Be Called To Update the Display
 */
void _vSSD1306_SafeToPass(DontPassWarningDirection_t Copy_u8Direction) ;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRF_CSN_PIN_Pin GPIO_PIN_3
#define NRF_CSN_PIN_GPIO_Port GPIOA
#define NRF_CE_PIN_Pin GPIO_PIN_4
#define NRF_CE_PIN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
