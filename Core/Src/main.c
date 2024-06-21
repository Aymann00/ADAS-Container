/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "NRF.h"
#include "../Inc/fonts.h"
#include "../Inc/ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/**
 * @enum  : @EventBits_t
 * @brief : Contains The Bits of the Event Group
 * 		Each Bit is Specified to Carry out Some Operation When Set
 *
 */
typedef enum
{
	DistanceCalcOnDMA = 0x01 ,	/**< DistanceCalcOnDMA 	*/
	EEBL_ASSERTED	  = 0x02 ,  /**< EEBL_ASSERTED 		*/
	FCW_ASSERTED	  = 0x03 ,  /**< FCW_ASSERTED 		*/
	ALGO_CheckonCalc  = 0x04 , 	/**< ALGO_CheckonCalc 	*/

}EventBits_t;
/**
 * @enum  : @EventBits_t
 * @brief : Contains The Bits of the Event Group
 * 		Each Bit is Specified to Carry out Some Operation When Set
 *
 */
typedef enum
{
	FCW_ID=0x55,
	DPW_R_ID=0x56,
	DPW_L_ID=0x57,
	BSW_R_ID=0x58,
	BSW_L_ID=0x59,
	EEBL_ID=0x60,
	ASK_DATA=0xf1

}Algo_message;

typedef enum
{
	Algorithm_NA 			= 0 ,
	Algorithm_Asserted  	= 1 ,
	Algorithm_Cancel		= 2 ,
}AlgorithmAsserted_t;

typedef enum{
	RPI_STOP = 0,
	RPI_MOVE=1
}Motors_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CAR_ID							0x11	/* My Own Car ID */
#define LOCALIZATION_OPERATION_ID		0x01	/* ID For Localization Operation */
#define ASK_DIRECTION_OPERATION_ID      0x02	/* ID For Request Direction */
#define FCW_Threshold                   2000u     /*distances in mm this comes first*/
#define EEBL_Threshold                  1000u     /*distances in mm this comes second*/
#define TOTAL_ANGLES					360
#define LOCALIZATION_TOLERANCE_VALUE	500

/*TODO : review angles & Thresholds for all algorithms*/
#define BSW_Minimum_Angle_L				90
#define BSW_Maximium_Angle_L			135

#define BSW_Minimum_Angle_R				225
#define BSW_Maximium_Angle_R			270

#define BSW_Threshold					700

/*TODO : review angles & Thresholds for all algorithms*/
#define DPW_Minimum_Angle_L				15
#define DPW_Maximium_Angle_L			45

#define DPW_Minimum_Angle_R				315
#define DPW_Maximium_Angle_R			345

#define DPW_Threshold					2000u

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;


/* Definitions for Startup_Task */
osThreadId_t Startup_TaskHandle;
const osThreadAttr_t Startup_Task_attributes = {
  .name = "Startup_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Calc_Dis */
osThreadId_t Calc_DisHandle;
const osThreadAttr_t Calc_Dis_attributes = {
  .name = "Calc_Dis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Local_Task */
osThreadId_t Local_TaskHandle;
const osThreadAttr_t Local_Task_attributes = {
  .name = "Local_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Check_Algo */
osThreadId_t Check_AlgoHandle;
const osThreadAttr_t Check_Algo_attributes = {
  .name = "Check_Algo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for BSW_Algo */
osThreadId_t BSW_AlgoHandle;
const osThreadAttr_t BSW_Algo_attributes = {
  .name = "BSW_Algo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DPW_Algo */
osThreadId_t DPW_AlgoHandle;
const osThreadAttr_t DPW_Algo_attributes = {
  .name = "DPW_Algo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Receiveing */
osThreadId_t ReceiveingHandle;
const osThreadAttr_t Receiveing_attributes = {
  .name = "Receiveing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FCW_Algo */
osThreadId_t FCW_AlgoHandle;
const osThreadAttr_t FCW_Algo_attributes = {
  .name = "FCW_Algo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EEBL_Algo */
osThreadId_t EEBL_AlgoHandle;
const osThreadAttr_t EEBL_Algo_attributes = {
  .name = "EEBL_Algo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_LidarData */
osThreadId_t Task_LidarDataHandle;
const osThreadAttr_t Task_LidarData_attributes = {
  .name = "Task_LidarData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for NRF_Mutex */
osMutexId_t NRF_MutexHandle;
const osMutexAttr_t NRF_Mutex_attributes = {
  .name = "NRF_Mutex"
};
/* Definitions for EventGroup */
osEventFlagsId_t EventGroupHandle;
const osEventFlagsAttr_t EventGroup_attributes = {
  .name = "EventGroup"
};
/* USER CODE BEGIN PV */
/* Distances Sent From Lidar of the 360 Degrees */
uint8_t Distances_Buffer_str[TOTAL_ANGLES][5] = {{0}};
uint16_t Distances_Buffer[TOTAL_ANGLES] = {0};
/* Indicies of Directions in the Final Calculated Average Distances Array
 *
 */
#define FRONT_RIGHT 	7
#define FRONT 			6
#define FRONT_LEFT 		5
#define LEFT 			4
#define BACK_LEFT 		3
#define BACK 			2
#define BACK_RIGHT 		1
#define RIGHT 			0



uint8_t My_Direction	=			0;
uint16_t * Obstcales_Detection = 	NULL;
uint8_t Front_Car_ID	=			0;
uint8_t Back_Car_ID		=			0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void Init_Task(void *argument);
void Distance_Calc(void *argument);
void Localization(void *argument);
void Check_Algorithm(void *argument);
void BSW_Algorithm(void *argument);
void DPW_Algorithm(void *argument);
void Wireless_Receiving(void *argument);
void FCW_Algorithm(void *argument);
void EEBL_Algorithm(void *argument);
void Ask_LidarData(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t RxpipeAddrs = 0x11223344AA;
char myRxData[32];
char myTxData[32] = "Hello From STM32";
char AckPayload[32] = "Acked by STM32";
char AckPayload_Buffer[32];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	/* Initialize DMA with UART to Generate Interrupt When Receiving all 360 Angle Distances */
	HAL_UART_Receive_DMA(&huart1, Distances_Buffer_str, (uint16_t)(TOTAL_ANGLES*5));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of NRF_Mutex */
  NRF_MutexHandle = osMutexNew(&NRF_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* creation of Startup_Task */
  Startup_TaskHandle = osThreadNew(Init_Task, NULL, &Startup_Task_attributes);

  /* creation of Calc_Dis */
  Calc_DisHandle = osThreadNew(Distance_Calc, NULL, &Calc_Dis_attributes);

  /* creation of Local_Task */
  Local_TaskHandle = osThreadNew(Localization, NULL, &Local_Task_attributes);

  /* creation of Check_Algo */
  Check_AlgoHandle = osThreadNew(Check_Algorithm, NULL, &Check_Algo_attributes);

  /* creation of BSW_Algo */
  BSW_AlgoHandle = osThreadNew(BSW_Algorithm, NULL, &BSW_Algo_attributes);

  /* creation of DPW_Algo */
  DPW_AlgoHandle = osThreadNew(DPW_Algorithm, NULL, &DPW_Algo_attributes);

  /* creation of Receiveing */
  ReceiveingHandle = osThreadNew(Wireless_Receiving, NULL, &Receiveing_attributes);

  /* creation of FCW_Algo */
  FCW_AlgoHandle = osThreadNew(FCW_Algorithm, NULL, &FCW_Algo_attributes);

  /* creation of EEBL_Algo */
  EEBL_AlgoHandle = osThreadNew(EEBL_Algorithm, NULL, &EEBL_Algo_attributes);

  /* creation of Task_LidarData */
  Task_LidarDataHandle = osThreadNew(Ask_LidarData, NULL, &Task_LidarData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of EventGroup */
  EventGroupHandle = osEventFlagsNew(&EventGroup_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRF_CSN_PIN_Pin|NRF_CE_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CSN_PIN_Pin NRF_CE_PIN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_PIN_Pin|NRF_CE_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @fn 		:	void HAL_GPIO_EXTI_Callback(uint16_t)
 * @brief 	:	EXTI Generated By NRF Module
 *
 * @param 	:	GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{

	}
}

/**
 * @fn 		:	void HAL_UART_RxCpltCallback(UART_HandleTypeDef*)
 * @brief 	:	UART Receive Interrupt with DMA
 *
 * @param huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Set Event Flag ( Bit 0 ) as Indication For Start Distance Calculation */
	osEventFlagsSet( EventGroupHandle , DistanceCalcOnDMA ) ;
}

/**
 * @fn		: uint8_t _CalcAvgDistance*(uint8_t*)
 * @brief 	: This Function Calculates Average Distances of 3 Angles Beside the Main Angle For Ex
 * 				--> @Angles: 87,88,89, 90 ,91,92,93 We Get The Average Distance and we Repeat the Process on
 * 				Other Angles Like 0(FRONT),45(FRONT_LEFT),90(LEFT),135(BACK_LEFT),180(BACK),
 * 				225(BACK_RIGHT),270(RIGHT),315(FRONT RIGHT)
 * @param	: Data_Arr --> Total Array Received From Rasberrypi of 360 Elements
 * @return	: An Array of 8 Elements Each element is an Average Distance @ a Pre-defined Angles
 */
uint16_t * _CalcAvgDistance( uint16_t * Data_Arr )
{
	uint16_t Local_CounterI = 0 ;
	int16_t Local_CounterII = 0;
	uint8_t Local_Zeros	= 0 ;
	static uint16_t Local_AvgDistance[8] = {0};

	for (Local_CounterI = 0; Local_CounterI < 8; Local_CounterI++) {
		uint32_t Local_TempI = 0; // Reset Local_TempI for each angle
		int16_t LowerLimit  = (Local_CounterI * 45) - 3;
		uint16_t UpperLimit = (Local_CounterI * 45) + 3;

		for (Local_CounterII = LowerLimit; Local_CounterII <= UpperLimit; Local_CounterII++) {
			// Make sure the index is within bounds (0-359)
			uint16_t Index = (Local_CounterII + TOTAL_ANGLES) % TOTAL_ANGLES;

			if( 0==Data_Arr[Index] )
			{
				Local_Zeros++;
			}
			else
			{
				Local_TempI += Data_Arr[Index];
			}
		}

		// Calculate average for this angle
		Local_AvgDistance[Local_CounterI] = Local_TempI / (7-Local_Zeros);
		Local_Zeros = 0 ;
	}

	return Local_AvgDistance;
}

void _vSSD1306_ForwardCollisionWarning(void)
{
	SSD1306_DrawRectangle(0, 0 , 128u ,  64u , SSD1306_COLOR_WHITE ) ;
	SSD1306_GotoXY(64-60,4) ;
	SSD1306_Puts("Forward Collision",&Font_7x10,SSD1306_COLOR_WHITE) ;
	SSD1306_GotoXY(64-(25),15) ;
	SSD1306_Puts("Warning",&Font_7x10,SSD1306_COLOR_WHITE) ;

	SSD1306_DrawBitmap(64-18 , 26  , ForwardCollision_Bitmap , 35 , 35, SSD1306_COLOR_WHITE) ;
}

void _vSSD1306_BlindSpotWarning( BlindSpotDirection_t Copy_u8Direction )
{
	SSD1306_DrawRectangle(0, 0 , 128u ,  64u , SSD1306_COLOR_WHITE ) ;
	SSD1306_GotoXY(64-(35),4) ;
	SSD1306_Puts("Blind Spot",&Font_7x10,SSD1306_COLOR_WHITE) ;

	if( Copy_u8Direction == BlindSpotDirection_Right )
	{
		SSD1306_GotoXY(64-(56),15) ;
		SSD1306_Puts("Warning On Right",&Font_7x10,SSD1306_COLOR_WHITE) ;
	}
	else if( Copy_u8Direction == BlindSpotDirection_Left )
	{
		SSD1306_GotoXY(64-(53),15) ;
		SSD1306_Puts("Warning On Left",&Font_7x10,SSD1306_COLOR_WHITE) ;
	}
	else
	{
		/* Do Nothing */
	}
	SSD1306_DrawBitmap(64-18 , 26  , BlindSpotWarning_Bitmap , 35 , 35, SSD1306_COLOR_WHITE) ;


}

void _vSSD1306_EmergencyElectronicBrake(void)
{
	SSD1306_DrawRectangle(0, 0 , 128u ,  64u , SSD1306_COLOR_WHITE ) ;
	SSD1306_GotoXY(64-28,4) ;
	SSD1306_Puts("Warning!",&Font_7x10,SSD1306_COLOR_WHITE) ;
	SSD1306_GotoXY(64-46,15) ;
	SSD1306_Puts("Front Vehicle",&Font_7x10,SSD1306_COLOR_WHITE) ;
	SSD1306_GotoXY(64-42,26) ;
	SSD1306_Puts("Hard Braking",&Font_7x10,SSD1306_COLOR_WHITE) ;

	SSD1306_DrawBitmap(64-13 , 37  , EEBL_Bitmap , 25 , 25, SSD1306_COLOR_WHITE) ;


}

void _vSSD1306_DontPassWarning(DontPassWarningDirection_t Copy_u8Direction)
{
	SSD1306_DrawRectangle(0, 0 , 128u ,  64u , SSD1306_COLOR_WHITE ) ;
	SSD1306_GotoXY(64-53,4) ;
	SSD1306_Puts("Don't Pass From",&Font_7x10,SSD1306_COLOR_WHITE) ;

	if( Copy_u8Direction == DontPassWarningDirection_Right )
	{
		SSD1306_GotoXY(64-18,15) ;
		SSD1306_Puts("Right",&Font_7x10,SSD1306_COLOR_WHITE) ;
	}
	else if( Copy_u8Direction == DontPassWarningDirection_Left )
	{
		SSD1306_GotoXY(64-14,15) ;
		SSD1306_Puts("Left",&Font_7x10,SSD1306_COLOR_WHITE) ;
	}
	else
	{
		/* Do Nothing */
	}

	SSD1306_DrawBitmap(64-18 , 26  , DontPassWarning_Bitmap , 35 , 35, SSD1306_COLOR_WHITE) ;

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */

/* USER CODE BEGIN Header_Init_Task */
/**
 * @brief Function implementing the Startup_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Init_Task */
void Init_Task(void *argument)
{
  /* USER CODE BEGIN Init_Task */

	/* Initializing SSD1306 ( OLED Display ) */
	SSD1306_Init();
	/* NRF Module Initialization -> Less Then 0.5 Sec */
	/* Protecting Shared Resource -> NRF Module
	 *  */
	osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY);

	NRF24_begin(hspi1);
	NRF24_setAutoAck(true);
	NRF24_setPayloadSize(32);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_openWritingPipe(RxpipeAddrs);
	NRF24_writeAckPayload(1, AckPayload, 32);
	NRF24_startListening();

	osMutexRelease(NRF_MutexHandle);

	/* Add Any Initializations Here */
	/* Stack Size for this Task ( @Run Time ) = 348 B */
	/* Terminating StartupTask as It is No longer Important in the Sys */
	osThreadTerminate(Startup_TaskHandle);

  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Distance_Calc */
/**
 * @brief Function implementing the Calc_Dis thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Distance_Calc */
void Distance_Calc(void *argument)
{
  /* USER CODE BEGIN Distance_Calc */
	/* Infinite loop */
	for(;;)
	{
		/* Wait on DMA Interrupt On Receive to Come */
		osEventFlagsWait( EventGroupHandle , DistanceCalcOnDMA , osFlagsWaitAny , HAL_MAX_DELAY ) ;

		/* Convert Strings to Integers */
		for( uint16_t LocalItterator = 0 ; LocalItterator < TOTAL_ANGLES ; LocalItterator++ )
		{
			Distances_Buffer[LocalItterator] = atoi(Distances_Buffer_str[LocalItterator]) ;
		}
		/* Arrange distances returned from the function to be :
		 * 			Front - Back - Right - Left - FR - FL - BR - BL*/
		Obstcales_Detection = _CalcAvgDistance(Distances_Buffer);

		/* Setting a Flag That Indicates For Distance Calculation Finished
		 * That Starts Checking on Distances in the Task -> (@Algo_Check)
		 */
		osEventFlagsSet( EventGroupHandle , ALGO_CheckonCalc ) ;
	}
  /* USER CODE END Distance_Calc */
}

/* USER CODE BEGIN Header_Localization */
/**
 * @brief Function implementing the Local_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Localization */
void Localization(void *argument)
{
  /* USER CODE BEGIN Localization */
	/* Infinite loop */
	for(;;)
	{
		/* Localization Frame to Be Sent via NRF */
		uint8_t Localization_Frame[10] = {CAR_ID,LOCALIZATION_OPERATION_ID,
				Obstcales_Detection[FRONT],Obstcales_Detection[FRONT_LEFT],
				Obstcales_Detection[LEFT],Obstcales_Detection[BACK_LEFT],
				Obstcales_Detection[BACK],Obstcales_Detection[BACK_RIGHT],
				Obstcales_Detection[RIGHT],Obstcales_Detection[FRONT_RIGHT]
		};

		/* Protecting Shared Resource -> NRF Module
		 * */
		osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY) ;

		NRF24_stopListening();
		NRF24_write(Localization_Frame, 10);
		NRF24_startListening();

		osMutexRelease(NRF_MutexHandle);

		/* TODO: Timing Should Be Considered */
		osDelay(1500);
	}
  /* USER CODE END Localization */
}

/* USER CODE BEGIN Header_Check_Algorithm */
/**
 * @brief Function implementing the Check_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Check_Algorithm */
void Check_Algorithm(void *argument)
{
  /* USER CODE BEGIN Check_Algorithm */
	uint8_t Local_u8SendToRaspiContMov = RPI_MOVE ;
	/* Infinite loop */
	for(;;)
	{
		/* Wait on Distance Calculation First To Finish
		 * */
		osEventFlagsWait( EventGroupHandle , ALGO_CheckonCalc , osFlagsWaitAny, HAL_MAX_DELAY ) ;

		/* Checking on Front Threshold */
		if( ( Obstcales_Detection[FRONT] <= FCW_Threshold ) && (!( Obstcales_Detection[FRONT] <= EEBL_Threshold )) )
		{
			//Invoke FCW algorithm
			osEventFlagsSet( EventGroupHandle , FCW_ASSERTED ) ;
		}
		else if( Obstcales_Detection[FRONT] <= EEBL_Threshold )
		{
			//Invoke EEBL algorithm
			osEventFlagsSet( EventGroupHandle , EEBL_ASSERTED ) ;
		}
		else
		{
			HAL_UART_Transmit(&huart1, &Local_u8SendToRaspiContMov, 1, HAL_MAX_DELAY ) ;
		}
	}
  /* USER CODE END Check_Algorithm */
}

/* USER CODE BEGIN Header_BSW_Algorithm */
/**
 * @brief Function implementing the BSW_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BSW_Algorithm */
void BSW_Algorithm(void *argument)
{
  /* USER CODE BEGIN BSW_Algorithm */
	bool Local_BSWLeft = false ;
	bool Local_BSWRight= false ;
	bool Local_BSWL_LastState = false ;
	bool Local_BSWR_LastState = false ;

	/* Infinite loop */
	for(;;)
	{
		/* Wait on DMA Interrupt On Receive to Come */
		osEventFlagsWait( EventGroupHandle , ALGO_CheckonCalc , osFlagsWaitAny , HAL_MAX_DELAY ) ;

		Local_BSWL_LastState = Local_BSWLeft  ;
		Local_BSWR_LastState = Local_BSWRight ;

		/*Check the Left Angles*/
		for (uint8_t Angle_Iterator = BSW_Maximium_Angle_L ;
				Angle_Iterator >= BSW_Minimum_Angle_L ;
				Angle_Iterator--)
		{
			if ( ( 0 != Distances_Buffer[Angle_Iterator] ) && (Distances_Buffer[Angle_Iterator] <= BSW_Threshold ))
			{
				/*break the loop and invoke BSW Left warning*/
				Local_BSWLeft = true ;

				break;
			}
		}
		/*Check the Right Angles*/
		for (uint16_t Angle_Iterator = BSW_Minimum_Angle_R ;
				Angle_Iterator <= BSW_Maximium_Angle_R ;
				Angle_Iterator++)
		{
			if ( ( 0 != Distances_Buffer[Angle_Iterator] ) && (Distances_Buffer[Angle_Iterator] <= BSW_Threshold) )
			{
				/*break the loop and invoke BSW Left warning*/
				Local_BSWRight = true;

				break;
			}
		}

		if ( ( Local_BSWLeft == true ) && ( Local_BSWL_LastState != true ) )
		{
			/*Invoke the Algorithm*/
			_vSSD1306_BlindSpotWarning(BlindSpotDirection_Left);
			SSD1306_UpdateScreen();

		}
		else if ( ( Local_BSWLeft == false ) && ( Local_BSWL_LastState == true ) )
		{
			/*Abort the Algorithm*/
			SSD1306_Clear();
		}
		else
		{
			/* Do Nothing */
		}

		if ( ( Local_BSWRight == true ) && ( Local_BSWR_LastState != true ) )
		{
			/*Invoke the Algorithm*/
			_vSSD1306_BlindSpotWarning(BlindSpotDirection_Right);
			SSD1306_UpdateScreen();
		}
		else if ( ( Local_BSWRight == false ) && ( Local_BSWR_LastState == true ) )
		{
			/*Abort the Algorithm*/
			SSD1306_Clear();
		}
		else
		{
			/* Do Nothing */
		}


	}
  /* USER CODE END BSW_Algorithm */
}

/* USER CODE BEGIN Header_DPW_Algorithm */
/**
 * @brief Function implementing the DPW_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DPW_Algorithm */
void DPW_Algorithm(void *argument)
{
  /* USER CODE BEGIN DPW_Algorithm */
	bool Local_DPWLeft = false ;
	bool Local_DPWRight= false ;
	bool Local_DPWL_LastState = false ;
	bool Local_DPWR_LastState = false ;


	/* Infinite loop */
	for(;;)
	{
		Local_DPWL_LastState = Local_DPWLeft ;
		Local_DPWR_LastState = Local_DPWRight;

		/* Wait on DMA Interrupt On Receive to Come */
		osEventFlagsWait( EventGroupHandle , ALGO_CheckonCalc , osFlagsWaitAny , HAL_MAX_DELAY ) ;

		uint8_t MessageToWarnBackCar[]={CAR_ID, 0 , Back_Car_ID};

		/*Check the Left Angles*/
		for (uint8_t Angle_Iterator = DPW_Maximium_Angle_L ;
				Angle_Iterator >= DPW_Minimum_Angle_L ;
				Angle_Iterator--)
		{
			if ( ( 0 != Distances_Buffer[Angle_Iterator] ) && (Distances_Buffer[Angle_Iterator] <= DPW_Threshold))
			{
				/*break the loop and invoke DPW Left warning*/

				Local_DPWLeft = true;

				break;
			}
		}
		/*Check the Right Angles*/
		for (uint16_t Angle_Iterator = DPW_Minimum_Angle_R ;
				Angle_Iterator <= DPW_Maximium_Angle_R ;
				Angle_Iterator++)
		{
			if ( ( 0 != Distances_Buffer[Angle_Iterator] ) && (Distances_Buffer[Angle_Iterator] <= DPW_Threshold) )
			{
				/*break the loop and invoke DPW Right warning*/
				Local_DPWRight = true;
				break;
			}
		}

		if ( ( Local_DPWLeft == true ) && ( Local_DPWL_LastState != true ) )
		{
			/*Invoke the Algorithm*/
			/* Send warning to the Backward Vehicle ( Don't Pass Warning ) via NRF */
			MessageToWarnBackCar[1]=DPW_L_ID;
			osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY) ;

			NRF24_stopListening();
			NRF24_write( MessageToWarnBackCar , 3 ) ;
			NRF24_startListening();

			osMutexRelease(NRF_MutexHandle);
		}
		else if ( ( Local_DPWLeft == false ) && ( Local_DPWL_LastState == true ) )
		{
			/*Abort the Algorithm*/
		}
		else
		{
			/* Do Nothing */
		}

		if ( ( Local_DPWRight == true ) && ( Local_DPWR_LastState != true ) )
		{
			/*Invoke the Algorithm*/
			/* Send warning to the Backward Vehicle ( Don't Pass Warning ) via NRF */
			MessageToWarnBackCar[1]=DPW_R_ID;
			osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY) ;

			NRF24_stopListening();
			NRF24_write( MessageToWarnBackCar , 3 ) ;
			NRF24_startListening();

			osMutexRelease(NRF_MutexHandle);

		}
		else if ( ( Local_DPWRight == false ) && ( Local_DPWR_LastState == true ) )
		{
			/*Abort the Algorithm*/
		}
		else
		{
			/* Do Nothing */
		}

	}
  /* USER CODE END DPW_Algorithm */
}

/* USER CODE BEGIN Header_Wireless_Receiving */
/**
 * @brief Function implementing the Receiveing thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Wireless_Receiving */
void Wireless_Receiving(void *argument)
{
  /* USER CODE BEGIN Wireless_Receiving */
	/* Infinite loop */
	for(;;)
	{
		if(NRF24_available()){
			uint8_t Received_Data[32] = {0};

			osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY);
			NRF24_read(Received_Data, 32);
			osMutexRelease(NRF_MutexHandle) ;

			switch(Received_Data[1])
			{
			case LOCALIZATION_OPERATION_ID:
				bool Is_Front = ((Received_Data[BACK+2] >= Obstcales_Detection[FRONT] - LOCALIZATION_TOLERANCE_VALUE) &&
						(Received_Data[BACK+2] <= Obstcales_Detection[FRONT] + LOCALIZATION_TOLERANCE_VALUE)) ||
						((Received_Data[BACK_RIGHT+2] >= Obstcales_Detection[FRONT_LEFT] - LOCALIZATION_TOLERANCE_VALUE) &&
								(Received_Data[BACK_LEFT+2] <= Obstcales_Detection[FRONT_RIGHT] + LOCALIZATION_TOLERANCE_VALUE)) ;


				bool Is_Back = ((Received_Data[FRONT+2] >= Obstcales_Detection[BACK] - LOCALIZATION_TOLERANCE_VALUE) &&
						(Received_Data[FRONT+2] <= Obstcales_Detection[BACK] + LOCALIZATION_TOLERANCE_VALUE)) ||
								((Received_Data[BACK_RIGHT+2] >= Obstcales_Detection[FRONT_LEFT] - LOCALIZATION_TOLERANCE_VALUE) &&
										(Received_Data[BACK_LEFT+2] <= Obstcales_Detection[FRONT_RIGHT] + LOCALIZATION_TOLERANCE_VALUE)) ;

				if(Is_Front){

					Front_Car_ID = Received_Data[0];
					if( Received_Data[0] == Back_Car_ID )
					{
						/* Reset */
						Back_Car_ID = 0;
					}
				}
				else if(Is_Back){
					Back_Car_ID = Received_Data[0];
					if( Received_Data[0] == Front_Car_ID )
					{
						/* Reset */
						Front_Car_ID = 0;
					}
				}
				else{

					/*
					 * Do Nothing
					 */
				}
				break;
			case EEBL_ID :
				/* This Message is For me & Came From the Front Car */
				if( ( Received_Data[2] == CAR_ID ) && ( Received_Data[0] == Front_Car_ID ) )
				{
					/* OLED Warning Front Vehicle Hard Braking */
					_vSSD1306_EmergencyElectronicBrake();
					SSD1306_UpdateScreen();
				}

				break;
			case DPW_L_ID :
				/* This Message is For me & Came From the Front Car */
				if( ( Received_Data[2] == CAR_ID ) && ( Received_Data[0] == Front_Car_ID ) )
				{
					_vSSD1306_DontPassWarning(DontPassWarningDirection_Left) ;
					SSD1306_UpdateScreen() ;
				}
				else
				{
					/* Do Nothing */
				}

				break ;

			case DPW_R_ID :
				/* This Message is For me & Came From the Front Car */
				if( ( Received_Data[2] == CAR_ID ) && ( Received_Data[0] == Front_Car_ID ) )
				{
					_vSSD1306_DontPassWarning(DontPassWarningDirection_Right) ;
					SSD1306_UpdateScreen() ;
				}
				else
				{
					/* Do Nothing */
				}
				break ;
			default:
				break;
			}

		}
		osDelay(1);
	}
  /* USER CODE END Wireless_Receiving */
}

/* USER CODE BEGIN Header_FCW_Algorithm */
/**
 * @brief Function implementing the FCW_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_FCW_Algorithm */
void FCW_Algorithm(void *argument)
{
  /* USER CODE BEGIN FCW_Algorithm */
	/* Infinite loop */
	for(;;)
	{
		osEventFlagsWait(EventGroupHandle, FCW_ASSERTED , osFlagsWaitAny , HAL_MAX_DELAY ) ;

		/* Implement the Algorithm
		 * */
		/* buzzer on as warning */
		_vSSD1306_ForwardCollisionWarning();
		SSD1306_UpdateScreen() ;

	}
  /* USER CODE END FCW_Algorithm */
}

/* USER CODE BEGIN Header_EEBL_Algorithm */
/**
 * @brief Function implementing the EEBL_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_EEBL_Algorithm */
void EEBL_Algorithm(void *argument)
{
  /* USER CODE BEGIN EEBL_Algorithm */
	uint8_t Local_u8SendToRaspiStopNow = RPI_STOP ;
	/* Infinite loop */
	for(;;)
	{
		osEventFlagsWait(EventGroupHandle, EEBL_ASSERTED , osFlagsWaitAny , HAL_MAX_DELAY ) ;

		/* Implement the Algorithm
		 * */
		uint8_t MessageToWarnBackCar[]={CAR_ID,EEBL_ID,Back_Car_ID};
		/* Send Message to the Raspberry Pi to Take Actions and Stop Motor */
		HAL_UART_Transmit(&huart1, &Local_u8SendToRaspiStopNow, 1, HAL_MAX_DELAY ) ;

		/* Send warning to the Backward Vehicle to check on Algorithm via NRF */
		osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY) ;

		NRF24_stopListening();
		NRF24_write( MessageToWarnBackCar , 3 ) ;
		NRF24_startListening();

		osMutexRelease(NRF_MutexHandle);
		//sending to backward cars to process the case using NRF

	}
  /* USER CODE END EEBL_Algorithm */
}

/* USER CODE BEGIN Header_Ask_LidarData */
/**
 * @brief Function implementing the Task_LidarData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Ask_LidarData */
void Ask_LidarData(void *argument)
{
  /* USER CODE BEGIN Ask_LidarData */
	uint8_t  Local_u8AskLidarForData = ASK_DATA ;
	/* Infinite loop */
	for(;;)
	{
		HAL_UART_Transmit(&huart1, &Local_u8AskLidarForData, 1, HAL_MAX_DELAY ) ;

		osDelay(1000);
	}
  /* USER CODE END Ask_LidarData */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
