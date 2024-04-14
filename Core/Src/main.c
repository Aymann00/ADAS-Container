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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*typedef enum
{

}Direction_t;
 */

typedef enum
{
	DistanceCalcOnDMA = 0x01 ,

}EventBits_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CAR_ID							0x11
#define LOCALIZATION_OPERATION_ID		0x01
#define ASK_DIRECTION_OPERATION_ID      0x02
#define Front_Threshold         		100

/*TODO : review angles for all algorithms*/
#define BSW_Minimum_Angle_L				90
#define BSW_Maximium_Angle_L			135

#define BSW_Minimum_Angle_R				225
#define BSW_Maximium_Angle_R			270

#define BSW_Threshold					500

/*TODO : review angles for all algorithms*/
#define DPW_Minimum_Angle_L				15
#define DPW_Maximium_Angle_L			45

#define DPW_Minimum_Angle_R				315
#define DPW_Maximium_Angle_R			345

#define DPW_Threshold					500

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

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
/* Definitions for Local */
osThreadId_t LocalHandle;
const osThreadAttr_t Local_attributes = {
		.name = "Local",
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
uint8_t Distances_Buffer[360] = {0};

#define FRONT 			0
#define FRONT_LEFT 		1
#define LEFT 			2
#define BACK_LEFT 		3
#define BACK 			4
#define BACK_RIGHT 		5
#define RIGHT 			6
#define FRONT_RIGHT 	7


uint8_t My_Direction	=			0;
uint8_t * Obstcales_Detection = 	NULL;
uint8_t Front_Car_ID	=			0;
uint8_t Back_Car_ID		=			0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void Init_Task(void *argument);
void Distance_Calc(void *argument);
void Localization(void *argument);
void Algo_Check(void *argument);
void BSW_Check(void *argument);
void DPW_Check(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint64_t RxpipeAddrs = 0x11223344AA;
char myRxData[32];
char myTxData[32] = "Hello From STM32";
char AckPayload[32] = "Acked by STM32";
char AckPayload_Buffer[32];


uint8_t MyRPIdata[360] = {0};



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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

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
	/* creation of Startup_Task */
	Startup_TaskHandle = osThreadNew(Init_Task, NULL, &Startup_Task_attributes);

	/* creation of Calc_Dis */
	Calc_DisHandle = osThreadNew(Distance_Calc, NULL, &Calc_Dis_attributes);

	/* creation of Local */
	LocalHandle = osThreadNew(Localization, NULL, &Local_attributes);

	/* creation of Check_Algo */
	Check_AlgoHandle = osThreadNew(Algo_Check, NULL, &Check_Algo_attributes);

	/* creation of BSW_Algo */
	BSW_AlgoHandle = osThreadNew(BSW_Check, NULL, &BSW_Algo_attributes);

	/* creation of DPW_Algo */
	DPW_AlgoHandle = osThreadNew(DPW_Check, NULL, &DPW_Algo_attributes);

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

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	huart1.Init.BaudRate = 9600;
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
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{
		uint8_t Received_Data[32] = {0};

		osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY);
		NRF24_read(Received_Data, 32);
		osMutexRelease(NRF_MutexHandle) ;

		switch(Received_Data[1])
		{
		case LOCALIZATION_OPERATION_ID:
			bool Is_Front = ((Received_Data[BACK] >= Obstcales_Detection[FRONT] - 7) &&
					(Received_Data[BACK] <= Obstcales_Detection[FRONT] + 7)) ||
					((Received_Data[BACK_RIGHT] >= Obstcales_Detection[FRONT_LEFT] - 7) &&
							(Received_Data[BACK_LEFT] <= Obstcales_Detection[FRONT_RIGHT] + 7)) ;


			bool Is_Back = ((Received_Data[FRONT] >= Obstcales_Detection[BACK] - 7) &&
					(Received_Data[FRONT] <= Obstcales_Detection[BACK] + 7)) ||
							((Received_Data[BACK_RIGHT] >= Obstcales_Detection[FRONT_LEFT] - 7) &&
									(Received_Data[BACK_LEFT] <= Obstcales_Detection[FRONT_RIGHT] + 7)) ;

			if(Is_Front){

				Front_Car_ID = Received_Data[0];
			}
			else if(Is_Back){
				Back_Car_ID = Received_Data[0];
			}
			else{

				/*
				 * Do Nothing
				 */
			}
		case ASK_DIRECTION_OPERATION_ID :

			bool TX_Flag =0;
			bool RX_Flag =0;

			NRF24_whatHappened(&TX_Flag,NULL,&RX_Flag);

			if(	(CAR_ID == Received_Data[2] && RX_Flag))
			{
				uint8_t ASK_Direction_Frame[4] ={0};

				ASK_Direction_Frame[0] = CAR_ID ;
				ASK_Direction_Frame[1] = ASK_DIRECTION_OPERATION_ID ;
				ASK_Direction_Frame[2] = Front_Car_ID ;
				ASK_Direction_Frame[3] = My_Direction ;

				NRF24_stopListening();

				NRF24_writeAckPayload(1, ASK_Direction_Frame, 4);

				NRF24_startListening();

			}
			else if ((CAR_ID == Received_Data[2] && TX_Flag))
			{
				if (	Received_Data[3]==	My_Direction)
				{
					/*Fire EEBL*/
				}
				else if(Received_Data[3]	!=	My_Direction)
				{
					/*Fire FCW*/
				}
				else
				{
					/*
					 * Do Nothing
					 */
				}
			}
			else {
				/*
				 * Stop immediately
				 */
			}
		default:
			break;
		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Set Event Flag ( Bit 0 ) as Indication For Start Distance Calculation */
	osEventFlagsSet( EventGroupHandle , DistanceCalcOnDMA ) ;
}

uint8_t * _CalcAvgDistance( uint8_t * Data_Arr )
{
	uint16_t Local_CounterI = 0 ;
	int16_t Local_CounterII = 0;
	static uint8_t Local_AvgDistance[8] = {0};

	for (Local_CounterI = 0; Local_CounterI < 8; Local_CounterI++) {
		uint32_t Local_TempI = 0; // Reset Local_TempI for each angle
		int16_t LowerLimit  = (Local_CounterI * 45) - 3;
		uint16_t UpperLimit = (Local_CounterI * 45) + 3;

		for (Local_CounterII = LowerLimit; Local_CounterII <= UpperLimit; Local_CounterII++) {
			// Make sure the index is within bounds (0-359)
			uint16_t Index = (Local_CounterII + 360) % 360;

			Local_TempI += Data_Arr[Index];
		}

		// Calculate average for this angle
		Local_AvgDistance[Local_CounterI] = Local_TempI / 7;
	}

	return Local_AvgDistance;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Init_Task */
/**
 * @brief  Function implementing the Startup_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Init_Task */
void Init_Task(void *argument)
{
	/* USER CODE BEGIN 5 */
	//Init DMA UART to Distances Buffer
	HAL_UART_Receive_DMA(&huart1, Distances_Buffer, 360);
	//NRF Module Initialization -> Less Then 0.5 Sec
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

	/*Add any Inits here*/
	/*Stack Size for this Task = 348 B*/
	osThreadTerminate(Startup_TaskHandle);

	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Distance_Calc */
/**
 * @brief Function implementing the myTask02 thread.
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
		/* Arrange distances returned from the function to be :
		 * 			Front - Back - Right - Left - FR - FL - BR - BL*/
		Obstcales_Detection = _CalcAvgDistance(Distances_Buffer);
	}
	/* USER CODE END Distance_Calc */
}

/* USER CODE BEGIN Header_Localization */
/**
 * @brief Function implementing the Local thread.
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
		uint8_t Localization_Frame[10] = {CAR_ID,LOCALIZATION_OPERATION_ID,
				Obstcales_Detection[FRONT],Obstcales_Detection[BACK],
				Obstcales_Detection[RIGHT],Obstcales_Detection[LEFT],
				Obstcales_Detection[FRONT_RIGHT],Obstcales_Detection[FRONT_LEFT],
				Obstcales_Detection[BACK_RIGHT],Obstcales_Detection[BACK_LEFT]
		};

		/* Protect NRF Shared Resuource */
		osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY) ;

		NRF24_stopListening();
		NRF24_write(Localization_Frame, 10);
		NRF24_startListening();

		osMutexRelease(NRF_MutexHandle);

		osDelay(3000);
	}
	/* USER CODE END Localization */
}

/* USER CODE BEGIN Header_Algo_Check */
/**
 * @brief Function implementing the Check_Algo thread.
 * @param argument: Not used
 * @retval None
 */

/* USER CODE END Header_Algo_Check */
void Algo_Check(void *argument)
{
	/* USER CODE BEGIN Algo_Check */

	/* Infinite loop */
	for(;;)
	{

		if(Obstcales_Detection[FRONT] <= Front_Threshold )
		{
			uint8_t ASK_Direction_Frame[3] ={0};

			ASK_Direction_Frame[0] = CAR_ID ;
			ASK_Direction_Frame[1] = ASK_DIRECTION_OPERATION_ID ;
			ASK_Direction_Frame[2] = Front_Car_ID ;

			/* Protect NRF Shared Resuource */
			osMutexAcquire(NRF_MutexHandle, HAL_MAX_DELAY) ;

			NRF24_stopListening();
			NRF24_write(ASK_Direction_Frame, 3) ;
			NRF24_startListening();

			osMutexRelease(NRF_MutexHandle);

		}
		osDelay(1000);
	}
	/* USER CODE END Algo_Check */
}

/* USER CODE BEGIN Header_BSW_Check */
/**
 * @brief Function implementing the BSW_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BSW_Check */
void BSW_Check(void *argument)
{
	/* USER CODE BEGIN BSW_Check */
	/* Infinite loop */
	for(;;)
	{

		bool BSW_Flag_R = false;
		bool BSW_Flag_L = false;

		/*Check the Left Angles*/
		for (uint8_t Angle_Iterator = BSW_Maximium_Angle_L ;
				Angle_Iterator >= BSW_Minimum_Angle_L ;
				Angle_Iterator--)
		{
			if (Distances_Buffer[Angle_Iterator] <= BSW_Threshold)
			{
				/*break the loop and invoke BSW Left warning*/
				BSW_Flag_L = true;
				break;
			}
		}
		/*Check the Right Angles*/
		for (uint16_t Angle_Iterator = BSW_Minimum_Angle_R ;
				Angle_Iterator <= BSW_Maximium_Angle_R ;
				Angle_Iterator++)
		{
			if (Distances_Buffer[Angle_Iterator] <= BSW_Threshold)
			{
				/*break the loop and invoke BSW Left warning*/
				BSW_Flag_R = true;
				break;
			}
		}

		if ( BSW_Flag_L == true )
		{
			/*Invoke the Algorithm*/
		}
		else
		{
			/*Abort the Algorithm*/
		}
		if ( BSW_Flag_R == true )
		{
			/*Invoke the Algorithm*/
		}
		else
		{
			/*Abort the Algorithm*/
		}

		BSW_Flag_L = false;

		BSW_Flag_R = false;

		osDelay(1000);
	}
	/* USER CODE END BSW_Check */
}

/* USER CODE BEGIN Header_DPW_Check */
/**
 * @brief Function implementing the DPW_Algo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DPW_Check */
void DPW_Check(void *argument)
{
	/* USER CODE BEGIN DPW_Check */
	/* Infinite loop */
	for(;;)
	{
		bool DPW_Flag_R = false;
		bool DPW_Flag_L = false;

		/*Check the Left Angles*/
		for (uint8_t Angle_Iterator = DPW_Maximium_Angle_L ;
				Angle_Iterator >= DPW_Minimum_Angle_L ;
				Angle_Iterator--)
		{
			if (Distances_Buffer[Angle_Iterator] <= DPW_Threshold)
			{
				/*break the loop and invoke DPW Left warning*/
				DPW_Flag_L = true;
				break;
			}
		}
		/*Check the Right Angles*/
		for (uint16_t Angle_Iterator = DPW_Minimum_Angle_R ;
				Angle_Iterator <= DPW_Maximium_Angle_R ;
				Angle_Iterator++)
		{
			if (Distances_Buffer[Angle_Iterator] <= DPW_Threshold)
			{
				/*break the loop and invoke DPW Left warning*/
				DPW_Flag_R = true;
				break;
			}
		}

		if ( DPW_Flag_L == true )
		{
			/*Invoke the Algorithm*/
		}
		else
		{
			/*Abort the Algorithm*/
		}
		if ( DPW_Flag_R == true )
		{
			/*Invoke the Algorithm*/
		}
		else
		{
			/*Abort the Algorithm*/
		}

		DPW_Flag_L = false;

		DPW_Flag_R = false;

		osDelay(1000);
	}

	/* USER CODE END DPW_Check */

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
