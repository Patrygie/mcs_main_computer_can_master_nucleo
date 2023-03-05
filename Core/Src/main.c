/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MCS_SCRN_ADDR 0x004Au

#define Print_Numbers_ID 0xB3u
#define Write_Toggle_Green_LED_ID 0xA3u
#define MAP_ID 0xC3
#define TC_ID 0xD3
#define SPEED_ID 0xE3
#define DIFF_ID 0xF3
#define TS_ID 0xF4
#define LENG_ID 0xF5
#define LINV_ID 0xF6
#define BAT_ID 0xF7
#define RINV_ID 0xF8
#define RENG_ID 0xF9
#define ERR_ID 0xFA
#define HV_ID 0xFB
#define LOW_ID 0xFC
#define delay_t 17

uint8_t can_map_value = 10;
uint8_t can_tc_value = 212;
uint8_t can_speed_value = 100;
uint8_t can_diff_value = 120;
uint8_t can_ts_value = 19;
uint8_t can_leng_value = 45;
uint8_t can_linv_value = 11;
uint8_t can_bat_value = 88;
uint8_t can_rinv_value = 88;
uint8_t can_reng_value = 90;
uint8_t can_err_value = 0;
uint8_t can_hv_value = 0;
uint8_t can_low_value = 0;


extern uint32_t TxMailbox;
extern CAN_TxHeaderTypeDef TxHeader;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == MAP_ID)
	{

	}
}


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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(internalLED_GPIO_Port, internalLED_Pin, 1);
  HAL_Delay(2000);


  TxHeader.StdId = MCS_SCRN_ADDR;

  uint8_t data_length = 2;

  uint8_t data_map_frame[2] = {MAP_ID, can_map_value};

  uint8_t data_tc_frame[2] = {TC_ID, can_tc_value};

  uint8_t data_speed_frame[2] = {SPEED_ID, can_speed_value};

  uint8_t data_diff_frame[2] = {DIFF_ID, can_diff_value};

  uint8_t data_ts_frame[2] = {TS_ID, can_ts_value};

  uint8_t data_leng_frame[2] = {LENG_ID, can_leng_value};

  uint8_t data_linv_frame[2] = {LINV_ID, can_linv_value};

  uint8_t data_bat_frame[2] = {BAT_ID, can_bat_value};

  uint8_t data_rinv_frame[2] = {RINV_ID, can_rinv_value};

  uint8_t data_reng_frame[2] = {RENG_ID, can_reng_value};

  uint8_t data_err_frame[2] = {ERR_ID, can_err_value};

  uint8_t data_hv_frame[2] = {HV_ID, can_hv_value};

  uint8_t data_low_frame[2] = {LOW_ID, can_low_value};

  while (1)
  {
	  data_map_frame[1] += 5;
	  CAN_Transmit(&TxHeader, data_length, data_map_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  --data_tc_frame[1];
	  CAN_Transmit(&TxHeader, data_length, data_tc_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_speed_frame[1]++;
	  CAN_Transmit(&TxHeader, data_length, data_speed_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_diff_frame[1] += 12;
	  CAN_Transmit(&TxHeader, data_length, data_diff_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_ts_frame[1] -= 12;
	  CAN_Transmit(&TxHeader, data_length, data_ts_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_leng_frame[1] += 4;
	  CAN_Transmit(&TxHeader, data_length, data_leng_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_linv_frame[1] -= 2;
	  CAN_Transmit(&TxHeader, data_length, data_linv_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_bat_frame[1] += 8;
	  CAN_Transmit(&TxHeader, data_length, data_bat_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_rinv_frame[1] += 8;
	  CAN_Transmit(&TxHeader, data_length, data_rinv_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_reng_frame[1] -= 1;
	  CAN_Transmit(&TxHeader, data_length, data_reng_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_err_frame[1]++;
	  CAN_Transmit(&TxHeader, data_length, data_err_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_hv_frame[1]++;
	  CAN_Transmit(&TxHeader, data_length, data_hv_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);

	  data_low_frame[1]++;
	  CAN_Transmit(&TxHeader, data_length, data_low_frame, &TxMailbox);
	  HAL_GPIO_TogglePin(internalLED_GPIO_Port, internalLED_Pin);
	  HAL_Delay(delay_t);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
