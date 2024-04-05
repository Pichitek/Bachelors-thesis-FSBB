/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* HRTIM typedefs */
#define HRTIM_INPUT_CLOCK		((uint64_t)168000000)

#define _100kHz_PERIOD			((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 100000))
#define _200kHz_PERIOD			((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 200000))
#define _300kHz_PERIOD			((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 300000))

#define _OPERATING_FREQ			_100kHz_PERIOD

#define VDDA					((uint16_t)3300)
#define DESIRED_VOLTAGE			((uint16_t)12000)

#define BUCK_THRSH				5000
#define MIXED_THRSH				((uint16_t)((DESIRED_VOLTAGE * 65 ) / 75))
#define BOOST_THRSH				((uint16_t)((DESIRED_VOLTAGE * 85 ) / 75))
#define OVERVOLTAGE				26000


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum state
{
	BUCK = 1,
	BOOST = 2,
	BUCKBOOST = 3,
	DE_ENERGIZE = 4,
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc4;

HRTIM_HandleTypeDef hhrtim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC4_Init(void);
/* USER CODE BEGIN PFP */
static void HRTIM_MultiplePWM(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  // location for the adc conversion results
  volatile uint16_t adc1[2], adc2[2], adc4;

  // processed results
  uint16_t input_voltage, input_current,
  	  	   output_voltage, output_current,
		   choke_current;

  // PI controllers
  PIController PI_Outer_Buck, PI_Inner_Buck;
  PIController PI_Outer_Boost, PI_Inner_Boost;
  PIController PI_Outer_BB, PI_Inner_BB;

  // PI controller gains
  float BUCK_OUTER_KP = 0;
  float BUCK_OUTER_KI = 0;
  float BUCK_INNER_KP = 0;
  float BUCK_INNER_KI = 0;

  float BOOST_OUTER_KP = 0;
  float BOOST_OUTER_KI = 0;
  float BOOST_INNER_KP = 0;
  float BOOST_INNER_KI = 0;

  float BB_OUTER_KP = 0;
  float BB_OUTER_KI = 0;
  float BB_INNER_KP = 0;
  float BB_INNER_KI = 0;

  // enum for switch case
  enum state Converter_state;

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
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  MX_ADC2_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */

  // Mutiple PWM init
  HRTIM_MultiplePWM();

  //HRTIM start-up
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

  //Start counters
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);

  // PI controllers init
  PIController_Init(PI_Outer_Buck, BUCK_OUTER_KP, BUCK_OUTER_KI);
  PIController_Init(PI_Inner_Buck, BUCK_INNER_KP, BUCK_INNER_KI);

  PIController_Init(PI_Outer_Boost, BOOST_OUTER_KP, BOOST_OUTER_KI);
  PIController_Init(PI_Inner_Boost, BOOST_INNER_KP, BOOST_INNER_KI);

  PIController_Init(PI_Outer_BB, BB_OUTER_KP, BB_OUTER_KI);
  PIController_Init(PI_Inner_BB, BB_INNER_KP, BB_INNER_KI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Input and output voltage, current measurements
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1, 2);
	  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2, 2);
	  HAL_ADC_Start_DMA(&hadc4, (uint16_t*)adc4, 1);

	  // Voltage bridge conversion is 120.05  	(16.5k / 1M + 1M + 16.5k), values in mV
	  // Shunt current conversion is 30.49		(1/800u * 41), values in mA

	  adc1[0] = adc1[0] * VDDA / 0xFFF;
	  adc1[1] = adc1[1] * VDDA / 0xFFF;
	  adc2[0] = adc2[0] * VDDA / 0xFFF;
	  adc2[1] = adc2[1] * VDDA / 0xFFF;
	  adc4    = adc4    * VDDA / 0xFFF;

	  input_voltage = (12005 * adc1[0]) / 100;
	  input_current = (3049 * adc1[1]) / 100;
	  output_voltage = (12005 * adc2[1]) / 100;
	  output_current = (3049 * adc2[0]) / 100;
	  choke_current = adc4;

	  if((input_voltage > BUCK_THRSH) && (input_voltage < MIXED_THRSH))  	  Converter_state = BUCK;
	  else if((input_voltage > MIXED_THRSH) && (input_voltage < BOOST_THRSH)) Converter_state = BUCKBOOST;
	  else if((input_voltage > BOOST_THRSH) && (input_voltage < OVERVOLTAGE)) Converter_state = BOOST;
	  else if(input_voltage > OVERVOLTAGE) 									  Converter_state = DE_ENERGIZE;

	  switch(Converter_state)
	  {

	  float pi_outer, pi_inner;
	  uint32_t newDC;
	  HRTIM_CompareCfgTypeDef compare_config;

	  case BUCK: // TIMER A - ON, TIMER B - OFF

		  // Set Duty Cycle as 0, TIMER B = 0 (Q3 open), ~B (Q4 shorted)
		  compare_config.CompareValue = 0;
		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_B, HRTIM_COMPAREUNIT_1, &compare_config);

		  // calculate outer and inner PI controllers responses
		  pi_outer = PIController_Update(&PI_Outer_Buck, DESIRED_VOLTAGE, output_voltage);
		  pi_inner = PIController_Update(&PI_Inner_Buck, pi_outer, output_current);

		  // update duty cycle
		  newDC = (HRTIM_INPUT_CLOCK / _OPERATING_FREQ) * pi_inner;
		  compare_config.CompareValue = newDC;
		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_A, HRTIM_COMPAREUNIT_1, &compare_config);

		  break;

	  case BOOST: // TIMER A - OFF, TIMER B - ON

		  // Set Duty Cycle as 1, TIMER A = 1 (Q1 shorted), ~A (Q2 open)
		  compare_config.CompareValue = 0xffff;
		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_A, HRTIM_COMPAREUNIT_1, &compare_config);

		  // calculate outer and inner PI controllers responses
		  pi_outer = PIController_Update(&PI_Outer_Boost, DESIRED_VOLTAGE, output_voltage);
		  pi_inner = PIController_Update(&PI_Inner_Boost, pi_outer, output_current);

		  // update duty cycle
		  newDC = (HRTIM_INPUT_CLOCK / _OPERATING_FREQ) * pi_inner;
		  compare_config.CompareValue = newDC;
		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_B, HRTIM_COMPAREUNIT_1, &compare_config);

		  break;

	  case BUCKBOOST: // TIMER A - ON (fixed duty cycle) , TIMER B - ON

		  // Set Duty Cycle as 0.75
		  compare_config.CompareValue = (HRTIM_INPUT_CLOCK / _OPERATING_FREQ) * 3 / 4;
		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_A, HRTIM_COMPAREUNIT_1, &compare_config);

		  // calculate outer and inner PI controllers responses
		  pi_outer = PIController_Update(&PI_Outer_BB, DESIRED_VOLTAGE, output_voltage);
		  pi_inner = PIController_Update(&PI_Inner_BB, pi_outer, output_current);

		  // update duty cycle
		  newDC = (HRTIM_INPUT_CLOCK / _OPERATING_FREQ) * pi_inner;
		  compare_config.CompareValue = newDC;
		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_B, HRTIM_COMPAREUNIT_1, &compare_config);
		  break;

	  case DE_ENERGIZE: // TIMER A - OFF (Q1 open, Q2 shorted), TIMER B - OFF (Q3 shorted, Q4 open)

		  // Set Duty Cycle as 0, TIMER A
		  compare_config.CompareValue = 0;
 		  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_A, HRTIM_COMPAREUNIT_1, &compare_config);

 		 // Set Duty Cycle as 1, TIMER B
 		 compare_config.CompareValue = 1;
 		 HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERID_TIMER_B, HRTIM_COMPAREUNIT_1, &compare_config);

		  break;

	  default:

		  break;

	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation = 0;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP4;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xd480;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x6900;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = 0xb;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 0xb;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void HRTIM_MultiplePWM(void)
{
	HRTIM_TimeBaseCfgTypeDef timebase_config;
	HRTIM_TimerCfgTypeDef timer_config;
	HRTIM_OutputCfgTypeDef output_config_TAx;
	HRTIM_OutputCfgTypeDef output_config_TBx;
	HRTIM_CompareCfgTypeDef compare_config;
	HRTIM_DeadTimeCfgTypeDef deadtime_config;


	/* HRTIM Global init */
	hhrtim1.Instance = HRTIM1;
	hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
	hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;

	// HR Timer init
	HAL_HRTIM_Init(&hhrtim1);
	// DLL calibration, set period to 14us
	HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_1);
	// Wait calibration completion
	if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 100) != HAL_OK)
	{
		Error_Handler(); /* if DLL or clock is not correctly set */
	}


	/* TIMERA init, TIMERA config */
	timebase_config.Period = _OPERATING_FREQ;
	timebase_config.RepetitionCounter = 0;
	timebase_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
	timebase_config.Mode = HRTIM_MODE_CONTINUOUS;
	HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timebase_config);

	timer_config.DMARequests = HRTIM_TIM_DMA_NONE;
	timer_config.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
	timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED;
	timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
	timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
	timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
	timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
	timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
	timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
	timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
	timer_config.InterruptRequests = HRTIM_TIM_IT_NONE;
	timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
	timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
	timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
	timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
	timer_config.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
	timer_config.UpdateTrigger= HRTIM_TIMUPDATETRIGGER_NONE;
	timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
	HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,&timer_config);

	deadtime_config.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
	deadtime_config.RisingValue = 0x000;
	deadtime_config.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
	deadtime_config.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
	deadtime_config.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
	deadtime_config.FallingValue = 0x000;
	deadtime_config.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
	deadtime_config.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
	deadtime_config.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
	HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &deadtime_config);

	/* TA1 waveform description */
	output_config_TAx.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
	output_config_TAx.SetSource = HRTIM_OUTPUTSET_TIMPER;
	output_config_TAx.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1;
	output_config_TAx.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
	output_config_TAx.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	output_config_TAx.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
	output_config_TAx.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
	output_config_TAx.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
	HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &output_config_TAx);

	/* TA2 waveform description */
	output_config_TAx.SetSource = HRTIM_OUTPUTSET_NONE;
	output_config_TAx.ResetSource  = HRTIM_OUTPUTRESET_NONE;
	HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &output_config_TAx);

	/* TIMERA Duty Cycles */
	compare_config.CompareValue = _OPERATING_FREQ/2;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &compare_config);

	compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
	compare_config.AutoDelayedTimeout = 0;
	compare_config.CompareValue = _OPERATING_FREQ/2;
	HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compare_config);

	/* TIMERD init */
	timebase_config.Period = _OPERATING_FREQ;
	timebase_config.RepetitionCounter = 0;
	timebase_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
	timebase_config.Mode = HRTIM_MODE_CONTINUOUS;
	HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timebase_config);

	timer_config.DMARequests = HRTIM_TIM_DMA_NONE;
	timer_config.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
	timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED;
	timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
	timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
	timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
	timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
	timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
	timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
	timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
	timer_config.InterruptRequests = HRTIM_TIM_IT_NONE;
	timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
	timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
	timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
	timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
	timer_config.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
	timer_config.UpdateTrigger= HRTIM_TIMUPDATETRIGGER_NONE;
	timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
	HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,&timer_config);

	deadtime_config.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
	deadtime_config.RisingValue = 0x000;
	deadtime_config.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
	deadtime_config.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
	deadtime_config.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
	deadtime_config.FallingValue = 0x000;
	deadtime_config.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
	deadtime_config.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
	deadtime_config.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
	HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &deadtime_config);

	/* TD1 waveform description */
	output_config_TBx.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
	output_config_TBx.SetSource = HRTIM_OUTPUTSET_TIMPER;
	output_config_TBx.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1;
	output_config_TBx.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
	output_config_TBx.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
	output_config_TBx.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
	output_config_TBx.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
	output_config_TBx.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
	HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &output_config_TBx);

	/* TD2 waveform description */
	output_config_TBx.SetSource = HRTIM_OUTPUTSET_TIMCMP2;     //HRTIM_OUTPUTSET_NONE;
	output_config_TBx.ResetSource  = HRTIM_OUTPUTRESET_TIMPER; //HRTIM_OUTPUTRESET_NONE;
	HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &output_config_TBx);
}



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
