/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "can1.h"
#include "can2.h"
#include "ltc6804.h"
#include "thermistor_lut.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STATE_IDLE = 0,
  STATE_PRECHARGE1,
  STATE_PRECHARGE2,
  STATE_TS_ON,
  STATE_ERROR
} State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OVP VOLTAGE2RAW(4.0) // Over-voltage protection threshold
#define UVP VOLTAGE2RAW(2.5) // Under-voltage protection threshold

// adc_from_temp_c = lambda t: (lambda x: round(30000.0*x/(1.0+x)))(math.exp(3500.0*((1.0/(t+273.15))-(1.0/298.15))))
#define OTP 6768 // 60 degrees
#define UTP 24796 // -10 degrees

#define CYCLE_TIME 50 // Main loop cycle time in milliseconds

#define OVP_COUNT_THRESHOLD 500/CYCLE_TIME // Number of consecutive OVP events to trigger protection
#define UVP_COUNT_THRESHOLD 500/CYCLE_TIME // Number of consecutive UVP events to trigger protection
#define OTP_COUNT_THRESHOLD 1000/CYCLE_TIME // Number of consecutive OTP events to trigger protection
#define UTP_COUNT_THRESHOLD 1000/CYCLE_TIME // Number of consecutive UTP events to trigger protection

#define SPI_ERROR_COUNT_THRESHOLD 500/CYCLE_TIME // Number of consecutive SPI errors to trigger protection

#define PRECHARGE_TIMEOUT 10000 // Timeout for precharge in milliseconds
#define MIN_PRECHARGE_TIME 5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VOLTAGE2RAW(V) ((uint16_t)((V) * 10000)) // Convert voltage in V to raw ADC value
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* CAN stuff */
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint8_t txData[8];
uint8_t rxData[8];

// TX structs
struct can1_ams_status_1_t can1_ams_status_1;
struct can1_ams_status_2_t can1_ams_status_2;
struct can1_ams_s01_voltages_1_t can1_ams_s01_voltages_1;
struct can1_ams_s01_voltages_2_t can1_ams_s01_voltages_2;
struct can1_ams_s02_voltages_1_t can1_ams_s02_voltages_1;
struct can1_ams_s02_voltages_2_t can1_ams_s02_voltages_2;
struct can1_ams_s03_voltages_1_t can1_ams_s03_voltages_1;
struct can1_ams_s03_voltages_2_t can1_ams_s03_voltages_2;
struct can1_ams_s04_voltages_1_t can1_ams_s04_voltages_1;
struct can1_ams_s04_voltages_2_t can1_ams_s04_voltages_2;
struct can1_ams_s05_voltages_1_t can1_ams_s05_voltages_1;
struct can1_ams_s05_voltages_2_t can1_ams_s05_voltages_2;
struct can1_ams_s06_voltages_1_t can1_ams_s06_voltages_1;
struct can1_ams_s06_voltages_2_t can1_ams_s06_voltages_2;
struct can1_ams_s07_voltages_1_t can1_ams_s07_voltages_1;
struct can1_ams_s07_voltages_2_t can1_ams_s07_voltages_2;
struct can1_ams_s08_voltages_1_t can1_ams_s08_voltages_1;
struct can1_ams_s08_voltages_2_t can1_ams_s08_voltages_2;
struct can1_ams_s09_voltages_1_t can1_ams_s09_voltages_1;
struct can1_ams_s09_voltages_2_t can1_ams_s09_voltages_2;
struct can1_ams_s10_voltages_1_t can1_ams_s10_voltages_1;
struct can1_ams_s10_voltages_2_t can1_ams_s10_voltages_2;
struct can1_ams_s11_voltages_1_t can1_ams_s11_voltages_1;
struct can1_ams_s11_voltages_2_t can1_ams_s11_voltages_2;
struct can1_ams_s12_voltages_1_t can1_ams_s12_voltages_1;
struct can1_ams_s12_voltages_2_t can1_ams_s12_voltages_2;
struct can1_ams_s01_temperatures_t can1_ams_s01_temperatures;
struct can1_ams_s02_temperatures_t can1_ams_s02_temperatures;
struct can1_ams_s03_temperatures_t can1_ams_s03_temperatures;
struct can1_ams_s04_temperatures_t can1_ams_s04_temperatures;
struct can1_ams_s05_temperatures_t can1_ams_s05_temperatures;
struct can1_ams_s06_temperatures_t can1_ams_s06_temperatures;
struct can1_ams_s07_temperatures_t can1_ams_s07_temperatures;
struct can1_ams_s08_temperatures_t can1_ams_s08_temperatures;
struct can1_ams_s09_temperatures_t can1_ams_s09_temperatures;
struct can1_ams_s10_temperatures_t can1_ams_s10_temperatures;
struct can1_ams_s11_temperatures_t can1_ams_s11_temperatures;
struct can1_ams_s12_temperatures_t can1_ams_s12_temperatures;

// RX structs
struct can1_dbu_status_1_t can1_dbu_status_1;
struct can1_ecu_status_t can1_ecu_status;
struct can1_ams_parameters_set_t can1_ams_parameters_set;
struct can2_ivt_msg_result_u2_t can2_ivt_msg_result_u2;
struct can2_ivt_msg_result_u1_t can2_ivt_msg_result_u1;
struct can2_ivt_msg_result_t_t can2_ivt_msg_result_t;
struct can2_ivt_msg_result_w_t can2_ivt_msg_result_w;
struct can2_ivt_msg_result_wh_t can2_ivt_msg_result_wh;
struct can2_ivt_msg_result_i_t can2_ivt_msg_result_i;
struct can2_ivt_msg_result_u3_t can2_ivt_msg_result_u3;
struct can2_ivt_msg_result_as_t can2_ivt_msg_result_as;

// voltage readings and conversions
uint16_t rawVoltages[126];
uint16_t rawTemps[60];

uint8_t  ovpCounter[126], ovpError = 0;
uint8_t  uvpCounter[126], uvpError = 0;
uint8_t  otpCounter[126], otpError = 0;
uint8_t  utpCounter[126], utpError = 0;
uint8_t  spiTemperatureErrorCounter[24], spiVoltageErrorCounter[24], spiError = 0;

uint8_t amsTxMessageCounter = 0;
uint8_t cellOrTemp = 0;
uint8_t spiTxData[8], spiRxData[8], voltagesPerRegister;
uint8_t nrOfCells[] = {10, 11, 10, 11, 10, 11, 10, 11, 10, 11, 10, 11};
uint8_t cellBaseNum[] = {0, 10, 21, 31, 42, 52, 63, 73, 84, 94, 105, 115};
uint16_t command, pec, rawVoltage, rawTemp;
uint32_t tick, lastTick = 0, deltaTick, isospiRxCount = 0, isospiRxErrorCount = 0;

// FSM stuff
State_t fsmState = STATE_IDLE;
uint32_t prechargeTimer = 0;
uint32_t prechargeStartTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void configCan1Filters(void);
static void configCan2Filters(void);
static void voltageConversions(void);
static void temperatureConversions(void);
static void voltageReadings(void);
static void temperatureReadings(void);
static void voltageSendCan(void);
static void temperatureSendCan(void);
static void readCanMessages(void);
static void sendStatus(void);
static void stepStateMachine(void);
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
  MX_SPI1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  configCan1Filters();
  configCan2Filters();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // set all output pins to the "safe" state
  HAL_GPIO_WritePin(MCU_STATUS_LED1_GPIO_Port, MCU_STATUS_LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MCU_STATUS_LED2_GPIO_Port, MCU_STATUS_LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MCU_STATUS_LED3_GPIO_Port, MCU_STATUS_LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MCU_STATUS_LED4_GPIO_Port, MCU_STATUS_LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(WATCHDOG_INPUT_GPIO_Port, WATCHDOG_INPUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIR_P_ENABLE_GPIO_Port, AIR_P_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIR_N_ENABLE_GPIO_Port, AIR_N_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PRECHARGE_ENABLE_GPIO_Port, PRECHARGE_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MCU_AMS_ERROR_N_GPIO_Port, MCU_AMS_ERROR_N_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SC_RESET_GPIO_Port, SC_RESET_Pin, GPIO_PIN_RESET);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    tick = HAL_GetTick();

    voltageConversions();
    HAL_Delay(3); // Wait for conversion to complete (2.3ms)
    temperatureConversions();
    HAL_Delay(3);
    voltageReadings();
    temperatureReadings();

    // send voltages and temperatures, on alternate passes
    if (cellOrTemp)
    {
      voltageSendCan();
      if (++amsTxMessageCounter >= 24)
        amsTxMessageCounter = 0; // Reset counter after sending all messages
    }
    else // Temperatures
    {
      temperatureSendCan();
    }
    cellOrTemp = !cellOrTemp; // Toggle between cells and temperatures on each pass


    // TODO: Consider moving this to an interrupt if IVT messages are more frequent and FIFO overflows
    // In general IVT overflows should not matter much since we will read the latest message
    readCanMessages();

    stepStateMachine();

    deltaTick = HAL_GetTick() - tick;
    sendStatus();

    while (HAL_GetTick() - tick < CYCLE_TIME);

    HAL_GPIO_TogglePin(WATCHDOG_INPUT_GPIO_Port, WATCHDOG_INPUT_Pin); // Toggle watchdog pin
    if (!amsTxMessageCounter) HAL_GPIO_TogglePin(MCU_STATUS_LED3_GPIO_Port, MCU_STATUS_LED3_Pin);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MCU_STATUS_LED1_Pin|MCU_STATUS_LED2_Pin|WATCHDOG_INPUT_Pin|AIR_P_ENABLE_Pin
                          |PRECHARGE_ENABLE_Pin|AIR_N_ENABLE_Pin|SC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI1_SSN_Pin|MCU_STATUS_LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_STATUS_LED4_GPIO_Port, MCU_STATUS_LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MCU_AMS_ERROR_N_Pin|RST_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MCU_STATUS_LED1_Pin MCU_STATUS_LED2_Pin WATCHDOG_INPUT_Pin AIR_P_ENABLE_Pin
                           PRECHARGE_ENABLE_Pin AIR_N_ENABLE_Pin SC_RESET_Pin */
  GPIO_InitStruct.Pin = MCU_STATUS_LED1_Pin|MCU_STATUS_LED2_Pin|WATCHDOG_INPUT_Pin|AIR_P_ENABLE_Pin
                          |PRECHARGE_ENABLE_Pin|AIR_N_ENABLE_Pin|SC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SSN_Pin MCU_STATUS_LED3_Pin */
  GPIO_InitStruct.Pin = SPI1_SSN_Pin|MCU_STATUS_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_STATUS_LED4_Pin */
  GPIO_InitStruct.Pin = MCU_STATUS_LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_STATUS_LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AMS_ERROR_LATCHED_Pin */
  GPIO_InitStruct.Pin = AMS_ERROR_LATCHED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AMS_ERROR_LATCHED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIR_N_CLOSED_Pin AIR_P_CLOSED_Pin IMD_ERROR_LATCHED_Pin */
  GPIO_InitStruct.Pin = AIR_N_CLOSED_Pin|AIR_P_CLOSED_Pin|IMD_ERROR_LATCHED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_AMS_ERROR_N_Pin RST_OUT_Pin */
  GPIO_InitStruct.Pin = MCU_AMS_ERROR_N_Pin|RST_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PRECHARGE_CLOSED_SIGNAL_Pin */
  GPIO_InitStruct.Pin = PRECHARGE_CLOSED_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PRECHARGE_CLOSED_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SC_PROBE_Pin */
  GPIO_InitStruct.Pin = SC_PROBE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SC_PROBE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void configCan1Filters(void) {
  CAN_FilterTypeDef canFilter;

  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;

  canFilter.FilterIdHigh = CAN1_DBU_STATUS_1_FRAME_ID << 5; // Shift ID to match filter format;
  canFilter.FilterIdLow = CAN1_ECU_STATUS_FRAME_ID << 5;
  canFilter.FilterMaskIdHigh = CAN1_AMS_PARAMETERS_SET_FRAME_ID << 5;
  canFilter.FilterMaskIdLow = 0xFFFF; // Unused

  HAL_CAN_ConfigFilter(&hcan1, &canFilter);
}

static void configCan2Filters(void) {
  CAN_FilterTypeDef canFilter;

  canFilter.FilterBank = 14;
  canFilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canFilter.FilterScale = CAN_FILTERSCALE_16BIT;
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 0;

  canFilter.FilterBank = 0;
  canFilter.FilterIdHigh = CAN2_IVT_MSG_RESULT_U2_FRAME_ID << 5; // Shift ID to match filter format
  canFilter.FilterIdLow = CAN2_IVT_MSG_RESULT_U1_FRAME_ID << 5;
  canFilter.FilterMaskIdHigh = CAN2_IVT_MSG_RESULT_T_FRAME_ID << 5;
  canFilter.FilterMaskIdLow = CAN2_IVT_MSG_RESULT_W_FRAME_ID << 5;
  if (HAL_CAN_ConfigFilter(&hcan2, &canFilter) != HAL_OK)
  {
    Error_Handler();
  }

  canFilter.FilterBank = 1;
  canFilter.FilterIdHigh = CAN2_IVT_MSG_RESULT_WH_FRAME_ID << 5; // Shift ID to match filter format
  canFilter.FilterIdLow = CAN2_IVT_MSG_RESULT_I_FRAME_ID << 5;
  canFilter.FilterMaskIdHigh = CAN2_IVT_MSG_RESULT_U3_FRAME_ID << 5;
  canFilter.FilterMaskIdLow = CAN2_IVT_MSG_RESULT_AS_FRAME_ID << 5;
  if (HAL_CAN_ConfigFilter(&hcan2, &canFilter) != HAL_OK)
  {
    Error_Handler();
  }
}

static void voltageConversions(void)
{
  command = MAKEBROADCASTCMD(ADCV(MD_NORMAL, DCP_NOT_PERMITTED, CH_ALL_CELLS));
  spiTxData[0] = command >> 8;
  spiTxData[1] = command & 0xFF;
  pec = pec15_calc(2, spiTxData);
  spiTxData[2] = pec >> 8;
  spiTxData[3] = pec & 0xFF;
  HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spiTxData, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_SET);
}

static void temperatureConversions(void)
{
  command = MAKEBROADCASTCMD(ADAX(MD_NORMAL, CH_ALL_CELLS));
  spiTxData[0] = command >> 8;
  spiTxData[1] = command & 0xFF;
  pec = pec15_calc(2, spiTxData);
  spiTxData[2] = pec >> 8;
  spiTxData[3] = pec & 0xFF;
  HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, spiTxData, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_SET);
}

static void voltageReadings(void)
{
  for (uint8_t i = 0; i < 12; i++) // foreach slave (12)
  {
    for (uint8_t j = 0; j < 4; j++) // foreach register (4 registers per slave)
    {
      switch (j)
      {
      case 0: // Cell voltages
        command = MAKEADDRCMD(i, RDCVA);
        break;
      case 1: // Cell voltages
        command = MAKEADDRCMD(i, RDCVB);
        break;
      case 2: // Cell voltages
        command = MAKEADDRCMD(i, RDCVC);
        break;
      case 3: // Cell voltages
        command = MAKEADDRCMD(i, RDCVD);
        break;
      }
      spiTxData[0] = command >> 8;
      spiTxData[1] = command & 0xFF;
      pec = pec15_calc(2, spiTxData);
      spiTxData[2] = pec >> 8;
      spiTxData[3] = pec & 0xFF;
      HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, spiTxData, spiRxData, 4+8, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_SET);
      isospiRxCount++;
      // check PEC
      pec = (0xFF00 & (spiRxData[6] << 8)) | (0xFF & spiRxData[7]);
      if (pec != pec15_calc(6, spiRxData))
      {
        spiVoltageErrorCounter[i]++;
        isospiRxErrorCount++;
        if (spiVoltageErrorCounter[i] >= SPI_ERROR_COUNT_THRESHOLD)
        {
          spiError = 1; // Set SPI error
        }
        for (uint8_t k = 0; k < 4; k++)
        {
          rawVoltages[cellBaseNum[i] + k] = 0xFFFF; // Set to invalid value]
        }
      }
      else
      {
        spiVoltageErrorCounter[i] = 0; // Reset error counter

        // check how many cells we should read from this register
        if (j == 4) // Last register
        {
          if (nrOfCells[i] == 10) // Last register and 10 cells
          {
            voltagesPerRegister = 1;
          }
          else // Last register and 11 cells
          {
            voltagesPerRegister = 2;
          }
        }
        else // Not last register
        {
          voltagesPerRegister = 3;
        }

        for (uint8_t k = 0; k < voltagesPerRegister; k++) // foreach cell (in a register)
        {
          rawVoltage = (spiRxData[2 * k + 1] << 8) | spiRxData[2*k];
          rawVoltages[cellBaseNum[i] + j * 4 + k] = rawVoltage;

          // Check for OVP and UVP
          if (rawVoltage > OVP)
          {
            ovpCounter[cellBaseNum[i] + j * 4 + k]++;
            if (ovpCounter[cellBaseNum[i] + j * 4 + k] >= OVP_COUNT_THRESHOLD)
            {
              ovpError = 1; // Set OVP error
            }
          }
          else
          {
            ovpCounter[cellBaseNum[i] + j * 4 + k] = 0; // Reset counter
          }

          if (rawVoltage < UVP)
          {
            uvpCounter[cellBaseNum[i] + j * 4 + k]++;
            if (uvpCounter[cellBaseNum[i] + j * 4 + k] >= UVP_COUNT_THRESHOLD)
            {
              uvpError = 1; // Set UVP error
            }
          }
          else
          {
            uvpCounter[cellBaseNum[i] + j * 4 + k] = 0; // Reset counter
          }
        } // End foreach cell in register
      } // End check PEC
    } // End foreach register
  } // End foreach slave
}

static void temperatureReadings(void)
{
  for (uint8_t i = 0; i < 12; i++) // foreach slave (12)
  {
    for (uint8_t j = 0; j < 2; j++) // foreach register (2 registers per slave)
    {
      switch (j)
      {
      case 0:
        command = MAKEADDRCMD(i, RDAUXA);
        break;
      case 1:
        command = MAKEADDRCMD(i, RDAUXB);
        break;
      }
      spiTxData[0] = command >> 8;
      spiTxData[1] = command & 0xFF;
      pec = pec15_calc(2, spiTxData);
      spiTxData[2] = pec >> 8;
      spiTxData[3] = pec & 0xFF;
      HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, spiTxData, spiRxData, 4+8, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(SPI1_SSN_GPIO_Port, SPI1_SSN_Pin, GPIO_PIN_SET);
      isospiRxCount++;
      // check PEC
      pec = (0xFF00 & (spiRxData[6] << 8)) | (0xFF & spiRxData[7]);
      if (pec != pec15_calc(6, spiRxData))
      {
        spiTemperatureErrorCounter[i]++;
        isospiRxErrorCount++;
        // check if we have too many errors
        if (spiTemperatureErrorCounter[i] >= SPI_ERROR_COUNT_THRESHOLD)
        {
          spiError = 1; // Set SPI error
        }
        for (uint8_t k = 0; k < 4; k++)
        {
          rawTemps[cellBaseNum[i] + k] = 0xFFFF; // Set to invalid value
        }
      } else
      {
        spiTemperatureErrorCounter[i] = 0; // Reset error counter
        // check how many temperatures we should read from this register
        voltagesPerRegister = (j==0) ? 3 : 2;
        for (uint8_t k = 0; k < voltagesPerRegister; k++) // foreach temperature in register
        {
          rawTemp = (spiRxData[2 * k + 1] << 8) | spiRxData[2 * k];
          rawTemps[i * 12 + j * 2 + k] = rawTemp;

          // Check for OTP and UTP
          if (rawTemp > OTP)
          {
            otpCounter[i * 12 + j * 2 + k]++;
            if (otpCounter[i * 12 + j * 2 + k] >= OTP_COUNT_THRESHOLD)
            {
              otpError = 1; // Set OTP error
            }
          } else
          {
            otpCounter[i * 12 + j * 2 + k] = 0; // Reset counter
          }

          if (rawTemp < UTP)
          {
            utpCounter[i * 12 + j * 2 + k]++;
            if (utpCounter[i * 12 + j * 2 + k] >= UTP_COUNT_THRESHOLD)
            {
              utpError = 1; // Set UTP error
            }
          } else
          {
            utpCounter[i * 12 + j * 2 + k] = 0; // Reset counter
          }
        } // End foreach temperature in register
      } // End check PEC
    } // End foreach register
  } // End foreach slave

}

static void voltageSendCan(void)
{
  switch (amsTxMessageCounter)
  {
  case 0:
    can1_ams_s01_voltages_1.s01v01 = can1_ams_s01_voltages_1_s01v01_encode((float)rawVoltages[0] / 10000.0);
    can1_ams_s01_voltages_1.s01v02 = can1_ams_s01_voltages_1_s01v02_encode((float)rawVoltages[1] / 10000.0);
    can1_ams_s01_voltages_1.s01v03 = can1_ams_s01_voltages_1_s01v03_encode((float)rawVoltages[2] / 10000.0);
    can1_ams_s01_voltages_1.s01v04 = can1_ams_s01_voltages_1_s01v04_encode((float)rawVoltages[3] / 10000.0);
    can1_ams_s01_voltages_1.s01v05 = can1_ams_s01_voltages_1_s01v05_encode((float)rawVoltages[4] / 10000.0);
    can1_ams_s01_voltages_1.s01v06 = can1_ams_s01_voltages_1_s01v06_encode((float)rawVoltages[5] / 10000.0);

    can1_ams_s01_voltages_1_pack(txData, &can1_ams_s01_voltages_1, CAN1_AMS_S01_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S01_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S01_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 1:
    can1_ams_s01_voltages_2.s01v07 = can1_ams_s01_voltages_2_s01v07_encode((float) rawVoltages[6] / 10000.0);
    can1_ams_s01_voltages_2.s01v08 = can1_ams_s01_voltages_2_s01v08_encode((float) rawVoltages[7] / 10000.0);
    can1_ams_s01_voltages_2.s01v09 = can1_ams_s01_voltages_2_s01v09_encode((float) rawVoltages[8] / 10000.0);
    can1_ams_s01_voltages_2.s01v10 = can1_ams_s01_voltages_2_s01v10_encode((float) rawVoltages[9] / 10000.0);
    can1_ams_s01_voltages_2.s01v11 = can1_ams_s01_voltages_2_s01v11_encode((float) rawVoltages[10] / 10000.0);

    can1_ams_s01_voltages_2_pack(txData, &can1_ams_s01_voltages_2, CAN1_AMS_S01_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S01_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S01_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 2:
    can1_ams_s02_voltages_1.s02v01 = can1_ams_s02_voltages_1_s02v01_encode((float) rawVoltages[11] / 10000.0);
    can1_ams_s02_voltages_1.s02v02 = can1_ams_s02_voltages_1_s02v02_encode((float) rawVoltages[12] / 10000.0);
    can1_ams_s02_voltages_1.s02v03 = can1_ams_s02_voltages_1_s02v03_encode((float) rawVoltages[13] / 10000.0);
    can1_ams_s02_voltages_1.s02v04 = can1_ams_s02_voltages_1_s02v04_encode((float) rawVoltages[14] / 10000.0);
    can1_ams_s02_voltages_1.s02v05 = can1_ams_s02_voltages_1_s02v05_encode((float) rawVoltages[15] / 10000.0);
    can1_ams_s02_voltages_1.s02v06 = can1_ams_s02_voltages_1_s02v06_encode((float) rawVoltages[16] / 10000.0);

    can1_ams_s02_voltages_1_pack(txData, &can1_ams_s02_voltages_1, CAN1_AMS_S02_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S02_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S02_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 3:
    can1_ams_s02_voltages_2.s02v07 = can1_ams_s02_voltages_2_s02v07_encode((float) rawVoltages[17] / 10000.0);
    can1_ams_s02_voltages_2.s02v08 = can1_ams_s02_voltages_2_s02v08_encode((float) rawVoltages[18] / 10000.0);
    can1_ams_s02_voltages_2.s02v09 = can1_ams_s02_voltages_2_s02v09_encode((float) rawVoltages[19] / 10000.0);
    can1_ams_s02_voltages_2.s02v10 = can1_ams_s02_voltages_2_s02v10_encode((float) rawVoltages[20] / 10000.0);

    can1_ams_s02_voltages_2_pack(txData, &can1_ams_s02_voltages_2, CAN1_AMS_S02_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S02_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S02_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 4:
    can1_ams_s03_voltages_1.s03v01 = can1_ams_s03_voltages_1_s03v01_encode((float) rawVoltages[21] / 10000.0);
    can1_ams_s03_voltages_1.s03v02 = can1_ams_s03_voltages_1_s03v02_encode((float) rawVoltages[22] / 10000.0);
    can1_ams_s03_voltages_1.s03v03 = can1_ams_s03_voltages_1_s03v03_encode((float) rawVoltages[23] / 10000.0);
    can1_ams_s03_voltages_1.s03v04 = can1_ams_s03_voltages_1_s03v04_encode((float) rawVoltages[24] / 10000.0);
    can1_ams_s03_voltages_1.s03v05 = can1_ams_s03_voltages_1_s03v05_encode((float) rawVoltages[25] / 10000.0);
    can1_ams_s03_voltages_1.s03v06 = can1_ams_s03_voltages_1_s03v06_encode((float) rawVoltages[26] / 10000.0);

    can1_ams_s03_voltages_1_pack(txData, &can1_ams_s03_voltages_1, CAN1_AMS_S03_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S03_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S03_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 5:
    can1_ams_s03_voltages_2.s03v07 = can1_ams_s03_voltages_2_s03v07_encode((float) rawVoltages[27] / 10000.0);
    can1_ams_s03_voltages_2.s03v08 = can1_ams_s03_voltages_2_s03v08_encode((float) rawVoltages[28] / 10000.0);
    can1_ams_s03_voltages_2.s03v09 = can1_ams_s03_voltages_2_s03v09_encode((float) rawVoltages[29] / 10000.0);
    can1_ams_s03_voltages_2.s03v10 = can1_ams_s03_voltages_2_s03v10_encode((float) rawVoltages[30] / 10000.0);
    can1_ams_s03_voltages_2.s03v11 = can1_ams_s03_voltages_2_s03v11_encode((float) rawVoltages[31] / 10000.0);

    can1_ams_s03_voltages_2_pack(txData, &can1_ams_s03_voltages_2, CAN1_AMS_S03_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S03_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S03_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 6:
    can1_ams_s04_voltages_1.s04v01 = can1_ams_s04_voltages_1_s04v01_encode((float) rawVoltages[32] / 10000.0);
    can1_ams_s04_voltages_1.s04v02 = can1_ams_s04_voltages_1_s04v02_encode((float) rawVoltages[33] / 10000.0);
    can1_ams_s04_voltages_1.s04v03 = can1_ams_s04_voltages_1_s04v03_encode((float) rawVoltages[34] / 10000.0);
    can1_ams_s04_voltages_1.s04v04 = can1_ams_s04_voltages_1_s04v04_encode((float) rawVoltages[35] / 10000.0);
    can1_ams_s04_voltages_1.s04v05 = can1_ams_s04_voltages_1_s04v05_encode((float) rawVoltages[36] / 10000.0);
    can1_ams_s04_voltages_1.s04v06 = can1_ams_s04_voltages_1_s04v06_encode((float) rawVoltages[37] / 10000.0);

    can1_ams_s04_voltages_1_pack(txData, &can1_ams_s04_voltages_1, CAN1_AMS_S04_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S04_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S04_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 7:
    can1_ams_s04_voltages_2.s04v07 = can1_ams_s04_voltages_2_s04v07_encode((float) rawVoltages[38] / 10000.0);
    can1_ams_s04_voltages_2.s04v08 = can1_ams_s04_voltages_2_s04v08_encode((float) rawVoltages[39] / 10000.0);
    can1_ams_s04_voltages_2.s04v09 = can1_ams_s04_voltages_2_s04v09_encode((float) rawVoltages[40] / 10000.0);
    can1_ams_s04_voltages_2.s04v10 = can1_ams_s04_voltages_2_s04v10_encode((float) rawVoltages[41] / 10000.0);

    can1_ams_s04_voltages_2_pack(txData, &can1_ams_s04_voltages_2, CAN1_AMS_S04_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S04_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S04_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 8:
    can1_ams_s05_voltages_1.s05v01 = can1_ams_s05_voltages_1_s05v01_encode((float) rawVoltages[42] / 10000.0);
    can1_ams_s05_voltages_1.s05v02 = can1_ams_s05_voltages_1_s05v02_encode((float) rawVoltages[43] / 10000.0);
    can1_ams_s05_voltages_1.s05v03 = can1_ams_s05_voltages_1_s05v03_encode((float) rawVoltages[44] / 10000.0);
    can1_ams_s05_voltages_1.s05v04 = can1_ams_s05_voltages_1_s05v04_encode((float) rawVoltages[45] / 10000.0);
    can1_ams_s05_voltages_1.s05v05 = can1_ams_s05_voltages_1_s05v05_encode((float) rawVoltages[46] / 10000.0);
    can1_ams_s05_voltages_1.s05v06 = can1_ams_s05_voltages_1_s05v06_encode((float) rawVoltages[47] / 10000.0);

    can1_ams_s05_voltages_1_pack(txData, &can1_ams_s05_voltages_1, CAN1_AMS_S05_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S05_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S05_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 9:
    can1_ams_s05_voltages_2.s05v07 = can1_ams_s05_voltages_2_s05v07_encode((float) rawVoltages[48] / 10000.0);
    can1_ams_s05_voltages_2.s05v08 = can1_ams_s05_voltages_2_s05v08_encode((float) rawVoltages[49] / 10000.0);
    can1_ams_s05_voltages_2.s05v09 = can1_ams_s05_voltages_2_s05v09_encode((float) rawVoltages[50] / 10000.0);
    can1_ams_s05_voltages_2.s05v10 = can1_ams_s05_voltages_2_s05v10_encode((float) rawVoltages[51] / 10000.0);
    can1_ams_s05_voltages_2.s05v11 = can1_ams_s05_voltages_2_s05v11_encode((float) rawVoltages[52] / 10000.0);

    can1_ams_s05_voltages_2_pack(txData, &can1_ams_s05_voltages_2, CAN1_AMS_S05_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S05_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S05_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 10:
    can1_ams_s06_voltages_1.s06v01 = can1_ams_s06_voltages_1_s06v01_encode((float) rawVoltages[53] / 10000.0);
    can1_ams_s06_voltages_1.s06v02 = can1_ams_s06_voltages_1_s06v02_encode((float) rawVoltages[54] / 10000.0);
    can1_ams_s06_voltages_1.s06v03 = can1_ams_s06_voltages_1_s06v03_encode((float) rawVoltages[55] / 10000.0);
    can1_ams_s06_voltages_1.s06v04 = can1_ams_s06_voltages_1_s06v04_encode((float) rawVoltages[56] / 10000.0);
    can1_ams_s06_voltages_1.s06v05 = can1_ams_s06_voltages_1_s06v05_encode((float) rawVoltages[57] / 10000.0);
    can1_ams_s06_voltages_1.s06v06 = can1_ams_s06_voltages_1_s06v06_encode((float) rawVoltages[58] / 10000.0);

    can1_ams_s06_voltages_1_pack(txData, &can1_ams_s06_voltages_1, CAN1_AMS_S06_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S06_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S06_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 11:
    can1_ams_s06_voltages_2.s06v07 = can1_ams_s06_voltages_2_s06v07_encode((float) rawVoltages[59] / 10000.0);
    can1_ams_s06_voltages_2.s06v08 = can1_ams_s06_voltages_2_s06v08_encode((float) rawVoltages[60] / 10000.0);
    can1_ams_s06_voltages_2.s06v09 = can1_ams_s06_voltages_2_s06v09_encode((float) rawVoltages[61] / 10000.0);
    can1_ams_s06_voltages_2.s06v10 = can1_ams_s06_voltages_2_s06v10_encode((float) rawVoltages[62] / 10000.0);

    can1_ams_s06_voltages_2_pack(txData, &can1_ams_s06_voltages_2, CAN1_AMS_S06_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S06_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S06_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 12:
    can1_ams_s07_voltages_1.s07v01 = can1_ams_s07_voltages_1_s07v01_encode((float) rawVoltages[63] / 10000.0);
    can1_ams_s07_voltages_1.s07v02 = can1_ams_s07_voltages_1_s07v02_encode((float) rawVoltages[64] / 10000.0);
    can1_ams_s07_voltages_1.s07v03 = can1_ams_s07_voltages_1_s07v03_encode((float) rawVoltages[65] / 10000.0);
    can1_ams_s07_voltages_1.s07v04 = can1_ams_s07_voltages_1_s07v04_encode((float) rawVoltages[66] / 10000.0);
    can1_ams_s07_voltages_1.s07v05 = can1_ams_s07_voltages_1_s07v05_encode((float) rawVoltages[67] / 10000.0);
    can1_ams_s07_voltages_1.s07v06 = can1_ams_s07_voltages_1_s07v06_encode((float) rawVoltages[68] / 10000.0);

    can1_ams_s07_voltages_1_pack(txData, &can1_ams_s07_voltages_1, CAN1_AMS_S07_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S07_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S07_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 13:
    can1_ams_s07_voltages_2.s07v07 = can1_ams_s07_voltages_2_s07v07_encode((float) rawVoltages[69] / 10000.0);
    can1_ams_s07_voltages_2.s07v08 = can1_ams_s07_voltages_2_s07v08_encode((float) rawVoltages[70] / 10000.0);
    can1_ams_s07_voltages_2.s07v09 = can1_ams_s07_voltages_2_s07v09_encode((float) rawVoltages[71] / 10000.0);
    can1_ams_s07_voltages_2.s07v10 = can1_ams_s07_voltages_2_s07v10_encode((float) rawVoltages[72] / 10000.0);
    can1_ams_s07_voltages_2.s07v11 = can1_ams_s07_voltages_2_s07v11_encode((float) rawVoltages[73] / 10000.0);

    can1_ams_s07_voltages_2_pack(txData, &can1_ams_s07_voltages_2, CAN1_AMS_S07_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S07_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S07_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 14:
    can1_ams_s08_voltages_1.s08v01 = can1_ams_s08_voltages_1_s08v01_encode((float) rawVoltages[74] / 10000.0);
    can1_ams_s08_voltages_1.s08v02 = can1_ams_s08_voltages_1_s08v02_encode((float) rawVoltages[75] / 10000.0);
    can1_ams_s08_voltages_1.s08v03 = can1_ams_s08_voltages_1_s08v03_encode((float) rawVoltages[76] / 10000.0);
    can1_ams_s08_voltages_1.s08v04 = can1_ams_s08_voltages_1_s08v04_encode((float) rawVoltages[77] / 10000.0);
    can1_ams_s08_voltages_1.s08v05 = can1_ams_s08_voltages_1_s08v05_encode((float) rawVoltages[78] / 10000.0);
    can1_ams_s08_voltages_1.s08v06 = can1_ams_s08_voltages_1_s08v06_encode((float) rawVoltages[79] / 10000.0);

    can1_ams_s08_voltages_1_pack(txData, &can1_ams_s08_voltages_1, CAN1_AMS_S08_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S08_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S08_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 15:
    can1_ams_s08_voltages_2.s08v07 = can1_ams_s08_voltages_2_s08v07_encode((float) rawVoltages[80] / 10000.0);
    can1_ams_s08_voltages_2.s08v08 = can1_ams_s08_voltages_2_s08v08_encode((float) rawVoltages[81] / 10000.0);
    can1_ams_s08_voltages_2.s08v09 = can1_ams_s08_voltages_2_s08v09_encode((float) rawVoltages[82] / 10000.0);
    can1_ams_s08_voltages_2.s08v10 = can1_ams_s08_voltages_2_s08v10_encode((float) rawVoltages[83] / 10000.0);

    can1_ams_s08_voltages_2_pack(txData, &can1_ams_s08_voltages_2, CAN1_AMS_S08_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S08_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S08_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 16:
    can1_ams_s09_voltages_1.s09v01 = can1_ams_s09_voltages_1_s09v01_encode((float) rawVoltages[84] / 10000.0);
    can1_ams_s09_voltages_1.s09v02 = can1_ams_s09_voltages_1_s09v02_encode((float) rawVoltages[85] / 10000.0);
    can1_ams_s09_voltages_1.s09v03 = can1_ams_s09_voltages_1_s09v03_encode((float) rawVoltages[86] / 10000.0);
    can1_ams_s09_voltages_1.s09v04 = can1_ams_s09_voltages_1_s09v04_encode((float) rawVoltages[87] / 10000.0);
    can1_ams_s09_voltages_1.s09v05 = can1_ams_s09_voltages_1_s09v05_encode((float) rawVoltages[88] / 10000.0);
    can1_ams_s09_voltages_1.s09v06 = can1_ams_s09_voltages_1_s09v06_encode((float) rawVoltages[89] / 10000.0);

    can1_ams_s09_voltages_1_pack(txData, &can1_ams_s09_voltages_1, CAN1_AMS_S09_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S09_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S09_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 17:
    can1_ams_s09_voltages_2.s09v07 = can1_ams_s09_voltages_2_s09v07_encode((float) rawVoltages[90] / 10000.0);
    can1_ams_s09_voltages_2.s09v08 = can1_ams_s09_voltages_2_s09v08_encode((float) rawVoltages[91] / 10000.0);
    can1_ams_s09_voltages_2.s09v09 = can1_ams_s09_voltages_2_s09v09_encode((float) rawVoltages[92] / 10000.0);
    can1_ams_s09_voltages_2.s09v10 = can1_ams_s09_voltages_2_s09v10_encode((float) rawVoltages[93] / 10000.0);
    can1_ams_s09_voltages_2.s09v11 = can1_ams_s09_voltages_2_s09v11_encode((float) rawVoltages[94] / 10000.0);

    can1_ams_s09_voltages_2_pack(txData, &can1_ams_s09_voltages_2, CAN1_AMS_S09_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S09_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S09_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 18:
    can1_ams_s10_voltages_1.s10v01 = can1_ams_s10_voltages_1_s10v01_encode((float) rawVoltages[95] / 10000.0);
    can1_ams_s10_voltages_1.s10v02 = can1_ams_s10_voltages_1_s10v02_encode((float) rawVoltages[96] / 10000.0);
    can1_ams_s10_voltages_1.s10v03 = can1_ams_s10_voltages_1_s10v03_encode((float) rawVoltages[97] / 10000.0);
    can1_ams_s10_voltages_1.s10v04 = can1_ams_s10_voltages_1_s10v04_encode((float) rawVoltages[98] / 10000.0);
    can1_ams_s10_voltages_1.s10v05 = can1_ams_s10_voltages_1_s10v05_encode((float) rawVoltages[99] / 10000.0);
    can1_ams_s10_voltages_1.s10v06 = can1_ams_s10_voltages_1_s10v06_encode((float) rawVoltages[100] / 10000.0);

    can1_ams_s10_voltages_1_pack(txData, &can1_ams_s10_voltages_1, CAN1_AMS_S10_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S10_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S10_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 19:
    can1_ams_s10_voltages_2.s10v07 = can1_ams_s10_voltages_2_s10v07_encode((float) rawVoltages[101] / 10000.0);
    can1_ams_s10_voltages_2.s10v08 = can1_ams_s10_voltages_2_s10v08_encode((float) rawVoltages[102] / 10000.0);
    can1_ams_s10_voltages_2.s10v09 = can1_ams_s10_voltages_2_s10v09_encode((float) rawVoltages[103] / 10000.0);
    can1_ams_s10_voltages_2.s10v10 = can1_ams_s10_voltages_2_s10v10_encode((float) rawVoltages[104] / 10000.0);

    can1_ams_s10_voltages_2_pack(txData, &can1_ams_s10_voltages_2, CAN1_AMS_S10_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S10_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S10_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 20:
    can1_ams_s11_voltages_1.s11v01 = can1_ams_s11_voltages_1_s11v01_encode((float) rawVoltages[105] / 10000.0);
    can1_ams_s11_voltages_1.s11v02 = can1_ams_s11_voltages_1_s11v02_encode((float) rawVoltages[106] / 10000.0);
    can1_ams_s11_voltages_1.s11v03 = can1_ams_s11_voltages_1_s11v03_encode((float) rawVoltages[107] / 10000.0);
    can1_ams_s11_voltages_1.s11v04 = can1_ams_s11_voltages_1_s11v04_encode((float) rawVoltages[108] / 10000.0);
    can1_ams_s11_voltages_1.s11v05 = can1_ams_s11_voltages_1_s11v05_encode((float) rawVoltages[109] / 10000.0);
    can1_ams_s11_voltages_1.s11v06 = can1_ams_s11_voltages_1_s11v06_encode((float) rawVoltages[110] / 10000.0);

    can1_ams_s11_voltages_1_pack(txData, &can1_ams_s11_voltages_1, CAN1_AMS_S11_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S11_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S11_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 21:
    can1_ams_s11_voltages_2.s11v07 = can1_ams_s11_voltages_2_s11v07_encode((float) rawVoltages[111] / 10000.0);
    can1_ams_s11_voltages_2.s11v08 = can1_ams_s11_voltages_2_s11v08_encode((float) rawVoltages[112] / 10000.0);
    can1_ams_s11_voltages_2.s11v09 = can1_ams_s11_voltages_2_s11v09_encode((float) rawVoltages[113] / 10000.0);
    can1_ams_s11_voltages_2.s11v10 = can1_ams_s11_voltages_2_s11v10_encode((float) rawVoltages[114] / 10000.0);
    can1_ams_s11_voltages_2.s11v11 = can1_ams_s11_voltages_2_s11v11_encode((float) rawVoltages[115] / 10000.0);

    can1_ams_s11_voltages_2_pack(txData, &can1_ams_s11_voltages_2, CAN1_AMS_S11_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S11_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S11_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 22:
    can1_ams_s12_voltages_1.s12v01 = can1_ams_s12_voltages_1_s12v01_encode((float) rawVoltages[116] / 10000.0);
    can1_ams_s12_voltages_1.s12v02 = can1_ams_s12_voltages_1_s12v02_encode((float) rawVoltages[117] / 10000.0);
    can1_ams_s12_voltages_1.s12v03 = can1_ams_s12_voltages_1_s12v03_encode((float) rawVoltages[118] / 10000.0);
    can1_ams_s12_voltages_1.s12v04 = can1_ams_s12_voltages_1_s12v04_encode((float) rawVoltages[119] / 10000.0);
    can1_ams_s12_voltages_1.s12v05 = can1_ams_s12_voltages_1_s12v05_encode((float) rawVoltages[120] / 10000.0);
    can1_ams_s12_voltages_1.s12v06 = can1_ams_s12_voltages_1_s12v06_encode((float) rawVoltages[121] / 10000.0);

    can1_ams_s12_voltages_1_pack(txData, &can1_ams_s12_voltages_1, CAN1_AMS_S12_VOLTAGES_1_LENGTH);
    txHeader.StdId = CAN1_AMS_S12_VOLTAGES_1_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S12_VOLTAGES_1_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 23:
    can1_ams_s12_voltages_2.s12v07 = can1_ams_s12_voltages_2_s12v07_encode((float) rawVoltages[122] / 10000.0);
    can1_ams_s12_voltages_2.s12v08 = can1_ams_s12_voltages_2_s12v08_encode((float) rawVoltages[123] / 10000.0);
    can1_ams_s12_voltages_2.s12v09 = can1_ams_s12_voltages_2_s12v09_encode((float) rawVoltages[124] / 10000.0);
    can1_ams_s12_voltages_2.s12v10 = can1_ams_s12_voltages_2_s12v10_encode((float) rawVoltages[125] / 10000.0);

    can1_ams_s12_voltages_2_pack(txData, &can1_ams_s12_voltages_2, CAN1_AMS_S12_VOLTAGES_2_LENGTH);
    txHeader.StdId = CAN1_AMS_S12_VOLTAGES_2_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S12_VOLTAGES_2_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;
  }
}

static void temperatureSendCan(void)
{
  switch (amsTxMessageCounter % 12)
  {
  case 0:
    can1_ams_s01_temperatures.s01t01 = can1_ams_s01_temperatures_s01t01_encode(
        thermistor_adc_to_c_float(rawTemps[0]));
    can1_ams_s01_temperatures.s01t02 = can1_ams_s01_temperatures_s01t02_encode(
        thermistor_adc_to_c_float(rawTemps[1]));
    can1_ams_s01_temperatures.s01t03 = can1_ams_s01_temperatures_s01t03_encode(
        thermistor_adc_to_c_float(rawTemps[2]));
    can1_ams_s01_temperatures.s01t04 = can1_ams_s01_temperatures_s01t04_encode(
        thermistor_adc_to_c_float(rawTemps[3]));
    can1_ams_s01_temperatures.s01t05 = can1_ams_s01_temperatures_s01t05_encode(
        thermistor_adc_to_c_float(rawTemps[4]));

    can1_ams_s01_temperatures_pack(txData, &can1_ams_s01_temperatures, CAN1_AMS_S01_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S01_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S01_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 1:
    can1_ams_s02_temperatures.s02t01 = can1_ams_s02_temperatures_s02t01_encode(
        thermistor_adc_to_c_float(rawTemps[5]));
    can1_ams_s02_temperatures.s02t02 = can1_ams_s02_temperatures_s02t02_encode(
        thermistor_adc_to_c_float(rawTemps[6]));
    can1_ams_s02_temperatures.s02t03 = can1_ams_s02_temperatures_s02t03_encode(
        thermistor_adc_to_c_float(rawTemps[7]));
    can1_ams_s02_temperatures.s02t04 = can1_ams_s02_temperatures_s02t04_encode(
        thermistor_adc_to_c_float(rawTemps[8]));
    can1_ams_s02_temperatures.s02t05 = can1_ams_s02_temperatures_s02t05_encode(
        thermistor_adc_to_c_float(rawTemps[9]));

    can1_ams_s02_temperatures_pack(txData, &can1_ams_s02_temperatures, CAN1_AMS_S02_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S02_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S02_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 2:
    can1_ams_s03_temperatures.s03t01 = can1_ams_s03_temperatures_s03t01_encode(
        thermistor_adc_to_c_float(rawTemps[10]));
    can1_ams_s03_temperatures.s03t02 = can1_ams_s03_temperatures_s03t02_encode(
        thermistor_adc_to_c_float(rawTemps[11]));
    can1_ams_s03_temperatures.s03t03 = can1_ams_s03_temperatures_s03t03_encode(
        thermistor_adc_to_c_float(rawTemps[12]));
    can1_ams_s03_temperatures.s03t04 = can1_ams_s03_temperatures_s03t04_encode(
        thermistor_adc_to_c_float(rawTemps[13]));
    can1_ams_s03_temperatures.s03t05 = can1_ams_s03_temperatures_s03t05_encode(
        thermistor_adc_to_c_float(rawTemps[14]));

    can1_ams_s03_temperatures_pack(txData, &can1_ams_s03_temperatures, CAN1_AMS_S03_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S03_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S03_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 3:
    can1_ams_s04_temperatures.s04t01 = can1_ams_s04_temperatures_s04t01_encode(
        thermistor_adc_to_c_float(rawTemps[15]));
    can1_ams_s04_temperatures.s04t02 = can1_ams_s04_temperatures_s04t02_encode(
        thermistor_adc_to_c_float(rawTemps[16]));
    can1_ams_s04_temperatures.s04t03 = can1_ams_s04_temperatures_s04t03_encode(
        thermistor_adc_to_c_float(rawTemps[17]));
    can1_ams_s04_temperatures.s04t04 = can1_ams_s04_temperatures_s04t04_encode(
        thermistor_adc_to_c_float(rawTemps[18]));
    can1_ams_s04_temperatures.s04t05 = can1_ams_s04_temperatures_s04t05_encode(
        thermistor_adc_to_c_float(rawTemps[19]));

    can1_ams_s04_temperatures_pack(txData, &can1_ams_s04_temperatures, CAN1_AMS_S04_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S04_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S04_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 4:
    can1_ams_s05_temperatures.s05t01 = can1_ams_s05_temperatures_s05t01_encode(
        thermistor_adc_to_c_float(rawTemps[20]));
    can1_ams_s05_temperatures.s05t02 = can1_ams_s05_temperatures_s05t02_encode(
        thermistor_adc_to_c_float(rawTemps[21]));
    can1_ams_s05_temperatures.s05t03 = can1_ams_s05_temperatures_s05t03_encode(
        thermistor_adc_to_c_float(rawTemps[22]));
    can1_ams_s05_temperatures.s05t04 = can1_ams_s05_temperatures_s05t04_encode(
        thermistor_adc_to_c_float(rawTemps[23]));
    can1_ams_s05_temperatures.s05t05 = can1_ams_s05_temperatures_s05t05_encode(
        thermistor_adc_to_c_float(rawTemps[24]));

    can1_ams_s05_temperatures_pack(txData, &can1_ams_s05_temperatures, CAN1_AMS_S05_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S05_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S05_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 5:
    can1_ams_s06_temperatures.s06t01 = can1_ams_s06_temperatures_s06t01_encode(
        thermistor_adc_to_c_float(rawTemps[25]));
    can1_ams_s06_temperatures.s06t02 = can1_ams_s06_temperatures_s06t02_encode(
        thermistor_adc_to_c_float(rawTemps[26]));
    can1_ams_s06_temperatures.s06t03 = can1_ams_s06_temperatures_s06t03_encode(
        thermistor_adc_to_c_float(rawTemps[27]));
    can1_ams_s06_temperatures.s06t04 = can1_ams_s06_temperatures_s06t04_encode(
        thermistor_adc_to_c_float(rawTemps[28]));
    can1_ams_s06_temperatures.s06t05 = can1_ams_s06_temperatures_s06t05_encode(
        thermistor_adc_to_c_float(rawTemps[29]));

    can1_ams_s06_temperatures_pack(txData, &can1_ams_s06_temperatures, CAN1_AMS_S06_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S06_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S06_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 6:
    can1_ams_s07_temperatures.s07t01 = can1_ams_s07_temperatures_s07t01_encode(
        thermistor_adc_to_c_float(rawTemps[30]));
    can1_ams_s07_temperatures.s07t02 = can1_ams_s07_temperatures_s07t02_encode(
        thermistor_adc_to_c_float(rawTemps[31]));
    can1_ams_s07_temperatures.s07t03 = can1_ams_s07_temperatures_s07t03_encode(
        thermistor_adc_to_c_float(rawTemps[32]));
    can1_ams_s07_temperatures.s07t04 = can1_ams_s07_temperatures_s07t04_encode(
        thermistor_adc_to_c_float(rawTemps[33]));
    can1_ams_s07_temperatures.s07t05 = can1_ams_s07_temperatures_s07t05_encode(
        thermistor_adc_to_c_float(rawTemps[34]));

    can1_ams_s07_temperatures_pack(txData, &can1_ams_s07_temperatures, CAN1_AMS_S07_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S07_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S07_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 7:
    can1_ams_s08_temperatures.s08t01 = can1_ams_s08_temperatures_s08t01_encode(
        thermistor_adc_to_c_float(rawTemps[35]));
    can1_ams_s08_temperatures.s08t02 = can1_ams_s08_temperatures_s08t02_encode(
        thermistor_adc_to_c_float(rawTemps[36]));
    can1_ams_s08_temperatures.s08t03 = can1_ams_s08_temperatures_s08t03_encode(
        thermistor_adc_to_c_float(rawTemps[37]));
    can1_ams_s08_temperatures.s08t04 = can1_ams_s08_temperatures_s08t04_encode(
        thermistor_adc_to_c_float(rawTemps[38]));
    can1_ams_s08_temperatures.s08t05 = can1_ams_s08_temperatures_s08t05_encode(
        thermistor_adc_to_c_float(rawTemps[39]));

    can1_ams_s08_temperatures_pack(txData, &can1_ams_s08_temperatures, CAN1_AMS_S08_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S08_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S08_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 8:
    can1_ams_s09_temperatures.s09t01 = can1_ams_s09_temperatures_s09t01_encode(
        thermistor_adc_to_c_float(rawTemps[40]));
    can1_ams_s09_temperatures.s09t02 = can1_ams_s09_temperatures_s09t02_encode(
        thermistor_adc_to_c_float(rawTemps[41]));
    can1_ams_s09_temperatures.s09t03 = can1_ams_s09_temperatures_s09t03_encode(
        thermistor_adc_to_c_float(rawTemps[42]));
    can1_ams_s09_temperatures.s09t04 = can1_ams_s09_temperatures_s09t04_encode(
        thermistor_adc_to_c_float(rawTemps[43]));
    can1_ams_s09_temperatures.s09t05 = can1_ams_s09_temperatures_s09t05_encode(
        thermistor_adc_to_c_float(rawTemps[44]));

    can1_ams_s09_temperatures_pack(txData, &can1_ams_s09_temperatures, CAN1_AMS_S09_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S09_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S09_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 9:
    can1_ams_s10_temperatures.s10t01 = can1_ams_s10_temperatures_s10t01_encode(
        thermistor_adc_to_c_float(rawTemps[45]));
    can1_ams_s10_temperatures.s10t02 = can1_ams_s10_temperatures_s10t02_encode(
        thermistor_adc_to_c_float(rawTemps[46]));
    can1_ams_s10_temperatures.s10t03 = can1_ams_s10_temperatures_s10t03_encode(
        thermistor_adc_to_c_float(rawTemps[47]));
    can1_ams_s10_temperatures.s10t04 = can1_ams_s10_temperatures_s10t04_encode(
        thermistor_adc_to_c_float(rawTemps[48]));
    can1_ams_s10_temperatures.s10t05 = can1_ams_s10_temperatures_s10t05_encode(
        thermistor_adc_to_c_float(rawTemps[49]));

    can1_ams_s10_temperatures_pack(txData, &can1_ams_s10_temperatures, CAN1_AMS_S10_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S10_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S10_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 10:
    can1_ams_s11_temperatures.s11t01 = can1_ams_s11_temperatures_s11t01_encode(
        thermistor_adc_to_c_float(rawTemps[50]));
    can1_ams_s11_temperatures.s11t02 = can1_ams_s11_temperatures_s11t02_encode(
        thermistor_adc_to_c_float(rawTemps[51]));
    can1_ams_s11_temperatures.s11t03 = can1_ams_s11_temperatures_s11t03_encode(
        thermistor_adc_to_c_float(rawTemps[52]));
    can1_ams_s11_temperatures.s11t04 = can1_ams_s11_temperatures_s11t04_encode(
        thermistor_adc_to_c_float(rawTemps[53]));
    can1_ams_s11_temperatures.s11t05 = can1_ams_s11_temperatures_s11t05_encode(
        thermistor_adc_to_c_float(rawTemps[54]));

    can1_ams_s11_temperatures_pack(txData, &can1_ams_s11_temperatures, CAN1_AMS_S11_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S11_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S11_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;

  case 11:
    can1_ams_s12_temperatures.s12t01 = can1_ams_s12_temperatures_s12t01_encode(
        thermistor_adc_to_c_float(rawTemps[55]));
    can1_ams_s12_temperatures.s12t02 = can1_ams_s12_temperatures_s12t02_encode(
        thermistor_adc_to_c_float(rawTemps[56]));
    can1_ams_s12_temperatures.s12t03 = can1_ams_s12_temperatures_s12t03_encode(
        thermistor_adc_to_c_float(rawTemps[57]));
    can1_ams_s12_temperatures.s12t04 = can1_ams_s12_temperatures_s12t04_encode(
        thermistor_adc_to_c_float(rawTemps[58]));
    can1_ams_s12_temperatures.s12t05 = can1_ams_s12_temperatures_s12t05_encode(
        thermistor_adc_to_c_float(rawTemps[59]));

    can1_ams_s12_temperatures_pack(txData, &can1_ams_s12_temperatures, CAN1_AMS_S12_TEMPERATURES_LENGTH);
    txHeader.StdId = CAN1_AMS_S12_TEMPERATURES_FRAME_ID;
    txHeader.DLC = CAN1_AMS_S12_TEMPERATURES_LENGTH;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
      Error_Handler();
    break;
  }
}

static void readCanMessages(void)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    {
      Error_Handler();
    }

    // Process the received message based on its ID
    switch (rxHeader.StdId)
    {
    case CAN1_DBU_STATUS_1_FRAME_ID:
      can1_dbu_status_1_unpack(&can1_dbu_status_1, rxData, CAN1_DBU_STATUS_1_LENGTH);
      break;
    case CAN1_ECU_STATUS_FRAME_ID:
      can1_ecu_status_unpack(&can1_ecu_status, rxData, CAN1_ECU_STATUS_LENGTH);
      break;
    case CAN1_AMS_PARAMETERS_SET_FRAME_ID:
      can1_ams_parameters_set_unpack(&can1_ams_parameters_set, rxData, CAN1_AMS_PARAMETERS_SET_LENGTH);
      break;
    default:
      // Unknown ID, handle if necessary
      break;
    }
  }

  // Read CAN2
  while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0)
  {
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    {
      Error_Handler();
    }

    // Process the received message based on its ID
    switch (rxHeader.StdId)
    {
    case CAN2_IVT_MSG_RESULT_U2_FRAME_ID:
      can2_ivt_msg_result_u2_unpack(&can2_ivt_msg_result_u2, rxData, CAN2_IVT_MSG_RESULT_U2_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_U1_FRAME_ID:
      can2_ivt_msg_result_u1_unpack(&can2_ivt_msg_result_u1, rxData, CAN2_IVT_MSG_RESULT_U1_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_T_FRAME_ID:
      can2_ivt_msg_result_t_unpack(&can2_ivt_msg_result_t, rxData, CAN2_IVT_MSG_RESULT_T_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_W_FRAME_ID:
      can2_ivt_msg_result_w_unpack(&can2_ivt_msg_result_w, rxData, CAN2_IVT_MSG_RESULT_W_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_WH_FRAME_ID:
      can2_ivt_msg_result_wh_unpack(&can2_ivt_msg_result_wh, rxData, CAN2_IVT_MSG_RESULT_WH_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_I_FRAME_ID:
      can2_ivt_msg_result_i_unpack(&can2_ivt_msg_result_i, rxData, CAN2_IVT_MSG_RESULT_I_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_U3_FRAME_ID:
      can2_ivt_msg_result_u3_unpack(&can2_ivt_msg_result_u3, rxData, CAN2_IVT_MSG_RESULT_U3_LENGTH);
      break;
    case CAN2_IVT_MSG_RESULT_AS_FRAME_ID:
      can2_ivt_msg_result_as_unpack(&can2_ivt_msg_result_as, rxData, CAN2_IVT_MSG_RESULT_AS_LENGTH);
      break;
    default:
      // Unknown ID, handle if necessary
      break;
    }
  }
}

static void sendStatus(void)
{
  uint16_t maxVoltage = 0;
  uint16_t minVoltage = 0xFFFF;
  uint16_t maxTemperature = 0;
  uint16_t minTemperature = 0xFFFF;
  uint8_t maxOvp = 0;
  uint8_t maxUvp = 0;
  uint8_t maxOtp = 0;
  uint8_t maxUtp = 0;
  uint8_t maxIsospiErrorCounter = 0;
  for (int i = 0; i < 126; i++)
  {
    if (rawVoltages[i] > maxVoltage)
      maxVoltage = rawVoltages[i];
    if (rawVoltages[i] < minVoltage)
      minVoltage = rawVoltages[i];
    if (ovpCounter[i] > maxOvp)
      maxOvp = ovpCounter[i];
    if (uvpCounter[i] > maxUvp)
      maxUvp = uvpCounter[i];
    if (otpCounter[i] > maxOtp)
      maxOtp = otpCounter[i];
    if (utpCounter[i] > maxUtp)
      maxUtp = utpCounter[i];
  }
  for (int i = 0; i < 24; i++)
  {
    if (spiVoltageErrorCounter[i] > maxIsospiErrorCounter)
      maxIsospiErrorCounter = spiVoltageErrorCounter[i];
  }
  for (int i = 0; i < 12; i++)
  {
    if (spiTemperatureErrorCounter[i] > maxIsospiErrorCounter)
      maxIsospiErrorCounter = spiTemperatureErrorCounter[i];
  }
  for (int i = 0; i < 12; i++)
  {
    if (rawTemps[i] > maxTemperature)
      maxTemperature = rawTemps[i];
    if (rawTemps[i] < minTemperature)
      minTemperature = rawTemps[i];
  }

  can1_ams_status_1.max_cell_voltage = can1_ams_status_1_max_cell_voltage_encode((float) maxVoltage / 10000.0);
  can1_ams_status_1.min_cell_voltage = can1_ams_status_1_min_cell_voltage_encode((float) minVoltage / 10000.0);
  can1_ams_status_1.max_cell_temperature = can1_ams_status_1_max_cell_temperature_encode(
      thermistor_adc_to_c_float(maxTemperature));
  can1_ams_status_1.min_cell_temperature = can1_ams_status_1_min_cell_temperature_encode(
      thermistor_adc_to_c_float(minTemperature));
  can1_ams_status_1.fsm_state = fsmState;
  can1_ams_status_1.ticks = can1_ams_status_1_ticks_encode(deltaTick);
  can1_ams_status_1_pack(txData, &can1_ams_status_1, CAN1_AMS_STATUS_1_LENGTH);
  txHeader.StdId = CAN1_AMS_STATUS_1_FRAME_ID;
  txHeader.DLC = CAN1_AMS_STATUS_1_LENGTH;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
    Error_Handler();

  can1_ams_status_2.isospi_rx_error_rate = can1_ams_status_2_isospi_rx_error_rate_encode(
      (float) isospiRxErrorCount / (float) isospiRxCount);
  can1_ams_status_2.isospi_tx_error_rate = 0xFF; // not implemented yet
  can1_ams_status_2.max_otp_counter = can1_ams_status_2_max_otp_counter_encode(maxOtp);
  can1_ams_status_2.max_uvp_counter = can1_ams_status_2_max_uvp_counter_encode(maxUvp);
  can1_ams_status_2.max_ovp_counter = can1_ams_status_2_max_ovp_counter_encode(maxOvp);
  can1_ams_status_2.max_utp_counter = can1_ams_status_2_max_utp_counter_encode(maxUtp);
  can1_ams_status_2.max_isospi_error_counter = can1_ams_status_2_max_isospi_error_counter_encode(maxIsospiErrorCounter);
  can1_ams_status_2_pack(txData, &can1_ams_status_2, CAN1_AMS_STATUS_2_LENGTH);
  txHeader.StdId = CAN1_AMS_STATUS_2_FRAME_ID;
  txHeader.DLC = CAN1_AMS_STATUS_2_LENGTH;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, NULL) != HAL_OK)
    Error_Handler();
}

static void stepStateMachine(void)
{
  switch (fsmState)
  {
  case STATE_IDLE:
    HAL_GPIO_WritePin(AIR_N_ENABLE_GPIO_Port, AIR_N_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIR_P_ENABLE_GPIO_Port, AIR_P_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PRECHARGE_ENABLE_GPIO_Port, PRECHARGE_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MCU_AMS_ERROR_N_GPIO_Port, MCU_AMS_ERROR_N_Pin, GPIO_PIN_SET);
    if (ovpError || uvpError || otpError || utpError || spiError)
    {
      fsmState = STATE_ERROR;
    }
    else if (   (HAL_GPIO_ReadPin(SC_PROBE_GPIO_Port, SC_PROBE_Pin) == GPIO_PIN_SET)
             && can1_dbu_status_1.activate_ts_button)
    {
      fsmState = STATE_PRECHARGE1;
    }
    break;

  case STATE_PRECHARGE1:
    HAL_GPIO_WritePin(AIR_N_ENABLE_GPIO_Port, AIR_N_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIR_P_ENABLE_GPIO_Port, AIR_P_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PRECHARGE_ENABLE_GPIO_Port, PRECHARGE_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MCU_AMS_ERROR_N_GPIO_Port, MCU_AMS_ERROR_N_Pin, GPIO_PIN_SET);
    if (ovpError || uvpError || otpError || utpError || spiError)
    {
      fsmState = STATE_ERROR;
    }
    else if (HAL_GPIO_ReadPin(PRECHARGE_CLOSED_SIGNAL_GPIO_Port, PRECHARGE_CLOSED_SIGNAL_Pin) == GPIO_PIN_SET)
    {
      fsmState = STATE_PRECHARGE2;
      prechargeStartTime = HAL_GetTick();
    }
    break;

  case STATE_PRECHARGE2:
    HAL_GPIO_WritePin(AIR_N_ENABLE_GPIO_Port, AIR_N_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIR_P_ENABLE_GPIO_Port, AIR_P_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PRECHARGE_ENABLE_GPIO_Port, PRECHARGE_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MCU_AMS_ERROR_N_GPIO_Port, MCU_AMS_ERROR_N_Pin, GPIO_PIN_SET);
    prechargeTimer = HAL_GetTick() - prechargeStartTime;
    if (ovpError || uvpError || otpError || utpError || spiError)
    {
      fsmState = STATE_ERROR;
    }
    else if (   (HAL_GPIO_ReadPin(PRECHARGE_CLOSED_SIGNAL_GPIO_Port, PRECHARGE_CLOSED_SIGNAL_Pin) == GPIO_PIN_SET)
             && (HAL_GPIO_ReadPin(AIR_N_CLOSED_GPIO_Port, AIR_N_CLOSED_Pin) == GPIO_PIN_SET)
             && (can2_ivt_msg_result_u1_ivt_result_u1_decode(can2_ivt_msg_result_u1.ivt_result_u1) >
                 0.95 * can2_ivt_msg_result_u2_ivt_result_u2_decode(can2_ivt_msg_result_u2.ivt_result_u2))
             && (prechargeTimer > MIN_PRECHARGE_TIME))
    {
      fsmState = STATE_TS_ON;
    }
    else if (prechargeTimer > PRECHARGE_TIMEOUT)
    {
      fsmState = STATE_IDLE;
    }
    break;

  case STATE_TS_ON:
    HAL_GPIO_WritePin(AIR_N_ENABLE_GPIO_Port, AIR_N_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIR_P_ENABLE_GPIO_Port, AIR_P_ENABLE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PRECHARGE_ENABLE_GPIO_Port, PRECHARGE_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MCU_AMS_ERROR_N_GPIO_Port, MCU_AMS_ERROR_N_Pin, GPIO_PIN_SET);
    if (ovpError || uvpError || otpError || utpError || spiError)
    {
      fsmState = STATE_ERROR;
    }
    else if (HAL_GPIO_ReadPin(SC_PROBE_GPIO_Port, SC_PROBE_Pin) == GPIO_PIN_RESET)
    {
      fsmState = STATE_IDLE;
    }
    break;

  case STATE_ERROR:
    HAL_GPIO_WritePin(AIR_N_ENABLE_GPIO_Port, AIR_N_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIR_P_ENABLE_GPIO_Port, AIR_P_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PRECHARGE_ENABLE_GPIO_Port, PRECHARGE_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MCU_AMS_ERROR_N_GPIO_Port, MCU_AMS_ERROR_N_Pin, GPIO_PIN_RESET);
    break;
  }
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
#ifdef USE_FULL_ASSERT
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
