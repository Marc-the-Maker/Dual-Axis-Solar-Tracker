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
#include <math.h>
#include <stdbool.h>
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_BUF_PANEL_LEN 2400
#define MIN_AZI_DELAY 15
#define MIN_VERT_DELAY 2 //Multiple of MIN_AZI_DELAY - Delay = MIN_AZI_DELAY * MIN_VERT_DELAY
#define MIN_VERTICAL_TRACKING_ANGLE 10
#define PI 3.14159265359



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t tx_buff[19];

//Panel V&I + Power Variables

double panel_Current_ADC = 0.0;
double actual_Panel_Current = 0.0;

double panel_Voltage_ADC = 0.0;
double actual_Panel_Voltage = 0.0;

double actual_Panel_Power = 0.0;

//Battery V&I + Power Variables

double battery_Current_ADC = 0.0;
double actual_Battery_Current = 0.0;

double battery_Voltage_ADC = 0.0;
double actual_Battery_Voltage = 0.0;

double actual_Battery_Power = 0.0;

//MPPT Variables

double prev_Panel_Voltage = 0.0;
double prev_Panel_Power = 20;

double delta_Voltage = 0.0;
double delta_Power =0.0;

double delta_Step = 0.0;

//CCCV Variables
double PWM_Control_Action = 0.0;

double voltage_Setpoint = 12.0;
double Kc_Voltage = 0.05;
double tau_i_Voltage = 0.006;
double error_Voltage_0 = 0.0;
double error_Voltage_1 = 0.0;

double current_Setpoint = 12.0;
double Kc_Current = 0.003;
double tau_i_Current = 0.003;
double error_Current_0 = 0.0;
double error_Current_1 = 0.0;


RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
uint32_t value_adc;

double value_adc_count;
double angle;
double Declination;
double Sin_Altitude_Angle;
double Altitude_Angle;
double Sin_Azimuth_Angle;
double Azimuth_Angle;
double H;
int16_t Day;
double Time;
double K_P_1 = 0.3;
double K_P_2 = 1;
double PWM_Duty = 0.0;
double Azi_Encoder_angle = 0.0;
double Azi_Encoder_angle_S = 0.0;
double Vert_Encoder_angle = 0.0;
double Vert_Encoder_angle_S = 0.0;

int16_t Azi_encoder_count = 0;
int16_t Vert_encoder_count = 45;
double PWM_Duty;


bool sampled = 0;

int tx_Panel_Voltage = 0;
int tx_Panel_Current = 0;
//int tx_Panel_Voltage = 0;

double current_Test = 0.0;
double test_Temp = 0.0;
int adc_value = 0;

int state_variable = 0;
double y_t = 0.0;
double prev_y = 0.0;
double alpha = 0.3;

double test_Voltage_Sensor = 0.0;

//Current & Voltage sensor testing

volatile uint16_t adc_buf_panel[ADC_BUF_PANEL_LEN];
double alpha_ema = 0.6;




volatile int panel_sensor_count = 0;
volatile int32_t current_Running_Total = 0;
volatile int32_t voltage_Running_Total = 0;
volatile int32_t bat_current_Running_Total = 0;
volatile int32_t bat_voltage_Running_Total = 0;

volatile int32_t voltage_Running_Bat_Total = 0;
volatile int32_t current_Running_Bat_Total = 0;
//double temp = 0;

//Limit switches

bool azi_Limit_Switch = 0;
bool vert_Limit_Switch = 0;
bool azi_Limit_Switch_2 = 0;
bool vert_Limit_Switch_2 = 0;

//LCD Variables
#define SLAVE_ADDRESS_LCD 0x3F
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

#define LCD_ROWS 4 // Number of rows on the LCD
#define LCD_COLS 20 // Number of columns on the LCD

uint8_t backlight_state = 1;

char *temp;

double vertical_SUN;

double Sun_Azi_Current = 0.0;
double Panel_Azi_Current = 0.0;
double Sun_Vert_Current = 0.0;
double Panel_Vert_Current = 0.0;

//PSA Solar tracker

int delta = 0;
int leap = 0;
double jd = 0.0;
double n = 0.0;
double Omega = 0.0;
double mnlong = 0.0;
double mnanom = 0.0;
double eclong = 0.0;
double oblqec = 0.0;

double num = 0.0;
double den = 0.0;
double ra = 0.0;
double dec = 0.0;
double gmst = 0.0;
double lmst= 0.0;
double ha = 0.0;
double az = 0.0;
double el = 0.0;

//States | 0 - wait | 1 - MPPT | 2 - CCCV | 3 - Azimuth tracking | 4 - Vertical tracking | 5 - Sleep


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */


double Calculate_Azimuth_Sun();
double Calculate_Panel_New_Encoder();
double Calculate_Vertical_Sun();
double Calculate_Vertical_Panel();
void AZI_Tracker_Control();
void VERT_Tracker_Control();
void AZI_Return_Tracker();
void VERT_Return_Tracker();
void Motor_Test();
void Encoder_Test();
double Degrees_to_Radianns(double deg);
double average(int16_t arr[], int n);
double get_Current_Power();
void state_machine();
void boost_Converter_Test();
double get_Panel_Current();
double read_Pot();
void buck_Converter_Test();
void vary_PWM();

//LCD Functions

void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{
//	state_variable = 1;
//    //HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//}

//void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
//{
////	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
////	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
////	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
////	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
////	if(GPIO_Pin == AZI_Limit_Pin){
////		__HAL_TIM_SET_COUNTER(&htim3, 0);
////		azi_Limit_Switch = 1;
//////		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
////	}
////	else{
////		__HAL_TIM_SET_COUNTER(&htim2, 0);
////		vert_Limit_Switch = 1;
////	}
//}



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
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  lcd_init() ;
  lcd_clear();
  HAL_Delay(2);







  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint16_t*)&adc_buf_panel,ADC_BUF_PANEL_LEN);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc2);


//  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
//			  Error_Handler();

// start pwm generation
//  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
//			  Error_Handler();

  HAL_TIM_Base_Start_IT(&htim6);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Azi_encoder_count = 0;
  actual_Panel_Voltage = 0.0;
  actual_Battery_Voltage = 0.0;

  if(HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
	  Error_Handler();
  }



	int temp_int = 1234;


//	for(int i = 0; i < 16; i+=2){
//	  tx_buff[i] = (temp_int>>8) & 0xFF;
//	  tx_buff[i+1] = (temp_int) & 0xFF;
//	}
//
//	tx_buff[16] = 12;
//	tx_buff[17] = 31;


//	QTP_VERT();


  while (1)
  {
	 //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
	 //VERT_Return_Tracker();
	 //Motor_Test();
//	  Encoder_Test();
//	  azi_Limit_Switch = 0;
//	  vert_Limit_Switch = 0;
//	  test_Dual_Axis_Tracking();
//	  test_Dual_Axis_Tracking();
	 //Inc_Cond();
//	 Inc_Cond_2();
	  boost_Set_PWM();
//	 vary_PWM();
//	 P_and_O_2();
//	 P_and_O();
	 //current_Test = read_Pot();
//	  state_machine();
//	  Encoder_Test();
//	  boost_Converter_Test();

//	  test_Dual_Axis_Tracking();
//	  Motor_Test();

//	 buck_Converter_Test()?;
//	 P_and_O_2();
//	 current_Test = get_Panel_Current();
//	 HAL_Delay(10);
//	 vertical_SUN = Calculate_Vertical_Sun();

	 //double test = get_Current_Power();
	// HAL_Delay(1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x10;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 0x2;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 15;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 143;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1199;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 719;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 719;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, MOT_SWITCH_Pin|LED_Pin|LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOT_SWITCH_Pin LED_Pin LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = MOT_SWITCH_Pin|LED_Pin|LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
		voltage_Running_Total = 0.0;
		current_Running_Total = 0.0;
		voltage_Running_Bat_Total = 0.0;
		current_Running_Bat_Total = 0.0;

		for(int i = 0; i<=2399 ; i += 4){
			voltage_Running_Total = voltage_Running_Total+ adc_buf_panel[i];
			current_Running_Total = current_Running_Total+ adc_buf_panel[i + 1];
			voltage_Running_Bat_Total = voltage_Running_Bat_Total+ adc_buf_panel[i + 2];
			current_Running_Bat_Total = current_Running_Bat_Total+ adc_buf_panel[i + 3];
		}
		sampled = 1;
}

double getTime(int Hours, int Minutes, int Seconds){
	double time = Seconds/(60.0*60.0);
	time = time + (Minutes/60.0);
	time = time + Hours;
	return time;
}

int getDay(int Date, int Month, int Year){
	// check for leap year
	int days_in_feb = 28;
	if( (Year % 4 == 0 && Year % 100 != 0 ) || (Year % 400 == 0) )
	{
		days_in_feb = 29;
	}
	int doy = Date;

	switch(Month)
	{
		case 2:
			doy += 31;
			break;
		case 3:
			doy += 31+days_in_feb;
			break;
		case 4:
			doy += 62+days_in_feb;
			break;
		case 5:
			doy += 92+days_in_feb;
			break;
		case 6:
			doy += 123+days_in_feb;
			break;
		case 7:
			doy += 153+days_in_feb;
			break;
		case 8:
			doy += 184+days_in_feb;
			break;
		case 9:
			doy += 215+days_in_feb;
			break;
		case 10:
			doy += 245+days_in_feb;
			break;
		case 11:
			doy += 276+days_in_feb;
			break;
		case 12:
			doy += 306+days_in_feb;
			break;
	}
	return doy;
}

double Degrees_to_Radians(double deg)
{
	return deg * (M_PI/180.0);
}

double Radians_to_Degrees(double rad)
{
	return rad * (180.0/M_PI);
}

double average(int16_t arr[], int n)
{
    // Find the sum of array element
    int sum = 0;
    for (int i = 0; i < n; i++)
        sum += arr[i];

    return (double)sum / n;
}

double Calculate_Azimuth_Panel()
{
	Azi_Encoder_angle = ((TIM3->CNT)>>2)*(34.0/42.0);
	return Azi_Encoder_angle;
}

double Calculate_Vertical_Panel()
{
	Vert_Encoder_angle = ((TIM2->CNT)>>2)*(1.0/3.0);
	Vert_Encoder_angle = Vert_Encoder_angle + 25.0;
	return Vert_Encoder_angle;
}

//double Calculate_Vertical_Panel()
//{
//	Vert_Encoder_angle = Vert_encoder_count * (1.0/3.0);
//	Vert_Encoder_angle = (90.0 - Vert_Encoder_angle);
//	return Vert_Encoder_angle;
//}
double PSA_Algorithm_Vert(){
	double Latitude = Degrees_to_Radians(-25.754208);
	double Longitude = 28.228351;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	Time = getTime(sTime.Hours, sTime.Minutes, sTime.Seconds);
	Time = Time - 2;
	Day = getDay(sDate.Date, sDate.Month, sDate.Year);

	delta = 2023 - 1949;
	leap = floor(delta/4);
	jd = 2432916.5 + (delta*365) + leap + Day + (Time/24);
	n = jd - 2451545.0;

	Omega = 2.1429 - (0.0010394594*n);

	mnlong = 4.8950630 + (0.017202791698*n);
	mnanom = 6.2400600 + (0.0172019699*n);

    eclong = mnlong + (0.03341607*sin(mnanom)) + (0.00034894*sin(2*mnanom)) - 0.0001134 - (0.0000203*sin(Omega));
    oblqec = 0.4090928 - (6.2140E-9*n) + (0.0000396*cos(Omega));

    num = cos(oblqec) * sin(eclong);
    den = cos(eclong);
    ra = atan2(num,den);

    while(ra > (2*M_PI)){
    	ra = ra - (2*M_PI);
    }
	while(ra < 0){
		ra = ra + (2*M_PI);
	}

    dec = asin(sin(oblqec)*sin(eclong));

	gmst =  6.6974243242 + (0.0657098283*n) + Time;
	lmst = ((gmst *15) + Longitude)*(PI/180);

	ha = lmst - ra;

	el = asin(((sin(dec)*sin(Latitude))+(cos(dec)*cos(Latitude)*cos(ha))));

	el = Radians_to_Degrees(el);

	return el;
}

double PSA_Algorithm_AZI(double el){
	el = Degrees_to_Radians(el);
	az = asin((cos(dec)*sin(ha))/cos(el));

	az = Radians_to_Degrees(az);
	az = az+90;

	return az;
}


double Calculate_Vertical_Sun()
{
	double Latitude = Degrees_to_Radians(-25.754208);
	double Longitude = 28.228351;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


	Time = getTime(sTime.Hours, sTime.Minutes, sTime.Seconds);
	Time = Time - (1.7 - (Longitude/15));//*****
	Day = getDay(sDate.Date, sDate.Month, sDate.Year);
	//Time = 10.5;

	H = Degrees_to_Radians(15.0*(12.05-Time));
	double Dec_Radians = Degrees_to_Radians((360.0)/(365.0)*(Day-81));
	Declination = Degrees_to_Radians(23.45*sin(Dec_Radians));
	Sin_Altitude_Angle = ((cos(Latitude))*(cos(Declination))*(cos(H)))+((sin(Latitude))*(sin(Declination)));
	Altitude_Angle = asin(Sin_Altitude_Angle);

	Altitude_Angle = Radians_to_Degrees(Altitude_Angle);
	return  Altitude_Angle;
}

double Calculate_Azimuth_Sun()
{
	double atl_Angle_deg = Calculate_Vertical_Sun();
	double atl_Angle = Degrees_to_Radians(atl_Angle_deg);
	Sin_Azimuth_Angle = ((cos(Declination))*(sin(H)))/(cos(atl_Angle));
	Azimuth_Angle = asin(Sin_Azimuth_Angle);
	Azimuth_Angle = Radians_to_Degrees(Azimuth_Angle);
	Azimuth_Angle = Azimuth_Angle - 90.0;
	Azimuth_Angle = (-1)*Azimuth_Angle;
	return  Azimuth_Angle;
}

void Motor_Test(){
	int PWM_Duty = 35;

	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	TIM16->CCR1 = round(PWM_Duty);
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
//	if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)){
//		HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
//	}

//	while((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)){
//		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
//	}
}

void Encoder_Test(){
	double azi_panel = 0.0;
	double vert_panel = 0.0;
	char buffer[10];

	temp = "PANEL_AZI = ";
	lcd_set_cursor(1, 0);
	lcd_write_string(temp);


	temp = "PANEL_VER = ";
	lcd_set_cursor(3, 0);
	lcd_write_string(temp);


	while(1)
	{
		azi_panel = Calculate_Azimuth_Panel();
		vert_panel = Calculate_Vertical_Panel();

		snprintf(buffer,5, "%f",azi_panel);
		lcd_set_cursor(1, 15);
		lcd_write_string(buffer);

		snprintf(buffer,5, "%f",vert_panel);
		lcd_set_cursor(3, 15);
		lcd_write_string(buffer);
	}
}

//void home_AZI(){
//	//Rotate AZI CW until limit switch is hit
//	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 0);
//
//	lcd_clear();
//	HAL_Delay(2);
//	temp = "HOME";
//	lcd_set_cursor(0, 0);
//	lcd_write_string(temp);
//
//	bool tracking = false;
//
//	PWM_Duty = 20.0;
//	TIM16->CCR1 = round(PWM_Duty);
//	//Check direction
//	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
//
//	while(tracking == false)
//	{
//		if(azi_Limit_Switch == 1){
//			azi_Limit_Switch = 0;
//			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
//			tracking = true;
//			__HAL_TIM_SET_COUNTER(&htim3, 0);
//		}
//	}
//}

void home_VERT(){
	//Rotate VERT Down until limit switch is hit

	//Rotate AZI CW until limit switch is hit
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	bool tracking = false;

	PWM_Duty = 15.0;
	TIM16->CCR1 = round(PWM_Duty);
	//Check direction
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	while(tracking == false)
	{
		if(vert_Limit_Switch == 1){
			vert_Limit_Switch = 0;
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			tracking = true;
			__HAL_TIM_SET_COUNTER(&htim2, 0);
		}
	}
}

void AZI_Tracker_Control()
{
	bool trigger_Pressed = 0;
	double Sun_Angle = Calculate_Azimuth_Sun();
	double Sun_Vert_Angle = Calculate_Vertical_Sun();
	double Panel_Angle = Calculate_Azimuth_Panel();
	double error = Sun_Angle - Panel_Angle;
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 0);
	//Sun_Angle = 45.0;

	PWM_Duty = 0.0;
	bool tracking = false;

	//tracking = true;

	if(Sun_Vert_Angle >= MIN_VERTICAL_TRACKING_ANGLE){


		while(tracking == false)
		{
			error = Sun_Angle - Panel_Angle;

			if(error > 0.5)
			{
				PWM_Duty = 25.0 + K_P_1*fabs(error);
				if(PWM_Duty>40.0){
					PWM_Duty = 40.0;
				}
				TIM16->CCR1 = round(PWM_Duty);
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
			}
			else if(error < -0.5){
				PWM_Duty = 25.0 + K_P_1*fabs(error);
				if(PWM_Duty>40.0){
					PWM_Duty = 40.0;
				}
				TIM17->CCR1 = round(PWM_Duty);
				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
			}
			else {
				PWM_Duty = 0;
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
				tracking = true;
			}

			Panel_Angle = Calculate_Azimuth_Panel();
		}
	}
	else{
		home_AZI();
		HAL_Delay(2000);
		home_VERT();
		HAL_Delay(2000);
		state_variable = 5;
	}

}

void AZI_Tracker_Control_Test(){

	char buffer[10];
	double Sun_Vert_Angle = PSA_Algorithm_Vert();
	double Sun_Angle = PSA_Algorithm_AZI(Sun_Vert_Angle);
	double Panel_Angle = Calculate_Azimuth_Panel();

	Sun_Vert_Current = Sun_Vert_Angle;
	Sun_Azi_Current = Sun_Angle;

	double error = Sun_Angle - Panel_Angle;
	bool trigger_Pressed = 0;
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 0);
	//Sun_Angle = 45.0;


	temp = "AZI Tracker";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);
//
	snprintf(buffer,5, "%f",Panel_Angle);
	lcd_set_cursor(1, 15);
	lcd_write_string(buffer);

	snprintf(buffer,5, "%f",Sun_Angle);
	lcd_set_cursor(2, 15);
	lcd_write_string(buffer);

	PWM_Duty = 0.0;
	bool tracking = false;

	//tracking = true;

//	HAL_Delay(3000);

	if((Sun_Vert_Angle >= 5.0)){
//		trigger_Pressed = HAL_GPIO_ReadPin(AZI_Limit_GPIO_Port, AZI_Limit_Pin);
		while(tracking == false)
		{
//			if(trigger_Pressed == 1){
//				__HAL_TIM_SET_COUNTER(&htim3, 0);
//				TIM17->CCR1 = 30;
//				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
//				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
//				HAL_Delay(100);
//				azi_Limit_Switch = 0;
//				trigger_Pressed = HAL_GPIO_ReadPin(AZI_Limit_GPIO_Port, AZI_Limit_Pin);
//			}
			if(azi_Limit_Switch == 1){
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
				tracking = true;
//				lcd_clear();
//				HAL_Delay(2);
//				temp = "AZI LIMIT";
//				lcd_set_cursor(0, 0);
//				lcd_write_string(temp);
				azi_Limit_Switch = 0;
				break;
			}
			else{

				error = Sun_Angle - Panel_Angle;

				if(error > 0.0)
				{
					PWM_Duty = 29.0 + K_P_1*fabs(error);
					if(PWM_Duty>35.0){
						PWM_Duty = 35.0;
					}
					TIM17->CCR1 = round(PWM_Duty);
					HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
				}
				else if(error < -2.0){
					PWM_Duty = 29.0 + K_P_1*fabs(error);
					if(PWM_Duty>35.0){
						PWM_Duty = 35.0;
					}
					TIM16->CCR1 = round(PWM_Duty);
					HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
				}
				else {
					PWM_Duty = 0;
					HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
					tracking = true;
				}

//				Panel_Angle = Calculate_Azimuth_Panel();
//
//				snprintf(buffer,5, "%f",Panel_Angle);
//				lcd_set_cursor(1, 15);
//				lcd_write_string(buffer);
			}
			Panel_Angle = Calculate_Azimuth_Panel();
			Panel_Azi_Current = Panel_Angle;

			//Panel_Angle = Calculate_Azimuth_Panel();
			//
			snprintf(buffer,5, "%f",Panel_Angle);
			lcd_set_cursor(1, 15);
			lcd_write_string(buffer);
}
	}
//	else{
//		trigger_Pressed = HAL_GPIO_ReadPin(AZI_Limit_GPIO_Port, AZI_Limit_Pin);
//		if(trigger_Pressed != 1){
//			home_AZI();
//			HAL_Delay(2000);
//			home_VERT();
//			HAL_Delay(2000);
//		}
//		state_variable = 6;//Need to make sure it doesnt overwrite

//	}

}

void QTP_AZI(){
	HAL_Delay(2000);
	char buffer[10];
	double Sun_Angle = 30.0;
	double Panel_Angle = Calculate_Azimuth_Panel();


	double error = Sun_Angle - Panel_Angle;
	bool trigger_Pressed = 0;
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 0);
	//Sun_Angle = 45.0;


	temp = "AZI Tracker";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	snprintf(buffer,5, "%f",Sun_Angle);
	lcd_set_cursor(1, 0);
	lcd_write_string(buffer);


	PWM_Duty = 0.0;
	bool tracking = false;

	while(tracking == false)
	{
		error = Sun_Angle - Panel_Angle;
		if(error > 1.0)
		{
			PWM_Duty = 25.0 + K_P_1*fabs(error);
			if(PWM_Duty>30.0){
				PWM_Duty = 30.0;
			}
			TIM17->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
		}
		else if(error < -1.0){
			PWM_Duty = 25.0 + K_P_1*fabs(error);
			if(PWM_Duty>30.0){
				PWM_Duty = 30.0;
			}
			TIM16->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		}
		else {
			PWM_Duty = 0;
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			tracking = true;
		}
		Panel_Angle = Calculate_Azimuth_Panel();
		Panel_Azi_Current = Panel_Angle;

		snprintf(buffer,5, "%f",Panel_Angle);
		lcd_set_cursor(2, 0);
		lcd_write_string(buffer);

	}
}

void VERT_Tracker_Control()
{
	double Sun_Angle = Calculate_Vertical_Sun();
	double Panel_Angle = Calculate_Vertical_Panel();
	//Sun_Angle = 45.0;


	PWM_Duty = 25.0;
	bool vert_tracking = false;
	//vert_tracking = true;
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	while(vert_tracking == false)
	{
		double error = Sun_Angle - Panel_Angle;

		if(error > 1.0)
		{
			TIM17->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
		}
		else if(error < -1.0){
			TIM16->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		}
		else {
			PWM_Duty = 0;
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			vert_tracking = true;
		}

		Panel_Angle = Calculate_Vertical_Panel();
	}
}

void VERT_Tracker_Control_Test(){
	char buffer[10];
	double Sun_Angle = PSA_Algorithm_Vert();
	double Panel_Angle = Calculate_Vertical_Panel();
	Sun_Vert_Current = Sun_Angle;
	//Panel_Vert_Current = Panel_Angle;
	bool trigger_Pressed = 0;
	//Sun_Angle = 45.0;

	snprintf(buffer,5, "%f",Panel_Angle);
	lcd_set_cursor(3, 15);
	lcd_write_string(buffer);

	snprintf(buffer,5, "%f",Sun_Angle);
	lcd_set_cursor(2, 15);
	lcd_write_string(buffer);


	temp = "VERT";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	PWM_Duty = 25.0;
	bool vert_tracking = false;
	vert_Limit_Switch == 0;
	//vert_tracking = true;
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	while(vert_tracking == false)
	{
		Panel_Angle = Calculate_Vertical_Panel();
		Panel_Vert_Current = Panel_Angle;
		snprintf(buffer,5, "%f",Panel_Angle);
		lcd_set_cursor(3, 15);
		lcd_write_string(buffer);

		if((vert_Limit_Switch == 1)){
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			vert_tracking = true;
//			lcd_clear();
//			HAL_Delay(2);
//			temp = "VERT LIMIT";
//			lcd_set_cursor(0, 0);
//			lcd_write_string(temp);
			vert_Limit_Switch = 0;
			break;
		}
		else{
			double error = Sun_Angle - Panel_Angle;

			if(error > 0.8)
			{

				PWM_Duty = 22.0 + K_P_1*fabs(error);
				if(PWM_Duty>30.0){
					PWM_Duty = 30.0;
				}

				TIM17->CCR1 = round(PWM_Duty);
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
			}
			else if(error < -0.8){

				PWM_Duty = 22.0 + K_P_1*fabs(error);
				if(PWM_Duty>30.0){
					PWM_Duty = 30.0;
				}

				TIM16->CCR1 = round(PWM_Duty);
				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
			}
			else {
				PWM_Duty = 0;
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
				vert_tracking = true;
			}
		}


	}
}

void QTP_VERT(){

	char buffer[10];
	double Sun_Angle = 25 + 65.0;
	double Panel_Angle = Calculate_Vertical_Panel();


	bool trigger_Pressed = 0;


	temp = "VERT";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	snprintf(buffer,5, "%f",Sun_Angle);
	lcd_set_cursor(1, 0);
	lcd_write_string(buffer);

	PWM_Duty = 25.0;
	bool vert_tracking = false;
	vert_Limit_Switch == 0;

	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	while(vert_tracking == false)
	{
		Panel_Angle = Calculate_Vertical_Panel();
		Panel_Vert_Current = Panel_Angle;

		double error = Sun_Angle - Panel_Angle;

		if(error > 1.0)
		{

			PWM_Duty = 22.0 + K_P_1*fabs(error);
			if(PWM_Duty>30.0){
				PWM_Duty = 30.0;
			}

			TIM17->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
		}
		else if(error < -1.0){

			PWM_Duty = 22.0 + K_P_1*fabs(error);
			if(PWM_Duty>30.0){
				PWM_Duty = 30.0;
			}

			TIM16->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		}
		else {
			PWM_Duty = 0;
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			vert_tracking = true;
		}
	}
}

void VERT_Return_Tracker(){
	double Set_Point = 0;
	double Panel_Angle = Calculate_Vertical_Panel();


	PWM_Duty = 25.0;
	bool vert_tracking = false;
	//vert_tracking = true;
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	while(vert_tracking == false)
	{
		double error = Set_Point - Panel_Angle;

		if(error > 1.0)
		{
			TIM2->CCR2 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		}
		else if(error < -1.0){
			TIM2->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		}
		else {
			PWM_Duty = 0;
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			vert_tracking = true;
		}

		Panel_Angle = Calculate_Vertical_Panel();
	}
}

void AZI_Return_Tracker(){
	double Set_Point = 0.0;
	double Panel_Angle = Calculate_Panel_New_Encoder();
	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 0);

	PWM_Duty = 0.0;
	bool tracking = false;
	//tracking = true;

	while(tracking == false)
	{
		double error = Set_Point - Panel_Angle;

		if(error > 0.5)
		{
			PWM_Duty = 25.0 + K_P_1*fabs(error);
			if(PWM_Duty>40.0){
				PWM_Duty = 40.0;
			}
			TIM17->CCR2 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
		}
		else if(error < -0.5){
			PWM_Duty = 25.0 + K_P_1*fabs(error);
			if(PWM_Duty>40.0){
				PWM_Duty = 40.0;
			}
			TIM16->CCR1 = round(PWM_Duty);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
		}
		else {
			PWM_Duty = 0;
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			tracking = true;
		}

		Panel_Angle = Calculate_Panel_New_Encoder();
	}

}

void P_and_O(){
	double delta_change_Duty = 5;
	double delta_Power =0.0;
	double delta_Duty_1 = 0.0;
	double alpha = 0.3;
	double filtered_Power = 0.0;

	double temp_test = 0.0;
	double PWM_Duty = 60.0;
	double scaled_duty_ratio = 10.0;
	char buffer[10];
	char *temp = "";
	double prev_Panel_Power = 20;
	double prev_Panel_Duty = 10;
	TIM1->CCR1 = round(PWM_Duty);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	while(1){
		panel_Current_ADC = current_Running_Total/20.0;
		panel_Voltage_ADC = voltage_Running_Total/20.0;


		actual_Panel_Current = (((((panel_Current_ADC)/4095.0)*3.3)-2.35)/0.04);//
		actual_Panel_Voltage = ((((panel_Voltage_ADC/4095.0)*3.3)*(166000))/(21900));

		actual_Panel_Power = (actual_Panel_Current*actual_Panel_Voltage);

		filtered_Power = round(((actual_Panel_Power*alpha) + ((1-alpha)*(prev_Panel_Power)))*100.0)/100;

		delta_Power = filtered_Power - prev_Panel_Power;
		delta_Duty_1 = PWM_Duty - prev_Panel_Duty;



		if(delta_Power > 1){
			delta_change_Duty = 5;

			if(actual_Panel_Voltage>0.02){
				PWM_Duty  = PWM_Duty + delta_change_Duty;
			}
			else if(actual_Panel_Voltage<-0.02){
				PWM_Duty  = PWM_Duty - delta_change_Duty;
			}
		}
		else if(delta_Power < -1){
			delta_change_Duty = 5;

			if(actual_Panel_Voltage>0.02){
				PWM_Duty  = PWM_Duty - delta_change_Duty;
			}
			else if(actual_Panel_Voltage<-0.02){
				PWM_Duty  = PWM_Duty + delta_change_Duty;
			}
		}
		else if(delta_Power > 0.5){
			delta_change_Duty = 1;

			if(actual_Panel_Voltage>0.02){
				PWM_Duty  = PWM_Duty + delta_change_Duty;
			}
			else if(actual_Panel_Voltage<-0.02){
				PWM_Duty  = PWM_Duty - delta_change_Duty;
			}
		}
		else if(delta_Power < -0.5){
			delta_change_Duty = 1;

			if(actual_Panel_Voltage>0.02){
				PWM_Duty  = PWM_Duty - delta_change_Duty;
			}
			else if(actual_Panel_Voltage<-0.02){
				PWM_Duty  = PWM_Duty + delta_change_Duty;
			}
		}

		else if(delta_Power > 0.1){
			delta_change_Duty = 0.2;

			if(actual_Panel_Voltage>0.02){
				PWM_Duty  = PWM_Duty + delta_change_Duty;
			}
			else if(actual_Panel_Voltage<-0.02){
				PWM_Duty  = PWM_Duty - delta_change_Duty;
			}
		}
		else if(delta_Power < -0.1){
			delta_change_Duty = 0.2;

			if(actual_Panel_Voltage>0.02){
				PWM_Duty  = PWM_Duty - delta_change_Duty;
			}
			else if(actual_Panel_Voltage<-0.02){
				PWM_Duty  = PWM_Duty + delta_change_Duty;
			}
		}



		if(PWM_Duty > 85.0){
			PWM_Duty = 85;
		}
		else if(PWM_Duty < 5.0){
			PWM_Duty = 5;
		}

		scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
		TIM1->CCR1 = round(scaled_duty_ratio);

		prev_Panel_Power = filtered_Power;
		prev_Panel_Duty = PWM_Duty;

		temp_test = PWM_Duty;
		snprintf(buffer,5, "%f",temp_test);
		lcd_set_cursor(0, 10);
		lcd_write_string(buffer);

		temp_test = round(actual_Panel_Current*1000)/1000.0;
		snprintf(buffer,5, "%f",temp_test);
		lcd_set_cursor(1, 10);
		lcd_write_string(buffer);


		temp_test = round(actual_Panel_Voltage*1000)/1000;
		snprintf(buffer,5, "%f",temp_test);
		lcd_set_cursor(2, 10);
		lcd_write_string(buffer);


		temp_test = round(filtered_Power*1000)/1000;
		snprintf(buffer,6, "%f",temp_test);
		lcd_set_cursor(3, 10);
		lcd_write_string(buffer);
	}
}

void P_and_O_2(){////WORKING DONT CHANGE
	double PWM_Duty = 15.0;
	double scaled_duty_ratio = 0.0;

	double panel_Current_ADC = 0.0;
	double actual_Panel_Current = 0.0;

	double panel_Voltage_ADC = 0.0;
	double actual_Panel_Voltage = 0.0;

	double actual_Panel_Power = 0.0;

	double delta_Power = 0.0;
	double delta_Duty = 0.0;
	double delta_Voltage = 0.0;

	double delta_Step = 0.0;
	double N = 0.2;

	double prev_Panel_Power = 0.0;
	double prev_Panel_Voltage = 0.0;

	char buffer[10];
	char *temp = "";
	double temp_test = 0.0;

	TIM1->CCR1 = round(PWM_Duty);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	while(1){

		if(sampled == 1){
			sampled = 0;
			panel_Current_ADC = current_Running_Total/600.0;
			actual_Panel_Current = round((((((panel_Current_ADC)/4095.0)*3.3)-2.35)/0.04)*10.0)/10.0;//
//			filtered_Current = round(((actual_Panel_Power*alpha_Power) + ((1-alpha_Power)*(prev_Panel_Power)))*100.0)/100;

			panel_Voltage_ADC = voltage_Running_Total/600.0;
			actual_Panel_Voltage = round(((((panel_Voltage_ADC/4095.0)*3.3)*(166000))/(21900))*10.0)/10.0;
//			filtered_Voltage = round(((actual_Panel_Power*alpha_Power) + ((1-alpha_Power)*(prev_Panel_Power)))*100.0)/100;

			actual_Panel_Power = (actual_Panel_Current*actual_Panel_Voltage);

			delta_Power = actual_Panel_Power - prev_Panel_Power;
			delta_Voltage = actual_Panel_Voltage - prev_Panel_Voltage;
//			delta_Duty = actual_Panel_Current - prev_Panel_Current;

//			delta_Step = N * abs(delta_Power/delta_Voltage);

//
			delta_Step = 0.4; ///Very crucial ******

			if(delta_Power > 0.0){
				if(delta_Voltage < 0.0){
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else {
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}
			else {
				if(delta_Voltage < 0.0){
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else {
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}

			if(PWM_Duty > 85.0){
				PWM_Duty = 85;
			}
			else if(PWM_Duty < 5.0){
				PWM_Duty = 5;
			}

			scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
			TIM1->CCR1 = round(scaled_duty_ratio);

			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	//		Refresh variables

			prev_Panel_Power = actual_Panel_Power;
//			prev_Panel_Current = actual_Panel_Current;
			prev_Panel_Voltage = actual_Panel_Voltage;

			//		LCD Writing

			snprintf(buffer,5, "%f",PWM_Duty);
			lcd_set_cursor(0, 10);
			lcd_write_string(buffer);

			temp_test = round(actual_Panel_Current*1000)/1000.0;
			snprintf(buffer,5, "%f",temp_test);
			lcd_set_cursor(1, 10);
			lcd_write_string(buffer);


			temp_test = round(actual_Panel_Voltage*1000)/1000;
			snprintf(buffer,5, "%f",temp_test);
			lcd_set_cursor(2, 10);
			lcd_write_string(buffer);


			temp_test = round(actual_Panel_Power*1000)/1000;
			snprintf(buffer,6, "%f",temp_test);
			lcd_set_cursor(3, 10);
			lcd_write_string(buffer);
//			HAL_Delay(10);
		}
	}
}

void P_and_O_3(){
	double PWM_Duty = 15.0;
	double scaled_duty_ratio = 0.0;

	double panel_Current_ADC = 0.0;
	double actual_Panel_Current = 0.0;

	double panel_Voltage_ADC = 0.0;
	double actual_Panel_Voltage = 0.0;

	double actual_Panel_Power = 0.0;

	double delta_Power = 0.0;
	double delta_Duty = 0.0;
	double delta_Voltage = 0.0;

	double delta_Step = 0.0;
	double N = 0.2;

	double prev_Panel_Power = 0.0;
	double prev_Panel_Voltage = 0.0;

	char buffer[10];
	char *temp = "";
	double temp_test = 0.0;

	TIM1->CCR1 = round(PWM_Duty);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	while(1){

		if(sampled == 1){
			sampled = 0;
			panel_Current_ADC = current_Running_Total/500.0;
			actual_Panel_Current = round((((((panel_Current_ADC)/4095.0)*3.3)-2.35)/0.04)*10.0)/10.0;//
//			filtered_Current = round(((actual_Panel_Power*alpha_Power) + ((1-alpha_Power)*(prev_Panel_Power)))*100.0)/100;

			panel_Voltage_ADC = voltage_Running_Total/500.0;
			actual_Panel_Voltage = round(((((panel_Voltage_ADC/4095.0)*3.3)*(166000))/(21900))*10.0)/10.0;
//			filtered_Voltage = round(((actual_Panel_Power*alpha_Power) + ((1-alpha_Power)*(prev_Panel_Power)))*100.0)/100;

			actual_Panel_Power = (actual_Panel_Current*actual_Panel_Voltage);

			delta_Power = actual_Panel_Power - prev_Panel_Power;
			delta_Voltage = actual_Panel_Voltage - prev_Panel_Voltage;

			delta_Step = 0.2;

			if(delta_Power > 4.0){
				delta_Step = 1;

				if(delta_Voltage < -0.01){
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else if(delta_Voltage > 0.01){
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}
			else if(delta_Power < -4.0){
				delta_Step = 1;

				if(delta_Voltage < -0.001){
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else if(delta_Voltage > 0.001){
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}
			else if(delta_Power > 0.001){
				if(delta_Voltage < -0.01){
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else if(delta_Voltage > 0.01){
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}
			else if(delta_Power < -0.001){
				if(delta_Voltage < -0.01){
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else if (delta_Voltage > 0.01) {
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}



			if(PWM_Duty > 85.0){
				PWM_Duty = 85;
			}
			else if(PWM_Duty < 5.0){
				PWM_Duty = 5;
			}

			scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
			TIM1->CCR1 = round(scaled_duty_ratio);

			//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	//		Refresh variables


			prev_Panel_Power = actual_Panel_Power;
//			prev_Panel_Current = actual_Panel_Current;
			prev_Panel_Voltage = actual_Panel_Voltage;

			//		LCD Writing

			snprintf(buffer,5, "%f",PWM_Duty);
			lcd_set_cursor(0, 10);
			lcd_write_string(buffer);

			temp_test = round(actual_Panel_Current*1000)/1000.0;
			snprintf(buffer,5, "%f",temp_test);
			lcd_set_cursor(1, 10);
			lcd_write_string(buffer);


			temp_test = round(actual_Panel_Voltage*1000)/1000;
			snprintf(buffer,5, "%f",temp_test);
			lcd_set_cursor(2, 10);
			lcd_write_string(buffer);


			temp_test = round(actual_Panel_Power*1000)/1000;
			snprintf(buffer,6, "%f",temp_test);
			lcd_set_cursor(3, 10);
			lcd_write_string(buffer);
//			HAL_Delay(10);
		}
	}
}

void Inc_Cond(){
	double delta_Power =0.0;
	double delta_Duty = 0.0;
	double delta_Voltage = 0.0;
	double delta_Current = 0.0;

	double prev_Panel_Power = 20;
	double prev_Panel_Duty = 50;
	double prev_Panel_Current = 0.0;
	double prev_Panel_Voltage = 0.0;


	double alpha_Power = 0.5;
	double alpha = 0.3;
	double filtered_Power = 0.0;
	double filtered_Current = 0.0;
	double filtered_Voltage = 0.0;

	double N = 1;

	double delta_Step = 0.0;

	double temp_test = 0.0;
	double PWM_Duty = 50.0;
	double scaled_duty_ratio = 10.0;
	char buffer[10];
	char *temp = "";

	double delta_IV = 0.0;
	double neg_I_over_V = 0.0;

	TIM1->CCR1 = round(PWM_Duty);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	while(1){

		PWM_Duty = (((adc_buf_panel[2])/4095.0)*60.0);
				PWM_Duty = PWM_Duty+5;

		panel_Current_ADC = current_Running_Total/100.0;
		panel_Voltage_ADC = voltage_Running_Total/100.0;


		actual_Panel_Current = (((((panel_Current_ADC)/4095.0)*3.3)-2.35)/0.04);//
		actual_Panel_Voltage = ((((panel_Voltage_ADC/4095.0)*3.3)*(166000))/(21900));

//		current_Running_Total = 0;
//		voltage_Running_Total = 0;

//		filtered_Current =  round(((actual_Panel_Current*alpha) + ((1-alpha)*(prev_Panel_Current)))*10.0)/10;
//		filtered_Voltage =  round(((actual_Panel_Voltage*alpha) + ((1-alpha)*(prev_Panel_Voltage)))*100.0)/100;

		actual_Panel_Power = (actual_Panel_Current*actual_Panel_Voltage);

		filtered_Power = round(((actual_Panel_Power*alpha_Power) + ((1-alpha_Power)*(prev_Panel_Power)))*100.0)/100;


		delta_Power = filtered_Power - prev_Panel_Power;
		delta_Voltage = actual_Panel_Voltage - prev_Panel_Current;
		delta_Current = actual_Panel_Current - prev_Panel_Current;


		N = 1.0;
		delta_Step = N * fabs(delta_Power*actual_Panel_Voltage);
		//delta_Step = 0.5;
		if(delta_Step > 1){
			delta_Step = 1;
		}



		if((delta_Voltage <= 0.005) || (delta_Voltage >= -0.005)){
			if(delta_Current > 0.005){
				PWM_Duty = 	PWM_Duty + delta_Step;
			}
			else if (delta_Current < -0.005){
				PWM_Duty = 	PWM_Duty - delta_Step;
			}
		}
		else{
			if((delta_Current/delta_Voltage) + (actual_Panel_Current/actual_Panel_Voltage)> 0.00001){
				PWM_Duty = 	PWM_Duty - delta_Step;
			}
			else if((delta_Current/delta_Voltage) + (actual_Panel_Current/actual_Panel_Voltage)< -0.00001){
				PWM_Duty = 	PWM_Duty + delta_Step;
			}
		}


		if(PWM_Duty > 85.0){
			PWM_Duty = 85;
		}
		else if(PWM_Duty < 5.0){
			PWM_Duty = 5;
		}

		scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
		TIM1->CCR1 = round(scaled_duty_ratio);


		prev_Panel_Power = actual_Panel_Power;
		prev_Panel_Duty = PWM_Duty;
		prev_Panel_Current = actual_Panel_Current;
		prev_Panel_Voltage = actual_Panel_Voltage;



		temp_test = PWM_Duty;
		snprintf(buffer,5, "%f",temp_test);
		lcd_set_cursor(0, 10);
		lcd_write_string(buffer);

		temp_test = round(actual_Panel_Current*1000)/1000.0;
		snprintf(buffer,5, "%f",temp_test);
		lcd_set_cursor(1, 10);
		lcd_write_string(buffer);


		temp_test = round(actual_Panel_Voltage*1000)/1000;
		snprintf(buffer,5, "%f",temp_test);
		lcd_set_cursor(2, 10);
		lcd_write_string(buffer);


		temp_test = round(actual_Panel_Power*1000)/1000;
		snprintf(buffer,6, "%f",temp_test);
		lcd_set_cursor(3, 10);
		lcd_write_string(buffer);
		//HAL_Delay(10);

	}
}


//azi_Tracking_Error working
//void azi_Tracking_Error(){
//	temp = "Error!!";
//	lcd_set_cursor(0, 0);
//	lcd_write_string(temp);
//
//	temp = "AZI Limit Not Found";
//	lcd_set_cursor(1, 0);
//	lcd_write_string(temp);
//
//	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 0);
//
//	bool tracking = false;
//
//	PWM_Duty = 22.0;
//	TIM16->CCR1 = round(PWM_Duty);
//	//Check direction
//	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
//
//	while(tracking == false)
//	{
//		if(azi_Limit_Switch == 1){
//			azi_Limit_Switch = 0;
//			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
//			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
//			HAL_Delay(300);
//			tracking = true;
//
//
//			lcd_clear();
//			HAL_Delay(2);
//			temp = "AZI Homed";
//			lcd_set_cursor(1, 0);
//			lcd_write_string(temp);
//			azi_Limit_Switch = 0;
//			__HAL_TIM_SET_COUNTER(&htim3, 0);
//		}
//	}
//	return;
//}


void vert_Tracking_Error(){
	bool vert_Limit_Flag = 0;

	temp = "Error!!";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	temp = "VER Limit Not Found";
	lcd_set_cursor(1, 0);
	lcd_write_string(temp);

	HAL_GPIO_WritePin(MOT_SWITCH_GPIO_Port, MOT_SWITCH_Pin, 1);

	bool tracking = false;

	PWM_Duty = 18.0;
	TIM16->CCR1 = round(PWM_Duty);
	//Check direction
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	while(tracking == false)
	{
		if(vert_Limit_Switch == 1){
			vert_Limit_Switch = 0;
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
			tracking = true;

			HAL_Delay(300);
			lcd_clear();
			HAL_Delay(2);
			temp = "AZI Homed";
			lcd_set_cursor(1, 0);
			lcd_write_string(temp);
			azi_Limit_Switch = 0;

			__HAL_TIM_SET_COUNTER(&htim2, 0);
		}
	}
	return;
}

void state_machine(){

	//CCCV Thresholds

	double V_bulk_thres = 13.5;
	double I_float_thres = 1.4; //C/100 -> 140/100
	double I_Bulk = 12.0;
	double V_Abs = 13.5;
	double V_float = 13.0;


	//General variables

	double PWM_Duty = 30.0;
	double PWM_Duty_CCCV = 30.0;
	double scaled_duty_ratio = 10.0;

	char buffer[10];

	int sample_count = 0;
	int CCCV_count_flag = 0;
	int VERT_tracker_count_flag = 0;
	int AZI_tracker_count_flag = 0;
	int Transmit_count_flag = 0;

	bool CCCV_State = 1;

//Start PWM for CCCV

	scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
	TIM1->CCR1 = round(scaled_duty_ratio);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	//HAL_Delay(500);

	scaled_duty_ratio = (PWM_Duty_CCCV/100.0)*1199.0;
	TIM15->CCR1 = round(scaled_duty_ratio);
	TIM15->CCR2 = round(scaled_duty_ratio);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

//Start PWM for MPPT



//Tracking Init

//	azi_Limit_Switch_2 = HAL_GPIO_ReadPin(AZI_Limit_GPIO_Port, AZI_Limit_Pin);
//	if(azi_Limit_Switch_2 == 0){
//
//		vert_Limit_Switch_2 = 0;
//		vert_Limit_Switch = 0;
//
//		azi_Limit_Switch_2 = 0;
//		azi_Limit_Switch = 0;
//
//		azi_Tracking_Error();
//		HAL_Delay(2000);
//	}

//	vert_Limit_Switch_2 = HAL_GPIO_ReadPin(VERT_Limit_GPIO_Port, VERT_Limit_Pin);
//	if(vert_Limit_Switch_2 == 0){
//
//		vert_Limit_Switch_2 = 0;
//		vert_Limit_Switch = 0;
//
//		azi_Limit_Switch_2 = 0;
//		azi_Limit_Switch = 0;
//
//		vert_Tracking_Error();
//		HAL_Delay(2000);
//	}

	azi_Limit_Switch = 0;

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	AZI_Tracker_Control_Test();
	HAL_Delay(2000);
//
//
	vert_Limit_Switch = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	VERT_Tracker_Control_Test();
	HAL_Delay(2000);

	lcd_clear();
	HAL_Delay(2);

//LCD Init

	temp = "Dp= ";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	temp = "Vp= ";
	lcd_set_cursor(1, 0);
	lcd_write_string(temp);

	temp = "Ip= ";
	lcd_set_cursor(2, 0);
	lcd_write_string(temp);

	temp = "Pp= ";
	lcd_set_cursor(3, 0);
	lcd_write_string(temp);



	temp = "Db= ";
	lcd_set_cursor(0, 10);
	lcd_write_string(temp);

	temp = "Vb= ";
	lcd_set_cursor(1, 10);
	lcd_write_string(temp);

	temp = "Ib= ";
	lcd_set_cursor(2, 10);
	lcd_write_string(temp);

	temp = "Pb= ";
	lcd_set_cursor(3, 10);
	lcd_write_string(temp);


//State Variable
	state_variable = 0;

	while(1){
		while(state_variable == 0){
			//CCCV
			if(sampled == 1)
			{
				//Capture ADC readings - Panel
				sampled = 0;
				panel_Current_ADC = current_Running_Total/600.0;
				actual_Panel_Current = round((((((panel_Current_ADC)/4095.0)*3.3)-2.45)/0.035)*10.0)/10.0;//

				panel_Voltage_ADC = voltage_Running_Total/600.0;
				actual_Panel_Voltage = round(((((panel_Voltage_ADC/4095.0)*3.3)*(253))/(31.6))*10.0)/10.0;

				//Transition
				state_variable = 1;

			}
		}

		while(state_variable == 1){
//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

			//MPPT

			actual_Panel_Power = (actual_Panel_Current*actual_Panel_Voltage);

			delta_Power = actual_Panel_Power - prev_Panel_Power;
			delta_Voltage = actual_Panel_Voltage - prev_Panel_Voltage;
//			delta_Duty = actual_Panel_Current - prev_Panel_Current;

			delta_Step = 0.6; ///Very crucial ******

			if(delta_Power > 0.0){
				if(delta_Voltage < 0.0){
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else {
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}
			else {
				if(delta_Voltage < 0.0){
					PWM_Duty = 	PWM_Duty - delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
				}
				else {
					PWM_Duty = 	PWM_Duty + delta_Step;
//					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
				}
			}

			if(PWM_Duty > 80.0){
				PWM_Duty = 30;
			}
			else if(PWM_Duty < 10.0){
				PWM_Duty = 50;
			}

			scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
			TIM1->CCR1 = round(scaled_duty_ratio);

	//		Refresh variables

			prev_Panel_Power = actual_Panel_Power;
			prev_Panel_Voltage = actual_Panel_Voltage;

			//LCD Writing

			snprintf(buffer,5, "%f",PWM_Duty);
			lcd_set_cursor(0, 4);
			lcd_write_string(buffer);

			snprintf(buffer,5, "%f",actual_Panel_Voltage);
			lcd_set_cursor(1, 4);
			lcd_write_string(buffer);


			snprintf(buffer,5, "%f",actual_Panel_Current);
			lcd_set_cursor(2, 4);
			lcd_write_string(buffer);


			snprintf(buffer,6, "%f",actual_Panel_Power);
			lcd_set_cursor(3, 4);
			lcd_write_string(buffer);




			sample_count = sample_count + 1;

//			Transition
			if(sample_count == 14){//(10 - 1)
				state_variable = 2;
				sample_count = 0;
			}
			else{
				state_variable = 0;
			}

		}

		while(state_variable == 2){

			//CCCV


			//CC = 0
			//CV = 1

//			CCCV_State = 0;


			//Measure Battery VI

			battery_Current_ADC = current_Running_Bat_Total/600.0;
			actual_Battery_Current = round((((((battery_Current_ADC)/4095.0)*3.3)-2.375)/0.038)*10.0)/10.0;//

			battery_Voltage_ADC = voltage_Running_Bat_Total/600.0;
			actual_Battery_Voltage = round(((((battery_Voltage_ADC/4095.0)*3.3)*(267))/(43))*10.0)/10.0;

			actual_Battery_Power = (actual_Battery_Current * actual_Battery_Voltage);

			//CCCV flow
			//CC until voltage goes above V_bulk_thres
			//CV when voltage is above V_bulk_thres AND current is above I_float_thres
			//Float charge when voltage is above V_bulk_thres AND current is below I_float_thres


			if(actual_Battery_Voltage < V_bulk_thres){
				//Define setpoint
				current_Setpoint = I_Bulk;

				//Calculate Error in Current
				error_Current_0 = current_Setpoint - actual_Battery_Current;

				//Determine control action;
				PWM_Control_Action = Kc_Current*((error_Current_0 - error_Current_1)+(((1/tau_i_Current)*(error_Current_0))));//((1/tau_i_Voltage)*(error_Voltage_0))
				PWM_Duty_CCCV = PWM_Duty_CCCV + PWM_Control_Action;
				error_Current_1 = error_Current_0;
			}
			else if((actual_Battery_Voltage >= V_bulk_thres) && (actual_Battery_Current > I_float_thres)){
				//Define setpoint
				voltage_Setpoint = V_Abs;

				//Calculater Error in Voltage
				error_Voltage_0 = voltage_Setpoint - actual_Battery_Voltage;

				//Determine control action;
				PWM_Control_Action = Kc_Voltage*((error_Voltage_0 - error_Voltage_1)+(((1/tau_i_Voltage)*(error_Voltage_0))));//((1/tau_i_Voltage)*(error_Voltage_0))
				PWM_Duty_CCCV = PWM_Duty_CCCV + PWM_Control_Action;
				error_Voltage_1 = error_Voltage_0;
			}
			else if((actual_Battery_Voltage >= V_bulk_thres) && (actual_Battery_Current < I_float_thres)){
				//Define setpoint
				voltage_Setpoint = V_float;

				//Calculater Error in Voltage
				error_Voltage_0 = voltage_Setpoint - actual_Battery_Voltage;

				//Determine control action;
				PWM_Control_Action = Kc_Voltage*((error_Voltage_0 - error_Voltage_1)+(((1/tau_i_Voltage)*(error_Voltage_0))));//((1/tau_i_Voltage)*(error_Voltage_0))
				PWM_Duty_CCCV = PWM_Duty_CCCV + PWM_Control_Action;
				error_Voltage_1 = error_Voltage_0;
			}

//			CCCV_State = 0;
//			current_Setpoint = 10.0;
//			voltage_Setpoint = 12.0;



			if(CCCV_State == 0){ //CC Control algorithm

				//Calculate Error in Current
				error_Current_0 = current_Setpoint - actual_Battery_Current;

				//Determine control action;
				PWM_Control_Action = Kc_Current*((error_Current_0 - error_Current_1)+(((1/tau_i_Current)*(error_Current_0))));//((1/tau_i_Voltage)*(error_Voltage_0))
				PWM_Duty_CCCV = PWM_Duty_CCCV + PWM_Control_Action;
				error_Current_1 = error_Current_0;

			}
			else{ //CV Control algorithm

				//Calculater Error in Voltage
				error_Voltage_0 = voltage_Setpoint - actual_Battery_Voltage;

				//Determine control action;
				PWM_Control_Action = Kc_Voltage*((error_Voltage_0 - error_Voltage_1)+(((1/tau_i_Voltage)*(error_Voltage_0))));//((1/tau_i_Voltage)*(error_Voltage_0))
				PWM_Duty_CCCV = PWM_Duty_CCCV + PWM_Control_Action;
				error_Voltage_1 = error_Voltage_0;
			}


			if(PWM_Duty_CCCV>85.0){
				PWM_Duty_CCCV = 85.0;
			}
			else if(PWM_Duty_CCCV<10.0){
				PWM_Duty_CCCV = 10.0;
			}

			scaled_duty_ratio = (PWM_Duty_CCCV/100.0)*1199.0;
			TIM15->CCR1 = round(scaled_duty_ratio);
			TIM15->CCR2 = round(scaled_duty_ratio);

			snprintf(buffer,5, "%f",PWM_Duty_CCCV);
			lcd_set_cursor(0, 14);
			lcd_write_string(buffer);

			snprintf(buffer,5, "%f",actual_Battery_Voltage);
			lcd_set_cursor(1, 14);
			lcd_write_string(buffer);


			snprintf(buffer,5, "%f",actual_Battery_Current);
			lcd_set_cursor(2, 14);
			lcd_write_string(buffer);


			snprintf(buffer,6, "%f",actual_Battery_Power);
			lcd_set_cursor(3, 14);
			lcd_write_string(buffer);

			//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);



			//Print to screen

//			snprintf(buffer,6, "%f",round(PWM_Duty_CCCV*100)/100);
//			lcd_set_cursor(0, 14);
//			lcd_write_string(buffer);
//
//			snprintf(buffer,5, "%f",actual_Battery_Voltage);
//			lcd_set_cursor(1, 14);
//			lcd_write_string(buffer);
//
//
//			snprintf(buffer,5, "%f",actual_Battery_Current);
//			lcd_set_cursor(2, 14);
//			lcd_write_string(buffer);
//
//
//			snprintf(buffer,6, "%f",actual_Battery_Power);
//			lcd_set_cursor(3, 14);
//			lcd_write_string(buffer);

			CCCV_count_flag = CCCV_count_flag + 1;
//			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

			if(CCCV_count_flag == ((10*20) - 1)){
				//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				CCCV_count_flag = 0;
				state_variable = 3;
			}
			else{
				state_variable = 0;
			}
		}

		while(state_variable == 3){
			//Transmit


			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//			Test whether timing is correct for this loop to occur every second when the *60 is removed



			tx_Panel_Voltage = round(actual_Panel_Voltage*100);
			tx_buff[0] = (tx_Panel_Voltage>>8) & 0xFF;
			tx_buff[1] = (tx_Panel_Voltage) & 0xFF;
			tx_Panel_Current = round(actual_Panel_Current*100);
			tx_buff[2] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[3] = (tx_Panel_Current) & 0xFF;
			tx_Panel_Current = round(actual_Battery_Voltage*100);
			tx_buff[4] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[5] = (tx_Panel_Current) & 0xFF;
			tx_Panel_Current = round(actual_Battery_Current*100);
			tx_buff[6] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[7] = (tx_Panel_Current) & 0xFF;
			tx_Panel_Current = round(Sun_Azi_Current*100);
			tx_buff[8] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[9] = (tx_Panel_Current) & 0xFF;
			tx_Panel_Current = round(Panel_Azi_Current*100);
			tx_buff[10] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[11] = (tx_Panel_Current) & 0xFF;
			tx_Panel_Current = round(Sun_Vert_Current*100);
			tx_buff[12] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[13] = (tx_Panel_Current) & 0xFF;
			tx_Panel_Current = round(Panel_Vert_Current*100);
			tx_buff[14] = (tx_Panel_Current>>8) & 0xFF;
			tx_buff[15] = (tx_Panel_Current) & 0xFF;
			tx_buff[16] = sTime.Hours;
			tx_buff[17] = sTime.Minutes;
			tx_buff[18] = sTime.Seconds;

			HAL_UART_Transmit(&huart3, tx_buff, 19, 1000);

			Transmit_count_flag = Transmit_count_flag + 1;

			if(Transmit_count_flag == (MIN_AZI_DELAY - 1)){
				Transmit_count_flag = 0;
				state_variable = 3;
			}
			else{
				state_variable = 0;
			}
		}

		while(state_variable == 4){
			//Azimuth Tracker
			AZI_tracker_count_flag = AZI_tracker_count_flag + 1;

			azi_Limit_Switch = 0;
			AZI_Tracker_Control_Test();

			if(AZI_tracker_count_flag ==  MIN_VERT_DELAY ){
				AZI_tracker_count_flag = 0;
				state_variable = 5;
			}
			else{
				state_variable = 0;
			}
		}

		while(state_variable == 5){
			//Vertical Tracker
			//VERT_tracker_count_flag = VERT_tracker_count_flag + 1;

			vert_Limit_Switch = 0;
			VERT_Tracker_Control_Test();
			state_variable = 0;
		}

		while(state_variable == 6){
			//SLEEP
			temp = "SLEEP";
			lcd_set_cursor(0,0);
			lcd_write_string(temp);
		}

	}
}

void test_Dual_Axis_Tracking(){
//	azi_Limit_Switch_2 = HAL_GPIO_ReadPin(AZI_Limit_GPIO_Port, AZI_Limit_Pin);
	if(azi_Limit_Switch_2 == 0){
		HAL_Delay(2000);
		vert_Limit_Switch_2 = 0;
		vert_Limit_Switch = 0;
		azi_Limit_Switch_2 = 0;
		azi_Limit_Switch = 0;
		azi_Tracking_Error();
		HAL_Delay(2000);
	}

//	vert_Limit_Switch_2 = HAL_GPIO_ReadPin(VERT_Limit_GPIO_Port, VERT_Limit_Pin);
	if(vert_Limit_Switch_2 == 0){
		HAL_Delay(2000);
		vert_Limit_Switch_2 = 0;
		vert_Limit_Switch = 0;
		azi_Limit_Switch_2 = 0;
		azi_Limit_Switch = 0;
		vert_Tracking_Error();
		HAL_Delay(2000);
	}

	lcd_clear();
	HAL_Delay(2);

	temp = "SUN_AZI = ";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	temp = "PANEL_AZI = ";
	lcd_set_cursor(1, 0);
	lcd_write_string(temp);

	temp = "SUN_VER = ";
	lcd_set_cursor(2, 0);
	lcd_write_string(temp);

	temp = "PANEL_VER = ";
	lcd_set_cursor(3, 0);
	lcd_write_string(temp);


	HAL_Delay(5000);


	//Azimuth Tracker
	azi_Limit_Switch = 0;
	AZI_Tracker_Control_Test();
	HAL_Delay(1000);

	lcd_clear();
	HAL_Delay(2);

	temp = "SUN_AZI = ";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	temp = "PANEL_AZI = ";
	lcd_set_cursor(1, 0);
	lcd_write_string(temp);

	temp = "SUN_VER = ";
	lcd_set_cursor(2, 0);
	lcd_write_string(temp);

	temp = "PANEL_VER = ";
	lcd_set_cursor(3, 0);
	lcd_write_string(temp);
	HAL_Delay(20000);



	vert_Limit_Switch = 0;
	VERT_Tracker_Control_Test();
	HAL_Delay(20000);
	while(1){

	}

}

void boost_Converter_Test(){
	char *temp;
	double PWM_Duty = 20.0;
	double power = 0.0;
	double scaled_duty_ratio = 20.0;
	int temp_int = 0;
	TIM1->CCR1 = round(PWM_Duty);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	scaled_duty_ratio = (PWM_Duty/100.0)*1199.0;
	TIM15->CCR1 = round(scaled_duty_ratio);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
	lcd_clear();
	HAL_Delay(2);

	temp = "Duty = ";
	lcd_set_cursor(0, 0);
	lcd_write_string(temp);

	temp = "Curre = ";
	lcd_set_cursor(1, 0);
	lcd_write_string(temp);

	temp = "Volt = ";
	lcd_set_cursor(2, 0);
	lcd_write_string(temp);

//	temp = "Pp= ";
//	lcd_set_cursor(3, 0);
//	lcd_write_string(temp);



	char buffer[10];
	//HAL_Delay(2000);

	while(1){
		power = read_Pot();
		PWM_Duty = power + 20.0;

		if(PWM_Duty < 30){
			PWM_Duty = 20;
		}
		else if((PWM_Duty < 50) && (PWM_Duty >= 30)){
			PWM_Duty = 40;
		}
		else if((PWM_Duty < 70) && (PWM_Duty >= 50)){
			PWM_Duty = 60;
		}
		else if((PWM_Duty >= 70)){
			PWM_Duty = 80;
		}



		temp_int = round(PWM_Duty);
		snprintf(buffer,10, "%d",temp_int);
		temp = buffer;
		lcd_set_cursor(0, 10);
		lcd_write_string(temp);

//		actual_Panel_ = (((((averaged_Current)/4095.0)*3.3)-2.375)/0.040);
//
//		temp_int = round(actual_Current*1000)/1000;
//		snprintf(buffer,10, "%d",temp_int);
//		temp = buffer;
//		lcd_set_cursor(1, 10);
//		lcd_write_string(temp);
//
//		actual_Panel_Voltage = ((((averaged_Voltage/4095.0)*3.3)*(166000))/(22000));
//
//		temp_int = round(actual_Voltage*1000)/1000;
//		snprintf(buffer,10, "%d",temp_int);
//		temp = buffer;
//		lcd_set_cursor(2, 10);
//		lcd_write_string(temp);

		scaled_duty_ratio = (PWM_Duty/100.0)*1199;
		TIM1->CCR1 = round(scaled_duty_ratio);
	}

}

void boost_Set_PWM(){
//
	double scaled_duty_ratio = (50/100.0)*1199.0;
	TIM15->CCR1 = round(scaled_duty_ratio);
	TIM15->CCR2 = round(scaled_duty_ratio);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

	//HAL_Delay(500);

}

double read_Pot(){
	double pot_result = 0.0;
	value_adc = HAL_ADC_GetValue(&hadc2);

	pot_result = (value_adc/4095.0)*60.0;
	return pot_result;
}

//LCD Functions

void lcd_clear(void) {
	lcd_send_cmd(0x01);
    //HAL_Delay(2);
}

void lcd_send_cmd(uint8_t cmd) {
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(upper_nibble, 0);
  lcd_write_nibble(lower_nibble, 0);
//  if (cmd == 0x01 || cmd == 0x02) {
//    HAL_Delay(2);
//  }
}

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= backlight_state << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
//  HAL_Delay(1);
  data &= ~(1 << EN_BIT);
  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}

void lcd_init() {
  HAL_Delay(50);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(5);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x02, 0);
  lcd_send_cmd(0x28);
  lcd_send_cmd(0x0C);
  lcd_send_cmd(0x06);
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

void lcd_write_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}

void lcd_send_data(uint8_t data) {
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(upper_nibble, 1);
  lcd_write_nibble(lower_nibble, 1);
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
    uint8_t address;
    switch (row) {
        case 0:
            address = 0x00;
            break;
        case 1:
            address = 0x40;
            break;
        case 2:
			address = 0x14;
			break;
        case 3:
			address = 0x54;
			break;
        default:
            address = 0x00;
    }
    address += column;
    lcd_send_cmd(0x80 | address);
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
