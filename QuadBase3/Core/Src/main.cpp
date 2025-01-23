/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include <cmath>
#include "GY_25.h"
#include "Motor.h"
#include "QuadBase.h"
#include "Encoder_v1.0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_SIZE (10)		// size of ESP32->STM32 UART data in bytes(uint8) [NOT including EXTRA_BYTES]
#define RAW_DATA_SIZE (12)	// size of ESP32->STM32 UART data in bytes(uint8) [including EXTRA_BYTES]
#define START_BYTE (1) 		// start byte of ESP->STM UART
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// variables for IMU(GY-25)
GY_25 imu(huart2);

// variables for encoders
Encoder encoder_x(&htim3, 100, 55, ENCODER_UNITS.MM);
Encoder encoder_y(&htim2, 100, 55, ENCODER_UNITS.MM);

// Variables for motors of the base

MOTOR motor1(huart1, 0b0000);
MOTOR motor2(huart1, 0b1000);
MOTOR motor3(huart1, 0b0001);
MOTOR motor4(huart1, 0b1001);

// variables for ESP to STM via UART
uint8_t received_data[RAW_DATA_SIZE];
uint8_t ordered_data[DATA_SIZE];
uint8_t check_sum_byte;
bool last_sync_status = false;
bool in_sync = false;
bool data1[8];
bool data2[8];

// variables for inverse kinematics
QuadBaseKinematics Base;
float x_velocity;
float y_velocity;
float omega;

// variables for odometry output
float raw_x_position;
float raw_y_position;
float raw_theta;

float x_position;
float y_position;
float theta;

float offset_x_position;
float offset_y_position;
float offset_theta;

// variables for odometry calculations
float encoder_x_offset = -0.185;  // in meters
float encoder_y_offset = -0.175;  // in meters

float last_theta;
float last_encoder_x;
float last_encoder_y;

float current_theta;
float current_encoder_x;
float current_encoder_y;

float delta_theta;
float delta_x;
float delta_y;

// variables for applying odometry
float desired_x_position = 0;
float desired_y_position = 0;
float max_error_x = 0.01;
float max_error_y = 0.01;
float max_x_velocity = 0.2;
float max_y_velocity = 0.2;
float x_velocity_scaler = 1;
float y_velocity_scaler = 1;

// variables for PID
PID x_position_pid(x_velocity_scaler, 0, 0);
PID y_position_pid(y_velocity_scaler, 0, 0);
PID theta_pid(1, 0, 0.4);

// function for converting an uint8_t to an array of 8 bool
void byte_to_bools(unsigned char c, bool b[8])
{
    for (int i=0; i < 8; ++i) {
        b[i] = (c & (1<<i)) != 0;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  motor1.motor_init();
  motor2.motor_init();
  motor3.motor_init();
  motor4.motor_init();
  HAL_UART_Receive_DMA(&huart3, received_data, RAW_DATA_SIZE);
  encoder_x.init();
  encoder_y.init();

  x_position_pid.set_minimum_error(0.01);
  y_position_pid.set_minimum_error(0.01);
  theta_pid.set_minimum_error(0.05);

  x_position_pid.set_output_constrains(max_x_velocity, -max_x_velocity);
  y_position_pid.set_output_constrains(max_y_velocity, -max_y_velocity);
  theta_pid.set_output_constrains(1/0.3125, -1/0.3125);

  imu.update_counts();
  imu.update_angles();
  encoder_x.update();
  encoder_y.update();
  offset_x_position = x_position;
  offset_y_position = y_position;
  offset_theta = (imu.x_angle)*2*M_PI/360;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	   * receiving UART data of ESP32 and storing it
	   * stores the value in ordered_data as byte(uint8) array
	   */
	  last_sync_status = in_sync;
	  in_sync = false;
  	  for (uint8_t i = 0; i<RAW_DATA_SIZE; i++){
  		  if (received_data[i] == START_BYTE) {
  			  check_sum_byte = 0;
  			  for (uint8_t j = 0; j<DATA_SIZE; j++){
  				  ordered_data[j] = received_data[(i+j+1)%(RAW_DATA_SIZE)];
  				  check_sum_byte += ordered_data[j];
  			  }
  			  if (received_data[(i+RAW_DATA_SIZE-1)%RAW_DATA_SIZE] == check_sum_byte) {
  				  in_sync = true;
  				  break;
  			  }
  		  }
  	  }
  	  // updating out of sync led
  	  if (last_sync_status != in_sync){
  		  if (!in_sync) {
  			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  		  }
  		  else {
  			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  		  }
  	  }
  	  // converting the ordered data (uint8 array) to array of bools
  	  byte_to_bools(ordered_data[1], data1);
  	  byte_to_bools(ordered_data[2], data2);
  	  desired_x_position = ((float)ordered_data[3]) - 127;
  	  desired_y_position = ((float)ordered_data[4]) - 127;
  	  /*
  	   * updating the reading of IMU
  	   */
  	  imu.update_counts();
	  imu.update_angles();

	  /*
	   * updating encoder values
	   */
	  encoder_x.update();
	  encoder_y.update();

	  /*
	   * storing the values of encoders and IMU appropriately
	   */
	  last_encoder_x = current_encoder_x;
	  last_encoder_y = current_encoder_y;
	  last_theta = current_theta;
	  current_encoder_x = encoder_x.getDistance(ENCODER_UNITS.METER);
	  current_encoder_y = encoder_y.getDistance(ENCODER_UNITS.METER);
	  current_theta = (imu.x_angle)*2*M_PI/360;

	  /*
	   * calculations for odometry
	   */
	  delta_theta = current_theta - last_theta;
	  delta_x = current_encoder_x - last_encoder_x;
	  delta_y = current_encoder_y - last_encoder_y;
	  float delta_position_x = delta_x + (delta_theta * encoder_x_offset);
	  float delta_position_y = delta_y - (delta_theta * encoder_y_offset);
	  raw_x_position += cos(current_theta)*delta_position_x - sin(current_theta)*delta_position_y;
	  raw_y_position += sin(current_theta)*delta_position_x + cos(current_theta)*delta_position_y;
	  raw_theta = current_theta;
	  /*
	   * 		 Y+
	   * 		  ^
	   * 		  |
	   *          |
	   *  4 /           \ 3
	   *   /\  ESP STM  /\
	   *     \         /
	   *      \_______/
	   *       |     | MDDS30
	   *       |_____| MDDS30  ----->X+
	   *      /      \
	   *     /        \
	   *   \/          \/
	   *  2 \          / 1
	   */

	  // applying PID onto x position, y position, and theta
	  x_velocity = 0;
	  y_velocity = 0;
	  omega = 0;

	  x_position = raw_x_position - offset_x_position;
	  y_position = raw_y_position - offset_y_position;
	  theta = raw_theta - offset_theta;
//	  if (y_position <= desired_y_position - max_error_y || y_position >= desired_y_position + max_error_y) {
//		  y_velocity += (desired_y_position - y_position)*y_velocity_scaler;
//	  }
//	  if (x_position <= desired_x_position - max_error_x || x_position >= desired_x_position + max_error_x) {
//		  x_velocity += (desired_x_position - x_position)*x_velocity_scaler;
//	  }
//	  if (y_velocity >  max_y_velocity) {y_velocity =  max_y_velocity;}
//	  if (y_velocity < -max_y_velocity) {y_velocity = -max_y_velocity;}
//	  if (x_velocity >  max_x_velocity) {x_velocity =  max_x_velocity;}
//	  if (x_velocity < -max_x_velocity) {x_velocity = -max_x_velocity;}
	  x_velocity = x_position_pid.compute(desired_x_position - x_position);
	  y_velocity = y_position_pid.compute(desired_y_position - y_position);
	  omega = theta_pid.compute(0 - theta);
	  Base.calculate_speeds(theta, x_velocity, y_velocity, omega*0.7);

	  if (abs(Base.speed1) <= 10) {Base.speed1 = 0;}
	  if (abs(Base.speed2) <= 10) {Base.speed2 = 0;}
	  if (abs(Base.speed3) <= 10) {Base.speed3 = 0;}
	  if (abs(Base.speed4) <= 10) {Base.speed4 = 0;}
	  /*
	   * Actuating the motors
	   */
	  motor1.set_speed(Base.speed1);
	  motor2.set_speed(Base.speed2);
	  motor3.set_speed(Base.speed3);
	  motor4.set_speed(Base.speed4);
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
  htim2.Init.Period = 400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  htim3.Init.Period = 400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(internal_led_GPIO_Port, internal_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : internal_led_Pin */
  GPIO_InitStruct.Pin = internal_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(internal_led_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
