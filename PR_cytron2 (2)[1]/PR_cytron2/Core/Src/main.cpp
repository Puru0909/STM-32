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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CYTRON.h"
#include "GY_25.h"
#include "Tribase.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
const uint8_t arraysize = 15;
uint8_t startbit;
uint8_t stopbit;
uint8_t ordered_data[arraysize];
uint8_t received_data[arraysize];

uint8_t sending_data[] = {0xA5, 0X51};
uint8_t received_1_data[8];
uint8_t start_1_bit;
uint8_t stop_1_bit;
uint8_t ordered_1_data[8];
int16_t Angle;

/* PID Variables */
float Kp = 1.0, Ki = 0.1, Kd = 0.05; // Tune these values based on testing
float error = 0.0, last_error = 0.0;
float integral = 0.0, derivative = 0.0;
float PID_Output = 0.0;

/* Variables for angular velocity calculation */
uint32_t last_time = 0;
uint32_t current_time = 0;
int16_t last_angle = 0;
int16_t current_angle = 0;
float omega = 0.0;
float desired_omega = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

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



  // Send a dummy bit to initialize

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

        Cytron motor_1(&huart2, 0b000, 0); // Motor 1 on Cytron 1
        motor_1.send_dummy_bit();
        Cytron motor_2(&huart2, 0b000, 1); // Motor 2 on Cytron 1
        motor_2.send_dummy_bit();
        Cytron motor_3(&huart2, 0b001, 0); // Motor 3 on Cytron 2
        motor_3.send_dummy_bit();

        GY_25 imu(sending_data, sizeof(sending_data));
        imu.GY_25Init(&huart1, 170, 85);

        tribase tribase1(&motor_1, &motor_2, &motor_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //tribase1.inverseKinematics(0, 100, 0);
//	  motor_1.anti_clockwise(255);
//	  motor_2.anti_clockwise(255);
//	  motor_3.anti_clockwise(255);
//	  HAL_Delay(1000);
//	  motor_1.clockwise(255);
//	  motor_2.clockwise(255);
//	  motor_3.clockwise(255);
//	  HAL_Delay(1000);
//	  current_Angle= imu.current_angle;
//	  last_Angle = Angle;

	  imu.update_angle(&huart1);
	  //Angle = imu.get_angle();
//	  last_time = current_time;
 //     current_time =HAL_GetTick()/1000;
	     current_time = HAL_GetTick();
	     current_angle = imu.get_angle();

	     /* Calculate omega (angular velocity) */
	     uint32_t delta_time = current_time - last_time;
	     if (delta_time > 0)
	     {
	         int16_t delta_angle = current_angle - last_angle;
	         omega = (float)delta_angle / delta_time ;
	     }

	     /* PID Calculation */
	     error = desired_omega - omega;
	     integral += error * delta_time;
	     derivative = (error - last_error) / delta_time;
	     PID_Output = (Kp * error) + (Ki * integral) + (Kd * derivative);

	     last_error = error;
	     last_time = current_time;
	     last_angle = current_angle;


    //  omega = (last_Angle - current_Angle)/(last_time - current_time);
	  	  startbit=1;
	  	  stopbit=4;
	  	  HAL_UART_Receive(&huart3, received_data, 21, HAL_MAX_DELAY);
              for (uint8_t i = 0; i<21; i++){
	  	  	  	  if (received_data[i] == startbit && received_data[(i+20)%21] == stopbit){
	  	  			  for (uint8_t j = 0; j<21; j++){
	  	   				  ordered_data[j] = received_data[(i+j)%21];
	   	   			  }
	  	   		  }
	  	  	  }

	  	  	   	if(ordered_data[5]==2){
	  	  	   		//forward
	  	  	   		//tribase1.forward(255, 255);
	  	  	         	desired_omega = 0;
	  	  	   	        tribase1.inverseKinematics(0, 100 - PID_Output, 0);
	  	  	   	}else if(ordered_data[6]==2){
	  	  	   		//left
	  	  	   		//tribase1.left(255, 255, 255);
	  	  	           desired_omega = -50;
	  	  	   	       tribase1.inverseKinematics(0, 0, -100 - PID_Output);
	  	  	   	}else if(ordered_data[7]==2){
	  	  	   		//backward
	  	  	   		//tribase1.backward(255, 255);
	  	  	          desired_omega = 0;
	  	  	          tribase1.inverseKinematics(0, -100 - PID_Output, 0);
                 }else if(ordered_data[8]==2){
	  	  	   		  //right
                	 //tribase1.right(255, 255, 255);
                	 desired_omega = 50;
                	 tribase1.inverseKinematics(0, 0, 100 - PID_Output);
 	   		  	}else if(ordered_data[14]==2){
 		  	   		//clockwise
 	   		  		//tribase1.clockwise(255, 255, 255);
 	   		      	desired_omega = 100;
 	   		        tribase1.inverseKinematics(100 - PID_Output, 0, 0);
 	   		  	}else if(ordered_data[13]==2){
     	  	   		//anticlockwise
 	   		  		//tribase1.anti_clockwise(255, 255, 255);
 	   		     	 desired_omega = -100;
 	   		         tribase1.inverseKinematics(-100 - PID_Output, 0, 0);
 	   		  	}
 		  	   	else if (ordered_data[1]==2){
 		  	   		//hold
 		  	   		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
 		  	     	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
 		  	   	}else if (ordered_data[2]==2){
 		  	   		//jump
// 		  	    	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
// 		  	        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
 		  	   	}else if (ordered_data[3]==2){
 		  	   		//shoot
 		  	   		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);

 		  	   	}else if (ordered_data[4]==2){
 		  	   		//release
 		  	   	    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
 		  	   	}else {
 		  	   	    desired_omega = 0;
 	             	tribase1.brake();
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ball_holding_Pin|ball_shooting_Pin|ball_shootingA6_Pin|jumping_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(jumpingB12_GPIO_Port, jumpingB12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ball_holding_Pin ball_shooting_Pin ball_shootingA6_Pin jumping_Pin */
  GPIO_InitStruct.Pin = ball_holding_Pin|ball_shooting_Pin|ball_shootingA6_Pin|jumping_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : jumpingB12_Pin */
  GPIO_InitStruct.Pin = jumpingB12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(jumpingB12_GPIO_Port, &GPIO_InitStruct);

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
