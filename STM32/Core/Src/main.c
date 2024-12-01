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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include "motor_encoders.h"
#include "DC_MOTOR.h"
#include "ESP8266_HAL.h"
#include "MP.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define DC_MOTOR1    0
#define DC_MOTOR2    1

#define delta_x 5							//[cm]
#define delta_delay 10
#define T_CIRCUMFERENCE 4.4*M_PI			//[cm]
#define SPIN_RADIUS 11						//[cm]
#define TICKS_PER_REV 28000
#define DISTANCE_SCALE 1.00					// CHANGE BASED BATTERY LEVEL AND FLOOR FRICTION.
#define TICKS_PER_CM 55.5555556						//Measure and scale accordingly.
#define TICKS_PER_CM_TURN 57.876232			//TURN measure

#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint16_t adc_dma_result[5];								//IR Sensors
int adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);
uint8_t adc_conv_complete_flag = 0;
char dma_result_buffer[100];

int16_t MOTOR1_encoder_velocity;
int32_t MOTOR1_encoder_position;

int16_t MOTOR2_encoder_velocity;
int32_t MOTOR2_encoder_position;

double MOTOR1_distance = 0;
double MOTOR2_distance = 0;

//uint16_t MOTOR1_timer_counter;
int delay = 0;
int battery_level;
int mode = 0;							// Autoconfigure to automatic.(0 = Automatic; 1 = Manual)
uint8_t rx_buff[3];						// single byte receiver.

static encoder_instance encoder_instance_3 = {0, 0, 0};
static encoder_instance encoder_instance_4 = {0, 0, 0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim3.Instance){
		//MOTOR1_timer_counter = __HAL_TIM_GET_COUNTER(&htim3);
		update_encoder(&htim3, &encoder_instance_3);
		encoder_instance_3.last_counter_value = __HAL_TIM_GET_COUNTER(&htim3);
		MOTOR1_encoder_position = encoder_instance_3.position;
		MOTOR1_encoder_velocity = encoder_instance_3.velocity;
		MOTOR1_distance = (double)(MOTOR1_encoder_position*T_CIRCUMFERENCE/TICKS_PER_REV);

	}else if(htim->Instance == htim4.Instance){
		//MOTOR2_timer_counter = __HAL_TIM_GET_COUNTER(&htim4);
		update_encoder(&htim4, &encoder_instance_4);
		encoder_instance_4.last_counter_value = __HAL_TIM_GET_COUNTER(&htim4);
		MOTOR2_encoder_position = encoder_instance_4.position;
		MOTOR2_encoder_velocity = encoder_instance_4.velocity;
		MOTOR2_distance = (double)(MOTOR2_encoder_position*T_CIRCUMFERENCE/TICKS_PER_REV);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	// I set adc_conv_complete_flag variable to 1 when, HAL_ADC_ConvCpltCallback function is call.
	adc_conv_complete_flag = 1;
}

void spinner(double set_final_angle, double current_orientation){
	double delta_theta = fmod((set_final_angle - current_orientation + 2*M_PI),(2*M_PI)) - 2*M_PI;
	reset_encoder(&encoder_instance_3);
	reset_encoder(&encoder_instance_4);
		 if(delta_theta > 0){
			  //rotate counter-clockwise:
			  DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, 135);
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 127);
		  }else{
			  //rotate clockwise:
			  DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 135);
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CW, 127);
		  }

		  //while(!(MOTOR1_distance==SPIN_RADIUS*fabs(delta_theta)));			//might have to take avg of the 2 distances
		 delay = (int)(DISTANCE_SCALE*SPIN_RADIUS*fabs(delta_theta)*TICKS_PER_CM_TURN);
		 HAL_Delay(delay);														//Using feedforward solution
		 // Stop Motors:
		 DC_MOTOR_Stop(DC_MOTOR1);
		 DC_MOTOR_Stop(DC_MOTOR2);
}

int obstacle_delay(int delay)
{
	int obstacle_found = -1;
	int num_of_delays = delay/delta_delay+1;
	int i;
	for(i=0;i<num_of_delays;i++)
	{
		HAL_Delay(delta_delay);
    // Script for polling of ir sensor
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_RESET ||HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==GPIO_PIN_RESET || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==GPIO_PIN_RESET)
    {
      obstacle_found = i;
      DC_MOTOR_Stop(DC_MOTOR1);
      DC_MOTOR_Stop(DC_MOTOR2);
      HAL_Delay(10);
      break;
    }
	}
	return obstacle_found;

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();


  /* USER CODE BEGIN 2 */

  	//ESP_Init("Archit2004","12345678");

    //HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_result , adc_channel_count); 	// Initialize the DMA conversion
    DC_MOTOR_Init(DC_MOTOR1);				// Initialization of motors
    DC_MOTOR_Init(DC_MOTOR2);

	DC_MOTOR_Stop(DC_MOTOR1);				// Because Motors run Randomly
	DC_MOTOR_Stop(DC_MOTOR2);
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);		//

    //int poly_x[] = {0, 1, 2, 3, 2, 1};							//Structure of test polygon
    //int poly_y[] = {0, 1, 1, 0, -1, -1};

    point _point[] = {{0,0},{100,100},{200,100},{300,0},{200,-100},{100,-100}};			//[cm]

    int min_x = _point[0].x;
    int max_x = _point[0].x;
    int argmax_x = 0;
    int argmin_x = 0;

    //Convex Polygon
    int j = 1;
    int number_of_points = 6;

    for(;j<number_of_points;j++)
    {
    	if(min_x>_point[j].x){
    		argmin_x = j;
    		min_x = _point[j].x;
    	}
    	if(max_x<_point[j].x){
    		argmax_x = j;
    		max_x = _point[j].x;
    	}
    }
    int top_segments = abs(argmax_x-argmin_x);
    int bottom_segments = number_of_points - abs(argmax_x-argmin_x);
    segment _segm_top[top_segments];
    segment _segm_bottom[bottom_segments];

    int i;
    for(i=0;i<top_segments;i++)		//top segments defined.
    {
    	_segm_top[i].p1 = &_point[(argmin_x+i)%number_of_points];
    	_segm_top[i].p2 = &_point[(argmin_x+i+1)%number_of_points];
    	calculate_segment(&_segm_top[i]);
    }
    for(i=0;i<bottom_segments;i++)		//Bottom segments defined.
    {
      _segm_bottom[bottom_segments-i-1].p2 = &_point[(argmax_x+i)%number_of_points];
      _segm_bottom[bottom_segments-i-1].p1 = &_point[(argmax_x+i+1)%number_of_points];
      calculate_segment(&_segm_bottom[bottom_segments-i-1]);
    }

    //polygon definition ends here
    double curr_x = 0;
    double curr_y = 0;
    double theta = 0;
    double prev_setpoint_x = 0;
    double prev_setpoint_y =0;
    double prev_setpoint_theta =0;
    double next_setpoint_x = _segm_top[argmin_x].p1->x;
    double next_setpoint_y = _segm_top[argmin_x].p1->y;
    double next_setpoint_theta = atan2((_segm_top[argmin_x].p1->y - curr_y), (_segm_top[argmin_x].p1->x - curr_x));

    //HAL_UART_Receive_IT(&huart3, rx_buff, 3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(mode == 1){									//control was switched to manual.
		  if(rx_buff[0]==51){
			  //Forward
			  DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 127);
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 127);

		  }else if(rx_buff[0]==52){
			  //Reverse
			  DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, 127);
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CW, 127);
		  }else if(rx_buff[0]==53){
			  //Left
			  DC_MOTOR_Start(DC_MOTOR1, DIR_CCW, 127);
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 127);
		  }else if(rx_buff[0]==54){
			  //Right
			  DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 127);
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CW, 127);
		  }else if(rx_buff[0]==49){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		  }else{
			  continue;							//Corresponding to UART code '7' 55.
		  }

		  HAL_Delay(4);							//To avoid rapid switching of motor.
	  }else{
		  //RUN IN AUTO MODE


		  spinner(next_setpoint_theta, theta);
		  theta = next_setpoint_theta;

			  //Go Straight:
			reset_encoder(&encoder_instance_3);
			reset_encoder(&encoder_instance_4);

			  DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 200);	//Left motor clockwise from inside.
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 200); 	//Right motor anti clockwise from inside.

			delay = (int)(sqrt((prev_setpoint_x-next_setpoint_x)*(prev_setpoint_x-next_setpoint_x)+(prev_setpoint_y-next_setpoint_y)*(prev_setpoint_y-next_setpoint_y))*DISTANCE_SCALE*TICKS_PER_CM);
			int x = obstacle_delay(delay);
      if(x !=-1)
      {
          spinner(theta + M_PI, theta);
      }
      //define what to do after obstacle is detected.


			//while(MOTOR1_distance*MOTOR1_distance<=(prev_setpoint_x-next_setpoint_x)*(prev_setpoint_x-next_setpoint_x)+(prev_setpoint_y-next_setpoint_y)*(prev_setpoint_y-next_setpoint_y));

			// Stop Motors:
			DC_MOTOR_Stop(DC_MOTOR1);
			DC_MOTOR_Stop(DC_MOTOR2);
		    curr_x = next_setpoint_x;
		    curr_y = next_setpoint_y;

		    //Update Setpoints:
			prev_setpoint_x = curr_x;
		    prev_setpoint_y = curr_y;
		    prev_setpoint_theta = theta;

		    next_setpoint_theta = atan2((_segm_top[argmin_x].p2->y - curr_y), (_segm_top[argmin_x].p2->x - curr_x));
		    spinner(next_setpoint_theta, theta);
		    theta = next_setpoint_theta;

		    int top_i = 0;
		    int bottom_i = 0;
		    segment segment_top = _segm_top[0];
		    segment segment_bottom = _segm_bottom[0];

			  int i;
			  for(i=0;i < ceil(max_x-min_x/(2*delta_x));i++)
		    {
		      next_setpoint_x = curr_x + delta_x;
		      next_setpoint_y = next_setpoint_x*segment_top.m + segment_top.c;

		      //Go Straight:
		      DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 200);	//Left motor clockwise from inside.
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 200); 	//Right motor anti clockwise from inside.

			  delay = (int)(sqrt((curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))*DISTANCE_SCALE*TICKS_PER_CM);
			  // HAL_Delay(delay);
          x = obstacle_delay(delay);
          if(x !=-1)
          {
              spinner(theta + M_PI, theta);
          }
//			  while(!(MOTOR1_distance*MOTOR1_distance==(curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))){
//				  //Check for Sensor Reads:
//			    }
			    // Stop Motors:
			    DC_MOTOR_Stop(DC_MOTOR1);
			    DC_MOTOR_Stop(DC_MOTOR2);

		      // Update Current State
		      curr_x = next_setpoint_x;
		      curr_y = next_setpoint_y;

		      next_setpoint_theta = -M_PI/2;
		      spinner(next_setpoint_theta, theta);
		      theta = next_setpoint_theta;

		      next_setpoint_x = curr_x;
		      next_setpoint_y = curr_x*segment_bottom.m + segment_bottom.c;

		      //Go Straight:
		      DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 200);	//Left motor clockwise from inside.
			    DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 200); 	//Right motor anti clockwise from inside.

			    delay = (int)(sqrt((curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))*DISTANCE_SCALE*TICKS_PER_CM);
			     x = obstacle_delay(delay);
          if(x !=-1)
          {
              spinner(theta + M_PI, theta);
          }
          
//			    while(!(MOTOR1_distance*MOTOR1_distance==(curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))){
//				  //Check for Sensor Reads:
//			    }
			    // Stop Motors:
			    DC_MOTOR_Stop(DC_MOTOR1);
			    DC_MOTOR_Stop(DC_MOTOR2);

		      // Update Current State
		      curr_x = next_setpoint_x;
		      curr_y = next_setpoint_y;

		      next_setpoint_theta = atan2((segment_bottom.p2->y - curr_y), (segment_bottom.p2->x - curr_x));
		      spinner(next_setpoint_theta, theta);
		      theta = next_setpoint_theta;

		      next_setpoint_x = curr_x+delta_x;
		      next_setpoint_y = next_setpoint_x*segment_bottom.m + segment_bottom.c;

		      //Go Straight:
		      DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 200);	//Left motor clockwise from inside.
			    DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 200); 	//Right motor anti clockwise from inside.

			    delay = (int)(sqrt((curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))*DISTANCE_SCALE*TICKS_PER_CM);
		x = obstacle_delay(delay);
          if(x !=-1)
          {
              spinner(theta + M_PI, theta);
          }

//			    while(!(MOTOR1_distance*MOTOR1_distance==(curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))){
//				  //Check for Sensor Reads:
//			    }
			    // Stop Motors:
			    DC_MOTOR_Stop(DC_MOTOR1);
			    DC_MOTOR_Stop(DC_MOTOR2);

		      // Update Current State
		      curr_x = next_setpoint_x;
		      curr_y = next_setpoint_y;

		      next_setpoint_theta = M_PI/2;
		      spinner(next_setpoint_theta, theta);
		      theta = next_setpoint_theta;

		      next_setpoint_x = curr_x;
		      next_setpoint_y = curr_x*segment_top.m + segment_top.c;

		      //Go Straight:
		      DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 200);	//Left motor clockwise from inside.
			  DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 200); 	//Right motor anti clockwise from inside.

			  delay = (int)(sqrt((curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))*DISTANCE_SCALE*TICKS_PER_CM);
			  x = obstacle_delay(delay);
          if(x !=-1)
          {
              spinner(theta + M_PI, theta);
          }

//			  while(!(MOTOR1_distance*MOTOR1_distance==(curr_x-next_setpoint_x)*(curr_x-next_setpoint_x)+(curr_y-next_setpoint_y)*(curr_y-next_setpoint_y))){
//				  //Check for Sensor Reads:
//			    }
			    // Stop Motors:
			    DC_MOTOR_Stop(DC_MOTOR1);
			    DC_MOTOR_Stop(DC_MOTOR2);

		      // Update Current State
		      curr_x = next_setpoint_x;
		      curr_y = next_setpoint_y;

		      next_setpoint_theta = atan2((segment_top.p2->y - curr_y), (segment_top.p2->x - curr_x));
		      spinner(next_setpoint_theta, theta);
		      theta = next_setpoint_theta;

		      if(curr_x>segment_top.p2->x){
		        top_i++;
		        if (top_i == top_segments)
		        {
		          break;
		        }
		        segment_top = _segm_top[top_i];
		      }
		      if(curr_x>segment_bottom.p2->x){
		        bottom_i++;
		        if (bottom_i == bottom_segments)
		        {
		          break;

		        }
		        segment_bottom = _segm_bottom[bottom_i];
		      }
		    }

	  }

	/* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 PA9 PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart3, rx_buff, 3); //You need to toggle a breakpoint on this line!

  if(rx_buff[0]==50){							//toggle automatic/manual control.
	  mode = 0;
  }else if(rx_buff[0]== 48){
	  mode = 1;
	  DC_MOTOR_Stop(DC_MOTOR1);
	  DC_MOTOR_Stop(DC_MOTOR2);
	  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);		//Reset the Blower
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//HANDLE_IR_SENSOR_DETECTION HERE!
	DC_MOTOR_Stop(DC_MOTOR1);
	DC_MOTOR_Stop(DC_MOTOR2);
	HAL_Delay(4);

	DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 127);
	DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 127);
	HAL_Delay(750);

	DC_MOTOR_Stop(DC_MOTOR1);
	DC_MOTOR_Stop(DC_MOTOR2);
	HAL_Delay(4);

	DC_MOTOR_Start(DC_MOTOR1, DIR_CW, 127);
	DC_MOTOR_Start(DC_MOTOR2, DIR_CCW, 127);
	HAL_Delay(4);
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
