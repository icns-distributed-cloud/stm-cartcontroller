
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include <stm32f4xx_hal_gpio.h>
#include "stm32f4xx_it.h"

// This file is a part of stm-cartcontroller
// BSD 3-Clause License
//
// Copyright (c) 2019, Intelligent-distributed Cloud and Security Laboratory (ICNS Lab.)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// title           : main.c
// description     : C, stm cart controller
// author          : Je Hyeon Yu, Ji Hoo Chun, Yun Hwan Kim.
// date            : 20190807
// version         : 0.3
// TrueSTUDIO	   : 9.3.0
// notes           : This stm cart controller is an implementation of a cart controller for
//					 self-driving in the C Programming Language.
// ==============================================================================

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/************BLUETOOTH BUFFER**********/
char Buf1[30];
char Buf2[30];
char Buf3[30];
char Buf4[30];
char Buf5[30];
char Buf6[30];
char Buf7[30];
char Buf8[30];

char enter1=13;
char enter2=10;
char space=32;
int encoderL, encoderR;
int cntLast;
char rx;

volatile int Mode_Bluetooth;

/********VELOCITY NORMALIZATION********/
int W1_MIN = 600;														// Left_Motor Initial Velocity
int W2_MIN = 600;														// Right_Motor Initial Velocity
#define RANGE_MAX 60
int n_v1, n_v2; 												// Normalized Velocity Value
int norm1,norm2;
int diff1,diff2;											            // For Normalization
int diff_w1, diff_w2;

/**********Encoder Normalization*********/
#define ENORM_MIN 0
#define ENORM_MAX 100
#define ERANGE_MAX 1000
int NocoderL;
int NocoderR;
volatile int SPEED = 500;

/**********SONAR NORMALIZATION*********/
#define NORM_MIN 3														// Minimum Value of Sonar Sensor
#define NORM_MAX 200													// Maximum Value of Sonar Sensor
volatile uint32_t distance1 , distance2, distance3, Ldistance, Rdistance;
int SonarLMAX = 300 , SonarRMAX = 300;

/***************PSD Normalization********************/
#define PSD_MIN 0
#define PSD_MAX 900
uint16_t adcval[10];
uint16_t PSDL[3];
uint16_t PSDR[3];
uint16_t SPSDL[2];
uint16_t SPSDR[2];
uint16_t FrontLPSD;
uint16_t FrontRPSD;
uint16_t DiaLPSD;
uint16_t DiaRPSD;
uint16_t SideLPSD;
uint16_t SideRPSD;
uint16_t PSDLeft;
uint16_t PSDRight;
int PSDdiff1;
int PSDdiff2;

//for delay
volatile int countmode = 0;
volatile int PSDcount = 0;
volatile int Flag = 0;
volatile int Count = 0;

/********ENCODER for estimating current speed********/
float SpeedL, SpeedR;
float Motor_Signal_L;
float Motor_Signal_R;
float Error_L=0;
float Error_R=0;
float Old_Error_L=0;
float Old_Error_R=0;
float Old_Motor_L;
float Old_Motor_R;
float Motor_mean;
float Motor_Speed;

/************PID Control for cornering and avoiding obstacles*************/
#define delta_t 0.02
float TermP_L;
float TermP_R;
float TermI_L;
float TermI_R;
float TermD_L;
float TermD_R;
float KP=  0.005;//0.011;//0.18; //2;//0.02
float KI=  0.0012;//0.004;//0.09; //0;//0.01
float KD=  -0.00003;//0.0005;//0.007; //0;

/***********FOR DELAY_US FUNC**********/
#define Delay_ms     HAL_Delay
#define millis()     HAL_GetTick()
#define SYS_CLOCK    168
#define SYSTICK_LOAD 167999
__IO uint32_t uwTick=0;
extern __IO uint32_t uwTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/******For Blue-tooth communication*******/
void SCI_OutString(char *pt)
{
  char letter;
  while((letter = *pt++)){
     HAL_UART_Transmit(&huart3,&letter, 1,10);
    //SCI_OutChar(letter);
  }
}
void SCI_OutChar(char letter)
{
   HAL_UART_Transmit(&huart3,&letter, 1,10);
}

/*****************For Delay****************/
uint32_t micros() {
  return (uwTick&0x3FFFFF)*1000 + (SYSTICK_LOAD-SysTick->VAL)/SYS_CLOCK;
}

void Delay_us(uint32_t us) {
  uint32_t temp = micros();
  uint32_t comp = temp + us;
  uint8_t  flag = 0;
  while(comp > temp){
    if(((uwTick&0x3FFFFF)==0)&&(flag==0)){
      flag = 1;
    }
    if(flag) temp = micros() + 0x400000UL * 1000;
    else     temp = micros();
  }
}


/**************Ultra-sonic Sensor External Interrupt***********/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //External interrupt for Sonar
{

  /* Prevent unused argument(s) compilation warning */

      switch(GPIO_Pin)
      {
         //left sonar
         case GPIO_PIN_10:{
            static uint32_t ss1=0;
            uint32_t temp1 = GPIOE->IDR & 0x0400;
            switch (temp1) {
              case 0x0400:  // Rising
                 ss1 = micros();
                 break;

              case 0x0000 :  // Falling
                 distance1 = (micros() - ss1) / 58;
                 break;
            }
            break;
         }
         //right sonar
         case GPIO_PIN_12:{
               static uint32_t ss2=0;
               uint32_t temp2 = GPIOE->IDR & 0x1000;

               switch (temp2) {
                 case 0x1000 :  // Rising
                    ss2 = micros();
                    break;

                 case 0x0000 :  // Falling
                    distance2 = (micros() - ss2) / 58;
                    break;
                     }
               break;
            }
      }

  }
void SONAR(){

	////// Sonar Sensor Normalization
	// limit min & max distance
	if(distance1>SonarLMAX) distance1=SonarLMAX;
	if(distance1<3) distance1=3;
	if(distance2>SonarRMAX) distance2=SonarRMAX;
	if(distance2<3) distance2=3;

	Ldistance = distance1;
	Rdistance = distance2;

	diff1 = distance2 - distance1;
	diff2 = distance1 - distance2;

	diff_w1 = (diff1/35)*(diff1/35)*(diff1/35) + diff1;//(x/10)^3 +x);
	diff_w2 = (diff2/35)*(diff2/35)*(diff2/35) + diff2;

    n_v1 = diff_w1 + W1_MIN + PSDdiff1; // Sonar + PSD + default Speed
    n_v2 = diff_w2 + W2_MIN + PSDdiff2;

    // limit min & max value.
    if(n_v1>1000)n_v1=1000;
    if(n_v2>1000)n_v2=1000;
    if(n_v1<1)n_v1=0;
    if(n_v2<1)n_v2=0;

}

void PSD(){

	////// PSD Sensor Normalization
	//adcval[0]~adcval[5] = Long distance PSD
	//adcval[6]~adcval[9] = Short distance PSD

	//short distance PSD (Analogue to specific range of digital value)
//	if (adcval[6]>700) adcval[6] = 1;
//	else adcval[6] =0;
//	if (adcval[7]>700) adcval[7] = 1;
//		else adcval[7] =0;
//	if (adcval[8]>700) adcval[8] = 1;
//		else adcval[8] =0;
//	if (adcval[9]>700) adcval[9] = 1;
//		else adcval[9] =0;

	PSDL[0]=adcval[0];
	PSDL[1]=adcval[1];
	PSDL[2]=adcval[2];
	PSDR[0]=adcval[3];
	PSDR[1]=adcval[4];
	PSDR[2]=adcval[5];

	SPSDL[0]=adcval[6];
	SPSDL[1]=adcval[7];
	SPSDR[0]=adcval[8];
	SPSDR[1]=adcval[9];

	//PSD normalization
	FrontLPSD = ((float)PSDL[0]-PSD_MIN)/(PSD_MAX-PSD_MIN)*300;	//PSD Front
	FrontRPSD = ((float)PSDR[0]-PSD_MIN)/(PSD_MAX-PSD_MIN)*300;

	DiaLPSD = ((float)PSDL[1]-PSD_MIN)/(PSD_MAX-PSD_MIN)*350;	//PSD Diagonal
	DiaRPSD = ((float)PSDR[1]-PSD_MIN)/(PSD_MAX-PSD_MIN)*350;

	SideLPSD = ((float)PSDL[2]-PSD_MIN)/(PSD_MAX-PSD_MIN)*400;	//PSD Side
	SideRPSD = ((float)PSDR[2]-PSD_MIN)/(PSD_MAX-PSD_MIN)*400;

	//sum of PSD for left and right
	PSDLeft = FrontLPSD + DiaLPSD + SideLPSD;
	PSDRight = FrontRPSD + DiaRPSD + SideRPSD;

	//differences between left and right PSD values
	PSDdiff1 = PSDLeft - PSDRight;
	PSDdiff2 = PSDRight - PSDLeft;

}
void Delay_DesiredSpeed(int x, int y)
{
	////// Maintain Desired Speed
	//maintain a target speed for a certain period of time using external interrupt(1cycle = 20ms)
	Flag = 1;

	while(Count < 5){
		n_v1 = x;
		n_v2 = y;
		PID(n_v1,n_v2,encoderL,encoderR);
	}

	Flag = 0;
	Count = 0;

}

void Driving_Control(){

	////// Obstacle avoidance driving
	// Detect obstacle on the left side
	if(SPSDL[0] || SPSDL[1]){

		countmode=1;
		// Break for 1sec
		while(PSDcount < 50){
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
		}

		PSDcount=0;
		countmode=0;

		while(SPSDL[0] || SPSDL[1]){
			Delay_DesiredSpeed(n_v1, n_v2);
		}
	}

	// Detect obstacle on the right side
	if(SPSDR[0] || SPSDR[1]){

		countmode=1;

		// Break for 1sec
		while(PSDcount < 50){
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
		}
		PSDcount=0;
		countmode=0;


		while(SPSDR[0] || SPSDR[1]){
			Delay_DesiredSpeed(n_v1, n_v2);
		}
	}

	////// Regular driving
	PID(n_v1,n_v2,encoderL,encoderR);
}

void PSD_Bluetooth(){

  	itoa(PSDL[0], Buf1, 10);
  	SCI_OutChar('F');
  	SCI_OutChar('L');
  	SCI_OutString(Buf1);
  	HAL_UART_Transmit(&huart3,&space,1,10);

  	itoa(PSDL[1], Buf2, 10);
  	SCI_OutChar('F');
  	SCI_OutChar('R');
  	SCI_OutString(Buf2);
  	HAL_UART_Transmit(&huart3,&space,1,10);

  	itoa(PSDL[2], Buf3, 10);
  	SCI_OutChar('D');
  	  	SCI_OutChar('L');
  	SCI_OutString(Buf3);
  	HAL_UART_Transmit(&huart3,&space,1,10);

  	itoa(PSDR[0], Buf4, 10);
  	SCI_OutChar('D');
  	  	SCI_OutChar('R');
  	SCI_OutString(Buf4);
  	HAL_UART_Transmit(&huart3,&space,1,10);

  	itoa(PSDR[1], Buf5, 10);
  	SCI_OutChar('S');
  	  	SCI_OutChar('L');
  	SCI_OutString(Buf5);
  	HAL_UART_Transmit(&huart3,&space,1,10);

  	itoa(PSDR[2], Buf6, 10);
  	SCI_OutChar('S');
  	  	SCI_OutChar('R');
  	SCI_OutString(Buf6);

  	HAL_UART_Transmit(&huart3,&enter1,1,10);
  	HAL_UART_Transmit(&huart3,&enter2,1,10);
}

void Bluetooth(int first, int second, int third, int forth){

	itoa(first, Buf1, 10);
	SCI_OutChar('A');
	SCI_OutString(Buf1);
	HAL_UART_Transmit(&huart3,&space,1,10);

	itoa(second, Buf2, 10);
	SCI_OutChar('B');
	SCI_OutString(Buf2);
	HAL_UART_Transmit(&huart3,&space,1,10);

	itoa(third, Buf3, 10);
	SCI_OutChar('C');
	SCI_OutString(Buf3);
	HAL_UART_Transmit(&huart3,&space,1,10);

	itoa(forth, Buf4, 10);
	SCI_OutChar('D');
	SCI_OutString(Buf4);
  	HAL_UART_Transmit(&huart3,&enter1,1,10);
  	HAL_UART_Transmit(&huart3,&enter2,1,10);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	//Timer interrupt every 20ms
{

	if(htim->Instance == TIM6){
		encoderL = TIM2->CNT;
		TIM2->CNT=0;
		encoderR = TIM4->CNT;
		TIM4->CNT=0;

		if(encoderL>200)encoderL=200;
		if(encoderR>200)encoderR=200;
		if(encoderL<1)encoderL=0;
		if(encoderR<1)encoderR=0;

		PSD();
		SONAR();

//		if(Flag==1){
//			Count++;
//		}
//
//		if(countmode==1){
//			PSDcount++;
//		}


		//			itoa(Motor_Speed, Buf4, 10);
		//			SCI_OutChar('S');
		//			SCI_OutString(Buf4);
		//			HAL_UART_Transmit(&huart3,&enter1,1,10);
		//			HAL_UART_Transmit(&huart3,&enter2,1,10);


		//Bluetooth(distance1,distance2,n_v1,n_v2);
		if(Mode_Bluetooth==1) {
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
//	//		Bluetooth(distance1,distance2,n_v1,n_v2);
//			PSD_Bluetooth();
		}

	}
}

void PID_Init() {

	Old_Motor_R = 0;
	Old_Error_R = 0;

}

void PID(unsigned int x,unsigned int y,unsigned int m,unsigned int n) {

	////// PID controller
	unsigned int Desired_Speed_L;
	unsigned int Desired_Speed_R;

	Desired_Speed_L = x;
	Desired_Speed_R = y;

	SpeedL =270.11 * exp(0.0065*m);  //measured speed for left
	SpeedR =271.38 * exp(0.0065*n); //measured speed for right

	Error_L = Desired_Speed_L - SpeedL;
	Error_R = Desired_Speed_R - SpeedR;

	TermP_L = Error_L;
	TermP_R = Error_R;

	TermI_L = Error_L*delta_t;  // every 20ms for triggering encoder => 0.02 for delta_t;
	TermI_R = Error_R*delta_t;

	TermD_L = (Error_L - Old_Error_L)/delta_t;
	TermD_R = (Error_R - Old_Error_R)/delta_t;

	Motor_Signal_L = KP * TermP_L + KI * TermI_L + KD*TermD_L + Old_Motor_L;
	Motor_Signal_R = KP * TermP_R + KI * TermI_R + KD*TermD_R + Old_Motor_R;

	//limit speed
	if(Motor_Signal_L<100) Motor_Signal_L=100; //400
	if(Motor_Signal_L>1000) Motor_Signal_L=1000; //여기바꿈
	if(Motor_Signal_R<100) Motor_Signal_R=100; //300
	if(Motor_Signal_R>1000) Motor_Signal_R=1000;

	Old_Error_L = Error_L;
	Old_Error_R = Error_R;

	Old_Motor_L = Motor_Signal_L;
	Old_Motor_R = Motor_Signal_R;

	TIM3->CCR1 = Motor_Signal_L; //actual PWM for motor
	TIM3->CCR2 = Motor_Signal_R;

	Motor_mean = (m + n)/2;
	Motor_Speed = 0.2371 * Motor_mean + 0.3324;

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  //Initialize for PSD
  HAL_ADC_Start_DMA(&hadc1,&adcval[0],10);

  //Initialize for UART
  HAL_UART_Receive_IT(&huart3, &rx,1);

  //Initialize for motor PWN
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  //Initialize for Sonar
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//left
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//right

  //Initialize for motor direction
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);

  //Initialize for encoder count value
  TIM2->CNT = 0;  // Initial CNT value
  TIM4->CNT = 0;  // Initial CNT value

  //Initialize for Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);

  //Initialize for timer interrupt initialization for Encoder (20ms)
  HAL_TIM_Base_Start_IT(&htim6);
  PID_Init();

  //Initialize for blue-tooth mode
  Mode_Bluetooth = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

//	  Driving_Control();
//	  PID(n_v1,n_v2,encoderL,encoderR);
	  if(Mode_Bluetooth==0){
//		  Delay_DesiredSpeed(n_v1, n_v2);
//		  TIM3->CCR1 = 0;
//		  TIM3->CCR2 = 0;
	  	  PID(n_v1,n_v2,encoderL,encoderR);
	  }

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	char g='g';

	if(huart->Instance == USART3){

		 HAL_UART_Receive_IT(&huart3,&rx,1);

		 if(rx=='w'){
			 Mode_Bluetooth = 0;
		 }

		 if(rx=='s') Mode_Bluetooth = 1;

		 if(rx=='q') {
			 SonarLMAX = Rdistance;
		 }
		 if(rx=='e'){
			 SonarRMAX = Ldistance;
		 }
		 if(rx=='l') SonarRMAX = 35;
		 if(rx=='r') SonarLMAX = 70;
		 if(rx=='o') {
			 SonarLMAX = 300;
			 SonarRMAX = 300;
		 }
		 if(rx=='z') {
			 W1_MIN += 50;
			 W2_MIN += 50;
		 }
		 if(rx=='x') {
			 W1_MIN -= 50;
			 W2_MIN -= 50;
		}
	}

 }//전체인터럽트 끝나는 괄호


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
