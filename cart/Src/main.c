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
#include "adc.h"
#include "dma.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
// author          : Je Hyeon Yu, Ji Hoo Chun, Yun Hwan Kim, Seung Jik Kim, Yung-in Kim
// date            : 20190807
// version         : 0.3
// TrueSTUDIO	   : 9.3.0
// notes           : This stm cart controller is an implementation of a cart controller for
//					 self-driving in the C Programming Language.
// ==============================================================================
#include <stdlib.h>
#include <math.h>
#include <stm32f4xx_hal_gpio.h>
#include "stm32f4xx_it.h"
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
/* Private variables ---------------------------------------------------------*/
/************BLUETOOTH BUFFER**********/
char Buf1[30];                                 // For blue-tooth buffer
char Buf2[30];                                 // For blue-tooth buffer
char Buf3[30];                                 // For blue-tooth buffer
char Buf4[30];
char Buf5[30];
char Buf6[30];

// For blue-tooth buffer
char ENTER1 = 13;                                // For Enter ASCII code
char ENTER2 = 10;                                // For Enter ASCII code
char SPACE = 32;                                 // For Space ASCII code
char RX;                                   	   // For received signal
int SPEEDY = 600;

/********VELOCITY NORMALIZATION********/
#define PWMMAX 1000                        	   // For Maximum Value of PWM
#define PWMMIN 0                       		   // For Minimum Value of PWM
int WMIN = 600;                                // Motor Initial Velocity
int NORMVL, NORMVR;                            // Normalized Velocity Value

/**********SONAR NORMALIZATION*********/
#define SONARMIN 3                             // Minimum Value of Sonar Sensor
#define SONARMAX 300                           // Maximum Value of Sonar Sensor
volatile uint32_t SONARLEFT, SONARRIGHT;       // Real Ultra-sonic Sensor's values
int LDIFF, RDIFF;                              // For Difference between Left & Right Velocity
int SONARDIFFL, SONARDIFFR;                    // For Normalized value of Sonar Sensor

/*********Encoder Normalization********/
#define ENCODERZERO 0                          // Zero Value of Encoder
#define ENCODERMIN 1                           // Minimum Value of Encoder
#define ENCODERMAX 200                         // Maximum Value of Encoder
int LENCODER;                                  // Store Left Encoder Value
int RENCODER;                                  // Store Right Encoder Value

/**********PSD Normalization***********/
#define PSDMIN 0                               // Minimum Value of PSD Sensor
#define PSDMAX 900                             // Maximum Value of PSD Sensor
uint16_t adcval[6];                       	   // Receive ADC Value from Memory
uint16_t LPSD[3];                      		   // Real Left PSD Sensor's values
uint16_t RPSD[3];                     		   // Real Right PSD Sensor's values
uint16_t FrontLPSD;                            // Normalized value of Left front PSD Sensor
uint16_t FrontRPSD;                            // Normalized value of Right front PSD Sensor
uint16_t DiaLPSD;                              // Normalized value of Left diagonal PSD Sensor
uint16_t DiaRPSD;                              // Normalized value of Right diagonal PSD Sensor
uint16_t SideLPSD;                             // Normalized value of Left side PSD Sensor
uint16_t SideRPSD;                             // Normalized value of Right side PSD Sensor
uint16_t LPSDSUM;                              // Sum of all Left PSD Sensor normalized values
uint16_t RPSDSUM;                              // Sum of all Left PSD Sensor normalized values
int LPSDDIFF;                              	   // Difference between LPSDSUM and RPSDSUM
int RPSDDIFF;                                  // Difference between RPSDSUM and LPSDSUM

/*ENCODER for estimating current speed**/
float LSPEED, RSPEED;                          // Real Speed of Left & Right Motor
float LMOTOR = 0, RMOTOR = 0;                  // Input PWM value of Left & Right Motor
float LERROR = 0, RERROR = 0;                  // Difference between Real Speed of Left & Right Motor
float LOLDERROR = 0, ROLDERROR = 0;            // Old Difference between Real Speed of Left & Right Motor
float LOLDMOTOR = 0, ROLDMOTOR = 0;            // Old Input PWM value of Left & Right Motor

/*PID Control for cornering and avoiding obstacles*/
#define DELTA 0.02                              // Control Period
float LPTERM;                                   // Proportional Term of Left Motor PID
float RPTERM;                                   // Proportional Term of Right Motor PID
float LITERM;                                   // Integral Term of Left Motor PID
float RITERM;                                   // Integral Term of Right Motor PID
float LDTERM;                                   // Differential Term of Left Motor PID
float RDTERM;                                   // Differential Term of Right Motor PID
float KP = 0.005;                               // Control variable of Proportional term
float KI = 0.0012;                              // Control variable of Integral term
float KD = -0.00003;                            // Control variable of Differential term

/***********FOR DELAY_US FUNC**********/
#define Delay_ms     HAL_Delay                  // Redefined HAL_Delay to Delay_ms
#define millis()     HAL_GetTick()              // Redefined HAL_GetTick to millis
#define SYS_CLOCK    168                        // Redefined SYS_CLOCK to 168
#define SYSTICK_LOAD 167999                     // Redefined SYSTICK_LOAD to 167999
__IO uint32_t uwTick = 0;                         // Redefined for user convenience
extern __IO uint32_t uwTick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*****For Blue-tooth communication*****/
void SCI_OutString(char* pt)                                            // This function is used for print String type
{
    char letter;
    while ((letter = *pt++)) {
        HAL_UART_Transmit(&huart3, &letter, 1, 10);
    }
}

void SCI_OutChar(char letter)                                           // This function is used for print Character type
{
    HAL_UART_Transmit(&huart3, &letter, 1, 10);
}

/***************For Delay**************/
uint32_t micros()                                                       // This function is used for get micros unit tick from System tick
{
    return (uwTick & 0x3FFFFF) * 1000 + (SYSTICK_LOAD - SysTick->VAL) / SYS_CLOCK;
}

void Delay_us(uint32_t us)                                              // This function is used for make micro unit delay
{
    uint32_t temp = micros();
    uint32_t comp = temp + us;
    uint8_t  flag = 0;
    while (comp > temp) {
        if (((uwTick & 0x3FFFFF) == 0) && (flag == 0)) {
            flag = 1;
        }
        if (flag) temp = micros() + 0x400000UL * 1000;
        else     temp = micros();
    }
}


/**Ultra-sonic Sensor External Interrupt**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)                         // This function is used for Ultra-Sonic Sensor External Interrupt
{
    /* Prevent unused argument(s) compilation warning */
    switch (GPIO_Pin)
    {
    case GPIO_PIN_10: {                                            // Check Received Signal on Echo Pin of Left Ultra-Sonic Sensor
        static uint32_t LEFTTICK = 0;                                // Static assignment to stored micro unit ticks when Echo Pin received trigger signal
        uint32_t LEFTECHO = GPIOE->IDR & 0x0400;                   // Check whether Echo Pin Received Signal or not, Compare GPIO PIN E 10 & 0x0000 0100 0000 0000
        switch (LEFTECHO) {
        case 0x0400:                                             // If Echo Pin received Signal from Trigger Signal (Rising Edge)
            LEFTTICK = micros();                                  // Save the system tick as soon as it is received Trigger Signal
            break;

        case 0x0000:                                             // If Echo Pin doesn't received Signal from Trigger Signal (Falling Edge)
            SONARLEFT = (micros() - LEFTTICK) / 58;                // Calculate the distance from Tick (58 is experimental value)
            break;
        }
        break;
    }

                    //Right Ultrasonic Sensor
    case GPIO_PIN_12: {                                            // Check Received Signal on Echo Pin of Right Ultra-Sonic Sensor
        static uint32_t RIGHTTICK = 0;                            // Static assignment to stored micro unit ticks when Echo Pin received trigger signal
        uint32_t RHIGTECHO = GPIOE->IDR & 0x1000;               // Check whether Echo Pin Received Signal or not, Compare GPIO PIN E 12 & 0x0001 0000 0000 0000
        switch (RHIGTECHO) {
        case 0x1000:                                          // If Echo Pin received Signal from Trigger Signal (Rising Edge)
            RIGHTTICK = micros();                              // Save the system tick as soon as it is received Trigger Signal
            break;

        case 0x0000:                                          // If Echo Pin doesn't received Signal from Trigger Signal (Falling Edge)
            SONARRIGHT = (micros() - RIGHTTICK) / 58;          // Calculate the distance from Tick (58 is experimental value)
            break;
        }
        break;
    }
    }
}

void SONAR() {

    ////// Ultra-Sonic Sensor Normalization

    if (SONARLEFT > SONARMAX) SONARLEFT = SONARMAX;                      // Limit the Maximum value of Left Ultra-Sonic Sensor
    if (SONARLEFT < SONARMIN) SONARLEFT = SONARMIN;                      // Limit the Minimum value of Left Ultra-Sonic Sensor
    if (SONARRIGHT > SONARMAX) SONARRIGHT = SONARMAX;                    // Limit the Maximum value of Right Ultra-Sonic Sensor
    if (SONARRIGHT < SONARMIN) SONARRIGHT = SONARMIN;                    // Limit the Minimum value of Right Ultra-Sonic Sensor

    LDIFF = SONARRIGHT - SONARLEFT;                                     // Difference of Right & Left, using Left Motor control
    RDIFF = SONARLEFT - SONARRIGHT;                                     // Difference of Left & Right, using Right Motor control

    SONARDIFFL = (LDIFF / 35) * (LDIFF / 35) * (LDIFF / 35) + LDIFF;    // Apply normalizing equation (x/10)^3 +x) for Left Motor control
    SONARDIFFR = (RDIFF / 35) * (RDIFF / 35) * (RDIFF / 35) + RDIFF;    // Apply normalizing equation (x/10)^3 +x) for Left Motor control

    NORMVL = WMIN + SONARDIFFL + LPSDDIFF;                              // Default Speed + Left Ultra-sonic Sensor + Left PSD Sensor
    NORMVR = WMIN + SONARDIFFR + RPSDDIFF;                              // Default Speed + Right Ultra-sonic Sensor + Right PSD Sensor

      //Limit Motor PWM min & max value.
    if (NORMVL > PWMMAX) NORMVL = PWMMAX;                                // Limit the Maximum value of Left Normalization value which is used for Left Motor PWM control
    if (NORMVR > PWMMAX) NORMVR = PWMMAX;                                // Limit the Maximum value of Right Normalization value which is used for Right Motor PWM control
    if (NORMVL < PWMMIN) NORMVL = PWMMIN;                                // Limit the Minimum value of Left Normalization value which is used for Left Motor PWM control
    if (NORMVR < PWMMIN) NORMVR = PWMMIN;                                // Limit the Minimum value of Right Normalization value which is used for Right Motor PWM control



}

void PSD() {

    ////// PSD Sensor Normalization

    LPSD[0] = adcval[0];                                                 // Store ADC Value of Left Front PSD Sensor
    LPSD[1] = adcval[1];                                       		    // Store ADC Value of Left Diagonal PSD Sensor
    LPSD[2] = adcval[2];                                       		    // Store ADC Value of Left Side PSD Sensor
    RPSD[0] = adcval[3];                                        		    // Store ADC Value of Right Front PSD Sensor
    RPSD[1] = adcval[4];                                       	 	    // Store ADC Value of Right Diagonal PSD Sensor
    RPSD[2] = adcval[5];                                       	        // Store ADC Value of Right Side PSD Sensor

    FrontLPSD = ((float)LPSD[0] - PSDMIN) / (PSDMAX - PSDMIN) * 250;     // Normalized equation of Left Front PSD Sensor
    FrontRPSD = ((float)RPSD[0] - PSDMIN) / (PSDMAX - PSDMIN) * 250;     // Normalized equation of Right Front PSD Sensor

    DiaLPSD = ((float)LPSD[1] - PSDMIN) / (PSDMAX - PSDMIN) * 300;       // Normalized equation of Left Diagonal PSD Sensor
    DiaRPSD = ((float)RPSD[1] - PSDMIN) / (PSDMAX - PSDMIN) * 300;       // Normalized equation of Right Diagonal PSD Sensor

    SideLPSD = ((float)LPSD[2] - PSDMIN) / (PSDMAX - PSDMIN) * 350;      // Normalized equation of Left Side PSD Sensor
    SideRPSD = ((float)RPSD[2] - PSDMIN) / (PSDMAX - PSDMIN) * 350;      // Normalized equation of Right Side PSD Sensor

    LPSDSUM = FrontLPSD + DiaLPSD + SideLPSD;                       	    // Total Left PSD Sensors value
    RPSDSUM = FrontRPSD + DiaRPSD + SideRPSD;                            // Total Right PSD Sensors value

    LPSDDIFF = LPSDSUM - RPSDSUM;                                 	    // Difference between Total Left PSD Sensors value and Total Right PSD Sensors value
    RPSDDIFF = RPSDSUM - LPSDSUM;                                 	    // Difference between Total Right PSD Sensors value and Total Left PSD Sensors value
}

void PSDBluetooth() {                                           		    // This function is used for blue-tooth print of 6 PSD sensors (just call this if you want)

    itoa(LPSD[0], Buf1, 10);
    SCI_OutChar('F');
    SCI_OutChar('L');
    SCI_OutString(Buf1);
    HAL_UART_Transmit(&huart3, &SPACE, 1, 10);

    itoa(LPSD[1], Buf2, 10);
    SCI_OutChar('D');
    SCI_OutChar('L');
    SCI_OutString(Buf2);
    HAL_UART_Transmit(&huart3, &SPACE, 1, 10);

    itoa(LPSD[2], Buf3, 10);
    SCI_OutChar('S');
    SCI_OutChar('L');
    SCI_OutString(Buf3);
    HAL_UART_Transmit(&huart3, &SPACE, 1, 10);

    itoa(RPSD[0], Buf4, 10);
    SCI_OutChar('F');
    SCI_OutChar('R');
    SCI_OutString(Buf4);
    HAL_UART_Transmit(&huart3, &SPACE, 1, 10);

    itoa(RPSD[1], Buf5, 10);
    SCI_OutChar('D');
    SCI_OutChar('R');
    SCI_OutString(Buf5);
    HAL_UART_Transmit(&huart3, &SPACE, 1, 10);

    itoa(RPSD[2], Buf6, 10);
    SCI_OutChar('S');
    SCI_OutChar('R');
    SCI_OutString(Buf6);
    HAL_UART_Transmit(&huart3, &ENTER1, 1, 10);
    HAL_UART_Transmit(&huart3, &ENTER2, 1, 10);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)            // Timer interrupt every 20ms to collect Encoder Value
{

    if (htim->Instance == TIM6) {

        LENCODER = TIM2->CNT;
        TIM2->CNT = 0;
        RENCODER = TIM4->CNT;
        TIM4->CNT = 0;
        LSPEED = 270.11 * exp(0.0065 * LENCODER);  					   //measured speed for left
        RSPEED = 271.38 * exp(0.0065 * RENCODER); 					   //measured speed for right
        if (LENCODER > ENCODERMAX) LENCODER = ENCODERMIN;
        if (RENCODER > ENCODERMAX) RENCODER = ENCODERMIN;
        if (LENCODER < ENCODERMIN) LENCODER = ENCODERZERO;
        if (RENCODER < ENCODERMIN) RENCODER = ENCODERZERO;

        SONAR();
        PSD();


    }
}

void PID(unsigned int x, unsigned int y, unsigned int m, unsigned int n) {

    ////// PID controller
    unsigned int LDESIREDSPEED;
    unsigned int RDESIREDSPEED;

    LDESIREDSPEED = x;
    RDESIREDSPEED = y;

    LSPEED = 270.11 * exp(0.0065 * m);  								   //measured speed for left
    RSPEED = 271.38 * exp(0.0065 * n); 								   //measured speed for right

    LERROR = LDESIREDSPEED - LSPEED;
    RERROR = RDESIREDSPEED - RSPEED;

    LPTERM = LERROR;
    RPTERM = RERROR;

    LITERM = LERROR * DELTA;  										   // every 20ms for triggering encoder => 0.02 for delta_t;
    RITERM = RERROR * DELTA;

    LDTERM = (LERROR - LOLDERROR) / DELTA;
    RDTERM = (RERROR - ROLDERROR) / DELTA;

    LMOTOR = KP * LPTERM + KI * LITERM + KD * LDTERM + LOLDMOTOR;
    RMOTOR = KP * RPTERM + KI * RITERM + KD * RDTERM + ROLDMOTOR;

    //limit speed
    if (LMOTOR < 100)  LMOTOR = 100;
    if (LMOTOR > 1000) LMOTOR = 1000;
    if (RMOTOR < 100)  RMOTOR = 100;
    if (RMOTOR > 1000) RMOTOR = 1000;

    LOLDERROR = LERROR;
    ROLDERROR = RERROR;

    LOLDMOTOR = LMOTOR;
    ROLDMOTOR = RMOTOR;

    TIM3->CCR1 = LMOTOR; //actual PWM for motor
    TIM3->CCR2 = RMOTOR;
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
    MX_ADC1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_USART3_UART_Init();
    MX_USB_HOST_Init();

    /* Initialize interrupts */
    MX_NVIC_Init();
    /* USER CODE BEGIN 2 */

    //Initialize for PSD
    HAL_ADC_Start_DMA(&hadc1, &adcval[0], 6);

    //Initialize for UART
    HAL_UART_Receive_IT(&huart3, &RX, 1);

    //Initialize for motor PWN
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
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

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        PID(NORMVL, NORMVR, LENCODER, RENCODER);
        /* USER CODE END WHILE */
        MX_USB_HOST_Process();

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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
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
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
    /* EXTI15_10_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    /* TIM6_DAC_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    /* USART3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{

    if (huart->Instance == USART3) {

        HAL_UART_Receive_IT(&huart3, &RX, 1);

        if (RX == 'z') {
            CAMVAL -= 15;
        }
        if (RX == 'x') {
            CAMVAL += 15;
        }
    }

}//전체인터럽트 끝나는 괄호

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
