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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdio.h>
#include "ssd1306.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{

	norte,
	sul,
	leste,
	oeste

} direcoes;


typedef enum{

	AguardaBotao,
	Frente,
	LeSensor,
	ViraDireita,
	ViraEsquerda,
	Achou

} BarataTonta;

typedef enum{

	// falta implementar
	WaitingButton,
	Forward,
	Turn,



} BlindSearch;


typedef struct{

	direcoes Sentido_Atual;
	uint8_t Posicao_Atual[2];
	int Sensor_Frente;
	int Sensor_Esquerda;
	int Sensor_Direita;
	bool Sensor_Chao;

} robo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

# define DO 528
# define RE 592
# define MI 665
# define FA 704
# define SOL 790
# define LA 888
# define SI 996

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RightMotor */
osThreadId_t RightMotorHandle;
const osThreadAttr_t RightMotor_attributes = {
		.name = "RightMotor",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LeftMotor */
osThreadId_t LeftMotorHandle;
const osThreadAttr_t LeftMotor_attributes = {
		.name = "LeftMotor",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
		.name = "StateMachine",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

int VariacaoSensor = 10;
uint16_t MediaSensorD[10];
uint16_t MediaSensorE[10];
uint16_t MediaSensorF[10];

// encoders
int LeftEncoderCount=0;
int RightEncoderCount =0;


long LeftEncoderCountTotal = 0;
long RightEncoderCountTotal = 0;
float k;
float y;

// flags movimentos
bool Flag_RightMotor_Mov_Forward=false;
bool Flag_LeftMotor_Mov_Forward=false;
uint8_t Count_Mov_Forward = 35;

bool Flag_RightMotor_Mov_Backward=false;
bool Flag_LeftMotor_Mov_Backward=false;
uint8_t Count_Mov_Backward = 35;

bool Flag_LeftMotor_Mov_RotateRight=false;
bool Flag_RightMotor_Mov_RotateRight=false;
uint8_t Count_Mov_RotateRight = 52;

bool Flag_LeftMotor_Mov_RotateLeft=false;
bool Flag_RightMotor_Mov_RotateLeft=false;
uint8_t Count_Mov_RotateLeft = 52;



// demais flags
bool Baratinha=true;
bool Flag_EncoderOrDistance = true;
bool achou = false;

// PWM motores
uint8_t MaxPWM_Right = 35;
uint8_t MaxPWM_Left = 32;

//Distancia para tomada de ação
uint16_t MaxDistanciaLeituraSensorSonico = 250; // centimentros
uint16_t DistanciaMinima = 15;

//Sensor anterior
int SensorFrenteAnterior = 0;
int SensorEsquerdaAnterior = 0;
int SensorDireitaAnterior = 0;

// structs e enums
robo Walle = { .Sentido_Atual = norte, .Posicao_Atual = {0,0} };
BarataTonta Barata = AguardaBotao;
BlindSearch BS = WaitingButton;

//timeouts
uint16_t Timeout=1000;
uint16_t ContadorRotateRigt=5;
//uint16_t TimeOutMotorDireito=10000;

//contagem
int ContagemRegressiva = 5;
int ContagemRotate = 0;
bool FlagInicial = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void *argument);
void StartRightMotor(void *argument);
void StartLeftMotor(void *argument);
void StartStateMachine(void *argument);

/* USER CODE BEGIN PFP */

uint16_t Read_Ultrasonic(GPIO_TypeDef* TriggerPort, uint16_t TriggerPin, GPIO_TypeDef* EchoPort, uint16_t EchoPin);

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
	MX_TIM4_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */


	//inicializacao do display
	ssd1306_Init();
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();


	//inicializacao dos sensores
	HAL_TIM_Base_Start(&htim3);
	HAL_GPIO_WritePin(SensorFrenteTrigger_GPIO_Port, SensorFrenteTrigger_Pin , GPIO_PIN_RESET);  // pull the TRIG pin low
	HAL_GPIO_WritePin(SensorEsquerdaTrigger_GPIO_Port, SensorEsquerdaTrigger_Pin , GPIO_PIN_RESET);  // pull the TRIG pin low
	HAL_GPIO_WritePin(SensorDireitoTrigger_GPIO_Port, SensorDireitoTrigger_Pin , GPIO_PIN_RESET);  // pull the TRIG pin low



	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

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
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of RightMotor */
	RightMotorHandle = osThreadNew(StartRightMotor, NULL, &RightMotor_attributes);

	/* creation of LeftMotor */
	LeftMotorHandle = osThreadNew(StartLeftMotor, NULL, &LeftMotor_attributes);

	/* creation of StateMachine */
	StateMachineHandle = osThreadNew(StartStateMachine, NULL, &StateMachine_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	Walle.Sensor_Frente = 0;
	Walle.Sensor_Direita = 0;
	Walle.Sensor_Esquerda = 0;
	while (1)
	{
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

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
	htim2.Init.Prescaler = 8400;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigOC.Pulse = 5;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 840;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 100-1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 50-1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SensorDireitoTrigger_Pin|SensorFrenteTrigger_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SensorEsquerdaTrigger_GPIO_Port, SensorEsquerdaTrigger_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Led_Pin */
	GPIO_InitStruct.Pin = Led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Key_Pin */
	GPIO_InitStruct.Pin = Key_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SensorDireitoEcho_Pin SensorFrenteEcho_Pin SensorChaoTras_Pin */
	GPIO_InitStruct.Pin = SensorDireitoEcho_Pin|SensorFrenteEcho_Pin|SensorChaoTras_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SensorDireitoTrigger_Pin SensorFrenteTrigger_Pin */
	GPIO_InitStruct.Pin = SensorDireitoTrigger_Pin|SensorFrenteTrigger_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Encoder_Direito_Pin Encoder_Esquerdo_Pin */
	GPIO_InitStruct.Pin = Encoder_Direito_Pin|Encoder_Esquerdo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SensorChao_Pin */
	GPIO_InitStruct.Pin = SensorChao_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SensorChao_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SensorEsquerdaEcho_Pin */
	GPIO_InitStruct.Pin = SensorEsquerdaEcho_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SensorEsquerdaEcho_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SensorEsquerdaTrigger_Pin */
	GPIO_InitStruct.Pin = SensorEsquerdaTrigger_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SensorEsquerdaTrigger_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



// buzzer
void PlayBuzzer(int Freq, int duration){
	/*
	 * 1 contagem no ARR = 10000Hz. 10 contagens = 1000Hz. 100 contagens = 100 Hz....
	 * CCR1 = ARR/2 para ter onda quadrada
	 */
	TIM2->ARR = 10000/Freq;
	TIM2->CCR1 = 5000/Freq;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	if(Freq >5000)
		return;

	TIM2->ARR = 10000/Freq;
	TIM2->CCR1 = 5000/Freq;

	osDelay(duration);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

}

void Printa1segTela(){
	char text[20] = {};
	ssd1306_SetCursor(56,25);
	sprintf(text,"%d",ContagemRegressiva--);

	if(FlagInicial){
		ssd1306_Fill(White);
		ssd1306_WriteString(text, Font_16x26, Black);
		FlagInicial = false;
	}
	else{
		ssd1306_Fill(Black);
		ssd1306_WriteString(text, Font_16x26, White);
		FlagInicial = true;
	}

	ssd1306_UpdateScreen();

}

void Buzzer5seg(){
	Printa1segTela();
	PlayBuzzer(500, 500);
	osDelay(500);
	Printa1segTela();
	PlayBuzzer(500, 500);
	osDelay(500);
	Printa1segTela();
	PlayBuzzer(500, 500);
	osDelay(500);
	Printa1segTela();
	PlayBuzzer(500, 500);
	osDelay(500);
	Printa1segTela();
	PlayBuzzer(500, 500);
	osDelay(500);
}

void BuzzerTocaMusica(){
	PlayBuzzer(MI, 200);
	osDelay(25);
	PlayBuzzer(MI, 200);
	osDelay(25);
	PlayBuzzer(RE, 1000);
	osDelay(200);
	PlayBuzzer(RE, 200);
	osDelay(25);
	PlayBuzzer(RE, 200);
	osDelay(25);
	PlayBuzzer(DO, 1000);
	osDelay(200);
}
void Buzzer_InitialSong()
{
	Buzzer1seg();
	Buzzer1seg();
	Buzzer1seg();
	Buzzer1seg();
	Buzzer1seg();
}



void Buzzer_FinishSong()
{
	PlayBuzzer(200, 100);
	osDelay(10);
	PlayBuzzer(500, 100);
	osDelay(10);
	PlayBuzzer(1000, 150);
	osDelay(500);
	PlayBuzzer(1000, 1000);
}



// display


void Print_Ajustando(){
	char text[20] = {};

	ssd1306_Fill(Black);

	ssd1306_SetCursor(0,0);
	strcpy(text,"Modo 'Achou'");
	ssd1306_WriteString(text, Font_7x10, White);

	ssd1306_SetCursor(0,20);
	strcpy(text,"Ajustando...");
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_UpdateScreen();
}

void Print_Achou(){

	char text[20] = {};
	ssd1306_Fill(Black);


	ssd1306_SetCursor(0,15);
	strcpy(text,"Achou");
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_SetCursor(0,30);
	strcpy(text," o ");
	ssd1306_WriteString(text, Font_11x18, White);


	ssd1306_SetCursor(0,45);
	strcpy(text,"objetivo!");
	ssd1306_WriteString(text, Font_11x18, White);


	ssd1306_UpdateScreen();
}


void Print_Direcao(){
	char text[20] = {};

	ssd1306_Fill(Black);

	if(Walle.Sensor_Frente<SensorFrenteAnterior && (SensorFrenteAnterior-Walle.Sensor_Frente)>VariacaoSensor){
		ssd1306_SetCursor(0,0);
		strcpy(text,"Frente");
		ssd1306_WriteString(text, Font_7x10, White);
	}
	/*else{
		ssd1306_SetCursor(0,0);
		sprintf(text,"nao Frente %d",SensorFrenteAnterior);
		ssd1306_WriteString(text, Font_7x10, White);
	}*/


	if(Walle.Sensor_Frente>SensorFrenteAnterior && (Walle.Sensor_Frente-SensorFrenteAnterior)>VariacaoSensor ){
		ssd1306_SetCursor(0,15);
		strcpy(text,"Tras");
		ssd1306_WriteString(text, Font_7x10, White);
	}
	/*else {
		ssd1306_SetCursor(0,15);
		sprintf(text,"nao Frente %d",Walle.Sensor_Frente);
		ssd1306_WriteString(text, Font_7x10, White);
	}*/

	if(Walle.Sensor_Esquerda > SensorEsquerdaAnterior && (Walle.Sensor_Esquerda - SensorEsquerdaAnterior)>VariacaoSensor){
		ssd1306_SetCursor(0,30);
		strcpy(text,"Esquerda");
		ssd1306_WriteString(text, Font_7x10, White);
	}
	/*else {
		ssd1306_SetCursor(0,30);
		strcpy(text,"nao Esquerda");
		ssd1306_WriteString(text, Font_7x10, White);
	}*/

	if(Walle.Sensor_Direita > SensorDireitaAnterior && (Walle.Sensor_Direita - SensorDireitaAnterior)>VariacaoSensor){
		ssd1306_SetCursor(0,45);
		strcpy(text,"Direita");
		ssd1306_WriteString(text, Font_7x10, White);
	}
	/*else{
		ssd1306_SetCursor(0,45);
		strcpy(text,"nao Direita");
		ssd1306_WriteString(text, Font_7x10, White);
	}*/


	ssd1306_UpdateScreen();
}


void Print_Espera(){
	char text[20] = {};

	ssd1306_Fill(Black);

	ssd1306_SetCursor(0,0);
	strcpy(text,"Modo de Espera");
	ssd1306_WriteString(text, Font_7x10, White);

	ssd1306_SetCursor(0,15);
	strcpy(text,"Clique no botao");
	ssd1306_WriteString(text, Font_7x10, White);

	ssd1306_SetCursor(0,30);
	strcpy(text,"'Key' --->");
	ssd1306_WriteString(text, Font_7x10, White);

	ssd1306_SetCursor(0,45);
	strcpy(text,"Para comecar");
	ssd1306_WriteString(text, Font_7x10, White);

	ssd1306_UpdateScreen();
}

void Print_Distance(){

	char text[20] = {};

	ssd1306_Fill(Black);

	ssd1306_SetCursor(0,0);
	strcpy(text,"Modo Competicao");
	ssd1306_WriteString(text, Font_6x8, White);

	ssd1306_SetCursor(0, 10);
	sprintf(text,"esq: %d", Walle.Sensor_Esquerda);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_SetCursor(0, 25);
	sprintf(text,"Dir: %d", Walle.Sensor_Direita);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_SetCursor(0, 40);
	sprintf(text,"Frt: %d", Walle.Sensor_Frente);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_UpdateScreen();

}

void Print_Encoders(){
	char text[20] = {};

	ssd1306_Fill(Black);
	k = LeftEncoderCountTotal;
	y = RightEncoderCountTotal;

	if(y == 0){
		k=k/1;
	}
	else{
		k=k/y;
	}
	/*ssd1306_SetCursor(0, 0);
	sprintf(text,"K: %.3f",k);
	ssd1306_WriteString(text, Font_11x18, White);*/

	ssd1306_SetCursor(0, 0);
	sprintf(text,"Esq: %d", LeftEncoderCount);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_SetCursor(0, 15);
	sprintf(text,"Dir: %d", RightEncoderCount);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_SetCursor(0, 30);
	sprintf(text,"Esq T: %ld", LeftEncoderCountTotal);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_SetCursor(0, 45);
	sprintf(text,"Dir T: %ld", RightEncoderCountTotal);
	ssd1306_WriteString(text, Font_11x18, White);

	ssd1306_UpdateScreen();

}



// sensores
void LerSensores(){
	SensorDireitaAnterior = Walle.Sensor_Direita ;
	SensorEsquerdaAnterior = Walle.Sensor_Esquerda;
	SensorFrenteAnterior = Walle.Sensor_Frente;
	/*do{
	Walle.Sensor_Direita = Read_Ultrasonic(SensorDireitoTrigger_GPIO_Port, SensorDireitoTrigger_Pin, SensorDireitoEcho_GPIO_Port, SensorDireitoEcho_Pin);
	Walle.Sensor_Esquerda = Read_Ultrasonic(SensorEsquerdaTrigger_GPIO_Port, SensorEsquerdaTrigger_Pin, SensorEsquerdaEcho_GPIO_Port, SensorEsquerdaEcho_Pin);
	Walle.Sensor_Frente = Read_Ultrasonic(SensorFrenteTrigger_GPIO_Port, SensorFrenteTrigger_Pin, SensorFrenteEcho_GPIO_Port, SensorFrenteEcho_Pin);
	}while(Walle.Sensor_Direita > MaxDistanciaLeituraSensorSonico || Walle.Sensor_Esquerda > MaxDistanciaLeituraSensorSonico || Walle.Sensor_Frente > MaxDistanciaLeituraSensorSonico );
	 */
	for(int i = 0; i<=9;i++){
		MediaSensorD[i] = Read_Ultrasonic(SensorDireitoTrigger_GPIO_Port, SensorDireitoTrigger_Pin, SensorDireitoEcho_GPIO_Port, SensorDireitoEcho_Pin);
		MediaSensorE[i] = Read_Ultrasonic(SensorEsquerdaTrigger_GPIO_Port, SensorEsquerdaTrigger_Pin, SensorEsquerdaEcho_GPIO_Port, SensorEsquerdaEcho_Pin);
		MediaSensorF[i] = Read_Ultrasonic(SensorFrenteTrigger_GPIO_Port, SensorFrenteTrigger_Pin, SensorFrenteEcho_GPIO_Port, SensorFrenteEcho_Pin);
	}

	for(int i = 0; i<=9;i++){
		if(MediaSensorD[i]>MaxDistanciaLeituraSensorSonico && i>0){
			MediaSensorD[i] = MediaSensorD[i-1];
		}
		if(MediaSensorE[i]>MaxDistanciaLeituraSensorSonico && i>0){
			MediaSensorE[i] = MediaSensorE[i-1];
		}
		if(MediaSensorF[i]>MaxDistanciaLeituraSensorSonico && i>0){
			MediaSensorF[i] = MediaSensorF[i-1];
		}
		int SomaD = 0;
		int SomaE = 0;
		int SomaF = 0;
		for(int i = 0; i<=9;i++){
			SomaD+=MediaSensorD[i];
			SomaE+=MediaSensorE[i];
			SomaF+=MediaSensorF[i];
		}

		Walle.Sensor_Direita = SomaD/10;
		Walle.Sensor_Esquerda = SomaE/10;
		Walle.Sensor_Frente = SomaF/10;
	}


	/*SensorDireitaAnterior = Walle.Sensor_Direita;
	SensorEsquerdaAnterior = Walle.Sensor_Esquerda;
	SensorFrenteAnterior = Walle.Sensor_Frente;
	Walle.Sensor_Direita = Read_Ultrasonic(SensorDireitoTrigger_GPIO_Port, SensorDireitoTrigger_Pin, SensorDireitoEcho_GPIO_Port, SensorDireitoEcho_Pin);
	Walle.Sensor_Esquerda = Read_Ultrasonic(SensorEsquerdaTrigger_GPIO_Port, SensorEsquerdaTrigger_Pin, SensorEsquerdaEcho_GPIO_Port, SensorEsquerdaEcho_Pin);
	Walle.Sensor_Frente = Read_Ultrasonic(SensorFrenteTrigger_GPIO_Port, SensorFrenteTrigger_Pin, SensorFrenteEcho_GPIO_Port, SensorFrenteEcho_Pin);
	Walle.Sensor_Chao=HAL_GPIO_ReadPin(SensorChao_GPIO_Port, SensorChao_Pin);

	if(Walle.Sensor_Direita > MaxDistanciaLeituraSensorSonico){
		Walle.Sensor_Direita = SensorDireitaAnterior;
	}
	if(Walle.Sensor_Esquerda > MaxDistanciaLeituraSensorSonico){
			Walle.Sensor_Esquerda = SensorEsquerdaAnterior;
		}
	if(Walle.Sensor_Frente > MaxDistanciaLeituraSensorSonico){
			Walle.Sensor_Frente = SensorFrenteAnterior;
		}*/


}

uint16_t Read_Ultrasonic(GPIO_TypeDef* TriggerPort, uint16_t TriggerPin, GPIO_TypeDef* EchoPort, uint16_t EchoPin){

	uint32_t pMillis;
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;
	uint16_t Distance  = 0;  // cm

	HAL_GPIO_WritePin(TriggerPort, TriggerPin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(__HAL_TIM_GET_COUNTER (&htim3) < 10);  // wait for 10 us
	HAL_GPIO_WritePin(TriggerPort, TriggerPin, GPIO_PIN_RESET);  // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin (EchoPort, EchoPin)) && pMillis + 10 >  HAL_GetTick());
	Value1 = __HAL_TIM_GET_COUNTER (&htim3);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin (EchoPort, EchoPin)) && pMillis + 50 > HAL_GetTick());
	Value2 = __HAL_TIM_GET_COUNTER (&htim3);

	Distance = (Value2-Value1)* 0.034/2;

	return Distance;
}



// controle velocidade
void SetDefaultSpeed(){
	TIM4->CCR1 = MaxPWM_Right;
	TIM4->CCR2 = MaxPWM_Right;
	TIM4->CCR3 = MaxPWM_Left;
	TIM4->CCR4 = MaxPWM_Left;
}

void RightMotorSpeed(uint8_t speed){
	//MaxPWM = speed;
	if (speed<=1)
		speed = 1;

	if (speed>100)
		speed=100;

	TIM4->CCR1 = speed-1;
	TIM4->CCR2 = speed-1;
}

void LeftMotorSpeed(uint8_t speed){
	//MaxPWM = speed;
	if (speed<=1)
		speed=1;

	if (speed>100)
		speed=100;

	TIM4->CCR3 = speed-1;
	TIM4->CCR4 = speed-1;
}


// macros roda direita
void RightMotorForward(){

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);

}

void RightMotorBackward(){
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
}

void RightMotorStop(){

	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);

}


// macros roda direita
void LeftMotorForward(){

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);

}

void LeftMotorBackward(){
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
}

void LeftMotorStop(){

	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);

}



// movimentos
void Mov_Forward(){
	uint16_t i = 0;
	RightEncoderCount=0;
	LeftEncoderCount=0;

	Flag_RightMotor_Mov_Forward=true;
	Flag_LeftMotor_Mov_Forward=true;

	SetDefaultSpeed();

	RightMotorForward();
	LeftMotorForward();

	while(Flag_RightMotor_Mov_Forward || Flag_LeftMotor_Mov_Forward){
		osDelay(10);
		i++;
		if(i>= Timeout)
			break;
	}

}

void Mov_Backward(){
	uint16_t i = 0;
	RightEncoderCount=0;
	LeftEncoderCount=0;

	Flag_RightMotor_Mov_Backward = true;
	Flag_LeftMotor_Mov_Backward = true;

	SetDefaultSpeed();

	RightMotorBackward();
	LeftMotorBackward();

	while(Flag_LeftMotor_Mov_Backward || Flag_RightMotor_Mov_Backward){
		osDelay(10);
		i++;
		if(i>= Timeout)
			break;
	}
}

void Mov_RotateRight(){
	uint16_t i = 0;
	RightEncoderCount=0;
	LeftEncoderCount=0;

	Flag_LeftMotor_Mov_RotateRight=true;
	SetDefaultSpeed();
	LeftMotorForward();

	while(Flag_LeftMotor_Mov_RotateRight){
		osDelay(10);
		i++;
		if(i>= Timeout)
			break;
	}

}

void Mov_RotateLeft(){
	uint16_t i = 0;
	RightEncoderCount=0;
	LeftEncoderCount=0;

	Flag_RightMotor_Mov_RotateLeft=true;
	SetDefaultSpeed();
	RightMotorForward();

	while(Flag_RightMotor_Mov_RotateLeft){
		osDelay(10);
		i++;
		if(i>= Timeout)
			break;
	}
}



// callback encoder
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_4){
		LeftEncoderCount++;
		LeftEncoderCountTotal++;
	}
	if(GPIO_Pin == GPIO_PIN_3){
		RightEncoderCount++;
		RightEncoderCountTotal++;
	}
	if(GPIO_Pin == SensorChao_Pin){
		if(Barata != AguardaBotao)
			Baratinha=false;
	}

}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{


		/*
		osDelay(1000);
		TIM2->ARR = 20;
		TIM2->CCR1 = 10;
		osDelay(1000);
		TIM2->ARR = 50;
		TIM2->CCR1 = 25;*/

		/*
		if(!HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin)){

			HAL_Delay(500);

			Mov_RotateRight();
			while(Flag_LeftMotor_Mov_RotateRight || Flag_RightMotor_Mov_RotateRight){
				Print_Encoders();
				osDelay(100);
			};
			Print_Encoders();


		}

		 */
		osDelay(10000);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRightMotor */
/**
 * @brief Function implementing the RightMotor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRightMotor */
void StartRightMotor(void *argument)
{
	/* USER CODE BEGIN StartRightMotor */
	/* Infinite loop */
	for(;;)
	{

		if(Flag_RightMotor_Mov_Forward){
			if(RightEncoderCount >=Count_Mov_Forward*0 && RightEncoderCount < Count_Mov_Forward*0.1){
				RightMotorSpeed(MaxPWM_Right*0.8);
			}

			else if(RightEncoderCount >=Count_Mov_Forward*0.1 && RightEncoderCount < Count_Mov_Forward*0.2){
				RightMotorSpeed(MaxPWM_Right*1);
			}



			else if(RightEncoderCount >=Count_Mov_Forward*0.8 && RightEncoderCount < Count_Mov_Forward*0.9){
				RightMotorSpeed(MaxPWM_Right*0.85);
			}

			else if(RightEncoderCount >=Count_Mov_Forward*0.9 && RightEncoderCount < Count_Mov_Forward){
				RightMotorSpeed(MaxPWM_Right*0.7);
			}


			else if(RightEncoderCount >=Count_Mov_Forward){
				RightMotorStop();
				Flag_RightMotor_Mov_Forward=false;
			}
		}

		if(Flag_RightMotor_Mov_RotateLeft){
			/*if(RightEncoderCount >=Count_Mov_Forward*0.8 && RightEncoderCount < Count_Mov_Forward){
				RightMotorSpeed(MaxPWM_Right*0.7);
			}*/
			if(RightEncoderCount >= Count_Mov_RotateRight){
				RightMotorStop();
				Flag_RightMotor_Mov_RotateLeft=false;
			}
		}

		if(Flag_RightMotor_Mov_Backward){
			if(RightEncoderCount>=Count_Mov_Backward){
				RightMotorStop();
				Flag_RightMotor_Mov_Backward = false;
			}
		}




		osDelay(1);
	}
	/* USER CODE END StartRightMotor */
}

/* USER CODE BEGIN Header_StartLeftMotor */
/**
 * @brief Function implementing the LeftMotor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLeftMotor */
void StartLeftMotor(void *argument)
{
	/* USER CODE BEGIN StartLeftMotor */
	/* Infinite loop */
	for(;;)
	{
		if(Flag_LeftMotor_Mov_Forward){

			if(LeftEncoderCount >= Count_Mov_Forward*0 && LeftEncoderCount < Count_Mov_Forward*0.1){
				LeftMotorSpeed(MaxPWM_Left*0.8);
			}

			else if(LeftEncoderCount >= Count_Mov_Forward*0.1 && LeftEncoderCount < Count_Mov_Forward*0.2){
				LeftMotorSpeed(MaxPWM_Left*1);
			}


			else if(LeftEncoderCount >= Count_Mov_Forward*0.8 && LeftEncoderCount < Count_Mov_Forward*0.9){
				LeftMotorSpeed(MaxPWM_Left*0.85);
			}

			else if(LeftEncoderCount >= Count_Mov_Forward*0.9 && LeftEncoderCount < Count_Mov_Forward){
				LeftMotorSpeed(MaxPWM_Left*0.7);
			}



			else if(LeftEncoderCount >= Count_Mov_Forward){
				LeftMotorStop();
				Flag_LeftMotor_Mov_Forward=false;
			}
		}

		if(Flag_LeftMotor_Mov_RotateRight)
		{
			/*if(LeftEncoderCount >= Count_Mov_RotateRight*0.8 && LeftEncoderCount < Count_Mov_RotateRight){
				LeftMotorSpeed(MaxPWM_Left*0.7);
			}*/
			if(LeftEncoderCount >= Count_Mov_RotateRight){
				LeftMotorStop();
				Flag_LeftMotor_Mov_RotateRight=false;
			}
		}


		if(Flag_LeftMotor_Mov_Backward){
			if(LeftEncoderCount >= Count_Mov_Backward){
				LeftMotorStop();
				Flag_LeftMotor_Mov_Backward = false;
			}
		}
		osDelay(1);
	}
	/* USER CODE END StartLeftMotor */
}

/* USER CODE BEGIN Header_StartStateMachine */
/**
 * @brief Function implementing the StateMachine thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStateMachine */
void StartStateMachine(void *argument)
{
	/* USER CODE BEGIN StartStateMachine */
	/* Infinite loop */
	Print_Espera();
	for(;;)
	{

		// modo barata tonta
		if(Baratinha){
			switch (Barata) {

			case AguardaBotao:
				if(!HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin)){
					Buzzer5seg();
					Barata = LeSensor;
				}

				break;

			case LeSensor:
				LerSensores();
				if(!HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin)){

					if(Flag_EncoderOrDistance){
						Flag_EncoderOrDistance = false;}
					else{
						Flag_EncoderOrDistance = true;}
				}
				if(Flag_EncoderOrDistance){
					Print_Distance();
				}
				else{
					Print_Direcao();}

				HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				if (Walle.Sensor_Frente < 8 || Walle.Sensor_Direita < 8 || Walle.Sensor_Esquerda < 8){
					Mov_Backward(); //se a contagem de ticks do sistema for par, vira para direita, se não para a esquerda
					if(HAL_GetTick()%2 == 0) Mov_RotateRight();
					else Mov_RotateLeft();
				}
				else if(Walle.Sensor_Frente > DistanciaMinima && Walle.Sensor_Direita > DistanciaMinima && Walle.Sensor_Esquerda > DistanciaMinima){
					Barata = Frente;
				}
				else if(Walle.Sensor_Direita > DistanciaMinima){
					Barata = ViraDireita;
				}
				else if(Walle.Sensor_Esquerda > DistanciaMinima){
					Barata = ViraEsquerda;
				}
				else{

					Mov_Backward(); //se a contagem de ticks do sistema for par, vira para direita, se não para a esquerda
					if(HAL_GetTick()%2 == 0) Mov_RotateRight();
					else Mov_RotateLeft();
				}

				break;

			case Frente:
				Mov_Forward();
				Barata = LeSensor;

				break;

			case ViraDireita:
				Mov_RotateRight();
				Barata = LeSensor;


				break;

			case ViraEsquerda:
				Mov_RotateLeft();
				Barata = LeSensor;

				break;

			default:
				break;
			}
		}

		// Achou
		else{

			if(!achou){
				Print_Ajustando();
				if(HAL_GPIO_ReadPin(SensorChaoTras_GPIO_Port, SensorChaoTras_Pin) && HAL_GPIO_ReadPin(SensorChao_GPIO_Port, SensorChao_Pin) ){
					achou = true;
				}
				else if(HAL_GPIO_ReadPin(SensorChaoTras_GPIO_Port, SensorChaoTras_Pin) && !HAL_GPIO_ReadPin(SensorChao_GPIO_Port, SensorChao_Pin)){
					Mov_Backward();
				}
				else if(!HAL_GPIO_ReadPin(SensorChaoTras_GPIO_Port, SensorChaoTras_Pin) && HAL_GPIO_ReadPin(SensorChao_GPIO_Port, SensorChao_Pin)){
					Mov_RotateRight();
				}
				else if(!HAL_GPIO_ReadPin(SensorChaoTras_GPIO_Port, SensorChaoTras_Pin) && !HAL_GPIO_ReadPin(SensorChao_GPIO_Port, SensorChao_Pin)){
					Mov_RotateRight();
					osDelay(100);
					ContadorRotateRigt--;
					if(ContadorRotateRigt<=0){
						ContadorRotateRigt=5;
						Baratinha=true;
					}
				}



			}
			else{
				Print_Achou();
				BuzzerTocaMusica();
				BuzzerTocaMusica();
				osDelay(1000);
				osDelay(1000);
			}


		}

		osDelay(1);

	}


	/* USER CODE END StartStateMachine */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
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
	while(1){ };
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
