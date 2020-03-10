/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "tim.h"
#include "adc.h"
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
/* USER CODE BEGIN Variables */

uint8_t drDir;
uint8_t tekst[30];
uint16_t dl_tekst;
double napiecie[2];
double natezenie[2];
double oporWew[2] = {11 , 11}; //zmierzyć opor wew silnikow
double predkoscPomiar[2];
double beta[2] = {0.05, 0.05};
int predkosc = 200; //predkosc poczatkowa 0-100
double dpulse[2] = {200, 200};
double integral[2] = {0, 0};
int pulse[2] = {200, 200};
int licznik;
int blokada;
double Kp = 0.7;
double Ki = 0.01;

extern uint8_t znak;
extern uint32_t adcVal[6];
extern double adcAvgVal[6];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId btReadHandle;
osThreadId motorControlHandle;
osThreadId adcAvgValHandle;
osSemaphoreId bSem1Handle;
osSemaphoreId bSem2Handle;
osSemaphoreId bSem3Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void pomiarBeta(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartBtRead(void const * argument);
void StartMotorControl(void const * argument);
void StartAdcAvgVal(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	  HAL_TIM_Base_Start(&htim2);
	  HAL_ADC_Start_DMA(&hadc1, adcVal, 6);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_UART_Receive_IT(&huart1, &znak, 1);


  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of bSem1 */
  osSemaphoreDef(bSem1);
  bSem1Handle = osSemaphoreCreate(osSemaphore(bSem1), 1);

  /* definition and creation of bSem2 */
  osSemaphoreDef(bSem2);
  bSem2Handle = osSemaphoreCreate(osSemaphore(bSem2), 1);

  /* definition and creation of bSem3 */
  osSemaphoreDef(bSem3);
  bSem3Handle = osSemaphoreCreate(osSemaphore(bSem3), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xSemaphoreTake(bSem1Handle, osWaitForever);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of btRead */
  osThreadDef(btRead, StartBtRead, osPriorityRealtime, 0, 128);
  btReadHandle = osThreadCreate(osThread(btRead), NULL);

  /* definition and creation of motorControl */
  osThreadDef(motorControl, StartMotorControl, osPriorityBelowNormal, 0, 128);
  motorControlHandle = osThreadCreate(osThread(motorControl), NULL);

  /* definition and creation of adcAvgVal */
  osThreadDef(adcAvgVal, StartAdcAvgVal, osPriorityAboveNormal, 0, 128);
  adcAvgValHandle = osThreadCreate(osThread(adcAvgVal), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBtRead */
/**
* @brief Function implementing the btRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtRead */
void StartBtRead(void const * argument)
{
  /* USER CODE BEGIN StartBtRead */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(bSem1Handle, osWaitForever);
	  switch(znak){
	  case 'e':
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  dl_tekst = sprintf(tekst, "Dioda on\n");
		  break;
	  case 'd':
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  dl_tekst = sprintf(tekst, "Dioda off\n");
		  break;
	  case 'f':
		  doPrzodu();
		  drDir = 'f';
		  dl_tekst = sprintf(tekst, "Do przodu!\n");
		  //dpulse[0] = predkosc;
		  //dpulse[1] = predkosc;
		  break;
	  case 'b':
		  doTylu();
		  drDir = 'b';
		  dl_tekst = sprintf(tekst, "Do tylu!\n");
		  //dpulse[0] = predkosc;
		  //dpulse[1] = predkosc;
		  break;
	  case 'l':
		  wLewo();
		  drDir = 'l';
		  dl_tekst = sprintf(tekst, "W lewo!\n");
		  //dpulse[0] = predkosc;
		  //dpulse[1] = predkosc;
		  break;
	  case 'r':
		  wPrawo();
		  drDir = 'r';
		  dl_tekst = sprintf(tekst, "W prawo!\n");
		  //dpulse[0] = predkosc;
		  //dpulse[1] = predkosc;
		  break;
	  case 's':
		  stoj();
		  drDir = 's';
		  dl_tekst = sprintf(tekst, "STOP!\n");
		  break;
	  case 'a':
		  if(predkosc >= 1000){
			  predkosc = 1000;
			  dl_tekst = sprintf(tekst, "Predkosc maksymalna!\n");
			  break;
		  }
		  predkosc += 50;
		  //dpulse[0] = predkosc;
		  //dpulse[1] = predkosc;
		  dl_tekst = sprintf(tekst, "Predkosc zwiekszona do %d %!\n", predkosc);
		  break;
	  case 'p':
		  if(predkosc <= 0){
			  predkosc = 0;
			  dl_tekst = sprintf(tekst, "Predkosc minimalna!\n");
			  break;
		  }
		  predkosc -= 50;
		  //dpulse[0] = predkosc;
		  //dpulse[1] = predkosc;
		  dl_tekst = sprintf(tekst, "Predkosc zmniejszona do %d %!\n", predkosc);
		  break;
	  case 'x':
		  pomiarBeta();
		  break;
	  default:
		  dl_tekst = sprintf(tekst, "Bledny kod\n");
	  }
	  HAL_UART_Transmit_IT(&huart1, tekst, dl_tekst);
  }
  /* USER CODE END StartBtRead */
}

/* USER CODE BEGIN Header_StartMotorControl */
/**
* @brief Function implementing the motorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControl */
void StartMotorControl(void const * argument)
{
  /* USER CODE BEGIN StartMotorControl */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(bSem3Handle, osWaitForever);


	  napiecie[0] = fabs(adcAvgVal[0]-adcAvgVal[1])*0.004593; //0.00014138 = 3.3 / 4095 / 1000 * 5700
	  napiecie[1] = fabs(adcAvgVal[2]-adcAvgVal[3])*0.004593; //dziala z kondensatorami

	  //napiecie[0] = pulse[0]/1000.*9.; //ze wzgledu na slaby odczyt pwm
	  //napiecie[1] = pulse[1]/1000.*9.; //ze wzgledu na slaby odczytw pwm

	  natezenie[0] = adcAvgVal[4]*0.0008059;
	  natezenie[1] = adcAvgVal[5]*0.0008059;
	  predkoscPomiar[0]=0.95*(napiecie[0]-natezenie[0]*oporWew[0])/beta[0];
	  predkoscPomiar[1]=(napiecie[1]-natezenie[1]*oporWew[1])/beta[1];
	  if(predkoscPomiar[0] < 0) predkoscPomiar[0] = 0;
	  if(predkoscPomiar[1] < 0) predkoscPomiar[1] = 0;
		  integral[0]+=((predkosc-predkoscPomiar[0]) * Ki);
		  integral[1]+=((predkosc-predkoscPomiar[1]) * Ki);
		  if(drDir == 's'){
			  integral[0] = 0;
			  integral[1] = 0;
		  }
		  dpulse[0] = integral[0] + Kp * (predkosc-predkoscPomiar[0]);
		  dpulse[1] = integral[1] + Kp * (predkosc-predkoscPomiar[1]);
		  //pulse[0] = (int) dpulse[0];
		  //pulse[1] = (int) dpulse[1];
		  if(dpulse[0]>1000){
			  dpulse[0]=1000;
			  integral[0]-=((predkosc-predkoscPomiar[0]) * Ki);
		  }
		  if(dpulse[1]>1000){
			  dpulse[1]=1000;
			  integral[1]-=((predkosc-predkoscPomiar[1]) * Ki);
		  }
		  if(dpulse[0]<0) dpulse[0]=0;
		  if(dpulse[1]<0) dpulse[1]=0;
		  htim1.Instance->CCR1 = dpulse[0];
		  htim1.Instance->CCR2 = dpulse[1];
		  osDelay(10); //bylo git przy 10


	  if(licznik == 500){
		  int PR = (int)(1000*napiecie[0] * natezenie[0]);
		  int PL = (int)(1000*napiecie[1] * natezenie[1]);
		  dl_tekst = sprintf(tekst, "Pow R = %dmW Pow L = %dmW\n", PR, PL);
		  HAL_UART_Transmit_IT(&huart1, tekst, dl_tekst);
		  licznik = 0;
	  }
	  licznik+=1;
	  if((drDir == 'f' || drDir == 'b') && (predkoscPomiar[0] < 0.15 * predkosc || predkoscPomiar[1] < 0.15 * predkosc)){
		  blokada += 1;
	  }
	  else{
		  blokada = 0;
	  }
	  if(blokada == 100){
		  dl_tekst = sprintf(tekst, "Wykryto przeszkode!\n");
		  HAL_UART_Transmit_IT(&huart1, tekst, dl_tekst);
		  dpulse[0] = 300;
		  dpulse[1] = 300;
		  htim1.Instance->CCR1 = dpulse[0];
		  htim1.Instance->CCR2 = dpulse[1];
		  if(drDir == 'f') doTylu();
		  switch(drDir){
		  case 'f':
			  doTylu();
			  break;
		  case 'b':
			  doPrzodu();
			  break;
		  case 'l':
			  wPrawo();
			  break;
		  case 'r':
			  wLewo();
			  break;
		  }
		  HAL_Delay(700);
		  stoj();
		  wLewo();
		  HAL_Delay(800);
		  switch(drDir){
		  case 'f':
			  doPrzodu();
			  break;
		  case 'b':
			  doTylu();
			  break;
		  }
		  blokada = 0;
		  integral[0] = 0;
		  integral[1] = 0;
		  HAL_Delay(200);

	  }

	  xSemaphoreGive(bSem3Handle);
  }
  /* USER CODE END StartMotorControl */
}

/* USER CODE BEGIN Header_StartAdcAvgVal */
/**
* @brief Function implementing the adcAvgVal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcAvgVal */
void StartAdcAvgVal(void const * argument)
{
  /* USER CODE BEGIN StartAdcAvgVal */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(bSem2Handle, osWaitForever);
	  for(int i = 0; i < 4; i++){
		  adcAvgVal[i]=adcAvgVal[i]*0.9+adcVal[i]*0.1; //uśrednianie

	  }
	  for(int i = 4; i < 6; i++){
		  adcAvgVal[i]=adcAvgVal[i]*0.99+adcVal[i]*0.01; //uśrednianie

	  }
    //osDelay(1); //czy potrzeba? ma dzialac szybko
  }
  /* USER CODE END StartAdcAvgVal */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void pomiarBeta(){
	osSemaphoreWait(bSem3Handle, osWaitForever);
	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_RESET);

	 htim1.Instance->CCR1 = 700;
	 htim1.Instance->CCR2 = 700;


	 osDelay(3000);

	 beta[0]=((fabs(adcAvgVal[0]-adcAvgVal[1])*0.004593)-adcAvgVal[4]*0.0008059*oporWew[0])/1000; //dziala z kondensatorami
	 beta[1]=((fabs(adcAvgVal[2]-adcAvgVal[3])*0.004593)-adcAvgVal[5]*0.0008059*oporWew[1])/1000; //dziala z kondensatorami

	 //beta[0]=(9-adcAvgVal[4]*0.0008059*oporWew[0])/1000; //ze wzgledu na slaby udczyt pwm
	 //beta[1]=(9-adcAvgVal[5]*0.0008059*oporWew[1])/1000; //ze wzgledu na slaby odzcyt pwm

	 htim1.Instance->CCR1 = predkosc;
	 htim1.Instance->CCR2 = predkosc;

	 HAL_GPIO_WritePin(RF_GPIO_Port, RF_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(LF_GPIO_Port, LF_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(RB_GPIO_Port, RB_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(LB_GPIO_Port, LB_Pin, GPIO_PIN_RESET);
	 drDir = 's';
	 int R = (int)10000*beta[0];
	 int L = (int)10000*beta[1];
	 dl_tekst = sprintf(tekst, "B R = %d B L = %d\n", R, L);
	 HAL_UART_Transmit_IT(&huart1, tekst, dl_tekst);

	 xSemaphoreGive(bSem3Handle);

}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
