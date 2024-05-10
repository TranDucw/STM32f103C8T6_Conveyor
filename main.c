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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int8_t dem = 0;

uint8_t autoo;

uint8_t state = 0;

char rec[10] = "";
uint16_t num = 0;
uint8_t vat = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// config TIMER
void tim_init(){
	RCC -> APB1ENR |= (1<<0) ;
	TIM2 -> PSC = 99;
	TIM2 -> ARR = 159;
	TIM2 -> DIER |= (1<<0) ; 
	NVIC -> ISER[0] |= (1<<28) ;
}

// ham xu ly ngat timer
void TIM2_IRQHandler (void)
{
	static int dem_time = 0; 
	TIM2->SR &= ~(1<< 0); 
	dem_time ++;
	if (dem_time>=num){ //dieu kien dung bang tai
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1); // dung bang tai
		if (vat == 0){ 
			TIM2 -> CR1 &= ~(1<<0) ; //reset timer
			TIM2 ->CNT = 0;
			dem_time = 0;
		}
		
		else{ // neu bang tai co vat thi dung 4s sau do hoat dong tiep
			static uint16_t c = 0;
			c++;
			if (c>=4000){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
				c=0;
				TIM2 -> CR1 &= ~(1<<0) ; // reset timer
				TIM2 ->CNT = 0;
				dem_time = 0;
			}
		}
	}
}

// ham truyen du lieu tu vxl len may tinh
void truyen(char data){
		while ((USART2->SR &(1<<6))==0); // doi vxl san sang de truyen du lieu 
		USART2->DR = data; // truyen du lieu
}

// ham ngat uart
void USART2_IRQHandler (){
	if (dem<5){
		if (USART2->DR > 47 && USART2->DR < 59){ //neu ky tu may tinh gui xuong la so (theo bang ma ascii) thi chuyen tu ky tu thanh so
			while (USART2->SR & 1<<5);
			rec[dem] = USART2->DR;
			truyen(rec[dem]);
			dem++;
			if (dem == 4){ // vi truyen mot chuoi ky tu XXXX nen den so thu 4 thi chuyen thanh so 
				num = atoi(rec);
				dem = 5;
			}
		}
		else{
			while (USART2->SR & 1<<5);
			rec[dem] = USART2->DR;
			truyen(rec[dem]);
			if(USART2->DR == 'p') // neu gap ky tu 'p' thi dung truyen
				dem = 5;
			dem++;
		}
	}
	else{
		for (int i = 0; i< dem; i++){
				rec[i] = 0; // reset cac gia tri ve 0 sau khi nhan cac gia tri xong
		}
		dem = 0;
	}
}

void switchh(){
	// doc gia tri switch
	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==0){
		autoo = 0;
	}
	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==1){
		autoo = 1;
	}
}

void stop (){
	// doc gia tri nut stop
	if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == 0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
	}
}

//config uart
void UART_Init(){
	USART2 -> CR1 |= (1<<2) | (1<<3);
	USART2 -> CR1 |= (1<<13);
	USART2 -> CR1 |= (1<<5); 
	NVIC -> ISER[1] |= (1<<6);
} 

void bangtai(){
	// nut nhan hoat dong bang tai
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11) == 0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
	}
}
void cambien_man(){
	// doc so luong san pham
  if((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)==1) & (state == 0)){ // tin hieu cam bien dau bang tai
		state = 1;
	}
	if ((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)==0) & (state == 1)){ // tin hieu cam bien cuoi bang tai
		dem++;
		state = 0;
		int buffer[] = {};
		sprintf(buffer, "%d", dem); // chuyen so luong san pham tu kieu du lieu uint8_t sang string
		truyen(buffer); // gui sso luong san pham len terminal
	}
}

void check(){
	// ham kiem tra xem tren bang tai co vat hay khong
	if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)==1)){
		vat ++;
	}
	if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12)==1)){
		vat --;
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
	UART_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	USART2 -> CR1 |= (1<<5) ;
	NVIC -> ISER[1] |= (1<<6) ;
	tim_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	unsigned char T = 10;
	unsigned char P = 10;
  while (1)
  {
		switchh();
		if (autoo == 0){ // che do manual
			bangtai();
			stop();
			cambien_man();
		}
		if (autoo == 1){ // che do auto
			check();
			T = strcmp("start", rec); // kiem tra du lieu gui tu may tinh ve uart
			P = strcmp("stop", rec);	// kiem tra du lieu gui tu may tinh ve uart
			if (T == 0)
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);
			if (P == 0)
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
