/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_data;
volatile uint8_t saved_command = 0; // 추가: 유효한 명령을 기억해 둘 변수
volatile uint8_t current_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void execute_robot_action(uint8_t signal);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
    // 라즈베리파이 UART 수신 대기
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    // 이전 적외선 신호 저장
    GPIO_PinState last_ir1_state = GPIO_PIN_SET;
    // 시퀀스 상태 변수
    uint8_t current_state = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
       // 1. 센서 상태 읽기
      GPIO_PinState ir1_state = HAL_GPIO_ReadPin(IR_1_GPIO_Port, IR_1_Pin);
      GPIO_PinState ir2_state = HAL_GPIO_ReadPin(IR_2_GPIO_Port, IR_2_Pin);

      // 2. [보고] IR_1(입구) 상태 실시간 전송 (파이썬이 4초 판정하기 위함)
      // 변화 감지는 유지하되, 주기적으로도 현재 상태 전송 추가
      if (ir1_state != last_ir1_state) {
          if (ir1_state == GPIO_PIN_RESET) HAL_UART_Transmit(&huart2, (uint8_t*)"H\n", 2, 10);
          else                             HAL_UART_Transmit(&huart2, (uint8_t*)"L\n", 2, 10);
          last_ir1_state = ir1_state;
      }

      // 200ms마다 현재 상태 주기적 전송 추가
      static uint32_t last_report_tick = 0;
      if (HAL_GetTick() - last_report_tick >= 200) {
          if (ir1_state == GPIO_PIN_RESET) HAL_UART_Transmit(&huart2, (uint8_t*)"H\n", 2, 10);
          else                             HAL_UART_Transmit(&huart2, (uint8_t*)"L\n", 2, 10);
          last_report_tick = HAL_GetTick();
      }

      if (saved_command == 'S') {
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // 벨트 ON
         current_state = 2;   // "나 이제 이송 중이야!"라고 상태 변경
         saved_command = 0;   // 명령 소비 완료
      }

      // 4. [시퀀스: 이송 중 도착 감지]
      if (current_state == 2 && ir2_state == GPIO_PIN_RESET) {
         // 도착하면 4초간 확실히 머무는지 확인
         uint8_t arrived = 1;
         for (int i = 0; i < 40; i++) {
            HAL_Delay(100);
            if (HAL_GPIO_ReadPin(IR_2_GPIO_Port, IR_2_Pin) == GPIO_PIN_SET) {
               arrived = 0; break;
            }
         }

         if (arrived) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // 1. 벨트 정지
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);   // 2. PC1(완료신호) SET

            // 파이썬에 End 신호
            HAL_UART_Transmit(&huart2, (uint8_t*)"E\n", 2, 10);

            current_state = 3;
         }
      }

      // 5. [시퀀스: 물건이 치워지면 초기화]
      if (current_state == 3) {
         if (HAL_GPIO_ReadPin(IR_2_GPIO_Port, IR_2_Pin) == GPIO_PIN_SET) {
            HAL_Delay(30); // 노이즈 방지
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // PC1 OFF
            current_state = 0; // 다시 처음 대기 상태로
         }
      }

      HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|RELAY_4_Pin|RELAY_5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin|RELAY_2_Pin|RELAY_3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IR_1_Pin|IR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|RELAY_4_Pin|RELAY_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RELAY_1_Pin|RELAY_2_Pin|RELAY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance == USART2)
       {
           switch(rx_data)
           {
               case 'a':
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                   break;
               case 'A':
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                   break;
               case 'b':
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
                   break;
               case 'B':
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                   break;
               case 'c':
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
                   break;
               case 'C':
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
                   break;

               case 'S': // Start
                  saved_command = 'S';
                  break;
               case 'T': // 비상 정지
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                  current_state = 0;
                  break;
               case 'Q': //즉시 현재 적외선 센서 신호 보냄
                  if (HAL_GPIO_ReadPin(IR_1_GPIO_Port, IR_1_Pin) == GPIO_PIN_RESET)
                      HAL_UART_Transmit(&huart2, (uint8_t*)"H\n", 2, 10);
                  else
                      HAL_UART_Transmit(&huart2, (uint8_t*)"L\n", 2, 10);
                  break;
           }
           HAL_UART_Receive_IT(&huart2, &rx_data, 1);
       }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
