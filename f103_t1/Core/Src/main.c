/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "global_define.h"
#include "system_def.h"
#include "global_define.h"
#include "SX1278_hw.h"
#include "SX1278.h"
#include <stdio.h>
#include "lora_task.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t aRxBuffer;
uint8_t UR3RxBuf_cnt = 0;		//
uint8_t UR3RxBuf_rxleng = 4;
uint8_t UR3RxBuf_rxleng_lat=0;
unsigned char UR3TxBuf[UR1_BUFSZ];
unsigned char UR3RxBuf[UR1_BUFSZ];

unsigned char UR3TxBuf2[UR1_BUFSZ];

unsigned char SPI1TxBuf[SPI_BUFSZ];
unsigned char SPI1RxBuf[SPI_BUFSZ];

unsigned char TEMPBuf1[32];
unsigned char TEMPBuf2[32];

int SPI1RxBuf_leng=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int master;
int ret;

char buffer[512];

int message;
int message_length;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int i;
  int reti;
  int erri;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  master=1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  parameter_init();
  //uart1
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
  //send uart1
  
  UR3TxBuf[0] = 0x4c;UR3TxBuf[1] = 0x52;UR3TxBuf[2] = 0x6d;UR3TxBuf[3] = 0x31;UR3TxBuf[4] = 0x30;UR3TxBuf[5] = 0x33;UR3TxBuf[6] = 0x32;
  if(master ==0)
  {UR3TxBuf[2] = 0x73;}
  SendUART3(7);HAL_Delay (100);

  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_SPI_ENABLE(&hspi1);
  
	//initialize LoRa module
	SX1278_hw.dio0.port = DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = DIO0_Pin;
	SX1278_hw.nss.port = NSS_GPIO_Port;
	SX1278_hw.nss.pin = NSS_Pin;
	SX1278_hw.reset.port = RESET_GPIO_Port;
	SX1278_hw.reset.pin = RESET_Pin;
	SX1278_hw.spi = &hspi1;

	SX1278.hw = &SX1278_hw;

	SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
  
	if (master == 1) {
		ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	} else {
		ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
  //parameter 
  message=0;
  master_status=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if((gSYS_ACT_flag & 0x1) == 0x1)
    {
        gSYS_ACT_flag &=0xfffffffe;
        //  HAL_GPIO_WritePin(GPIOB,LED_BLUE_Pin,GPIO_PIN_SET);
        //HAL_GPIO_TogglePin(GPIOB,LED_BLUE_Pin);
      if (master == 1) {
        if(gSYS_100ms_cnt == 0)
        {
          
          SX1278_LoRaEntryTx(&SX1278, 16, 2000);
          //sending beacon
          //message_length = sprintf(buffer, "Hello %d", message);
          message_length = compose_beacon(&UR3TxBuf[0],gSYS_100ms_cnt,message);
          reti = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
          master_status = 1;
          //ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer,message_length, 2000);
          reti = SX1278_LoRaTxPacket(&SX1278, &UR3TxBuf[0],message_length, 2000);
          //
          //message++;
          //UR3TxBuf[0] = message;SendUART3(1);HAL_Delay (10);
        }
        else if(gSYS_100ms_cnt == 2)
        {
          //master sending 
          #if 1
          message_length = send_info_r(&UR3TxBuf[0],gSYS_100ms_cnt,0x81,message);
          //no need to re-enter tx??
          reti = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
          //advance 
          message++;gSYS_MST_txcnt++;
          //UR3TxBuf[0] = message;SendUART3(1);HAL_Delay (5);
          master_status = 1;
          reti = SX1278_LoRaTxPacket(&SX1278, &UR3TxBuf[0],message_length, 2000);                  
          #endif
        }
        else if(gSYS_100ms_cnt == 4)
        {
          //master enter rx 
          ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
          master_status=3;
          HAL_GPIO_TogglePin(GPIOB,LED_BLUE_Pin);
          //UR3TxBuf[0] = message;SendUART3(1);HAL_Delay (100);
        
        }

       
      }  //end of master == 1
      else{ //slave
        if(gSYS_100ms_cnt == 0)
        {
          HAL_GPIO_TogglePin(GPIOB,LED_BLUE_Pin);
        }
        else if(gSYS_100ms_cnt == slave_rsp_flag)
		    {
		      slave_rsp_flag = 100;//disable
          //slave tx response
          slave_status = 1;
          reti = SX1278_LoRaEntryTx(&SX1278, slave_rsp_leng, 2000);
          reti = SX1278_LoRaTxPacket(&SX1278, &SPI1TxBuf[0],slave_rsp_leng, 2000);
		    }
		    else if(gSYS_100ms_cnt == slave_rx_flag)
        {
          //back to rx
          ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
          slave_status=3;
        }
      }
#ifndef __EN_DIO0_INT
      else {

			ret = SX1278_LoRaRxPacket(&SX1278);
			//printf("Received: %d\r\n", ret);
			if (ret > 0) {
        
				SX1278_read(&SX1278, (uint8_t*) buffer, ret);
				//printf("Content (%d): %s\r\n", ret, buffer);
        for(i=0;i<ret;i++)
        {
          UR3TxBuf[i] = *(buffer+i);
          
        }
        SendUART3(ret);HAL_Delay (100);
			}
      }
#endif
    }
    else if((gSYS_ACT_flag & 0x2) == 0x2)
    {
      gSYS_ACT_flag &=0xfffffffd;
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
      SPI1TxBuf[0] = 0x42;
      //HAL_SPI_TransmitReceive_IT(&hspi1,&SPI1TxBuf[0],&SPI1RxBuf[0],2);
    }
    else if((gSYS_ACT_flag & 0x4) == 0x4)
    {
      gSYS_ACT_flag &=0xfffffffb;
	    if(master == 1)
	    {
		    master_status = 5;//master rx_done
        gRXPARAM_rssi = SX1278_RSSI_LoRa(&SX1278);
        gRXPARAM_snr = SX1278_ESTSNR_LoRa(&SX1278);
        gSYS_MST_rxcnt++;
        if(gSYS_MST_errcnt < 1000)
        {gSYS_MST_errcnt = gSYS_MST_txcnt - gSYS_MST_rxcnt;        }
        //show master rx
        #ifdef _EN_MST_SHOW_RAWBTE
          UR3TxBuf[0] = 0x10;
          UR3TxBuf[1] = 0x08;
          
          UR3TxBuf[2] = SPI1RxBuf[7];//seq no
          UR3TxBuf[3] = gRXPARAM_rssi;
          UR3TxBuf[4] = gRXPARAM_snr;
          UR3TxBuf[5] = 0;
          UR3TxBuf[6] = 0;
          UR3TxBuf[7] = 0x0a;
          SendUART3(8);HAL_Delay (5);
        #else
          TEMPBuf1[0] = gRXPARAM_rssi;//rssi
          TEMPBuf1[1] = gRXPARAM_snr;//snr
          //TEMPBuf1[2] = (unsigned char)((gSYS_MST_txcnt>>16)&0xff);
          TEMPBuf1[2] = (unsigned char)((gSYS_MST_txcnt>>8)&0xff);
          TEMPBuf1[3] = (unsigned char)((gSYS_MST_txcnt>>0)&0xff);
          //TEMPBuf1[5] = (unsigned char)((gSYS_MST_errcnt>>16)&0xff);
          TEMPBuf1[4] = (unsigned char)((gSYS_MST_errcnt>>8)&0xff);
          TEMPBuf1[5] = (unsigned char)((gSYS_MST_errcnt>>0)&0xff);
          reti = compose_rsptxt(&TEMPBuf1[0],&UR3TxBuf[0],6,1);
          SendUART3(reti);HAL_Delay (5);
        #endif
        
         master_status = 3;//master rx_idle
	    }
      else //slave
	    {
          //
          erri = decode_slvrxdata(&SPI1RxBuf[0],&reti,&SPI1TxBuf[0]);
          slave_status = 5;
		      //compose response data,register time slot
          UR3TxBuf[0] = 0x55;
          UR3TxBuf[1] = reti&0xff;
          if(reti >0)
          {
            //reti > 0,slave will tx and back to rx
            slave_rsp_flag = 5;
		        slave_rx_flag = 7;
            slave_rsp_leng = reti;
            slave_status = 0;
            UR3TxBuf[1] = reti&0xff;
          }
          else
          {
            slave_status = 3;
            UR3TxBuf[1] = 0x00;
          }
          
          //show slave rx
          //UR3TxBuf[0] = 0x55;
          //UR3TxBuf[1] = 0xaa;
		      if(reti >0)
		      {
            #if _EN_SLV_SHOW_RAWBTE
            for(i=0;i<SPI1RxBuf_leng;i++)
            {
              UR3TxBuf[i+2] = SPI1RxBuf[i];//*(buffer+i);
            }
            SendUART3(SPI1RxBuf_leng+2);HAL_Delay (5);
            #else
            TEMPBuf1[0] = SPI1TxBuf[4];//rssi
            TEMPBuf1[1] = SPI1TxBuf[5];//snr
            TEMPBuf1[2] = 0;
            TEMPBuf1[3] = SPI1TxBuf[7];
            TEMPBuf1[4] = gRXPARAM_errcnt;
            reti = compose_rsptxt(&TEMPBuf1[0],&UR3TxBuf[0],5,0);
            SendUART3(reti);HAL_Delay (5);
            #endif
          }
          
          
          
	      }
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SW_NSS_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_RESET_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LORA_DIO4_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = LORA_DIO4_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_NSS_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = SW_NSS_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LOEA_DIO1_Pin */
  GPIO_InitStruct.Pin = LOEA_DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LOEA_DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RESET_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
  if(UR3RxBuf_cnt ==0)
	{
		//check command
		//if((aRxBuffer == CMD_ECHO)|(aRxBuffer == CMD_REGW)|(aRxBuffer == CMD_REGR)|(aRxBuffer == CMD_STAT)|(aRxBuffer == CMD_CTRL))
		{
		UR3RxBuf[UR3RxBuf_cnt] = aRxBuffer;
		UR3RxBuf_cnt++;
		}
	}
	else if (UR3RxBuf_cnt ==1)
	{
		UR3RxBuf_rxleng = aRxBuffer;
		UR3RxBuf[UR3RxBuf_cnt] = UR3RxBuf_rxleng;UR3RxBuf_cnt++;
		if(UR3RxBuf_rxleng > 32)//too large
		{
			UR3RxBuf_cnt=0;UR3RxBuf_rxleng=4;
		}
	}
	else if(UR3RxBuf_cnt == (UR3RxBuf_rxleng-1))
	{
		UR3RxBuf[UR3RxBuf_cnt] = aRxBuffer;
		UR3RxBuf_rxleng_lat = UR3RxBuf_rxleng;
		UR3RxBuf_cnt=0;
		gSYS_ACT_flag|=0x2;
	}
	else
	{
		UR3RxBuf[UR3RxBuf_cnt] = aRxBuffer;
		UR3RxBuf_cnt++;
	}
	

  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
}
void parameter_init(void)
{
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,LED_BLUE_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,LORA_RESET_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,LORA_RESET_Pin,GPIO_PIN_SET);
}

void SendUART3(uint8_t cnt)
{
  HAL_UART_Transmit_IT(&huart3, (uint8_t *)&UR3TxBuf, cnt);
}

void MultiSendUART3(uint16_t param)
{
	uint8_t cnt;
	uint8_t sel;
	
	cnt = (param&0xff);
	sel = (param>>8)&0x7;
	
	if(sel == 0)
    {HAL_UART_Transmit_IT(&huart3, (uint8_t *)&UR3TxBuf, cnt);}
    else if(sel == 1)
    {HAL_UART_Transmit_IT(&huart3, (uint8_t *)&UR3TxBuf2, cnt);}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //BSP_LED_Toggle(LED2);
  
  if(htim->Instance ==TIM1)
  {
    if(gSYS_100ms_cnt == 9)
    {
      gSYS_100ms_cnt = 0;
      
      //SYS_10ms_fcnt=0;
      gSYS_ACT_flag |= 0x1;
    }
    else
    {
      gSYS_100ms_cnt ++;
      gSYS_ACT_flag |= 0x1;
      //if((gSYS_100ms_cnt == 9)||(gSYS_100ms_cnt == 19)||(gSYS_100ms_cnt == 29)||
      //   (gSYS_100ms_cnt == 39)||(gSYS_100ms_cnt == 49)||(gSYS_100ms_cnt == 59)||
      //   (gSYS_100ms_cnt == 69)||(gSYS_100ms_cnt == 79)||(gSYS_100ms_cnt == 89))
      //{
      //  SYS_10ms_fcnt=0;
      //  gSYS_ACT_flag |= 0x1;
      //}
      //else
      //{SYS_10ms_fcnt++;}
      
    }
  }
  else if(htim->Instance ==TIM3)
  {
    if(gSYS_SEC_cnt == 59)
    {
      gSYS_SEC_cnt = 0;
      
    }
    else
    {
      if((gSYS_SEC_cnt == 15)||(gSYS_SEC_cnt == 45))
      {gSYS_ACT_flag |= 0x2;}
      gSYS_SEC_cnt++;
      
    }
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  gSYS_ACT_flag|=0x4;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t ret;
  
  if(GPIO_Pin == GPIO_PIN_0)
  {
    if(master == 1) //master mode
    {
      if(master_status == 1)
      {
        //tx is busy
        SX1278_SPIRead(&SX1278, LR_RegIrqFlags);
			  SX1278_clearLoRaIrq(&SX1278); //Clear irq
			  SX1278_standby(&SX1278); //Entry Standby mode
        
        master_status = 2;//tx done
      }
      else if(master_status == 3)
      {
        ret = SX1278_LoRaRxPacket(&SX1278);
        if (ret > 0) {
        
				//SX1278_read(&SX1278, (uint8_t*) buffer, ret);
          SX1278_read(&SX1278, &SPI1RxBuf[0], ret);
          SPI1RxBuf_leng=ret;
        }
        //SendUART3(ret);HAL_Delay (100);
        master_status = 4;
        gSYS_ACT_flag |= 0x4;
			}
      
    }
    else //slave mode
    {
       if(slave_status == 3) //rx idle and receive lora
       {
        ret = SX1278_LoRaRxPacket(&SX1278);
        if (ret > 0) {
        
				//SX1278_read(&SX1278, (uint8_t*) buffer, ret);
          SX1278_read(&SX1278, &SPI1RxBuf[0], ret);
          SPI1RxBuf_leng=ret;
        }
        //get CR,rssi,snr
        gRXPARAM_rssi = SX1278_RSSI_LoRa(&SX1278);
        gRXPARAM_snr = SX1278_ESTSNR_LoRa(&SX1278);
        //SendUART3(ret);HAL_Delay (100);
        slave_status = 4;
        gSYS_ACT_flag |= 0x4;      
      }
      else if(slave_status == 1)
      {
        //tx is busy
        SX1278_SPIRead(&SX1278, LR_RegIrqFlags);
			  SX1278_clearLoRaIrq(&SX1278); //Clear irq
			  SX1278_standby(&SX1278); //Entry Standby mode
        
        slave_status = 2;//rsptx done
        //slave will back to rx at pre-defined time slot
      }
    }
  }
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
