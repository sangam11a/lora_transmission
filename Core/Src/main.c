/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "app_subghz_phy.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"radio_driver.h"
#include "stdio.h"
#include "string.h"
#include<stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PAYLOAD_LENGTH          (5)
#define CMD_LEN					(5) // Receiveing GS Command
#define CW_TX_LEN				(5)

//#define CRC_TYPE                (0x01) // crc disabled
//#define WHITENING               (0x00) // whitening disabled

#define FREQ_401_MHZ          	(401375000)	//  (401375000) //dlink
#define FREQ_402_MHZ		  	(402375000)	//	(402375000) //UPLINK

#define PA_DUTY_CYCLE           (0x04)
#define HP_MAX                  (0x07)
#define PA_SEL                  (0x00)

#define POWER                   (0x15)
#define RAMP_TIME               (0x05)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static __IO uint8_t TX_Flag = 0;
uint8_t rx_ack[2] = { 0xac, 0xad };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tx_buf[PAYLOAD_LENGTH];
uint8_t rx_buf[CMD_LEN] = { 0 };
uint8_t p_len = CMD_LEN;
uint8_t RX_FLAG=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DioIrqHndlr(RadioIrqMasks_t radioIrq) {
	switch (radioIrq) {
	case IRQ_RADIO_NONE:

		break;
	case IRQ_RX_DONE:

		myprintf("DATA RECEIVED: \n");
		SUBGRF_GetPayload(rx_buf, &p_len, PAYLOAD_LENGTH);
		myprintf("%s",rx_buf);
		HAL_UART_Transmit_IT(&huart1, rx_buf, CMD_LEN);

		SUBGRF_SetSwitch(1, RFSWITCH_RX);
		SUBGRF_SetRxBoosted(0xFFFFFF);
//		rx_buf[33] = SUBGRF_GetRssiInst();
//		RX_FLAG =1;
		break;

	case IRQ_TX_DONE:


		memset((char*) tx_buf, '\0', PAYLOAD_LENGTH);

		//            myprintf("%s",tx_buf);
		SUBGRF_SetSwitch(1, RFSWITCH_RX); /*Set RF switch*/
		SUBGRF_SetRxBoosted(0xFFFFFF);
		//            myprintf("\n\rGoing RX_Mode\n\r");
		SUBGRF_SetRfFrequency(FREQ_402_MHZ);
		memset(rx_buf, '\0', CMD_LEN); //Clear RX_BUFFER

		break;
	case IRQ_PREAMBLE_DETECTED:

		break;
	case IRQ_SYNCWORD_VALID:

		break;
	case IRQ_HEADER_VALID:

		break;
	case IRQ_HEADER_ERROR:

		break;
	case IRQ_CRC_ERROR:

		break;
	case IRQ_CAD_CLEAR:

		break;
	case IRQ_CAD_DETECTED:

		break;
	case IRQ_RX_TX_TIMEOUT:

		break;
	}
}


int buffersize(char *buff) {
	int i = 0;

	while (*buff++ != '\0')
		i++;
	return i;
}
void myprintf(const char *fmt, ...) {
	static char temp[255];
	va_list args;
	va_start(args, fmt);
	vsnprintf(temp, sizeof(temp), fmt, args);
	va_end(args);
	int len = buffersize(temp);
	HAL_UART_Transmit(&huart1, (uint8_t*)temp, len, 5000);

}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SubGHz_Phy_Init();
  /* USER CODE BEGIN 2 */



  PacketParams_t pkt_params;
  	pkt_params.PacketType = PACKET_TYPE_LORA;
  	pkt_params.Params.LoRa.PayloadLength = PAYLOAD_LENGTH;
  	pkt_params.Params.LoRa.PreambleLength = 8;
  	pkt_params.Params.LoRa.HeaderType = LORA_PACKET_IMPLICIT;
  	pkt_params.Params.LoRa.CrcMode = LORA_CRC_ON;
  	pkt_params.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

  ModulationParams_t mod_params;
  	mod_params.PacketType = PACKET_TYPE_LORA;
  	mod_params.Params.LoRa.Bandwidth = LORA_BW_031;
  	mod_params.Params.LoRa.SpreadingFactor = LORA_SF10;
  	mod_params.Params.LoRa.CodingRate = LORA_CR_4_8;
  	mod_params.Params.LoRa.LowDatarateOptimize = 1;


	SUBGRF_Init(DioIrqHndlr);
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);
	SUBGRF_SetPayload(tx_buf, CMD_LEN);
	SUBGRF_SetPacketParams(&pkt_params);
	SUBGRF_SetSyncWord(( uint8_t[] ) { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00,
					0x00 });
	SUBGRF_SetWhiteningSeed(0x01FF);
	SUBGRF_SetRfFrequency(FREQ_402_MHZ);
	SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
	SUBGRF_SetTxParams(RFO_HP, POWER, RAMP_TIME);
	SUBGRF_SetModulationParams(&mod_params);
	SUBGRF_SetDioIrqParams(
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);


	myprintf("########## MUNAL SATELLITE COM: BEGIN ##########\r\n");

	myprintf("########## COMMUNICATION PARAMETERS:  ##########\r\n");
	myprintf("Modulation  LoRa PACKET	./\r\n");
	myprintf("FREQUENCY MODS: UPLINK FREQ: %lu\r\n DOWNLINK FREQ: %lu\r\n",
	FREQ_402_MHZ, FREQ_401_MHZ);
	myprintf(
			"POWER CONFIG: PA_DUTY_CYCLE : %x, HP_MAX: %x,PA_SEL : %x \n\r POWER TX: %u dBm\n\r",
			PA_DUTY_CYCLE, HP_MAX, PA_SEL, POWER);
	myprintf("RECEVING BANDWIDTH: 	%d\n\r", mod_params.Params.LoRa.Bandwidth);
	myprintf("Packet Type 			%d\n\r  ", pkt_params.PacketType);
	myprintf("PayloadLength 			%d\n\r  ", pkt_params.Params.LoRa.PayloadLength);
	myprintf("PreambleLength 		%d\n\r", pkt_params.Params.LoRa.PreambleLength);
	myprintf("PreambleMinDetect		%d\n\r",
			pkt_params.Params.Gfsk.PreambleMinDetect);
	myprintf("HeaderType 			%d\n\r", pkt_params.Params.LoRa.HeaderType);
	myprintf("__________________________________________________\r\n");
	myprintf("________________Waiting OBC DATA____________\r\n");

	SUBGRF_SetSwitch(RFO_HP, RFSWITCH_RX); /*Set RF switch*/
	SUBGRF_SetRxBoosted(0xFFFFFF);
	HAL_UART_Receive_IT(&huart1,tx_buf,sizeof(tx_buf));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_SubGHz_Phy_Process();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Transmit_IT(&huart1,"received",8);
	SUBGRF_SetSwitch(1, RFSWITCH_TX);
	SUBGRF_SendPayload("lorada", 6, 0);
	SUBGRF_SendPayload(tx_buf, sizeof(tx_buf), 0);
	SUBGRF_SetSwitch(1, RFSWITCH_RX);
	HAL_UART_Receive_IT(&huart1,tx_buf,sizeof(tx_buf));
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
  /* USER CODE BEGIN 5 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 5 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

