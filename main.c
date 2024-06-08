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
#include "adc.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FPEC.h"
#include "ESP8266.h"
#include "stm32f4xx_hal_flash.h"
#include "FLASH_SECTOR_F4.h"
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
volatile uint8_t  u8RecBuffer[100]  ;
volatile uint8_t  u8RecCounter    = 0;
volatile uint8_t  u8TimeOutFlag   = 0;
volatile uint8_t  u8BLWriteReq    = 1;
uint8_t data[60] ={0};
#define Version_Address  0x08010000
#define LED_Address	0x08020100
uint8_t one []={'1'};
uint8_t zero []={'0'};
uint8_t Asstric='*';
uint8_t  Speed = 60 ;

extern uint8_t error;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t PIEZO_SENSOR(void);
void autopilot(void);
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
	uint8_t Local_u8RecStatus;

	volatile int x = 0;
	uint32_t Flash_address =0x08020100;


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
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	char  Hellow [] = "Hello";
	char  Yosry [] = "Yosry";
	char AT [] = "AT\r\n";
	char Echo [] = "ATE1\r\n";
	char Mode [] ="AT+CWMODE=3\r\n";
	char Connicting_ToWifi []="AT+CWJAP_CUR=\"YOSRY\",\"MOHAMED331021490\"\r\n";
	char Connicting_ToWebServer  []="AT+CIPSTART=\"TCP\",\"192.168.1.3\",80\r\n";
	char SendNumberofTcpByets_ToWebServer[] =	  "AT+CIPSEND=44\r\n";
	char SendGET_Req_ToWebServer[] ="GET http://192.168.1.3:80?ID=15 HTTP/1.1\r\n";
	char Colse_Req_ToWebServer[] ="AT+CIPCLOSE=0\r\n";
	//HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	char Restart [] = "AT+RST\r\n";
	char Enter  [] = "000000000000000000\r\n";
	uint8_t TimStr='-';
	uint8_t Str = 'N';	uint8_t Rx_data[10];
	uint8_t Rx_data1[32];
	uint8_t Rx_UART1 = '\0';
	uint8_t Buffer_UART1 [32];
	uint8_t CRCFlag[1] = {'C'};
	uint8_t Flag[] = "D";

	uint8_t UART6_RX ;
	//TIM11->CCR1 =50;
	uint8_t *buffer;
	uint8_t version [] = {"Version:1.1"};
	uint32_t version_value [sizeof(version)] ;
	uint8_t var1;
	uint8_t Str1;

	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	uint8_t LED_RET = 'C';
	uint8_t  UART1_RX = '0';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*---------------------------connect to WiFi--------------------------*/
	//ESP8266_1_voidInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*---------------------------------mobile application----------------------------------*/
		TIM11->CCR1 =Speed;
		TIM2->CCR1 =Speed;
//		HAL_UART_Receive(&huart1, &UART1_RX, sizeof(&UART1_RX), 10);
	HAL_UART_Receive(&huart6, &UART6_RX, sizeof(&UART6_RX), 10);
//		if(UART1_RX == '*'){
//			for(int Data_size = 0 ;Data_size <50 ;Data_size ++){
//				HAL_UART_Receive(&huart1, &UART1_RX, sizeof(&UART1_RX), 10);
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//				if (UART1_RX == '*'){
//					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//					break;	}
//			}
//		}
		HAL_UART_Receive(&huart2, &Str, sizeof(Str), 50);
		Flash_Read_Data(LED_Address , &LED_RET, sizeof(zero));
		if(	LED_RET == '0')
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		else if(	LED_RET == '1')
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		}
		if(UART6_RX == 'S'){	HAL_UART_Transmit(&huart2, &UART6_RX, sizeof(UART6_RX), 50);UART6_RX ='0';}

		if(UART6_RX == 'V'){
			UART6_RX='6';
			HAL_UART_Transmit(&huart2, &UART6_RX, sizeof(UART6_RX), 50);UART6_RX ='0';}

		if(UART6_RX == 'D'){
			UART6_RX ='D';
			HAL_UART_Transmit(&huart2, &UART6_RX, sizeof(UART6_RX), 50);UART6_RX ='0';}

		if(UART6_RX == 'Z'){

			UART6_RX='8';

			HAL_UART_Transmit(&huart2, &UART6_RX, sizeof(UART6_RX), 50);UART6_RX ='0';}
		if (Str != 'N'){
			if(Str == 'U')
			{
				Flash_Read_Data(Version_Address, version_value, sizeof(version));
				HAL_UART_Transmit(&huart2, version_value, sizeof(version), 50);
			}
			if(Str == 'P')
			{
				Flash_Read_Data(LED_Address , &LED_RET, sizeof(zero));
				if(	LED_RET == '0')
				{
					Flash_Write_Data(LED_Address, one, sizeof(one)) ;
				}
				else if (	LED_RET == '1')
				{
					Flash_Write_Data(LED_Address, zero, sizeof(zero)) ;
				}//				else{Flash_Write_Data(LED_Address, one, sizeof(one)) ;}
			}
			if(Str == 'F')
			{
				HAL_UART_Transmit(&huart2, &Hellow, sizeof(Hellow), 50);

				FORWARD();

			}
			else if(Str == 'B')
			{
				BACK();
			}
			else if(Str == 'L')
			{
				LEFT();
			}
			else if(Str == 'R')
			{
				RIGHT();
			}
			else if(Str == 'J'){
				while(1){

					autopilot();
					HAL_UART_Receive(&huart2, &Str, sizeof(Str), 50);
					if (Str =='O'){

						break;
					}
				}


			}
			else if(Str == 'W')
			{
				HAL_UART_Receive(&huart2, &Speed, sizeof(Speed), 50);
				Speed -='0';
				Speed *=10;
				TIM11->CCR1 =Speed;
				TIM2->CCR1 =Speed;

			}
			else if(Str == 'T'){}

			else if(Str == 'G')
			{
				Str == 'N';
				uint8_t Counter = 0;
				while(1){
					HAL_UART_Receive(&huart2, &Rx_UART1, sizeof(Rx_UART1), 50);
					if (Rx_UART1 == 'Z'){
						if (Rx_UART1 == 'Z')
						{if(Counter <=2){continue;}
						Flag[0] = 'C';
						HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
						uint32_t (CRC_Val) = HAL_CRC_Calculate(&hcrc,Buffer_UART1, Counter);

						HAL_UART_Transmit(&huart1, &Asstric,sizeof(Asstric) , 50);

						HAL_UART_Transmit(&huart1, Buffer_UART1, Counter, 50);
						///uint8_t CRC_arr[4] = {((uint8_t)((CRC_Val)>>24)+'0'),((uint8_t)((CRC_Val)>>16)+'0'),((uint8_t)((CRC_Val)>>8)+'0'),((uint8_t)((CRC_Val)>>0)+'0')};
						uint8_t CRC1 =(uint8_t)(CRC_Val%10)+'0';
						CRC_Val/=10;
						uint8_t CRC2 =(uint8_t)(CRC_Val%10)+'0';
						CRC_Val/=10;
						uint8_t CRC3 =(uint8_t)(CRC_Val%10)+'0';
						CRC_Val/=10;
						uint8_t CRC4 =(uint8_t)(CRC_Val%10)+'0';
						uint8_t CRC_arr[]={CRC4,CRC3,CRC2,CRC1,'*','\r','\n'};

						HAL_UART_Transmit(&huart1, CRC_arr, sizeof(CRC_arr), 50);



						HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
						HAL_Delay(50);
						break;
						}
					}
					else
					{
						Buffer_UART1[Counter] = Rx_UART1;

						Counter ++;
					}
				}
			}

			else
			{
				STOP();
				Str ='N';
			}
		}
		else
		{
			STOP();
			Str ='N';
		}
		Str ='N';

		/*------------------------------------------V2I------------------------------------   */
		var1=PIEZO_SENSOR();
		uint8_t Flag_ADC[] = "E";
		if(var1>2)
		{
			HAL_UART_Transmit(&huart6, Flag_ADC, sizeof(Flag_ADC), 50);
			//	HAL_Delay(5000);
		}
		else if(var1<2)
		{
			continue;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void autopilot(void)
{
	/*----------------------------lane tracking----------------------------------*/
	uint8_t	   local_u8value=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	uint8_t    RS=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	uint8_t  speed = 50;

	/*----------------------------avoidness collision---------------------------*/
	uint8_t	   local_u8value2=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	uint8_t    local_u8value3=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

	if( local_u8value3 == 0 && local_u8value2 == 0)
	{
		STOP();
	}
	else
	{
		if(	local_u8value2 ==0)
		{
			TIM11->CCR1 =speed;
			TIM2->CCR1 =speed;
		}
		else
		{
			TIM11->CCR1 =Speed;
			TIM2->CCR1 =Speed;
			if((local_u8value == 0 )&&( RS == 1))
			{
				RIGHT();
			}
			else if((local_u8value == 1) &&( RS == 0))
			{
				LEFT();
			}
			else if((local_u8value == 0) &&( RS == 0))
			{
				FORWARD();
			}
		}
	}
}




void STOP(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}
void FORWARD(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}
void BACK(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}
void RIGHT(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}
void LEFT(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

uint32_t PIEZO_SENSOR(void)
{
	uint32_t var=0,voltage=0;
	HAL_ADC_Start(&hadc1);
	var= HAL_ADC_GetValue(&hadc1);
	/*ADC reference voltage =3.3 ----------- ADC resolution =4096*/
	voltage = (var * 3.3) /4096;
	return voltage ;
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
