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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "atk_mw579_uart.h"
#include "atk_mw579.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Dis = 0;
float t1;
float t2;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//是否由TIM1触发 + 是否由间接捕获触�?
	if(htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
	{
		t1 = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
		t2 = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
		Dis = ((t2-t1)*0.034) / 2;
	}
}


void RCCdelay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

void delay_us(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;
 
  startval = SysTick->VAL;
  tickn = HAL_GetTick();
  //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
  delays =udelay * 72; //sysc / 1000 * udelay;
  if(delays > startval)
    {
      while(HAL_GetTick() == tickn)
        {
 
        }
      wait = 72000 + startval - delays;
      while(wait < SysTick->VAL)
        {
 
        }
    }
  else
    {
      wait = startval - delays;
      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {
 
        }
    }
}


extern UART_HandleTypeDef huart4;
uint8_t tmp;
uint8_t ret = 0;
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
  MX_TIM1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_SET);
	

	//HAL_UART_Receive_IT(&huart4,&tmp,1);

	/*��ʼ��*/
  ret = atk_mw579_init(ATK_MW579_UART_BAUDRATE_115200);
	
	if(ret != 0){
		//ʧ��	
		while(1){
				HAL_GPIO_TogglePin(GPIOC, LED_Pin);
				HAL_Delay(1000);

		}
	
	}
	
	
	/* ����ATK-MW579 */
	atk_mw579_enter_config_mode();
	ret  = atk_mw579_set_name(DEMO_BLE_NAME);								//����ATK-MW579��������
	ret += atk_mw579_set_hello(DEMO_BLE_HELLO);							//����ATK-MW579������ӭ��
	ret += atk_mw579_set_tpl(ATK_MW579_TPL_P0DBM);					//����ATK-MW579���书��
	ret += atk_mw579_set_uart(ATK_MW579_UART_BAUDRATE_115200, ATK_MW579_UART_DATA_8, ATK_MW579_UART_PARI_NONE, ATK_MW579_UART_STOP_1);//����ATK-MW579���ڲ���
	ret += atk_mw579_set_adptim(DEMO_BLE_ADPTIM);						//����ATK-MW579�㲥�ٶ�
	ret += atk_mw579_set_linkpassen(ATK_MW579_LINKPASSEN_OFF);//����ATK-MW579��·ƥ��
	ret += atk_mw579_set_leden(ATK_MW579_LEDEN_ON);					//����ATK-MW579����LED
	ret += atk_mw579_set_slavesleepen(ATK_MW579_SLAVESLEEPEN_OFF);//����ATK-MW579���豸����˯��
	ret += atk_mw579_set_maxput(ATK_MW579_MAXPUT_OFF);			//����ATK-MW579ͨ��������
	ret += atk_mw579_set_mode(ATK_MW579_MODE_S);						//����ATK-MW579����ģʽ
	
	if(ret != 0){
		//ʧ��	
		while(1){
				HAL_GPIO_TogglePin(GPIOC, LED_Pin);
				HAL_Delay(500);

		}
	
	}
		
	atk_mw579_uart_rx_restart(); /* ���¿�ʼ�������� */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

		atk_mw579_uart_printf("From ATK-MW579\r\n");
		HAL_Delay(100);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
