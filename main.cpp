/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "sensor_fusion2.h"
#include "millis.h"
#include "quaternion.h"
#include<stdio.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PI 3.14159265358

#ifdef __cplusplus
 extern "C" {
#endif

	int __io_putchar(int ch)
	{
	 uint8_t c[1];
	 c[0] = ch & 0x00FF;
	 HAL_UART_Transmit(&huart2, &*c, 1, 100);
	 return ch;
	}

	int _write(int file,char *ptr, int len)
	{
	 int DataIdx;
	 for(DataIdx= 0; DataIdx< len; DataIdx++)
	 {
	 __io_putchar(*ptr++);
	 }
	return len;
	}

#ifdef __cplusplus
}
#endif
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  float gyro_data[3] = {0, 0, 0};
    float accel_data[3] = {0, 0, 0};
    bool read = true;
    MPU6050 m_hi2c(&hi2c1);
    m_hi2c.start();
    millis_begin();
    uint32_t tick = 0;

    if (HAL_I2C_IsDeviceReady(&hi2c1, 0xD1, 2, HAL_MAX_DELAY) == HAL_OK)
    {
  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }


    vector orient;
    orient.x = 0;
    orient.y = 0;
    orient.z = 1;
    vector comp;
    comp.x = 0;
    comp.y = 0;
    comp.z = 1;

  //bias compensation
    float trialNum = 1;
    float g_adjust[3] = {0, 0, 0};
    float a_adjust[3] = {0, 0, 0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
      {
    	  read = m_hi2c.read_raw(&gyro_data[0], &gyro_data[1], &gyro_data[2], &accel_data[0], &accel_data[1], &accel_data[2]);

    	  //bias compensation
//    	  if (trialNum <= 20)
//    	  {
//    		  for (int i = 0; i < 3; i++)
//    		  {
//    			  g_adjust[i] = (g_adjust[i] * (trialNum - 1) + gyro_data[i]) / trialNum;
//    			  a_adjust[i] = (a_adjust[i] * (trialNum - 1) + accel_data[i]) / trialNum;
//    		  }
//    	  }
//    	  else
//    	  {
//    		  for (int i = 0; i < 2; i++)
//    		  {
//    			  gyro_data[i] -= g_adjust[i];
//    			  accel_data[i] -= a_adjust[i];
//    		  }
//    		  gyro_data[2] -= g_adjust[2];
//    		  accel_data[2] -= (a_adjust[2] - 1);

//    	  }
    	  trialNum++;

    	  //convert accel data to unit vector
    	  vector a_u;
    	  a_u.x = 0;
    	  a_u.y = 0;
    	  a_u.z = 0;
    	  vector a;
    	  a.x = accel_data[0];
    	  a.y = accel_data[1];
    	  a.z = accel_data[2];

    	  printf("%f \r\n ", a.x);
    	  vector_normalize(&a, &a_u);

    	  //convert gyro data
      	  vector g;
      	  g.x = gyro_data[0] * PI / 180.0 * (millis() - tick) / 1000.0;
      	  g.y = gyro_data[1] * PI / 180.0 * (millis() - tick) / 1000.0;
       	  g.z = gyro_data[2] * PI / 180.0 * (millis() - tick) / 1000.0;
       	  tick = millis();
      	  vector g_u;
      	  quaternion q;
      	  float g_mag = vector_normalize(&g, &g_u);
      	  g_u.x *= -1;
      	  g_u.y *= -1;
      	  g_u.z *= -1;
      	  quaternion_create(&g_u, g_mag, &q);
      	  quaternion_rotate(&orient, &q , &orient);
      	  vector_normalize(&orient, &orient);

      	  //comp filter
      	  float alpha = 0.3;
      	  vector comp1;
      	  vector_multiply(&a_u, alpha, &comp1);
      	  vector rotated_comp;
      	  quaternion_rotate(&comp, &q, &rotated_comp);
      	  vector comp2;
      	  vector_multiply(&rotated_comp, 1 - alpha, &comp2);
      	  vector_add(&comp1, &comp2, &comp);
      	  vector_normalize(&comp, &comp);

      	  //printf("%f  %f  %f  %f  %f  %f  %f  %f  %f\r\n", a_u.x, a_u.y, a_u.z, orient.x, orient.y, orient.z, comp.x, comp.y, comp.z);

    	  HAL_Delay(100);


    	  //remove const adjustment, take first 20 ish samples
    	  //wait when actually implementing, leave on flat
    	  //can be > 20, delay rn removed
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(myLED_GPIO_Port, myLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : myLED_Pin */
  GPIO_InitStruct.Pin = myLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(myLED_GPIO_Port, &GPIO_InitStruct);

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

