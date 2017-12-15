/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "sd_hal_mpu6050.h"
#include "stm32f1xx.h"
//#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#define ACCELERATION_THRESHOLD 10000
#define ACCELERATION_THRESHOLD_AFTER_VARIATION 2500
#define SHORT_SLEEP 200 // 1/5 sec.
#define LONG_SLEEP 1000 // 1 sec.

void turnLedOn();
int get_norm(); // Returns norm from acceleration (x,y,z);
bool calculate_fall(); // Calculates fall in specific case.

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SD_MPU6050 mpu1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	SD_MPU6050_Result result ;
	uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
	uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
  	  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
  	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  	  HAL_Delay(50);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  //	  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
  //	  int16_t a_x = mpu1.Accelerometer_X;
  //	  int16_t a_y = mpu1.Accelerometer_Y;
  //	  int16_t a_z = mpu1.Accelerometer_Z;
  //
  //	  SD_MPU6050_ReadGyroscope(&hi2c1, &mpu1);
  //	  int16_t g_x = mpu1.Gyroscope_X;
  //	  int16_t g_y = mpu1.Gyroscope_Y;
  //	  int16_t g_z = mpu1.Gyroscope_Z;
  //	  HAL_Delay(10);
  	  printf("Calculating...\n");
  	  if (calculate_fall()) {
  		  turnLedOn();
  		  HAL_Delay(LONG_SLEEP*2);
  		}


  /* USER CODE END 3 */

    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void turnLedOn() {
	int i =0;
	for(i=0;i<5;i++){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		HAL_Delay(1000);
	}
}

//int get_norm() {
//	  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
//	  int16_t a_x = mpu1.Accelerometer_X;
//	  int16_t a_y = mpu1.Accelerometer_Y;
//	  int16_t a_z = mpu1.Accelerometer_Z;
//	  HAL_Delay(10);
//	  double a2 = pow(a_x, a_x);
//	  double teste = (a_x*a_x)+(a_y*a_y)+(a_z*a_z);
////	  double teste = sqrt((double) ());
//	  return a_z >= 0 ? a_z : a_z*(-1);
//}

int get_norm() {
	  SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
	  int a_x = mpu1.Accelerometer_X;
	  int a_y = mpu1.Accelerometer_Y;
	  int a_z = mpu1.Accelerometer_Z;
	  double quadrado_x = pow(a_x,2);
	  double quadrado_y = pow(a_y,2);
	  double quadrado_z = pow(a_z,2);
	  double norma = sqrt(quadrado_x+quadrado_y+quadrado_z);
	  HAL_Delay(10);
	  return norma >= 0 ? norma : norma*(-1);
}

bool calculate_fall() {
	int initialAcceleration = get_norm();
	HAL_Delay(SHORT_SLEEP);
	int finalAcceleration = get_norm();

	if (finalAcceleration - initialAcceleration > ACCELERATION_THRESHOLD) {
		HAL_Delay(LONG_SLEEP*3);
		int acceleration = get_norm();
		HAL_Delay(SHORT_SLEEP);

		for (int i=0; i<10; i++) {
			int delta = get_norm() - acceleration;
			acceleration = get_norm();

			if (delta > ACCELERATION_THRESHOLD_AFTER_VARIATION) {
				return false;
			}
			HAL_Delay(SHORT_SLEEP);
		}
		return true;
	}
	return false;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
