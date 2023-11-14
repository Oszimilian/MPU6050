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
#include "mpu6050.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1)
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

#define SUCCESS 0
#define ERROR 1

struct MyFloat {
	int16_t l;
	int16_t r;
	float val;
};
typedef struct MyFloat MyFloat;

struct Accel {
	MyFloat x;
	MyFloat y;
	MyFloat z;

};
typedef struct Accel Accel;

struct Temp {

	MyFloat t;

};
typedef struct Temp Temp;

struct Gyro {
	MyFloat x;
	MyFloat y;
	MyFloat z;
};
typedef struct Gyro Gyro;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
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

int16_t accX, accY, accZ, gyrX, gyrY, gyrZ, tVal;
double temperature = 0.0;

uint16_t strlen(char* str) {
	int len = 0;
	while(*str != '\0') {
		str++;
		len++;
	}
	return len;
}

void uart_send_string(char *ptr) {
	  if(HAL_UART_Transmit(&huart2, (uint8_t*)ptr, strlen(ptr), 100) == HAL_OK) {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  } else {
		  while(1) {
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  }
	  }
}

void uart_send_byte(uint8_t* data) {

	char c[20];
	snprintf(c, sizeof(c), "%d  \r\n", *data);
	uart_send_string(c);
}

void uart_send_temp(Temp* temperature) {
	char c[20];
	sprintf(c, "Temp: %d.%d °C \r\n", temperature->t.l, temperature->t.r);

	uart_send_string(c);
}

void uart_send_gyro(Gyro* gyro) {
	char c[100];
	//sprintf(c, "Gyro x: %d.%d ° \r\nGyro y: %d.%d ° \r\nGyro z: %d.%d ° \r\n", gyro->x.l, gyro->x.r, gyro->y.l, gyro->y.r, gyro->z.l, gyro->z.r);
	//sprintf(c, "Gyro x: %f \r\nGyro y: %f\r\n Gyro z: %f\r\n", gyro->x.val, gyro->y.val, gyro->z.val);
	sprintf(c, "%f %f %f \r\n",gyro->x.val, gyro->y.val, gyro->z.val);
	uart_send_string(c);
}

void uart_send_acc(Accel* acc) {
	char c[100];
	sprintf(c, "Accel x: %d.%d  \r\nAccel y: %d.%d  \r\nAccel z: %d.%d  \r\n", acc->x.l, acc->x.r, acc->y.l, acc->y.r, acc->z.l, acc->z.r);
	uart_send_string(c);
}

void float_to_myfloat(float* val, MyFloat* myfloat) {
	myfloat->l = (int16_t) *val;
	myfloat->r = (int16_t)((*val - myfloat->l) * 1000);
}

uint8_t init_mpu6050(I2C_HandleTypeDef* i2c_ptr) {
	uint8_t check;
	uint8_t data;

	HAL_I2C_Mem_Read(i2c_ptr, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

	if(check == (MPU6050_ADDR >> 1)) {
		data = 0x00;
		if(HAL_I2C_Mem_Write(i2c_ptr, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100) == HAL_OK) uart_send_string("wake up sensor! \r\n");

		data = 0x07;
		if(HAL_I2C_Mem_Write(i2c_ptr, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100) == HAL_OK) uart_send_string("set data rate to 1kHz \r\n");

		data = 0x00;
		if(HAL_I2C_Mem_Write(i2c_ptr, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100) == HAL_OK) uart_send_string("set accelerometer config \r\n");

		data = 0x00;
		if(HAL_I2C_Mem_Write(i2c_ptr, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100) == HAL_OK) uart_send_string("set gyroscopic config \r\n");

		return SUCCESS;
	} else {
		uart_send_string("Error: device is not reachable  \r\n");
		return ERROR;
	}
}


uint8_t read_mpu6050_temp(I2C_HandleTypeDef* i2c_ptr, Temp* temperature) {
	uint8_t data[2];
	int16_t temp;

	if(HAL_I2C_Mem_Read(i2c_ptr, MPU6050_ADDR, TEMP_OUT_H_REG, 1, data, 2, 10) == HAL_OK) {
		temp = (int16_t)((data[0] << 8) | data[0]);
		temperature->t.val = (double)((int16_t)temp / 340.0 + 36.53);
		float_to_myfloat(&temperature->t.val, &temperature->t);

		return SUCCESS;
	} else {
		uart_send_string("Error: cant read temp\r\n ");
		return ERROR;
	}
}

uint8_t read_mpu6050_gyro(I2C_HandleTypeDef* i2c_ptr, Gyro* gyrovalue) {
	uint8_t data[6];

	if(HAL_I2C_Mem_Read(i2c_ptr, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data, 6, 100) == HAL_OK) {
		uint16_t x = ((data[0] << 8) | data[1]);
		gyrovalue->x.val = x / (131 * 0.001);
		//gyrovalue->x.val = (x / 131.0 - 151) * 0.2 * -1 * 0.15;
		//gyrovalue->x.val = (float)(((data[0] << 8) | data[1]));
		float_to_myfloat(&gyrovalue->x.val, &gyrovalue->x);

		uint16_t y = ((data[2] << 8) | data[3]);
		gyrovalue->y.val = y / (131 * 0.001);
		//gyrovalue->y.val = (float)(((data[2] << 8) | data[3]) / 131.0);
		//gyrovalue->y.val = (float)(((data[2] << 8) | data[3]));
		float_to_myfloat(&gyrovalue->y.val, &gyrovalue->y);

		uint16_t z = ((data[4] << 8) | data[5]);
		gyrovalue->z.val = z / (131 * 0.001);
		//gyrovalue->z.val = (float)(((data[4] << 8) | data[5]) / 131.0);
		//gyrovalue->z.val = (float)(((data[4] << 8) | data[5]));
		float_to_myfloat(&gyrovalue->z.val, &gyrovalue->z);

		return SUCCESS;
	} else {
		uart_send_string("Error: cant read gyro \r\n");
		return ERROR;
	}
}

uint8_t read_mpu6050_acc(I2C_HandleTypeDef* i2c_ptr, Accel* acc) {
	uint8_t data[6];

	if(HAL_I2C_Mem_Read(i2c_ptr, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 10) == HAL_OK) {
		acc->x.val = (float)(((data[0] << 8) | data[1]) / 16384.0);
		float_to_myfloat(&acc->x.val, &acc->x);

		acc->y.val = (float)(((data[2] << 8) | data[3]) / 16384.0);
		float_to_myfloat(&acc->y.val, &acc->y);

		acc->z.val = (float)(((data[4] << 8) | data[5]) / 16384.0);
		float_to_myfloat(&acc->z.val, &acc->z);

		return SUCCESS;
	} else {
		uart_send_string("Error: cant read gyro \r\n");
		return ERROR;
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
  //init_mpu6050(&hi2c1);
  while (MPU6050_Init(&hi2c1) == 1);

  Temp temperatur;
  Gyro gyro;
  Accel acc;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //if (HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)) {
		  /*
		  if(read_mpu6050_temp(&hi2c1, &temperatur) == SUCCESS) {
			  uart_send_temp(&temperatur);
		  }
		  */

	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  HAL_Delay (100);

	  	  char c[50];

	  	  sprintf(c, "%f %f \r\n", MPU6050.KalmanAngleX, MPU6050.KalmanAngleY);

	  	  uart_send_string(c);
		  /*
		  if(read_mpu6050_acc(&hi2c1, &acc) == SUCCESS) {
			  uart_send_acc(&acc);
		  }
		  */

		  //while(HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_Pin)) {
			  //HAL_Delay(50);
		  //}

		  //uart_send_string("\r\n\r\n");
	  //}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BlueButton_Pin */
  GPIO_InitStruct.Pin = BlueButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BlueButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
