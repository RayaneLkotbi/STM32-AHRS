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
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_I2C_7BIT 0x68 // I2C address
#define MPU_I2C_8BIT (MPU_I2C_7BIT << 1)
#define MPU_WhoAmI 0x75
#define MPU_PWR 0x6B
#define MPU_DLPF 0x1A

#define ACC_XH 0x3B // Once you know this one, the rest is just the next hex number.
#define ACC_XL 0x3C
#define ACC_YH 0x3D
#define ACC_YL 0x3E
#define ACC_ZH 0x3F
#define ACC_ZL 0x40

#define TEMP_H 0x41 // Not using this
#define TEMP_L 0x42

#define GYRO_XH 0x43
#define GYRO_XL 0x44
#define GYRO_YH 0x45
#define GYRO_YL 0x46
#define GYRO_ZH 0x47
#define GYRO_ZL 0x48

#define ACC_SCALE 0x1C
#define GYRO_SCALE 0x1B

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t ax = 0; // Data or memory that requires continuous power to maintain information and is lost when power is cut
int16_t ay = 0;
int16_t az = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

float ax_sum = 0;
float ay_sum = 0;
float az_sum = 0;
float gx_sum = 0;
float gy_sum = 0;
float gz_sum = 0;

float ax_avg = 0;
float ay_avg = 0;
float az_avg = 0;
float gx_avg = 0;
float gy_avg = 0;
float gz_avg = 0;

float ax_real = 0;
float ay_real = 0;
float az_real = 0;

float gx_real = 0;
float gy_real = 0;
float gz_real = 0;

float AccScale = 16384; // 16384 LSB/g
float GyroScaleVal = 131; // 131 LSB (degree/s)

float AccRoll = 0;
float AccPitch = 0;
float GyroRoll = 0;
float GyroPitch = 0;

float PreviousRoll= 0;
float PreviousPitch= 0;
float Roll = 0;
float Pitch = 0;

char uartBuf[50];          // buffer to hold formatted string
uint32_t uartTimer = 0;    // timer for UART output

uint8_t motion[14];
uint32_t test = 1; // Just a test value... Don't mind

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
static void light(uint32_t time){ // use static to keep it local
	uint32_t dt = HAL_GetTick();
	while (HAL_GetTick() - dt < 1000){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(time);
	}

}

static void FailedLight(){
	while (true){
		light(500);
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
  MX_USART2_UART_Init(); // UART 115200 baud
  MX_I2C1_Init();
  HAL_Delay(200); // Let IMU wake-up...
  /* USER CODE BEGIN 2 */
  uint8_t WhoAmI = 0xFF;
  uint8_t Sleep = 0x00;
  uint8_t ReadSleepVal = 0xFF;
  uint8_t NoiseHz = 0x03;
  uint8_t ReadNoiseVal = 0xFF;
  uint8_t AccelRead = 0xFF;
  uint8_t GyroRead = 0xFF;
  bool AckLoop = true;
  bool WhoLoop = true;
  HAL_StatusTypeDef AckState = HAL_ERROR;
  HAL_StatusTypeDef WhoState = HAL_ERROR;
  HAL_StatusTypeDef WriteSleep = HAL_ERROR;
  HAL_StatusTypeDef ReadSleep = HAL_ERROR;
  HAL_StatusTypeDef WriteNoise = HAL_ERROR;
  HAL_StatusTypeDef ReadNoise = HAL_ERROR;
  HAL_StatusTypeDef AccelScale = HAL_ERROR;
  HAL_StatusTypeDef GyroScale = HAL_ERROR;

  uint32_t AckTick = HAL_GetTick();

  while (AckLoop){
	  AckState = HAL_I2C_IsDeviceReady(&hi2c1, MPU_I2C_8BIT, 1, 100);
	  if (AckState == HAL_OK){
		  AckLoop = false;
	  } else if (HAL_GetTick() - AckTick > 2000){
		  FailedLight();
	  }
  }

  uint32_t WhoTick = HAL_GetTick();

  while (WhoLoop){
	  WhoState = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_8BIT, MPU_WhoAmI, I2C_MEMADD_SIZE_8BIT, &WhoAmI, 1, 100);
	  if (WhoState == HAL_OK && WhoAmI == 0x68){
		  WhoLoop = false;
		  light(50);
		}else if (HAL_GetTick() - WhoTick > 2000){
			FailedLight();
		}
  }

  HAL_Delay(1000);

  uint32_t sleepTick = HAL_GetTick();

  WriteSleep = HAL_I2C_Mem_Write(&hi2c1, MPU_I2C_8BIT, MPU_PWR, I2C_MEMADD_SIZE_8BIT, &Sleep, 1 , 100);
  if (WriteSleep != HAL_OK){
	  FailedLight();
  }

  ReadSleep = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_8BIT, MPU_PWR, I2C_MEMADD_SIZE_8BIT, &ReadSleepVal, 1, 100);
  if (ReadSleep == HAL_OK && (ReadSleepVal & 0x40) == 0x00){
	  light(50);// success
  } else {
	  FailedLight();
  }

  HAL_Delay(1000);

  uint32_t noiseTick = HAL_GetTick();

  WriteNoise = HAL_I2C_Mem_Write(&hi2c1, MPU_I2C_8BIT, MPU_DLPF, I2C_MEMADD_SIZE_8BIT, &NoiseHz, 1, 100); // Low Pass Filter
  if (WriteNoise != HAL_OK){
	  FailedLight();
  }

  ReadNoise = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_8BIT, MPU_DLPF, I2C_MEMADD_SIZE_8BIT, &ReadNoiseVal, 1, 100);
  if (ReadNoise == HAL_OK && ReadNoiseVal == 0x03){
	  light(50);
  } else {
	  FailedLight();
  }

  HAL_Delay(1000);

  AccelScale = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_8BIT, ACC_SCALE, I2C_MEMADD_SIZE_8BIT, &AccelRead, 1, 100);
  GyroScale = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_8BIT, GYRO_SCALE, I2C_MEMADD_SIZE_8BIT, &GyroRead, 1, 100);

  uint32_t ScaleTick = HAL_GetTick();

  if (AccelScale == HAL_OK && GyroScale == HAL_OK && (AccelRead & 0x18) == 0x00 && (GyroRead & 0x18) == 0x00){ // Sanity check (Extract bit 3 and 4)
	  light(50); // Default Settings confirmed
  } else {
	  FailedLight();
  }

  float Timer = HAL_GetTick();
  float OutputTimer = HAL_GetTick();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
     if (HAL_GetTick() - Timer >= 10){
    	HAL_StatusTypeDef AccelState = HAL_ERROR;

    	    AccelState = HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_8BIT, ACC_XH, I2C_MEMADD_SIZE_8BIT, motion, 14, 100);
    	    if (AccelState != HAL_OK){
    	    	FailedLight();
    	    }

    	    ax = (motion[0] << 8) + motion[1]; // Shift the high byte by 8 and add the low byte
    	    ay = (motion[2] << 8) + motion[3];
    	    az = (motion[4] << 8) + motion[5];

    	    gx = (motion[8] << 8) + motion[9];
    	    gy = (motion[10] << 8) + motion[11];
    	    gz = (motion[12] << 8) + motion[13];

    	    if (0){ // Bias Calculation over 8000 tests (Done already) // Keeping it here to just check what I have done.
    	        ax_sum += ax;
    	        ay_sum += ay;
    	        az_sum += az;

    	        gx_sum += gx;
    	        gy_sum += gy;
    	        gz_sum += gz;

    	        ax_avg = ax_sum / test; // 891.23
    	        ay_avg = ay_sum / test; // 198.76
    	        az_avg = az_sum / test; // 16843.13

    	        gx_avg = gx_sum / test; // -698.03
    	        gy_avg = gy_sum / test; // 46.05
    	        gz_avg = gz_sum / test; // -12.62
    	    }

    	    ax_real = ((float)ax - 891.23) / AccScale; // 16864 LSB
    	    ay_real = ((float)ay - 198.76) / AccScale;
    	    az_real = ((float)az - 470.89) / AccScale;

    	    gx_real = ((float)gx + 698.03) / GyroScaleVal; // 131 LSB
    	    gy_real = ((float)gy - 44.05) / GyroScaleVal;
    	    gz_real = ((float)gz + 12.62) / GyroScaleVal;

    	    AccRoll = atan2(ax_real, az_real) * 180 /M_PI; // We use atan2 so we can compute accurately on all 4 quadrants (negative sign included).
    	    AccPitch = atan2(-ay_real, az_real) * 180/M_PI;

    	    float dt = (HAL_GetTick() - Timer)/1000;

    	    GyroRoll += -gy_real * dt;
    	    GyroPitch += -gx_real * dt;

    	    Roll = 0.98*(PreviousRoll + (-gy_real*dt)) + 0.02*AccRoll; // Complementary filter
    	    Pitch = 0.98*(PreviousPitch + (-gx_real*dt)) + 0.02*AccPitch; // Complementary filter

    	    PreviousRoll = Roll;
    	    PreviousPitch = Pitch;

    	    if (HAL_GetTick() - OutputTimer > 50 ){
        	    printf("Roll: %.2f, Pitch: %.2f\n", Roll, Pitch); // Outputting values
        	    OutputTimer = HAL_GetTick();
    	    }

    	    Timer = HAL_GetTick();
    	    test++; // Just to check Live Expression change ...
    }

  /* USER CODE END 3 */
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
#ifdef USE_FULL_ASSERT
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
