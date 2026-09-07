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
#include <math.h>
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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
extern void flight_main(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADXL314_CS_PORT GPIOB
#define ADXL314_CS_PIN  GPIO_PIN_9

#define ADXL314_DEVID       0x00
#define ADXL314_BW_RATE     0x2C
#define ADXL314_POWER_CTL   0x2D
#define ADXL314_DATA_FORMAT 0x31
#define ADXL314_DATAX0      0x32

#define ADXL314_READ        0x80
#define ADXL314_MULTI_BYTE  0x40

#define ADXL314_SCALE_G_PER_LSB 0.04883f

static void ADXL314_CS_Low(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ADXL314_CS_PORT, ADXL314_CS_PIN, GPIO_PIN_RESET);
}

static void ADXL314_CS_High(void) {
    HAL_GPIO_WritePin(ADXL314_CS_PORT, ADXL314_CS_PIN, GPIO_PIN_SET);
}

static uint8_t ADXL314_ReadReg(uint8_t regAddr) {
    uint8_t txData[2];
    uint8_t rxData[2] = {0};

    txData[0] = regAddr | ADXL314_READ;
    txData[1] = 0x00;

    ADXL314_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    ADXL314_CS_High();

    return rxData[1];
}

static void ADXL314_WriteReg(uint8_t regAddr, uint8_t data) {
    uint8_t txData[2];

    txData[0] = regAddr & 0x7F;
    txData[1] = data;

    ADXL314_CS_Low();
    HAL_SPI_Transmit(&hspi1, txData, 2, 10);
    ADXL314_CS_High();
}

uint8_t ADXL314_Init(void) {
    ADXL314_CS_High();
    HAL_Delay(5);

    uint8_t id = ADXL314_ReadReg(ADXL314_DEVID);

    if (id != 0xE5) {
        return 0;
    }

    ADXL314_WriteReg(ADXL314_POWER_CTL, 0x00);

    ADXL314_WriteReg(ADXL314_BW_RATE, 0x0A);

    ADXL314_WriteReg(ADXL314_DATA_FORMAT, 0x0B);

    ADXL314_WriteReg(ADXL314_POWER_CTL, 0x08);

    HAL_Delay(5);

    return 1;
}

uint8_t ADXL314_ReadXYZ(ADXL314_Data *data) {
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    tx[0] = ADXL314_READ | ADXL314_MULTI_BYTE | ADXL314_DATAX0;

    ADXL314_CS_Low();

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, 10);

    ADXL314_CS_High();

    if (status != HAL_OK) {
        return 0;
    }

    data->x_raw = (int16_t)((rx[2] << 8) | rx[1]);
    data->y_raw = (int16_t)((rx[4] << 8) | rx[3]);
    data->z_raw = (int16_t)((rx[6] << 8) | rx[5]);

    data->x_g = data->x_raw * ADXL314_SCALE_G_PER_LSB;
    data->y_g = data->y_raw * ADXL314_SCALE_G_PER_LSB;
    data->z_g = data->z_raw * ADXL314_SCALE_G_PER_LSB;

    data->magnitude_g = sqrtf(
        data->x_g * data->x_g +
        data->y_g * data->y_g +
        data->z_g * data->z_g
    );

    return 1;
}
#define MS5607_CS_PORT GPIOB
#define MS5607_CS_PIN  GPIO_PIN_8

#define MS5607_CMD_RESET      0x1E
#define MS5607_CMD_ADC_READ   0x00

#define MS5607_CMD_CONV_D1_4096 0x48
#define MS5607_CMD_CONV_D2_4096 0x58

#define MS5607_CMD_PROM_READ  0xA0

static uint16_t ms5607_prom[8];

static void MS5607_CS_Low(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    HAL_GPIO_WritePin(MS5607_CS_PORT, MS5607_CS_PIN, GPIO_PIN_RESET);
}

static void MS5607_CS_High(void) {
    HAL_GPIO_WritePin(MS5607_CS_PORT, MS5607_CS_PIN, GPIO_PIN_SET);
}

static void MS5607_WriteCommand(uint8_t cmd) {
    MS5607_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    MS5607_CS_High();
}

static uint8_t MS5607_SPI_Transfer(uint8_t tx) {
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100);
    return rx;
}

static uint16_t MS5607_ReadPROM(uint8_t index) {
    uint8_t cmd = MS5607_CMD_PROM_READ + (index * 2);
    uint8_t msb;
    uint8_t lsb;

    MS5607_CS_Low();

    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);

    msb = MS5607_SPI_Transfer(0x00);
    lsb = MS5607_SPI_Transfer(0x00);

    MS5607_CS_High();

    return ((uint16_t)msb << 8) | lsb;
}

static uint32_t MS5607_ReadADC(void) {
    uint8_t cmd = MS5607_CMD_ADC_READ;
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;

    MS5607_CS_Low();

    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);

    b1 = MS5607_SPI_Transfer(0x00);
    b2 = MS5607_SPI_Transfer(0x00);
    b3 = MS5607_SPI_Transfer(0x00);

    MS5607_CS_High();

    return ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
}

static uint32_t MS5607_ConvertAndRead(uint8_t command) {
    MS5607_WriteCommand(command);

    HAL_Delay(10);

    return MS5607_ReadADC();
}

uint8_t MS5607_Init(void) {
    MS5607_CS_High();
    HAL_Delay(5);

    MS5607_WriteCommand(MS5607_CMD_RESET);

    HAL_Delay(5);

    for (uint8_t i = 0; i < 8; i++) {
        ms5607_prom[i] = MS5607_ReadPROM(i);
    }

    for (uint8_t i = 1; i <= 6; i++) {
        if (ms5607_prom[i] == 0x0000 || ms5607_prom[i] == 0xFFFF) {
            return 0;
        }
    }

    return 1;
}

uint8_t MS5607_Read(MS5607_Data *data) {
    uint32_t D1;
    uint32_t D2;

    int32_t dT;
    int32_t TEMP;

    int64_t OFF;
    int64_t SENS;
    int64_t P64;
    int32_t P;

    D1 = MS5607_ConvertAndRead(MS5607_CMD_CONV_D1_4096);
    D2 = MS5607_ConvertAndRead(MS5607_CMD_CONV_D2_4096);

    if (D1 == 0 || D2 == 0) {
        return 0;
    }

    data->D1 = D1;
    data->D2 = D2;

    dT = (int32_t)D2 - ((int32_t)ms5607_prom[5] << 8);

    TEMP = 2000 + (int32_t)(((int64_t)dT * ms5607_prom[6]) >> 23);

    OFF = ((int64_t)ms5607_prom[2] << 17)
        + (((int64_t)ms5607_prom[4] * dT) >> 6);

    SENS = ((int64_t)ms5607_prom[1] << 16)
         + (((int64_t)ms5607_prom[3] * dT) >> 7);

    if (TEMP < 2000) {
        int64_t T2;
        int64_t OFF2;
        int64_t SENS2;
        int64_t temp_diff;

        T2 = ((int64_t)dT * dT) >> 31;

        temp_diff = TEMP - 2000;
        OFF2 = (61 * temp_diff * temp_diff) >> 4;
        SENS2 = 2 * temp_diff * temp_diff;

        if (TEMP < -1500) {
            int64_t very_low_diff = TEMP + 1500;

            OFF2 += 15 * very_low_diff * very_low_diff;
            SENS2 += 8 * very_low_diff * very_low_diff;
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    P64 = (((int64_t)D1 * SENS) >> 21) - OFF;
    P = (int32_t)(P64 >> 15);

    data->temperature_c = TEMP / 100.0f;
    data->pressure_mbar = P / 100.0f;

    data->altitude_m =
        44330.0f * (1.0f - powf(data->pressure_mbar / 1013.25f, 0.19029495f));

    return 1;
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
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  if (!ADXL314_Init()) {
    Error_Handler();
  }
  if (!MS5607_Init()) {
    Error_Handler();
  }
  flight_main();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 95;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE8
                           PE12 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_12|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD14 PD1 PD2
                           PD3 PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
