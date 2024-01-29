/* USER CODE BEGIN Header */
/*
Copyright(C) 2023 Colton Baldridge

This program is free software : you can redistribute it and /or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see < https://www.gnu.org/licenses/>.
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM_LDC16xx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOOTLOADER_ADDRESS 0x1FFFC400 // address of the bootloader ROM
#define LDC_HONE_DEADBAND 20 // the magnitude of all channel readings should be less than this before LDC honing begins
#define LDC_HONE_PERIOD 100 // this many samples should pass within LDC_HONE_DEADBAND before LDC honing executes
#define X_SCALE_FACTOR 120 // Scaling factors to bring the values up to us the full range of int16_t so other programs play nice
#define Y_SCALE_FACTOR 120
#define Z_SCALE_FACTOR 35
#define RX_SCALE_FACTOR 30
#define RY_SCALE_FACTOR 30
#define RZ_SCALE_FACTOR 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
typedef enum {
  HONED = 0,
  ACTIVE = 1, 
} honing_state_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
extern USBD_HandleTypeDef hUsbDeviceFS;

static inline void sendGamepadReport(int16_t x, int16_t y, int16_t z, int16_t rx, int16_t ry, int16_t rz)
{
  int16_t buffer[6] = {x, y, z, rx, ry, rz};
  USBD_HID_SendReport(&hUsbDeviceFS, buffer, sizeof(buffer));
}

int16_t boundToInt16(int32_t value) {
    if (value < INT16_MIN) {
        return INT16_MIN;
    } else if (value > INT16_MAX) {
        return INT16_MAX;
    } else {
        return (int16_t)value;
    }
}

void jumpToBootloader(void) {
  // Taken from https://community.st.com/t5/stm32-mcus/how-to-jump-to-system-bootloader-from-application-code-on-stm32/tac-p/596379/highlight/true#M617 
  void (*SysMemBootJump)(void);
  uint8_t i;

  __disable_irq();
  // Reset USB
  USB->CNTR = 0x0003;

  //De-init all peripherals
  HAL_I2C_DeInit(&hi2c1);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

  // Disable Systick
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  // Reset clock to default
  HAL_RCC_DeInit();

  // Clear all interrupt bits
  for (i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  __enable_irq();

  SysMemBootJump = (void (*)(void)) (*((uint32_t *) (BOOTLOADER_ADDRESS + 4)));
  __set_MSP(*(uint32_t *)BOOTLOADER_ADDRESS);
  SysMemBootJump();

  while (1); // Just in case...
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 LDC_configReg default_config[] = {
  { LDC16xx_CLOCK_DIVIDERS_CH0,   0x5002 }, // get weird behavior with other dividers like 1002, just stick with this for now
  { LDC16xx_CLOCK_DIVIDERS_CH1,   0x5002 },
  { LDC16xx_CLOCK_DIVIDERS_CH2,   0x5002 },
  { LDC16xx_CLOCK_DIVIDERS_CH3,   0x5002 },
  { LDC16xx_SETTLECOUNT_CH0,      0x0040 },
  { LDC16xx_SETTLECOUNT_CH1,      0x0040 },
  { LDC16xx_SETTLECOUNT_CH2,      0x0040 },
  { LDC16xx_SETTLECOUNT_CH3,      0x0040 },
  { LDC16xx_RCOUNT_CH0,           0x1fff },
  { LDC16xx_RCOUNT_CH1,           0x1fff },
  { LDC16xx_RCOUNT_CH2,           0x1fff },
  { LDC16xx_RCOUNT_CH3,           0x1fff },
  { LDC16xx_DRIVE_CURRENT_CH0,    0xD000 },
  { LDC16xx_DRIVE_CURRENT_CH1,    0xD000 },
  { LDC16xx_DRIVE_CURRENT_CH2,    0xD000 },
  { LDC16xx_DRIVE_CURRENT_CH3,    0xD000 },
  { LDC16xx_ERROR_CONFIG,         0x0001 },
  { LDC16xx_MUX_CONFIG,           LDC16xx_BITS_DEGLITCH_3_3Mhz | LDC16xx_BITS_AUTOSCAN_EN | LDC16xx_BITS_RR_SEQUENCE_CH0_CH1_CH2_CH3 },
  { LDC16xx_CONFIG,               LDC16xx_BITS_ACTIVE_CHAN_CH0 // select channel 0
                                  | LDC16xx_BITS_AUTO_AMP_DIS // if 0, IDRIVE constantly adjusts
                                  | LDC16xx_BITS_RP_OVERRIDE_EN // if 0, IDRIVE auto calibrates on sensor startup
                                  | LDC16xx_BITS_INTB_DIS // disable intb pin
                                  | LDC16xx_BITS_REF_CLK_SRC // 40MHz ext clock
                                  // | LDC16xx_BITS_SLEEP_MODE_EN // start in sleep mode
                                  }, 
};

LDC_configReg SLEEP_CONFIG_config = { LDC16xx_CONFIG, LDC16xx_BITS_ACTIVE_CHAN_CH0 | LDC16xx_BITS_AUTO_AMP_DIS | LDC16xx_BITS_INTB_DIS | LDC16xx_BITS_SLEEP_MODE_EN};
LDC_configReg AWAKE_CONFIG_config = { LDC16xx_CONFIG, LDC16xx_BITS_ACTIVE_CHAN_CH0 | LDC16xx_BITS_AUTO_AMP_DIS | LDC16xx_BITS_INTB_DIS};

#define LDC_CONFIG_SIZE sizeof(default_config)/sizeof(LDC_configReg)
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  uint8_t is_earlier_revision = 0;
  // Check if user has earlier revision of board where UART pins are shorted by default
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
    is_earlier_revision = 1;
  }
  // Reset the LDCs
  resetDevice(0x2a);
  resetDevice(0x2b);
  // Load the config onto the LDC1614
  loadConfig(0x2a, default_config, LDC_CONFIG_SIZE);
  // Change the mux register in the config to only scan 2 channels instead of 4 (for LDC1612)
  LDC_configReg LDC1612_MUX_config = {LDC16xx_MUX_CONFIG, LDC16xx_BITS_DEGLITCH_3_3Mhz | LDC16xx_BITS_AUTOSCAN_EN | LDC16xx_BITS_RR_SEQUENCE_CH0_CH1};
  default_config[16] = LDC1612_MUX_config;
  // Send the new config to the LDC1612
  loadConfig(0x2b, default_config, LDC_CONFIG_SIZE);

  // Channel data and cal data variables
  uint32_t ldc1_ch0 = 0;
  uint32_t ldc1_ch1 = 0;
  uint32_t ldc1_ch2 = 0;
  uint32_t ldc1_ch3 = 0;
  uint32_t ldc2_ch0 = 0;
  uint32_t ldc2_ch1 = 0;
  int32_t ldc1_ch0_dif = 0;
  int32_t ldc1_ch1_dif = 0;
  int32_t ldc1_ch2_dif = 0;
  int32_t ldc1_ch3_dif = 0;
  int32_t ldc2_ch0_dif = 0;
  int32_t ldc2_ch1_dif = 0;
  uint32_t ldc1_ch0_cal = 0;
  uint32_t ldc1_ch1_cal = 0;
  uint32_t ldc1_ch2_cal = 0;
  uint32_t ldc1_ch3_cal = 0;
  uint32_t ldc2_ch0_cal = 0;
  uint32_t ldc2_ch1_cal = 0;

  // Honing counter
  uint32_t ldc_honing_count = 0;
  honing_state_t ldc_honing_state = HONED;
  uint8_t in_deadband = 1;

  // Grab starting values
  readChannel(0x2a, 0, &ldc1_ch0_cal);
  readChannel(0x2a, 1, &ldc1_ch1_cal);
  readChannel(0x2a, 2, &ldc1_ch2_cal);
  readChannel(0x2a, 3, &ldc1_ch3_cal);

  readChannel(0x2b, 0, &ldc2_ch0_cal);
  readChannel(0x2b, 1, &ldc2_ch1_cal);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET && !is_earlier_revision) {
      jumpToBootloader();
    }
    // Grab a new set of values
    readChannel(0x2a, 0, &ldc1_ch0);
    readChannel(0x2a, 1, &ldc1_ch1);
    readChannel(0x2a, 2, &ldc1_ch2);
    readChannel(0x2a, 3, &ldc1_ch3);

    readChannel(0x2b, 0, &ldc2_ch0);
    readChannel(0x2b, 1, &ldc2_ch1);

    // Compare to cal'd values
    ldc1_ch0_dif = ((int32_t)ldc1_ch0_cal - (int32_t)ldc1_ch0) / 125;
    ldc1_ch1_dif = ((int32_t)ldc1_ch1_cal - (int32_t)ldc1_ch1) / 125;
    ldc1_ch2_dif = ((int32_t)ldc1_ch2_cal - (int32_t)ldc1_ch2) / 125;
    ldc1_ch3_dif = ((int32_t)ldc1_ch3_cal - (int32_t)ldc1_ch3) / 125;
    ldc2_ch0_dif = ((int32_t)ldc2_ch0_cal - (int32_t)ldc2_ch0) / 125;
    ldc2_ch1_dif = ((int32_t)ldc2_ch1_cal - (int32_t)ldc2_ch1) / 125;

    // Perform honing (if necessary)
    switch (ldc_honing_state){
      case HONED:
        if(
          abs(ldc1_ch0_dif) > LDC_HONE_DEADBAND ||
          abs(ldc1_ch1_dif) > LDC_HONE_DEADBAND ||
          abs(ldc1_ch2_dif) > LDC_HONE_DEADBAND ||
          abs(ldc1_ch3_dif) > LDC_HONE_DEADBAND ||
          abs(ldc2_ch0_dif) > LDC_HONE_DEADBAND ||
          abs(ldc2_ch1_dif) > LDC_HONE_DEADBAND
        ){
          ldc_honing_state = ACTIVE;
        }
        break;
      case ACTIVE:
        if(
          abs(ldc1_ch0_dif) < LDC_HONE_DEADBAND &&
          abs(ldc1_ch1_dif) < LDC_HONE_DEADBAND &&
          abs(ldc1_ch2_dif) < LDC_HONE_DEADBAND &&
          abs(ldc1_ch3_dif) < LDC_HONE_DEADBAND &&
          abs(ldc2_ch0_dif) < LDC_HONE_DEADBAND &&
          abs(ldc2_ch1_dif) < LDC_HONE_DEADBAND
        ){
          ldc_honing_count++;
          in_deadband = 1;
        }else{
          ldc_honing_count = 0;
          in_deadband = 0;
        }
        if(ldc_honing_count > LDC_HONE_PERIOD){
          ldc1_ch0_cal = ldc1_ch0;
          ldc1_ch1_cal = ldc1_ch1;
          ldc1_ch2_cal = ldc1_ch2;
          ldc1_ch3_cal = ldc1_ch3;
          ldc2_ch0_cal = ldc2_ch0;
          ldc2_ch1_cal = ldc2_ch1;
          ldc_honing_count = 0;
          ldc_honing_state = HONED;
        }
      default:
        break;
    }   


    // Get sums and differences
    int32_t cm1 = ldc1_ch0_dif+ldc1_ch1_dif;
    int32_t dm1 = ldc1_ch0_dif-ldc1_ch1_dif;
    int32_t cm2 = ldc1_ch2_dif+ldc1_ch3_dif;
    int32_t dm2 = ldc1_ch2_dif-ldc1_ch3_dif;
    int32_t cm3 = ldc2_ch0_dif+ldc2_ch1_dif;
    int32_t dm3 = ldc2_ch0_dif-ldc2_ch1_dif;

    // Compute tranformation
    int32_t z = cm1 + cm2 + cm3;
    int32_t y = - dm1 + dm3;
    int32_t x = - (dm2 - dm1 / 2 - dm3 / 2);
    int32_t rz = dm1 + dm2 + dm3;
    int32_t rx = cm1 / 2 - cm2 + cm3 / 2;
    int32_t ry =  - cm1 + cm3;

    if(in_deadband){
      z = 0;
      y = 0;
      x = 0;
      rz = 0;
      rx = 0;
      ry = 0;
    }

    x = X_SCALE_FACTOR * x;
    y = Y_SCALE_FACTOR * y;
    z = Z_SCALE_FACTOR * z;
    rx = RX_SCALE_FACTOR * rx;
    ry = RY_SCALE_FACTOR * ry;
    rz = RZ_SCALE_FACTOR * rz;
    // Delay a bit for the LDCs to get new readings 
    // (in the future, add INTB pin support so the LDCs can alert the MCU when they have new data ready)
    HAL_Delay(20);

    // Send the data.
    sendGamepadReport(boundToInt16(x),boundToInt16(y),boundToInt16(z),boundToInt16(rx),boundToInt16(ry),boundToInt16(rz));

    // Debug send for if you want the raw coil data
    // sendGamepadReport(
    //   ldc1_ch0_dif,
    //   ldc1_ch1_dif,
    //   ldc1_ch2_dif,
    //   ldc1_ch3_dif,
    //   ldc2_ch0_dif,
    //   ldc2_ch1_dif
    // );

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

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
  hi2c1.Init.Timing = 0x2010091A;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
