/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    NONE,
    PREPARATION,
    FLASH_ERASING,
    FLASH_ERASE_DONE,
    FLASH_WRITE_IN_PROGRESS,
    FLASH_WRITE_DONE,
    UPDATE_DONE,
} updateState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Number of pages for firmware
#define NUM_PAGES 100

// Number of bytes for firmware
#define NUM_BYTES       NUM_PAGES * FLASH_PAGE_SIZE

// Number of double words (64 bits) for firmware
#define NUM_DOUBLEWORDS NUM_BYTES / 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile updateState_t updateState = NONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin == GPIO_PIN_13) {
        if (updateState == NONE) {
            updateState = PREPARATION;
        }
    }

}

void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue) {
    if (updateState == FLASH_ERASING && (ReturnValue == 1 || ReturnValue == 2)) {
        updateState = FLASH_ERASE_DONE;
    } else if (updateState == FLASH_WRITE_IN_PROGRESS) {
        updateState = FLASH_WRITE_DONE;
    }
}

void toggleBankAndReset() {
    FLASH_OBProgramInitTypeDef OBInit;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    HAL_FLASH_OB_Unlock();
    HAL_FLASHEx_OBGetConfig(&OBInit);

    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BFB2;

    if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE) {
        OBInit.USERConfig = OB_BFB2_DISABLE;
    } else {
        OBInit.USERConfig = OB_BFB2_ENABLE;
    }
    if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK) {
        // uint32_t errorCode = HAL_FLASH_GetError();
        while (1) {
            HAL_Delay(1000);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
    if (HAL_FLASH_OB_Launch() != HAL_OK) {
        //uint32_t errorCode = HAL_FLASH_GetError();
        while (1) {
            HAL_Delay(100);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

uint8_t getActiveBank() {
    volatile uint32_t remap = READ_BIT(SYSCFG->MEMRMP, 0x1 << 8);
    return remap == 0 ? 1 : 2;
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
  /* USER CODE BEGIN 2 */
    uint8_t bank = getActiveBank();

    uint32_t last = HAL_GetTick();

    uint32_t delay;
    if (bank == 1) {
        delay = 1000;
    } else {
        delay = 500;
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        uint32_t now = HAL_GetTick();
        if (now - last > delay) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            last = now;
        }
        switch (updateState) {
        case NONE:
            break;
        case PREPARATION: {
            FLASH_EraseInitTypeDef erase = { 0 };
            //erase.TypeErase = FLASH_TYPEERASE_PAGES;
            erase.TypeErase = FLASH_TYPEERASE_MASSERASE;
            erase.Banks = bank == 1 ? FLASH_BANK_2 : FLASH_BANK_1;
            //erase.NbPages = NUM_PAGES;
            //erase.Page = 0;

            HAL_FLASH_Unlock();
            HAL_StatusTypeDef status = HAL_FLASHEx_Erase_IT(&erase);
            if (status != HAL_OK) {
                // TODO error case
            }

            updateState = FLASH_ERASING;
        }
            break;

        case FLASH_ERASING:
        case FLASH_WRITE_IN_PROGRESS:
            delay = 200;
            break;

        case FLASH_ERASE_DONE:
        case FLASH_WRITE_DONE: {
            delay = 50;
            static size_t index = 0;

            uint32_t dest = 0x08040000;
            uint8_t *src = (uint8_t*) 0x08000000;

            if (index < NUM_DOUBLEWORDS) {
                uint64_t doubleword = *(uint64_t*) (src + (index * 8));

                updateState = FLASH_WRITE_IN_PROGRESS;
                HAL_FLASH_Program_IT(
                        FLASH_TYPEPROGRAM_DOUBLEWORD,
                        dest + index * 8,
                        doubleword);
                index++;
            } else {
                updateState = UPDATE_DONE;
            }
        }
            break;

        case UPDATE_DONE:
            toggleBankAndReset();
            break;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_TX_Pin|LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
