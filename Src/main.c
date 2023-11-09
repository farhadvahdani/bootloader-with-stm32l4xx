/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bootloader.h"
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
SD_HandleTypeDef hsd1;

/* USER CODE BEGIN PV */

//static uint8_t BTNcounter = 0;
/* USER CODE BEGIN Includes */
//extern char SDPath[4]; /* SD logical drive path */
//extern FATFS SDFatFs;  /* File system object for SD logical drive */
//extern FIL SDFile;     /* File object for SD */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */
void Enter_Bootloader(void);
static void MX_GPIO_DeInit(void);
uint8_t SD_Init(void);
void SD_DeInit(void);
void SD_Eject(void);
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
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
/* Check system reset flags */
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST))
	{
#if(CLEAR_RESET_FLAGS)
        /* Clear system reset flags */
			__HAL_RCC_CLEAR_RESET_FLAGS();
        //print("Reset flags cleared.");
#endif
	}
	Enter_Bootloader();
	/* Check if there is application in user flash area */
	if(Bootloader_CheckForApplication() == BL_OK)
	{
#if(USE_CHECKSUM)
			/* Verify application checksum */
			if(Bootloader_VerifyChecksum() != BL_OK)
			{
					//print("Checksum Error.");
					Error_Handler();
			}
			else
			{
					//print("Checksum OK.");
			}
#endif

			HAL_Delay(1000);
			/* De-initialize bootloader hardware & peripherals */
			SD_DeInit();
			MX_GPIO_DeInit();
			/* Launch application */
			Bootloader_JumpToApplication();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED3_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_GPIO_DeInit(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();

}
void Enter_Bootloader(void)
{
    FRESULT fr;
    UINT num;
    uint8_t i;
    uint8_t status;
    uint64_t data;
    uint32_t cntr;
    uint32_t addr;

    /* Check for flash write protection */
    if(Bootloader_GetProtectionStatus() & BL_PROTECTION_WRP)
    {
        //print("Application space in flash is write protected.");
        //print("Press button to disable flash write protection...");
        //LED_R_ON();
        for(i = 0; i < 100; ++i)
        {
            //LED_Y_TG();
            HAL_Delay(50);
						Bootloader_ConfigProtection(BL_PROTECTION_NONE);
            /*if(IS_BTN_PRESSED())
            {
                //print("Disabling write protection and generating system reset...");
                Bootloader_ConfigProtection(BL_PROTECTION_NONE);
            }*/
        }
        //LED_R_OFF();
        //LED_Y_OFF();
       // print("Button was not pressed, write protection is still active.");
       // print("Exiting Bootloader.");
        return;
    }

    /* Initialize SD card */
    /*if(SD_Init())
    {
        // SD init failed 
       // print("SD card cannot be initialized.");
        return;
    }*/

    /* Mount SD card */
    fr = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
    if(fr != FR_OK)
    {
        /* f_mount failed */
        //print("SD card cannot be mounted.");
        //sprintf(msg, "FatFs error code: %u", fr);
       // print(msg);
        return;
    }
   // print("SD mounted.");
		/* open version file programming */
		/*fr = f_open(&SDFile, CONF_VERSION, FA_READ);
		if(fr != FR_OK)
		{
			 SD_Eject();
       return;
		}*/
		
    /* Open file for programming */
    fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
    if(fr != FR_OK)
    {
       /* f_open failed */
       SD_Eject();
       return;
    }
    //print("Software found on SD.");

    /* Check size of application found on SD card */
    if(Bootloader_CheckSize(f_size(&SDFile)) != BL_OK)
    {
        //print("Error: app on SD card is too large.");

        f_close(&SDFile);
        SD_Eject();
        //print("SD ejected.");
        return;
    }
    //print("App size OK.");

    /* Step 1: Init Bootloader and Flash */
    Bootloader_Init();

    /* Step 2: Erase Flash */
    //print("Erasing flash...");
    //LED_Y_ON();
    Bootloader_Erase();
    //LED_Y_OFF();
    //print("Flash erase finished.");

    /* If BTN is pressed, then skip programming */
    /*if(IS_BTN_PRESSED())
    {
        //print("Programming skipped.");

        f_close(&SDFile);
        SD_Eject();
        //print("SD ejected.");
        return;
    }*/

    /* Step 3: Programming */
    //print("Starting programming...");
    //LED_Y_ON();
    cntr = 0;
    Bootloader_FlashBegin();
    do
    {
        data =0xFFFFFFFFFFFFFF;
        fr   = f_read(&SDFile, &data, 8, &num);
        if(num)
        {
            status = Bootloader_FlashNext(data);
            if(status == BL_OK)
            {
                cntr++;
            }
            else
            {
                //sprintf(msg, "Programming error at: %lu byte", (cntr * 8));
               // print(msg);

                f_close(&SDFile);
                SD_Eject();
                //print("SD ejected.");

                //LED_G_OFF();
                //LED_Y_OFF();
                return;
            }
        }
        if(cntr % 256 == 0)
        {
            /* Toggle green LED during programming */
            //LED_G_TG();
        }
    } while((fr == FR_OK) && (num > 0));

    /* Step 4: Finalize Programming */
    Bootloader_FlashEnd();
    f_close(&SDFile);
    //LED_G_OFF();
    //LED_Y_OFF();
    //print("Programming finished.");
    /* Open file for verification */
    fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
    if(fr != FR_OK)
    {
        /* f_open failed */
        //print("File cannot be opened.");
        //sprintf(msg, "FatFs error code: %u", fr);
       // print(msg);

        SD_Eject();
        //print("SD ejected.");
        return;
    }

    /* Step 5: Verify Flash Content */
    addr = APP_ADDRESS;
    cntr = 0;
    do
    {
        data = 0xFFFFFFFFFFFFFFFF;
        fr   = f_read(&SDFile, &data, 4, &num);
        if(num)
        {
            if(*(uint32_t*)addr == (uint32_t)data)
            {
                addr += 4;
                cntr++;
            }
            else
            {
                //sprintf(msg, "Verification error at: %lu byte.", (cntr * 4));
                //print(msg);

                f_close(&SDFile);
                SD_Eject();
                //print("SD ejected.");

                //LED_G_OFF();
                return;
            }
        }
        if(cntr % 256 == 0)
        {
            /* Toggle green LED during verification */
            //LED_G_TG();
        }
    } while((fr == FR_OK) && (num > 0));
    //print("Verification passed.");
    //LED_G_OFF();

    /* Eject SD card */
    SD_Eject();
    //print("SD ejected.");

    /* Enable flash write protection */
#if(USE_WRITE_PROTECTION)
    //print("Enablig flash write protection and generating system reset...");
    if(Bootloader_ConfigProtection(BL_PROTECTION_WRP) != BL_OK)
    {
        //print("Failed to enable write protection.");
        //print("Exiting Bootloader.");
    }
#endif
}


void SD_DeInit(void)
{
			BSP_SD_DeInit();
			MX_FATFS_DEInit();
    //SDCARD_OFF();
}

void SD_Eject(void)
{
    f_mount(NULL, (TCHAR const*)SDPath, 0);
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
while(1)
{}
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
