/* USER CODE BEGIN Header */
/**
  * @file main.c
  * @author Grant Wilk
  * @created 2/11/2020
  * @modified 2/25/2020
  * @desc a digital camera application
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
# include <stdio.h>
# include "main.h"
# include "fatfs.h"
# include "uart_print.h"
# include "dcmi_driver.h"

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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef enum {
    DCM_INIT,
    DCM_IDLE,
    DCM_CAPTURE,
    DCM_SD_WRITE,
    DCM_ERR
} DCM_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_SPI2_Init(void);

static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    MX_FATFS_Init();
    /* USER CODE BEGIN 2 */

    uart_printf("\n\n==================================\n");
    uart_printf("---------- SYSTEM RESET ----------\n");
    uart_printf("==================================\n");

    // state variables
    DCM_state state = DCM_INIT;

    // FATFS variables
    FATFS file_system;    /* FAT file system object */
    FRESULT f_result;       /* FAT operation result */
    FIL file;           /* FAT file object */

    // DCMI variables
    uint8_t *img_buf;        /* DCMI image buffer */
    uint32_t img_buf_size;   /* DCMI image buffer size (bytes) */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        // initialization state
        if (state == DCM_INIT) {

            uart_printf("\n----------- INIT STATE -----------\n");

            // mount the SD card
            uart_printf("\nMounting SD Card...\n");
            f_result = f_mount(&file_system, "", 1);

            if (f_result == FR_OK) {
                uart_printf("Card Mounted!\n");
            } else {
                uart_printf("Failed To Mount! (%02X)\n", f_result);
                break;
            }

            // initialize the camera
            uart_printf("\nInitializing Camera...\n");
            dcmi_init();
            uart_printf("Camera Initialized!\n");

            // initialize blue button
            uart_printf("\nInitializing Blue Button...\n");
            GPIOC->MODER |= (GPIO_MODE_INPUT << GPIO_MODER_MODE13_Pos);
            GPIOC->PUPDR |= (GPIO_PULLDOWN << GPIO_PUPDR_PUPD13_Pos);
            uart_printf("Blue Button Initialized!\n");

            // delay 10ms for peripheral stabilization
            HAL_Delay(10);

            // always switch to IDLE state or error state
            uart_printf("\n----------- IDLE STATE -----------\n");
            state = DCM_IDLE;

        }

        // idle state
        else if (state == DCM_IDLE) {
            // if blue button pressed, switch to capture state
            if (!(GPIOC->IDR & GPIO_IDR_ID13)) {
                uart_printf("\n--------- CAPTURE STATE ----------\n");
                state = DCM_CAPTURE;
            }
        }

        // capture state
        else if (state == DCM_CAPTURE) {

            // capture an image
            uart_printf("\nCapturing Image...\n");
            dcmi_capture();

            // if capture successful, switch to SD write state, otherwise error state
            if (1) {
                uart_printf("Image Captured!\n");
                uart_printf("\n--------- SD WRITE STATE ---------\n");
                state = DCM_SD_WRITE;
            } else {
                uart_printf("Image Capture Failed!\n");
                uart_printf("\n---------- ERROR STATE -----------\n");
                state = DCM_ERR;
            }
        }

        // SD write state
        else if (state == DCM_SD_WRITE) {

            // generate the file name
            int image_num = 0;
            char file_name[12];

            do {
                sprintf(file_name, "DCM%04d.PGM", image_num++);
            } while (f_stat(file_name, NULL) != FR_NO_FILE);

            // open or create a new image file
            uart_printf("\nCreating \"%s\"...\n", file_name);
            f_result = f_open(&file, file_name, FA_WRITE | FA_CREATE_ALWAYS);

            if (f_result == FR_OK) {
                uart_printf("File Created!\n");
            } else {
                uart_printf("Failed to Create! (%02X)\n", f_result);
                break;
            }

            // get DCMI variables
            img_buf = dcmi_get_img_buf();
            img_buf_size = dcmi_get_img_buf_size();

            // write to image file
            uart_printf("\nWriting to \"%s\"...\n", file_name);

            int bytes_written;

            // file header
            f_printf(&file, "P5\n");
            f_printf(&file, "256 256\n");
            f_printf(&file, "255\n");

            f_result = f_write(&file, img_buf, img_buf_size, (UINT *) &bytes_written);

            if (f_result == FR_OK) {
                uart_printf("Wrote %d of %d bytes!\n", bytes_written, img_buf_size);
            } else {
                uart_printf("Write failed! Wrote %d of %d bytes! (%02X)\n", bytes_written, img_buf_size, f_result);
                break;
            }

            // close file
            uart_printf("\nClosing \"%s\"...\n", file_name);
            f_result = f_close(&file);
            if (f_result == FR_OK) {
                uart_printf("File Closed!\n", file_name);
            } else {
                uart_printf("File Failed to Close!\n", file_name);
            }

            // switch to idle state
            uart_printf("\n---------- IDLE STATE ----------\n");
            state = DCM_IDLE;

        }

        // break to error state
        else {
            state = DCM_ERR;
            break;
        }

        /* USER CODE BEGIN 3 */
    }

    // error state
    uart_printf("\n---------- ERROR STATE ---------\n");
    while (1);

    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    /** Initializes the CPU, AHB and APB busses clocks
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void) {

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    // configure CS pin (PB12) as output
    GPIOB->MODER |= (GPIO_MODE_OUTPUT_PP << GPIO_MODER_MODE12_Pos);

    // set CS pin high
    GPIOB->ODR |= GPIO_ODR_OD12;

    // set the MOSI line high
    BYTE txData = 0xFF;
    HAL_SPI_Transmit(&hspi2, &txData, 1, 1000);

    /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */
    uart_print_init();
    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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
