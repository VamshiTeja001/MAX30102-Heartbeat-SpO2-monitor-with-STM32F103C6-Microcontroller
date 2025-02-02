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
#include "cmsis_os.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

//osThreadId defaultTaskHandle;

osThreadId configurationTaskHandle;
osThreadId heartBeat_Spo2TaskHandle;
osThreadId sensorStandbyMonitorTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
max30102_t max30102;
osMutexId i2cMutex;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  osThreadDef(configurationTask,MAX30102_Configuration, osPriorityNormal, 0, 128);
  osThreadDef(heartBeat_Spo2Task, MAX30102_Heartbeat_SpO2,osPriorityNormal, 0, 128 );
  configurationTaskHandle = osThreadCreate(osThread(configurationTask), NULL);
  if(configurationTaskHandle != NULL) osStatus osThreadTerminate(osThreadid configurationtaskHandle);
  heartBeat_Spo2TaskHandle= osThreadCreate(osThread(heartBeat_Spo2Task), NULL);

  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}


void MAX30102_Configuration(void const * argument){
	osMutexWait(i2cMutex, osWaitForever);
	char message[64];
	snprintf(message, sizeof(message), "Setting UP MAX30102 Sensor....\n");

	max30102_set_ppg_rdy(max30102,1); //Enable PPG Ready Flag
	osDelay(1);
	max30102_set_mode(max30102,max30102_multi_led); osDelay(1);// Mode set to Multi-LED, for both Spo2 and Heartbeat calculation
	max30102_set_sampling_rate(max30102,max30102_sr_800); osDelay(1); //800 samples per second
	max30102_set_led_pulse_width(max30102,max30102_pw_18_bit); osDelay(1);//
	//max30102_set_adc_resolution(max30102,1);
	max30102_set_led_current_1(max30102,12.6); osDelay(1);
	max30102_set_led_current_2(max30102,12.6); osDelay(1);
	max30102_set_multi_led_slot_1_2(max30102,max30102_led_ir,max30102_led_ir); osDelay(1);
	max30102_set_multi_led_slot_3_4(max30102,max30102_led_red, max30102_led_red); osDelay(1);

	snprintf(message, sizeof(message), "Configuration Complete!!\n");
	osMutexRelease(i2cMutex);
}




////////////////////Process Heartbeat and Spo2



void MAX30102_Heartbeat_SpO2(void *argument) {
    while (1) {  // Infinite loop to keep the thread alive
        osMutexWait(i2cMutex, osWaitForever);

        // Check if the interrupt pin is low
        int ITRPT = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        if (!ITRPT) {
            // Read data from the MAX30102 FIFO
            max30102_read_fifo(max30102);

            // Access IR and RED samples
            uint32_t *IR_samples = max30102->_ir_samples;
            uint32_t *RED_samples = max30102->_red_samples;

            // Calculate heart rate
            float bpm = calculate_heartbeat(IR_samples, 32, 100); // Assuming 100 Hz sampling rate

            // Print heart rate
            char message[64];
            snprintf(message, sizeof(message), "Heart Rate: %.2f BPM\n", bpm);
            HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

            // Calculate SpO2
            float spo2 = calculate_spo2(IR_samples, RED_samples, 32);

            // Print SpO2
            snprintf(message, sizeof(message), "SpO2: %.2f%%\n", spo2);
            HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
        }

        // Release the mutex to allow other threads to access I2C
        osMutexRelease(i2cMutex);

        // Add a delay to prevent hogging the CPU
        osDelay(100);  // 100 ms delay
    }
}


		// Function to calculate heart rate (BPM)
		float calculate_heartbeat(uint32_t *ir_samples, int num_samples, int sampling_rate) {
		    int peak_count = 0;
		    float total_interval = 0;

		    for (int i = 1; i < num_samples - 1; i++) {
		        if (ir_samples[i] > ir_samples[i - 1] && ir_samples[i] > ir_samples[i + 1] &&
		            ir_samples[i] > 1000) { // Threshold to detect valid peaks
		            peak_count++;
		        }
		    }

		    if (peak_count > 1) {
		        float time_between_peaks = (float)(num_samples) / (float)sampling_rate;
		        float bpm = (peak_count / time_between_peaks) * 60;
		        return bpm;
		    }
		    return 0.0; // Return 0 if no valid peaks detected
		}

		// Function to calculate SpO2
		float calculate_spo2(uint32_t *ir_samples, uint32_t *red_samples, int num_samples) {
		    float red_dc = 0, ir_dc = 0;
		    float red_ac = 0, ir_ac = 0;

		    // Calculate DC and AC components
		    for (int i = 0; i < num_samples; i++) {
		        red_dc += red_samples[i];
		        ir_dc += ir_samples[i];
		    }
		    red_dc /= num_samples;
		    ir_dc /= num_samples;

		    for (int i = 0; i < num_samples; i++) {
		        red_ac += fabs(red_samples[i] - red_dc);
		        ir_ac += fabs(ir_samples[i] - ir_dc);
		    }
		    red_ac /= num_samples;
		    ir_ac /= num_samples;

		    // Calculate SpO2 ratio
		    float ratio = (red_ac / red_dc) / (ir_ac / ir_dc);
		    float spo2 = 110 - (25 * ratio); // Typical calibration equation
		    return (spo2 > 100.0) ? 100.0 : ((spo2 < 0.0) ? 0.0 : spo2);
		}



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
