#include "main.h"
#include "arm_math.h"
#include "stdio.h"
#include <stdbool.h>
#include "string.h"

#define NUM_TAPS 16
float32_t low_cutoff = 0.0f;    //(HZ)  Initial lower cutoff frequency
float32_t LOW_CUTOFF;
float32_t high_cutoff = 5000.0f;	//HZ nitial higher cutoff frequency
float32_t HIGH_CUTOFF;
int VOLUME = 1;
#define BLOCK_SIZE 1

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
int16_t toReceive;
int16_t toWrite;
arm_fir_instance_f32 S;
float32_t firCoeffs[NUM_TAPS]; // Filter coefficients
float32_t firState[NUM_TAPS + BLOCK_SIZE - 1]; // Filter state
float32_t filter_input;    // filter input
float32_t filter_output;   // filter output
float32_t phase_change = 100.0f;
bool active_mode_select = false;
bool setting_mode_select = false;
bool mode_button_pressed = false;
bool setting_button_pressed = false;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool check_s1_pressed()
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

    bool pressed = false;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
    {
        if (current_time - last_press_time > 200)
        {
            pressed = true;
            last_press_time = current_time;
        }
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

    return pressed;
}

bool check_specific_button_pressed(int button_id)
{
    int row, col;

    // Map button_id to row and column
    map_button_to_row_col(button_id, &row, &col);

    // Activate the specific row (R1~R4 -> PC6~PC9)
    HAL_GPIO_WritePin(GPIOC, 1 << (row + 6), GPIO_PIN_SET); // Set the corresponding row high

    // Check if the corresponding column (C1~C4 -> PA8~PA11) is high
    bool pressed = HAL_GPIO_ReadPin(GPIOA, 1 << (col + 8)) == GPIO_PIN_SET;

    // Reset the row to low
    HAL_GPIO_WritePin(GPIOC, 1 << (row + 6), GPIO_PIN_RESET);

    return pressed;
}

// Function to map a button ID to row and column
void map_button_to_row_col(int button_id, int *row, int *col)
{
    *row = (button_id - 1) / 4; // Calculate row (0 to 3)
    *col = (button_id - 1) % 4; // Calculate column (0 to 3)
}

void bandpass_fir(float32_t* coeffs, int num_taps, float low_cutoff, float high_cutoff)
{
    int M = num_taps - 1;
    for (int n = 0; n < num_taps; n++)
    {
        if (n == M / 2)
        {
            coeffs[n] = 2 * (high_cutoff - low_cutoff);
        }
        else
        {
            coeffs[n] = (sin(2 * PI * high_cutoff * (n - M / 2)) - sin(2 * PI * low_cutoff * (n - M / 2)))
                        / (PI * (n - M / 2));
            coeffs[n] *= (0.54 - 0.46 * cos(2 * PI * n / M)); 	// 使用 Hamming  ?
        }
    }
}

void lowpass_fir(float32_t* coeffs, int num_taps, float cutoff)
{
    int M = num_taps - 1;
    for (int n = 0; n < num_taps; n++)
    {
        if (n == M / 2)
        {
            coeffs[n] = 2 * cutoff;
        }
        else
        {
            coeffs[n] = sin(2 * PI * cutoff * (n - M / 2)) / (PI * (n - M / 2));
            coeffs[n] *= (0.54 - 0.46 * cos(2 * PI * n / M));  // 使用 Hamming  ?
        }
    }
}

void init_filter()
{
	LOW_CUTOFF = low_cutoff / 16000.0f;
	HIGH_CUTOFF = high_cutoff / 16000.0f;							//If using lowpass filter
    //bandpass_fir(firCoeffs, NUM_TAPS, LOW_CUTOFF, HIGH_CUTOFF);   //If using bandpass filter
	lowpass_fir(firCoeffs, NUM_TAPS, HIGH_CUTOFF);
	arm_fir_init_f32(&S, NUM_TAPS, firCoeffs, firState, BLOCK_SIZE);
}

void adaptive_signal_process(float input_signal[], float output_signal[], int length) {
    float error_signal;      
    float current_output = 0; 
    
    for (int i = 0; i < length; i++) {
        error_signal = input_signal[i] + current_output;  

        // 輸出下一個信號 (反轉並調整)
        if (fabs(input_signal[i]) > THRESHOLD) {
            current_output = -error_signal * ALPHA;  
        } else {
            current_output = current_output;  
        }
        
        output_signal[i] = current_output;  
    }
}

char buffer[16];
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    //Check whether press blue button
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) // If pressed
    {
    	if (!mode_button_pressed) // If mode_button_pressed = false
    	        {
					active_mode_select = !active_mode_select; // Mode change
					if (active_mode_select){
						high_cutoff = phase_change;
					}else{
						high_cutoff = 10000.0f;
					}
					init_filter();
					mode_button_pressed = true;      // Button pressed
    	        }
    }else{
    	mode_button_pressed= false;
    }

    //Check S1 button
    if (check_s1_pressed()){
		if (!setting_button_pressed) // Debouncing
		{
			setting_mode_select = !setting_mode_select; // Switch Setting Mode
			setting_button_pressed = true;
		}
	}else{
		setting_button_pressed = false;
	}

    //Set mode or work mode(monitor or active)
    if (setting_mode_select==false){
    	//Apply DSP Filter
    	filter_input = (float32_t)toReceive; // Transfer to float type
    	arm_fir_f32(&S, &filter_input, &filter_output, BLOCK_SIZE);
		//filter_output = toReceive;    // IF no filter apply

    	// Mode select
		if (active_mode_select)
		{
			//toWrite = toReceive * (-1);
			toWrite = (int16_t)(filter_output)* -(VOLUME);    //after bandpass filter
			HAL_I2S_Transmit_DMA(&hi2s2, &toWrite, 1);
		}
    }else{
    	filter_output = 0;
    	// Adjust parameter
		static uint32_t last_adjust_time = 0;
		uint32_t current_time = HAL_GetTick();
		if (current_time - last_adjust_time > 200){    //Adjust every 200 ms
			//Adjusting volume
			if (check_specific_button_pressed(9)){   //Increase volume
				VOLUME++;}
			else if (check_specific_button_pressed(13)){  //Decrease volume
				VOLUME--;}
			//Adjusting frequency band
			else if (check_specific_button_pressed(11)){  // Increase lower cutoff frequency
				low_cutoff += 100.0f;}
			else if (check_specific_button_pressed(15)){  // Decrease lower cutoff frequency
				low_cutoff -= 100.0f;}
			else if (check_specific_button_pressed(12)){  // Increase higher cutoff frequency
				high_cutoff += 100.0f;}
			else if (check_specific_button_pressed(16)){  // Decrease higher cutoff frequency
				high_cutoff -= 100.0f;}
			init_filter();
			last_adjust_time = current_time; // Update last adjustment time
		}
    }
    HAL_I2S_Receive_DMA(hi2s, &toReceive, 1);
}

char str[120];
int main(void)
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  init_filter();
  /* USER CODE BEGIN 2 */
  HAL_I2S_Receive_DMA(&hi2s3, &toReceive, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //The code below can plot the signal in serial port plotter
	  sprintf(str, "$%d %d;", toWrite, (int16_t)filter_output);    //The signal transmitted to the speaker and the signal pass through the filter
	  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
	  HAL_Delay(1);
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  huart2.Init.BaudRate = 115200;//115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
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
