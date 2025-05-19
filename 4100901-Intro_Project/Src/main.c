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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // Para usar sprintf
#include <string.h> // Para usar strlen
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

/* USER CODE BEGIN PV */
// Variables para UART
uint8_t rx_data; // Buffer para recibir 1 byte por UART
char tx_buffer[50]; // Buffer para transmitir mensajes por UART

// Variables para el botón y LED externo
volatile uint8_t button_flag = 0; // Flag para indicar que el botón fue presionado (volatile porque se modifica en interrupción)
volatile uint32_t led_ext_off_time = 0; // Momento en que se debe apagar el LED externo
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Callback que se ejecuta cuando se completa la recepción de 1 byte por UART2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // Hacemos eco del caracter recibido
    HAL_UART_Transmit(&huart2, &rx_data, 1, 10); // Enviar el byte recibido de vuelta

    // Volvemos a habilitar la interrupción de recepción UART para el próximo byte
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
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
  /* USER CODE BEGIN 2 */
  // Habilitar la recepción UART por interrupción por primera vez
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  // Mensaje de bienvenida por UART-------------------------------------------------ACA INICIA
  char *welcome_msg = "Sistema de Control Basico - Listo\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)welcome_msg, strlen(welcome_msg), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // ---- Heartbeat LED ----
    static uint32_t last_heartbeat_time = 0;
    static uint32_t tiempo = 1000;
    static uint32_t estado = 0; 


    if (HAL_GetTick() - last_heartbeat_time >= tiempo) // Cada 500 ms  //Realice algo cada cierto tiempo
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Cambia el estado del LED LD2   //Cambiar el estado de un pin
      last_heartbeat_time = HAL_GetTick();
    }

    // ---- Procesamiento del Botón ----
    if (button_flag == 1)
    {
      if (estado==0){
        estado=1;
        tiempo=50;
      }
      else{
        estado=0;
        tiempo=1000;
      }
      
      

      // 1. Encender LED Externo
      HAL_GPIO_WritePin(LED_EXT_GPIO_Port, LED_EXT_Pin, GPIO_PIN_SET); // Enciende LED_EXT

      // 2. Calcular cuándo apagarlo (5 segundos desde ahora)
      led_ext_off_time = HAL_GetTick() + 5000;

      // 3. Enviar mensaje por UART
      sprintf(tx_buffer, "LED EXT ON, LED INT PARPADEA MAS RAPIDO.\r\n");  //Asigna nuevo valor al buffer
      HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

      // 4. Limpiar el flag
      button_flag = 0;
    }

    // ---- Control del Apagado del LED Externo ----
    // Si el tiempo de apagado es distinto de 0 (significa que está programado)
    // y ya hemos alcanzado o superado ese tiempo...
    if (led_ext_off_time != 0 && HAL_GetTick() >= led_ext_off_time)
    {
      // 1. Apagar el LED
      HAL_GPIO_WritePin(LED_EXT_GPIO_Port, LED_EXT_Pin, GPIO_PIN_RESET); // Apaga LED_EXT

      // 2. Enviar mensaje por UART (opcional)
      sprintf(tx_buffer, "LED EXT OFF (timeout).\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);

      // 3. Resetear el tiempo de apagado a 0 (para no volver a apagarlo)
      led_ext_off_time = 0;
    }

  } // Fin del while(1)
  
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  /** Initializes the CPU, AHB and APB buses clocks
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
// Callback que se ejecuta cuando ocurre una interrupción externa (Botón B1)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_press_time = 0; // Guarda el tick de la última pulsación válida
  uint32_t current_time = HAL_GetTick(); // Obtiene el tiempo actual en ms

  if (GPIO_Pin == B1_Pin) // Nos aseguramos que la interrupción es del B1
  {
    // Debounce sencillo: ignorar si han pasado menos de 200ms desde la última vez
    if (current_time - last_press_time > 200)
    {
      button_flag = 1; // Activamos el flag para que el bucle principal lo procese
      last_press_time = current_time; // Actualizamos el tiempo de la última pulsación válida
    }
  }
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
