# Implementación del Código de la Aplicación

Ahora que STM32CubeMX ha generado el código de inicialización para los periféricos, vamos a añadir la lógica de nuestra aplicación en el archivo `main.c`.

## 1. Ubicación del Código de Usuario

STM32CubeMX genera marcadores especiales en el código (`/* USER CODE BEGIN ... */` y `/* USER CODE END ... */`). **¡Siempre debes escribir tu código entre estos marcadores!** De lo contrario, se borrará la próxima vez que regeneres el código desde CubeMX.

Abre el archivo `Core/Src/main.c`.

## 2. Variables Globales y Includes

Necesitamos algunas variables para manejar los datos de UART y el estado del botón/LED. Añade lo siguiente en las secciones correspondientes:

```c
/* USER CODE BEGIN Includes */
#include <stdio.h> // Para usar sprintf
#include <string.h> // Para usar strlen
/* USER CODE END Includes */

// ... otras includes ...

/* USER CODE BEGIN PV */
// Variables para UART
uint8_t rx_data; // Buffer para recibir 1 byte por UART
char tx_buffer[50]; // Buffer para transmitir mensajes por UART

// Variables para el botón y LED externo
volatile uint8_t button_flag = 0; // Flag para indicar que el botón fue presionado (volatile porque se modifica en interrupción)
volatile uint32_t led_ext_off_time = 0; // Momento en que se debe apagar el LED externo

/* USER CODE END PV */

// ... resto del código autogenerado ...
```

## 3. Callbacks de Interrupción

Las interrupciones son eventos (como recibir un dato UART o pulsar un botón) que pausan temporalmente el programa principal para ejecutar una función específica llamada callback.

### 3.1 Callback de Recepción UART

Esta función se llama automáticamente por la HAL cada vez que se recibe un byte por USART2. La usaremos para leer el byte y habilitar la recepción del siguiente.

Añade esta función en la sección `/* USER CODE BEGIN 0 */` o `/* USER CODE BEGIN 4 */` (cualquiera de las dos sirve para funciones de usuario):

```c
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
```

### 3.2 Callback de Interrupción Externa (Botón)

Esta función se llama cuando se detecta un flanco descendente en el pin PC13 (Botón B1). Usaremos un simple debounce por software basado en tiempo para evitar múltiples detecciones por rebotes mecánicos.

Añade esta función también en la sección `/* USER CODE BEGIN 0 */` o `/* USER CODE BEGIN 4 */`:

```c
/* USER CODE BEGIN 0 */ // O 4, si ya pusiste el callback UART en 0

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
/* USER CODE END 0 */
```

**Nota**: `HAL_GetTick()` devuelve el número de milisegundos desde que se inició el microcontrolador. Es la base para nuestro manejo del tiempo.

## 4. Código Principal (main function)

El main contiene el bucle infinito (while(1)) donde se ejecuta la lógica principal de la aplicación.

### 4.1 Inicializaciones Adicionales

Dentro de main(), después de las llamadas MX_..._Init() pero antes del while(1), debemos habilitar la recepción UART por primera vez:

```c
/* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Habilitar la recepción UART por interrupción por primera vez
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  // Mensaje de bienvenida por UART
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
    if (HAL_GetTick() - last_heartbeat_time >= 500) // Cada 500 ms
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // Cambia el estado del LED LD2
      last_heartbeat_time = HAL_GetTick();
    }

    // ---- Procesamiento del Botón ----
    if (button_flag == 1)
    {
      // 1. Encender LED Externo
      HAL_GPIO_WritePin(LED_EXT_GPIO_Port, LED_EXT_Pin, GPIO_PIN_SET); // Enciende LED_EXT

      // 2. Calcular cuándo apagarlo (3 segundos desde ahora)
      led_ext_off_time = HAL_GetTick() + 3000;

      // 3. Enviar mensaje por UART
      sprintf(tx_buffer, "Boton B1 presionado! LED EXT ON.\r\n");
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
```

## 5. Compilar, Cargar y Depurar

- **Construir (Build)**: Usa la interfaz de la extensión STM32 en VS Code para construir el proyecto. Busca un icono de "martillo" o una opción en la paleta de comandos (Ctrl+Shift+P -> STM32: Build Project). Revisa la salida en la terminal de VS Code por errores.

- **Conectar la Placa**: Conecta tu Nucleo-L476RG al PC mediante el cable USB. Debería aparecer como un dispositivo ST-Link.

- **Cargar (Flash)**: Usa la opción de la extensión STM32 para cargar el firmware compilado en la placa (icono de "flecha hacia abajo" o STM32: Download Project to Target).

- **Depurar (Opcional pero recomendado)**: Inicia una sesión de depuración (icono de "play con bicho" o STM32: Start Debug Session). Esto te permite ejecutar el código paso a paso, ver variables, etc.

- **Conectar Monitor Serie**: Abre un monitor serie (como el integrado en VS Code, o herramientas como PuTTY, Tera Term, tio) conectado al puerto COM asociado con el ST-Link de tu placa Nucleo. Configúralo a 115200 baudios, 8N1.

En VS Code, puedes buscar Serial Monitor en las extensiones e instalar uno. Luego, conéctalo al puerto COM correcto.

## 6. Prueba

- Deberías ver el LED LD2 (verde) parpadeando cada medio segundo.

- En el monitor serie, deberías ver el mensaje "Sistema de Control Basico - Listo".

- Escribe algo en el monitor serie; deberías ver los caracteres que escribes aparecer de vuelta (eco).

- Presiona el botón B1 (azul). Deberías:
  - Ver el mensaje "Boton B1 presionado! LED EXT ON." en el monitor serie.
  - Ver el LED externo que conectaste encenderse.
  - Después de 3 segundos, el LED externo debería apagarse y deberías ver el mensaje "LED EXT OFF (timeout)." en el monitor serie.

## Entendiendo el Código (Brevemente)

- **HAL (Hardware Abstraction Layer)**: Usamos funciones HAL_... para interactuar con el hardware (GPIO, UART, Timers). Esto hace el código más portable pero oculta los detalles de bajo nivel (que veremos más adelante).

- **Interrupciones**: El botón y la recepción UART usan interrupciones. El hardware detecta el evento y llama a nuestra función Callback sin que tengamos que preguntar constantemente (polling) en el bucle principal.

- **volatile**: Le decimos al compilador que la variable button_flag puede cambiar inesperadamente (desde una interrupción), así que no debe optimizar su acceso.

- **Máquina de Estados Implícita**: Aunque simple, hay una pequeña máquina de estados para el LED externo: APAGADO -> (Botón presionado) -> ENCENDIDO_TEMPORAL -> (Timeout) -> APAGADO.

- **HAL_GetTick()**: Proporciona una base de tiempo en milisegundos para tareas como el parpadeo del LED, el debounce y el temporizador del LED externo.

- **Flujo del código:** 

![Code Diagram](assets/code_diagram.png)

## Experimentación

¡Ahora es tu turno! Intenta modificar el código:

- Cambia el tiempo que el LED externo permanece encendido.
- Haz que el LED LD2 parpadee más rápido o más lento.
- Envía un mensaje diferente por UART cuando se presiona el botón.
- Intenta controlar ambos LEDs (LD2 y el externo) con el botón.

¡Felicidades! Has completado tu primera práctica con STM32. Has configurado periféricos, manejado interrupciones y creado una aplicación básica interactiva. Guarda tus cambios usando Git (`git add .`, `git commit -m "Completada practica introductoria"`, `git push origin main`).

En las próximas sesiones, profundizaremos en cómo funciona todo esto "bajo el capó".
