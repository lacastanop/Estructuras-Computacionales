# Taller: Implementación del Driver de Teclado Hexadecimal

## Introducción

En esta guía, implementaremos un driver para un teclado matricial de 4x4. A diferencia de un simple sondeo (polling) que consume ciclos de CPU constantemente, utilizaremos un método más eficiente y elegante: **interrupciones externas (EXTI)**.

Configuraremos las 4 filas del teclado como salidas y las 4 columnas como entradas con interrupciones. Cuando se presiona una tecla, se conecta una fila (que mantendremos en estado **BAJO**) con una columna (en estado **ALTO** gracias a una resistencia de pull-up). Esto provoca un flanco de bajada en el pin de la columna, lo que dispara una interrupción. Solo entonces nuestro código se activará para determinar exactamente qué tecla fue presionada.

Este enfoque es:

* **Eficiente**: El microcontrolador puede realizar otras tareas o entrar en modo de bajo consumo mientras espera una pulsación.
* **Reactivo**: La respuesta a la pulsación es casi inmediata.
* **Didáctico**: Es una excelente práctica para entender el manejo de interrupciones de hardware.

## 1. Configuración de Periféricos en STM32CubeMX

### 1.1 Configuración de Pines de Filas (Salidas)

Configura los siguientes pines como `GPIO_Output`:

* `PA10` → **User Label:** `KEYPAD_R1`
* `PB3`  → **User Label:** `KEYPAD_R2`
* `PB5`  → **User Label:** `KEYPAD_R3`
* `PB4`  → **User Label:** `KEYPAD_R4`

### 1.2 Configuración de Pines de Columnas (Entradas con EXTI)

Configura los siguientes pines como entradas con capacidad de interrupción externa:

* `PB10` → **User Label:** `KEYPAD_C1`
* `PA8`  → **User Label:** `KEYPAD_C2`
* `PA9`  → **User Label:** `KEYPAD_C3`
* `PC7`  → **User Label:** `KEYPAD_C4`

En `System Core > GPIO`, para cada pin de columna:

* **GPIO mode:** External Interrupt Mode with Falling edge trigger detection
* **GPIO Pull-up/Pull-down:** Pull-up

### 1.3 Habilitar Interrupciones (NVIC)

En `System Core > NVIC`, habilita:

* `EXTI line[9:5] interrupts` (PA8, PA9, PC7)
* `EXTI line[15:10] interrupts` (PB10)

## 2. Diseño y Creación de la Librería del Teclado

### keypad\_driver.h

```c
#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

#include "main.h"
#include <stdint.h>

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

typedef struct {
    GPIO_TypeDef* row_ports[KEYPAD_ROWS];
    uint16_t row_pins[KEYPAD_ROWS];
    GPIO_TypeDef* col_ports[KEYPAD_COLS];
    uint16_t col_pins[KEYPAD_COLS];
} keypad_handle_t;

void keypad_init(keypad_handle_t* keypad);
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin);

#endif // KEYPAD_DRIVER_H
```

### keypad\_driver.c - Plantilla

```c
#include "keypad_driver.h"

static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

void keypad_init(keypad_handle_t* keypad) {
    // TAREA: Implementar esta función.
}

char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
    char key_pressed = '\0';
    // TAREA: Implementar la lógica de escaneo
    return key_pressed;
}
```

## 3. Lógica Detallada del Escaneo

1. **Debounce:** `HAL_Delay(5);`
2. **Identificar columna:**

   * Itera `col_pins`, compara con `col_pin` para hallar `col_index`.
3. **Identificar fila:**

   * Pon todas las filas en ALTO
   * Itera `row_pins`, cada una la pones en BAJO
   * Lee el pin de columna
   * Si está en BAJO, has detectado la tecla
   * Guarda el caracter `keypad_map[row][col_index]`
   * Espera hasta que se suelte la tecla (vuelva a ser ALTA)
   * Sal del bucle
4. **Restaurar:** Llama a `keypad_init()` al final.
5. **Return:** Devuelve la tecla encontrada.

## 4. Integración en `main.c`

### Includes

```c
#include "keypad_driver.h"
#include "ring_buffer.h"
#include <stdio.h>
```

### Variables globales

```c
keypad_handle_t keypad = {
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port},
    .row_pins  = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port},
    .col_pins  = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}
};

#define KEYPAD_BUFFER_LEN 16
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN];
ring_buffer_t keypad_rb;
```

### Callback de interrupción

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    char key = keypad_scan(&keypad, GPIO_Pin);
    if (key != '\0') {
        ring_buffer_write(&keypad_rb, (uint8_t)key);
    }
}
```

### Dentro de `main()`

```c
ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN);
keypad_init(&keypad);
printf("Sistema listo. Esperando pulsaciones del teclado...\r\n");

while (1) {
    uint8_t key_from_buffer;
    if (ring_buffer_read(&keypad_rb, &key_from_buffer)) {
        printf("Tecla presionada: %c\r\n", (char)key_from_buffer);
    }
}
```

## 5. Ejercicio Final

Tu tarea es:

1. Completar `keypad_init()` y `keypad_scan()`
2. Integrar los archivos `.h` y `.c`
3. Modificar `main.c`
4. Compilar, cargar y probar en tu Nucleo
5. Implementar control de acceso:

   * Esperar 4 teclas
   * Verificar contra clave
   * Encender/parpadear LED según resultado

**Siguiente Paso:** [Uso de una libreria existente (SSD1306.md)](SSD1306.md)