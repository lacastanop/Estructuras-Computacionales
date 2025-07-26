#include "keypad_driver.h"
#include "ring_buffer.h"

// Mapa de caracteres del teclado.
static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

/**
 * @brief Inicializa el teclado poniendo todas las filas en estado BAJO.
 * @param keypad: Handle del teclado.
 * @note Esto prepara al sistema para que cualquier pulsación de tecla
 *       pueda generar una interrupción en la columna correspondiente.
 */
void keypad_init(keypad_handle_t* keypad) {
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
    }
}

/**
 * @brief Escanea el teclado para determinar qué tecla fue presionada.
 * @param keypad: Handle del teclado.
 * @param col_pin: El pin de la columna que generó la interrupción.
 * @return El caracter de la tecla presionada o '\0' si no se detecta ninguna.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
    char key_pressed = '\0';
    int col_index = -1;

    // Pequeño retardo para eliminar el rebote (debounce) del pulsador.
    HAL_Delay(20);

    // 1. Identificar el índice de la columna que causó la interrupción.
    for (int i = 0; i < KEYPAD_COLS; i++) {
        if (keypad->col_pins[i] == col_pin) {
            col_index = i;
            break;
        }
    }

    if (col_index == -1) {
        return '\0'; // No debería ocurrir si la configuración es correcta.
    }

    // 2. Poner todas las filas en ALTO.
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }

    // 3. Poner cada fila en BAJO, una por una, y leer la columna para encontrar la fila activa.
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        // Poner la fila actual en BAJO.
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);

        // Leer el estado del pin de la columna que generó la interrupción.
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], col_pin) == GPIO_PIN_RESET) {
            // ¡Tecla encontrada!
            key_pressed = keypad_map[i][col_index];

            // Esperar a que la tecla se suelte (el pin de columna vuelve a ALTO).
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], col_pin) == GPIO_PIN_RESET);
            
            break; // Salir del bucle una vez que se encuentra la tecla.
        }

        // Restaurar la fila actual a ALTO antes de probar la siguiente.
         HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }

    // 4. Restaurar el estado inicial del teclado para la próxima interrupción.
    keypad_init(keypad);

    return key_pressed;
}
