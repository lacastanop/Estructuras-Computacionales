#include "room_control.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3; // Timer for fan control

// Default password
static const char DEFAULT_PASSWORD[] = "1234";

// Temperature thresholds for automatic fan control
static const float TEMP_THRESHOLD_LOW = 25.0f;
static const float TEMP_THRESHOLD_MED = 28.0f;  
static const float TEMP_THRESHOLD_HIGH = 31.0f;

// Timeouts in milliseconds
static const uint32_t INPUT_TIMEOUT_MS = 10000;  // 10 seconds
static const uint32_t ACCESS_DENIED_TIMEOUT_MS = 3000;  // 3 seconds

// Private function prototypes
// Funciones para gestionar la máquina de estados
static void room_control_change_state(room_control_t *room, room_state_t new_state);

// Funciones para actualizar los subsistemas
static void room_control_update_display(room_control_t *room);
//static void room_control_update_door(room_control_t *room);
static void room_control_update_fan(room_control_t *room);

// Funciones de utilidad
static fan_level_t room_control_calculate_fan_level(float temperature);
static void room_control_clear_input(room_control_t *room);


// --- Implementación de Funciones Públicas ---

/**
 * @brief Inicializa la estructura de control de la habitación y el hardware asociado.
 */
void room_control_init(room_control_t *room) {
    // Inicializa la estructura de datos del sistema con valores por defecto
    room->current_state = ROOM_STATE_LOCKED;
    strcpy(room->password, DEFAULT_PASSWORD);
    room_control_clear_input(room);
    room->last_input_time = 0;
    room->state_enter_time = HAL_GetTick();
    
    // Inicializa el estado lógico de la puerta
    room->door_locked = true;
    
    // Inicializa variables de ambiente
    room->current_temperature = 22.0f;  // Temperatura inicial por defecto
    room->current_fan_level = FAN_LEVEL_OFF;
    room->manual_fan_override = false;
    
    // Bandera para optimizar la actualización de la pantalla
    room->display_update_needed = true;
    
    // Inicializa el hardware
    //HAL_GPIO_WritePin(DOOR_STATUS_Pin, DOOR_STATUS_Port, GPIO_PIN_RESET); // LED de puerta apagado

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   
    room->current_fan_level = FAN_LEVEL_OFF;               // Inicia el PWM del ventilador
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);                       // Ventilador empieza apagado
}

/**
 * @brief Función principal de actualización, se llama en cada ciclo del bucle principal.
 *        Maneja las transiciones de estado basadas en el tiempo (timeouts).
 */
void room_control_update(room_control_t *room) {
    uint32_t current_time = HAL_GetTick();
    
    // Máquina de estados: lógica basada en el tiempo
    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
        case ROOM_STATE_UNLOCKED:
            // No hay acciones basadas en tiempo en estos estados. Se espera a eventos.
            break;
            
        case ROOM_STATE_INPUT_PASSWORD:
            // Si pasan 10 segundos desde la última tecla, el sistema se bloquea.
            if (current_time - room->last_input_time > INPUT_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;
            
        case ROOM_STATE_ACCESS_DENIED:
            // El mensaje de acceso denegado dura 3 segundos y luego se bloquea.
            if (current_time - room->state_enter_time > ACCESS_DENIED_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;
            
        case ROOM_STATE_EMERGENCY:
            // Lógica para un estado de emergencia (opcional, no implementado).
            break;
    }
    
    // Actualiza los subsistemas físicos en cada ciclo
    room_control_update_door(room);
    room_control_update_fan(room);

    // Optimización: solo se actualiza la pantalla si ha habido un cambio
    if (room->display_update_needed) {
        room_control_update_display(room);
        room->display_update_needed = false;
    }
}

/**
 * @brief Procesa una tecla presionada por el usuario.
 *        Maneja las transiciones de estado basadas en eventos de teclado.
 */
void room_control_process_key(room_control_t *room, char key) {
    room->last_input_time = HAL_GetTick(); // Reinicia el temporizador de inactividad
    
    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            if (key >= '0' && key <= '9') {
                room_control_change_state(room, ROOM_STATE_INPUT_PASSWORD);
                room->input_buffer[room->input_index++] = key;
            }
            break;
            
        case ROOM_STATE_INPUT_PASSWORD:
            if ((key >= '0' && key <= '9') && (room->input_index < PASSWORD_LENGTH)) {
                room->input_buffer[room->input_index++] = key;
            } else if (key == '#') {
                if (room->input_index == PASSWORD_LENGTH) {
                    room->input_buffer[room->input_index] = '\0';
                    if (strcmp(room->input_buffer, room->password) == 0) {
                        room_control_change_state(room, ROOM_STATE_UNLOCKED);
                    } else {
                        room_control_change_state(room, ROOM_STATE_ACCESS_DENIED);
                    }
                }
            } else if (key == '*') {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;
            
        case ROOM_STATE_UNLOCKED:
            if (key == '*') {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;
            
        default:
            break;
    }
    
    room->display_update_needed = true; // Marca la pantalla para que se refresque.
}

/**
 * @brief Actualiza la temperatura del sistema y ajusta el ventilador si está en modo automático.
 */
void room_control_set_temperature(room_control_t *room, float temperature) {
    if (fabs(room->current_temperature - temperature) > 0.1f) {
        room->current_temperature = temperature;
        room->display_update_needed = true;
    }

    if (!room->manual_fan_override) {
        fan_level_t new_level = room_control_calculate_fan_level(temperature);
        if (new_level != room->current_fan_level) {
            room->current_fan_level = new_level;
            room->display_update_needed = true; // Refrescar si el nivel del fan cambia
        }
    }
}

/**
 * @brief Pone el ventilador en un nivel específico, anulando el control automático.
 */
void room_control_force_fan_level(room_control_t *room, fan_level_t level) {
    room->manual_fan_override = true;
    if (room->current_fan_level != level) {
        room->current_fan_level = level;
        room->display_update_needed = true;
    }
}

/**
 * @brief Cambia la contraseña del sistema.
 */
void room_control_change_password(room_control_t *room, const char *new_password) {
    if (strlen(new_password) == PASSWORD_LENGTH) {
        strcpy(room->password, new_password);
    }
}


// --- Implementación de Funciones de Estado (Getters) ---

room_state_t room_control_get_state(room_control_t *room) { return room->current_state; }
bool room_control_is_door_locked(room_control_t *room) { return room->door_locked; }
fan_level_t room_control_get_fan_level(room_control_t *room) { return room->current_fan_level; }
float room_control_get_temperature(room_control_t *room) { return room->current_temperature; }


// --- Implementación de Funciones Privadas (Helpers) ---

/**
 * @brief Cambia el estado del sistema y ejecuta acciones de entrada al nuevo estado.
 */
static void room_control_change_state(room_control_t *room, room_state_t new_state) {
    if (room->current_state == new_state) return;

    room->current_state = new_state;
    room->state_enter_time = HAL_GetTick();
    room->display_update_needed = true;
    
    switch (new_state) {
        case ROOM_STATE_LOCKED:
            room->door_locked = true;
            room->manual_fan_override = false; // El control automático se restaura al bloquear
            room_control_clear_input(room);
            break;
        case ROOM_STATE_INPUT_PASSWORD:
            room->last_input_time = HAL_GetTick();
            room_control_clear_input(room);
            break;
        case ROOM_STATE_UNLOCKED:
            room->door_locked = false;
            break;
        case ROOM_STATE_ACCESS_DENIED:
            break;
        default:
            break;
    }
}

/**
 * @brief Actualiza la pantalla OLED según el estado actual del sistema.
 */
static void room_control_update_display(room_control_t *room) {
    char display_buffer[32];
    
    ssd1306_Fill(Black);
    
    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            ssd1306_SetCursor(5, 25);
            ssd1306_WriteString("SISTEMA BLOQUEADO", Font_7x10, White);
            break;
            
        case ROOM_STATE_INPUT_PASSWORD:
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("INGRESE CLAVE:", Font_7x10, White);
            memset(display_buffer, '*', room->input_index);
            display_buffer[room->input_index] = '\0';
            ssd1306_SetCursor(45, 30);
            ssd1306_WriteString(display_buffer, Font_7x10, White);
            break;
            
        case ROOM_STATE_UNLOCKED:
            ssd1306_SetCursor(5, 5);
            ssd1306_WriteString("ACCESO CONCEDIDO", Font_7x10, White);
            snprintf(display_buffer, sizeof(display_buffer), "Temp: %.1fC", room->current_temperature);
            ssd1306_SetCursor(5, 20);
            ssd1306_WriteString(display_buffer, Font_7x10, White);
            snprintf(display_buffer, sizeof(display_buffer), "Fan: %d%%", room->current_fan_level);
            ssd1306_SetCursor(5, 35);
            ssd1306_WriteString(display_buffer, Font_7x10, White);
            break;
            
        case ROOM_STATE_ACCESS_DENIED:
            ssd1306_SetCursor(10, 25);
            ssd1306_WriteString("ACCESO DENEGADO", Font_7x10, White);
            break;
            
        default:
            break;
    }
    
    ssd1306_UpdateScreen();
}

/**
 * @brief Actualiza el estado físico de la puerta (el LED de estado).
 
static void room_control_update_door(room_control_t *room) {
    HAL_GPIO_WritePin(DOOR_STATUS_Port, DOOR_STATUS_Pin, room->door_locked ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
*/
/**
 * @brief Actualiza la velocidad del ventilador ajustando el ciclo de trabajo del PWM.
 */
static void room_control_update_fan(room_control_t *room) {
    uint32_t pwm_value = room->current_fan_level;
    if (pwm_value > 99) pwm_value = 99; // Limita el PWM al máximo del contador (Periodo - 1)
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
}

/**
 * @brief Calcula el nivel de ventilador requerido basado en la temperatura.
 */
static fan_level_t room_control_calculate_fan_level(float temperature) {
    if (temperature < TEMP_THRESHOLD_LOW) return FAN_LEVEL_OFF;
    if (temperature < TEMP_THRESHOLD_MED) return FAN_LEVEL_LOW;
    if (temperature < TEMP_THRESHOLD_HIGH) return FAN_LEVEL_MED;
    return FAN_LEVEL_HIGH;
}

/**
 * @brief Limpia el buffer de entrada de la contraseña y resetea su índice.
 */
static void room_control_clear_input(room_control_t *room) {
    memset(room->input_buffer, 0, sizeof(room->input_buffer));
    room->input_index = 0;
}