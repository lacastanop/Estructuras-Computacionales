#include "room_control.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // Para atoi

// --- VARIABLES PRIVADAS DEL MÓDULO ---
static const char DEFAULT_PASSWORD[] = "1234";
static TIM_HandleTypeDef *g_fan_timer_htim; // Puntero al timer del ventilador

// --- PROTOTIPOS DE FUNCIONES PRIVADAS ---
static void room_control_change_state(room_control_t *room, room_state_t new_state);
static void room_control_update_display(room_control_t *room);
static void room_control_update_door_led(room_control_t *room);
static void room_control_update_fan_pwm(room_control_t *room);
static fan_level_t room_control_calculate_fan_level(float temperature);
static void room_control_clear_input(room_control_t *room);
static void room_control_send_alert(const char* message);

// --- IMPLEMENTACIÓN ---

void room_control_init(room_control_t *room, TIM_HandleTypeDef *fan_timer) {
    g_fan_timer_htim = fan_timer;

    strcpy(room->password, DEFAULT_PASSWORD);
    room->current_temperature = 22.0f;
    room->manual_fan_override = false;
    room->current_fan_level = FAN_LEVEL_OFF;

    // Iniciar el sistema en estado bloqueado
    room_control_change_state(room, ROOM_STATE_LOCKED);

    // Inicializar hardware
    HAL_TIM_PWM_Start(g_fan_timer_htim, TIM_CHANNEL_1);
    room_control_update_door_led(room);
    room_control_update_fan_pwm(room);

    // Forzar actualización inicial de la pantalla
    room->display_update_needed = true;
    room_control_update_display(room);
}

void room_control_update(room_control_t *room) {
    uint32_t current_time = HAL_GetTick();

    // Lógica de la máquina de estados basada en timeouts
    switch (room->current_state) {
        case ROOM_STATE_AWAITING_PASSWORD:
            if (current_time - room->last_activity_time > INPUT_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_UNLOCKED:
            if (current_time - room->state_enter_time > UNLOCKED_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_ACCESS_DENIED:
            if (current_time - room->state_enter_time > ACCESS_DENIED_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_LOCKED:
        default:
            // No hay timeouts en estos estados
            break;
    }

    // Actualizar subsistemas (solo si es necesario para evitar trabajo extra)
    // El control de la puerta y el ventilador ya se actualizan al cambiar de estado/temperatura.
    // La pantalla se actualiza si el flag está activo.
    if (room->display_update_needed) {
        room_control_update_display(room);
        room->display_update_needed = false;
    }
}

void room_control_process_key(room_control_t *room, char key) {
    room->last_activity_time = HAL_GetTick();

    if (key == '*') {
        room_control_change_state(room, ROOM_STATE_LOCKED);
        return;
    }

    if (room->current_state == ROOM_STATE_LOCKED) {
        room_control_change_state(room, ROOM_STATE_AWAITING_PASSWORD);
    }

    if (room->current_state == ROOM_STATE_AWAITING_PASSWORD && room->input_index < PASSWORD_LENGTH) {
        room->input_buffer[room->input_index++] = key;
        room->input_buffer[room->input_index] = '\0';

        if (room->input_index == PASSWORD_LENGTH) {
            if (strcmp(room->input_buffer, room->password) == 0) {
                room_control_change_state(room, ROOM_STATE_UNLOCKED);
            } else {
                room_control_change_state(room, ROOM_STATE_ACCESS_DENIED);
                room_control_send_alert("ALERT:WRONG_PASS_ATTEMPT\n");
            }
        }
        room->display_update_needed = true;
    }
}

void room_control_set_temperature(room_control_t *room, float temperature) {
    room->current_temperature = temperature;

    if (!room->manual_fan_override) {
        fan_level_t new_level = room_control_calculate_fan_level(temperature);
        if (new_level != room->current_fan_level) {
            room->current_fan_level = new_level;
            room_control_update_fan_pwm(room);
            room->display_update_needed = true;
        }
    }
}

// --- NUEVA FUNCIÓN PARA PROCESAR COMANDOS ---
void room_control_process_command(room_control_t *room, char *command_line) {
    char response[64];
    
    // Eliminar '\n' o '\r'
    command_line[strcspn(command_line, "\r\n")] = 0;

    if (strcmp(command_line, "GET_TEMP") == 0) {
        snprintf(response, sizeof(response), "TEMP:%.1f\n", room_control_get_temperature(room));
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    }
    else if (strcmp(command_line, "GET_STATUS") == 0) {
        snprintf(response, sizeof(response), "STATUS:%s,FAN:%d\n", room_control_get_state_string(room), room->current_fan_level);
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    }
    else if (strncmp(command_line, "SET_PASS:", 9) == 0) {
        char *new_pass = command_line + 9;
        if (strlen(new_pass) == PASSWORD_LENGTH) {
            strcpy(room->password, new_pass);
            snprintf(response, sizeof(response), "ACK:PASS_SET\n");
        } else {
            snprintf(response, sizeof(response), "NACK:INVALID_PASS\n");
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    }
    else if (strncmp(command_line, "FORCE_FAN:", 10) == 0) {
        int level = atoi(command_line + 10);
        if (level >= 0 && level <= 3) {
            room->manual_fan_override = true;
            room->current_fan_level = level * 30 + (level > 1 ? 10 : 0) + (level > 2 ? 30 : 0); // Simplificado
            if(level == 0) room->current_fan_level = FAN_LEVEL_OFF;
            if(level == 1) room->current_fan_level = FAN_LEVEL_LOW;
            if(level == 2) room->current_fan_level = FAN_LEVEL_MED;
            if(level == 3) room->current_fan_level = FAN_LEVEL_HIGH;
            room_control_update_fan_pwm(room);
            snprintf(response, sizeof(response), "ACK:FAN_FORCED\n");
        } else {
            room->manual_fan_override = false; // "A" o valor invalido vuelve a auto
            snprintf(response, sizeof(response), "ACK:FAN_AUTO\n");
        }
        room->display_update_needed = true;
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    }
    else {
        snprintf(response, sizeof(response), "NACK:UNKNOWN_CMD\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    }
}

// --- FUNCIONES PRIVADAS ---

static void room_control_change_state(room_control_t *room, room_state_t new_state) {
    if (room->current_state == new_state) return;

    room->current_state = new_state;
    room->state_enter_time = HAL_GetTick();
    room->last_activity_time = HAL_GetTick();
    room->display_update_needed = true;

    // Acciones de entrada al estado
    switch (new_state) {
        case ROOM_STATE_LOCKED:
            room_control_clear_input(room);
            room->door_locked = true;
            break;
        case ROOM_STATE_UNLOCKED:
            room->door_locked = false;
            break;
        case ROOM_STATE_AWAITING_PASSWORD:
            room_control_clear_input(room);
            break;
        case ROOM_STATE_ACCESS_DENIED:
            room_control_clear_input(room);
            break;
    }
    room_control_update_door_led(room);
}

static void room_control_update_display(room_control_t *room) {
    char line1[20];
    char line2[20];
    char line3[20];

    ssd1306_Fill(Black);

    // Líneas 1 y 2: Estado y Clave/Info
    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            strcpy(line1, "SISTEMA");
            strcpy(line2, "BLOQUEADO");
            break;
        case ROOM_STATE_AWAITING_PASSWORD:
            strcpy(line1, "CLAVE:");
            snprintf(line2, sizeof(line2), "%s", room_control_get_input_buffer_masked(room));
            break;
        case ROOM_STATE_UNLOCKED:
            strcpy(line1, "ACCESO");
            strcpy(line2, "CONCEDIDO");
            break;
        case ROOM_STATE_ACCESS_DENIED:
            strcpy(line1, "ACCESO");
            strcpy(line2, "DENEGADO");
            break;
    }
    ssd1306_SetCursor(17, 0);
    ssd1306_WriteString(line1, Font_11x18, White);
    ssd1306_SetCursor(17, 20);
    ssd1306_WriteString(line2, Font_11x18, White);

    // Línea 3: Temperatura y Ventilador
    uint8_t fan_percent;
    if (room->current_fan_level == FAN_LEVEL_OFF) fan_percent = 0;
    else if (room->current_fan_level == FAN_LEVEL_LOW) fan_percent = 30;
    else if (room->current_fan_level == FAN_LEVEL_MED) fan_percent = 70;
    else fan_percent = 100;

    snprintf(line3, sizeof(line3), "T:%.1fC Fan:%d%%", room->current_temperature, fan_percent);
    ssd1306_SetCursor(0, 50);
    ssd1306_WriteString(line3, Font_7x10, White);

    ssd1306_UpdateScreen();
}

static void room_control_update_door_led(room_control_t *room) {
    if (room->door_locked) {
        HAL_GPIO_WritePin(DOOR_STATUS_GPIO_Port, DOOR_STATUS_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DOOR_STATUS_GPIO_Port, DOOR_STATUS_Pin, GPIO_PIN_SET);
    }
}

static void room_control_update_fan_pwm(room_control_t *room) {
    uint32_t pulse = 0;
    uint32_t period = g_fan_timer_htim->Init.Period;
    pulse = (period * room->current_fan_level) / 100;
    __HAL_TIM_SET_COMPARE(g_fan_timer_htim, TIM_CHANNEL_1, pulse);
}

static fan_level_t room_control_calculate_fan_level(float temperature) {
    if (temperature < 25.0f) return FAN_LEVEL_OFF;
    if (temperature < 28.0f) return FAN_LEVEL_LOW;
    if (temperature < 31.0f) return FAN_LEVEL_MED;
    return FAN_LEVEL_HIGH;
}

static void room_control_clear_input(room_control_t *room) {
    memset(room->input_buffer, 0, sizeof(room->input_buffer));
    room->input_index = 0;
}

const char* room_control_get_input_buffer_masked(const room_control_t *room) {
    static char masked_buffer[PASSWORD_LENGTH + 1];
    for (int i = 0; i < room->input_index; ++i) {
        masked_buffer[i] = '*';
    }
    masked_buffer[room->input_index] = '\0';
    return masked_buffer;
}

const char* room_control_get_state_string(room_control_t *room) {
    switch (room->current_state) {
        case ROOM_STATE_LOCKED: return "LOCKED";
        case ROOM_STATE_AWAITING_PASSWORD: return "INPUT";
        case ROOM_STATE_UNLOCKED: return "UNLOCKED";
        case ROOM_STATE_ACCESS_DENIED: return "DENIED";
    }
    return "UNKNOWN";
}

static void room_control_send_alert(const char* message)
{
    // Esta función enviaría la alerta a través del USART conectado al ESP-01
    // Por simplicidad, lo enviamos al mismo USART2 del debug.
    // En tu implementación final, cambia `huart2` por `huart3` (o el que uses para el ESP-01).
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 100);
}