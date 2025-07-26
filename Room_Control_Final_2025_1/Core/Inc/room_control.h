#ifndef ROOM_CONTROL_H
#define ROOM_CONTROL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define PASSWORD_LENGTH 4
#define MAX_TEMP_READINGS 5

// --- NUEVAS CONSTANTES ---
#define UNLOCKED_TIMEOUT_MS         10000 // 10 segundos para que se vuelva a bloquear
#define INPUT_TIMEOUT_MS            10000 // 10 segundos para ingresar clave
#define ACCESS_DENIED_TIMEOUT_MS    3000  // 3 segundos mostrando error

typedef enum {
    ROOM_STATE_LOCKED,
    ROOM_STATE_AWAITING_PASSWORD, // Renombrado para más claridad
    ROOM_STATE_UNLOCKED,
    ROOM_STATE_ACCESS_DENIED,
} room_state_t;

typedef enum {
    FAN_LEVEL_OFF = 0,    // 0% PWM
    FAN_LEVEL_LOW = 30,   // 30% PWM
    FAN_LEVEL_MED = 70,   // 70% PWM
    FAN_LEVEL_HIGH = 100  // 100% PWM
} fan_level_t;

typedef struct {
    room_state_t current_state;
    char password[PASSWORD_LENGTH + 1];
    char input_buffer[PASSWORD_LENGTH + 1];
    uint8_t input_index;
    uint32_t last_activity_time; // Un solo temporizador para timeouts
    uint32_t state_enter_time;

    // Control de la puerta
    bool door_locked;

    // Control de temperatura y ventilador
    float current_temperature;
    fan_level_t current_fan_level;
    bool manual_fan_override;

    // Flags de actualización
    bool display_update_needed;
} room_control_t;

// --- FUNCIONES PÚBLICAS ---
void room_control_init(room_control_t *room, TIM_HandleTypeDef *fan_timer);
void room_control_update(room_control_t *room);
void room_control_process_key(room_control_t *room, char key);
void room_control_set_temperature(room_control_t *room, float temperature);

// --- NUEVAS FUNCIONES PARA COMANDOS ---
void room_control_process_command(room_control_t *room, char *command_line);
const char* room_control_get_state_string(room_control_t *room);


// Getters de estado (útiles para comandos y display)
room_state_t room_control_get_state(const room_control_t *room);
bool room_control_is_door_locked(const room_control_t *room);
fan_level_t room_control_get_fan_level(const room_control_t *room);
float room_control_get_temperature(const room_control_t *room);
const char* room_control_get_input_buffer_masked(const room_control_t *room);


#endif