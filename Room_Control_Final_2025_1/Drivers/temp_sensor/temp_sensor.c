#include "temp_sensor.h"

static ADC_HandleTypeDef* g_hadc; // Puntero al handle del ADC

void temp_sensor_init(ADC_HandleTypeDef* hadc) {
    g_hadc = hadc;
}

float temp_sensor_read(void) {
    if (g_hadc == NULL) {
        return -1.0f; // Retornar un error si no está inicializado
    }

    float temperature = 0.0f;

    // Iniciar, sondear, leer y detener el ADC
    if (HAL_ADC_Start(g_hadc) == HAL_OK) {
        if (HAL_ADC_PollForConversion(g_hadc, 100) == HAL_OK) {
            uint32_t adc_value = HAL_ADC_GetValue(g_hadc);
            float voltage = ((float)adc_value / 4095.0f) * 3.3f;
            temperature = voltage * 100.0f; // Para LM35
        }
        HAL_ADC_Stop(g_hadc);
    }
    
    return temperature;
}