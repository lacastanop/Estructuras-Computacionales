# Guía de Optimización para STM32L476RG

## Objetivo

Establecer criterios claros para aplicar técnicas de optimización de **memoria**, **tiempo de ejecución** y **consumo energético** en aplicaciones embebidas, con énfasis en buenas prácticas orientadas a la arquitectura STM32L4.

---

## 1. Optimización de Memoria

### 1.1 Diagnóstico

* Revisa la salida del compilador para observar el uso consolidado de RAM, FLASH, heap y stack.
* Genera un archivo `.map` para identificar el uso detallado de secciones de memoria.
* Usa `arm-none-eabi-size` para obtener una vista rápida de la memoria ocupada.

### 1.2 Estrategias

* Declara variables `const` cuando el contenido no cambia, para que se ubiquen en FLASH.
* Reemplaza mensajes repetidos con tablas `const char*`.
* Ajusta el tamaño de buffers al mínimo necesario.
* Usa tipos de datos precisos (ej. `uint8_t` en lugar de `int`).
* Desactiva módulos HAL innecesarios desde `.ioc`.
* Si lo requiere, reemplaza HAL por acceso LL o directo a registros.
* Activa optimizaciones de compilador:

  * `-Os`: minimiza el tamaño del código.
  * `--gc-sections`: elimina secciones sin referencias.
* Considera el uso de FPU/DSP/hardware crypto para reducir código en operaciones específicas.

### 1.3 Ejercicio Guiado

Declara inicialmente una variable global en la sección:

```c
/* USER CODE BEGIN PV */
char lorem_text[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed non risus...";
/* USER CODE END PV */
```

E imprímela en:

```c
/* USER CODE BEGIN 2 */
printf("%s\n", lorem_text);
/* USER CODE END 2 */
```

Después:

1. Muévela dentro de `main()` como variable local dentro de `/* USER CODE BEGIN 2 */`.
2. Declárala como `const char lorem_text[]` para que pase a FLASH.
3. Compara el uso de RAM y FLASH antes y después (usa `.map` o `arm-none-eabi-size`).

También puedes aplicar:

```c
const char* messages[] = {"OK", "ERROR", "WAIT", "READY"};
```

---

## 2. Optimización de Tiempo de Ejecución

### 2.1 Diagnóstico

* Usa `HAL_GetTick()` o `SysTick` para medir duración de rutinas.
* Alternativamente, activa un pin GPIO antes/después de una rutina crítica y mide con osciloscopio.
* Herramientas avanzadas: SWV, ITM, STM32CubeMonitor.

### 2.2 Estrategias

* Evita operaciones costosas (`/`, `%`, `pow`) en tiempo real.
* Prefiere estructuras `switch-case`.
* Usa periféricos en modo interrupt o DMA cuando sea posible.
* Reemplaza `HAL_Delay()` por lógica basada en `HAL_GetTick()` para evitar bloqueo.
* Usa hardware como FPU o extensiones SIMD si haces cálculos complejos.

### 2.3 Ejercicio Guiado

Implementa una función heartbeat para parpadeo de LED cada 500 ms.

**Versión bloqueante:**
Agrega en `/* USER CODE BEGIN WHILE */`:

```c
while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(500);
}
```

**Versión no bloqueante:**
Reemplaza el `while (1)` por:

```c
uint32_t last_toggle = 0;
while (1) {
    if ((HAL_GetTick() - last_toggle) >= 500) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        last_toggle = HAL_GetTick();
    }
    // Puedes hacer otras tareas aquí
}
```

Observa que la segunda versión permite multitarea y reduce el tiempo bloqueado del CPU.

---

## 3. Optimización de Consumo Energético

### 3.1 Diagnóstico

* Usa un medidor USB o ST-LINK EnergyMeter.
* Observa cómo cambia el consumo cuando el MCU entra en reposo.

### 3.2 Estrategias

* Usa `HAL_PWR_EnterSTOPMode()` para reducir consumo al mínimo.
* Desactiva clocks y periféricos no usados.
* Usa `__WFI()` para detener la CPU mientras espera interrupciones.
* Reduce la frecuencia del reloj del sistema si el rendimiento lo permite.

### 3.3 Ejercicio Guiado

Implementa lógica para entrar en modo STOP luego de 5 segundos sin actividad.
En `/* USER CODE BEGIN WHILE */`:

```c
if ((HAL_GetTick() - last_key_tick) > 5000) {
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config(); // Restaurar configuración de reloj
}
```

Despierta con EXTI o UART. Mide el consumo antes y después.

---

## 4. Análisis Comparativo

Completa la siguiente tabla durante tus pruebas:

| Métrica                   | Antes | Después | Mejora |
| ------------------------- | ----- | ------- | ------ |
| Uso de FLASH (bytes)      |       |         |        |
| Uso de RAM (bytes)        |       |         |        |
| Tiempo de ciclo LED       |       |         |        |
| Consumo en reposo (mA)    |       |         |        |
| Consumo en actividad (mA) |       |         |        |

---

**Siguiente paso:** Aplica estas optimizaciones al proyecto "Room Control" y documenta tus resultados. Cada ciclo de reloj, byte y miliampere cuenta en sistemas embebidos.
