# Proyecto Final: Sistema de Control de Acceso y Climatización Remoto

**Curso:** Estructuras Computacionales (4100901)

**Universidad Nacional de Colombia - Sede Manizales**

## 1. Resumen del Proyecto

¡Es hora de integrar todo lo aprendido! El proyecto final consiste en diseñar e implementar un **"Sistema de Control de Acceso y Climatización Remoto"** utilizando la placa NUCLEO-L476RG.

Este proyecto consolida los conocimientos del semestre, desde el uso de la capa HAL, la creación de drivers modulares, la gestión de interrupciones y periféricos, hasta la conectividad en red. El objetivo no es solo construir un sistema funcional, sino también **documentar de forma clara la arquitectura de hardware y firmware**, prestando especial atención a los **patrones de diseño** utilizados.

## 2. Descripción Funcional

### 2.1 Control de Acceso por Teclado
*   El sistema debe permitir el ingreso de una clave de 4 dígitos mediante un teclado matricial.
*   Si la clave es correcta, un LED (simulador de cerradura) se enciende y el estado se muestra en la pantalla OLED.
*   Si la clave es incorrecta, se debe indicar el error en la pantalla y enviar una alerta a internet.

### 2.2 Interfaz de Usuario (Display OLED)
*   La pantalla OLED es la interfaz visual principal.
*   Debe mostrar el estado del sistema ("Bloqueado", "Acceso Concedido"), la temperatura actual, y la velocidad del ventilador.
*   Debe enmascarar la entrada de la clave (ej. `****`).
*   **(Opcional/Bonus)**: Mostrar la hora actual. La hora se puede sincronizar enviando un comando HTTP a través del ESP-01 con `esp-link`.

### 2.3 Control de Climatización
*   Leer la temperatura de la sala usando un sensor analógico (LM35) o digital (I2C/1-Wire).
*   Controlar un ventilador DC mediante una señal **PWM** con 4 niveles de velocidad discretos basados en la temperatura:
    *   **Nivel 0 (0%):** Si Temp < 25°C
    *   **Nivel 1 (30%):** Si 25°C <= Temp < 28°C
    *   **Nivel 2 (70%):** Si 28°C <= Temp < 31°C
    *   **Nivel 3 (100%):** Si Temp >= 31°C
*   **(Opcional/Bonus)**: Implementar transiciones suaves entre los niveles de velocidad del ventilador utilizando DMA para actualizar el registro de captura/comparación del Timer (CCR).

### 2.4 Conectividad y Control Remoto (Requerido)
*   Se debe integrar un módulo **ESP-01** con el firmware **`esp-link`** para actuar como un puente UART-a-WiFi.
*   El sistema debe ser controlable a través de dos interfaces de comandos de texto:
    1.  **Consola de Depuración Local (USART2):** Conectada al PC a través del ST-Link (Virtual COM Port).
    2.  **Consola Remota (USART3):** Conectada al ESP-01 y accesible a través de una conexión TCP/IP (ej. usando `netcat` o PuTTY).
*   Se debe implementar un **parser de comandos** que acepte órdenes simples (ej. `COMANDO:VALOR\n`). Comandos mínimos a implementar:
    *   `GET_TEMP`: Devuelve la temperatura actual.
    *   `GET_STATUS`: Devuelve el estado del sistema (Bloqueado/Desbloqueado, velocidad del ventilador).
    *   `SET_PASS:XXXX`: Permite cambiar la contraseña de acceso de forma remota.
    *   `FORCE_FAN:N`: Permite forzar una velocidad del ventilador (N=0,1,2,3).
*   **(Opcional/Bonus)**: Enviar datos periódicamente a un servicio de IoT como ThingSpeak o un broker MQTT.

## 3. Arquitectura del Sistema (Criterio Clave de Evaluación)

### 3.1. Arquitectura de Hardware (HW)
Deben incluir un diagrama de bloques que muestre la interconexión de todos los componentes.

| Componente | Conexión con STM32L476RG | Periférico Clave | Guía de Referencia |
| :--- | :--- | :--- | :--- |
| Teclado Matricial | 8 pines GPIO | GPIO, EXTI | `KEYPAD.md` |
| Display OLED (SSD1306) | 2 pines (SDA, SCL) | I2C | `SSD1306.md` |
| LED "Cerradura" | 1 pin GPIO (salida) | GPIO | `LIB_PRINCIPLES.md` |
| Sensor de Temperatura | 1 pin Analógico | ADC | *Investigación del estudiante* |
| Ventilador DC (vía Transistor) | 1 pin PWM | Timer (PWM) | *Investigación del estudiante* |
| **ESP-01 (`esp-link`)** | 2 pines (TX, RX) | **USART3** | `RING_BUFFER.md` |
| **Consola Debug PC** | Virtual COM Port | **USART2** | `RING_BUFFER.md` |

### 3.2. Arquitectura de Firmware (FW)
El diseño del firmware es crucial. Deben explicar y justificar su diseño.

*   **Patrón de Diseño "Super Loop":** El núcleo de la aplicación será un **Super Loop no bloqueante**. Es un requisito explícito que el bucle `while(1)` en `main` se ejecute rápidamente y sin demoras (`HAL_Delay`). La gestión del tiempo para tareas periódicas debe hacerse con `HAL_GetTick()`. Su informe debe explicar por qué este patrón es adecuado para este tipo de sistema reactivo.

*   **Máquina de Estados (State Machine):** Se recomienda implementar la lógica principal del sistema (ej. estados `LOCKED`, `UNLOCKED`, `INPUT_PASSWORD`) como una máquina de estados explícita dentro del Super Loop.

*   **Modularidad:** El código debe estar organizado en drivers y librerías reutilizables, como se ha practicado en clase.

*   **Diagrama de Componentes de Software:** Deben crear un diagrama que muestre la interacción entre los módulos.

## 4. Entregables y Criterios de Evaluación

### Entregables
1.  **Código Fuente:** Repositorio de Git (GitHub) con todo el código del proyecto.
2.  **Documentación (`INFORME.md`):** Un informe en Markdown en la raíz del repositorio que contenga:
    *   Nombres de los integrantes.
    *   **Arquitectura de Hardware:** Diagrama y explicación.
    *   **Arquitectura de Firmware:** Explicación de los patrones de diseño (Super Loop, State Machine) y diagrama de componentes.
    *   **Protocolo de Comandos:** Descripción de los comandos remotos implementados.
    *   **Optimización:** Explicación de las técnicas de optimización aplicadas.
3.  **Presentación Final:** Una breve presentación (5-7 minutos) del proyecto a los compañeros del curso, explicando la arquitectura y demostrando su funcionamiento.
4.  **Video Demostración(Opcional):** Video corto (máx. 3 minutos) mostrando el sistema en funcionamiento.

### Criterios de Evaluación
| Criterio | Descripción | Ponderación |
| :--- | :--- | :--- |
| **Funcionalidad Completa** | El sistema implementa las características requeridas (acceso, display, climatización, control remoto). | 40% |
| **Calidad del Código y Arquitectura** | Código modular, no bloqueante. Uso y justificación claros de los patrones de diseño. | 30% |
| **Calidad de la Documentación** | Claridad y completitud del informe y los diagramas. | 15% |
| **Presentación Final** | Claridad, dominio del tema y calidad de la demostración en vivo. | 15% |
| **Puntos Extra (Bonus)** | Implementación de cualquiera de las funcionalidades opcionales. | Hasta +10% sobre la nota final |

---

**¡Manos a la obra! Este proyecto es su oportunidad para brillar y demostrar su capacidad para diseñar y construir un sistema embebido completo y robusto.**
