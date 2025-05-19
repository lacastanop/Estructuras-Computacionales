# Práctica Introductoria: Estructuras Computacionales - Sistema de Control Básico

**Universidad Nacional de Colombia - Sede Manizales**

**Curso:** Estructuras Computacionales (4100901)

## Introducción

¡Bienvenidos al curso de Estructuras Computacionales! Esta primera práctica está diseñada para familiarizarte con las herramientas de desarrollo y la plataforma de hardware que utilizaremos a lo largo del semestre: la placa Nucleo-L476RG con un microcontrolador STM32.

**Objetivo:** Implementar un sistema muy simple que responda a eventos (pulsación de un botón) y se comunique con un PC, utilizando herramientas estándar de la industria como VS Code, STM32CubeMX y la librería HAL (Hardware Abstraction Layer) de STMicroelectronics.

**¿Por qué este enfoque?**
Comenzaremos con un enfoque práctico y guiado, utilizando herramientas que abstraen parte de la complejidad del hardware (HAL, CubeMX). Esto te permitirá tener un sistema funcionando rápidamente. A medida que avance el curso, profundizaremos en *cómo* funciona este sistema a bajo nivel: exploraremos la arquitectura del procesador ARM Cortex-M4, cómo se controlan los periféricos directamente (acceso a registros), la compilación de C a ensamblador y los principios de diseño de firmware eficiente. Esta práctica inicial sienta las bases para entender el proyecto final del curso, que será una versión más completa de este sistema de control.

**Funcionalidad de esta Práctica:**
*   Hacer parpadear un LED integrado (LD2) como señal de "heartbeat" (el sistema está vivo).
*   Detectar la pulsación del botón de usuario integrado (B1).
*   Encender un LED externo durante 3 segundos cuando se presiona el botón B1 (simulando un desbloqueo temporal).
*   Enviar un mensaje al PC a través de comunicación serie (UART) cuando se presiona el botón.
*   Recibir caracteres desde el PC y enviarlos de vuelta (eco).

**Diagrama de Hardware:**

![HW Diagram](Doc/assets/hw_diagram.png)

**Documentación:**

*  [NUCLEO-L476RG](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html).
*  [STM32 Nucleo-64 Boards](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf).
*  [MB1136 Esquematico](https://www.st.com/resource/en/schematic_pack/mb1136-default-c03_schematic.pdf)
*  [STM32L4 Series](https://www.st.com/resource/en/product_presentation/microcontrollers-stm32l4-series-product-overview.pdf)

## Estructura de la Guía

Esta guía está dividida en varias secciones para facilitar el proceso:

1.  **[Configuración del Entorno (SETUP.md)](Doc/SETUP.md):** Instalación de herramientas (VS Code, STM32 Extensions, Git) y cómo preparar tu copia del proyecto (Fork & Clone).
2.  **[Configuración del Proyecto con STM32CubeMX (CUBEMX_CONFIG.md)](Doc/CUBEMX_CONFIG.md):** Pasos para configurar los periféricos necesarios (GPIO, UART, Reloj, Interrupciones) usando la interfaz gráfica integrada en VS Code.
3.  **[Implementación del Código (CODE_IMPLEMENTATION.md)](Doc/CODE_IMPLEMENTATION.md):** Escribir el código C necesario en `main.c` y los archivos de interrupción para implementar la lógica de la aplicación.

***¡Empecemos!***

Dirígete primero a [SETUP.md](Doc/SETUP.md) para configurar tu entorno de desarrollo.
