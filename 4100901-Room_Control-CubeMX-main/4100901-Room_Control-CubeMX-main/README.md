# Segunda Fase del Curso: Estructuras Computacionales

**Universidad Nacional de Colombia - Sede Manizales**

**Curso:** Estructuras Computacionales (4100901)

## Introducción

En esta segunda fase del curso **Estructuras Computacionales**, nos enfocaremos en la **implementación de los componentes fundamentales de un dispositivo computacional**. El propósito principal es fortalecer las habilidades en la creación y reutilización de drivers/librerías, así como en la integración de componentes de entrada/salida (I/O).

Los conocimientos adquiridos se aplicarán progresivamente en la propuesta para el proyecto final llamado **"Control de Sala"**, que también integrará:

* Un teclado Hexadecimal.
* Un display OLED para visualización.
* Un sensor de temperatura.
* Un sistema de ventilación controlado por PWM.
* Conexión a Internet mediante el módulo ESP01.

## Objetivos Específicos

* Comprender y dominar el uso de STM32CubeMX para la configuración inicial de proyectos.
* Aprender los principios y la estructura de librerías modulares en C.
* Implementar componentes como GPIO, UART, Timers y Buffers circulares.
* Integrar estos componentes para construir sistemas robustos y reutilizables.

## Documentos Iniciales

Este repositorio contiene las siguientes guías para acompañar el proceso de aprendizaje:

* [`Doc/SETUP.md`](Doc/SETUP.md): Guía de configuración del entorno de desarrollo.
* [`Doc/CUBEMX.md`](Doc/CUBEMX.md): Introducción a STM32CubeMX y generación de proyectos.
* [`Doc/LIB_PRINCIPLES.md`](Doc/LIB_PRINCIPLES.md): Principios para la creación de librerías en C.
* [`Doc/RING_BUFFER.md`](Doc/RING_BUFFER.md): Implementación de una librería de buffer circular.

Cada guía contiene ejemplos prácticos y explicaciones orientadas a fortalecer la comprensión conceptual y la implementación práctica.

---

¡Manos a la obra! Comienza por revisar la [Configuración del Entorno (Doc/SETUP.md)](Doc/SETUP.md).
