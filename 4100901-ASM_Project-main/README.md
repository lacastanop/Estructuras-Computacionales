## Práctica Introductoria: Procesadores - ARM Cortex-M4 y ISA

**Universidad Nacional de Colombia - Sede Manizales**  
**Curso:** Estructuras Computacionales (4100901)  
**Módulo:** Procesadores y Arquitectura de Conjunto de Instrucciones (ISA)

### 1. Introducción

En este módulo exploraremos los fundamentos de la arquitectura ARM Cortex‑M4 y los principios de los Conjuntos de Instrucciones (ISA) a través de ejemplos prácticos en la placa NUCLEO‑L476RG.

**Objetivos de la práctica:**
- Familiarizarse con la configuración de un proyecto STM32 vacío en VS Code usando CMake.  
- Aprender a incorporar y compilar código ensamblador (ASM) junto con C.  
- Comprender el flujo de ejecución de instrucciones en un procesador Cortex‑M4.  
- Desarrollar diagramas de contexto y componentes para visualizar la interacción entre hardware y código.

### 2. Hardware y Herramientas

- **Placa:** NUCLEO‑L476RG (ARM Cortex‑M4 a 80 MHz)  
- **Editor/IDE:** Visual Studio Code (con extensiones STM32 y CMake Tools)  
- **Compilación:** CMake + toolchain ARM (arm-none-eabi-gcc)  

### 3. Estructura de la Guía

La guía se organiza en varios documentos Markdown que encontrarás en la carpeta `Doc/`:

1. **[SETUP.md](Doc/SETUP.md):** Configuración del entorno y creación del proyecto vacío en VS Code.  
2. **[ASM_CONFIG.md](Doc/ASM_CONFIG.md):** Renombrar `main`, modificar `CMakeLists.txt` e incluir soporte para archivos `.s`.  
3. **[CODE_EXPLANATION.md](Doc/CODE_EXPLANATION.md):** Explicación detallada del código ensamblador de ejemplo, con comentarios y mejoras para los estudiantes.  
4. **[DIAGRAMS.md](Doc/DIAGRAMS.md):** Diagramas de contexto, contenedor y componentes que ilustran la ejecución y recursos del procesador.

> **Nota:** Cada documento incluye enlaces relativos para facilitar la navegación y referencias cruzadas.

![ASM](https://github.com/user-attachments/assets/cd4f5ce2-9beb-4d5d-bc49-9722f4ec6afa)
