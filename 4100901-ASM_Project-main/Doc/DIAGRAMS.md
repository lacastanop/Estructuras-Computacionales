# DIAGRAMS.md

## 1. Propósito

Este documento presenta los diagramas que ayudan a visualizar la interacción entre el código ensamblador y el hardware del ARM Cortex‑M4 en la placa NUCLEO‑L476RG.

---

## 2. Diagrama de Contexto

Muestra la relación entre el sistema embebido y los actores externos.

```plaintext
+-------------------------------------------------+
|                     PC Host                    |
|  - Herramienta de debug (GDB/OpenOCD)           |
|  - Monitor serie (UART 115200 8N1)             |
+-------------------------------------------------+
               |           ^
               | SWD       | UART
               v           |
+-------------------------------------------------+
|             NUCLEO-L476RG (Cortex-M4)          |
|                                                 |
|  [ RAM ] <-- Código ASM <-> CMake Toolchain     |
|  [ FLASH ]  Ejecución de instrucciones           |
|  Periféricos: UART, GPIO, SysTick               |
+-------------------------------------------------+
```  

**Actores:**
- **PC Host:** Inicia y controla la sesión de debug; envía y recibe datos UART.  
- **NUCLEO-L476RG:** Ejecuta el código ASM+C, interactúa con la memoria y periféricos.

---

## 3. Diagrama de Contenedor

Estructura de alto nivel del proyecto y cómo se organizan los archivos.

```plaintext
procesadores-arm/                      <-- Repositorio raíz
├── CMakeLists.txt                    <-- Configuración de compilación
├── Src/                              <-- Código fuente
│   ├── main.s                        <-- Rutina principal en ASM
│   ├── decrement.s                   <-- Subrutina decrement
│   └── startup_stm32l476xx.s         <-- Startup (ISR vector) opcional
├── Inc/                              <-- Headers (vacío o con .h)
├── build/                            <-- Archivos generados (elf, bin, hex)
└── Doc/                              <-- Documentación de la práctica
    ├── SETUP.md
    ├── ASM_CONFIG.md
    ├── CODE_EXPLANATION.md
    └── DIAGRAMS.md                   <-- Este archivo
```  

**Contenedores:**
- **CMakeLists.txt:** Orquesta la compilación.
- **Src/:** Implementación en ASM de la lógica.
- **Inc/:** Definiciones de símbolos (opcional).
- **build/:** Artefactos resultado de `CMake: Build`.
- **Doc/:** Guías y diagramas para estudiantes.

---

## 4. Diagrama de Componentes

Detalle de componentes lógicos dentro del firmware.

```plaintext
+-------------------+     +-------------------+     +-------------------+
|    Reset Handler  | --> |   main_asm        | --> |   decrement       |
+-------------------+     +-------------------+     +-------------------+
        |                       |                         |
        v                       v                         v
  Inicialización        Carga MEM_LOC           Bucle de decremento
  (vectores, stack)    (movw/movt, ldr/str)     (subs, str, cmp, b)
```

**Componentes:**
- **Reset Handler:** Código de inicio (startup) que configura el MPU, stack y vectores.
- **main_asm:** Inicializa variables en SRAM y llama a `decrement`.
- **decrement:** Lógica de resta iterativa y contador en registro R7.

---

Con estos diagramas, tienes una visión clara de las interacciones entre hardware, proyecto y componentes de firmware en la práctica de Procesadores.

