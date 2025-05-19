# SETUP.md

## 1. Revisión de la instalación de herramientas

Antes de comenzar, asegúrate de haber completado la instalación de todas las herramientas necesarias. Para ello, consulta la [Guía Inicial de Instalación](https://github.com/saacifuentesmu/4100901-Intro_Project/blob/main/Doc/SETUP.md).

## 2. Fork y clonación del repositorio de la práctica

El código base ya está disponible en un repositorio que compartiré contigo. Sigue estos pasos:

1. Abre el enlace al repositorio: `https://github.com/saacifuentesmu/4100901-ASM_Project`.
2. Haz clic en **Fork** para crear una copia en tu cuenta de GitHub.
3. Entra en tu repositorio forkeado (`https://github.com/{TuUsuario}/4100901-ASM_Project`).
4. Haz clic en **Code** > **HTTPS** y copia la URL.
5. En tu terminal, clona el fork:
   ```bash
   git clone https://github.com/{TuUsuario}/4100901-ASM_Project.git
   ```

## 3. Crear el proyecto vacío con la extensión STM32 para VS Code

1. Abre la carpeta del repositorio en VS Code.
2. Pulsa STM32 Extension (barra izquierda) y ejecuta el comando **Create Empty Project**.
3. Usa el mismo nombre del repositorio clonado `4100901-ASM_Project` y selecciona la carpeta que acabaste de clonar
4. Selecciona `BOARD` y luego el modelo `NUCLEO‑L476RG`.
4. Pulsa **Create project** y *Open this window*. Esto generará un `CMakeLists.txt` básico y la estructura de carpetas (`Src/`, `Inc/`).
5. Verifica que existan los archivos:
   - `Src/main.c`
   - `CMakeLists.txt`
   - `stm32l476rgtx_FLASH.ld`

## 4. Explicación del `main.c` inicial

Abre `Src/main.c`. Contiene lo mínimo para arrancar:

```c
#include <stdint.h>

int main(void)
{
    /* Loop forever */
	for(;;);
}
```

- `#include <stdint.h>`: tipos estándar de ancho fijo.
- `main()`: entra en bucle infinito para que el programa no termine.

## 5. Renombrar `main.c` a `main.s` y ajustar CMakeLists.txt

1. En `Src/`, renombra `main.c` a `main.s`.
2. Modifica `cmake/vscode_generated.cmake`:
   - Cambia la extensión de los fuentes:
     ```cmake
      # Sources
      set(sources_SRCS ${sources_SRCS}
          
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.s
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/syscall.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/sysmem.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Startup/startup_stm32l476rgtx.s
      )
     ```
   - Asegúrate de que el compilador ASM está configurado:

3. Guarda y vuelve a ejecutar **CMake: Configure** y **CMake: Build**.

---

A continuación, continúa con **ASM_CONFIG.md** para profundizar en la configuración y edición del código ensamblador.

