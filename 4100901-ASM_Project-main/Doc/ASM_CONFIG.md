# ASM_CONFIG.md

## 1. Objetivo

Profundizar en la configuración y edición del archivo ensamblador `main.s`, entender las directivas clave y organizar el código ASM para la práctica de Procesadores.

## 2. Estructura del archivo ASM

- **Sección de texto**: define el código ejecutable.
- **Directivas de sintaxis**: indicar al compilador el modo Thumb y la sintaxis unificada.
- **Símbolos globales**: exponer etiquetas para enlazar desde C.
- **Constantes**: usar `.equ` para definir direcciones o parámetros.

Ejemplo básico de encabezado en `Src/main.s`:
```assembly
    .section .text            @ Código ejecutable
    .syntax unified           @ Sintaxis unificada ARM/Thumb
    .thumb                    @ Genera instrucciones Thumb (16/32 bits)
    .global main              @ Hacer visible la etiqueta main

    .equ MEM_LOC, 0x20000200  @ Dirección fija en SRAM para datos
```  

## 3. Definición de la función `main`

Implementa en ASM la rutina principal que inicializa datos y llama a una subrutina:

```assembly
main:
    @ Inicializar registros
    movw    r0, #:lower16:MEM_LOC    @ Parte baja de la dirección
    movt    r0, #:upper16:MEM_LOC    @ Parte alta (ahora R0 = MEM_LOC)
    ldr     r1, =813017              @ Valor inicial en R1
    str     r1, [r0]                 @ Almacena valor en MEM_LOC

    @ Llamar a la subrutina decrement
    bl      decrement

    @ Bucle infinito de final
loop:
    b       loop
```  

## 4. Implementación de la rutina `decrement`

Esta subrutina decrementa el valor en memoria hasta que sea ≤ 0 y cuenta iteraciones:

```assembly
    .global decrement
decrement:
    @ r0 apunta a MEM_LOC, obtener valor
    ldr     r0, [r0]
    subs    r0, r0, r1          @ r0 = r0 - r1
    str     r0, [r0]           @ guarda r0 en MEM_LOC
    cmp     r0, #0             @ ¿≤ 0?
    ble     exit_decrement     @ Si sí, salir

    @ Iteración válida
    add     r7, r7, #1         @ Contador en r7
    b       decrement          @ Repetir

exit_decrement:
    @ Retorno a main
    bx      lr
```  

> **Nota:** `bx lr` retorna al link register hacia la función que llamó.

## 6. Depuración en VS Code

1. Coloca breakpoints en `main_asm` o `decrement` para inspeccionar registros.
2. Inicia la depuración y observa cómo avanza la ejecución línea a línea.

---

Continúa con `CODE_EXPLANATION.md` para documentar cada instrucción y su efecto en registros/memoria.

