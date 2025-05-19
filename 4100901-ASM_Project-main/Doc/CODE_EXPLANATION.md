# CODE_EXPLANATION.md

## 1. Propósito de este documento

Este archivo describe en detalle cada instrucción del ejemplo en ensamblador (`main.s`). El objetivo es que entiendas cómo se traduce el flujo de tu programa a nivel de registros, memoria y control de flujo en el ARM Cortex‑M4.

---

## 2. Encabezados y directivas

```assembly
    .section .text            @ Define la sección de código en memoria flash
    .syntax unified           @ Usa la sintaxis unificada ARM/Thumb
    .thumb                    @ Genera instrucciones Thumb de 16/32 bits
    .global main_asm          @ Exporta etiqueta para enlazar desde C
    .equ MEM_LOC, 0x20000200  @ Define una constante para dirección en SRAM
```  

- **.section .text**: Ubica el ensamblador en la región de código ejecutable.
- **.syntax unified**: Permite usar una única sintaxis para instrucciones ARM y Thumb, facilitando migraciones.
- **.thumb**: Indica al ensamblador que genere instrucciones Thumb, que son más compactas y usadas en Cortex‑M.
- **.global main_asm**: Hace visible la etiqueta `main_asm` al enlazador para que el `main()` en C pueda invocarla.
- **.equ MEM_LOC**: Asigna una etiqueta simbólica a la dirección `0x20000200`, donde almacenaremos datos en RAM.

---

## 3. Función `main_asm`

```assembly
main_asm:
    @ Cargar la dirección base en R0 usando movw/movt
    movw    r0, #:lower16:MEM_LOC    @ R0 <- 0x0200 (parte baja de 0x20000200)
    movt    r0, #:upper16:MEM_LOC    @ R0 <- 0x2000 (parte alta) => Ahora R0 = 0x20000200

    @ Cargar valor inicial en R1
    ldr     r1, =813017              @ R1 <- 813017 (literal cargado desde la tabla de constantes)

    @ Almacenar R1 en la dirección apuntada por R0
    str     r1, [r0]                 @ Mem[MEM_LOC] <- 813017

    @ Invocar la rutina decrement
    bl      decrement                @ Link Register = PC+4; PC = address(decrement)

    @ Bucle infinito de finalización
loop:
    b       loop                     @ Salto incondicional a sí mismo para detener ejecución
```  

| Instrucción | Registros/Memoria                        | Comentario                                                  |
|-------------|------------------------------------------|-------------------------------------------------------------|
| movw r0,... | R0[15:0] = lower16(MEM_LOC)             | Carga los 16 bits bajos de la dirección MEM_LOC en R0.      |
| movt r0,... | R0[31:16] = upper16(MEM_LOC)            | Carga los 16 bits altos de MEM_LOC en R0, completando 32b.  |
| ldr r1,=813017 | R1 = 813017                          | Literal cargado mediante pseudo-instrucción.                |
| str r1,[r0] | Mem[0x20000200] = 813017               | Guarda el valor inicial en SRAM.                           |
| bl decrement| LR = return_address; PC = &decrement     | Llama a `decrement`, guardando PC de retorno en LR.        |
| loop: b loop| PC = loop                               | Bucle infinito al acabar.                                  |

---

## 4. Subrutina `decrement`

```assembly
    .global decrement

decrement:
    @ Leer valor actual de MEM_LOC
    ldr     r2, [r0]                @ R2 <- Mem[MEM_LOC]                  
    @ Restar R1 (el decremento)
    subs    r2, r2, r1              @ R2 = R2 - R1; actualiza flags N,Z,C,V |
    @ Guardar nuevo valor en memoria
    str     r2, [r0]                @ Mem[MEM_LOC] <- R2                  
    @ Comparar con cero
    cmp     r2, #0                  @ Actualiza flags N,Z                   
    @ Si ≤ 0, salir
    ble     exit_decrement          @ Si Z=1 o N=1, branch a exit_decrement |

    @ Si > 0, incrementar contador en R7
    add     r7, r7, #1              @ R7 = R7 + 1                        
    @ Repetir
    b       decrement               @ Loop hasta que valor ≤ 0             

exit_decrement:
    bx      lr                      @ PC = LR, retorno a caller (main_asm)
```  

- **ldr r2, [r0]**: carga el valor actual de la dirección en R0 (MEM_LOC) a R2.
- **subs r2, r2, r1**: resta R1 de R2 y actualiza los *condition flags*:
  - **Z** (zero) se activa si el resultado es cero.
  - **N** (negative) si el bit más significativo del resultado es 1.
- **str r2, [r0]**: escribe el nuevo valor de vuelta en MEM_LOC.
- **cmp r2, #0**: compara con cero, afectando Z y N.
- **ble exit_decrement**: salta si *Z=1* o *N=1* (resultado ≤ 0).
- **add r7, r7, #1**: incrementa el contador de iteraciones en R7.
- **b decrement**: vuelve al inicio para otro ciclo.
- **bx lr**: retorna a la instrucción siguiente tras el `bl decrement` en `main_asm`.

---

## 5. Visualización de flujo

1. **main_asm** inicializa memoria y llama a `decrement`.
2. **decrement** ejecuta un bucle de resta hasta llegar a cero o negativo.
3. Cada iteración válida incrementa **R7**.
4. Al finalizar, retorna a **main_asm**, que entra en un bucle infinito.

Con esta explicación, deberías comprender cómo funciona el código a nivel de ISA y registros en la placa NUCLEO‑L476RG.

