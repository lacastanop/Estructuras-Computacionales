// --- Ejemplo de parpadeo de LED LD2 en STM32F476RGTx -------------------------

// --- Variables globales ------------------------------------------------------
    .section .data
button_pressed: .word 0   // Bandera: 0 = no presionado, 1 = presionado
tick_counter:   .word 0   // Contador de milisegundos (control de tiempo)

// --- Código --------------------------------------------------------
    .section .text
    .syntax unified
    .thumb

    .global main
    .global init_led
    .global init_button
    .global read_button
    .global init_systick
    .global SysTick_Handler

// --- Direcciones y registros------------------------------------------------
    // LED LD2 (PA5)
    .equ RCC_BASE,           0x40021000
    .equ RCC_AHB2ENR,        RCC_BASE + 0x4C
    .equ GPIOA_BASE,         0x48000000
    .equ GPIOA_MODER,        GPIOA_BASE + 0x00
    .equ GPIOA_ODR,          GPIOA_BASE + 0x14
    .equ LD2_PIN,            5
    .equ LD2_PIN_MODER,      (5 * 2)
    .equ LD2_PIN_MASK,       (1 << 5)

    // Botón B1 (PC13)
    .equ GPIOC_BASE,         0x48000800
    .equ GPIOC_MODER,        GPIOC_BASE + 0x00
    .equ GPIOC_IDR,          GPIOC_BASE + 0x10
    .equ BUTTON_PIN,         13
    .equ BUTTON_PIN_MODER,   (13 * 2)
    .equ BUTTON_PIN_MASK,    (1 << 13)

    // SysTick
    .equ SYST_CSR,           0xE000E010
    .equ SYST_RVR,           0xE000E014
    .equ SYST_CVR,           0xE000E018
    .equ HSI_FREQ,           4000000    // Oscilador interno (4 MHz)

// --- Programa principal --------------------------------------------------------
main:
    bl init_led         // Inicializa configuración de LED
    bl init_button      // Inicializa configuración de botón
    bl init_systick     // Configura SysTick para interrupción cada 1 ms

loop:
    bl read_button      // Lee el botón y controla el LED
    wfi                 // Espera interrupción (optimiza energía)
    b loop              // Vuelve a leer el botón

// --- Inicialización de GPIOA PA5 para el LED LD2 -----------------------------
init_led:
    movw  r0, #:lower16:RCC_AHB2ENR // Dirección RCC_AHB2ENR baja
    movt  r0, #:upper16:RCC_AHB2ENR // Dirección RCC_AHB2ENR alta
    ldr   r1, [r0]                  // Carga contenido
    orr   r1, r1, #(1 << 0)        // Activa reloj GPIOA
    str   r1, [r0]                  // Guarda

    movw  r0, #:lower16:GPIOA_MODER  // Dirección GPIOA_MODER baja
    movt  r0, #:upper16:GPIOA_MODER // Dirección GPIOA_MODER alta
    ldr   r1, [r0]                   // Carga configuración actual
    bic   r1, r1, #(0b11 << LD2_PIN_MODER) // Limpia configuración previa
    orr   r1, r1, #(0b01 << LD2_PIN_MODER) // Configura PA5 como salida
    str   r1, [r0]      // Guarda
    bx    lr         // Retorna

// --- Inicialización de Botón GPIOC PC13 como entrada -------------------------
init_button:
    movw  r0, #:lower16:RCC_AHB2ENR
    movt  r0, #:upper16:RCC_AHB2ENR
    ldr   r1, [r0]
    orr   r1, r1, #(1 << 2)        // Habilitar reloj GPIOC
    str   r1, [r0]

    movw  r0, #:lower16:GPIOC_MODER
    movt  r0, #:upper16:GPIOC_MODER
    ldr   r1, [r0]
    bic   r1, r1, #(0b11 << BUTTON_PIN_MODER) // Configura PC13 como entrada
    str   r1, [r0]
    bx    lr

// --- Leer estado del botón y controlar LED ------------------------------------
read_button:
    movw  r0, #:lower16:GPIOC_IDR
    movt  r0, #:upper16:GPIOC_IDR
    ldr   r1, [r0]                      // Lee estado del puerto C
    movs  r2, #BUTTON_PIN_MASK          
    tst   r1, r2                        // Testea si el botón está presionado
    bne   button_not_pressed           //  Si no, salta

    ldr   r3, =button_pressed
    ldr   r4, [r3]                     // Carga estado del botón
    cmp   r4, #0
    bne   button_not_pressed          // Si ya estaba presionado, salta

    movs  r4, #1
    str   r4, [r3]                     // Marca como presionado

    // Encender LED
    movw  r0, #:lower16:GPIOA_ODR
    movt  r0, #:upper16:GPIOA_ODR
    ldr   r1, [r0]
    orr   r1, r1, #LD2_PIN_MASK
    str   r1, [r0]

    // Reiniciar contador de ticks
    ldr   r5, =tick_counter
    movs  r6, #0
    str   r6, [r5]

button_not_pressed:
    // Verificar si se deben apagar el LED después de 3s
    ldr   r3, =button_pressed
    ldr   r4, [r3]
    cmp   r4, #1
    bne   end_read_button                   // Si no está presionado, salir

    ldr   r5, =tick_counter
    ldr   r6, [r5]
    ldr   r7, =3000                    // 3000 ms
    cmp   r6, r7
    blt   end_read_button              // Si aún no pasan 3s, salir

    // Apagar LED
    movw  r0, #:lower16:GPIOA_ODR
    movt  r0, #:upper16:GPIOA_ODR
    ldr   r1, [r0]
    bic   r1, r1, #LD2_PIN_MASK
    str   r1, [r0]

    movs  r4, #0
    str   r4, [r3]                     // Resetea bandera

end_read_button:
    bx    lr

// --- Inicializar SysTick para 1 ms --------------------------------------------
init_systick:
    movw  r0, #:lower16:SYST_RVR
    movt  r0, #:upper16:SYST_RVR
    movw  r1, #3999                   // Valor para 1 ms
    movt  r1, #0
    str   r1, [r0]

    movw  r0, #:lower16:SYST_CSR
    movt  r0, #:upper16:SYST_CSR
    movs  r1, #(1 << 0)|(1 << 1)|(1 << 2) // ENABLE, TICKINT, CLKSOURCE // Habilita SysTick + interrupciones
    str   r1, [r0]
    bx    lr

// --- Manejador de la interrupción SysTick -------------------------------------
    .thumb_func
SysTick_Handler:
    ldr   r0, =tick_counter
    ldr   r1, [r0]
    adds  r1, r1, #1                   // Incrementa contador cada 1 ms
    str   r1, [r0]
    bx    lr
