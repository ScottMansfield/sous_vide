;-------------------------------------------------------------------------------
; Main Sous Vide code file
;-------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
; Register / constant definitions
;-------------------------------------------------------------------------------

WDTCTL      .set    0120h                   ; Watchdog timer control register
WDTPW       .set    05A00h                  ; Watchdog timer password (must be written with changes)
WDTHOLD     .set    080h                    ; Watchdog timer hold bit

DCOCTL      .set    056h                    ; Digitally Controlled Oscillator control register
DCO0        .set    020h                    ; DCO Control bit 0
DCO1        .set    040h                    ; DCO Control bit 1
DCO2        .set    080h                    ; DCO Control bit 2

DCOMAX      .set    DCO0|DCO1|DCO2          ; Convenience constant - maximum frequency in range

BCSCTL1     .set    057h                    ; Basic Clock System control register 1
BCSCTLDEF   .set    087h                    ; Default value for BCSCTL register on startup
RSEL0       .set    001h                    ; Range select bit 0
RSEL1       .set    002h                    ; Range select bit 1
RSEL2       .set    004h                    ; Range select bit 2
RSEL3       .set    008h                    ; Range select bit 3

BCSCTLMAX   .set    BCSCTLDEF|RSEL0|RSEL1|RSEL2|RSEL3 ; Convenience constant - maximum frequency range

P1IN        .set    020h                    ; Port 1 in register
P1OUT       .set    021h                    ; Port 1 out register
P1DIR       .set    022h                    ; Port 1 direction register

PIN0        .set    001h                    ; Port 1 Pin 0
PIN1        .set    002h                    ; Port 1 Pin 1
PIN2        .set    004h                    ; Port 1 Pin 2
PIN3        .set    008h                    ; Port 1 Pin 3
PIN4        .set    010h                    ; Port 1 Pin 4
PIN5        .set    020h                    ; Port 1 Pin 5
PIN6        .set    040h                    ; Port 1 Pin 6
PIN7        .set    080h                    ; Port 1 Pin 7

OUTPINS     .set    PIN0|PIN2|PIN6|PIN7     ; Convenience constant - output pins
INPINS      .set    PIN5                    ; Convenience constant - input pins

RELAY       .set    PIN2                    ; Alias for relay circuit

SPICS       .set    PIN7                    ; Bit-banged SPI constants
SPICLK      .set    PIN6                    ; Since we are manually reading from the SPI interface
SPIMISO     .set    PIN5                    ; these constants give a sense of normalcy

RAMBASE     .set    0200h                   ; start address of the 128 bytes of RAM
RAMSIZE     .set    07fh                    ; the total size of the RAM
RAMEND      .set    RAMBASE + RAMSIZE       ; the last address in RAM
OBSSTART    .set    RAMBASE                 ; convenience alias for the start of RAM
NUMOBS      .set    64                      ; the number of observations held in RAM
OBSEND      .set    RAMBASE + NUMOBS        ; the address after the last observation held in RAM

TARGET      .set    0CFh                    ; the target water temperature, 132F, or 55.5C

; Delay Gold HI and LO
DLYGLDHI    .set    0032Dh                  ; High 16 bits of the base delay value
DLYGLDLO    .set    0CD55h                  ; Low 16 bits of the base delay value

;-------------------------------------------------------------------------------
; Linker directives
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to make it known to linker.

            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking and retain current section.
            .retainrefs                     ; And retain any sections that have references to current section.

;-------------------------------------------------------------------------------
; Setup
;-------------------------------------------------------------------------------
RESET       mov     #__STACK_END, SP        ; Initialize stack pointer

            mov     #WDTPW|WDTHOLD, &WDTCTL ; Stop watchdog timer

            mov     #BCSCTLMAX, &BCSCTL1    ; Select highest frequency range
            mov     #DCOMAX, &DCOCTL        ; Select maximum DCO frequency in range

            or      #OUTPINS, &P1DIR        ; Set port 1 pins 0, 2, 4, and 5 as output
            and     #~INPINS, &P1DIR        ; Set port 1 pin 7 as input

            or      #SPICS, &P1OUT          ; SPI chip select high by default

            mov     #OBSSTART, r10           ; set observation position counter to "0"

            ; Zero out all the RAM
            mov     #RAMBASE, r4            ; Start at the base of RAM
clr_ram     mov     #0, 0(r4)               ; write to address in R4
            add     #2, r4                  ; increment address
            cmp     #RAMEND, r4             ; Compare address to RAM end address to set flags
            jl      clr_ram                 ; loop as long as there's more words to write

;-------------------------------------------------------------------------------
; Main
;-------------------------------------------------------------------------------

loop        ;xor     #PIN0, &P1OUT           ; Flip pin 0 (red LED) output state

            ; Busy wait for 10 seconds
            ;mov     #0032Dh, r4             ; Delay 10 seconds (10 seconds * 16 MHz / 3 inst/count)
            ;mov     #0CD55h, r5             ; 0x32DCD55
            ;call    #fn_delay

            ; Read the most recent temperature
            call    #fn_readtemp            ; Read data from thermocouple chip
            and     #01FEh, r4              ; Mask off the 8 bits that has the 1/2 degree C bit as the LSB
            rra     r4                      ; Get rid of the 1/4 degree C bit

            ; Create the delta
            sub.b   #TARGET, r4             ; Get the delta between the reading and the target
            mov.b   r4, 0(r10)              ; Copy the delta into the current RAM slot
            inc     r10                     ; Increment RAM slot index
            cmp     #OBSEND, r10            ;
            jnz     calc                    ; If the counter is equal to the max number of observations,
            mov     #OBSSTART, r10          ; then reset it to the base address of RAM

            ; Calculate the duty cycle
calc        call    #fn_calcduty            ; Calculate the duty cycle for the next round
            ; on r4 r5
            bis     #PIN0|RELAY, &P1OUT     ; Turn on relay and LED showing heater is on
            call    #fn_delay               ; Busy wait while heater is on

            ; off r6 r7
            bic     #PIN0|RELAY, &P1OUT     ; Turn off relay and LED
            mov     r6, r4                  ; Copy off delay time to proper regsters
            mov     r7, r5                  ;
            call    #fn_delay               ; Busy wait until end of cycle

            jmp loop
                                            

;-------------------------------------------------------------------------------
; Functions
;-------------------------------------------------------------------------------

; Delay function
;
; The numbers passed in need to be calculated by the caller to match the delay expected
; This also varies based on the clock speed. In this program, it is always at the maximum,
; which is 16 MHz
;
; With this function, 1 second takes (2^24)/3 clock cycles to run (give or take a dozen cycles)
;
; Because of the way this function works, delaying a certain time by clock cycles means
; calculating the right number of clock cycles and then dividing by 3.
;
fn_delay
            ;; dec r5 then sbc (subtract the carry flag from) r4 to create a 32 bit number
dly_lp      dec     r5
            sbc     r4
            jnz     dly_lp

            ret

; Read Data from MAX31855
;
; Reads the SPI data from the MAX31855 chip to get the thermocouple data
; max 5 MHz clock frequency; 1 cycle here is 62.5 ns
; The instructions are laid out such that the timing of the clock up and down should be sufficient
; The fact that it's not perfectly equal should be fine. This actually gives more time to the
; data valid from the clock going low.
;
; INPUTS:
;   None
;
; OUTPUTS:
;   All 32 bits of the reading from the chip, including the diagnostic and fault information
;   r4 contains the higher order bits of the output
;   r5 contains the lower order bits
;   see MAX31855 documentation to see what the bits mean
;   https://datasheets.maximintegrated.com/en/ds/MAX31855.pdf
;
; CLOBBERS:
;   r11, r12, r13, r14, r15
;
fn_readtemp
            ;; Preparation section
            clr      r14                    ; clear high 16 bits for data
            clr      r15                    ; clear low 16 bits for data

            and      #~SPICS, &P1OUT        ; Pull chip select low to signal we are about to read

            mov      #SPIMISO, r12          ; value to compare with read data
            mov      #32, r11               ; r11 is the loop counter

            ;; Main SPI reading loop
maxloop     or       #SPICLK, &P1OUT        ; SCLK high

            mov      &P1IN, r13             ; Copy the P1IN register to a temp register
            and      #SPIMISO, r13          ; mask off port 1 pin 7

            and      #~SPICLK, &P1OUT       ; SCLK low

            bit      r12, r13               ; compare pin 7 high to pin 7 read value, carry flag is set if they are equal
            rlc      r15                    ; shift r15 left one, putting the high bit in the carry flag and pulling in the carry bit
            rlc      r14                    ; shift r14 left one, inserting the carry flag as the low bit

            dec      r11                    ; Decrement loop var
            jnz      maxloop                ; loop if loop var hasn't reached 0

            ;; Cleanup and return
            or       #SPICS, &P1OUT         ; Done reading, set chip select high again

            mov      r14, r4                ; return high bits in r4
            mov      r15, r5                ; and low bits in r5

            ret

; Calculates the duty cycle for the next time interval that the heating element should be on
;
; This implements a rudimentary PID controller. It is expected that the previous 64 deltas
; are all stored in the first 64 bytes of RAM.
;
; INPUTS:
;   Current delta between the observed and target temperature in r4
;
; OUTPUTS:
;   On duty cycle delay in r4/r5
;   Off duty cycle delay in r6/r7
;
; Clobbers:
;   r4, r5, r6, r7 for return values
;   r11, r12, r13, r14, r15 for scratch
fn_calcduty
            cmp     #0, r4                  ; Check if we're over temp
            jge     zero_pct_duty           ;

            ;; Calculate the proportional component (log base 2)
            call    fn_log_2                ; get log
            mov     r4, r11                 ; r11 is the total of the logs

            ;; Calculate the integral from the last 64 readings
            mov     #0, r6                  ; Integral accumulator
            mov     #OBSSTART, r4           ; Start at the base of RAM

sum_integ
            mov.b   0(r4), r5               ; Read byte from address in r4
            sxt     r5                      ; Make sure things add properly if negative
            add     r5, r6                  ; Add to accumulator.
            inc     r4                      ; increment loop variable / Increment address one byte.
            cmp     #OBSEND, r4             ;
            jl      sum_integ               ; loop until all 64 deltas have been accumulated

            ;; check if integral is negative, meaning we've been over temp overall
            cmp     #0, r6                  ;
            jge     zero_pct_duty           ; If so, zero time on for this period

            ;; Get the integtral and accumulate
            mov     r6, r4                  ; move integral to input for log func
            call    fn_log_2                ;

            add     r4, r11                 ; r11 now has the sum of the log2 of the proportion and integral

            ;; Shift the gold value (total period delay) by the sum of the logarithms
            ; First, to get the on period, shift the gold value to the right r11 times.
            mov     #DLYGLDHI, r14          ; Set up the raw gold value
            mov     #DLYGLDLO, r15          ;
            mov     r11, r12                ; Set up the loop counter (r12)

shft_on
            rra     r14                     ; Shift the 32 bit value right (using the carry bit)
            rrc     r15                     ;
            dec     r12                     ; loop check. This is ok to do afterwards because we have checks above for 0% duty
            jne     shft_on                 ; Loop until r12 is 0

            mov     r14, r4                 ; Move return value into place
            mov     r15, r5                 ;

            ; Second, subtract the on delay from the gold value to get the off delay
            mov     #DLYGLDHI, r12          ; Move gold delay value into place
            mov     #DLYGLDLO, r13          ;

            sub     r14, r12                ; 32 bit subtract using carry
            subc    r15, r13                ;

            mov     r12, r6                 ; move off delay value into return registers
            mov     r13, r7                 ;

zero_pct_duty  ; If we're over temp, just keep it off
            mov     #0, r4                  ; 0 seconds on
            mov     #0, r5                  ;
            mov     #DLYGLDHI, r6           ; 10 seconds off
            mov     #DLYGLDLO, r7           ;
            ret

; Calculates the integer log (base 2) of the 16 bit int passed in
;
; INPUTS:
;   r4: The input int to get log base 2 of
;       The input is assumed to be positive (unsigned int)
;
; OUTPUTS:
;   r4: the log (base 2) of the int passed in
;
; CLOBBERS:
;   r5
;
fn_log_2
            mov     #0, r5                  ; Zero out counter

log_loop    cmp     #0, r4                  ; loop as long as we are > 0
            jz      log_end                 ;
            inc     r5                      ; increment log
            rra     r4                      ; shift number one right
            jmp     log_loop                ;

log_end     mov     r5, r4                  ; r4 is the return value
            ret


; calculate integral
; calculate proportional
; maybe use derivative
; weight all of these and create a final proportion
; use proportion to generate timing delays


; 3/8, 5/16, 1/8 holes

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack
            
;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET
            
