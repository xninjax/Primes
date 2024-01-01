; Sieve of Eratosthenes implementation for Ben Eater's breadboard 65c02 computer
;
; Peculiarities:
; * The sieve size is set to 250,000: there's not enough memory in the computer for 1,000,000
; * The sieve size needs to be divisable by 16
; * The square root of the sieve size needs to be divisible by 8, or else rounded up to the nearest multiple of 8


;====================================================
; Main symbols
;====================================================

UART_CS     .equ $8080
UART_DAT    .equ $8000

SIEVE_SIZE = 262144 ;250000
SIEVE_SQRT = 512 ;504                                ; Actual square root is 500, rounded up to multiple of 8

; expected prime count is 22044 = $561c
CNT_LOW =  $D8 ;$1c
CNT_HIGH = $59 ;$56

BUF_BITS = SIEVE_SIZE / 2
BUF_byteES = BUF_BITS / 8
BUF_LAST = BUF_byteES - 1
SQRT_byteES = SIEVE_SQRT / 8

buffer .equ $200
buffer_end = buffer + BUF_LAST

;====================================================
; I/O defines
;====================================================

DISPLAY_WIDTH = 80  ; 16 update for uart display of 80 column

;====================================================
; Zero page variables
;====================================================

;    .dsect
.data
.org $0000              ; .dsect defaults to this, just adding it for clarity

counter .word ?              ; General counter variable.
fctr_bytee .byte ?            ; Factor bytee component. Initialized to 00 at startup.
fctr_bitcnt .byte ?          ; Factor bit component. Initialized to 03 at startup.
curptr .word ?               ; Address part of current prime candidate pointer. Initialized to 0000 at startup.
curptr_bit .byte ?           ; Bit part of current prime candidate pointer. Initialized to 01 at startup.
cndptr .word ?               ; Address part of last found unset prime candidate pointer.
cndptr_bit .byte ?           ; Bit part of last found unset prime candidate pointer.
lcd_line .byte ?             ; LCD line we're printing to, either 0 or 1. Initialized to 00 at startup.
lcd_charspace .byte ?        ; Number of characters left on current LCD line. Initialized to 17 at startup.
count_string .byte ?,?,?,?,?,?,?        ; Memory space for string with number of primes found. It's 6 bytees, because the 
                            ;   largest value a 16-bit word can hold is 65535. The 6th bytee in the string is for
                            ;   the terminating zero. First bytee is initialized to 00 at startup.
remainder .byte ?             ; Variable for long division remainder.
stringptr .word ?           ; Pointer to current string to be printed.




;====================================================
; Main sieve buffer
;====================================================
// .segment "working_buffer"
// ;.org $200
// ;.storage BUF_LAST


// buffer = $200
// buffer_end = buffer + BUF_LAST

;====================================================
; Program - ROM starts here
;====================================================
.code
;    .org $8000
    .org $7000		; end of RAM

reset:

    ; Set up stack pointer
    ldx #$ff
    txs

    ; Initialize variables
    lda #00
    sta fctr_bytee
    sta curptr
    sta curptr + 1
    sta lcd_line
    sta count_string

    lda #03
    sta fctr_bitcnt

    lda #01
    sta curptr_bit

    lda #DISPLAY_WIDTH + 1
    sta lcd_charspace


	uartScreenClear()			; using uart_screen.65c02.asm

    ; Set display line to the first one.
    lda #0
    sta lcd_line
    line_start()

    ; Print "Processing..." on first line
    lda #<processing_label
    sta stringptr
    lda #>processing_label
    sta stringptr + 1

    print_string()

    ; Set display line to the second one. We'll show progress there.
    lda #1
    sta lcd_line
    line_start()
    
    ; Show that we're clearing memory
    lda #"-"
    print_char()

    ; Initialize (zero) buffer
    jsr init_buf

    ;
    ; Main logic
    ;

    ; Set pointer to prime sieve buffer
    lda #<buffer
    sta curptr
    lda #>buffer
    sta curptr + 1

cnd_loop:
    lda (curptr)            ; Load bytee curptr points to

    bit curptr_bit          ; Check if bit at pointer is set 
    bne unset_fctrs        ; Is it? Unclear multiples then!

    ; It wasn't, so we increase factor and move pointer
    jsr inc_ptrbit
    jsr inc_fctr            ; This also loads the factor bytee number into X

    ; We can stop looking for factor if we've reached the square root of the sieve size
    cpx #SQRT_byteES
    bcc cnd_loop
    jmp validate

; We found a factor, let's clear multiples!
unset_fctrs:

    ; Save pointer, so we can resume factor searching later
    lda curptr
    sta cndptr
    lda curptr + 1
    sta cndptr + 1
    lda curptr_bit
    sta cndptr_bit

    ; Show that we found a factor
    lda #"."
    print_char()

    ; Clear multiples in buffer
    jsr clear_multiples

    ; We're done clearing multiples of our current factor, so increase it
    jsr inc_fctr            ; This also loads the factor bytee number into X

    ; We can stop if we've reached the square root of the sieve size
    cpx #SQRT_byteES  
    bcc next_fctr
    jmp validate

next_fctr:

    ; Restore pointer and search for next factor
    lda cndptr
    sta curptr
    lda cndptr + 1
    sta curptr + 1
    lda cndptr_bit
    sta curptr_bit

    jsr inc_ptrbit
    jmp cnd_loop

; We're done!
validate:

    ; Set counter to 1 (2 is also prime but not in the bitmap)
    lda #1
    sta counter
    lda #0
    sta counter + 1

    ldy #0

    ; Show that we're counting
    lda #"+"
    print_char()

    ; Count bits in buffer
    jsr cnt_buf


	uartScreenClear()

    ; We write the "result valid" line first, but at the bottom line of the display.
    ;   Why? Because I think it's more logical that way :)
    lda #1
    sta lcd_line
    line_start()

    ; Print valid label
    lda #<valid_label
    sta stringptr
    lda #>valid_label
    sta stringptr + 1

    print_string()

    ; Check the number of primes we found
    lda counter
    cmp #CNT_LOW
    bne set_invalid
    lda counter + 1
    cmp #CNT_HIGH
    bne set_invalid

    ; Prime count is valid
    lda #<yes_value
    sta stringptr
    lda #>yes_value
    sta stringptr + 1
    
    jmp write_validflag

; Prime count is invalid
set_invalid:
    lda #<no_value
    sta stringptr
    lda #>no_value
    sta stringptr + 1

write_validflag:
    print_string()

    ; Move to top line of the display
    lda #0
    sta lcd_line
    line_start()

    ; Print time label
    lda #<count_label
    sta stringptr
    lda #>count_label
    sta stringptr + 1

    print_string()

    ; Convert count value to decimal string
    jsr count_to_string

    ; Print the primes count
    lda #<count_string
    sta stringptr
    lda #>count_string
    sta stringptr + 1

    print_string()

; That's it! Time for our "active halt"
    jmp uartBeep
finish:
    jmp finish

;====================================================
; Initialize the sieve buffer by setting it to all 0s.
;
; Uses: A, X, Y, counter, curptr
;====================================================
init_buf:

    ; Set pointer to buffer address
    lda #<buffer
    sta curptr
    lda #>buffer
    sta curptr + 1

    lda #<BUF_LAST
    sta counter
    lda #>BUF_LAST
    sta counter + 1

    ; Set up registers
    lda #$ff
    ldx #0
    ldy #0

init_loop:
    sta (curptr), y

    ; Check if we've counted down to 0
    cpx counter
    bne init_deccntr
    cpx counter + 1
    beq init_buf_end

; Decrease counter
init_deccntr:
    dec counter
    cmp counter             ; Use the fact that A contains $ff
    bne init_next
    dec counter + 1

; move on to next bytee
init_next:
    iny
    bne init_loop           ; If Y has wrapped, increase high bytee of pointer
    inc curptr + 1
    jmp init_loop

init_buf_end:
    rts

;====================================================
; Make bit pointer point to the next bit in the sieve buffer.
;
; In: 
;   curptr and curptr_bit: pointer to be increased
; Out: 
;   curptr and curptr_bit: updated to point to tne next sieve bit
;====================================================
inc_ptrbit:
    asl curptr_bit          ; Shift bit part of pointer to the left
    bcc inc_ptrbit_end     ; If we didn't shift it into the Carry flag, we're done

    ; Time to move on to the next bytee
    rol curptr_bit          ; First, rotate the bit out of Carry into bit 0 of the pointer

    inc curptr              ; Increase pointer low bytee
    bne inc_ptrbit_end      ; If we didn't roll over to 0, we're done

    inc curptr + 1          ; Increase high bytee of pointer

inc_ptrbit_end:
    rts

;====================================================
; Add 2 to factor.
;
; In: 
;   fctr_bytee and fctr_bitcnt: factor value to be increased
; Out: 
;   X: updated value of fctr_bytee
;   fctr_bytee and fctr_bitcnt: updated to next factor
; Uses: A
;====================================================
inc_fctr:
    ; Increase factor bit value twice
    inc fctr_bitcnt
    inc fctr_bitcnt

    ; Check if we moved into the next bytee
    lda fctr_bitcnt
    cmp #8
    bcc ld_fctrroot        ; Value <8? We're done!
    sbc #8                  ; Subtract 8 from bit value
    sta fctr_bitcnt         ;   and save it

    inc fctr_bytee           ; Add 1 to factor bytee value

ld_fctrroot:
    ldx fctr_bytee           ; Load factor bytee into X
    rts

;====================================================
; Clear multiples of factor in the sieve buffer. This routine is not 
; optimal from an arithmatic perspective, in the sense that it clears
; all odd multiples of the factor. The most efficient approach would
; be to start at factor * factor. However, with the 65c02 not having a 
; multiply instruction, computing that would itself be almost as much work
; as just starting from the beginning. As such, saving the few actual 
; "clear bit" instructions at runtime would almost double the code of this 
; routine.
;
; In: 
;   curptr and curptr_bit: pointer to the factor bit in the sieve. The
;                          clearing will start at the next (i.e. first) 
;                          multiple from a sieve bit perspective.
;   fctr_bytee and fctr_bitcnt: factor of which multiples must be cleared
; Uses: A, X, curptr and curptr_bit
;====================================================
clear_multiples:
    ; Check if the bit part of the factor is 0. If so, 
    ;   skip adding it to the prime candidate pointer
    ldx fctr_bitcnt
    beq fctr_addbytee

; Increase pointer with factor, bit part first
fctr_decloop:
    jsr inc_ptrbit
    dex
    bne fctr_decloop

; Now add factor bytee to pointer
fctr_addbytee:
    lda curptr
    clc
    adc fctr_bytee
    sta curptr

    ; If we rolled over the pointer low bytee, increase the high one
    bcc fctr_chkend
    inc curptr + 1

fctr_chkend:
    ; See if we've reached the end of our buffer. Start with the high bytee
    ;   of the pointer.
    lda curptr + 1
    cmp #>buffer_end
    bcc unset_curbit
    bne unset_buf_end

    lda curptr
    cmp #<buffer_end
    bcc unset_curbit
    bne unset_buf_end

unset_curbit:
    jsr clear_ptrbit        ; Clear the bit under the pointer

    ; Clear next multiple
    jmp clear_multiples

unset_buf_end:
    rts

;====================================================
; Clear the bit in the sieve buffer that the relevant pointer points to.
;
; In:
;   curptr and curptr_bit: pointer to sieve bit to be cleared
; Uses: A
;====================================================
clear_ptrbit:
    ; Load and invert bit part of pointer, to create a bit mask for the next step
    lda curptr_bit
    eor #$ff

    ; Clear the correct bit in the bytee at curptr by ANDing the mask with it
    and (curptr)
    sta (curptr)

    rts

;====================================================
; Count the bits that are set in the sieve buffer.
;
; Out:
;   counter: number of set bits found
; Uses: A, curptr
;====================================================
cnt_buf:
    ; Make curptr point to start of buffer
    lda #<buffer
    sta curptr
    lda #>buffer
    sta curptr + 1

cnt_byteeloop
    ; Load bytee, and move to next one if 0
    lda (curptr)
    beq cnt_chkend

cnt_bitloop:
    ; We rotate the leftmost bit into the Carry flag, so we can
    ;   use bcc to check if it is set or not.
    asl
    bcc cnt_chkempty       ; If Carry is clear, there's nothing to see here

    ; We found a set bit, so increase our counter
    inc counter
    bne cnt_chkempty       ; If the low bytee in counter has rolled over to 0,
    inc counter + 1         ;   then also increase its high bytee.

; If A is now zero, we're done with this bytee
cnt_chkempty:
    cmp #0
    beq cnt_chkend
    jmp cnt_bitloop        ; More bits left in this bytee, so keep looking

; check if we're at the last buffer bytee
cnt_chkend:
    lda #<buffer_end
    cmp curptr
    bne cnt_next

    lda #>buffer_end
    cmp curptr + 1
    bne cnt_next

    ; We're done
    rts

; Move on to the next bytee
cnt_next:
    inc curptr
    bne cnt_byteeloop       ; If the low bytee in curptr has rolled over to 0,
    inc curptr + 1          ;   then also increase its high bytee.
    jmp cnt_byteeloop

;====================================================
; Convert the 16-bit value in counter to a decimal string.
; The approach used here is explained in one of the Ben Eater YouTube episodes 
; about the 65c02 breadboard computer.
;
; Note that the value in counter is cleared by this routine!
;
; Out:
;   count_string: zero-terminated decimal value previously held in counter
; Uses: A, X, Y, counter, remainder
;====================================================
count_to_string:
    ; Set remainder to 0
    lda #0
    sta remainder

    ldx #16                 ; Bit width of the word

    clc                     ; Also clear Carry flag

div_loop:
    ; Rotate counter and remainder left
    rol counter
    rol counter + 1
    rol remainder

    ; Try to substract divisor
    sec                     ; Set Carry flag before we substract
    lda remainder
    sbc #10                 ; Take 10 off
    bcc ignore_result      ; If remainder < 10, we overshot

    ; Store current remainder
    sta remainder

ignore_result:
    ; Continue if there are bits left to shift
    dex
    bne div_loop

    ; Rotate last bit into quotient
    rol counter
    rol counter + 1

    ; Current remainder is counter digit value
    lda remainder
    clc                     ; Clear Carry flag before we add
    adc #"0"                ; A now holds the digit we want to insert into the string

    ldy #0                  ; Prepare for indexed addressing

; Add counter digit to beginning of count_string. We do this by shifting all
;   existing digits to the right. We use X to temporarily hold the characters we
;   need to push one place to the right.
push_loop:
    ldx count_string, y     ; X = "old" character at current string position
    sta count_string, y     ; Write the digit we want to put at current position
    iny                     ; Move to next character in the string
    txa                     ; A = "old" character we now want to put there

    bne push_loop          ; Stop when we're about to write the terminating null...

    ; ...but don't forget to write it!
    sta count_string, y

    ; We're done dividing when counter is 0.
    lda counter
    ora counter + 1
    bne count_to_string     ; Not 0 yet? Continue dividing!

    rts

.function uartScreenClear()
    ;PHA
    LDA #<hCLRSCN
	STA stringptr
    LDA #>hCLRSCN
    STA stringptr+1
	print_string()
    ;PLA
.endfunction


.function clear_line():
    line_start()          ; Move to beginning of current line
    PHX
    PHA
    ldx #DISPLAY_WIDTH      ; Number of spaces to put

space_loop:
    lda #" "
    jsr send_char

    dex
;    bne space_loop
	
	line_start()
	PLA
    PLX
.endfunction



.function line_start()
    PHA             ; save A
    ; Reset char space counter, as we'll be at the beginning of the line
	; likely not needed here for uart display...
    lda #DISPLAY_WIDTH + 1
    sta lcd_charspace

	LDA lcd_line
	
	BEQ home0

home1:
    LDA #<hHOME1 
	STA stringptr
    LDA #>hHOME1
	STA stringptr+1
	JMP send_string
home0:
    LDA #<hHOME0 
	STA stringptr
    LDA #>hHOME0
	STA stringptr+1
send_string:
    print_string()
	
    PLA             ; restore A
.endfunction


.function print_string()
; Gets address of string from A (low) and Y (high)
; prints string using send_char function
; strings must be null terminated (end with $00)
;
    PHY                     ; save Y
    ;PHA
    ; STA CMD_BUFF
    ; STY CMD_BUFF+1
    
    LDY  #$00
getChar:
    LDA  (stringptr),Y       ; CMD_BUFF points to string location
    BEQ  NoMoreChar

    send_char()

    INY
    JMP  getChar

NoMoreChar:

    ;PLA
    PLY                     ; restore Y

.endfunction

.function print_char()
    dec lcd_charspace       ; Decrease "space left counter"
    bne send_char           ; Not at zero yet? We have room left!

    ; If we're here, we filled the LCD line. We'll clear it and start at the beginning

    pha                     ; Save A
    jsr clear_line          ; Clear current line
    pla                     ; Retrieve A
.endfunction



.function send_char()
    PHA
@WaitForUart:
    LDA  UART_CS
    AND  #$01
    BEQ  @WaitForUart
    PLA
    STA  UART_DAT
.endfunction

.function uartBeep()
    PHA
@WaitForUart:
    LDA  UART_CS
    AND  #$01
    BEQ  @WaitForUart

    LDA #$7
    STA UART_DAT
    PLA
    
.endfunction

;====================================================
; String constants
;====================================================

hCLRSCN     .byte $1b, "[2J", $00        ;ESC[ 2J    ; Clear screen

hHOME0      .byte $1b, "[1;1H", $00    ; Used for debug locations at bottom of screen 
hHOME1      .byte $1b, "[2;1H", $00    ; Used for debug locations at bottom of screen 
// hHOME2      .byte $1b, "[1;10H", $00   ; Used for debug locations at bottom of screen 
// hHOME3      .byte $1b, "[2;10H", $00   ; Used for debug locations at bottom of screen 
// hHOME4      .byte $1b, "[1;20H", $00   ; Used for debug locations at bottom of screen 
// hHOME5      .byte $1b, "[2;20H", $00   ; Used for debug locations at bottom of screen 


;----------------------------------------
; hHOME0    hHOME2      hHOME4          [row 1]
; hHOME1    hHOME3      hHOME5          [row 2]
;----------------------------------------

processing_label   .byte "Processing...", $00
count_label        .byte "Count: ", $00
valid_label        .byte "Valid: ", $00
yes_value          .byte "yes", $00
no_value           .byte "no", $00


