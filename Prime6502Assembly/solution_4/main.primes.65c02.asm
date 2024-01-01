.setting "OutputSaveInfoFile", true
.setting "OutputSaveIndividualSegments", false
.setting "HandleLongBranch", false
.setting "ShowLabelsAfterCompiling", true



; Define loading address
LOAD  .equ $7000

;Insert starting address when saving as PRG (for xmodem upload)
prg   .org LOAD-2
    .word LOAD


.include "primes.65c02.asm"
;.include "uart_screen.65c02.asm"








