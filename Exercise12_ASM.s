		TTL Exercise 11 PWM and LEDs
;****************************************************************
;Assembly and C mixed programming to control the Red LED 
; on the KL05Z board.
;Name:  Yu Tan
;Date:  11/07/2022
;Class:  CMPE-250
;Section:  04 Thursday 2 pm 
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1             ;Turn on listing
;****************************************************************
;EQUates

PWM_FREQ          EQU  50
;TPM_SOURCE_FREQ  EQU  48000000
TPM_SOURCE_FREQ   EQU  47972352
TPM_SC_PS_VAL     EQU  4
;PWM_PERIOD       EQU  ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_VAL)) /
;                       PWM_FREQ)
;PWM_DUTY_5       EQU  (PWM_PERIOD / 20)  ;  5% duty cycle
;PWM_DUTY_10      EQU  (PWM_PERIOD / 10)  ; 10% duty cycle
PWM_PERIOD_20ms   EQU  60000
PWM_DUTY_10       EQU  6500
PWM_DUTY_5        EQU  2700
	
;PWM_FREQ	 			EQU 60
;TPM_SOURCE_FREQ			EQU 47972352
;TPM_SC_PS_VAL 			EQU 4
;PWM_PERIOD				EQU ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_VAL)) / PWM_FREQ)
PWM_DUTY_MAX			EQU (PWM_PERIOD_20ms-1)	;100% duty cycle
	
;DAC0
DAC0_BITS   EQU   12
DAC0_STEPS  EQU   4096
DAC0_0V     EQU   0x00
	
;Servo
SERVO_POSITIONS  EQU  5	
	
;Number output characteristics
MAX_WORD_DECIMAL_DIGITS  EQU  10
	
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
FiveHundred EQU   50
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue contents
Q_REC_SZ    EQU   18 ;Queue management record
RXTX_BUF    EQU   80
RXTX_REC    EQU   18
	;EQUates
;Characters
MAX_STRING  EQU  79
CR          EQU  0x0D
LF          EQU  0x0A
BS    		EQU  0x08 
NULL        EQU  0x00
;---------------------------------------------------------------
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;--UART0--------------------
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SCGC6
;1->23:PIT clock gate control (enabled)
;Use provided SIM_SCGC6_PIT_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------

;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
      	
			;Exports and imports to interface with c code
			EXPORT PWM_duty_table_0
			EXPORT DAC0_table_0
			EXPORT PutNumUB
			EXPORT PutNumU
			EXPORT GetStringSB
			EXPORT PutStringSB
			EXPORT Init_UART0_IRQ

			EXPORT GetChar
			EXPORT PutChar
			EXPORT PutNumHex 
				
			EXPORT Init_UART0_IRQ


				
;>>>>> begin subroutine code <<<<<
;-------------------------------------------
;>>>>> begin subroutine code <<<<<
Init_UART0_IRQ  PROC {R0-R14}
; Select/Configure UART0 Sources
; Enable clocks for UART0 and Port B
; Select port B mux pins to connect to UART0 
; ConfigureS UART0
            PUSH  {R0-R7,LR}         		
			LDR 	R1,=RxQRecord
			LDR 	R0,=RxQBuffer
			MOVS 	R2, #80
			BL		InitQueue			 
			 ;Initialize TxQBuffer
			LDR 	R1,=TxQRecord
			LDR 	R0,=TxQBuffer
			MOVS 	R2,#80
			BL	    InitQueue
			LDR   R0,=SIM_SOPT2            ;connect Sources
            LDR   R1,=SIM_SOPT2_UART0SRC_MASK
            LDR   R2,[R0,#0]               ;current SIM_SOPT2 value
            BICS  R2,R2,R1                 ;bits cleared of UART0SRC
            LDR   R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK  
            ORRS  R2,R2,R1                 ;UART0 bits changed
            STR   R2,[R0,#0]               ;update SIM_SOPT2
			  
			LDR   R0,=SIM_SOPT5            ;set SIM_SOPT5 for UART0 external
            LDR   R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
            LDR   R2,[R0,#0]
            BICS  R2,R2,R1
            STR   R2,[R0,#0]
			  
			LDR   R0,=SIM_SCGC4          ;enable SIM_SCGC4 as clock for UART0 Module
			LDR   R1,=SIM_SCGC4_UART0_MASK 
            LDR   R2,[R0,#0]
			ORRS  R2,R2,R1
			STR   R2,[R0,#0]
			   
			LDR   R0,=SIM_SCGC5          ;enable clock for Port B module
            LDR   R1,= SIM_SCGC5_PORTB_MASK
            LDR   R2,[R0,#0]
            ORRS  R2,R2,R1
            STR   R2,[R0,#0]
			 
			LDR   R0,=PORTB_PCR2         ;connect Port B pin 1 to UART0 Rx
            LDR   R1,=PORT_PCR_SET_PTB2_UART0_RX
            STR   R1,[R0,#0]
			 
			LDR   R0,=PORTB_PCR1         ;connect Port B pin 2 to UART0 Tx
            LDR   R1,=PORT_PCR_SET_PTB1_UART0_TX
            STR   R1,[R0,#0]
			 
			 
			LDR   R0,=UART0_BASE         ;load base address
			 
            MOVS  R1,#UART0_C2_T_R       ;Diasble UART0
            LDRB  R2,[R0,#UART0_C2_OFFSET]
            BICS  R2,R2,R1
            STRB  R2,[R0,#UART0_C2_OFFSET]
			
			LDR   R0,=UART0_IPR
			LDR   R2,=NVIC_IPR_UART0_PRI_3
			LDR   R3,[R0,#0]
			ORRS  R3,R3,R2
			STR   R3,[R0,#0]
			LDR   R0,=NVIC_ICPR
	 	    LDR   R1,=NVIC_ICPR_UART0_MASK
		    STR   R1,[R0, #0]		     
            LDR   R0,=NVIC_ISER
		    LDR   R1,=NVIC_ISER_UART0_MASK
		    STR   R1,[R0, #0]
			
			LDR   R0,=UART0_BASE
			MOVS  R1,#UART0_BDH_9600     ;set UART0 baud rate
            STRB  R1,[R0,#UART0_BDH_OFFSET]
            MOVS  R1,#UART0_BDL_9600
            STRB  R1,[R0,#UART0_BDL_OFFSET]
			 
            MOVS  R1,#UART0_C1_8N1        ;set UART0 8 bit serial stream 
            STRB  R1,[R0,#UART0_C1_OFFSET]
            MOVS  R1,#UART0_C3_NO_TXINV
            STRB  R1,[R0,#UART0_C3_OFFSET]
            MOVS  R1,#UART0_C4_NO_MATCH_OSR_16
            STRB  R1,[R0,#UART0_C4_OFFSET]
            MOVS  R1,#UART0_C5_NO_DMA_SSR_SYNC
            STRB  R1,[R0,#UART0_C5_OFFSET]
            MOVS  R1,#UART0_S1_CLEAR_FLAGS
            STRB  R1,[R0,#UART0_S1_OFFSET]
            MOVS  R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
            STRB  R1,[R0,#UART0_S2_OFFSET] 
			 
			MOVS  R1,#UART0_C2_T_RI        ;Enable UART0
            STRB  R1,[R0,#UART0_C2_OFFSET]
			 
			POP   {R0-R7,PC}              ;Register retention 
			 
		                        ;exit subroutine
            ENDP   
;------------------------------------------------				
InitQueue	PROC  {R0-R14}
			PUSH  {R0-R2}
			STR   R0,[R1,#IN_PTR]
			STR   R0,[R1,#OUT_PTR]
			STR   R0,[R1,#BUF_STRT]
			ADDS  R0,R0,R2
			STR   R0,[R1,#BUF_PAST]
			STRB  R2,[R1,#BUF_SIZE]
			MOVS  R0,#0
			STRB  R0,[R1,#NUM_ENQD]
			POP   {R0-R2}
			BX    LR
			ENDP
;-----------------------------------------------------
Enqueue     PROC  {R0-R14}
			PUSH  {R2-R7,LR}   ;Temporary allows changes to register 1-6, and LR
			LDRB  R2,[R1,#NUM_ENQD]  ;Assigns Num_ENQD to register 2.
			LDRB  R3,[R1,#BUF_SIZE]   ;Assigns BUF_Size to register 3.
			LDR   R4,[R1,#IN_PTR]    ;Assigns INT_PTR to register 4.
			LDR   R5,[R1,#BUF_PAST]  ;Assigns BUF_PAST to register 5.
			LDR   R6,[R1,#BUF_STRT] ;Assigns BUF_STRT to register 6.
			CMP   R2,R3
			BGE   QFull
			STRB  R0,[R4,#0]
			LDRB  R2,[R1,#NUM_ENQD]
			ADDS  R2,R2,#1
			STRB  R2,[R1,#NUM_ENQD]  ;Stores the new number enqueued to the array.
			ADDS  R4,R4,#1   ;Increments In_PTR by 1 to move the pointer to the next memory address
			STR   R4,[R1,#IN_PTR] ;Stores the new IN_PTR value in R1.
			CMP   R4,R5
			BGE   GE
			MRS   R2,APSR
			LDR   R7,=0x20000000
			BICS  R2,R2,R7
			MSR   APSR,R2
			B     in
GE  
			LDR   R2,[R1,#BUF_STRT]
			STR   R2,[R1,#IN_PTR]

			MRS   R2,APSR
			LDR   R7,=0x20000000
			LSLS  R2,R2,R7
			BICS  R2,R2,R7
			MSR   APSR,R2

			B     in
			
QFull
		
            MRS   R2,APSR ;Sets the C bit, changing the bit value to one.
            LDR   R7,=0x20000000
			LSLS  R2,R2,R7
            ORRS  R2,R2,R7
            MSR   APSR,R2
            B     in 
			

in
			POP   {R2-R7,PC}
			;BX    LR
			ENDP
;--------------------------------------------------			

Dequeue		PROC	{R1-R14}
			PUSH	{R1-R7, LR}
			LDRB    R2,[R1,#NUM_ENQD]	
			CMP		R2,#0				
			BEQ		DequeueSet
			LDR		R3,[R1,#OUT_PTR]	
			LDRB	R0,[R3,#0]
			SUBS	R2,R2,#1            
			ADDS	R3,R3,#1            
			LDR		R4,[R1,#BUF_PAST]   
			STRB	R2,[R1,#NUM_ENQD]   
			STR		R3,[R1,#OUT_PTR]    
			CMP		R3,R4               
			BLO		DequeueClear                     
			LDR		R3,[R1,#BUF_STRT]  
			STR		R3,[R1,#OUT_PTR]	
			B 		DequeueClear  
				
DequeueClear		
			MRS 	R6, APSR		    
			MOVS 	R7,#0X20		    
			LSLS 	R7,R7,#24
			BICS 	R6,R6,R7
			MSR 	APSR, R6
			B 		DequeueFinished
		
DequeueSet
			MRS 	R6,APSR		    	;Sets the flag on return
			MOVS 	R7,#0x20		    ;Sets the C flag to a 1
			LSLS 	R7,R7,#24
			ORRS 	R6,R6,R7
			MSR 	APSR, R6
			B		DequeueFinished
DequeueFinished		
			POP		{R1-R7, PC}
			ENDP

;----------------------------------------------------
PutNumUB	PROC  {R0-R14} 
			PUSH  {R1,LR}  ;Allow changes to Register 1.
			MOVS  R1,#0xF  ;Change value to hex.
			ANDS  R0,R0,R1 ;ANDS R0 and R1 to take the last few characters of R0.
			BL    PutNumU ;Displays total number in U.
			POP   {R1,PC}
			BX    LR
			ENDP
;----------------------------------------------------
PutNumHex   PROC  {R0-R14}
			PUSH  {R2-R7,LR}
			MOVS  R3,R0   ;Uses Register 3 as a temporary register to hold R0.
			MOVS  R2,#28  ;Moves the immediate value of 28 to Register 2
			
Loopie      
			MOVS  R0,R3
			LDR   R7,=0x0000000F
			LSLS  R7,R7,R2
			ANDS  R0,R0,R7
			LSRS  R0,R0,R2
			CMP   R0,#9
			BLE   HexValue
			ADDS  R0,R0,#55
			BAL   Print
HexValue	
			ADDS  R0,R0,#48
Print     
			BL    PutChar
			SUBS  R2,R2,#4
			BLO   HexDone
			BAL   Loopie
HexDone
			POP   {R2-R7,PC}
			BX    LR
			ENDP
				
;-------------------------------------------
UART0_ISR   PROC  {R4-R13}
	        CPSID I
			PUSH  {LR}
			LDR   R0,=UART0_BASE
			LDRB  R1,[R0,#UART0_C2_OFFSET]
			MOVS  R2,#UART0_C2_TI_RI
			TST   R1,R2
			BEQ    RX_INTERRUPTENABLED
TX_INTERRUPTENABLED
			LDRB  R1,[R0,#UART0_S1_OFFSET]
			MOVS  R2,#UART0_S1_TDRE_MASK
			TST   R1,R2
			BEQ   RX_INTERRUPTENABLED
			LDR   R1,=TxQRecord
			BL    Dequeue
			BCS   DisableTx
			LDR   R1,=UART0_BASE
			STRB  R0,[R1,#UART0_D_OFFSET]
			B     RX_INTERRUPTENABLED	
DisableTx	
			LDR   R0, =UART0_BASE
			MOVS  R1,#UART0_C2_T_RI
			STRB  R1,[R0,#UART0_C2_OFFSET]
RX_INTERRUPTENABLED			
			LDR   R0,=UART0_BASE
			LDRB  R1,[R0,#UART0_S1_OFFSET]
			MOVS  R2,#UART0_S1_RDRF_MASK
			TST   R1,R2
			BEQ   ISR_FINISHED
			LDR   R3,=UART0_BASE
			LDRB  R0,[R3,#UART0_D_OFFSET]
			LDR   R1,=RxQRecord
			BL    Enqueue
ISR_FINISHED
			CPSIE I
			POP   {PC}
			ENDP
;------------------------------------------
				
GetChar		PROC  {R1-R14}
	;Output : R0 : Character Input
	;Repeat
	;{check RDRF bit of UART0_S1
	;   }until (RDRF= 1)
			PUSH  {R1-R3,LR}
			;Poll RDRF until UART0 ready to receive
			CPSID I
			LDR   R1,=RxQRecord
			

Repeat1		
			
		    BL    Dequeue
			
			CPSIE I
			;Receive character and store in Ri
			BCS   Repeat1
			POP   {R1-R3,PC}
			
			
			ENDP
;---------------------------------------------			 
PutChar		PROC  {R0-R14}
	;/*Input: R0: Character to transmit */
	; repeat{
	;    check TDRE bit of UART0_S1
	; } until(TDRE == 1)
			PUSH  {R0-R3,LR}
			LDR   R1,=TxQRecord ;Poll TDRE until UART0 ready to transmit
			
Repeat2		
			;LDR   R1,=TxQRecord ;Poll TDRE until UART0 ready to transmit
			CPSID I
			BL    Enqueue
			CPSIE I
			BCS   Repeat2
			LDR   R1,=UART0_BASE
			MOVS  R2,#UART0_C2_TI_RI
			STRB  R2,[R1,#UART0_C2_OFFSET]
			POP   {R0-R3,PC}
		
			ENDP			

;-----------------------------------------------
GetStringSB PROC  {R1-R14}
; Reads a string from the terminal keyboard,
; (i.e., characters typed until “Enter” is pressed),
; stores it in memory starting at the address
; where R0 points, and echoes it to the terminal screen.
; Parameters
; Input: R0: Pointer to destination string
; Modify: APSR
; Uses:
; GetChar, PutChar
			PUSH  {R0-R4,LR}
			MOVS  R2,#0  ;Sets array offset to 0
			MOVS  R3,R0   ;Saves original character input to a register
Input
			BL    GetChar   ;Asks the user for a input.
			CMP   R0,#CR    ;Checks for carriage return
			BEQ   E_GetStringSB   
			CMP   R1,#1      ;Checks to see if the input is almost at its MAX_STRING
			BEQ   Input 
			BL    PutChar		;Echoes the user input to terminal
			STRB  R0,[R3,R2]	;Stores the new output into temporary register holder.
			SUBS  R1,R1,#1		;Moves to the next memory address for the next input to read.
			ADDS  R2,R2,#1		;Increments the offset by 1
			B     Input

E_GetStringSB  
			MOVS R0,#0       ;Nulls the character input
			STRB R0,[R3,R2]   ;Stores the character input into temporary register and increments by the offset.
			
			POP  {R0-R4,PC}
			

			ENDP
				
				
PutStringSB PROC {R0-R13}	
; Displays a null-terminated string from memory,
; starting at the address where R0 points, to the
; terminal screen.
; Parameters
; Input: R0: Pointer to source string
; Modify: APSR
; Uses:
; PutChar 
			PUSH {R0-R2,LR}
			CMP  R1,#0					;Checks if all input are processed
			BEQ  E_PutStringSB          ;Ends subroutine
			ADDS R1,R1,R0				
			MOVS R2,R0					;Saves character input into temporary register
Load_Char	
			LDRB R0,[R2,#0]				;Stores the memory value of r2 at increment 0 to R0.
			CMP  R0,#0					;Checks to see if value is NULL
			BEQ  E_PutStringSB
			BL   PutChar                ;Prints character to terminal
			ADDS R2,R2,#1				;Inrease array offset by 1 
			CMP  R2,R1					;If offset not equal to buffer capacity, return to top of loop.
			BNE  Load_Char
			
E_PutStringSB
			POP  {R0-R2,PC}
			
			ENDP
;-------------------------------------------------------------------------				
DIVU     	PROC    {R2-R14}
            PUSH    {R2} ;Saves the original values in R2 and R3
            MOVS    R2,#0   ;Sets the quotient value to zero.
            CMP     R0,R2   ;Checks to see if the divider is equal to zero.
            BEQ     SetCarry ;Checks to see if the dividend is equal to zero. Branches if equal to zero.
			CMP     R1,#0
			BEQ     ClearCarry
Check       CMP     R1,R0	;Compares the dividend with the divisor
            BLO     ClearCarry ;Branches if the dividend is less than the divisor.
            ADDS    R2,R2,#1  ;Adds 1 to the quotient.
            SUBS    R1,R1,R0  ;Subtracts the dividend by the divisor
            B      Check	;Loops if the dividend is still greater than the divisor
            
            
ClearCarry  MOV     R0,R2    ;Moves 0 to the divisor to display as quotient.
            PUSH    {R0-R1} ;Temporarily allow changes to R0,R1
            MRS     R0,APSR ;Clears the C Bit, changing the bit value to zero.
            LDR     R1,=0x20000000
            BICS    R0,R0,R1
            MSR     APSR,R0
            POP     {R0-R1} ;Returns the original Value of the registers in R0 and R1
            ;POP     {R2}   ;Returns the original value of the register in R2
            B       Quit
            
            
SetCarry    PUSH    {R0-R1} ;Temporarily allow changes to R0,R1
            MRS     R0,APSR ;Sets the C bit, changing the bit value to one.
            LDR     R1,=0x20000000
            ORRS    R0,R0,R1
            MSR     APSR,R0
            POP     {R0-R1} ;Returns the original Value of the registers in R0 and R1
            ;POP     {R2}  ;Returns the original value of the register in R2
            B       Quit 
Quit        POP		{R2}
			BX      LR   ;Operation for the expression is done.
			ENDP		

;---------------------------------------------------------------------------------------------

PutNumU		PROC    {R0-R14}
			PUSH    {R0-R2,LR}
			MOVS    R2,#0    ;Offset Value
			
DIVV
			CMP     R0,#10   ;Checks to see if value is less than 10. Can't divide or result is a fractional number.
			BLT     EDIV
			
			MOVS    R1,R0
			MOVS    R0,#10    ;sets the divisor to 10
			BL      DIVU
			PUSH    {R0}
			LDR     R0,=StringSize
			STRB    R1,[R0,R2]    ;stores the new value of the dividend into memory address
			ADDS    R2,R2,#1     ;increase offset by 1 
			POP     {R0}
			B       DIVV
EDIV
			ADDS    R0,R0,#'0'    ;converts to ASCII value
			BL      PutChar
			SUBS    R2,R2,#1

Final_Array_String
			
			LDR     R0,=StringSize
			CMP     R2,#0
			BLT     EDIVV
			LDRB    R1,[R0,R2]
			MOVS    R0,R1    ;moves the new dividend to r0
			
			ADDS    R0,R0,#'0'   ;converts to ASCII Character
			BL      PutChar      ;Prints out final quotient
			
			SUBS    R2,R2,#1     ;Prints out the previous value in reverse order until offset = 0.
			
			B       Final_Array_String
EDIVV
			POP     {R0-R2,PC}
			
			
			
			ENDP			
;-----------------------------------------------------------------
Init_PIT_IRQ PROC  {R0-R14}
			PUSH    {R0-R2,LR}
			LDR 	R0 ,=SIM_SCGC6
			LDR 	R1 ,=SIM_SCGC6_PIT_MASK
			LDR 	R2,[R0,#0]
			ORRS    R2,R2,R1
			STR     R2,[R0,#0]

			LDR 	R0, =PIT_CH0_BASE
			LDR 	R1, =PIT_TCTRL_TEN_MASK
			LDR 	R2, [R0, #PIT_TCTRL_OFFSET]
			BICS	R2, R2, R1
			STR 	R2, [R0, #PIT_TCTRL_OFFSET]

			LDR 	R0,=PIT_IPR
			LDR 	R1,=NVIC_IPR_PIT_MASK
;LDR 	R2,=NVIC_IPR_PIT_PR0_0
			LDR 	R3,[R0,#0]
			BICS 	R3,R3,R1
;ORRS 	R3,R3,R2
			STR	    R3,[R0,#0]

			LDR	    R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_PIT_MASK
			STR		R1,[R0,#0]
;Unmask PIT interrupts
			LDR 	R0,=NVIC_ISER
			LDR 	R1,=NVIC_ISER_PIT_MASK
			STR 	R1,[R0,#0]

			LDR 	R0,=PIT_BASE
			LDR 	R1,=PIT_MCR_EN_FRZ
			STR 	R1,[R0,#PIT_MCR_OFFSET]
;Set PIT timer 0 peR0od for 0.01 s
			LDR 	R0,=PIT_CH0_BASE
			LDR 	R1,=PIT_LDVAL_10ms
			STR 	R1,[R0,#PIT_LDVAL_OFFSET]
;Enable PIT timer 0 interrupt
			LDR 	R1,=PIT_TCTRL_CH_IE
			STR 	R1,[R0,#PIT_TCTRL_OFFSET]
			POP     {R0-R2,PC}
			ENDP
;-----------------------------------------------------			
			
PIT_ISR     PROC {R0-R14}
			PUSH      {R0-R3}
			LDR       R0,=RunStopWatch
			LDRB      R1,[R0,#0]
			CMP       R1,#0
			BNE       CounterInc
			B         EXIT
CounterInc
			
			LDR       R2,=Count
			LDR       R3,[R2,#0]
			ADDS      R3,R3,#1
			STR       R3,[R2,#0]
EXIT 		
			LDR        R0, =PIT_CH0_BASE
            LDR        R1, =PIT_TFLG_TIF_MASK
            STR        R1, [R0, #PIT_TFLG_OFFSET]
			POP       {R0-R3}
			BX   	  LR
			ENDP
;-----------------------------------------------------			

;>>>>>   end subroutine code <<<<<
            ALIGN


;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
PWM_duty_table
PWM_duty_table_0
					DCW (PWM_DUTY_10 - 1)										;100% Range
					DCW (((3 * (PWM_DUTY_10 - PWM_DUTY_5)/4) + PWM_DUTY_5)-1)	;75% Ramge
					DCW ((((PWM_DUTY_10 - PWM_DUTY_5) / 2) + PWM_DUTY_5)-1) 	;50% Range
					DCW ((((PWM_DUTY_10 - PWM_DUTY_5) / 4) + PWM_DUTY_5)-1) 	;25% Range
					DCW (PWM_DUTY_5 - 1)										;0% Range

			ALIGN
			
DAC0_table_0

					DCW ((DAC0_STEPS - 1) / (SERVO_POSITIONS * 2))
					DCW (((DAC0_STEPS - 1) * 3) / (SERVO_POSITIONS * 2)) 
					DCW (((DAC0_STEPS - 1) * 5) / (SERVO_POSITIONS * 2)) 
					DCW (((DAC0_STEPS - 1) * 7) / (SERVO_POSITIONS * 2))
					DCW (((DAC0_STEPS - 1) * 9) / (SERVO_POSITIONS * 2))
			ALIGN


;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;Lookup table for pulse width modulation values


Count       SPACE  4
			ALIGN
RunStopWatch  SPACE  1
			  ALIGN

QBuffer     SPACE  Q_BUF_SZ
			ALIGN
QRecord     SPACE  Q_REC_SZ
			ALIGN
RxQBuffer   SPACE  RXTX_BUF
			ALIGN
RxQRecord   SPACE  RXTX_REC 
		    ALIGN
				
TxQBuffer   SPACE  RXTX_BUF
			ALIGN
TxQRecord   SPACE  RXTX_REC 
			ALIGN
				
StringBuffer   SPACE   MAX_STRING
			   ALIGN
StringSize     SPACE   MAX_STRING
;>>>>>   end variables here <<<<<
            ALIGN
            END