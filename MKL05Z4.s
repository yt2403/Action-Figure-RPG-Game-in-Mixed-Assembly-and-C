            OPT   2   ;Turn off listing
            IF    :DEF:MIXED_ASM_C
            PRESERVE8
            ELSE
            PRESERVE8   {FALSE}
            ENDIF
;**********************************************************************
;Freescale MKL05Z32xxx4 device values and configuration code
;* CPU Architecture values
;* CPU Registers
;* Various core, system, and peripheral modules and interfaces
;[1] Freescale Semiconductor, <B>Kinetis L Peripheral Module Quick
;    Reference</B>, KLQRUG, Rev. 0, 9/2012.
;[2] Freescale Semiconductor, <B>KL05 Sub-Family Reference Manual</B>,
;    KL05P48M48SF1RM, Rev. 3.1, 11/2012.
;[3] Freescale Semiconductor, MKL05Z4.h, rev. 1.6, 4/11/2013
;[4] Arm, core_cm0plus.h, v5.0.9, 8/21/2019
;[5] RIT CMPE-250 MKL46Z4.s, rev. 1/5/2018
;[6] Freescale startup_MKL05Z4.s
;    CMSIS Cortex-M0plus Core Device Startup File for the MKL05Z4
;    v1.6, 4/11/
;    FTFA_FlashConfig and NV
;[7] Freescale startup_MKL46Z4.s
;    CMSIS Cortex-M0plus Core Device Startup File for the MKL64Z4
;    v2.2, 4/12/2013
;    FCF_ naming convention
;---------------------------------------------------------------
;Author:  R. W. Melton
;Date:  August 2, 2022
;***************************************************************
;EQUates
;Standard data masks
BYTE_MASK         EQU  0xFF
NIBBLE_MASK       EQU  0x0F
;Standard data sizes (in bits)
BYTE_BITS         EQU  8
NIBBLE_BITS       EQU  4
;Architecture data sizes (in bytes)
WORD_SIZE         EQU  4  ;Cortex-M0+
HALFWORD_SIZE     EQU  2  ;Cortex-M0+
;Architecture data masks
HALFWORD_MASK     EQU  0xFFFF
;Return                 
RET_ADDR_T_MASK   EQU  1  ;Bit 0 of ret. addr. must be
                          ;set for BX, BLX, or POP
                          ;mask in thumb mode
;---------------------------------------------------------------
;Vectors
VECTOR_TABLE_SIZE EQU  0x000000C0  ;KL46
VECTOR_SIZE       EQU  4           ;Bytes per vector
;---------------------------------------------------------------
;CPU CONTROL:  Control register
;31-2:(reserved)
;   1:SPSEL=current stack pointer select
;           0=MSP (main stack pointer) (reset value)
;           1=PSP (process stack pointer)
;   0:nPRIV=not privileged
;        0=privileged (Freescale/NXP "supervisor") (reset value)
;        1=not privileged (Freescale/NXP "user")
CONTROL_SPSEL_MASK   EQU  2
CONTROL_SPSEL_SHIFT  EQU  1
CONTROL_nPRIV_MASK   EQU  1
CONTROL_nPRIV_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PRIMASK:  Interrupt mask register
;31-1:(reserved)
;   0:PM=prioritizable interrupt mask:
;        0=all interrupts unmasked (reset value)
;          (value after CPSIE I instruction)
;        1=prioritizable interrrupts masked
;          (value after CPSID I instruction)
PRIMASK_PM_MASK   EQU  1
PRIMASK_PM_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PSR:  Program status register
;Combined APSR, EPSR, and IPSR
;----------------------------------------------------------
;CPU APSR:  Application Program Status Register
;31  :N=negative flag
;30  :Z=zero flag
;29  :C=carry flag
;28  :V=overflow flag
;27-0:(reserved)
APSR_MASK     EQU  0xF0000000
APSR_SHIFT    EQU  28
APSR_N_MASK   EQU  0x80000000
APSR_N_SHIFT  EQU  31
APSR_Z_MASK   EQU  0x40000000
APSR_Z_SHIFT  EQU  30
APSR_C_MASK   EQU  0x20000000
APSR_C_SHIFT  EQU  29
APSR_V_MASK   EQU  0x10000000
APSR_V_SHIFT  EQU  28
;----------------------------------------------------------
;CPU EPSR
;31-25:(reserved)
;   24:T=thumb state bit
;23- 0:(reserved)
EPSR_MASK     EQU  0x01000000
EPSR_SHIFT    EQU  24
EPSR_T_MASK   EQU  0x01000000
EPSR_T_SHIFT  EQU  24
;----------------------------------------------------------
;CPU IPSR
;31-6:(reserved)
; 5-0:Exception number=number of current exception
;      0=thread mode
;      1:(reserved)
;      2=NMI
;      3=hard fault
;      4-10:(reserved)
;     11=SVCall
;     12-13:(reserved)
;     14=PendSV
;     15=SysTick
;     16=IRQ0
;     16-47:IRQ(Exception number - 16)
;     47=IRQ31
;     48-63:(reserved)
IPSR_MASK             EQU  0x0000003F
IPSR_SHIFT            EQU  0
IPSR_EXCEPTION_MASK   EQU  0x0000003F
IPSR_EXCEPTION_SHIFT  EQU  0
;----------------------------------------------------------
PSR_N_MASK           EQU  APSR_N_MASK
PSR_N_SHIFT          EQU  APSR_N_SHIFT
PSR_Z_MASK           EQU  APSR_Z_MASK
PSR_Z_SHIFT          EQU  APSR_Z_SHIFT
PSR_C_MASK           EQU  APSR_C_MASK
PSR_C_SHIFT          EQU  APSR_C_SHIFT
PSR_V_MASK           EQU  APSR_V_MASK
PSR_V_SHIFT          EQU  APSR_V_SHIFT
PSR_T_MASK           EQU  EPSR_T_MASK
PSR_T_SHIFT          EQU  EPSR_T_SHIFT
PSR_EXCEPTION_MASK   EQU  IPSR_EXCEPTION_MASK
PSR_EXCEPTION_SHIFT  EQU  IPSR_EXCEPTION_SHIFT
;---------------------------------------------------------------
;Cortex-M0+ Core
__CM0PLUS_REV           EQU  0x0000  ;Core revision r0p0
__MPU_PRESENT           EQU  0       ;Whether MPU is present
__NVIC_PRIO_BITS        EQU  2       ;Number of NVIC priority bits
__Vendor_SysTickConfig  EQU  0       ;Whether vendor-specific 
                                     ;SysTickConfig is defined
__VTOR_PRESENT          EQU  1       ;Whether VTOR is present
;---------------------------------------------------------------
;Interrupt numbers (IRQn)
;Interrupt vector is 16 + IRQn
;Core interrupts
NonMaskableInt_IRQn  EQU  -14  ;Non-maskable interrupt (NMI)
HardFault_IRQn       EQU  -13  ;Hard fault interrupt
SVCall_IRQn          EQU  -5   ;Supervisor call interrupt (SVCall)
PendSV_IRQn          EQU  -2   ;Pendable request for system-level service interrupt
                               ;(PendableSrvReq)
SysTick_IRQn         EQU  -1   ;System tick timer interrupt (SysTick)
;--------------------------
;Device specific interrupts
DMA0_IRQn            EQU  0   ;DMA channel 0 transfer complete/error interrupt
DMA1_IRQn            EQU  1   ;DMA channel 1 transfer complete/error interrupt
DMA2_IRQn            EQU  2   ;DMA channel 2 transfer complete/error interrupt
DMA3_IRQn            EQU  3   ;DMA channel 3 transfer complete/error interrupt
Reserved20_IRQn      EQU  4   ;Reserved interrupt 20
FTFA_IRQn            EQU  5   ;FTFA command complete/read collision interrupt
LVD_LVW_IRQn         EQU  6   ;Low-voltage detect, low-voltage warning interrupt
LLW_IRQn             EQU  7   ;Low leakage wakeup interrupt
I2C0_IRQn            EQU  8   ;I2C0 interrupt
Reserved25_IRQn      EQU  9   ;Reserved interrupt 25
SPI0_IRQn            EQU  10  ;SPI0 interrupt
Reserved27_IRQn      EQU  11  ;Reserved interrupt 27
UART0_IRQn           EQU  12  ;UART0 status/error interrupt
Reserved29_IRQn      EQU  13  ;Reserved interrupt 29
Reserved30_IRQn      EQU  14  ;Reserved interrupt 30
ADC0_IRQn            EQU  15  ;ADC0 interrupt
CMP0_IRQn            EQU  16  ;CMP0 interrupt
TPM0_IRQn            EQU  17  ;TPM0 fault, overflow, and channels interrupt
TPM1_IRQn            EQU  18  ;TPM1 fault, overflow, and channels interrupt
Reserved35_IRQn      EQU  19  ;Reserved interrupt 35
RTC_IRQn             EQU  20  ;RTC alarm interrupt
RTC_Seconds_IRQn     EQU  21  ;RTC seconds interrupt
PIT_IRQn             EQU  22  ;PIT interrupt
Reserved39_IRQn      EQU  23  ;Reserved interrupt 39
Reserved40_IRQn      EQU  24  ;Reserved interrupt 40
DAC0_IRQn            EQU  25  ;DAC0 interrupt
TSI0_IRQn            EQU  26  ;TSI0 interrupt
MCG_IRQn             EQU  27  ;MCG interrupt
LPTimer_IRQn         EQU  28  ;LPTMR0 interrupt
Reserved45_IRQn      EQU  29  ;Reserved interrupt 45
PORTA_IRQn           EQU  30  ;Port A interrupt
PORTB_IRQn           EQU  31  ;Port B interrupt
;---------------------------------------------------------------
;Memory map major version
;(Memory maps with equal major version number are compatible)
MCU_MEM_MAP_VERSION        EQU 0x0100
;Memory map minor version
MCU_MEM_MAP_VERSION_MINOR  EQU  0x0006
;---------------------------------------------------------------
;ADC
ADC0_BASE        EQU  0x4003B000
;ADC_BASES        EQU  ADC0_BASE
;ADC_SC1_OFFSET   EQU  0x00
ADC_SC1A_OFFSET  EQU  0x00
ADC_SC1B_OFFSET  EQU  0x04
ADC_CFG1_OFFSET  EQU  0x08
ADC_CFG2_OFFSET  EQU  0x0C
;ADC_R_OFFSET     EQU  0x10
ADC_RA_OFFSET    EQU  0x10
ADC_RB_OFFSET    EQU  0x14
ADC_CV1_OFFSET   EQU  0x18
ADC_CV2_OFFSET   EQU  0x1C
ADC_SC2_OFFSET   EQU  0x20
ADC_SC3_OFFSET   EQU  0x24
ADC_OFS_OFFSET   EQU  0x28
ADC_PG_OFFSET    EQU  0x2C
;ADC_RESERVED_0_OFFSET  EQU  0x30
ADC_CLPD_OFFSET  EQU  0x34
ADC_CLPS_OFFSET  EQU  0x38
ADC_CLP4_OFFSET  EQU  0x3C
ADC_CLP3_OFFSET  EQU  0x40
ADC_CLP2_OFFSET  EQU  0x44
ADC_CLP1_OFFSET  EQU  0x48
ADC_CLP0_OFFSET  EQU  0x4C
ADC0_CFG1        EQU  (ADC0_BASE + ADC_CFG1_OFFSET)
ADC0_CFG2        EQU  (ADC0_BASE + ADC_CFG2_OFFSET)
ADC0_CLPD        EQU  (ADC0_BASE + ADC_CLPD_OFFSET)
ADC0_CLPS        EQU  (ADC0_BASE + ADC_CLPS_OFFSET)
ADC0_CLP0        EQU  (ADC0_BASE + ADC_CLP0_OFFSET)
ADC0_CLP1        EQU  (ADC0_BASE + ADC_CLP1_OFFSET)
ADC0_CLP2        EQU  (ADC0_BASE + ADC_CLP2_OFFSET)
ADC0_CLP3        EQU  (ADC0_BASE + ADC_CLP3_OFFSET)
ADC0_CLP4        EQU  (ADC0_BASE + ADC_CLP4_OFFSET)
ADC0_CV1         EQU  (ADC0_BASE + ADC_CV1_OFFSET)
ADC0_CV2         EQU  (ADC0_BASE + ADC_CV2_OFFSET)
ADC0_OFS         EQU  (ADC0_BASE + ADC_OFS_OFFSET)
ADC0_PG          EQU  (ADC0_BASE + ADC_PG_OFFSET) 
;ACD0_R           EQU  (ADC0_BASE + ADC_R_OFFSET)  
ADC0_RA          EQU  ( ADC0_BASE + ADC_RA_OFFSET)  
ADC0_RB          EQU  ( ADC0_BASE + ADC_RB_OFFSET)
;ADC0_RESERVED_0  EQU  ( ADC0_BASE + ADC_RESERVED_0_OFFSET)
;ADC0_SC1         EQU  ( ADC0_BASE + ADC_SC1_OFFSET)
ADC0_SC1A        EQU  ( ADC0_BASE + ADC_SC1A_OFFSET)
ADC0_SC1B        EQU  ( ADC0_BASE + ADC_SC1B_OFFSET)
ADC0_SC2         EQU  ( ADC0_BASE + ADC_SC2_OFFSET)
ADC0_SC3         EQU  ( ADC0_BASE + ADC_SC3_OFFSET)
;---------------------------------------------------------------
;ADC_CFG1:  ADC configuration register 1
;31-8:(reserved):read-only:0
;   7:ADLPC=ADC low-power configuration
; 6-5:ADIV=ADC clock divide select
;     Internal ADC clock = input clock / 2^ADIV
;   4:ADLSMP=ADC long sample time configuration
;            0=short
;            1=long
; 3-2:MODE=conversion mode selection
;          00=single-ended 8-bit conversion
;          01=single-ended 12-bit conversion
;          10=single-ended 10-bit conversion
;          11=(reserved; do not set this value)
; 1-0:ADICLK=ADC input clock select
;          00=bus clock
;          01=bus clock / 2
;          10=alternate clock (ALTCLK)
;          11=asynchronous clock (ADACK)
ADC_CFG1_ADLPC_MASK   EQU  0x80
ADC_CFG1_ADLPC_SHIFT  EQU  7
ADC_CFG1_ADIV_MASK    EQU  0x60
ADC_CFG1_ADIV_SHIFT   EQU  5
ADC_CFG1_ADLSMP_MASK  EQU  0x10
ADC_CFG1_ADLSMP_SHIFT EQU  4
ADC_CFG1_MODE_MASK    EQU  0x0C
ADC_CFG1_MODE_SHIFT   EQU  2
ADC_CFG1_ADICLK_MASK  EQU  0x03
ADC_CFG1_ADICLK_SHIFT EQU  0
;---------------------------------------------------------------
;ADC_CFG2:  ADC configuration register 2
;31-8:(reserved):read-only:0
; 7-5:(reserved):read-only:0
;   4:MUXSEL=ADC mux select
;            0=ADxxA channels are selected
;            1=ADxxB channels are selected
;   3:ADACKEN=ADC asynchronous clock output enable
;             0=asynchronous clock determined by ACD0_CFG1.ADICLK 
;             1=asynchronous clock enabled
;   2:ADHSC=ADC high-speed configuration
;           0=normal conversion
;           1=high-speed conversion (only 2 additional ADK cycles)
; 1-0:ADLSTS=ADC long sample time select (ADK cycles)
;          00=default longest sample time:  
;             24 total ADK cycles (20 extra)
;          01=16 total ADK cycles (12 extra)
;          10=10 total ADK cycles (6 extra)
;          11=6 total ADK cycles (2 extra)
ADC_CFG2_MUXSEL_MASK    EQU  0x10
ADC_CFG2_MUXSEL_SHIFT   EQU  4
ADC_CFG2_ADACKEN_MASK   EQU  0x08
ADC_CFG2_ADACKEN_SHIFT  EQU  3
ADC_CFG2_ADHSC_MASK     EQU  0x04
ADC_CFG2_ADHSC_SHIFT    EQU  2
ADC_CFG2_ADLSTS_MASK    EQU  0x03
ADC_CFG2_ADLSTS_SHIFT   EQU  0
;---------------------------------------------------------------
;ADC_CLPD:  ADC plus-side general calibration value register D
;31-6:(reserved):read-only:0
; 5-0:CLPD=calibration value
ADC_CLPD_MASK   EQU  0x3F
ADC_CLPD_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CLPS:  ADC plus-side general calibration value register S
;31-6:(reserved):read-only:0
; 5-0:CLPS=calibration value
ADC_CLPS_MASK   EQU  0x3F
ADC_CLPS_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CLP0:  ADC plus-side general calibration value register 0
;31-6:(reserved):read-only:0
; 5-0:CLP0=calibration value
ADC_CLP0_MASK   EQU  0x3F
ADC_CLP0_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CLP1:  ADC plus-side general calibration value register 1
;31-7:(reserved):read-only:0
; 6-0:CLP1=calibration value
ADC_CLP1_MASK   EQU  0x7F
ADC_CLP1_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CLP2:  ADC plus-side general calibration value register 2
;31-8:(reserved):read-only:0
; 7-0:CLP2=calibration value
ADC_CLP2_MASK   EQU  0xFF
ADC_CLP2_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CLP3:  ADC plus-side general calibration value register 3
;31-9:(reserved):read-only:0
; 8-0:CLP3=calibration value
ADC_CLP3_MASK   EQU  0x1FF
ADC_CLP3_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CLP4:  ADC plus-side general calibration value register 4
;31-10:(reserved):read-only:0
; 9- 0:CLP4=calibration value
ADC_CLP4_MASK   EQU  0x3FF
ADC_CLP4_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_CVn:  ADC channel n compare value register
;CV1 used to compare result when ADC_SC2.ACFE=1
;CV2 used to compare result when ADC_SC2.ACREN=1
;31-16:(reserved):read-only:0
;15- 0:compare value (zero-extended to 16 bits,
;                     consistent with format of ACD_Rn)
ADC_CV_MASK   EQU  0xFFFF
ADC_CV_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_OFS:  ADC offset correction register
;31-16:(reserved):read-only:0
;15- 0:OFS=offset error correction value
ADC_OFS_MASK   EQU  0xFFFF
ADC_OFS_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_PG:  ADC plus-side gain register
;31-16:(reserved):read-only:0
;15- 0:PG=plus-side gain
ADC_PG_MASK   EQU  0xFFFF
ADC_PG_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_Rn:  ADC channel n data result register
;31-16:(reserved):read-only:0
;15- 0:data result (zero-extended to 16 bits)
ADC_R_D_MASK   EQU  0xFFFF
ADC_R_D_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_SC1n:  ADC channel n status and control register 1
;31-8:(reserved):read-only:0
;   7:COCO=conversion complete flag (read-only)
;   6:AIEN=ADC interrupt enabled
;   5:(reserved)
; 4-0:ADCH=ADC input channel select
;          00000=AD0
;          00001=AD1
;          00010=AD2
;          00011=AD3
;          00100=AD4
;          00101=AD5 (12-bit DAC0 Output)
;          00110=AD6
;          00111=AD7
;          01000=AD8
;          01001=AD9
;          01010=AD10
;          01011=AD11
;          01100=AD12
;          01101=AD13
;          01110=(reserved)
;          01111=(reserved)
;          10000=(reserved)
;          10001=(reserved)
;          10010=(reserved)
;          10011=(reserved)
;          10100=(reserved)
;          10101=(reserved)
;          10110=(reserved)
;          10111=(reserved)
;          11000=(reserved)
;          11001=(reserved)
;          11010=temp sensor (single-ended)
;          11011=bandgap
;          11100=(reserved)
;          11101=VREFSH
;          11110=VREFSL
;          11111=disabled
ADC_SC1_COCO_MASK   EQU  0x80
ADC_SC1_COCO_SHIFT  EQU  7
ADC_SC1_AIEN_MASK   EQU  0x40
ADC_SC1_AIEN_SHIFT  EQU  6
ADC_SC1_ADCH_MASK   EQU  0x1F
ADC_SC1_ADCH_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_SC2:  ADC status and control register 2
;31-8:(reserved):read-only:0
;   7:ADACT=ADC conversion active
;   6:ADTRG=ADC conversion trigger select
;           0=software trigger
;           1=hardware trigger
;   5:ACFE=ADC compare function enable
;   4:ACFGT=ADC compare function greater than enable
;           based on values in ADC_CV1 and ADC_CV2
;           0=configure less than threshold and non-inclusive range
;           1=configure greater than threshold and non-inclusive range
;   3:ACREN=ADC compare function range enable
;           0=disabled; only ADC_CV1 compared
;           1=enabled; both ADC_CV1 and ADC_CV2 compared
;   2:DMAEN=DMA enable
; 1-0:REFSEL=voltage reference selection
;            00=default:VREFH and VREFL
;            01=alternative:VALTH and VALTL
;            10=(reserved)
;            11=(reserved)
ADC_SC2_ADACT_MASK    EQU  0x80
ADC_SC2_ADACT_SHIFT   EQU  7
ADC_SC2_ADTRG_MASK    EQU  0x40
ADC_SC2_ADTRG_SHIFT   EQU  6
ADC_SC2_ACFE_MASK     EQU  0x20
ADC_SC2_ACFE_SHIFT    EQU  5
ADC_SC2_ACFGT_MASK    EQU  0x10
ADC_SC2_ACFGT_SHIFT   EQU  4
ADC_SC2_ACREN_MASK    EQU  0x08
ADC_SC2_ACREN_SHIFT   EQU  3
ADC_SC2_DMAEN_MASK    EQU  0x04
ADC_SC2_DMAEN_SHIFT   EQU  2
ADC_SC2_REFSEL_MASK   EQU  0x03
ADC_SC2_REFSEL_SHIFT  EQU  0
;---------------------------------------------------------------
;ADC_SC3:  ADC status and control register 3
;31-8:(reserved):read-only:0
;   7:CAL=calibration
;         write:0=(no effect)
;               1=start calibration sequence
;         read:0=calibration sequence complete
;              1=calibration sequence in progress
;   6:CALF=calibration failed flag
; 5-4:(reserved):read-only:0
;   3:ADC=ADC continuous conversion enable (if ADC_SC3.AVGE = 1)
;   2:AVGE=hardware average enable
; 1-0:AVGS=hardware average select:  2^(2+AVGS) samples
ADC_SC3_CAL_MASK    EQU  0x80
ADC_SC3_CAL_SHIFT   EQU  7
ADC_SC3_CALF_MASK   EQU  0x40
ADC_SC3_CALF_SHIFT  EQU  6
ADC_SC3_ADCO_MASK   EQU  0x08
ADC_SC3_ADCO_SHIFT  EQU  3
ADC_SC3_AVGE_MASK   EQU  0x04
ADC_SC3_AVGE_SHIFT  EQU  2
ADC_SC3_AVGS_MASK   EQU  0x03
ADC_SC3_AVGS_SHIFT  EQU  0
;---------------------------------------------------------------
;CMP
CMP0_BASE          EQU  0x40073000
;CMP_BASES          EQU  CMP0_BASE
CMP0_CR0_OFFSET    EQU  0x00
CMP0_CR1_OFFSET    EQU  0x01
CMP0_FPR_OFFSET    EQU  0x02
CMP0_SCR_OFFSET    EQU  0x03
CMP0_DACCR_OFFSET  EQU  0x04
CMP0_MUXCR_OFFSET  EQU  0x05
CMP0_CR0           EQU  (CMP0_BASE + CMP0_CR0_OFFSET)
CMP0_CR1           EQU  (CMP0_BASE + CMP0_CR1_OFFSET)
CMP0_FPR           EQU  (CMP0_BASE + CMP0_FPR_OFFSET)
CMP0_SCR           EQU  (CMP0_BASE + CMP0_SCR_OFFSET)
CMP0_DACCR         EQU  (CMP0_BASE + CMP0_DACCR_OFFSET)
CMP0_MUXCR         EQU  (CMP0_BASE + CMP0_MUXCR_OFFSET)
;---------------------------------------------------------------
;CMP0_CR0:  CMP0 control register 0 (0x00)
;  7:(reserved):read-only:0
;6-4:FILTER_CNT=filter sample count (00)
;  3:(reserved):read-only:0
;  2:(reserved):read-only:0
;1-0:HYSTCTR=comparator hard block hysteresis control (00)
CMP_CR0_HYSTCTR_MASK      EQU  0x3
CMP_CR0_HYSTCTR_SHIFT     EQU  0
CMP_CR0_FILTER_CNT_MASK   EQU  0x70
CMP_CR0_FILTER_CNT_SHIFT  EQU  4
;---------------------------------------------------------------
;CMP0_CR1:  CMP0 control register 1 (0x00)
;7:SE=sample enable (0)
;6:WE=windowing enable (0)
;5:TRIGM=trigger mode enable (0)
;4:PMODE=power mode select (0)
;        0=low-speed comparison mode
;        1=high-speed comparison mode
;3:INV=comparator invert (0)
;2:COS=comparator output select (0)
;      0=filtered output COUT
;      1=unfiltered output COUTA
;1:OPE=comparator output pin enable (0)
;0:EN=comparator module enable (0)
CMP_CR1_EN_MASK      EQU  0x1
CMP_CR1_EN_SHIFT     EQU  0
CMP_CR1_OPE_MASK     EQU  0x2
CMP_CR1_OPE_SHIFT    EQU  1
CMP_CR1_COS_MASK     EQU  0x4
CMP_CR1_COS_SHIFT    EQU  2
CMP_CR1_INV_MASK     EQU  0x8
CMP_CR1_INV_SHIFT    EQU  3
CMP_CR1_PMODE_MASK   EQU  0x10
CMP_CR1_PMODE_SHIFT  EQU  4
CMP_CR1_TRIGM_MASK   EQU  0x20
CMP_CR1_TRIGM_SHIFT  EQU  5
CMP_CR1_WE_MASK      EQU  0x40
CMP_CR1_WE_SHIFT     EQU  6
CMP_CR1_SE_MASK      EQU  0x80
CMP_CR1_SE_SHIFT     EQU  7
;---------------------------------------------------------------
;CMP0_FPR=CMP filter period register (0x00)
;7-0:FILT_PER=CMP filter period register (0x00)
CMP_FPR_FILT_PER_MASK   EQU  0xFF
CMP_FPR_FILT_PER_SHIFT  EQU  0
;---------------------------------------------------------------
;CMP0_SCR=CMP status and control register (0x00)
;7:(reserved):read-only:0
;6:DMAEN=DMA enable control (0)
;5:(reserved):read-only:0
;4:IER=comparator interrupt enable rising (0)
;3:IEF=comparator interrupt enable falling (0)
;2:CFR=analog comparator flag rising: w1c (0)
;1:CFF=analog comparator flag falling: w1c (0)
;0:COUT=analog comparator output:  read-only (0)
CMP_SCR_COUT_MASK    EQU  0x1
CMP_SCR_COUT_SHIFT   EQU  0
CMP_SCR_CFF_MASK     EQU  0x2
CMP_SCR_CFF_SHIFT    EQU  1
CMP_SCR_CFR_MASK     EQU  0x4
CMP_SCR_CFR_SHIFT    EQU  2
CMP_SCR_IEF_MASK     EQU  0x8
CMP_SCR_IEF_SHIFT    EQU  3
CMP_SCR_IER_MASK     EQU  0x10
CMP_SCR_IER_SHIFT    EQU  4
CMP_SCR_DMAEN_MASK   EQU  0x40
CMP_SCR_DMAEN_SHIFT  EQU  6
;---------------------------------------------------------------
;CMP0_DACCR=DAC control register (0x00)
;  7:DACEN=DAC enable (0)
;  6:VRSEL=supply voltage reference source select (0)
;          0=Selected from mux by Vin1
;          1=Selected from mux by Vin2
;5-0:VOSEL=DAC output voltage select (00000)
;          DAC0 = (Vin / 64) x (VOSEL[5:0] + 1)
CMP_DACCR_VOSEL_MASK   EQU  0x3F
CMP_DACCR_VOSEL_SHIFT  EQU  0
CMP_DACCR_VRSEL_MASK   EQU  0x40
CMP_DACCR_VRSEL_SHIFT  EQU  6
CMP_DACCR_DACEN_MASK   EQU  0x80
CMP_DACCR_DACEN_SHIFT  EQU  7
;---------------------------------------------------------------
;CMP0_MUXCR=MUX control register (0x00)
;  7:PSTM=pass through mode enable (0)
;  6:(reserved):read-only:0
;5-3:PSEL=plus input mux control (000)
;         selects IN[PSEL]
;2-0:MSEL=minus input mux control (000)
;         selects IN[MSEL]
CMP_MUXCR_MSEL_MASK   EQU  0x7
CMP_MUXCR_MSEL_SHIFT  EQU  0
CMP_MUXCR_PSEL_MASK   EQU  0x38
CMP_MUXCR_PSEL_SHIFT  EQU  3
CMP_MUXCR_PSTM_MASK   EQU  0x80
CMP_MUXCR_PSTM_SHIFT  EQU  7
;---------------------------------------------------------------
;DAC
DAC0_BASE          EQU  0x4003F000
;DAC_BASES          EQU  DAC0_BASE
;DAC0_DAT_OFFSET    EQU  0x00
DAC0_DAT0L_OFFSET  EQU  0x00
DAC0_DAT0H_OFFSET  EQU  0x01
DAC0_DAT1L_OFFSET  EQU  0x02
DAC0_DAT1H_OFFSET  EQU  0x03
;DAC0_RESERVED_0_OFFSET  EQU  0x04
DAC0_SR_OFFSET     EQU  0x20
DAC0_C0_OFFSET     EQU  0x21
DAC0_C1_OFFSET     EQU  0x22
DAC0_C2_OFFSET     EQU  0x23
;DAC0_DAT           EQU  (DAC0_BASE + DAC0_DAT_OFFSET)
DAC0_DAT0L         EQU  (DAC0_BASE + DAC0_DAT0L_OFFSET)
DAC0_DAT0H         EQU  (DAC0_BASE + DAC0_DAT0H_OFFSET)
DAC0_DAT1L         EQU  (DAC0_BASE + DAC0_DAT1L_OFFSET)
DAC0_DAT1H         EQU  (DAC0_BASE + DAC0_DAT1H_OFFSET)
;DAC0_RESERVED_0    EQU  (DAC0_BASE + DAC0_RESERVED_0_OFFSET)
DAC0_SR            EQU  (DAC0_BASE + DAC0_SR_OFFSET)
DAC0_C0            EQU  (DAC0_BASE + DAC0_C0_OFFSET)
DAC0_C1            EQU  (DAC0_BASE + DAC0_C1_OFFSET)
DAC0_C2            EQU  (DAC0_BASE + DAC0_C2_OFFSET)
;---------------------------------------------------------------
;DAC_DAT0H:  DAC data high register 0
;If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
;7-4:(reserved):read-only:0
;3-0:DATA1=DATA[11:8] (0000)
DAC_DAT0H_MASK   EQU  0x0F
DAC_DAT0H_SHIFT  EQU  0
;---------------------------------------------------------------
;DAC_DAT0L:  DAC data low register 0
;If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
;7-0:DATA0=DATA[7:0] (00000000)
DAC_DAT0L_MASK   EQU  0xFF
DAC_DAT0L_SHIFT  EQU  0
;---------------------------------------------------------------
;DAC_DAT1H:  DAC data high register 1
;If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
;7-4:(reserved):read-only:0
;3-0:DATA1=DATA[11:8] (0000)
DAC_DAT1H_MASK   EQU  0x0F
DAC_DAT1H_SHIFT  EQU  0
;---------------------------------------------------------------
;DAC_DAT1L:  DAC data low register 1
;If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
;7-0:DATA0=DATA[7:0] (00000000)
DAC_DAT1L_MASK   EQU  0xFF
DAC_DAT1L_SHIFT  EQU  0
;;---------------------------------------------------------------
;;DAC_DATH:  DAC data high registers
;;If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
;;7-4:(reserved):read-only:0
;;3-0:DATA1=DATA[11:8] (0000)
;DAC_DATH_DATA0_MASK   EQU  0x0F
;DAC_DATH_DATA0_SHIFT  EQU  0
;;---------------------------------------------------------------
;;DAC_DATL:  DAC data low registers
;;If buffer not enabled, Vout = Vin * (1 + DATA[11:0])/4096.
;;7-0:DATA0=DATA[7:0] (00000000)
;DAC_DATL_DATA0_MASK   EQU  0xFF
;DAC_DATL_DATA0_SHIFT  EQU  0
;---------------------------------------------------------------
;DAC_C0:  DAC control register 0
;7:DACEN=DAC enable (0)
;6:DACRFS=DAC reference select (0)
;         0:DACREF_1=VREFH
;         1:DACREF_2=VDDA (best for ADC operation)
;5:DACTRGSEL=DAC trigger select (0)
;            0:HW
;            1:SW
;4:DACSWTRG=DAC software trigger (0)
;           active-high write-only field that reads 0
;           DACBFEN & DACTRGSEL:  writing 1 advances buffer pointer
;3:LPEN=DAC low power control (0)
;       0:high-power mode
;       1:low-power mode
;2:(reserved):read-only:0
;1:DACBTIEN=DAC buffer read pointer top flag interrupt enable (0)
;0:DACBBIEN=DAC buffer read pointer bottom flag interrupt enable (0)
DAC_C0_DACEN_MASK       EQU  0x80
DAC_C0_DACEN_SHIFT      EQU  7
DAC_C0_DACRFS_MASK      EQU  0x40
DAC_C0_DACRFS_SHIFT     EQU  6
DAC_C0_DACTRGSEL_MASK   EQU  0x20
DAC_C0_DACTRGSEL_SHIFT  EQU  5
DAC_C0_DACSWTRG_MASK    EQU  0x10
DAC_C0_DACSWTRG_SHIFT   EQU  4
DAC_C0_LPEN_MASK        EQU  0x08
DAC_C0_LPEN_SHIFT       EQU  3
DAC_C0_DACBTIEN_MASK    EQU  0x02
DAC_C0_DACBTIEN_SHIFT   EQU  1
DAC_C0_DACBBIEN_MASK    EQU  0x01
DAC_C0_DACBBIEN_SHIFT   EQU  0
;---------------------------------------------------------------
;DAC_C1:  DAC control register 1
;  7:DMAEN=DMA enable select (0)
;6-3:(reserved):read-only:0000
;  2:DACBFMD=DAC buffer work mode select (0)
;            0:normal
;            1:one-time scan
;  1:(reserved):read-only:0
;  0:DACBFEN=DAC buffer enable (0)
;            0:disabled:data in first word of buffer
;            1:enabled:read pointer points to data
DAC_C1_DMAEN_MASK       EQU  0x80
DAC_C1_DMAEN_SHIFT      EQU  7
DAC_C1_DACBFMD_MASK     EQU  0x04
DAC_C1_DACBFMD_SHIFT    EQU  2
DAC_C1_DACBFEN_MASK     EQU  0x01
DAC_C1_DACBFEN_SHIFT    EQU  0
;---------------------------------------------------------------
;DAC_C2:  DAC control register 2
;7-5:(reserved):read-only:0
;  4:DACBFRP=DAC buffer read pointer (0)
;3-1:(reserved):read-only:0
;  0:DACBFUP=DAC buffer read upper limit (1)
DAC_C2_DACBFRP_MASK   EQU  0x10
DAC_C2_DACBFRP_SHIFT  EQU  4
DAC_C2_DACBFUP_MASK   EQU  0x01
DAC_C2_DACBFUP_SHIFT  EQU  0
;---------------------------------------------------------------
;DAC_SR:  DAC status register
;Writing 0 clears a field; writing 1 has no effect.
;7-2:(reserved):read-only:000000
;1:DACBFRPTF=DAC buffer read pointer top position flag (1)
;            Indicates whether pointer is zero
;0:DACBFRPBF=DAC buffer read pointer bottom position flag (0)
;            Indicates whether pointer is equal to DAC0_C2.DACBFUP.
DAC_SR_DACBFRPTF_MASK   EQU 0x02
DAC_SR_DACBFRPTF_SHIFT  EQU 1
DAC_SR_DACBFRPBF_MASK   EQU 0x01
DAC_SR_DACBFRPBF_SHIFT  EQU 0
;---------------------------------------------------------------
;Flash Configuration Field (FCF) 0x400-0x40F
;Following [6, 7]
;16-byte flash configuration field that stores default protection settings
;(loaded on reset) and security information that allows the MCU to 
;restrict acces to the FTFL module.
;FCF Backdoor Comparison Key
;8 bytes from 0x400-0x407
;-----------------------------------------------------
;FCF Backdoor Comparison Key 0
;7-0:Backdoor Key 0
FCF_BACKDOOR_KEY0  EQU  0xFF
BackDoorK0         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 1
;7-0:Backdoor Key 1
FCF_BACKDOOR_KEY1  EQU  0xFF
BackDoorK1         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 2
;7-0:Backdoor Key 2
FCF_BACKDOOR_KEY2  EQU  0xFF
BackDoorK2         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 3
;7-0:Backdoor Key 3
FCF_BACKDOOR_KEY3  EQU  0xFF
BackDoorK3         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 4
;7-0:Backdoor Key 4
FCF_BACKDOOR_KEY4  EQU  0xFF
BackDoorK4         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 5
;7-0:Backdoor Key 5
FCF_BACKDOOR_KEY5  EQU  0xFF
BackDoorK5         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 6
;7-0:Backdoor Key 6
FCF_BACKDOOR_KEY6  EQU  0xFF
BackDoorK6         EQU  0xFF
;-----------------------------------------------------
;FCF Backdoor Comparison Key 7
;7-0:Backdoor Key 7
FCF_BACKDOOR_KEY7  EQU  0xFF
BackDoorK7         EQU  0xFF
;-----------------------------------------------------
;FCF Flash nonvolatile option byte (FCF_FOPT)
;Allows user to customize operation of the MCU at boot time.
;7-6:11:(reserved)
;  5: 1:FAST_INIT=fast initialization
;4,0:11:LPBOOT=core and system clock divider:  2^(3-LPBOOT)
;  3: 1:RESET_PIN_CFG=enable reset pin following POR
;  2: 1:NMI_DIS=Enable NMI
;  1: 1:(reserved)
;  0:(see bit 4 above)
FCF_FOPT  EQU  0xFF
FOPT      EQU  0xFF
;-----------------------------------------------------
;FCF Program flash protection bytes (FCF_FPROT)
;Each program flash region can be protected from program and erase 
;operation by setting the associated PROT bit.  Each bit protects a 
;1/32 region of the program flash memory.
;FCF FPROT0
;7:1:FCF_PROT7=Program flash region 7/32 not protected
;6:1:FCF_PROT6=Program flash region 6/32 not protected
;5:1:FCF_PROT5=Program flash region 5/32 not protected
;4:1:FCF_PROT4=Program flash region 4/32 not protected
;3:1:FCF_PROT3=Program flash region 3/32 not protected
;2:1:FCF_PROT2=Program flash region 2/32 not protected
;1:1:FCF_PROT1=Program flash region 1/32 not protected
;0:1:FCF_PROT0=Program flash region 0/32 not protected
FCF_FPROT0  EQU  0xFF
nFPROT0     EQU  0x00
FPROT0      EQU  nFPROT0:EOR:0xFF
;-----------------------------------------------------
;FCF FPROT1
;7:1:FCF_PROT15=Program flash region 15/32 not protected
;6:1:FCF_PROT14=Program flash region 14/32 not protected
;5:1:FCF_PROT13=Program flash region 13/32 not protected
;4:1:FCF_PROT12=Program flash region 12/32 not protected
;3:1:FCF_PROT11=Program flash region 11/32 not protected
;2:1:FCF_PROT10=Program flash region 10/32 not protected
;1:1:FCF_PROT9=Program flash region 9/32 not protected
;0:1:FCF_PROT8=Program flash region 8/32 not protected
FCF_FPROT1  EQU  0xFF
nFPROT1     EQU  0x00
FPROT1      EQU  nFPROT1:EOR:0xFF
;-----------------------------------------------------
;FCF FPROT2
;7:1:FCF_PROT23=Program flash region 23/32 not protected
;6:1:FCF_PROT22=Program flash region 22/32 not protected
;5:1:FCF_PROT21=Program flash region 21/32 not protected
;4:1:FCF_PROT20=Program flash region 20/32 not protected
;3:1:FCF_PROT19=Program flash region 19/32 not protected
;2:1:FCF_PROT18=Program flash region 18/32 not protected
;1:1:FCF_PROT17=Program flash region 17/32 not protected
;0:1:FCF_PROT16=Program flash region 16/32 not protected
FCF_FPROT2  EQU  0xFF
nFPROT2     EQU  0x00
FPROT2      EQU  nFPROT2:EOR:0xFF
;-----------------------------------------------------
;FCF FPROT3
;7:1:FCF_PROT31=Program flash region 31/32 not protected
;6:1:FCF_PROT30=Program flash region 30/32 not protected
;5:1:FCF_PROT29=Program flash region 29/32 not protected
;4:1:FCF_PROT28=Program flash region 28/32 not protected
;3:1:FCF_PROT27=Program flash region 27/32 not protected
;2:1:FCF_PROT26=Program flash region 26/32 not protected
;1:1:FCF_PROT25=Program flash region 25/32 not protected
;0:1:FCF_PROT24=Program flash region 24/32 not protected
FCF_FPROT3  EQU  0xFF
nFPROT3     EQU  0x00
FPROT3      EQU  nFPROT3:EOR:0xFF
;-----------------------------------------------------
;FCF Flash security byte (FCF_FSEC)
;WARNING: If SEC field is configured as "MCU security status is 
;secure" and MEEN field is configured as "Mass erase is disabled",
;MCU's security status cannot be set back to unsecure state since 
;mass erase via the debugger is blocked !!!
;7-6:01:KEYEN=backdoor key security enable
;            :00=Backdoor key access disabled
;            :01=Backdoor key access disabled (preferred value)
;            :10=Backdoor key access enabled
;            :11=Backdoor key access disabled
;5-4:11:MEEN=mass erase enable bits
;           (does not matter if SEC unsecure)
;           :00=mass erase enabled
;           :01=mass erase enabled
;           :10=mass erase disabled
;           :11=mass erase enabled
;3-2:11:FSLACC=Freescale failure analysis access code
;             (does not matter if SEC unsecure)
;             :00=Freescale factory access granted
;             :01=Freescale factory access denied
;             :10=Freescale factory access denied
;             :11=Freescale factory access granted
;1-0:10:SEC=flash security
;          :00=MCU secure
;          :01=MCU secure
;          :10=MCU unsecure (standard value)
;          :11=MCU secure
FCF_FSEC  EQU  0x7E
FSEC      EQU  0xFE
;---------------------------------------------------------------
;Fast (zero wait state) GPIO (FGPIO) or (IOPORT)
;FGPIOx_PDD:  Port x Data Direction Register
;  Bit n:  0=Port x pin n configured as input
;          1=Port x pin n configured as output
;FGPIOx_PDIR:  Port x Data Input Register
;  Bit n:  Value read from Port x pin n (if input pin)
;FGPIOx_PDOR:  Port x Data Output Register
;  Bit n:  Value written to Port x pin n (if output pin)
;FGPIOx_PoOR: Port x operation o direction x Register
;  Operation o:  C=Clear (clear to 0)
;                S=Set (set to 1)
;                T=Toggle (complement)
;  Bit n:  0=Port x pin n not affected
;          1=Port x pin n affected
FGPIO_BASE         EQU  0xF80FF000
FGPIO_PDOR_OFFSET  EQU  0x00
FGPIO_PSOR_OFFSET  EQU  0x04
FGPIO_PCOR_OFFSET  EQU  0x08
FGPIO_PTOR_OFFSET  EQU  0x0C
FGPIO_PDIR_OFFSET  EQU  0x10
FGPIO_PDDR_OFFSET  EQU  0x14
;FGPIOx not present in MKL05Z4.h:  FPTx instead
;FGPIOx included for compatibility with MKL46Z4.h
;Fast Port A
FGPIOA_BASE        EQU  0xF80FF000
FGPIOA_PDOR        EQU  (FGPIOA_BASE + GPIO_PDOR_OFFSET)
FGPIOA_PSOR        EQU  (FGPIOA_BASE + GPIO_PSOR_OFFSET)
FGPIOA_PCOR        EQU  (FGPIOA_BASE + GPIO_PCOR_OFFSET)
FGPIOA_PTOR        EQU  (FGPIOA_BASE + GPIO_PTOR_OFFSET)
FGPIOA_PDIR        EQU  (FGPIOA_BASE + GPIO_PDIR_OFFSET)
FGPIOA_PDDR        EQU  (FGPIOA_BASE + GPIO_PDDR_OFFSET)
;Fast Port B
FGPIOB_BASE        EQU  0xF80FF040
FGPIOB_PDOR        EQU  (FGPIOB_BASE + GPIO_PDOR_OFFSET)
FGPIOB_PSOR        EQU  (FGPIOB_BASE + GPIO_PSOR_OFFSET)
FGPIOB_PCOR        EQU  (FGPIOB_BASE + GPIO_PCOR_OFFSET)
FGPIOB_PTOR        EQU  (FGPIOB_BASE + GPIO_PTOR_OFFSET)
FGPIOB_PDIR        EQU  (FGPIOB_BASE + GPIO_PDIR_OFFSET)
FGPIOB_PDDR        EQU  (FGPIOB_BASE + GPIO_PDDR_OFFSET)
;Fast GPIO
;FGPIO_BASES        EQU  FGPIOA_BASE
;---------------------------------------------------------------
;FPTx not present in MKL46Z4.h
;Fast (zero wait state) general-purpose input and output (FPTx)
;FPTx_PDD:  Port x Data Direction Register
;  Bit n:  0=Port x pin n configured as input
;          1=Port x pin n configured as output
;FPTx_PDIR:  Port x Data Input Register
;  Bit n:  Value read from Port x pin n (if input pin)
;FPTx_PDOR:  Port x Data Output Register
;  Bit n:  Value written to Port x pin n (if output pin)
;FPTx_PoOR: Port x operation o direction x Register
;  Operation o:  C=Clear (clear to 0)
;                S=Set (set to 1)
;                T=Toggle (complement)
;  Bit n:  0=Port x pin n not affected
;          1=Port x pin n affected
FPT_BASE         EQU  0xF80FF000
FPT_PDOR_OFFSET  EQU  0x00
FPT_PSOR_OFFSET  EQU  0x04
FPT_PCOR_OFFSET  EQU  0x08
FPT_PTOR_OFFSET  EQU  0x0C
FPT_PDIR_OFFSET  EQU  0x10
FPT_PDDR_OFFSET  EQU  0x14
FPTA_OFFSET      EQU  0x00
FPTB_OFFSET      EQU  0x40
;Fast Port A (FPTA)
FPTA_BASE        EQU  0xF80FF000
FPTA_PSOR        EQU  (FPTA_BASE + FPT_PSOR_OFFSET)
FPTA_PDOR        EQU  (FPTA_BASE + FPT_PDOR_OFFSET)
FPTA_PCOR        EQU  (FPTA_BASE + FPT_PCOR_OFFSET)
FPTA_PTOR        EQU  (FPTA_BASE + FPT_PTOR_OFFSET)
FPTA_PDIR        EQU  (FPTA_BASE + FPT_PDIR_OFFSET)
FPTA_PDDR        EQU  (FPTA_BASE + FPT_PDDR_OFFSET)
;Fast Port B (FPTB)
FPTB_BASE        EQU  0xF80FF040
FPTB_PDOR        EQU  (FPTB_BASE + FPT_PDOR_OFFSET)
FPTB_PSOR        EQU  (FPTB_BASE + FPT_PSOR_OFFSET)
FPTB_PCOR        EQU  (FPTB_BASE + FPT_PCOR_OFFSET)
FPTB_PTOR        EQU  (FPTB_BASE + FPT_PTOR_OFFSET)
FPTB_PDIR        EQU  (FPTB_BASE + FPT_PDIR_OFFSET)
FPTB_PDDR        EQU  (FPTB_BASE + FPT_PDDR_OFFSET)
;---------------------------------------------------------------
;Flash configuration field
;Nonvolatile (NV) Peripheral access layer
;Following [3]
FTFA_FlashConfig_BASE             EQU  0x400
FTFA_FlashConfig_BACKKEY3_OFFSET  EQU  0x0
FTFA_FlashConfig_BACKKEY2_OFFSET  EQU  0x1
FTFA_FlashConfig_BACKKEY1_OFFSET  EQU  0x2
FTFA_FlashConfig_BACKKEY0_OFFSET  EQU  0x3
FTFA_FlashConfig_BACKKEY7_OFFSET  EQU  0x4
FTFA_FlashConfig_BACKKEY6_OFFSET  EQU  0x5
FTFA_FlashConfig_BACKKEY5_OFFSET  EQU  0x6
FTFA_FlashConfig_BACKKEY4_OFFSET  EQU  0x7
FTFA_FlashConfig_FPROT3_OFFSET    EQU  0x8
FTFA_FlashConfig_FPROT2_OFFSET    EQU  0x9
FTFA_FlashConfig_FPROT1_OFFSET    EQU  0xA
FTFA_FlashConfig_FPROT0_OFFSET    EQU  0xB
FTFA_FlashConfig_FSEC_OFFSET      EQU  0xC
FTFA_FlashConfig_FOPT_OFFSET      EQU  0xD
FTFA_FlashConfig_BACKKEY3         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY3_OFFSET)
FTFA_FlashConfig_BACKKEY2         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY2_OFFSET)
FTFA_FlashConfig_BACKKEY1         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY1_OFFSET)
FTFA_FlashConfig_BACKKEY0         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY0_OFFSET)
FTFA_FlashConfig_BACKKEY7         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY7_OFFSET)
FTFA_FlashConfig_BACKKEY6         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY6_OFFSET)
FTFA_FlashConfig_BACKKEY5         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY5_OFFSET)
FTFA_FlashConfig_BACKKEY4         EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_BACKKEY4_OFFSET)
FTFA_FlashConfig_FPROT3           EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_FPROT3_OFFSET)
FTFA_FlashConfig_FPROT2           EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_FPROT2_OFFSET)
FTFA_FlashConfig_FPROT1           EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_FPROT1_OFFSET)
FTFA_FlashConfig_FPROT0           EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_FPROT0_OFFSET)
FTFA_FlashConfig_FSEC             EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_FSEC_OFFSET)
FTFA_FlashConfig_FOPT             EQU  (FTFA_FlashConfig_BASE + FTFA_FlashConfig_FOPT_OFFSET)
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 3
FTFA_FlashConfig_BACKKEY3_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY3_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 2
FTFA_FlashConfig_BACKKEY2_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY2_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 1
FTFA_FlashConfig_BACKKEY1_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY1_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 0
FTFA_FlashConfig_BACKKEY0_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY0_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 7
FTFA_FlashConfig_BACKKEY7_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY7_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 6
FTFA_FlashConfig_BACKKEY6_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY6_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 5
FTFA_FlashConfig_BACKKEY5_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY5_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 4
FTFA_FlashConfig_BACKKEY4_KEY_MASK   EQU  0xFF
FTFA_FlashConfig_BACKKEY4_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 3
FTFA_FlashConfig_FPROT3_PROT_MASK   EQU  0xFF
FTFA_FlashConfig_FPROT3_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 2
FTFA_FlashConfig_FPROT2_PROT_MASK   EQU  0xFF
FTFA_FlashConfig_FPROT2_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 1
FTFA_FlashConfig_FPROT1_PROT_MASK   EQU  0xFF
FTFA_FlashConfig_FPROT1_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 0
FTFA_FlashConfig_FPROT0_PROT_MASK   EQU  0xFF
FTFA_FlashConfig_FPROT0_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Security Register
;7-6:KEYEN=backdoor key security enable
;         :00,01(preferred),11=backdoor key access disabled
;         :10=backdoor key access enabled
;5-4:MEEN=mass erase enable bits
;        :00,01,11=mass erase enabled
;        :10=mass erase disabled
;3-2:FSLACC=Freescale failure analysis access code
;          :00,11=Freescale factory access granted
;          :01,10=Freescale factory access denied
;1-0:SEC=flash security
;       :00,01,11=secure
;       :10=unsecure (standard shipping condition)
FTFA_FlashConfig_FSEC_SEC_MASK      EQU  0x3
FTFA_FlashConfig_FSEC_SEC_SHIFT     EQU  0
FTFA_FlashConfig_FSEC_FSLACC_MASK   EQU  0xC
FTFA_FlashConfig_FSEC_FSLACC_SHIFT  EQU  2
FTFA_FlashConfig_FSEC_MEEN_MASK     EQU  0x30
FTFA_FlashConfig_FSEC_MEEN_SHIFT    EQU  4
FTFA_FlashConfig_FSEC_KEYEN_MASK    EQU  0xC0
FTFA_FlashConfig_FSEC_KEYEN_SHIFT   EQU  6
;---------------------------------------------------------------
;NV FCF Flash Option Register
FTFA_FlashConfig_FOPT_LPBOOT0_MASK         EQU  0x1
FTFA_FlashConfig_FOPT_LPBOOT0_SHIFT        EQU  0
FTFA_FlashConfig_FOPT_NMI_DIS_MASK         EQU  0x4
FTFA_FlashConfig_FOPT_NMI_DIS_SHIFT        EQU  2
FTFA_FlashConfig_FOPT_RESET_PIN_CFG_MASK   EQU  0x8
FTFA_FlashConfig_FOPT_RESET_PIN_CFG_SHIFT  EQU  3
FTFA_FlashConfig_FOPT_LPBOOT1_MASK         EQU  0x10
FTFA_FlashConfig_FOPT_LPBOOT1_SHIFT        EQU  4
FTFA_FlashConfig_FOPT_FAST_INIT_MASK       EQU  0x20
FTFA_FlashConfig_FOPT_FAST_INIT_SHIFT      EQU  5
;---------------------------------------------------------------
;General-purpose input and output (GPIO)
;GPIOx_PDD:  Port x Data Direction Register
;  Bit n:  0=Port x pin n configured as input
;          1=Port x pin n configured as output
;GPIOx_PDIR:  Port x Data Input Register
;  Bit n:  Value read from Port x pin n (if input pin)
;GPIOx_PDOR:  Port x Data Output Register
;  Bit n:  Value written to Port x pin n (if output pin)
;GPIOx_PoOR: Port x operation o direction x Register
;  Operation o:  C=Clear (clear to 0)
;                S=Set (set to 1)
;                T=Toggle (complement)
;  Bit n:  0=Port x pin n not affected
;          1=Port x pin n affected
GPIO_BASE         EQU  0x400FF000
GPIO_PDOR_OFFSET  EQU  0x00
GPIO_PSOR_OFFSET  EQU  0x04
GPIO_PCOR_OFFSET  EQU  0x08
GPIO_PTOR_OFFSET  EQU  0x0C
GPIO_PDIR_OFFSET  EQU  0x10
GPIO_PDDR_OFFSET  EQU  0x14
;GPIOx not present in MKL05Z4.h:  PTx instead
;GPIOx included for compatibility with MKL46Z4.h
GPIOA_OFFSET      EQU  0x00
GPIOB_OFFSET      EQU  0x40
;Port A
GPIOA_BASE        EQU  0x400FF000
GPIOA_PDOR        EQU  (GPIOA_BASE + GPIO_PDOR_OFFSET)
GPIOA_PSOR        EQU  (GPIOA_BASE + GPIO_PSOR_OFFSET)
GPIOA_PCOR        EQU  (GPIOA_BASE + GPIO_PCOR_OFFSET)
GPIOA_PTOR        EQU  (GPIOA_BASE + GPIO_PTOR_OFFSET)
GPIOA_PDIR        EQU  (GPIOA_BASE + GPIO_PDIR_OFFSET)
GPIOA_PDDR        EQU  (GPIOA_BASE + GPIO_PDDR_OFFSET)
;Port B
GPIOB_BASE        EQU  0x400FF040
GPIOB_PDOR        EQU  (GPIOB_BASE + GPIO_PDOR_OFFSET)
GPIOB_PSOR        EQU  (GPIOB_BASE + GPIO_PSOR_OFFSET)
GPIOB_PCOR        EQU  (GPIOB_BASE + GPIO_PCOR_OFFSET)
GPIOB_PTOR        EQU  (GPIOB_BASE + GPIO_PTOR_OFFSET)
GPIOB_PDIR        EQU  (GPIOB_BASE + GPIO_PDIR_OFFSET)
GPIOB_PDDR        EQU  (GPIOB_BASE + GPIO_PDDR_OFFSET)
;---------------------------------------------------------------
;IOPORT:  GPIO alias for zero wait state access to GPIO
;See FGPIO
;---------------------------------------------------------------
;Multipurpose clock generator (MCG)
MCG_BASE          EQU  0x40064000
MCG_C1_OFFSET     EQU  0x00
MCG_C2_OFFSET     EQU  0x01
MCG_C3_OFFSET     EQU  0x02
MCG_C4_OFFSET     EQU  0x03
MCG_C6_OFFSET     EQU  0x05
MCG_S_OFFSET      EQU  0x06
MCG_SC_OFFSET     EQU  0x08
MCG_ATCVH_OFFSET  EQU  0x0A
MCG_ATCVL_OFFSET  EQU  0x0B
MCG_C1            EQU  (MCG_BASE + MCG_C1_OFFSET)
MCG_C2            EQU  (MCG_BASE + MCG_C2_OFFSET)
MCG_C3            EQU  (MCG_BASE + MCG_C3_OFFSET)
MCG_C4            EQU  (MCG_BASE + MCG_C4_OFFSET)
MCG_C6            EQU  (MCG_BASE + MCG_C6_OFFSET)
MCG_S             EQU  (MCG_BASE + MCG_S_OFFSET)
MCG_SC            EQU  (MCG_BASE + MCG_SC_OFFSET)
MCG_ATCVH         EQU  (MCG_BASE + MCG_ATCVH_OFFSET)
MCG_ATCVL         EQU  (MCG_BASE + MCG_ATCVL_OFFSET)
;---------------------------------------------------------------
;MCG_C1 MCG Control 1 Register(0x04)
;7-6:CLKS=clock source select (00)
;        :00=output of FLL
;        :01=internal reference clock
;        :10=external reference clock
;        :11=(reserved)
;5-3:FRDIV=FLL external reference divider (000)
;    (depends on MCG_C2.RANGE0)
;         :first divider is for RANGE0=0
;         :second divider is for all other RANGE0 values
;         :000=  1 or   32
;         :001=  2 or   64
;         :010=  4 or  128
;         :011=  8 or  256
;         :100= 16 or  512
;         :101= 32 or 1024
;         :110= 64 or 1280
;         :111=128 or 1536
;  2:IREFS=internal reference select (for FLL) (1)
;         :0=external reference clock
;         :1=slow internal reference clock
;  1:IRCLKEN=internal reference clock (MCGIRCLK) enable (0)
;  0:IREFSTEN=internal reference stop enable (0)
MCG_C1_CLKS_MASK       EQU 0xC0
MCG_C1_CLKS_SHIFT      EQU 6
MCG_C1_FRDIV_MASK      EQU 0x38
MCG_C1_FRDIV_SHIFT     EQU 3
MCG_C1_IREFS_MASK      EQU 0x04
MCG_C1_IREFS_SHIFT     EQU 2
MCG_C1_IRCLKEN_MASK    EQU 0x02
MCG_C1_IRCLKEN_SHIFT   EQU 1
MCG_C1_IREFSTEN_MASK   EQU 0x01
MCG_C1_IREFSTEN_SHIFT  EQU 0
;---------------------------------------------------------------
;MCG_C2 MCG Control 2 Register(0xC0)
;  7:LOCRE0=loss of clock reset enable (1)
;          :0=interrupt request on loss of OCS0 external reference clock
;          :1=reset request on loss of OCS0 external reference clock
;  6:(reserved):read-only:0
;5-4:RANGE0=frequency range select (00)
;          :00=low frequency range for crystal oscillator
;          :01=high frequency range for crystal oscillator
;          :1X=very high frequency range for crystal oscillator
;  3:HGO0=high gain oscillator select (0)
;        :0=low-power operation
;        :1=high-gain operation
;  2:EREFS0=external reference select (0)
;          :0=external reference clock
;          :1=oscillator
;  1:LP=low power select (0)
;      :0=FLL or PLL not disabled in bypass modes
;      :1=FLL or PLL disabled in bypass modes (lower power)
;  0:IRCS=internal reference clock select (0)
;        :0=slow internal reference clock
;        :1=fast internal reference clock
MCG_C2_LOCRE0_MASK        EQU  0x80
MCG_C2_LOCRE0_SHIFT       EQU  7
MCG_C2_RANGE0_MASK        EQU  0x30
MCG_C2_RANGE0_SHIFT       EQU  4
MCG_C2_HGO0_MASK          EQU  0x08
MCG_C2_HGO0_SHIFT         EQU  3
MCG_C2_EREFS0_MASK        EQU  0x04
MCG_C2_EREFS0_SHIFT       EQU  2
MCG_C2_LP_MASK            EQU  0x02
MCG_C2_LP_SHIFT           EQU  1
MCG_C2_IRCS_MASK          EQU  0x01
MCG_C2_IRCS_SHIFT         EQU  0
;---------------------------------------------------------------
;MCG_C3 MCG Control 3 Register (0xXX)
;7-0:SCTRIM=slow internal reference clock trim setting;
;           on reset, loaded with a factory trim value
MCG_C3_SCTRIM_MASK   EQU  0xFF
MCG_C3_SCTRIM_SHIFT  EQU  0
;---------------------------------------------------------------
;MCG_C4 MCG Control 4 Register (2_000XXXXX)
;  7:DMX32=DCO maximum frequency with 32.768 kHz reference (0)
;         :0=default range of 25%
;         :1=fine-tuned for 32.768 kHz reference
;6-5:DRST_DRS=DCO range select (00)
;            :00=low range (default)
;            :01=mid range
;            :10=mid-high range
;            :11=high range
;4-1:FCTRIM=fast internal reference clock trim setting (XXXX)
;           on reset, loaded with a factory trim value
;  0:SCFTRIM=slow internal reference clock fine trim (X)
;           on reset, loaded with a factory trim value
MCG_C4_DMX32_MASK      EQU  0x80
MCG_C4_DMX32_SHIFT     EQU  7
MCG_C4_DRST_DRS_MASK   EQU  0x60
MCG_C4_DRST_DRS_SHIFT  EQU  5
MCG_C4_FCTRIM_MASK     EQU  0x1E
MCG_C4_FCTRIM_SHIFT    EQU  1
MCG_C4_SCFTRIM_MASK    EQU  0x1
MCG_C4_SCFTRIM_SHIFT   EQU  0
;---------------------------------------------------------------
;MCG_C6 MCG Control 6 Register (0x00)
;7-6:(reserved):read-only:00
;  5:CME=clock monitor enable (0)
;4-0:(reserved):read-only:00000
MCG_C6_CME_MASK      EQU  0x20
MCG_C6_CME_SHIFT     EQU  5
;---------------------------------------------------------------
;MCG_S MCG Status Register (0x10)
;7-5:(reserved):read-only:000
;  4:IREFST=internal reference status (1)
;          :0=FLL source external
;          :1=FLL source internal
;3-2:CLKST=clock mode status (00)
;         :00=FLL
;         :01=internal reference
;         :10=external reference
;         :11=(reserved)
;  1:OSCINIT0=OSC initialization (complete)
;  0:IRCST=internal reference clock status
;         :0=slow (32 kHz)
;         :1=fast (4 MHz)
MCG_S_IREFST_MASK      EQU  0x10
MCG_S_IREFST_SHIFT     EQU  4
MCG_S_CLKST_MASK       EQU  0x0C
MCG_S_CLKST_SHIFT      EQU  2
MCG_S_OSCINIT0_MASK    EQU  0x02
MCG_S_OSCINIT0_SHIFT   EQU  1
MCG_S_IRCST_MASK       EQU  0x01
MCG_S_IRCST_SHIFT      EQU  0
;---------------------------------------------------------------
;MCG_SC:  MCG Status and Control Register (0x02)
;  7:ATME=automatic trim machine enable (0)
;  6:ATMS=automatic trim machine select (0)
;        :0=32-kHz internal reference clock
;        :1=4-MHz internal reference clock
;  5:ATMF=automatic trim machine fail flag (read only) (0)
;  4:FLTPRSRV=FLL filter preserve enable (0)
;3-1:FCRDIV=fast clock internal reference divider (001)
;          :000=  1
;          :001=  2
;          :010=  4
;          :011=  8
;          :100= 16
;          :101= 32
;          :110= 64
;          :111=128
;  0:LOCS0=OSC0 loss of clock status (0)
MCG_SC_ATME_MASK       EQU  0x80
MCG_SC_ATME_SHIFT      EQU  7
MCG_SC_ATMS_MASK       EQU  0x40
MCG_SC_ATMS_SHIFT      EQU  6
MCG_SC_ATMF_MASK       EQU  0x20
MCG_SC_ATMF_SHIFT      EQU  5
MCG_SC_FLTPRSRV_MASK   EQU  0x10
MCG_SC_FLTPRSRV_SHIFT  EQU  4
MCG_SC_FCRDIV_MASK     EQU  0xE
MCG_SC_FCRDIV_SHIFT    EQU  1
MCG_SC_LOCS0_MASK      EQU  0x01
MCG_SC_LOCS0_SHIFT     EQU  0
;---------------------------------------------------------------
;MCG_ATCVH:  MCG Auto Trim Compare Value High Register (0x00)
;7-0:ATCVH=Auto trim machine compare value high (0x00)
MCG_ATCVH_ATCVH_MASK   EQU  0xFF
MCG_ATCVH_ATCVH_SHIFT  EQU  0
;---------------------------------------------------------------
;MCG_ATCVL:  MCG Auto Trim Compare Value Low Register (0x00)
;7-0:ATCVL=Auto trim machine compare value low (0x00)
MCG_ATCVL_ATCVL_MASK   EQU  0xFF
MCG_ATCVL_ATCVL_SHIFT  EQU  0
;---------------------------------------------------------------
;Nonvolatile (flash configuration field)
;NV Peripheral access layer
;Following [3]
NV_BASE             EQU  0x400
NV_BACKKEY3_OFFSET  EQU  0x0
NV_BACKKEY2_OFFSET  EQU  0x1
NV_BACKKEY1_OFFSET  EQU  0x2
NV_BACKKEY0_OFFSET  EQU  0x3
NV_BACKKEY7_OFFSET  EQU  0x4
NV_BACKKEY6_OFFSET  EQU  0x5
NV_BACKKEY5_OFFSET  EQU  0x6
NV_BACKKEY4_OFFSET  EQU  0x7
NV_FPROT3_OFFSET    EQU  0x8
NV_FPROT2_OFFSET    EQU  0x9
NV_FPROT1_OFFSET    EQU  0xA
NV_FPROT0_OFFSET    EQU  0xB
NV_FSEC_OFFSET      EQU  0xC
NV_FOPT_OFFSET      EQU  0xD
NV_BACKKEY3         EQU  (NV_BASE + NV_BACKKEY3_OFFSET)
NV_BACKKEY2         EQU  (NV_BASE + NV_BACKKEY2_OFFSET)
NV_BACKKEY1         EQU  (NV_BASE + NV_BACKKEY1_OFFSET)
NV_BACKKEY0         EQU  (NV_BASE + NV_BACKKEY0_OFFSET)
NV_BACKKEY7         EQU  (NV_BASE + NV_BACKKEY7_OFFSET)
NV_BACKKEY6         EQU  (NV_BASE + NV_BACKKEY6_OFFSET)
NV_BACKKEY5         EQU  (NV_BASE + NV_BACKKEY5_OFFSET)
NV_BACKKEY4         EQU  (NV_BASE + NV_BACKKEY4_OFFSET)
NV_FPROT3           EQU  (NV_BASE + NV_FPROT3_OFFSET)
NV_FPROT2           EQU  (NV_BASE + NV_FPROT2_OFFSET)
NV_FPROT1           EQU  (NV_BASE + NV_FPROT1_OFFSET)
NV_FPROT0           EQU  (NV_BASE + NV_FPROT0_OFFSET)
NV_FSEC             EQU  (NV_BASE + NV_FSEC_OFFSET)
NV_FOPT             EQU  (NV_BASE + NV_FOPT_OFFSET)
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 3
NV_BACKKEY3_KEY_MASK   EQU  0xFF
NV_BACKKEY3_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 2
NV_BACKKEY2_KEY_MASK   EQU  0xFF
NV_BACKKEY2_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 1
NV_BACKKEY1_KEY_MASK   EQU  0xFF
NV_BACKKEY1_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 0
NV_BACKKEY0_KEY_MASK   EQU  0xFF
NV_BACKKEY0_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 7
NV_BACKKEY7_KEY_MASK   EQU  0xFF
NV_BACKKEY7_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 6
NV_BACKKEY6_KEY_MASK   EQU  0xFF
NV_BACKKEY6_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 5
NV_BACKKEY5_KEY_MASK   EQU  0xFF
NV_BACKKEY5_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Backdoor Comparison Key 4
NV_BACKKEY4_KEY_MASK   EQU  0xFF
NV_BACKKEY4_KEY_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 3
NV_FPROT3_PROT_MASK   EQU  0xFF
NV_FPROT3_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 2
NV_FPROT2_PROT_MASK   EQU  0xFF
NV_FPROT2_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 1
NV_FPROT1_PROT_MASK   EQU  0xFF
NV_FPROT1_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Program Protection Byte 0
NV_FPROT0_PROT_MASK   EQU  0xFF
NV_FPROT0_PROT_SHIFT  EQU  0
;---------------------------------------------------------------
;NV FCF Flash Security Register
;7-6:KEYEN=backdoor key security enable
;         :00,01(preferred),11=backdoor key access disabled
;         :10=backdoor key access enabled
;5-4:MEEN=mass erase enable bits
;        :00,01,11=mass erase enabled
;        :10=mass erase disabled
;3-2:FSLACC=Freescale failure analysis access code
;          :00,11=Freescale factory access granted
;          :01,10=Freescale factory access denied
;1-0:SEC=flash security
;       :00,01,11=secure
;       :10=unsecure (standard shipping condition)
NV_FSEC_SEC_MASK      EQU  0x3
NV_FSEC_SEC_SHIFT     EQU  0
NV_FSEC_FSLACC_MASK   EQU  0xC
NV_FSEC_FSLACC_SHIFT  EQU  2
NV_FSEC_MEEN_MASK     EQU  0x30
NV_FSEC_MEEN_SHIFT    EQU  4
NV_FSEC_KEYEN_MASK    EQU  0xC0
NV_FSEC_KEYEN_SHIFT   EQU  6
;---------------------------------------------------------------
;NV FCF Flash Option Register
NV_FOPT_LPBOOT0_MASK         EQU  0x1
NV_FOPT_LPBOOT0_SHIFT        EQU  0
NV_FOPT_NMI_DIS_MASK         EQU  0x4
NV_FOPT_NMI_DIS_SHIFT        EQU  2
NV_FOPT_RESET_PIN_CFG_MASK   EQU  0x8
NV_FOPT_RESET_PIN_CFG_SHIFT  EQU  3
NV_FOPT_LPBOOT1_MASK         EQU  0x10
NV_FOPT_LPBOOT1_SHIFT        EQU  4
NV_FOPT_FAST_INIT_MASK       EQU  0x20
NV_FOPT_FAST_INIT_SHIFT      EQU  5
;---------------------------------------------------------------
;Nested vectored interrupt controller (NVIC)
;Part of system control space (SCS)
NVIC_BASE         EQU  0xE000E100
NVIC_ISER_OFFSET  EQU  0x00
NVIC_ICER_OFFSET  EQU  0x80
NVIC_ISPR_OFFSET  EQU  0x100
NVIC_ICPR_OFFSET  EQU  0x180
NVIC_IPR0_OFFSET  EQU  0x300
NVIC_IPR1_OFFSET  EQU  0x304
NVIC_IPR2_OFFSET  EQU  0x308
NVIC_IPR3_OFFSET  EQU  0x30C
NVIC_IPR4_OFFSET  EQU  0x310
NVIC_IPR5_OFFSET  EQU  0x314
NVIC_IPR6_OFFSET  EQU  0x318
NVIC_IPR7_OFFSET  EQU  0x31C
NVIC_ISER         EQU  (NVIC_BASE + NVIC_ISER_OFFSET)
NVIC_ICER         EQU  (NVIC_BASE + NVIC_ICER_OFFSET)
NVIC_ISPR         EQU  (NVIC_BASE + NVIC_ISPR_OFFSET)
NVIC_ICPR         EQU  (NVIC_BASE + NVIC_ICPR_OFFSET)
NVIC_IPR0         EQU  (NVIC_BASE + NVIC_IPR0_OFFSET)
NVIC_IPR1         EQU  (NVIC_BASE + NVIC_IPR1_OFFSET)
NVIC_IPR2         EQU  (NVIC_BASE + NVIC_IPR2_OFFSET)
NVIC_IPR3         EQU  (NVIC_BASE + NVIC_IPR3_OFFSET)
NVIC_IPR4         EQU  (NVIC_BASE + NVIC_IPR4_OFFSET)
NVIC_IPR5         EQU  (NVIC_BASE + NVIC_IPR5_OFFSET)
NVIC_IPR6         EQU  (NVIC_BASE + NVIC_IPR6_OFFSET)
NVIC_IPR7         EQU  (NVIC_BASE + NVIC_IPR7_OFFSET)
;---------------------------------------------------------------
;NVIC IPR assignments
DMA0_IPR         EQU  NVIC_IPR0  ;DMA channel 0 transfer complete/error interrupt
DMA1_IPR         EQU  NVIC_IPR0  ;DMA channel 1 transfer complete/error interrupt
DMA2_IPR         EQU  NVIC_IPR0  ;DMA channel 2 transfer complete/error interrupt
DMA3_IPR         EQU  NVIC_IPR0  ;DMA channel 3 transfer complete/error interrupt
Reserved20_IPR   EQU  NVIC_IPR1  ;Reserved interrupt 20
FTFA_IPR         EQU  NVIC_IPR1  ;FTFA command complete/read collision interrupt
LVD_LVW_IPR      EQU  NVIC_IPR1  ;Low-voltage detect, low-voltage warning interrupt
LLW_IPR          EQU  NVIC_IPR1  ;Low leakage wakeup interrupt
I2C0_IPR         EQU  NVIC_IPR2  ;I2C0 interrupt
Reserved25_IPR   EQU  NVIC_IPR2  ;Reserved interrupt 25
SPI0_IPR         EQU  NVIC_IPR2  ;SPI0 interrupt
Reserved27_IPR   EQU  NVIC_IPR2  ;Reserved interrupt 27
UART0_IPR        EQU  NVIC_IPR3  ;UART0 status/error interrupt
Reserved29_IPR   EQU  NVIC_IPR3  ;Reserved interrupt 29
Reserved30_IPR   EQU  NVIC_IPR3  ;Reserved interrupt 30
ADC0_IPR         EQU  NVIC_IPR3  ;ADC0 interrupt
CMP0_IPR         EQU  NVIC_IPR4  ;CMP0 interrupt
TPM0_IPR         EQU  NVIC_IPR4  ;TPM0 fault, overflow, and channels interrupt
TPM1_IPR         EQU  NVIC_IPR4  ;TPM1 fault, overflow, and channels interrupt
Reserved35_IPR   EQU  NVIC_IPR4  ;Reserved interrupt 35
RTC_IPR          EQU  NVIC_IPR5  ;RTC alarm interrupt
RTC_Seconds_IPR  EQU  NVIC_IPR5  ;RTC seconds interrupt
PIT_IPR          EQU  NVIC_IPR5  ;PIT interrupt
Reserved39_IPR   EQU  NVIC_IPR5  ;Reserved interrupt 39
Reserved40_IPR   EQU  NVIC_IPR6  ;Reserved interrupt 40
DAC0_IPR         EQU  NVIC_IPR6  ;DAC0 interrupt
TSI0_IPR         EQU  NVIC_IPR6  ;TSI0 interrupt
MCG_IPR          EQU  NVIC_IPR6  ;MCG interrupt
LPTimer_IPR      EQU  NVIC_IPR7  ;LPTMR0 interrupt
Reserved45_IPR   EQU  NVIC_IPR7  ;Reserved interrupt 45
PORTA_IPR        EQU  NVIC_IPR7  ;Port A interrupt
PORTB_IPR        EQU  NVIC_IPR7  ;Port B interrupt
;---------------------------------------------------------------
;NVIC IPR position
;priority is a 2-bit value (0-3)
;position EQUates are for LSB of priority
DMA0_PRI_POS         EQU   (8 -__NVIC_PRIO_BITS)  ;DMA channel 0 transfer complete/error interrupt
DMA1_PRI_POS         EQU  (16 -__NVIC_PRIO_BITS)  ;DMA channel 1 transfer complete/error interrupt
DMA2_PRI_POS         EQU  (24 -__NVIC_PRIO_BITS)  ;DMA channel 2 transfer complete/error interrupt
DMA3_PRI_POS         EQU  (32 -__NVIC_PRIO_BITS)  ;DMA channel 3 transfer complete/error interrupt
Reserved20_PRI_POS   EQU   (8 -__NVIC_PRIO_BITS)  ;Reserved interrupt 20
FTFA_PRI_POS         EQU  (16 -__NVIC_PRIO_BITS)  ;FTFA command complete/read collision interrupt
LVD_LVW_PRI_POS      EQU  (24 -__NVIC_PRIO_BITS)  ;Low-voltage detect, low-voltage warning interrupt
LLW_PRI_POS          EQU  (32 -__NVIC_PRIO_BITS)  ;Low leakage wakeup interrupt
I2C0_PRI_POS         EQU   (8 -__NVIC_PRIO_BITS)  ;I2C0 interrupt
Reserved25_PRI_POS   EQU  (16 -__NVIC_PRIO_BITS)  ;Reserved interrupt 25
SPI0_PRI_POS         EQU  (24 -__NVIC_PRIO_BITS)  ;SPI0 interrupt
Reserved27_PRI_POS   EQU  (32 -__NVIC_PRIO_BITS)  ;Reserved interrupt 27
UART0_PRI_POS        EQU   (8 -__NVIC_PRIO_BITS)  ;UART0 status/error interrupt
Reserved29_PRI_POS   EQU  (16 -__NVIC_PRIO_BITS)  ;Reserved interrupt 29
Reserved30_PRI_POS   EQU  (24 -__NVIC_PRIO_BITS)  ;Reserved interrupt 30
ADC0_PRI_POS         EQU  (32 -__NVIC_PRIO_BITS)  ;ADC0 interrupt
CMP0_PRI_POS         EQU   (8 -__NVIC_PRIO_BITS)  ;CMP0 interrupt
TPM0_PRI_POS         EQU  (16 -__NVIC_PRIO_BITS)  ;TPM0 fault, overflow, and channels interrupt
TPM1_PRI_POS         EQU  (24 -__NVIC_PRIO_BITS)  ;TPM1 fault, overflow, and channels interrupt
Reserved35_PRI_POS   EQU  (32 -__NVIC_PRIO_BITS)  ;Reserved interrupt 35
RTC_PRI_POS          EQU   (8 -__NVIC_PRIO_BITS)  ;RTC alarm interrupt
RTC_Seconds_PRI_POS  EQU  (16 -__NVIC_PRIO_BITS)  ;RTC seconds interrupt
PIT_PRI_POS          EQU  (24 -__NVIC_PRIO_BITS)  ;PIT interrupt
Reserved39_PRI_POS   EQU  (32 -__NVIC_PRIO_BITS)  ;Reserved interrupt 39
Reserved40_PRI_POS   EQU   (8 -__NVIC_PRIO_BITS)  ;Reserved interrupt 40
DAC0_PRI_POS         EQU  (16 -__NVIC_PRIO_BITS)  ;DAC0 interrupt
TSI0_PRI_POS         EQU  (24 -__NVIC_PRIO_BITS)  ;TSI0 interrupt
MCG_PRI_POS          EQU  (32 -__NVIC_PRIO_BITS)  ;MCG interrupt
LPTimer_PRI_POS      EQU   (8 -__NVIC_PRIO_BITS)  ;LPTMR0 interrupt
Reserved45_PRI_POS   EQU  (16 -__NVIC_PRIO_BITS)  ;Reserved interrupt 45
PORTA_PRI_POS        EQU  (24 -__NVIC_PRIO_BITS)  ;Port A interrupt
PORTB_PRI_POS        EQU  (32 -__NVIC_PRIO_BITS)  ;Port B interrupt
;---------------------------------------------------------------
;NVIC IRQ masks for ICER, ISER, ICPR, and ISPR
DMA0_IRQ_MASK         EQU  (1 << DMA0_IRQn      )   ;DMA channel 0 transfer complete/error interrupt
DMA1_IRQ_MASK         EQU  (1 << DMA1_IRQn      )   ;DMA channel 1 transfer complete/error interrupt
DMA2_IRQ_MASK         EQU  (1 << DMA2_IRQn      )   ;DMA channel 2 transfer complete/error interrupt
DMA3_IRQ_MASK         EQU  (1 << DMA3_IRQn      )   ;DMA channel 3 transfer complete/error interrupt
Reserved20_IRQ_MASK   EQU  (1 << Reserved20_IRQn)   ;Reserved interrupt 20
FTFA_IRQ_MASK         EQU  (1 << FTFA_IRQn      )   ;FTFA command complete/read collision interrupt
LVD_LVW_IRQ_MASK      EQU  (1 << LVD_LVW_IRQn   )   ;Low-voltage detect, low-voltage warning interrupt
LLW_IRQ_MASK          EQU  (1 << LLW_IRQn       )   ;Low leakage wakeup interrupt
I2C0_IRQ_MASK         EQU  (1 << I2C0_IRQn      )   ;I2C0 interrupt
Reserved25_IRQ_MASK   EQU  (1 << Reserved25_IRQn)   ;Reserved interrupt 25
SPI0_IRQ_MASK         EQU  (1 << SPI0_IRQn      )   ;SPI0 interrupt
Reserved27_IRQ_MASK   EQU  (1 << Reserved27_IRQn)   ;Reserved interrupt 27
UART0_IRQ_MASK        EQU  (1 << UART0_IRQn     )   ;UART0 status/error interrupt
Reserved29_IRQ_MASK   EQU  (1 << Reserved29_IRQn)   ;Reserved interrupt 29
Reserved30_IRQ_MASK   EQU  (1 << Reserved30_IRQn)   ;Reserved interrupt 30
ADC0_IRQ_MASK         EQU  (1 << ADC0_IRQn      )   ;ADC0 interrupt
CMP0_IRQ_MASK         EQU  (1 << CMP0_IRQn      )   ;CMP0 interrupt
TPM0_IRQ_MASK         EQU  (1 << TPM0_IRQn      )   ;TPM0 fault, overflow, and channels interrupt
TPM1_IRQ_MASK         EQU  (1 << TPM1_IRQn      )   ;TPM1 fault, overflow, and channels interrupt
Reserved35_IRQ_MASK   EQU  (1 << Reserved35_IRQn)   ;Reserved interrupt 35
RTC_IRQ_MASK          EQU  (1 << RTC_IRQn       )   ;RTC alarm interrupt
RTC_Seconds_IRQ_MASK  EQU  (1 << RTC_Seconds_IRQn)  ;RTC seconds interrupt
PIT_IRQ_MASK          EQU  (1 << PIT_IRQn       )   ;PIT interrupt
Reserved39_IRQ_MASK   EQU  (1 << Reserved39_IRQn)   ;Reserved interrupt 39
Reserved40_IRQ_MASK   EQU  (1 << Reserved40_IRQn)   ;Reserved interrupt 40
DAC0_IRQ_MASK         EQU  (1 << DAC0_IRQn      )   ;DAC0 interrupt
TSI0_IRQ_MASK         EQU  (1 << TSI0_IRQn      )   ;TSI0 interrupt
MCG_IRQ_MASK          EQU  (1 << MCG_IRQn       )   ;MCG interrupt
LPTimer_IRQ_MASK      EQU  (1 << LPTimer_IRQn   )   ;LPTMR0 interrupt
Reserved45_IRQ_MASK   EQU  (1 << Reserved45_IRQn)   ;Reserved interrupt 45
PORTA_IRQ_MASK        EQU  (1 << PORTA_IRQn     )   ;Port A interrupt
PORTB_IRQ_MASK        EQU  (1 << PORTB_IRQn     )   ;Port B interrupt
;---------------------------------------------------------------
;NVIC vectors
Init_SP_Vector      EQU  00  ;End of stack
Reset_Vector        EQU  01  ;Reset
NMI_Vector          EQU  02  ;Non-maskable interrupt (NMI)
Hard_Fault_Vector   EQU  03  ;Hard fault interrupt
Reserved04_Vector   EQU  04  ;(reserved)
Reserved05_Vector   EQU  05  ;(reserved)
Reserved06_Vector   EQU  06  ;(reserved)
Reserved07_Vector   EQU  07  ;(reserved)
Reserved08_Vector   EQU  08  ;(reserved)
Reserved09_Vector   EQU  09  ;(reserved)
Reserved10_Vector   EQU  10  ;(reserved)
SVCall_Vector       EQU  11  ;Supervisor call interrupt (SVCall)
Reserved12_Vector   EQU  12  ;(reserved)
Reserved13_Vector   EQU  13  ;(reserved)
PendSV_Vector       EQU  14  ;Pendable request for system-level service interrupt
                             ;(PendableSrvReq)
SysTick_Vector      EQU  15  ;System tick timer interrupt (SysTick)
DMA0_Vector         EQU  16  ;DMA channel 0 transfer complete/error interrupt
DMA1_Vector         EQU  17  ;DMA channel 1 transfer complete/error interrupt
DMA2_Vector         EQU  18  ;DMA channel 2 transfer complete/error interrupt
DMA3_Vector         EQU  19  ;DMA channel 3 transfer complete/error interrupt
Reserved20_Vector   EQU  20  ;Reserved interrupt 20
FTFA_Vector         EQU  21  ;FTFA command complete/read collision interrupt
LVD_LVW_Vector      EQU  22  ;Low-voltage detect, low-voltage warning interrupt
LLW_Vector          EQU  23  ;Low leakage wakeup interrupt
I2C0_Vector         EQU  24  ;I2C0 interrupt
Reserved25_Vector   EQU  25  ;Reserved interrupt 25
SPI0_Vector         EQU  26  ;SPI0 interrupt
Reserved27_Vector   EQU  27  ;Reserved interrupt 27
UART0_Vector        EQU  28  ;UART0 status/error interrupt
Reserved29_Vector   EQU  29  ;Reserved interrupt 29
Reserved30_Vector   EQU  30  ;Reserved interrupt 30
ADC0_Vector         EQU  31  ;ADC0 interrupt
CMP0_Vector         EQU  32  ;CMP0 interrupt
TPM0_Vector         EQU  33  ;TPM0 fault, overflow, and channels interrupt
TPM1_Vector         EQU  34  ;TPM1 fault, overflow, and channels interrupt
Reserved35_Vector   EQU  35  ;Reserved interrupt 35
RTC_Vector          EQU  36  ;RTC alarm interrupt
RTC_Seconds_Vector  EQU  37  ;RTC seconds interrupt
PIT_Vector          EQU  38  ;PIT interrupt
Reserved39_Vector   EQU  39  ;Reserved interrupt 39
Reserved40_Vector   EQU  40  ;Reserved interrupt 40
DAC0_Vector         EQU  41  ;DAC0 interrupt
TSI0_Vector         EQU  42  ;TSI0 interrupt
MCG_Vector          EQU  43  ;MCG interrupt
LPTimer_Vector      EQU  44  ;LPTMR0 interrupt
Reserved45_Vector   EQU  45  ;Reserved interrupt 45
PORTA_Vector        EQU  46  ;Port A interrupt
PORTB_Vector        EQU  47  ;Port B interrupt
;---------------------------------------------------------------
;OSC
OSC0_BASE       EQU  0x40065000
OSC0_CR_OFFSET  EQU  0
OSC0_CR         EQU  (OSC0_BASE + OSC0_CR_OFFSET)
;---------------------------------------------------------------
;OSC0_CR (0x00)
;7:ERCLKEN=external reference enable, OSCERCLK (0)
;6:(reserved):read-only:0
;5:EREFSTEN=external reference stop enable (0)
;4:(reserved):read-only:0
;3:SC2P=oscillator 2-pF capacitor load configure (0)
;2:SC4P=oscillator 4-pF capacitor load configure (0)
;1:SC8P=oscillator 8-pF capacitor load configure (0)
;0:SC16P=oscillator 16-pF capacitor load configure (0)
OSC_CR_SC16P_MASK      EQU  0x1
OSC_CR_SC16P_SHIFT     EQU  0
OSC_CR_SC8P_MASK       EQU  0x2
OSC_CR_SC8P_SHIFT      EQU  1
OSC_CR_SC4P_MASK       EQU  0x4
OSC_CR_SC4P_SHIFT      EQU  2
OSC_CR_SC2P_MASK       EQU  0x8
OSC_CR_SC2P_SHIFT      EQU  3
OSC_CR_EREFSTEN_MASK   EQU  0x20
OSC_CR_EREFSTEN_SHIFT  EQU  5
OSC_CR_ERCLKEN_MASK    EQU  0x80
OSC_CR_ERCLKEN_SHIFT   EQU  7
;---------------------------------------------------------------
;PIT
PIT_BASE            EQU  0x40037000
PIT_CH0_BASE        EQU  0x40037100
PIT_CH1_BASE        EQU  0x40037110
PIT_LDVAL_OFFSET    EQU 0x00
PIT_CVAL_OFFSET     EQU 0x04
PIT_TCTRL_OFFSET    EQU 0x08
PIT_TFLG_OFFSET     EQU 0x0C
PIT_MCR_OFFSET      EQU  0x00
PIT_LTMR64H_OFFSET  EQU  0xE0
PIT_LTMR64L_OFFSET  EQU  0xE4
PIT_LDVAL0_OFFSET   EQU  0x100
PIT_CVAL0_OFFSET    EQU  0x104
PIT_TCTRL0_OFFSET   EQU  0x108
PIT_TFLG0_OFFSET    EQU  0x10C
PIT_LDVAL1_OFFSET   EQU  0x110
PIT_CVAL1_OFFSET    EQU  0x114
PIT_TCTRL1_OFFSET   EQU  0x118
PIT_TFLG1_OFFSET    EQU  0x11C
PIT_MCR      EQU  (PIT_BASE + PIT_MCR_OFFSET)
PIT_LTMR64H  EQU  (PIT_BASE + PIT_LTMR64H_OFFSET)
PIT_LTMR64L  EQU  (PIT_BASE + PIT_LTMR64L_OFFSET)
PIT_LDVAL0   EQU  (PIT_BASE + PIT_LDVAL0_OFFSET)
PIT_CVAL0    EQU  (PIT_BASE + PIT_CVAL0_OFFSET)
PIT_TCTRL0   EQU  (PIT_BASE + PIT_TCTRL0_OFFSET)
PIT_TFLG0    EQU  (PIT_BASE + PIT_TFLG0_OFFSET)
PIT_LDVAL1   EQU  (PIT_BASE + PIT_LDVAL1_OFFSET)
PIT_CVAL1    EQU  (PIT_BASE + PIT_CVAL1_OFFSET)
PIT_TCTRL1   EQU  (PIT_BASE + PIT_TCTRL1_OFFSET)
PIT_TFLG1    EQU  (PIT_BASE + PIT_TFLG1_OFFSET)
;---------------------------------------------------------------
;PIT_CVALn:  current timer value register (channel n)
;31-0:TVL=current timer value
;---------------------------------------------------------------
;PIT_LDVALn:  timer load value register (channel n)
;31-0:TSV=timer start value
;         PIT chan. n counts down from this value to 0,
;         then sets TIF and loads LDVALn
;---------------------------------------------------------------
;PIT_LTMR64H:  PIT upper lifetime timer register
;for applications chaining timer 0 and timer 1 for 64-bit timer
;31-0:LTH=life timer value (high word)
;         value of timer 1 (CVAL1); read before PIT_LTMR64L
;---------------------------------------------------------------
;PIT_LTMR64L:  PIT lower lifetime timer register
;for applications chaining timer 0 and timer 1 for 64-bit timer
;31-0:LTL=life timer value (low word)
;         value of timer 0 (CVAL0); read after PIT_LTMR64H
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;31-3:(reserved):read-only:0
;   2:(reserved)
;   1:MDIS=module disable (PIT section)
;          RTI timer not affected by this field
;          must be enabled before any other setup
;   0:FRZ=freeze:  continue'/stop timers in debug mode
PIT_MCR_MDIS_MASK   EQU  0x00000002
PIT_MCR_MDIS_SHIFT  EQU  1
PIT_MCR_FRZ_MASK    EQU  0x00000001
PIT_MCR_FRZ_SHIFT   EQU  0
;---------------------------------------------------------------
;PIT_TCTRLn:  timer control register (channel n)
;31-3:(reserved):read-only:0
;   2:CHN=chain mode (enable)
;          in chain mode, channel n-1 must expire before
;                         channel n counts
;          timer 0 cannot be changed
;   1:TIE=timer interrupt enable (on TIF)
;   0:TEN=timer enable
PIT_TCTRL_CHN_MASK   EQU  0x00000004
PIT_TCTRL_CHN_SHIFT  EQU  2
PIT_TCTRL_TIE_MASK   EQU  0x00000002
PIT_TCTRL_TIE_SHIFT  EQU  1
PIT_TCTRL_TEN_MASK   EQU  0x00000001
PIT_TCTRL_TEN_SHIFT  EQU  0
;---------------------------------------------------------------
;PIT_TFLGn:  timer flag register (channel n)
;31-1:(reserved):read-only:0
;   0:TIF=timer interrupt flag
;         write 1 to clear
PIT_TFLG_TIF_MASK   EQU  0x00000001
PIT_TFLG_TIF_SHIFT  EQU  0
;---------------------------------------------------------------
;Port A
PORTA_BASE         EQU  0x40049000
PORTA_PCR0_OFFSET  EQU  0x00
PORTA_PCR1_OFFSET  EQU  0x04
PORTA_PCR2_OFFSET  EQU  0x08
PORTA_PCR3_OFFSET  EQU  0x0C
PORTA_PCR4_OFFSET  EQU  0x10
PORTA_PCR5_OFFSET  EQU  0x14
PORTA_PCR6_OFFSET  EQU  0x18
PORTA_PCR7_OFFSET  EQU  0x1C
PORTA_PCR8_OFFSET  EQU  0x20
PORTA_PCR9_OFFSET  EQU  0x24
PORTA_PCR10_OFFSET EQU  0x28
PORTA_PCR11_OFFSET EQU  0x2C
PORTA_PCR12_OFFSET EQU  0x30
PORTA_PCR13_OFFSET EQU  0x34
PORTA_PCR14_OFFSET EQU  0x38
PORTA_PCR15_OFFSET EQU  0x3C
PORTA_PCR16_OFFSET EQU  0x40
PORTA_PCR17_OFFSET EQU  0x44
PORTA_PCR18_OFFSET EQU  0x48
PORTA_PCR19_OFFSET EQU  0x4C
PORTA_PCR20_OFFSET EQU  0x50
PORTA_PCR21_OFFSET EQU  0x54
PORTA_PCR22_OFFSET EQU  0x58
PORTA_PCR23_OFFSET EQU  0x5C
PORTA_PCR24_OFFSET EQU  0x60
PORTA_PCR25_OFFSET EQU  0x64
PORTA_PCR26_OFFSET EQU  0x68
PORTA_PCR27_OFFSET EQU  0x6C
PORTA_PCR28_OFFSET EQU  0x70
PORTA_PCR29_OFFSET EQU  0x74
PORTA_PCR30_OFFSET EQU  0x78
PORTA_PCR31_OFFSET EQU  0x7C
PORTA_GPCLR_OFFSET EQU  0x80
PORTA_GPCHR_OFFSET EQU  0x84
PORTA_ISFR_OFFSET  EQU  0xA0
PORTA_PCR0         EQU  (PORTA_BASE + PORTA_PCR0_OFFSET)
PORTA_PCR1         EQU  (PORTA_BASE + PORTA_PCR1_OFFSET)
PORTA_PCR2         EQU  (PORTA_BASE + PORTA_PCR2_OFFSET)
PORTA_PCR3         EQU  (PORTA_BASE + PORTA_PCR3_OFFSET)
PORTA_PCR4         EQU  (PORTA_BASE + PORTA_PCR4_OFFSET)
PORTA_PCR5         EQU  (PORTA_BASE + PORTA_PCR5_OFFSET)
PORTA_PCR6         EQU  (PORTA_BASE + PORTA_PCR6_OFFSET)
PORTA_PCR7         EQU  (PORTA_BASE + PORTA_PCR7_OFFSET)
PORTA_PCR8         EQU  (PORTA_BASE + PORTA_PCR8_OFFSET)
PORTA_PCR9         EQU  (PORTA_BASE + PORTA_PCR9_OFFSET)
PORTA_PCR10        EQU  (PORTA_BASE + PORTA_PCR10_OFFSET)
PORTA_PCR11        EQU  (PORTA_BASE + PORTA_PCR11_OFFSET)
PORTA_PCR12        EQU  (PORTA_BASE + PORTA_PCR12_OFFSET)
PORTA_PCR13        EQU  (PORTA_BASE + PORTA_PCR13_OFFSET)
PORTA_PCR14        EQU  (PORTA_BASE + PORTA_PCR14_OFFSET)
PORTA_PCR15        EQU  (PORTA_BASE + PORTA_PCR15_OFFSET)
PORTA_PCR16        EQU  (PORTA_BASE + PORTA_PCR16_OFFSET)
PORTA_PCR17        EQU  (PORTA_BASE + PORTA_PCR17_OFFSET)
PORTA_PCR18        EQU  (PORTA_BASE + PORTA_PCR18_OFFSET)
PORTA_PCR19        EQU  (PORTA_BASE + PORTA_PCR19_OFFSET)
PORTA_PCR20        EQU  (PORTA_BASE + PORTA_PCR20_OFFSET)
PORTA_PCR21        EQU  (PORTA_BASE + PORTA_PCR21_OFFSET)
PORTA_PCR22        EQU  (PORTA_BASE + PORTA_PCR22_OFFSET)
PORTA_PCR23        EQU  (PORTA_BASE + PORTA_PCR23_OFFSET)
PORTA_PCR24        EQU  (PORTA_BASE + PORTA_PCR24_OFFSET)
PORTA_PCR25        EQU  (PORTA_BASE + PORTA_PCR25_OFFSET)
PORTA_PCR26        EQU  (PORTA_BASE + PORTA_PCR26_OFFSET)
PORTA_PCR27        EQU  (PORTA_BASE + PORTA_PCR27_OFFSET)
PORTA_PCR28        EQU  (PORTA_BASE + PORTA_PCR28_OFFSET)
PORTA_PCR29        EQU  (PORTA_BASE + PORTA_PCR29_OFFSET)
PORTA_PCR30        EQU  (PORTA_BASE + PORTA_PCR30_OFFSET)
PORTA_PCR31        EQU  (PORTA_BASE + PORTA_PCR31_OFFSET)
PORTA_GPCLR        EQU  (PORTA_BASE + PORTA_GPCLR_OFFSET)
PORTA_GPCHR        EQU  (PORTA_BASE + PORTA_GPCHR_OFFSET)
PORTA_ISFR         EQU  (PORTA_BASE + PORTA_ISFR_OFFSET)
;---------------------------------------------------------------
;Port B
PORTB_BASE         EQU  0x4004A000
PORTB_PCR0_OFFSET  EQU  0x00
PORTB_PCR1_OFFSET  EQU  0x04
PORTB_PCR2_OFFSET  EQU  0x08
PORTB_PCR3_OFFSET  EQU  0x0C
PORTB_PCR4_OFFSET  EQU  0x10
PORTB_PCR5_OFFSET  EQU  0x14
PORTB_PCR6_OFFSET  EQU  0x18
PORTB_PCR7_OFFSET  EQU  0x1C
PORTB_PCR8_OFFSET  EQU  0x20
PORTB_PCR9_OFFSET  EQU  0x24
PORTB_PCR10_OFFSET EQU  0x28
PORTB_PCR11_OFFSET EQU  0x2C
PORTB_PCR12_OFFSET EQU  0x30
PORTB_PCR13_OFFSET EQU  0x34
PORTB_PCR14_OFFSET EQU  0x38
PORTB_PCR15_OFFSET EQU  0x3C
PORTB_PCR16_OFFSET EQU  0x40
PORTB_PCR17_OFFSET EQU  0x44
PORTB_PCR18_OFFSET EQU  0x48
PORTB_PCR19_OFFSET EQU  0x4C
PORTB_PCR20_OFFSET EQU  0x50
PORTB_PCR21_OFFSET EQU  0x54
PORTB_PCR22_OFFSET EQU  0x58
PORTB_PCR23_OFFSET EQU  0x5C
PORTB_PCR24_OFFSET EQU  0x60
PORTB_PCR25_OFFSET EQU  0x64
PORTB_PCR26_OFFSET EQU  0x68
PORTB_PCR27_OFFSET EQU  0x6C
PORTB_PCR28_OFFSET EQU  0x70
PORTB_PCR29_OFFSET EQU  0x74
PORTB_PCR30_OFFSET EQU  0x78
PORTB_PCR31_OFFSET EQU  0x7C
PORTB_GPCLR_OFFSET EQU  0x80
PORTB_GPCHR_OFFSET EQU  0x84
PORTB_ISFR_OFFSET  EQU  0xA0
PORTB_PCR0         EQU  (PORTB_BASE + PORTB_PCR0_OFFSET)
PORTB_PCR1         EQU  (PORTB_BASE + PORTB_PCR1_OFFSET)
PORTB_PCR2         EQU  (PORTB_BASE + PORTB_PCR2_OFFSET)
PORTB_PCR3         EQU  (PORTB_BASE + PORTB_PCR3_OFFSET)
PORTB_PCR4         EQU  (PORTB_BASE + PORTB_PCR4_OFFSET)
PORTB_PCR5         EQU  (PORTB_BASE + PORTB_PCR5_OFFSET)
PORTB_PCR6         EQU  (PORTB_BASE + PORTB_PCR6_OFFSET)
PORTB_PCR7         EQU  (PORTB_BASE + PORTB_PCR7_OFFSET)
PORTB_PCR8         EQU  (PORTB_BASE + PORTB_PCR8_OFFSET)
PORTB_PCR9         EQU  (PORTB_BASE + PORTB_PCR9_OFFSET)
PORTB_PCR10        EQU  (PORTB_BASE + PORTB_PCR10_OFFSET)
PORTB_PCR11        EQU  (PORTB_BASE + PORTB_PCR11_OFFSET)
PORTB_PCR12        EQU  (PORTB_BASE + PORTB_PCR12_OFFSET)
PORTB_PCR13        EQU  (PORTB_BASE + PORTB_PCR13_OFFSET)
PORTB_PCR14        EQU  (PORTB_BASE + PORTB_PCR14_OFFSET)
PORTB_PCR15        EQU  (PORTB_BASE + PORTB_PCR15_OFFSET)
PORTB_PCR16        EQU  (PORTB_BASE + PORTB_PCR16_OFFSET)
PORTB_PCR17        EQU  (PORTB_BASE + PORTB_PCR17_OFFSET)
PORTB_PCR18        EQU  (PORTB_BASE + PORTB_PCR18_OFFSET)
PORTB_PCR19        EQU  (PORTB_BASE + PORTB_PCR19_OFFSET)
PORTB_PCR20        EQU  (PORTB_BASE + PORTB_PCR20_OFFSET)
PORTB_PCR21        EQU  (PORTB_BASE + PORTB_PCR21_OFFSET)
PORTB_PCR22        EQU  (PORTB_BASE + PORTB_PCR22_OFFSET)
PORTB_PCR23        EQU  (PORTB_BASE + PORTB_PCR23_OFFSET)
PORTB_PCR24        EQU  (PORTB_BASE + PORTB_PCR24_OFFSET)
PORTB_PCR25        EQU  (PORTB_BASE + PORTB_PCR25_OFFSET)
PORTB_PCR26        EQU  (PORTB_BASE + PORTB_PCR26_OFFSET)
PORTB_PCR27        EQU  (PORTB_BASE + PORTB_PCR27_OFFSET)
PORTB_PCR28        EQU  (PORTB_BASE + PORTB_PCR28_OFFSET)
PORTB_PCR29        EQU  (PORTB_BASE + PORTB_PCR29_OFFSET)
PORTB_PCR30        EQU  (PORTB_BASE + PORTB_PCR30_OFFSET)
PORTB_PCR31        EQU  (PORTB_BASE + PORTB_PCR31_OFFSET)
PORTB_GPCLR        EQU  (PORTB_BASE + PORTB_GPCLR_OFFSET)
PORTB_GPCHR        EQU  (PORTB_BASE + PORTB_GPCHR_OFFSET)
PORTB_ISFR         EQU  (PORTB_BASE + PORTB_ISFR_OFFSET)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;31-25:(reserved):read-only:0
;   24:ISF=interrupt status flag; write 1 clears
;23-20:(reserved):read-only:0
;19-16:IRCQ=interrupt configuration
;          :0000=interrupt/DMA request disabled
;          :0001=DMA request on rising edge
;          :0010=DMA request on falling edge
;          :0011=DMA request on either edge
;          :1000=interrupt when logic zero
;          :1001=interrupt on rising edge
;          :1010=interrupt on falling edge
;          :1011=interrupt on either edge
;          :1100=interrupt when logic one
;          :others=reserved
;15-11:(reserved):read-only:0
;10-08:MUX=Pin mux control
;         :000=pin disabled (analog)
;         :001=alternative 1 (GPIO)
;         :010-111=alternatives 2-7 (chip-specific)
;    7:(reserved):read-only:0
;    6:DSE=Drive strength enable
;         :0=low
;         :1=high
;    5:(reserved):read-only:0
;    4:PFE=Passive filter enable
;    3:(reserved):read-only:0
;    2:SRE=Slew rate enable
;         :0=fast
;         :1=slow
;    1:PE=Pull enable
;    0:PS=Pull select (if PE=1)
;        :0=internal pulldown
;        :1=internal pullup
PORT_PCR_ISF_MASK           EQU  0x1000000
PORT_PCR_ISF_SHIFT          EQU  24
PORT_PCR_IRCQ_MASK          EQU  0xF0000
PORT_PCR_IRCQ_SHIFT         EQU  16
PORT_PCR_MUX_MASK           EQU  0x700
PORT_PCR_MUX_SHIFT          EQU  8
PORT_PCR_DSE_MASK           EQU  0x40
PORT_PCR_DSE_SHIFT          EQU  6
PORT_PCR_PFE_MASK           EQU  0x10
PORT_PCR_PFE_SHIFT          EQU  4
PORT_PCR_SRE_MASK           EQU  0x04
PORT_PCR_SRE_SHIFT          EQU  2
PORT_PCR_PE_MASK            EQU  0x02
PORT_PCR_PE_SHIFT           EQU  1
PORT_PCR_PS_MASK            EQU  0x01
PORT_PCR_PS_SHIFT           EQU  0
PORT_PCR_MUX_SELECT_0_MASK  EQU  0x00000000 ;analog
PORT_PCR_MUX_SELECT_1_MASK  EQU  0x00000100 ;GPIO
PORT_PCR_MUX_SELECT_2_MASK  EQU  0x00000200
PORT_PCR_MUX_SELECT_3_MASK  EQU  0x00000300
PORT_PCR_MUX_SELECT_4_MASK  EQU  0x00000400
PORT_PCR_MUX_SELECT_5_MASK  EQU  0x00000500
PORT_PCR_MUX_SELECT_6_MASK  EQU  0x00000600
PORT_PCR_MUX_SELECT_7_MASK  EQU  0x00000700
;---------------------------------------------------------------
;PORTx_GPCLR (Port x global pin control low register) 
;(32-bit write only)
;31-16:GPWE=global pin write enable; write only (0x0000)
;      bit n:  0=PORTx_PCR<n-16> not updated from GPWD
;              1=PORTx_PCR<n-16> is updated from GPWD
; 15-0:GPWD=global pin write data; write only (0x0000)
;      Value written to PORTx_PCR[15:0] selected by GPWE
PORT_GPCLR_GPWE_MASK   EQU  0xFFFF0000
PORT_GPCLR_GPWE_SHIFT  EQU  16
PORT_GPCLR_GPWD_MASK   EQU  0xFFFF
PORT_GPCLR_GPWD_SHIFT  EQU  0
;---------------------------------------------------------------
;PORTx_GPCHR (Port x global pin control high register)
;(32-bit write only)
;31-16:GPWE=global pin write enable; write only (0x0000)
;      bit n:  0=PORTx_PCRn not updated from GPWD
;              1=PORTx_PCRn is updated from GPWD
; 15-0:GPWD=global pin write data; write only (0x0000)
;      Value written to PORTx_PCR[15:0] selected by GPWE
PORT_GPCHR_GPWE_MASK   EQU  0xFFFF0000
PORT_GPCHR_GPWE_SHIFT  EQU  16
PORT_GPCHR_GPWD_MASK   EQU  0xFFFF
PORT_GPCHR_GPWD_SHIFT  EQU  0
;---------------------------------------------------------------
;PORTx_ISFR (Port x interrupt status flag register)
;(bit n read-only if PORTX pin n does not support interrupts)
;(bit n write 1 clears bit n)
;bit n:  0:  PORTx pin n configured interrupt not detected
;        1:  PORTx pin n configured interrupt detected
;            * If used to generate DMA request,
;              automatically cleared on DMA transfer completion
;            * Otherwise, remains 1 until 1 written
;      bit nn:  0=PORTx_PCRnn not updated from GPWD
;               1=PORTx_PCRnn is updated from GPWD
; 15-0:GPWD=global pin write data; write only (0x0000)
;      Value written to PORTx_PCR[15:0] selected by GPWE
PORT_ISFR_ISF_MASK   EQU  0xFFFFFFFF
PORT_ISFR_ISF_SHIFT  EQU  0
;---------------------------------------------------------------
;PTx not present in MKL46Z4.h
;General-purpose input and output (PTx)
;PTx_PDD:  Port x Data Direction Register
;  Bit n:  0=Port x pin n configured as input
;          1=Port x pin n configured as output
;PTx_PDIR:  Port x Data Input Register
;  Bit n:  Value read from Port x pin n (if input pin)
;PTx_PDOR:  Port x Data Output Register
;  Bit n:  Value written to Port x pin n (if output pin)
;PTx_PoOR: Port x operation o direction x Register
;  Operation o:  C=Clear (clear to 0)
;                S=Set (set to 1)
;                T=Toggle (complement)
;  Bit n:  0=Port x pin n not affected
;          1=Port x pin n affected
PT_BASE         EQU  0x400FF000
PT_PDOR_OFFSET  EQU  0x00
PT_PSOR_OFFSET  EQU  0x04
PT_PCOR_OFFSET  EQU  0x08
PT_PTOR_OFFSET  EQU  0x0C
PT_PDIR_OFFSET  EQU  0x10
PT_PDDR_OFFSET  EQU  0x14
PTA_OFFSET      EQU  0x00
PTB_OFFSET      EQU  0x40
;Port A (PTA)
PTA_BASE        EQU  0x400FF000
PTA_PDOR        EQU  (PTA_BASE + PT_PDOR_OFFSET)
PTA_PSOR        EQU  (PTA_BASE + PT_PSOR_OFFSET)
PTA_PCOR        EQU  (PTA_BASE + PT_PCOR_OFFSET)
PTA_PTOR        EQU  (PTA_BASE + PT_PTOR_OFFSET)
PTA_PDIR        EQU  (PTA_BASE + PT_PDIR_OFFSET)
PTA_PDDR        EQU  (PTA_BASE + PT_PDDR_OFFSET)
;Port B (PTB)
PTB_BASE         EQU  0x400FF040
PTB_PDOR         EQU  (PTB_BASE + PT_PDOR_OFFSET)
PTB_PSOR         EQU  (PTB_BASE + PT_PSOR_OFFSET)
PTB_PCOR         EQU  (PTB_BASE + PT_PCOR_OFFSET)
PTB_PTOR         EQU  (PTB_BASE + PT_PTOR_OFFSET)
PTB_PDIR         EQU  (PTB_BASE + PT_PDIR_OFFSET)
PTB_PDDR         EQU  (PTB_BASE + PT_PDDR_OFFSET)
;---------------------------------------------------------------
;IOPORT:  GPIO alias for zero wait state access to GPIO
;See FGPIO
;---------------------------------------------------------------
;System integration module (SIM)
SIM_BASE             EQU  0x40047000
SIM_SOPT1_OFFSET     EQU  0x00
SIM_SOPT1CFG_OFFSET  EQU  0x04
SIM_SOPT2_OFFSET     EQU  0x1004
SIM_SOPT4_OFFSET     EQU  0x100C
SIM_SOPT5_OFFSET     EQU  0x1010
SIM_SOPT7_OFFSET     EQU  0x1018
SIM_SDID_OFFSET      EQU  0x1024
SIM_SCGC4_OFFSET     EQU  0x1034
SIM_SCGC5_OFFSET     EQU  0x1038
SIM_SCGC6_OFFSET     EQU  0x103C
SIM_SCGC7_OFFSET     EQU  0x1040
SIM_CLKDIV1_OFFSET   EQU  0x1044
SIM_FCFG1_OFFSET     EQU  0x104C
SIM_FCFG2_OFFSET     EQU  0x1050
SIM_UIDMH_OFFSET     EQU  0x1058
SIM_UIDML_OFFSET     EQU  0x105C
SIM_UIDL_OFFSET      EQU  0x1060
SIM_COPC_OFFSET      EQU  0x1100
SIM_SRVCOP_OFFSET    EQU  0x1104
SIM_CLKDIV1          EQU  (SIM_BASE + SIM_CLKDIV1_OFFSET)
SIM_COPC             EQU  (SIM_BASE + SIM_COPC_OFFSET)
SIM_FCFG1            EQU  (SIM_BASE + SIM_FCFG1_OFFSET)
SIM_FCFG2            EQU  (SIM_BASE + SIM_FCFG2_OFFSET)
SIM_SCGC4            EQU  (SIM_BASE + SIM_SCGC4_OFFSET) 
SIM_SCGC5            EQU  (SIM_BASE + SIM_SCGC5_OFFSET)
SIM_SCGC6            EQU  (SIM_BASE + SIM_SCGC6_OFFSET)
SIM_SCGC7            EQU  (SIM_BASE + SIM_SCGC7_OFFSET)
SIM_SDID             EQU  (SIM_BASE + SIM_SDID_OFFSET)
SIM_SOPT1            EQU  (SIM_BASE + SIM_SOPT1_OFFSET)
SIM_SOPT1CFG         EQU  (SIM_BASE + SIM_SOPT1CFG_OFFSET)
SIM_SOPT2            EQU  (SIM_BASE + SIM_SOPT2_OFFSET)
SIM_SOPT4            EQU  (SIM_BASE + SIM_SOPT4_OFFSET)
SIM_SOPT5            EQU  (SIM_BASE + SIM_SOPT5_OFFSET)
SIM_SOPT7            EQU  (SIM_BASE + SIM_SOPT7_OFFSET)
SIM_SRVCOP           EQU  (SIM_BASE + SIM_SRVCOP_OFFSET)
SIM_UIDL             EQU  (SIM_BASE + SIM_UIDL_OFFSET)
SIM_UIDMH            EQU  (SIM_BASE + SIM_UIDMH_OFFSET)
SIM_UIDML            EQU  (SIM_BASE + SIM_UIDML_OFFSET)
;---------------------------------------------------------------
;SIM_CLKDIV1
;31-28:OUTDIV1=clock 1 output divider value
;             :set divider for core/system clock,
;             :from which bus/flash clocks are derived
;             :divide by OUTDIV1 + 1
;27-19:Reserved; read-only; always 0
;18-16:OUTDIV4=clock 4 output divider value
;             :sets divider for bus and flash clocks,
;             :relative to core/system clock
;             :divide by OUTDIV4 + 1
;15-00:Reserved; read-only; always 0
SIM_CLKDIV1_OUTDIV1_MASK       EQU 0xF0000000
SIM_CLKDIV1_OUTDIV1_SHIFT      EQU 28
SIM_CLKDIV1_OUTDIV4_MASK       EQU 0x00070000
SIM_CLKDIV1_OUTDIV4_SHIFT      EQU 16
;---------------------------------------------------------------
;SIM_COPC
;31-04:Reserved; read-only; always 0
;03-02:COPT=COP watchdog timeout
;          :00=disabled
;          :01=timeout after 2^5 LPO cycles or 2^13 bus cycles
;          :10=timeout after 2^8 LPO cycles or 2^16 bus cycles
;          :11=timeout after 2^10 LPO cycles or 2^18 bus cycles
;   01:COPCLKS=COP clock select
;             :0=internal 1 kHz
;             :1=bus clock
;   00:COPW=COP windowed mode
COP_COPT_MASK      EQU  0x0000000C
COP_COPT_SHIFT     EQU  2
COP_COPCLKS_MASK   EQU  0x00000002
COP_COPCLKS_SHIFT  EQU  1
COP_COPW_MASK      EQU  0x00000001
COP_COPW_SHIFT     EQU  1
;---------------------------------------------------------------
;SIM_SCGC4
;1->31-28:Reserved; read-only; always 1
;0->27-24:Reserved; read-only; always 0
;0->   23:SPI1=SPI1 clock gate control (disabled)
;0->   22:SPI0=SPI0 clock gate control (disabled)
;0->21-20:Reserved; read-only; always 0
;0->   19:CMP=comparator clock gate control (disabled)
;0->   18:USBOTG=USB clock gate control (disabled)
;0->17-14:Reserved; read-only; always 0
;0->   13:Reserved; read-only; always 0
;0->   12:UART2=UART2 clock gate control (disabled)
;1->   11:UART1=UART1 clock gate control (disabled)
;0->   10:UART0=UART0 clock gate control (disabled)
;0->09-08:Reserved; read-only; always 0
;0->   07:I2C1=I2C1 clock gate control (disabled)
;0->   06:I2C0=I2C0 clock gate control (disabled)
;1->05-04:Reserved; read-only; always 1
;0->03-00:Reserved; read-only; always 0
SIM_SCGC4_SPI0_MASK     EQU  0x00400000
SIM_SCGC4_SPI0_SHIFT    EQU  22
SIM_SCGC4_CMP_MASK      EQU  0x00080000
SIM_SCGC4_CMP_SHIFT     EQU  19
SIM_SCGC4_UART0_MASK    EQU  0x00000400
SIM_SCGC4_UART0_SHIFT   EQU  10
SIM_SCGC4_I2C0_MASK     EQU  0x00000040
SIM_SCGC4_I2C0_SHIFT    EQU  6
;---------------------------------------------------------------
;SIM_SCGC5
;31-20:Reserved; read-only; always 0
;   19:SLCD=segment LCD clock gate control
;18-14:Reserved; read-only; always 0
;   13:PORTE=Port E clock gate control
;   12:PORTD=Port D clock gate control
;   11:PORTC=Port C clock gate control
;   10:PORTB=Port B clock gate control
;    9:PORTA=Port A clock gate control
;08-07:Reserved; read-only; always 1
;    6:Reserved; read-only; always 0
;    5:TSI=TSI access control
;04-02:Reserved; read-only; always 0
;    1:Reserved; read-only; always 0
;    0:LPTMR=Low power timer access control
SIM_SCGC5_PORTB_MASK   EQU  0x00000400
SIM_SCGC5_PORTB_SHIFT  EQU  10
SIM_SCGC5_PORTA_MASK   EQU  0x00000200
SIM_SCGC5_PORTA_SHIFT  EQU  9
SIM_SCGC5_TSI_MASK     EQU  0x00000020
SIM_SCGC5_TSI_SHIFT    EQU  5
SIM_SCGC5_LPTMR_MASK   EQU  0x00000001
SIM_SCGC5_LPTMR_SHIFT  EQU  0
;---------------------------------------------------------------
;SIM_SCGC6
;   31:DAC0=DAC0 clock gate control
;   30:(reserved):read-only:0
;   29:RTC=RTC access control
;   28:(reserved):read-only:0
;   27:ADC0=ADC0 clock gate control
;   26:TPM2=TPM2 clock gate control
;   25:TPM1=TMP1 clock gate control
;   24:TPM0=TMP0 clock gate control
;   23:PIT=PIT clock gate control
;22-16:(reserved)
;   15:(reserved)
;14-02:(reserved)
;    1:DMAMUX=DMA mux clock gate control
;    0:FTF=flash memory clock gate control
SIM_SCGC6_DAC0_MASK     EQU  0x80000000
SIM_SCGC6_DAC0_SHIFT    EQU  31
SIM_SCGC6_RTC_MASK      EQU  0x20000000
SIM_SCGC6_RTC_SHIFT     EQU  29
SIM_SCGC6_ADC0_MASK     EQU  0x08000000
SIM_SCGC6_ADC0_SHIFT    EQU  27
SIM_SCGC6_TPM1_MASK     EQU  0x02000000
SIM_SCGC6_TPM1_SHIFT    EQU  25
SIM_SCGC6_TPM0_MASK     EQU  0x01000000
SIM_SCGC6_TPM0_SHIFT    EQU  24
SIM_SCGC6_PIT_MASK      EQU  0x00800000
SIM_SCGC6_PIT_SHIFT     EQU  23
SIM_SCGC6_DMAMUX_MASK   EQU  0x00000002
SIM_SCGC6_DMAMUX_SHIFT  EQU  1
SIM_SCGC6_FTF_MASK      EQU  0x00000001
SIM_SCGC6_FTF_SHIFT     EQU  0
;---------------------------------------------------------------
;SIM_SOPT1 (POR or LVD:  0x80000000)
;   31:USBREGEN=USB voltage regulator enable (1)
;   30:USBSSTBY=USB voltage regulator standby during stop, VLPS, LLS, and VLLS (0)
;   29:USBVSTBY=USB voltage regulator standby during VLPS and VLLS (0)
;28-20:(reserved):read-only:000000000
;19-18:OSC32KSEL=32K oscillator clock select (00)
;      (ERCLK32K for sLCD, RTC, and LPTMR)
;                00:System oscillator (OSC32KCLK)
;                01:(reserved)
;                10:RTC_CLKIN
;                11:LPO 1kHz
; 17-6:(reserved):read-only:000000000000
;  5-0:(reserved)
SIM_SOPT1_OSC32KSEL_MASK   EQU  0xC0000
SIM_SOPT1_OSC32KSEL_SHIFT  EQU  18
;---------------------------------------------------------------
;SIM_SOPT2
;31-28:(reserved):read-only:0
;27-26:UART0SRC=UART0 clock source select
;               00:clock disabled
;               01:MCGFLLCLK
;               10:OSCERCLK
;               11:MCGIRCLK
;25-24:TPMSRC=TPM clock source select
;             00:clock disabled
;             01:MCGFLLCLK
;             10:OSCERCLK
;             11:MCGIRCLK
;23-19:(reserved):read-only:0
;15- 8:(reserved):read-only:0
; 7- 5:CLKOUTSEL=CLKOUT select
;                000:(reserved)
;                001:(reserved)
;                010:bus clock
;                011:LPO clock (1 KHz)
;                100:MCGIRCLK
;                101:(reserved)
;                110:OSCERCLK
;                110:(reserved)
;    4:RTCCLKOUTSEL=RTC clock out select
;                   0:RTC (1 Hz)
;                   1:OSCERCLK
; 3- 0:(reserved):read-only:0
SIM_SOPT2_UART0SRC_MASK       EQU  0x0C000000
SIM_SOPT2_UART0SRC_SHIFT      EQU  26
SIM_SOPT2_TPMSRC_MASK         EQU  0x03000000
SIM_SOPT2_TPMSRC_SHIFT        EQU  24
SIM_SOPT2_CLKOUTSEL_MASK      EQU  0xE0
SIM_SOPT2_CLKOUTSEL_SHIFT     EQU  5
SIM_SOPT2_RTCCLKOUTSEL_MASK   EQU  0x10
SIM_SOPT2_RTCCLKOUTSEL_SHIFT  EQU  4
;---------------------------------------------------------------
;SIM_SOPT5
;31-20:Reserved; read-only; always 0
;   19:Reserved; read-only; always 0
;   18:UART2ODE=UART2 open drain enable
;   17:UART1ODE=UART1 open drain enable
;   16:UART0ODE=UART0 open drain enable
;15-07:Reserved; read-only; always 0
;   06:UART1TXSRC=UART1 receive data select
;                :0=UART1_RX pin
;                :1=CMP0 output
;05-04:UART1TXSRC=UART1 transmit data select source
;                :00=UART1_TX pin
;                :01=UART1_TX pin modulated with TPM1 channel 0 output
;                :10=UART1_TX pin modulated with TPM2 channel 0 output
;                :11=(reserved)
;   03:Reserved; read-only; always 0
;   02:UART0RXSRC=UART0 receive data select
;                :0=UART0_RX pin
;                :1=CMP0 output
;01-00:UART0TXSRC=UART0 transmit data select source
;                :00=UART0_TX pin
;                :01=UART0_TX pin modulated with TPM1 channel 0 output
;                :10=UART0_TX pin modulated with TPM2 channel 0 output
;                :11=(reserved)
SIM_SOPT5_UART0ODE_MASK     EQU  0x00010000
SIM_SOPT5_UART0ODE_SHIFT    EQU  16
SIM_SOPT5_UART0RXSRC_MASK   EQU  0x00000004
SIM_SOPT5_UART0RXSRC_SHIFT  EQU  2
SIM_SOPT5_UART0TXSRC_MASK   EQU  0x00000001
SIM_SOPT5_UART0TXSRC_SHIFT  EQU  0
;---------------------------------------------------------------
;Timer/PWM modules (TPMx)
TPM_SC_OFFSET      EQU  0x00
TPM_CNT_OFFSET     EQU  0x04
TPM_MOD_OFFSET     EQU  0x08
TPM_C0SC_OFFSET    EQU  0x0C
TPM_C0V_OFFSET     EQU  0x10
TPM_C1SC_OFFSET    EQU  0x14
TPM_C1V_OFFSET     EQU  0x18
TPM_C2SC_OFFSET    EQU  0x1C
TPM_C2V_OFFSET     EQU  0x20
TPM_C3SC_OFFSET    EQU  0x24
TPM_C3V_OFFSET     EQU  0x28
TPM_C4SC_OFFSET    EQU  0x2C
TPM_C4V_OFFSET     EQU  0x30
TPM_C5SC_OFFSET    EQU  0x34
TPM_C5V_OFFSET     EQU  0x38
;TPM_CONTROLS_OFFSET  EQU  TPM_C0SC_OFFSET
;TPM_RESERVED_0_OFFSET  EQU  0x3C
TPM_STATUS_OFFSET  EQU  0x50
;TPM_RESERVED_1_OFFSET  EQU  0x54
TPM_CONF_OFFSET    EQU  0x84
;---------------------------------------------------------------
;Timer/PWM module 0 (TPM0)
TPM0_BASE           EQU  0x40038000
;TPM_BASES           EQU  TPM0_BASE
TPM0_SC      EQU (TPM0_BASE + TPM_SC_OFFSET    )
TPM0_CNT     EQU (TPM0_BASE + TPM_CNT_OFFSET   )
TPM0_MOD     EQU (TPM0_BASE + TPM_MOD_OFFSET   )
TPM0_C0SC    EQU (TPM0_BASE + TPM_C0SC_OFFSET  )
TPM0_C0V     EQU (TPM0_BASE + TPM_C0V_OFFSET   )
TPM0_C1SC    EQU (TPM0_BASE + TPM_C1SC_OFFSET  )
TPM0_C1V     EQU (TPM0_BASE + TPM_C1V_OFFSET   )
TPM0_C2SC    EQU (TPM0_BASE + TPM_C2SC_OFFSET  )
TPM0_C2V     EQU (TPM0_BASE + TPM_C2V_OFFSET   )
TPM0_C3SC    EQU (TPM0_BASE + TPM_C3SC_OFFSET  )
TPM0_C3V     EQU (TPM0_BASE + TPM_C3V_OFFSET   )
TPM0_C4SC    EQU (TPM0_BASE + TPM_C4SC_OFFSET  )
TPM0_C4V     EQU (TPM0_BASE + TPM_C4V_OFFSET   )
TPM0_C5SC    EQU (TPM0_BASE + TPM_C5SC_OFFSET  )
TPM0_C5V     EQU (TPM0_BASE + TPM_C5V_OFFSET   )
;TPM0_CONTROLS  EQU  (TPM0_BASE + TPM_CONTROLS_OFFSET)
;TPM0_RESERVED_0  EQU (TPM0_BASE + TPM_RESERVED_0_OFFSET)
TPM0_STATUS  EQU (TPM0_BASE + TPM_STATUS_OFFSET)
;TPM0_RESERVED_1  EQU (TPM0_BASE + TPM_RESERVED_1_OFFSET)
TPM0_CONF    EQU (TPM0_BASE + TPM_CONF_OFFSET  )
;---------------------------------------------------------------
;Timer/PWM module 1 (TPM1)
TPM1_BASE           EQU  0x40039000
TPM1_SC      EQU (TPM1_BASE + TPM_SC_OFFSET    )
TPM1_CNT     EQU (TPM1_BASE + TPM_CNT_OFFSET   )
TPM1_MOD     EQU (TPM1_BASE + TPM_MOD_OFFSET   )
TPM1_C0SC    EQU (TPM1_BASE + TPM_C0SC_OFFSET  )
TPM1_C0V     EQU (TPM1_BASE + TPM_C0V_OFFSET   )
TPM1_C1SC    EQU (TPM1_BASE + TPM_C1SC_OFFSET  )
TPM1_C1V     EQU (TPM1_BASE + TPM_C1V_OFFSET   )
TPM1_C2SC    EQU (TPM1_BASE + TPM_C2SC_OFFSET  )
TPM1_C2V     EQU (TPM1_BASE + TPM_C2V_OFFSET   )
TPM1_C3SC    EQU (TPM1_BASE + TPM_C3SC_OFFSET  )
TPM1_C3V     EQU (TPM1_BASE + TPM_C3V_OFFSET   )
TPM1_C4SC    EQU (TPM1_BASE + TPM_C4SC_OFFSET  )
TPM1_C4V     EQU (TPM1_BASE + TPM_C4V_OFFSET   )
TPM1_C5SC    EQU (TPM1_BASE + TPM_C5SC_OFFSET  )
TPM1_C5V     EQU (TPM1_BASE + TPM_C5V_OFFSET   )
;TPM1_CONTROLS  EQU  (TPM0_BASE + TPM_CONTROLS_OFFSET)
;TPM1_RESERVED_0  EQU (TPM0_BASE + TPM_RESERVED_0_OFFSET)
TPM1_STATUS  EQU (TPM1_BASE + TPM_STATUS_OFFSET)
;TPM1_RESERVED_1  EQU (TPM0_BASE + TPM_RESERVED_1_OFFSET)
TPM1_CONF    EQU (TPM1_BASE + TPM_CONF_OFFSET  )
;---------------------------------------------------------------
;TPMx_CnSC:  Channel n Status and Control
;31-8:(reserved):read-only:0
;   7:CHF=channel flag (0)
;         set on channel event
;         write 1 to clear
;   6:CHIE=channel interrupt enable (0)
;   5:MSB=channel mode select B (0) (see selection table below)
;   4:MSA=channel mode select A (0) (see selection table below)
;   3:ELSB=edge or level select B (0) (see selection table below)
;   2:ELSA=edge or level select A (0) (see selection table below)
;   1:(reserved):read-only:0
;   0:DMA=DMA enable (0)
;Mode, Edge, and Level Selection
;MSB:MSA | ELSB:ELSA | Mode           | Configuration
;  0 0   |    0 0    | (none)         | channel disabled
;  0 1   |    0 0    | SW compare     | pin not used
;  1 X   |    0 0    | SW compare     | pin not used
;  0 0   |    0 1    | input capture  | rising edge
;  0 0   |    1 0    | input capture  | falling edge
;  0 0   |    1 1    | input capture  | either edge
;  0 1   |    0 1    | output compare | toggle on match
;  0 1   |    1 0    | output compare | clear on match
;  0 1   |    1 1    | output compare | set on match
;  1 0   |    X 1    | PWM            | low pulse
;  1 0   |    1 0    | PWM            | high pulse
;  1 1   |    X 1    | output compare | pulse high on match
;  1 1   |    1 0    | output compare | pulse low on match
TPM_CnSC_CHF_MASK    EQU  0x80
TPM_CnSC_CHF_SHIFT   EQU  7
TPM_CnSC_CHIE_MASK   EQU  0x40
TPM_CnSC_CHIE_SHIFT  EQU  6
TPM_CnSC_MSB_MASK    EQU  0x20
TPM_CnSC_MSB_SHIFT   EQU  5
TPM_CnSC_MSA_MASK    EQU  0x10
TPM_CnSC_MSA_SHIFT   EQU  4
TPM_CnSC_ELSB_MASK   EQU  0x08
TPM_CnSC_ELSB_SHIFT  EQU  3
TPM_CnSC_ELSA_MASK   EQU  0x04
TPM_CnSC_ELSA_SHIFT  EQU  2
TPM_CnSC_CDMA_MASK   EQU  0x01
TPM_CnSC_CDMA_SHIFT  EQU  0
;---------------------------------------------------------------
;TPMx_CnV:  Channel n Value
;31-16:(reserved):read-only:0
;16- 0:VAL (0x0000) (all bytes must be written at the same time)
TPM_CnV_VAL_MASK   EQU 0xFFFF
TPM_CnV_VAL_SHIFT  EQU 0
;---------------------------------------------------------------
;TPMx_CONF:  Configuration
;31-28:(reserved):read-only:0
;27-24:TRGSEL=trigger select (0000)
;             should be changed only when counter disabled
;             0000:EXTRG_IN (external trigger pin input)
;             0001:CMP0 output
;             0010:(reserved)
;             0011:(reserved)
;             0100:PIT trigger 0
;             0101:PIT trigger 1
;             0110:(reserved)
;             0111:(reserved)
;             1000:TPM0 overflow
;             1001:TPM1 overflow
;             1010:TPM2 overflow
;             1011:(reserved)
;             1100:RTC alarm
;             1101:RTC seconds
;             1110:LPTMR trigger
;             1111:(reserved)
;23-19:(reserved):read-only:0
;   18:CROT=counter reload on trigger (0)
;           should be changed only when counter disabled
;   17:CSOO=counter stop on overflow (0)
;           should be changed only when counter disabled
;   16:CSOT=counter start on trigger (0)
;           should be changed only when counter disabled
;15-10:(reserved):read-only:000000
;    9:GTBEEN=global time base enable (0)
;    8:(reserved):read-only:0
; 7- 6:DBGMODE=debug mode (00)
;              00:paused during debug, and during debug
;                 trigger and input capture events ignored
;              01:(reserved)
;              10:(reserved)
;              11:counter continues during debug
;    5:DOZEEN=doze enable (0)
;             0:counter continues during debug
;             1:paused during debug, and during debug
;               trigger and input capture events ignored
; 4- 0:(reserved):read-only:0000
TPM_CONF_TRGSEL_MASK    EQU  0x0F000000
TPM_CONF_TRGSEL_SHIFT   EQU  24
TPM_CONF_CROT_MASK      EQU  0x00040000
TPM_CONF_CROT_SHIFT     EQU  18
TPM_CONF_CSOO_MASK      EQU  0x00020000
TPM_CONF_CSOO_SHIFT     EQU  17
TPM_CONF_CSOT_MASK      EQU  0x00010000
TPM_CONF_CSOT_SHIFT     EQU  16
TPM_CONF_GTBEEN_MASK    EQU  0x200
TPM_CONF_GTBEEN_SHIFT   EQU  9
TPM_CONF_DBGMODE_MASK   EQU  0xC0
TPM_CONF_DBGMODE_SHIFT  EQU  6
TPM_CONF_DOZEEN_MASK    EQU  0x20
TPM_CONF_DOZEEN_SHIFT   EQU  5
;---------------------------------------------------------------
;TPMx_MOD:  Modulo
;31-16:(reserved):read-only:0
;16- 0:MOD (0xFFFF) (all bytes must be written at the same time)
TPM_MOD_MOD_MASK   EQU 0xFFFF
TPM_MOD_MOD_SHIFT  EQU 0
;---------------------------------------------------------------
;TPMx_SC:  Status and Control
;31-9:(reserved):read-only:0
;   8:DMA=DMA enable (0)
;   7:TOF=timer overflow flag (0)
;         set on count after TPMx_CNT = TPMx_MOD
;         write 1 to clear
;   6:TOIE=timer overflow interrupt enable (0)
;   5:CPWMS=center-aligned PWM select
;           0:edge align (up count)
;           1:center align (up-down count)
; 4-3:CMOD=clock mode selection (00)
;          00:counter disabled
;          01:TPMx_CNT increments on every TPMx clock
;          10:TPMx_CNT increments on rising edge of TPMx_EXTCLK
;          11:(reserved)
; 2-0:PS=prescale factor selection (000)
;        can be written only when counter is disabled
;        count clock = CMOD selected clock / 2^PS
TPM_SC_DMA_MASK     EQU 0x100
TPM_SC_DMA_SHIFT    EQU 8
TPM_SC_TOF_MASK     EQU 0x80
TPM_SC_TOF_SHIFT    EQU 7
TPM_SC_TOIE_MASK    EQU 0x40
TPM_SC_TOIE_SHIFT   EQU 6
TPM_SC_CPWMS_MASK   EQU 0x20
TPM_SC_CPWMS_SHIFT  EQU 5
TPM_SC_CMOD_MASK    EQU 0x18
TPM_SC_CMOD_SHIFT   EQU 3
TPM_SC_PS_MASK      EQU 0x07
TPM_SC_PS_SHIFT     EQU 0
;---------------------------------------------------------------
;TPMx_STATUS:  Capture and Compare Status
;31-9:(reserved):read-only:0
;   8:TOF=timer overflow flag=TPMx_SC.TOF: w1c (0)
; 7-6:(reserved):read-only:0
;   5:CH5F=channel 5 flag=TPMx_C5SC.CHF: w1c (0)
;   4:CH4F=channel 4 flag=TPMx_C4SC.CHF: w1c (0)
;   3:CH3F=channel 3 flag=TPMx_C3SC.CHF: w1c (0)
;   2:CH2F=channel 2 flag=TPMx_C2SC.CHF: w1c (0)
;   1:CH1F=channel 1 flag=TPMx_C1SC.CHF: w1c (0)
;   0:CH0F=channel 0 flag=TPMx_C0SC.CHF: w1c (0)
TPM_STATUS_TOF_MASK    EQU 0x100
TPM_STATUS_TOF_SHIFT   EQU 8
TPM_STATUS_CH5F_MASK   EQU 0x20
TPM_STATUS_CH5F_SHIFT  EQU 5
TPM_STATUS_CH4F_MASK   EQU 0x10
TPM_STATUS_CH4F_SHIFT  EQU 4
TPM_STATUS_CH3F_MASK   EQU 0x08
TPM_STATUS_CH3F_SHIFT  EQU 3
TPM_STATUS_CH2F_MASK   EQU 0x04
TPM_STATUS_CH2F_SHIFT  EQU 2
TPM_STATUS_CH1F_MASK   EQU 0x02
TPM_STATUS_CH1F_SHIFT  EQU 1
TPM_STATUS_CH0F_MASK   EQU 0x01
TPM_STATUS_CH0F_SHIFT  EQU 0
;---------------------------------------------------------------
;UART 0
UART0_BASE  EQU  0x4006A000
;UART_BASES  EQU  UART0_BASE
UART0_BDH_OFFSET  EQU  0x00
UART0_BDL_OFFSET  EQU  0x01
UART0_C1_OFFSET   EQU  0x02
UART0_C2_OFFSET   EQU  0x03
UART0_S1_OFFSET   EQU  0x04
UART0_S2_OFFSET   EQU  0x05
UART0_C3_OFFSET   EQU  0x06
UART0_D_OFFSET    EQU  0x07
UART0_MA1_OFFSET  EQU  0x08
UART0_MA2_OFFSET  EQU  0x09
UART0_C4_OFFSET   EQU  0x0A
UART0_C5_OFFSET   EQU  0x0B
UART0_BDH        EQU  (UART0_BASE + UART0_BDH_OFFSET)
UART0_BDL        EQU  (UART0_BASE + UART0_BDL_OFFSET)
UART0_C1         EQU  (UART0_BASE + UART0_C1_OFFSET)
UART0_C2         EQU  (UART0_BASE + UART0_C2_OFFSET)
UART0_S1         EQU  (UART0_BASE + UART0_S1_OFFSET)
UART0_S2         EQU  (UART0_BASE + UART0_S2_OFFSET)
UART0_C3         EQU  (UART0_BASE + UART0_C3_OFFSET)
UART0_D          EQU  (UART0_BASE + UART0_D_OFFSET)
UART0_MA1        EQU  (UART0_BASE + UART0_MA1_OFFSET)
UART0_MA2        EQU  (UART0_BASE + UART0_MA2_OFFSET)
UART0_C4         EQU  (UART0_BASE + UART0_C4_OFFSET)
UART0_C5         EQU  (UART0_BASE + UART0_C5_OFFSET)
;---------------------------------------------------------------
;UART0_BDH
;  7:LBKDIE=LIN break detect IE
;  6:RXEDGIE=RxD input active edge IE
;  5:SBNS=Stop bit number select
;4-0:SBR[12:0] (BUSCLK / (16 x 9600))
UART0_BDH_LBKDIE_MASK    EQU  0x80
UART0_BDH_LBKDIE_SHIFT   EQU  7
UART0_BDH_RXEDGIE_MASK   EQU  0x40
UART0_BDH_RXEDGIE_SHIFT  EQU  6
UART0_BDH_SBNS_MASK      EQU  0x20
UART0_BDH_SBNS_SHIFT     EQU  5
UART0_BDH_SBR_MASK       EQU  0x1F
UART0_BDH_SBR_SHIFT      EQU  0
;---------------------------------------------------------------
;UART0_BDL
;7-0:SBR[7:0] (BUSCLK / 16 x 9600))
UART0_BDL_SBR_MASK   EQU  0xFF
UART0_BDL_SBR_SHIFT  EQU  0
;---------------------------------------------------------------
;UART0_C1
;7:LOOPS=loop mode select (normal)
;6:DOZEEN=UART disabled in wait mode (enabled)
;5:RSRC=receiver source select (internal--no effect LOOPS=0)
;4:M=9- or 8-bit mode select (1 start, 8 data [lsb first], 1 stop)
;3:WAKE=receiver wakeup method select (idle)
;2:IDLE=idle line type select (idle begins after start bit)
;1:PE=parity enable (disabled)
;0:PT=parity type (even parity--no effect PE=0)
UART0_C1_LOOPS_MASK      EQU  0x80
UART0_C1_LOOPS_SHIFT     EQU  7
UART0_C1_DOZEEN_MASK     EQU  0x40
UART0_C1_DOZEEN_SHIFT    EQU  6
UART0_C1_RSRC_MASK       EQU  0x20
UART0_C1_RSRC_SHIFT      EQU  5
UART0_C1_M_MASK          EQU  0x10
UART0_C1_M_SHIFT         EQU  4
UART0_C1_WAKE_MASK       EQU  0x08
UART0_C1_WAKE_SHIFT      EQU  3
UART0_C1_ILT_MASK        EQU  0x04
UART0_C1_ILT_SHIFT       EQU  2
UART0_C1_PE_MASK         EQU  0x02
UART0_C1_PE_SHIFT        EQU  1
UART0_C1_PT_MASK         EQU  0x01
UART0_C1_PT_SHIFT        EQU  0
;---------------------------------------------------------------
;UART0_C2
;7:TIE=transmitter IE for TDRE (disabled)
;6:TCIE=trasmission complete IE for TC (disabled)
;5:RIE=receiver IE for RDRF (disabled)
;4:ILIE=idle line IE for IDLE (disabled)
;3:TE=transmitter enable (disabled)
;2:RE=receiver enable (disabled)
;1:RWU=receiver wakeup control (normal)
;0:SBK=send break (disabled, normal)
UART0_C2_TIE_MASK    EQU  0x80
UART0_C2_TIE_SHIFT   EQU  7
UART0_C2_TCIE_MASK   EQU  0x40
UART0_C2_TCIE_SHIFT  EQU  6
UART0_C2_RIE_MASK    EQU  0x20
UART0_C2_RIE_SHIFT   EQU  5
UART0_C2_ILIE_MASK   EQU  0x10
UART0_C2_ILIE_SHIFT  EQU  4
UART0_C2_TE_MASK     EQU  0x08
UART0_C2_TE_SHIFT    EQU  3
UART0_C2_RE_MASK     EQU  0x04
UART0_C2_RE_SHIFT    EQU  2
UART0_C2_RWU_MASK    EQU  0x02
UART0_C2_RWU_SHIFT   EQU  1
UART0_C2_SBK_MASK    EQU  0x01
UART0_C2_SBK_SHIFT   EQU  0
;---------------------------------------------------------------
;UART0_C3
;7:R8T9=Receive bit 8; transmit bit 9 (not used M=0)
;6:R9T8=Receive bit 9; transmit bit 8 (not used M=0)
;5:TXDIR=TxD pin direction in single-wire mode 
;                        (input--no effect LOOPS=0)
;4:TXINV=transmit data inversion (not invereted)
;3:ORIE=overrun IE for OR (disabled)
;2:NEIE=noise error IE for NF (disabled)
;1:FEIE=framing error IE for FE (disabled)
;0:PEIE=parity error IE for PF (disabled)
UART0_C3_R8T9_MASK    EQU  0x80
UART0_C3_R8T9_SHIFT   EQU  7
UART0_C3_R9T8_MASK    EQU  0x40
UART0_C3_R9T8_SHIFT   EQU  6
UART0_C3_TXDIR_MASK   EQU  0x20
UART0_C3_TXDIR_SHIFT  EQU  5
UART0_C3_TXINV_MASK   EQU  0x10
UART0_C3_TXINV_SHIFT  EQU  4
UART0_C3_ORIE_MASK    EQU  0x08
UART0_C3_ORIE_SHIFT   EQU  3
UART0_C3_NEIE_MASK    EQU  0x04
UART0_C3_NEIE_SHIFT   EQU  2
UART0_C3_FEIE_MASK    EQU  0x02
UART0_C3_FEIE_SHIFT   EQU  1
UART0_C3_PEIE_MASK    EQU  0x01
UART0_C3_PEIE_SHIFT   EQU  0
;---------------------------------------------------------------
;UART0_C4
;  7:MAEN1=Match address mode enable 1 (disabled)
;  6:MAEN2=Match address mode enable 2 (disabled)
;  5:M10=10-bit mode select (not selected)
;4-0:OSR=Over sampling ratio (01111)
;        00000 <= OSR <= 00010:  (invalid; defaults to ratio = 16)        
;        00011 <= OSR <= 11111:  ratio = OSR + 1
UART0_C4_MAEN1_MASK   EQU  0x80
UART0_C4_MAEN1_SHIFT  EQU  7
UART0_C4_MAEN2_MASK   EQU  0x40
UART0_C4_MAEN2_SHIFT  EQU  6
UART0_C4_M10_MASK     EQU  0x20
UART0_C4_M10_SHIFT    EQU  5
UART0_C4_OSR_MASK     EQU  0x1F
UART0_C4_OSR_SHIFT    EQU  0
;---------------------------------------------------------------
;UART0_C5
;  7:TDMAE=Transmitter DMA enable (disabled)
;  6:(reserved):  read-only:  0
;  5:RDMAE=Receiver full DMA enable (disabled)
;4-2:(reserved):  read-only:  000
;  1:BOTHEDGE=Both edge sampling (only rising edge)
;  0:RESYNCDIS=Resynchronization disable (enabled)
UART0_C5_TDMAE_MASK       EQU  0x80
UART0_C5_TDMAE_SHIFT      EQU  7
UART0_C5_RDMAE_MASK       EQU  0x20
UART0_C5_RDMAE_SHIFT      EQU  5
UART0_C5_BOTHEDGE_MASK    EQU  0x02
UART0_C5_BOTHEDGE_SHIFT   EQU  1
UART0_C5_RESYNCDIS_MASK   EQU  0x01
UART0_C5_RESYNCDIS_SHIFT  EQU  0
;---------------------------------------------------------------
;UART0_D
;7:R7T7=Receive data buffer bit 7; 
;       transmit data buffer bit 7
;6:R6T6=Receive data buffer bit 6; 
;       transmit data buffer bit 6
;5:R5T5=Receive data buffer bit 5; 
;       transmit data buffer bit 5
;4:R4T4=Receive data buffer bit 4; 
;       transmit data buffer bit 4
;3:R3T3=Receive data buffer bit 3; 
;       transmit data buffer bit 3
;2:R2T2=Receive data buffer bit 2; 
;       transmit data buffer bit 2
;1:R1T1=Receive data buffer bit 1; 
;       transmit data buffer bit 1
;0:R0T0=Receive data buffer bit 0; 
;       transmit data buffer bit 0
UART0_D_R7T7_MASK   EQU  0x80
UART0_D_R7T7_SHIFT  EQU  7
UART0_D_R6T6_MASK   EQU  0x40
UART0_D_R6T6_SHIFT  EQU  6
UART0_D_R5T5_MASK   EQU  0x20
UART0_D_R5T5_SHIFT  EQU  5
UART0_D_R4T4_MASK   EQU  0x10
UART0_D_R4T4_SHIFT  EQU  4
UART0_D_R3T3_MASK   EQU  0x08
UART0_D_R3T3_SHIFT  EQU  3
UART0_D_R2T2_MASK   EQU  0x04
UART0_D_R2T2_SHIFT  EQU  2
UART0_D_R1T1_MASK   EQU  0x02
UART0_D_R1T1_SHIFT  EQU  1
UART0_D_R0T0_MASK   EQU  0x01
UART0_D_R0T0_SHIFT  EQU  0
;---------------------------------------------------------------
;UART0_MA1
;7-0:MA=Match address
UART0_MA1_MA_MASK   EQU  0xFF
UART0_MA1_MA_SHIFT  EQU  0
;---------------------------------------------------------------
;UART0_MA2
;7-0:MA=Match address
UART0_MA2_MA_MASK   EQU  0xFF
UART0_MA2_MA_SHIFT  EQU  0
;---------------------------------------------------------------
;UART0_S1
;7:TDRE=transmit data register empty flag
;6:TC=transmission complete flag
;5:RDRF=receive data register full flag
;4:IDLE=idle line flag
;3:OR=receiver overrun flag
;2:NF=noise flag
;1:FE=framing error flag
;0:PF=parity error flag
UART0_S1_TDRE_MASK   EQU 0x80
UART0_S1_TDRE_SHIFT  EQU 7
UART0_S1_TC_MASK     EQU 0x40
UART0_S1_TC_SHIFT    EQU 6
UART0_S1_RDRF_MASK   EQU 0x20
UART0_S1_RDRF_SHIFT  EQU 5
UART0_S1_IDLE_MASK   EQU 0x10
UART0_S1_IDLE_SHIFT  EQU 4
UART0_S1_OR_MASK     EQU 0x08
UART0_S1_OR_SHIFT    EQU 3
UART0_S1_NF_MASK     EQU 0x04
UART0_S1_NF_SHIFT    EQU 2
UART0_S1_FE_MASK     EQU 0x02
UART0_S1_FE_SHIFT    EQU 1
UART0_S1_PF_MASK     EQU 0x01
UART0_S1_PF_SHIFT    EQU 0
;---------------------------------------------------------------
;UART0_S2
;7:LBKDIF=LIN break detect interrupt flag
;6:RXEDGIF=RxD pin active edge interrupt flag
;5:MSBF=MSB first (LSB first)
;4:RXINV=receive data inversion (not inverted)
;3:RWUID=receive wake-up idle detect (not detected)
;2:BRK13=break character generation length (10 bit times)
;1:LBKDE=LIN break detect enable (10 bit times)
;0:RAF=receiver active flag
UART0_S2_LBKDIF_MASK   EQU 0x80
UART0_S2_LBKDIF_SHIFT  EQU 7
UART0_S2_RXEDGIF_MASK  EQU 0x40
UART0_S2_RXEDGIF_SHIFT EQU 6
UART0_S2_MSBF_MASK     EQU 0x20
UART0_S2_MSBF_SHIFT    EQU 5
UART0_S2_RXINV_MASK    EQU 0x10
UART0_S2_RXINV_SHIFT   EQU 4
UART0_S2_RWUID_MASK    EQU 0x08
UART0_S2_RWUID_SHIFT   EQU 3
UART0_S2_BRK13_MASK    EQU 0x04
UART0_S2_BRK13_SHIFT   EQU 2
UART0_S2_LBKDE_MASK    EQU 0x02
UART0_S2_LBKDE_SHIFT   EQU 1
UART0_S2_RAF_MASK      EQU 0x01
UART0_S2_RAF_SHIFT     EQU 0
;---------------------------------------------------------------
            END
