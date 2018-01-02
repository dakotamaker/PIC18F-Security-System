  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; SecuritySystem.asm	    Dakota Maker    12.6.17
  ;;  A basic shell to initialize and run Timing and INT interrupts 
  ;;  Inputs: none 
  ;;
  ;;  Outputs: none 
  ;;
  ;;  Side effects: 
  ;;    The High_Priority_ISR executes once every 500 mS
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    LIST P=18F4520		    ;directive to define processor
    #include <P18F4520.INC>	;processor specific variable definitions
    CONFIG  OSC = INTIO67       ; internal clock @ 8MHz (1MHz with prescaler),
    CONFIG  WDT = OFF	        ; watch dog timer OFF
    CONFIG  MCLRE = OFF	        ; MCLEAR (master clear)pin enabled
    CONFIG  PBADEN = OFF        ; PORTB pins digital (disable A/D for PORTB)

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Constants Section 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Standard constants for RSP 
TC_Port:             EQU     PORTA  ;; Port for Timing and Control Code 
TC_Dir:              EQU     TRISA  ;; Direction register for Timing and Control Code 
E_Clk_Port:          EQU     PORTB  ;; Port for E-Clock
SS_PORT:	     EQU     PORTB   ;;Port for Momentary Switch
E_Clk_Dir:           EQU     TRISB  ;; Direction register for E-Clock 
SS_DIR:		     EQU     TRISB   ;;Direction register for Momentary Switch
Data_Bus_Port:       EQU     PORTC  ;; Port for RSP Data Bus 
Data_Bus_Dir:        EQU     TRISC  ;; Direction register for RSP Data Bus 
PIC18_Top_Of_Data_Stack: EQU 0x5FF  ;; Initialization value for PIC18 Data Stack 

  ;; Special constants for CCP1.asm 
CountsPerInterrupt  EQU D'25000'  ;; Increment for CCP1  

 ;; Constants for SecureitySystem.asm
TwoSecondValue	    equ	    0x04	  ; Value to allow to delay 2 sec
SIAddr		    equ	    0x02	  ; Address of Switch Input
RegF_Addr		EQU	0x00	    ; Address of Reg F
RegG_Addr		EQU	0x01	    ; Address of Reg G
SI_Addr			EQU	0x02	    ; Address of Switch input
ALU_Addr		EQU	0x03	    ; Address of ALU
NoLightCode	    equ	    0x00	  ; Clear LEDs 
RedLightCode	    equ	    0x01	  ; Light up Red LED
GreenLightCode	    equ	    0x02	  ; Light up Green LED
InitialPassword	    equ	    0xFF	  ; Password value to be set
InputPortCode	    equ	    0x02	  ; Code to make PortB bit 1 an input
E_Clk_Bit	    equ	    0x00	  ; The 0 bit of PortB
Write_RegF	    equ     0x80	  ; Write to RegF
Read_RegF	    equ     0x08	  ; RegF -> Data Bus

Init_T3CON	    equ     B'1011000'	   ; initial value for T3CON
LatD_0_bit	    equ	    0x00	    
LatD_2_bit	    equ	    0x02
;******************************************************************************
;Timing and Control Codes
;******************************************************************************
RSP_TGF               equ     0x89         ; Transfer RegG -> RegF
RSP_TFG               equ     0x98         ; Transfer RegF-> RegG
RSP_TSF               equ     0x8A         ; Transfer SI -> RegF
RSP_TSG		      equ     0x9A	   ; Transfer SI -> RegG
RSP_ADDF	      equ     0x8B	   ; Add F + G -> RegF
RSP_ADDG	      equ     0x9B	   ; Add F + G -> RegG
Write_RegF	      equ     0x80	   ; Write to RegF
Read_RegF	      equ     0x08	   ; RegF -> Data Bus
Write_RegG	      equ     0x90	   ; Write to RegG
;******************************************************************************
;;Constants for traffic control
;******************************************************************************
Traffic_RG EQU 0x14
Traffic_RY EQU 0x12
Traffic_GR EQU 0x41
Traffic_YR EQU 0x21
Traffic_Entries EQU 0x0B; 11 entries in array
;******************************************************************************
;;Constants for interrupts
;******************************************************************************
T1SETUP EQU B'10100001'
T1PRESET EQU 0xFF
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Variables Section 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    udata_acs  0x00 
T_C_Code:	    RES     1  ;; Space for application
Del_1             res     1                ; Variable used in delay routine
Del_2             res     1                ; Variable used in delay routine
SI_Data:	    RES     1  ;; Space for application
RegG_Data	  res	  1		   ; Variable used to capture RegG data
CCP1_Counter:	    RES     2  ;; Counter for CCP1 interrupt 
Password	    RES	    1 ;; Holds Current Password for Security System
DelayCounter	    RES	    1  ;; Counter for Security System
RSPWrite_Address    RES	    1  ;; RSPWrite_Address for RSPWrite
RSPWrite_Data	    RES	    1  ;; RSPWrite_Data for RSPWrite
RSPSwitch_Status    RES	    1  ;; RSPSwitch_Status for whether button is pressed
Traffic_Sequence RES	32  ;; Max size of array for traffic sequence 
Traffic_Counter RES	1   ;; Traffic sequence counter
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Macros Section 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Macro PUSHF, push a file register onto the data stack
  ;; Usage: PUSHF FileReg
  ;; Side Effect: [FileReg] moved onto stack, [FSR2]-1 -> [FSR2] 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PUSHF: macro FileReg
  MOVFF FileReg, POSTDEC2
  endm

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Macro PULLF, pull data from the stack, and place a file register
  ;; Usage: PULLF FileReg
  ;; Side Effect: [FSR2]-1 -> [FSR2] and *[FSR2] written to [FileReg]
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PULLF: macro FileReg
  MOVFF PREINC2, FileReg
  endm

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Macro INCF16, Increment a 16 bit value, with the low byte at FileReg
  ;; Usage: INCF16 FileReg
  ;; Side Effect: (FileReg+1):FileReg  is incremented
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
INCF16: macro FileReg
  INFSNZ  FileReg,F
  INCF    FileReg+1,F
  endm

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;  Program
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ORG 0x0000               ;; In the PIC18, there are 3 initial "Vectors"
  Goto  Main               ;; That is, places that the hardware jumps to. 
                           ;; Address 0x0000 is the target for a reset

  ORG 0x0008               ;; Address 0x0008 is the target for the 
  Goto  High_Priority_ISR  ;; High-priority interrupt 

  ORG 0x0018               ;; Address 0x0018 is the target for the 
  Goto  Low_Priority_ISR   ;; Low-priority interrupt 
 
Main:
  LFSR FSR2, PIC18_Top_Of_Data_Stack ;; Initialize the data stack 
  Call Init_Timer1_Interrupt           ;; Initialize the CCP1 interrupt 
  Call Sequence_Init
  Call Init_SecuritySystem		;;Initialize variables for Security System
  Call Init_Security_Interrupt           ;; Initialize the CCP1 interrupt 

Loop:                                
  NOP
  GOTO Loop     ;;  Go into an idle loop 


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Init_CCP1_Interrupt   Initialize the CCP1 Interrupt 
  ;; Inputs:  None 
  ;; Outputs: None
  ;; Side effects:  Zero CCP1_Counter, 
  ;;    Set T1CON bits to activate Timer1
  ;;    Set T3Con bits for Timer3 (does not activate) 
  ;;    Set CCP1CON bits, to activate compare mode and interrupt 
  ;;    Set RCON:IPEN bit,  to active PIC18 mode interrupts (high/low priority)
  ;;    Set IPR1 bit,  to make CCP1 interrupt a high-priority interrupt
  ;;    Clear PIR1 bit,  to clear CCP1 interrupt flag 
  ;;    Set PIE1 bit,    to enable CCP1 interrupt 
  ;;    Set INTCON, GIEH to generally enable high-priority interrupts
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init_Timer1_Interrupt:

  PUSHF  WREG           ;; WReg is used

  CLRF   TRISD          ;; Setup port D for pulses to visualize action on OScope

  MOVLW  T1SETUP    ;; Setup    T1CON for Counting  
                        ;; RD16:    b7, latched 16-bit read
                        ;; T1CKPS1: b5, 1:2 prescaler (options, 1:1, 1:2, 1:4, 1:8)
                        ;; T1CKPS0: b4, 1:2 prescaler (options, 1:1, 1:2, 1:4, 1:8)
                        ;; T1SYNC_bar:  b2=0, T1 clock with synchronized with internal phase clock
                        ;; TMR1ON:  b0, Turn timer 1 on 
  MOVWF  T1CON,

  BSF    RCON,IPEN      ;; Active PIC18F High-priority / Low-priority mode
  BCF    IPR1,TMR1IP    ;; Make TMR1 a low-priority interrupt 
  BCF    PIR1,TMR1IF    ;; Clear the TMR1 Interrupt Flag (so that it can be set to generate IRQ)
  BSF    PIE1,TMR1IE    ;; Enable the TMR1 interrupt 
  BSF    INTCON,GIEL    ;; Enable low-priority interrupts 
  BSF    INTCON,GIEH    ;; Enable high-priority interrupts and all interrupts
  MOVFF	 T1PRESET,TMR1H ;; Preset timer to tune value
  
  PULLF  WREG 

  RETURN 

Sequence_Init:
  ;16 Bytes reserved, 1st element is number of entries, sequence is backwards in Data structure
  ;Each sequence entry represents 1 second
  MOVLW Traffic_Entries; The number of entries initialized
  MOVWF Traffic_Sequence ;First element is number of entries in Array
  
  ;Sequence is in reverse order!
  MOVLW Traffic_YR;
  MOVWF Traffic_Sequence+0x01 ;Time = 11, Yellow Red
  MOVLW Traffic_YR;
  MOVWF Traffic_Sequence+0x02 ;Time = 10, Yellow Red
  MOVLW Traffic_GR;
  MOVWF Traffic_Sequence+0x03 ;Time = 09, Green Red
  MOVLW Traffic_GR;
  MOVWF Traffic_Sequence+0x04 ;Time = 08, Green Red
  MOVLW Traffic_GR;
  MOVWF Traffic_Sequence+0x05 ;Time = 07, Green Red
  MOVLW Traffic_RY;
  MOVWF Traffic_Sequence+0x06 ;Time = 06, Red Yellow
  MOVLW Traffic_RY;
  MOVWF Traffic_Sequence+0x07 ;Time = 05, Red Yellow
  MOVLW Traffic_RG;
  MOVWF Traffic_Sequence+0x08 ;Time = 04, Red Green
  MOVLW Traffic_RG;
  MOVWF Traffic_Sequence+0x09 ;Time = 03, Red Green
  MOVLW Traffic_RG;
  MOVWF Traffic_Sequence+0x0A ;Time = 02, Red Green
  MOVLW Traffic_RG;
  MOVWF Traffic_Sequence+0x0B ;Time = 01, Red Green

  
  CLRF TRISA
  CLRF TRISB
  CLRF TRISC
  CLRF TRISD

  RETURN
  
Init_Security_Interrupt:
  PUSHF  WREG           ;; WReg is used

  CLRF   TRISD          ;; Setup port D for pulses to visualize action on OScope

  MOVLW  Init_T3CON    ;; Setup    T3Con for Counting
  MOVWF  T3CON         ;; RD16L:   latched 16-bit read

  BCF	 INTCON2,INTEDG1    ;; Check for falling edge
  BSF    RCON,IPEN          ;; Active PIC18F High-priority / Low-priority mode
  BCF	 INTCON3, INT1IF    ;; Clear Int1 flag
  BCF	 PIR2, TMR3IF	    ;; Clear Timer flag
  BSF    INTCON3, INT1IE    ;; Enable Int1 
  BSF    INTCON,GIEL        ;; Enable low-priority interrupts 
  BSF    INTCON,GIEH        ;; Enable high-priority interrupts and all interrupts

  PULLF  WREG 

  RETURN 


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; Interrupt Service Routine for the high priority interrupt
  ;;   The pattern in this high-level part of the interrupt service routine is: 
  ;;     Check that the interrupt is enabled and that the flag is raised
  ;;     If the interrupt is not enabled or not requesting, branch to the next check
  ;;       Otherwise (int. enable and flag were set), service the interrupt.
  ;;       Go back up to the top of the list, and start again. 
  ;;
  ;;   This pattern has the characteristic that High_Priority_ISR doesn't exit 
  ;;   until all interrupts in the list that are requesting service 
  ;;   have been serviced. 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
High_Priority_ISR:

  BSF LATD, LatD_0_bit	; Set PortD,0
  BTFSC PIR2, TMR3IF	; If timer3 flag cleared, skip, else contine
  RCALL TMR3_ISR	; goto timer 3 ISR

  
  BTFSC INTCON3, INT1IF ; If Interrupt1 cleared, skip, else continue
  RCALL INT1_ISR	; call intr1 ISR
  
  BCF LATD, LatD_0_bit	; Clear PortD,0

  RETFIE FAST
  
INT1_ISR:
   BSF   LATD, LatD_2_bit   ; Set PortD,2
   BTFSS PORTB, 1	    ; If PortB bit 1 is set, go to ButtonPressed
   call ButtonPressed
   BTFSC PORTB, 1	    ; If PortB bit 1 is set, go to ButtonReleased
   call ButtonReleased
   BCF	LATD, LatD_2_bit    ; Clear PortD,2
   RETURN
  
TMR3_ISR: 

  decf DelayCounter	  ; Decrease DelayCounter by 1
  bz CheckPassword	  ; If zero, go and check password
  
  BCF  PIR2, TMR3IF       ;Clear timer flag bit
  RETURN	
  
Low_Priority_ISR:
    
   
LP_ISR01:
  BSF LATD,0x00
  BTFSS  PIR1,TMR1IF        ;; Test whether TMR1IF is set (TMR1 interrupt requested)   
  BRA    LP_ISR02           ;; If not set, go to next candidate 
  CALL TRAFFIC_UPDATE
  BCF  PIR1,TMR1IF 
  BCF LATD,0x00
  BRA    High_Priority_ISR  ;; Go to top, test all IRQs again 

LP_ISR02:

  RETFIE                    ;; Return from the interrupt, No FAST for low-priority interrupt 

 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; CCP1_ISR  Service the needs of the CCP1 interrupt 
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:  Clear CCP1IF, to enable next CCP1 interrupt
  ;;    Increment CCP1_Counter
  ;;    Call User subroutine 
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP1_ISR:
        
  RETURN 

         ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;;  Put STudent writes their MySubroutine (call)  Code Below.
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; TMR1_ISR  Service the needs of the CCPI interrupt 
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:   Clear TMR1IF, to setup for next TMR1 interrupt
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR1_ISR:
  BCF  PIR1,TMR1IF        
  RETURN 


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; CCP2_ISR  Service the needs of the CCP2 interrupt 
  ;; Inputs:  none
  ;; Outputs: none
  ;; Side effects:   Clear CCP2IF, to setup for next CCP2 interrupt
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP2_ISR:
  BCF PIR2,CCP2IF  
  RETURN 
  
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
    ;;  Subroutine: Init_SecuritySystem 
    ;;    Read a byte of data from na RSP device into the PIC18 
    ;;    The RSP device address is passed in Reg W. 
    ;;    the data is passed back into Reg W. 
    ;; 
    ;;  Inputs: Reg W:Adress of RSP device to read 
    ;;  Outputs:    Reg W:Data read from device 
    ;;  Side Effects: none 
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
TRAFFIC_UPDATE:
  TSTFSZ Traffic_Counter; Check to see if Traffic_Counter is Zero
  GOTO TRAFFIC_CONT;if not zero, skip reset
  ;Reset Counter:
  MOVF Traffic_Sequence, WREG ;Mov the number of entries in array to WREG
  MOVWF Traffic_Counter;
  
TRAFFIC_CONT:
  ;Trafic Counter is not zero, get state from array and
  ;Set traffic state, decrement.
  MOVF Traffic_Counter, WREG; Mov the cycle number (aka index) to WREG
  ADDLW Traffic_Sequence ;Dynamic array: add the cycle (aka index) to the address of the array's first address
  ;Get state from memory location in WREG
  ;LFSR 1, WREG ; Put address to read into File Select Register to read mem location
  CLRF FSR0H
  MOVWF FSR0L
  
  ;MOVWF INDF0
  MOVF INDF0, WREG ;Read memory address, move to WREG
  ;WREG is now the light state for this 1-sec cycle
  MOVWF RSPWrite_Data
  MOVLW RegF_Addr
  MOVWF RSPWrite_Address
  CALL RSPWrite 
  DECF Traffic_Counter	;Decrement the cycle
  
  RETURN
    
Init_SecuritySystem
    PUSHF STATUS			; Reserve Status Wreg
    PUSHF WREG
    
    clrf DelayCounter			; Clear Counter
    clrf RSPWrite_Address		; Clear RSPWrite_Address
    clrf RSPWrite_Data		        ; Clear RSPWrite_Data
    clrf RSPSwitch_Status		; Clear RSPSwitch_Status
    movlw InitialPassword		; 0xFF-> [W]
    movwf Password		        ; [W] -> [Password]

    clrf	TC_Port                 ; Clear T&C Port (PORTA)
    clrf	TC_Dir                  ; TC_Port_Dir (TRISA) all bits output
    clrf	E_Clk_Port              ; Clear E_Clk_Port (PORTB)
    movlw	InputPortCode		; Make PORTB bit 1 an input
    movwf	E_Clk_Dir

    PULLF WREG			; Restore variables
    PULLF STATUS
    return			; Exit Subroutine

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;    Subroutine: SecuritySystem       
;;	Tests Switch Input against stored password. Green LED if correct, Red if
;;	incorrect
;;   
;;	Inputs:	None
;;
;;	Outputs: 0x01 if Password incorrect, 0x02 if password correct
;;    
;;	Side Effects:	None
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
ButtonReleased:
    BCF	 INTCON3, INT1IF    ; Clear int1 flag
    BCF	 T3CON, TMR3ON	    ; Turn timer off
    BCF	 INTCON2,INTEDG1    ; Check for falling edge
    return
    
    
ButtonPressed:
    PUSHF STATUS		    ; Backup STATUS and WREG
    PUSHF WREG
    BSF T3CON, TMR3ON		    ; enable timer 3
    BSF	 PIE2, TMR3IE		    ; Enable timer 3
    movlw TwoSecondValue	    ; Move 2 sec to timer counter
    movwf DelayCounter		   
    BCF	 INTCON3, INT1IF	    ; Clear Int1 flag
    BSF	 INTCON2,INTEDG1	    ; Check for Rising edge
    
    movlw RegG_Addr		    ; 0x01 -> [W]
    movwf RSPWrite_Address	    ; [W] -> [RSPWrite_Address]
    movlw NoLightCode		    ; 0x00 -> [W]
    movwf RSPWrite_Data		    ; [W] -> [RSPWrite_Data]
    call RSPWrite
    
    return
    
CheckPassword:
    BCF  INTCON2, INTEDG1	    ; Check for falling edge
    movlw SIAddr		    ; 0x02 -> [W]
    call RSPRead		    ; Calls ReadSI. SI -> [W]
    subwf Password, 0		    ; [W] - [Password] -> [W] Sets Z Flag
    
    bz PasswordCorrect		    ; If Password Correct, RSPWrite 0x02 -> [G]
    bnz PasswordIncorrect	    ; If Password incorrect,RSPWrite 0x01 -> [G]
    

PasswordIncorrect:
	movlw RegG_Addr		    ; 0x01 -> [W]
	movwf RSPWrite_Address	    ; [W] -> [RSPWrite_Address]
	movlw RedLightCode	    ; 0x01 -> [W]
	movwf RSPWrite_Data	    ; [W] -> [RSPWrite_Data]
	call RSPWrite		    ; Calls WriteRegG. 0x01 -> RegG
	goto ReturnPoint	    ; Go to Restore Variables and Return

PasswordCorrect:
	movlw RegG_Addr		    ; 0x01 -> [W]
	movwf RSPWrite_Address	    ; [W] -> RSPWrite_Address
	movlw GreenLightCode	    ; 0x02 -> [W]
	movwf RSPWrite_Data	    ; [W] -> RSPWrite_Data
	call RSPWrite		    ; Calls WriteRegG. 0x02 -> RegG
	goto ReturnPoint	    ; Go to Restore Variables and Return

ReturnPoint:	
    PULLF WREG			    ; Restore stack variables
    PULLF STATUS
    
    return			    ; Exit Subroutine
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;    Subroutine: RSPRead       
;;	Read a byte of data from an RSP device into the PIC18.
;;	The RSP device address is passed in Reg W.
;;	The data is passed back in Reg W.
;;   
;;	Inputs:  Reg W: Address of RSP device to read (0x0 .. 0x9)     
;;
;;	Outputs: Reg W: data read from device
;;    
;;	Side Effects: NONE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
RSPRead	
    PUSHF STATUS		; Push Status to stack
    PUSHF TRISC 
    BCF LATB, 0		; make sure eclock is low to start read transaction

    IORLW Read_RegF		; bitwise OR the address with the lower nibble to
				; form T&C Code for read  
    MOVWF LATA			; Move T&C code onto LATA
    BSF LATB, 0		        ; raise the E-Clock for step 3

    
    MOVF PORTC, W		; Read port C for step 4
    
    BCF E_Clk_Port, 0		; Lower the E-clock for the end of the transaction  
    PULLF TRISC
    PULLF STATUS		; Pull Status from stack
	return
    
;************************************************************
; Run the 6 steps of an RSPRead
; Step 1: Drop E Clock
;************************************************************
    bcf PORTB,E_Clk_Bit                    ; Lower E-Clk (PORTB.0)
;************************************************************
; Step 2: Exert T&C Code
;  Store the T&C code to the T&C Port
;************************************************************
    movwf TC_Port                  ; Move T&C code from W register to TC_PORT 
				   ;(PORTA)
     				   
; Step 3: Raise the E Clock
    bsf   PORTB,E_Clk_Bit		    ; Raise E-Clk (PORTB.0)
; Step 4 is automatic, when E-Clock Raises
   MOVFF PORTC,WREG
; Step 5: Lower the E Clock
    bcf   PORTB,E_Clk_Bit		    ; Lower E-Clk (PORTB.0)
; Step 6 is automatic, when E-Clock Lowers
    PULLF STATUS
    
    return
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;    Subroutine: RSPWrite          
;;	Write a byte of data from the PICc18 into a specified RSP register.
;;    
;;   Inputs:	Address of the RSP register to write (i.e., 0x0 or 0x1)
;;		is passed in static variable RSPWrite_Address.
;;		The data to write are passed in static variable RSPWrite_Data.
;;   Outputs: NONE     
;;   Side Effects: Write Data to RSP Reg
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
    
RSPWrite

    PUSHF STATUS		; Push Status to stack
    PUSHF WREG			; Push RegW to stack
    BCF E_Clk_Port, 0		; make sure eclock is low to start read transaction
    
    ;Put address into W register 
    MOVF RSPWrite_Address, W
    
    ;Rotate address 4 bits to the left
    ;To line up Address with upper nibble
    RLNCF WREG, 0
    RLNCF WREG, 0
    RLNCF WREG, 0
    RLNCF WREG, 0
    IORLW Write_RegF		; bitwise OR the address with the upper nibble 
				; to form T&C Code for write    
    
    MOVWF TC_Port		; Move T&C code onto PortA
    BSF LATB, 0		        ; raise the E-Clock for step 3

    MOVF RSPWrite_Data, W	; Move RSPWrite_Data to WReg
    
    MOVWF PORTC			; Write port C for step 4
    CLRF TRISC			;S et TRISC to output

    BCF E_Clk_Port, 0		; Lower the E-clock for the end of the transaction  
    SETF TRISC			; Set TRISC to high impedence to release bus
    
    PULLF WREG			; Pull WReg from stack
    PULLF STATUS		; Pull Status from stack
		
	return
    
;************************************************************
; Run the 6 steps of an RSPWrite
; Step 1: Drop E Clock
;************************************************************
    bcf PORTB,E_Clk_Bit                    ; Lower E-Clk (PORTB.0)
;************************************************************
; Step 2: Load T&C Code
;************************************************************
    movwf   TC_Port		    ; Load T&C Code onto TC_Port
; Step 3: Raise the E Clock
    bsf   PORTB,E_Clk_Bit		    ; Raise E-Clk (PORTB.0)
; Step 4: Make PortC an Output Port
    clrf    TRISC		    ; Make TRISC an Output Port
    MOVFF   RSPWrite_Data, LATC	    ; Load Data onto LATC
; Step 5: Lower the E Clock
    bcf   PORTB,E_Clk_Bit		    ; Lower E-Clk (PORTB.0)
; Step 6: Make PortC an Input Port
    movlw   InputPortCode	    ; Load 0xFF onto WReg
    movwf   TRISC		    ; Make TRISC an Input Port
   
    PULLF STATUS
    PULLF WREG
    return

  end 