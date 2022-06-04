/* 
 * File:  Proyecto 2- Pic Esclavo.c
 * Author: 
 * Saby Andrade 20882
 * Pablo Fuentes 20888
 *
 *Esclavo: Reproduce las posiciones de 2 Servo motores por SPI
 * 
 */


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF               // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF              // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF              // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                 // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF              // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF               // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF              // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V           // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                // Flash Program Memory Self Write Enable bits (Write protection off)


#include <xc.h>
#include <stdint.h>
#include <stdio.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000      // Frecuencia de oscilador en 1 MHz
#define FLAG_SPI 0xFF           // Variable bandera para lectura del esclavo
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 127              // Valor m?ximo de entrada del potenciometro
#define OUT_MIN 0               // Valor minimo de ancho de pulso de se?al PWM
#define OUT_MAX 125             // Valor m?ximo de ancho de pulso de se?al PWM
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
unsigned short CCPR = 0, CCPR_2 = 0;        // Variable para almacenar ancho de pulso
uint8_t buffer=0, verificador=0;            //Verificador de llegada y valor a usar
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){
        
        buffer = 127 & SSPBUF;          //Obtiene el valor a mapear
        verificador = 128 & SSPBUF;     //Obtiene el valor de identificaci?n de servo
 
        if (verificador == 128){        //Identifica Servo 1
            CCPR = map(buffer, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
        }
        else if (verificador==0){       //Identifica Servo 2
            CCPR_2 = map(buffer, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); 
            CCPR2L = (uint8_t)(CCPR_2>>2);
            CCP2CONbits.DC2B0 = CCPR_2 & 0b01;    
            CCP2CONbits.DC2B1 = (CCPR_2 & 0b10)>>1;
        }
        PIR1bits.SSPIF = 0;
        
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){        
    
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){   
    // Configuraci?n del oscilador interno    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
  
    ANSEL = 0;
    ANSELH = 0;
    TRISA = 0b00010011;
    PORTA = 0;
    TRISD = 0;
    PORTD = 0;
    TRISC = 0b00011000;
    PORTC = 0;

    // Configuraci?n PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    TRISCbits.TRISC1 = 1;
    PR2 = 31;                   // periodo de 2ms -> 0.002 = (PR2+1)*4*1/(Fosc)*Pes
    
    // Configuraci?n CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    CCP2CON = 0;                // Apagamos CCP2
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 32>>2;                  //Resolucion 125
    CCPR2L = 32>>2;
    CCP1CONbits.DC1B = 32 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    CCP2CONbits.DC2B0 = 32 & 0b01;         // 
    CCP2CONbits.DC2B1 = (32 & 0b10)>>1; 
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;
    
    //SPI ESCALVO    
    SSPCONbits.SSPM  =  0b0100; //-> FOSC/4 -> 250kbits/s
    SSPCONbits.CKP   =       0;
    SSPCONbits.SSPEN =       1;
    
    SSPSTATbits.CKE = 1;
    SSPSTATbits.SMP = 0;
    
    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
}


/*interpolaci?n
*  y = y0 + [(y1 - y0)/(x1-x0)]*(x-x0)
*  -------------------------------------------------------------------
*  | x0 -> valor m?nimo de ADC | y0 -> valor m?nimo de ancho de pulso|
*  | x  -> valor actual de ADC | y  -> resultado de la interpolaci?n | 
*  | x1 -> valor m?ximo de ADC | y1 -> valor m?ximo de ancho de puslo|
*  ------------------------------------------------------------------- 
*/
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
        unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
} 
