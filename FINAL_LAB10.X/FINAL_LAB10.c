/* 
 * File:   FINAL_LAB10.c
 * Author: ALBA RODAS
 *
 * Created on 6 de mayo de 2022, 01:36 PM
 */

// CONFIGURATION WORDS 1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIGURATION WORDS 2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
//DEFINO MI OSCILADOR A 1MHz:
#define _XTAL_FREQ 1000000
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t valor_potenciometro = 0;                    // AQUI GUARDO EL VALOR EN DEL POT.
uint8_t value_ASCII = 0;                            // AQUI GUARDO EL VALOR DEL ASCII.
uint8_t indice_menu = 0;                            // ME SIRVE PARA SABER EL VALOR DEL MENSAJE QUE SE ENVIA AL SERIAL
char    mensaje_pot[3] = {0,0,0};                   // POSICIONES PARA UNIDADES, DECENAS, CENTENAS
uint8_t indice_potenciometro = 0;                                // FLAG PARA EL VALOR DEL POT
uint8_t modo = 0;                                   // MODO: ASCII O POT.
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
// LLAMO A TODAS MIS FUNCIONES:
void setup(void);                       
void get_value(uint8_t values);     
void printing(char *texto);               
void TX_usart(char data);               
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.RCIF){                          // CHECK: se recibieron datos?
        value_ASCII = RCREG;                    // VALOR RECIBIDO --> SE GUARDA EN EL RCREG
        if(modo == 1){                          // MODO ASCII: MUESTRO VALORES EN PORTB, 
            PORTB = value_ASCII;               
            modo = 0;                           // RESET AL MODO
            indice_menu = 0;                    // VUELVO AL MENU
        }
    }
    else if(PIR1bits.ADIF){                     // INTERRUPCION DEL ADC
        valor_potenciometro = ADRESH;           // GUARDO VALOR DEL POT --> ADRESH
        PIR1bits.ADIF = 0;                      // CLR A BANDERA
    }
    return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){                         // SI NO HAY CONVERSION == 0        
            ADCON0bits.GO = 1;                          // INICIA CONVERSION
        } 
        if (indice_menu == 0){                          // INDICE = 0, PRINT A LAS PREGUNTAS
            printing("\r1. Leer Potenciometro. \r");    
            printing("2. Enviar ASCII.\r");
            indice_menu = 1;                            // INDICE = 1, STOP EL PRINT A LAS PREGUNTAS
        }
        if(value_ASCII == '1'){                         // MODO 1: ASCII
            indice_menu = 0;                            // INDICE_MENU = 0, IMPRMIMOS EL MENU.
            modo = 0;                                   // RESET AL MODO
            get_value(valor_potenciometro);             
            indice_potenciometro = 0;                   // indice_potenciometro = 0, IMPRIMIOS VALOR DEL POT
            if (indice_potenciometro == 0){             // INICIAMOS CONDICION PARA CONVERSION
                TX_usart(mensaje_pot[0]);               // CENTENAS
                TX_usart(mensaje_pot[1]);               // DECENAS
                TX_usart(mensaje_pot[2]);               // UNIDADES
                indice_potenciometro = 1;               // INDICE = 1, STOP EL PRINT
            }
            value_ASCII = 0;                            // CLR A MI VARIABLE QUE GUARDA VALORES DE ASCII
        }
        if(value_ASCII == '2'){                         // MODO: ASCII
            modo = 1;                                   // MODO --> 1
            value_ASCII = 0;                            // CLR AL ASCII
        }
    }
    return;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0x01;               // AN0 --> ANALOGIC INPUT
    ANSELH = 0;                 // ENTRADAS/SALIDAS DIGITALES
    
    TRISA = 0x01;               // AN0 --> ENTRADA
    PORTA = 0;
    TRISB = 0;
    PORTB = 0;                  // PORTB --> SALIDA
    TRISD = 0;                  // LIMPIO LOS TRIS
    PORTD = 0;                  // IGUALAMOS A CERO PARA EVITAR EMPEZAR CON VALROES ALEATORIOS
        
    // CONFIGURACION DEL RELOJ INTERNO
    OSCCONbits.IRCF = 0b0100;   // VALOR --> 1MHz
    OSCCONbits.SCS = 1;         // OSCILADOR: ON
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b00;     // Fosc/08  para 1MHz
    ADCON1bits.VCFG0 = 0;       // REFERENCIA --> VDD
    ADCON1bits.VCFG1 = 0;       // REFERENCIA --> VSS
    ADCON0bits.CHS = 0b0000;    // CHS --> AN0
    ADCON1bits.ADFM = 0;        // JUSTIFICADO A LA IZQUIERDA
    ADCON0bits.ADON = 1;        // ADC: ON
    __delay_us(40);             // DELAY PARA ESTABILIZACION DE CIRCUITERIA <= 40us
    
    // CONFIGURACION COMUNICACION SERIAL
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores PARA tabla del 12-5
    TXSTAbits.SYNC = 0;         // TIPO DE COMUNICACION --> ASINCRONA
    TXSTAbits.BRGH = 1;         // ALTA VELOCIDAD = 1
    BAUDCTLbits.BRG16 = 1;      // BAUD RATE --> 16BITS
    
    SPBRG = 25;                 // CARGAMOS VALOR DEL BAUD RATE
    SPBRGH = 0;                 // ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // COMUNICACION: HABILITADA
    TXSTAbits.TX9 = 0;          // TX9 --> 8 BITS
    TXSTAbits.TXEN = 1;         // TRANSMISOR: HABILITADO
    RCSTAbits.CREN = 1;         // RECEPTOR: HABILITADO
    
    // CONFIGURACION DE INTERRUPCIONES
    PIR1bits.ADIF = 0;          // CLR BANDERA DEL ADC
    PIE1bits.ADIE = 1;          // HABILITAR: INTERRUPCION DEL ADC
    INTCONbits.GIE = 1;         // ACTIVADAS: INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;        // ACTIVADAS: INTERRUPCIONES DE PERIFERICOS
    PIE1bits.RCIE = 1;          // ACTIVADAS: INTERRUPCIONES DE RECEPCION
}
/*------------------------------------------------------------------------------
 * FUNCIONES 
 ------------------------------------------------------------------------------*/

void get_value(uint8_t values){
    mensaje_pot[0] = values/100;                                    // DEFINIMOS CENTENAS --> X/100
    mensaje_pot[1] = (values-mensaje_pot[0]*100)/10;                // DEFINIMOS DECENAS --> X/10
    mensaje_pot[2] = values-mensaje_pot[0]*100-mensaje_pot[1]*10;   // DEFINIMOS UNIDADES --> X/1
    
    // VALOR --> ASCII
    mensaje_pot[0] = mensaje_pot[0]+0x30;
    mensaje_pot[1] = mensaje_pot[1]+0x30;
    mensaje_pot[2] = mensaje_pot[2]+0x30;
}

void TX_usart(char data){
    while(TXSTAbits.TRMT==0);   // WAIT HASTA QUE EL TSR -> VACIO
    TXREG = data;               // LE DAMOS NUEVO VALOR AL TXREG, CON LOS DATOS OBTENIDOS
}

void printing(char *texto){
    while (*texto != '\0'){       // EN FUNCIONAMIENTO SOLO SI NO ESTÁ VACIO
        TX_usart(*texto);         // ENVIAMOS CADA ELEMENTO QUE COMPONE AL TEXTO
        texto++;                  // INCREMENTEAMOS TEXTO.
    }
}

