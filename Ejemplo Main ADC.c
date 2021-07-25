//******************************************************************************
/* 
 * File:   Interrupciones y Librerias Main.c
 * Author: Daniel
 *
 * Created on July 18, 2021, 7:06 PM
 */
//******************************************************************************
// Importación de Librerías
//******************************************************************************

#include <xc.h>
#include <stdint.h>
#include "ADC_header.h"

//******************************************************************************
// Palabra de configuración
//******************************************************************************
// CONFIG1
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void Setup(void);
void delay(char n);

//******************************************************************************
// Variables
//******************************************************************************

unsigned char Low_ADC;
unsigned char High_ADC;
unsigned char Resultado_ADC;
char DisplayLow_ADC;
char DisplayHigh_ADC;
char contador_ADC;

//******************************************************************************
// Vector de interrupcion
//******************************************************************************

void __interrupt() ISR(void){
//****************************************************************
//      INTERRUPCION ADC
//****************************************************************    
    InterruptADC(&DisplayLow_ADC, &DisplayHigh_ADC);        //Funcion para saber que fue lo que se convirtio del ADC
    
    if (contador_ADC == 1) {                                //Escogemos puerto 
        PORTD = (DisplayHigh_ADC << 4) + DisplayLow_ADC;
    }
    
    if (contador_ADC == 0) {
        PORTC = (DisplayHigh_ADC << 4) + DisplayLow_ADC;
    }
    
}

//******************************************************************************
// Ciclo principal
//******************************************************************************
void main(void) { 
    Setup();
    //**************************************************************************
    // Loop principal
    //**************************************************************************
    while(1){
        contador_ADC = 1;   //Indicamos en que puerto mostraremos nuestra conversion
        Read_ADC(13);        //Hacemos una conversion usando canal 13
        
        delay(5);
        
        contador_ADC = 0;   
        Read_ADC(11);       //Hacemos una conversion usando canal 11

    }
}
//******************************************************************************
// Configuración
//******************************************************************************
void Setup(void){     
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    TRISA = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    
    TRISB = 0xFF;  
    PORTB = 0;
    
    ANS11 = 1; //Indicamos que pines son analogicos y cuales digitales del puerto B
    ANS10 = 0;
    ANS12 = 0;
    ANS13 = 1;

    
    ADCON0bits.ADCS = 2;
    //ADCON0bits.CHS = 13;
    ADCON0bits.GO = 0;
    ADCON0bits.ADON = 1;
    ADCON1 = 0x80;
 
    ADIF = 0;   //Habilita interrupcion del adc
    ADIE = 1;
    PEIE=1;     //Habilita interrupciones perifericas
    
    //IOCB0 = 1; //Habilita interrupciones por cambio
    //IOCB1 = 1;
    //RBIE = 1;   //Habilita interrupciones puerto B
    //RBIF = 0;
    
    //OPTION_REGbits.T0CS = 0;   // Configuración para tener un desborde   
    //OPTION_REGbits.PSA = 0;
    //OPTION_REGbits.PS = 6;     // Prescaler de 64 
    //T0IE = 1;   //Habilita interrupciones timer0
    //T0IF = 1;
    //TMR0 =  6;  //Numero calculado para desborde  de 8ms 
    
    GIE=1;    //Habilita interrupciones globales
    
            
    IRCF0 = 1; //Oscilador a 8MHz
    IRCF1 = 1;
    IRCF2 = 1;
    SCS = 1;
}
//******************************************************************************
// Delay
//******************************************************************************
void delay(char n){
    for(int i = 0; i < n; i++){
        for(int j = 0; j < 255; j++ ){
            
        }
    }

}
//******************************************************************************
// Funciones Adicionales
//******************************************************************************

