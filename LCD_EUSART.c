//******************************************************************************
/* 
 * File:   Pantalla LCD 8 bits Main.c
 * Author: Daniel
 *
 * Created on July 23, 2021, 7:06 PM
 */
//******************************************************************************
// Importación de Librerías
//******************************************************************************

#pragma config FOSC = XT        // Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
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
#define _XTAL_FREQ 8000000

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "LCD header.h"
#include "ADC header.h"

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void Setup(void);

//******************************************************************************
// Variables
//******************************************************************************
unsigned char DISPLAY[0b00010000] =  {'0', //contiene los digitos
                                      '1', //0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F
                                      '2', // en formato para ser mostradosendisplay
                                      '3', // de 7segmentos
                                      '4',
                                      '5',
                                      '6',
                                      '7',
                                      '8',
                                      '9',
                                      'A',
                                      'B',
                                      'C',
                                      'D',
                                      'E',
                                      'F',};

int Low_ADC1;
int Low_ADC2;
int High_ADC1;
int High_ADC2;

char DisplayLow_ADC;
char DisplayHigh_ADC;
char contador_ADC;

char prueba[4];
int temp;
int contadorNum = 0;
int contadorDec = 0;
int contadorDec2 = 0;



//******************************************************************************
// Vector de interrupcion
//******************************************************************************

void __interrupt() ISR(void){
//****************************************************************
//      INTERRUPCION ADC
//****************************************************************    
    InterruptADC(&DisplayLow_ADC, &DisplayHigh_ADC);        //Funcion para saber que fue lo que se convirtio del ADC
    
    if (contador_ADC == 1) {                                //Escogemos puerto 
        High_ADC1 = DisplayHigh_ADC;
        Low_ADC1 = DisplayLow_ADC;
    }
    
    if (contador_ADC == 0) {
        High_ADC2 = DisplayHigh_ADC;
        Low_ADC2 = DisplayLow_ADC;
    }
    
}

//******************************************************************************
// Ciclo principal
//******************************************************************************
void main(void) {
  Setup(); 
    
  unsigned int a;
  Lcd_Init();
  
        Lcd_Clear();
        Lcd_Set_Cursor(1,1);
        Lcd_Write_String("S1");
        Lcd_Set_Cursor(1,7);
        Lcd_Write_String("S2");
        Lcd_Set_Cursor(1,14);
        Lcd_Write_String("S3");
        Lcd_Set_Cursor(2,5);
        Lcd_Write_String("V");
        Lcd_Set_Cursor(2,11);
        Lcd_Write_String("V");
  while(1)
  {
//******************************************************************************
        contador_ADC = 1;   //Indicamos en que puerto mostraremos nuestra conversion
        Read_ADC(13);        //Hacemos una conversion usando canal 13
        
        __delay_ms(5);
        
        contador_ADC = 0;   
        Read_ADC(11);       //Hacemos una conversion usando canal 11
        
//******************************************************************************
        
        Lcd_Set_Cursor(2,1);
        temp = (High_ADC2 << 4) + Low_ADC2;
        
        while (temp > 0){
            temp = temp - 51;
            contadorNum = contadorNum + 1; 
            if (temp <= 0){
                break;
            }
        }
        if (temp == 0){
            sprintf(prueba, "%d", contadorNum);
            Lcd_Write_String(prueba);
            contadorNum = 0;
            __delay_ms(5); //Imprimir en LCD la parte entera de la division:
            Lcd_Set_Cursor(2,2);
            Lcd_Write_Char('.'); 
            __delay_ms(5);  //Imprimir punto decimal
            Lcd_Set_Cursor(2,3);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
            Lcd_Set_Cursor(2,4);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
        }
        if (temp < 0){
            temp = temp + 51;
            contadorNum = contadorNum - 1;
            sprintf(prueba, "%d", contadorNum);
            Lcd_Write_String(prueba); 
            contadorNum = 0;
            __delay_ms(5); //Imprimir en LCD la parte entera de la division:
            Lcd_Set_Cursor(2,2);
            Lcd_Write_Char('.'); 
            __delay_ms(5);  //Imprimir punto decimal
            temp = temp * 10;
//*******************************************************************************            
            while (temp > 0){
            temp = temp - 51;
            contadorDec = contadorDec + 1;
                if (temp <= 0){
                    break;
                }
            }
            if (temp == 0){
            Lcd_Set_Cursor(2,3);
            sprintf(prueba, "%d", contadorDec);
            Lcd_Write_String(prueba); 
            contadorDec = 0;
            __delay_ms(5);  //Imprimir en LCD unico decimal de la division:
            Lcd_Set_Cursor(2,4);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
            }
            if (temp < 0){
            temp = temp + 51;
            contadorDec = contadorDec - 1;
            Lcd_Set_Cursor(2,3);
            sprintf(prueba, "%d", contadorDec);
            Lcd_Write_String(prueba); 
            contadorDec = 0;
            __delay_ms(5);  //Imprimir en LCD el primer decimal de la division:
            temp = temp * 10;
//*******************************************************************************              
                while (temp > 0){
                temp = temp - 51;
                contadorDec2 = contadorDec2 + 1;
                    if (temp <= 0){
                        break;
                    }
                }
                if (temp == 0){
                Lcd_Set_Cursor(2,4);
                sprintf(prueba, "%d", contadorDec2);
                Lcd_Write_String(prueba); 
                contadorDec2 = 0;
                __delay_ms(5); //Imprimir en LCD unicos dos decimales de la division: 
                }
                if (temp < 0){
                temp = temp + 51;
                contadorDec2 = contadorDec2 - 1;
                Lcd_Set_Cursor(2,4);
                sprintf(prueba, "%d", contadorDec2);
                Lcd_Write_String(prueba); 
                contadorDec2 = 0;
                 __delay_ms(5);  //Imprimir en LCD el segundo decimal de la division:
                }
            }
        }
  }
    return;
}
//******************************************************************************
// Configuración
//******************************************************************************
void Setup(void){     
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    TRISA = 0x00;
    TRISC = 0x00;
    TRISD = 0x00;
    TRISE = 0x00; 

    TRISB = 0xFF;  
    PORTB = 0;
    
    ANSEL = 0;
    ANSELH = 0;
    ANS11 = 1; //Indicamos que pines son analogicos y cuales digitales del puerto B
    ANS13 = 1;

    ADCON0bits.ADCS = 2;
    //ADCON0bits.CHS = 13;
    ADCON0bits.GO = 0;
    ADCON0bits.ADON = 1;
    ADCON1 = 0x80;
 
    ADIF = 0;   //Habilita interrupcion del adc
    ADIE = 1;
    PEIE=1;     //Habilita interrupciones perifericas
    
    GIE=1;    //Habilita interrupciones globales
    
    IRCF0 = 1; //Oscilador a 8MHz
    IRCF1 = 1;
    IRCF2 = 1;
    SCS = 1;
}