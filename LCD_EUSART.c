//******************************************************************************
/* 
 * File:   Comunicación Serial y Pantalla LCD 8 bits Main.c
 * Author: Daniel
 *
 * Created on July 27, 2021, 11:27 AM
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
#include "USART header.h"

//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void Setup(void);

//******************************************************************************
// Variables
//******************************************************************************
unsigned char VOLTAJE[10] =  {'0','1','2','3','4','5','6','7','8','9',};

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
int temp2;
int contadorNum2 = 0;
int contadorDec3 = 0;
int contadorDec4 = 0;

int DatoRecibido;
int contadorUART = 0;
int espacio = 32;
int borrar = 8;
int banderaUART1 = 1;

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
//****************************************************************
//      INTERRUPCION UART
//**************************************************************** 
    InterruptReciboUSART(&DatoRecibido);
    
    if (DatoRecibido == 43){
        contadorUART = contadorUART + 1;
        DatoRecibido = 32;
    }
    if (DatoRecibido == 45){
        contadorUART = contadorUART - 1;
        DatoRecibido = 32;
    }
}

//******************************************************************************
// Ciclo principal
//******************************************************************************
void main(void) {
  Setup(); 
    
  unsigned int a;
  Lcd_Init();
  
        Lcd_Clear(); //Interfaz grafica para la LCD
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
//  CONVERSION ADC       
//******************************************************************************
        contador_ADC = 1;   //Indicamos en que puerto mostraremos nuestra conversion
        Read_ADC(13);        //Hacemos una conversion usando canal 13
        
        __delay_ms(5);
        
        contador_ADC = 0;   
        Read_ADC(11);       //Hacemos una conversion usando canal 11
        
//******************************************************************************
//  CONTADOR LCD    
//******************************************************************************
        Lcd_Set_Cursor(2,14);
        sprintf(prueba, "%d", contadorUART);    //convierte contador a string
        Lcd_Write_String(prueba);               //Imprime contador o sensor 3
        
        if (contadorUART >= 0 ){                //Condiciones de seguridad para borrar ceros indeseados al incrementar y decrementar contador
            if (contadorUART < 10){
                Lcd_Set_Cursor(2,15);
                Lcd_Write_String(" ");
                Lcd_Set_Cursor(2,16);
                Lcd_Write_String(" ");
            }
        }
        if (contadorUART >= 10 ){
            if (contadorUART < 100){
                Lcd_Set_Cursor(2,16);
                Lcd_Write_String(" ");
            }
        }
        if (contadorUART <= 0 ){
            if (contadorUART > -10){
                Lcd_Set_Cursor(2,16);
                Lcd_Write_String(" ");
            }
        }
        if (contadorUART <= -10 ){
            if (contadorUART > -100){
                Lcd_Set_Cursor(2,17);
                Lcd_Write_String(" ");
            }
        }
        __delay_ms(5); //Imprimir en LCD la parte entera de la division:
        
//******************************************************************************
//  PRIMER POTENCIOMETRO        
//******************************************************************************     
        Lcd_Set_Cursor(2,1);                    
        temp = (High_ADC1 << 4) + Low_ADC1;     //Conversion completa ADC
        
        while (temp > 0){                       //Ciclos de divisiones para obtener un valor de 5V
            temp = temp - 51;
            contadorNum = contadorNum + 1; 
            if (temp <= 0){
                break;
            }
        }
        if (temp == 0){
            EnvioSerial(VOLTAJE[contadorNum]);  //Enviamos primero el numero serial y luego a la LCD
            sprintf(prueba, "%d", contadorNum);
            Lcd_Write_String(prueba);
            contadorNum = 0;
            __delay_ms(5); //Imprimir en LCD la parte entera de la division:
            EnvioSerial('.');
            Lcd_Set_Cursor(2,2);
            Lcd_Write_Char('.'); 
            __delay_ms(5);  //Imprimir punto decimal
            EnvioSerial('0');
            Lcd_Set_Cursor(2,3);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
            EnvioSerial('0');
            Lcd_Set_Cursor(2,4);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
        }
        if (temp < 0){
            temp = temp + 51;
            contadorNum = contadorNum - 1;
            EnvioSerial(VOLTAJE[contadorNum]);
            sprintf(prueba, "%d", contadorNum);
            Lcd_Write_String(prueba); 
            contadorNum = 0;
            __delay_ms(5); //Imprimir en LCD la parte entera de la division:
            EnvioSerial('.');
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
            EnvioSerial(VOLTAJE[contadorDec]);    
            Lcd_Set_Cursor(2,3);
            sprintf(prueba, "%d", contadorDec);
            Lcd_Write_String(prueba); 
            contadorDec = 0;
            __delay_ms(5);  //Imprimir en LCD unico decimal de la division:
            EnvioSerial('0');
            Lcd_Set_Cursor(2,4);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
            }
            if (temp < 0){
            temp = temp + 51;
            contadorDec = contadorDec - 1;
            EnvioSerial(VOLTAJE[contadorDec]);
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
                EnvioSerial(VOLTAJE[contadorDec2]);    
                Lcd_Set_Cursor(2,4);
                sprintf(prueba, "%d", contadorDec2);
                Lcd_Write_String(prueba); 
                contadorDec2 = 0;
                __delay_ms(5); //Imprimir en LCD unicos dos decimales de la division: 
                }
                if (temp < 0){
                temp = temp + 51;
                contadorDec2 = contadorDec2 - 1;
                EnvioSerial(VOLTAJE[contadorDec2]);
                Lcd_Set_Cursor(2,4);
                sprintf(prueba, "%d", contadorDec2);
                Lcd_Write_String(prueba); 
                contadorDec2 = 0;
                 __delay_ms(5);  //Imprimir en LCD el segundo decimal de la division:
                }
            }
        }
        EnvioSerial('V');
        EnvioSerial(espacio);
//******************************************************************************
//  SEGUNDO POTENCIOMETRO        
//******************************************************************************        
        Lcd_Set_Cursor(2,7);
        temp = (High_ADC2 << 4) + Low_ADC2;
        
        while (temp > 0){
            temp = temp - 51;
            contadorNum2 = contadorNum2 + 1; 
            if (temp <= 0){
                break;
            }
        }
        if (temp == 0){
            EnvioSerial(VOLTAJE[contadorNum2]);
            sprintf(prueba, "%d", contadorNum2);
            Lcd_Write_String(prueba);
            contadorNum2 = 0;
            __delay_ms(5); //Imprimir en LCD la parte entera de la division:
            EnvioSerial('.');
            Lcd_Set_Cursor(2,8);
            Lcd_Write_Char('.'); 
            __delay_ms(5);  //Imprimir punto decimal
            EnvioSerial('0');
            Lcd_Set_Cursor(2,9);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
            EnvioSerial('0');
            Lcd_Set_Cursor(2,10);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
        }
        if (temp < 0){
            temp = temp + 51;
            contadorNum2 = contadorNum2 - 1;
            EnvioSerial(VOLTAJE[contadorNum2]);
            sprintf(prueba, "%d", contadorNum2);
            Lcd_Write_String(prueba); 
            contadorNum2 = 0;
            __delay_ms(5); //Imprimir en LCD la parte entera de la division:
            EnvioSerial('.');
            Lcd_Set_Cursor(2,8);
            Lcd_Write_Char('.'); 
            __delay_ms(5);  //Imprimir punto decimal
            temp = temp * 10;
//*******************************************************************************            
            while (temp > 0){
            temp = temp - 51;
            contadorDec3 = contadorDec3 + 1;
                if (temp <= 0){
                    break;
                }
            }
            if (temp == 0){
            EnvioSerial(VOLTAJE[contadorDec3]);
            Lcd_Set_Cursor(2,9);
            sprintf(prueba, "%d", contadorDec3);
            Lcd_Write_String(prueba); 
            contadorDec3 = 0;
            __delay_ms(5);  //Imprimir en LCD unico decimal de la division:
            EnvioSerial('0');
            Lcd_Set_Cursor(2,10);
            Lcd_Write_Char('0'); 
            __delay_ms(5);  //Imprimir punto decimal
            }
            if (temp < 0){
            temp = temp + 51;
            contadorDec3 = contadorDec3 - 1;
            EnvioSerial(VOLTAJE[contadorDec3]);
            Lcd_Set_Cursor(2,9);
            sprintf(prueba, "%d", contadorDec3);
            Lcd_Write_String(prueba); 
            contadorDec3 = 0;
            __delay_ms(5);  //Imprimir en LCD el primer decimal de la division:
            temp = temp * 10;
//*******************************************************************************              
                while (temp > 0){
                temp = temp - 51;
                contadorDec4 = contadorDec4 + 1;
                    if (temp <= 0){
                        break;
                    }
                }
                if (temp == 0){
                EnvioSerial(VOLTAJE[contadorDec4]);
                Lcd_Set_Cursor(2,10);
                sprintf(prueba, "%d", contadorDec4);
                Lcd_Write_String(prueba); 
                contadorDec4 = 0;
                __delay_ms(5); //Imprimir en LCD unicos dos decimales de la division: 
                }
                if (temp < 0){
                temp = temp + 51;
                contadorDec4 = contadorDec4 - 1;
                EnvioSerial(VOLTAJE[contadorDec4]);
                Lcd_Set_Cursor(2,10);
                sprintf(prueba, "%d", contadorDec4);
                Lcd_Write_String(prueba); 
                contadorDec4= 0;
                 __delay_ms(5);  //Imprimir en LCD el segundo decimal de la division:
                }
            }
        }
        EnvioSerial('V');
        EnvioSerial(espacio);
        
        EnvioSerial(borrar);  //Borramos la consola para que no se repitan valores o se sature la consola
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
        EnvioSerial(borrar);
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
    TRISC = 0b10000000;
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
    
    //Configuración de la comunicación, recepción y transmisión
    
    SPBRG = 12; //baud rate 9600
    SYNC = 0; //comunicación asíncrona
    SPEN = 1; //habilita comunicación
    CREN = 1;
    TXEN = 1;
    //Limpiamos bandera de recepción
    RCIF = 0;
    RCIE = 1; //Habilita interrupción de recepción
 
    ADIF = 0;   //Habilita interrupcion del adc
    ADIE = 1;
    PEIE=1;     //Habilita interrupciones perifericas
    
    GIE=1;    //Habilita interrupciones globales
    
    IRCF0 = 1; //Oscilador a 8MHz
    IRCF1 = 1;
    IRCF2 = 1;
    SCS = 1;
}