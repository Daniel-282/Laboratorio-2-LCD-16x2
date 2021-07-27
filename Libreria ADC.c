#include <stdint.h>
#include <pic16f887.h>
#include "ADC header.h"

unsigned char Low_ADC;
unsigned char High_ADC;
unsigned char Resultado_ADC; 

void InterruptADC(char* LowNibble_ADC, char* HighNibble_ADC){
    if (ADIF == 1){    
        ADIF = 0;
    
        High_ADC = ADRESH;
        High_ADC = High_ADC << 6; //Movemos los 2 bits del ADRESH 6 espacios arriba
    
        Low_ADC = ADRESL;
        Low_ADC = (Low_ADC & 0b11111100) >> 2; //Borramos los dos primeros bits de ADRESL y los movemos 2 espacios abajo
    
        Resultado_ADC = High_ADC + Low_ADC; //sumamos ambos para obtener 8 espacios de bits significativos
        
        *LowNibble_ADC = Resultado_ADC & 0b00001111;
        *HighNibble_ADC = (Resultado_ADC & 0b11110000)>>4;
    }
}

void Read_ADC(char channel){
        ADCON0bits.CHS = channel;  
        delayADC(10);                  //delay necesario luego de cambiar de canal
        GO = 1;                        // inicio de conversion  ADC
        while(GO==1);                  // nos aseguramos que la conversion termino
}

void delayADC(char n){                  // delay de la libreria para el ADC
    for(int i = 0; i < n; i++){
        for(int j = 0; j < 255; j++ ){
            
        }
    }

}

