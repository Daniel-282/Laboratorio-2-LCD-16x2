#include <stdint.h>
#include <pic16f887.h>
#include "USART header.h"

int value;
int value2;

void EnvioSerial (int value){
    TXREG = value; //Carga al registro de envio el dato deseado
    while (TXSTAbits.TRMT == 0){  //Esperar a que termine envio.
    }
}

void InterruptReciboUSART (char *value2){
    if (RCIF == 1){
        *value2 = RCREG; //lee registro de recepción
        RCIF = 0; //limpia bandera de recepción
    }
}
