#ifndef __DIO_H_
#define __DIO_H_


#include "bitwise_operation.h"
#include "tm4c123gh6pm.h"


#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5

#define P0 0
#define P1 1
#define P2 2
#define P3 3 
#define P4 4
#define P5 5
#define P6 6
#define P7 7
#define P30 30

#define INPUT_PIN 0
#define OUTPUT_PIN 1

#define LOGIC_HIGH 1
#define LOGIC_LOW 0

void delay(int n);
void DIO_Init(int port, int bit, int dir);
void DIO_WritePin(int port,int bit, int value);
void DIO_WritePort(int port,int value);
int toggle_pin(int port,int bit);
int read_pin(int port,int pin);
unsigned long read_port(int port);
void DIO_InterruptInit(int port,int pin);
#endif