#include "DIO.h"


void delay(int n){
  for(int i = 0 ; i<n;i++)
  {
    for(int j = 0 ; j<3160; j++)
    {}
  }
}


//****************************************************************

//*************** INIT *************************************

//*****************************************************************
void DIO_Init(int port, int bit, int dir){
  
    Set_Bit(SYSCTL_RCGCGPIO_R,port);
    while((SYSCTL_PRGPIO_R & (1<<port)) == 0){}; 
    switch(port){
   //****************** PORT A ************************   
    case PORTA:
      GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY;
      Set_Bit(GPIO_PORTA_CR_R ,bit);
      if(dir ==1)
      {
        Set_Bit(GPIO_PORTA_DIR_R,bit);
      }
      else if(dir ==0)
      {
        Clear_Bit(GPIO_PORTA_DIR_R,bit);
        Set_Bit(GPIO_PORTA_PUR_R,bit);
      }
      Set_Bit(GPIO_PORTA_DEN_R,bit);
      break;
   //****************** PORT B ************************
    case PORTB:
      GPIO_PORTB_LOCK_R = GPIO_LOCK_KEY;
      Set_Bit(GPIO_PORTB_CR_R ,bit);
      if(dir ==1)
      {
        Set_Bit(GPIO_PORTB_DIR_R,bit);
      }
      else if(dir ==0)
      {
        Clear_Bit(GPIO_PORTB_DIR_R,bit);
        Set_Bit(GPIO_PORTB_PDR_R,bit);
      }
      Set_Bit(GPIO_PORTB_DEN_R,bit);
      break;
   //****************** PORT C ************************
    case PORTC:
      GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;
      Set_Bit(GPIO_PORTC_CR_R ,bit);

      if(dir ==1)
      {
        Set_Bit(GPIO_PORTC_DIR_R,bit);
      }
      else if(dir ==0)
      {
        Clear_Bit(GPIO_PORTC_DIR_R,bit);
        Set_Bit(GPIO_PORTC_PDR_R,bit);
      }
      Set_Bit(GPIO_PORTC_DEN_R,bit);
      break;
   //****************** PORT D ************************
    case PORTD:
      GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
      Set_Bit(GPIO_PORTD_CR_R ,bit);
      if(dir ==1)
      {
        Set_Bit(GPIO_PORTD_DIR_R,bit);
      }
      else if(dir ==0)
      {
        Clear_Bit(GPIO_PORTD_DIR_R,bit);
        Set_Bit(GPIO_PORTD_PDR_R,bit);
      }
      Set_Bit(GPIO_PORTD_DEN_R,bit);
      break;
   //****************** PORT E ************************
    case PORTE:
      GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY;
      Set_Bit(GPIO_PORTE_CR_R ,bit);
      if(dir ==1)
      {
        Set_Bit(GPIO_PORTE_DIR_R,bit);
      }
      else if(dir ==0)
      {
        Clear_Bit(GPIO_PORTE_DIR_R,bit);
        Set_Bit(GPIO_PORTE_PUR_R,bit);
      }
      Set_Bit(GPIO_PORTE_DEN_R,bit);
      break;
   //****************** PORT F ************************     
    case PORTF:
      GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
      Set_Bit(GPIO_PORTF_CR_R ,bit);
      if(dir ==1)
      {
        Set_Bit(GPIO_PORTF_DIR_R,bit);
      }
      else if(dir ==0)
      {
        Clear_Bit(GPIO_PORTF_DIR_R,bit);
        Set_Bit(GPIO_PORTF_PUR_R,bit);
      }
      Set_Bit(GPIO_PORTF_DEN_R,bit);
      break;
    default:
      break;
    }
}
//****************************************************************

//*************** WRITE PIN *************************************

//*****************************************************************

void DIO_WritePin(int port,int bit, int value){
    switch(port)
    {
// ******************** PORT A *******************
    case PORTA:
      if(value == 1)
      {
        Set_Bit(GPIO_PORTA_DATA_R,bit);
      }
      else if (value ==0)
      {
        Clear_Bit(GPIO_PORTA_DATA_R,bit);
      }
       break;
// ******************** PORT B *******************
    case PORTB:
      if(value == 1)
      {
        Set_Bit(GPIO_PORTB_DATA_R,bit);
      }
      else if (value ==0)
      {
        Clear_Bit(GPIO_PORTB_DATA_R,bit);
      }
       break;
// ******************** PORT C *******************
    case PORTC:
      if(value == 1)
      {
        Set_Bit(GPIO_PORTC_DATA_R,bit);
      }
      else if (value ==0)
      {
        Clear_Bit(GPIO_PORTC_DATA_R,bit);
      }
      break;
// ******************** PORT D *******************
    case PORTD:
      if(value == 1)
      {
        Set_Bit(GPIO_PORTD_DATA_R,bit);
      }
      else if (value ==0)
      {
        Clear_Bit(GPIO_PORTD_DATA_R,bit);
      }
      break;
// ******************** PORT E *******************
    case PORTE:
      if(value == 1)
      {
        Set_Bit(GPIO_PORTE_DATA_R,bit);
      }
      else if (value ==0)
      {
        Clear_Bit(GPIO_PORTE_DATA_R,bit);
      }
       break;
// ******************** PORT F *******************
    case PORTF:
      if(value == 1)
      {
        Set_Bit(GPIO_PORTF_DATA_R,bit);
      }
      else if (value ==0)
      {
        Clear_Bit(GPIO_PORTF_DATA_R,bit);
      }
    default:
      break; 
    }
}

//****************************************************************

//*************** WRITE PORT *************************************

//*****************************************************************
void DIO_WritePort(int port,int value){
  switch(port){
    
//******************** PORTA *****************************
  case PORTA:
    GPIO_PORTA_DATA_R = value;
     break;
//******************** PORTB *****************************
  case PORTB:
    GPIO_PORTB_DATA_R  = value;
     break;
//******************** PORTC *****************************
  case PORTC:
    GPIO_PORTC_DATA_R = value;
     break;
//******************** PORTD *****************************
  case PORTD:
    GPIO_PORTD_DATA_R = value;
     break;
//******************** PORTE *****************************
  case PORTE:
    GPIO_PORTE_DATA_R = value;
     break;
//******************** PORTF *****************************
  case PORTF:
    GPIO_PORTF_DATA_R = value;
     break;
  default:
    break;
  }
}
//***************************************************************

//*************** READ PIN *************************************

//*****************************************************************
int read_pin(int port,int bit){
  switch(port)
  {
//****************** PORTA ************************************
  case PORTA:
    return Get_Bit(GPIO_PORTA_DATA_R,bit);
     break;
//****************** PORTB ************************************
  case PORTB:
    return Get_Bit(GPIO_PORTB_DATA_R,bit);
     break;
//****************** PORTC ************************************
  case PORTC:
    return Get_Bit(GPIO_PORTC_DATA_R,bit);
     break;
//****************** PORTD ************************************
  case PORTD:
    return Get_Bit(GPIO_PORTD_DATA_R,bit);
     break;
//****************** PORTE ************************************
  case PORTE:
    return Get_Bit(GPIO_PORTE_DATA_R,bit);
     break;
//****************** PORTF ************************************
  case PORTF:
    return Get_Bit(GPIO_PORTF_DATA_R,bit);
     break;
  default:
    return 0;
    break;
  }
  
}

int toggle_pin(int port,int bit){
  switch(port)
  {
  case PORTA:
    return Toggle_Bit(GPIO_PORTA_DATA_R,bit);
    break;
  case PORTF:
    return Toggle_Bit(GPIO_PORTF_DATA_R,bit);
    break;
  }

}
//****************************************************************

//*************** READ PORT *************************************

//*****************************************************************
 unsigned long read_port(int port)
{
  switch(port){
//****************** PORTA *************************************
  case PORTA:
    return GPIO_PORTA_DATA_R;
     break;
//****************** PORTB *************************************
  case PORTB:
    return GPIO_PORTB_DATA_R;
     break;
//****************** PORTC *************************************
  case PORTC:
    return GPIO_PORTC_DATA_R;
    break;
//****************** PORTD *************************************
  case PORTD:
    return GPIO_PORTD_DATA_R;
    break;
//****************** PORTE *************************************
  case PORTE:
    return GPIO_PORTE_DATA_R;
    break;
//****************** PORTF *************************************
  case PORTF:
    return GPIO_PORTF_DATA_R;
  default:
    break;
  } 
}

void DIO_InterruptInit(int port,int pin)
{

  
  switch(port)
  {
  case PORTA:
    Clear_Bit(GPIO_PORTA_IS_R,pin); // to trigger edgess
    Clear_Bit(GPIO_PORTA_IBE_R,pin); //to make the control of falling or rising to the IEV register
    Clear_Bit(GPIO_PORTA_IEV_R,pin); //to trigger falling edge as we configure pin in pull up mode
    Set_Bit(GPIO_PORTA_ICR_R,pin); // to clear any other intterupt if happened on this corresponding pin ... clear interrupt flag
    Set_Bit(GPIO_PORTA_IM_R,pin); //to make the corresponding pin sent to the interrupt controller
   //Enable external interrupt from NVIC to port A
    Set_Bit(NVIC_EN0_R,P0); //enable interrput for portA ... known from CORTEX-M4F Processor --> Exception Model --> Interrupt table --> PORTA is number 0 priority number
    //Priority 
    NVIC_PRI0_R = NVIC_PRI0_R | 0xc0; //priority 7       1 1 1 -    - - - -  
    break;
  case PORTC:
    Clear_Bit(GPIO_PORTC_IS_R,pin); // to trigger edgess
    Clear_Bit(GPIO_PORTC_IBE_R,pin); //to make the control of falling or rising to the IEV register
    Clear_Bit(GPIO_PORTC_IEV_R,pin); //to trigger falling edge as we configure pin in pull up mode
    Set_Bit(GPIO_PORTC_ICR_R,pin); // to clear any other intterupt if happened on this corresponding pin ... clear interrupt flag
    Set_Bit(GPIO_PORTC_IM_R,pin); //to make the corresponding pin sent to the interrupt controller
   //Enable external interrupt from NVIC to port C
    Set_Bit(NVIC_EN0_R,P2); //enable interrput for portC ... known from CORTEX-M4F Processor --> Exception Model --> Interrupt table --> PORTA is number 0 priority number
    //Priority 
    NVIC_PRI0_R = NVIC_PRI0_R | 0xa00000; //priority 7       1 1 1 -    - - - - 
    break;
	case PORTE:
    Clear_Bit(GPIO_PORTE_IS_R,pin); // to trigger edgess
    Clear_Bit(GPIO_PORTE_IBE_R,pin); //to make the control of falling or rising to the IEV register
    Clear_Bit(GPIO_PORTE_IEV_R,pin); //to trigger falling edge as we configure pin in pull up mode
    Set_Bit(GPIO_PORTE_ICR_R,pin); // to clear any other intterupt if happened on this corresponding pin ... clear interrupt flag
    Set_Bit(GPIO_PORTE_IM_R,pin); //to make the corresponding pin sent to the interrupt controller
   //Enable external interrupt from NVIC to port F
    Set_Bit(NVIC_EN0_R,P4); //enable interrput for portE ... known from CORTEX-M4F Processor --> Exception Model --> Interrupt table --> PORTF is number 0 priority number
    //Priority 4 as    1 0 0 -    - - - -  as PRI7 resevred for interrupts 28 - 29 -30 -31
    // each priorty take 8 bits so we need to 00 for 28 / 00 for 29 / 100- --- for 30
    NVIC_PRI1_R = NVIC_PRI1_R | 0xa0; //priority 7
		break;
  case PORTF:
    Clear_Bit(GPIO_PORTF_IS_R,pin); // to trigger edgess
    Clear_Bit(GPIO_PORTF_IBE_R,pin); //to make the control of falling or rising to the IEV register
    Clear_Bit(GPIO_PORTF_IEV_R,pin); //to trigger falling edge as we configure pin in pull up mode
    Set_Bit(GPIO_PORTF_ICR_R,pin); // to clear any other intterupt if happened on this corresponding pin ... clear interrupt flag
    Set_Bit(GPIO_PORTF_IM_R,pin); //to make the corresponding pin sent to the interrupt controller
   //Enable external interrupt from NVIC to port F
    Set_Bit(NVIC_EN0_R,P30); //enable interrput for portF ... known from CORTEX-M4F Processor --> Exception Model --> Interrupt table --> PORTF is number 0 priority number
    //Priority 4 as    1 0 0 -    - - - -  as PRI7 resevred for interrupts 28 - 29 -30 -31
    // each priorty take 8 bits so we need to 00 for 28 / 00 for 29 / 100- --- for 30
    NVIC_PRI7_R = NVIC_PRI7_R | 0xa00000; //priority 7
  
    break;    
  }
}