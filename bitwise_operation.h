#ifndef __bitwise_operation_H_ 
#define __bitwise_operation_H_

#define Set_Bit(Register,Bit) (Register |= (1<<Bit))
#define Clear_Bit(Register,Bit) (Register &= ~(1<<Bit))
#define Get_Bit(Register,Bit) ((Register & (1<<Bit)) >> Bit)
#define Toggle_Bit(Register,Bit) (Register ^= (1<<Bit)) 

#endif