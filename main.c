#include "tm4c123gh6pm.h"
#include "TM4C123.h"
#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOSConfig.h>
#include <queue.h>
#include <semphr.h>
#include <stdbool.h>
#include "DIO.h"


#define Set_Bit(Register,Bit) (Register |= (1<<Bit))
#define Clear_Bit(Register,Bit) (Register &= ~(1<<Bit))
#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5

#define PF0 0
#define PF1 1
#define PF2 2
#define PF3 3 
#define PF4 4
#define PF5 5
#define PF6 6
#define PF7 7

QueueHandle_t msgs;
SemaphoreHandle_t motor;

SemaphoreHandle_t driver_semaphore;
SemaphoreHandle_t passenger_semaphore;

SemaphoreHandle_t timer_d_u;
SemaphoreHandle_t timer_d_d;
SemaphoreHandle_t timer_p_u;
SemaphoreHandle_t timer_p_d;

SemaphoreHandle_t currTask = NULL;

bool passenger_lock = 0;
char is_rotating = 'N';
char is_limit_up = '0';
char is_limit_down = '0';

void my_handler(void);
void switch_handler(void);

void drive_motor(char direction){
	is_rotating = direction;
	if(direction == 'U'){
		GPIO_PORTE_DATA_R |= 0x10;
		GPIO_PORTE_DATA_R &= ~(0x20);
	}else if(direction == 'D'){
		GPIO_PORTE_DATA_R |= 0x20;
		GPIO_PORTE_DATA_R &= ~(0x10);
	}else{
		GPIO_PORTE_DATA_R &= ~(0x10);
		GPIO_PORTE_DATA_R &= ~(0x20);
	}
}

void buttonAinit(int pin){
		Clear_Bit(GPIO_PORTA_IS_R,pin); 
		Clear_Bit(GPIO_PORTA_IBE_R,pin);
		Clear_Bit(GPIO_PORTA_IEV_R,pin); 
		Set_Bit(GPIO_PORTA_ICR_R,pin);
		Set_Bit(GPIO_PORTA_IM_R,pin); 
}

void buttonBinit(int pin){
		Clear_Bit(GPIO_PORTB_IS_R,pin); 
		Clear_Bit(GPIO_PORTB_IBE_R,pin);
		Set_Bit(GPIO_PORTB_IEV_R,pin); 
		Set_Bit(GPIO_PORTB_ICR_R,pin);
		Set_Bit(GPIO_PORTB_IM_R,pin); 
}

void buttonEinit(int pin){
    Clear_Bit(GPIO_PORTE_IS_R,pin); 
    Clear_Bit(GPIO_PORTE_IBE_R,pin); 
    Clear_Bit(GPIO_PORTE_IEV_R,pin); 
    Set_Bit(GPIO_PORTE_ICR_R,pin);
    Set_Bit(GPIO_PORTE_IM_R,pin); 
}


void rising_handler(){
 	BaseType_t base = pdFALSE;
	char up = 'U';
	char down = 'D';
	if(GPIO_PORTB_MIS_R & 0x20){
		passenger_lock ^= 1;
	}
	if(((GPIO_PORTB_MIS_R & 0x04) && is_limit_up != '1') ){ // DRIVER UP
		if(currTask != NULL){
			xSemaphoreGiveFromISR(currTask, &base);
		}
		xSemaphoreGiveFromISR(driver_semaphore, &base);
		xQueueSendToBack(msgs, (void *)&up, (TickType_t)2);
		is_limit_down = '0';
		xSemaphoreGiveFromISR(timer_d_u, &base);
	}else if((GPIO_PORTB_MIS_R & 0x08) && is_limit_down != '1'){ // DRIVER DOWN
		if(currTask != NULL){
			xSemaphoreGiveFromISR(currTask, &base);
		}
		xSemaphoreGiveFromISR(driver_semaphore, &base);
		xQueueSendToBack(msgs, (void *)&down, (TickType_t)2);
		is_limit_up = '0';
		xSemaphoreGiveFromISR(timer_d_d, &base);
	}else if(((GPIO_PORTB_MIS_R & 0x40) && is_limit_up != '1') && passenger_lock == 0){ // PASSENGER UP
		if(currTask != NULL){
			xSemaphoreGiveFromISR(currTask, &base);
		}
		xSemaphoreGiveFromISR(passenger_semaphore, &base);
		xQueueSendToBack(msgs, (void *)&up, (TickType_t)2);
		is_limit_down = '0';
		xSemaphoreGiveFromISR(timer_p_u, &base);
	}else if((GPIO_PORTB_MIS_R & 0x80) && is_limit_down !='1' && passenger_lock == 0){ // PASSENGER DOWN
		if(currTask != NULL){
			xSemaphoreGiveFromISR(currTask, &base);
		}
		xSemaphoreGiveFromISR(passenger_semaphore, &base);
		xQueueSendToBack(msgs, (void *)&down, (TickType_t)2);
		is_limit_up = '0';
		xSemaphoreGiveFromISR(timer_p_d, &base);
	}
	GPIO_PORTB_ICR_R = 0xff;
	portEND_SWITCHING_ISR(base);
}

void falling_handler(){
	BaseType_t base = pdFALSE;
	if(((GPIO_PORTA_MIS_R & 0x04) && is_limit_up != '1') ){ // DRIVER UP
		xSemaphoreGiveFromISR(timer_d_u, &base);
	}else if((GPIO_PORTA_MIS_R & 0x08) && is_limit_down != '1'){ // DRIVER DOWN
		xSemaphoreGiveFromISR(timer_d_d, &base);
	}else if(((GPIO_PORTA_MIS_R & 0x40) && is_limit_up != '1') && passenger_lock == 0){ // PASSENGER UP
		xSemaphoreGiveFromISR(timer_p_u, &base);
	}else if((GPIO_PORTA_MIS_R & 0x80)&& is_limit_down != '1' && passenger_lock == 0){ // PASSENGER DOWN
		xSemaphoreGiveFromISR(timer_p_d, &base);
	}
	GPIO_PORTA_ICR_R = 0xff;
	portEND_SWITCHING_ISR(base);
}

void switch_handler(){
	BaseType_t base = pdFALSE;
	char none = 'N';
	char jam = 'J';
	if((GPIO_PORTE_MIS_R & 0x04) && (is_rotating == 'D')){
		is_limit_down = '1';
		xSemaphoreGiveFromISR(currTask, &base);
	}else if((GPIO_PORTE_MIS_R & 0x08) && (is_rotating == 'U')){
		is_limit_up = '1';
		xSemaphoreGiveFromISR(currTask, &base);
	}else if((GPIO_PORTE_MIS_R & 0x02) && is_rotating == 'U'){
		if(currTask != NULL){
			xSemaphoreGiveFromISR(currTask, &base);
		}
		xSemaphoreGiveFromISR(driver_semaphore, &base);
		xQueueSendToBack(msgs, (void *)&jam, (TickType_t)2);
	}
	GPIO_PORTE_ICR_R = 0xFF;
	portEND_SWITCHING_ISR(base);
}

void window_manager(void * vSem){
	while(1){
		SemaphoreHandle_t sem  = (SemaphoreHandle_t)vSem;
		xSemaphoreTake(sem,portMAX_DELAY);
		char msg;
		if(xQueueReceive(msgs, &msg, (TickType_t)portMAX_DELAY) == pdPASS){
			if(msg == 'J'){
				// jamming mode
				xSemaphoreTake(motor, (TickType_t)portMAX_DELAY);
				drive_motor('D');
				vTaskDelay((TickType_t)(1000 / portTICK_PERIOD_MS));
				drive_motor('N');
				xSemaphoreGive(motor);
			}else{
				xSemaphoreTake(motor, (TickType_t)portMAX_DELAY);
				currTask = sem;
				drive_motor(msg);
				xSemaphoreTake(sem, portMAX_DELAY);
				drive_motor('N');
				xSemaphoreGive(motor);
				currTask = NULL;
			}
		}
	}
}

void time_counter(void * vTimer){
	SemaphoreHandle_t timer = (SemaphoreHandle_t)vTimer;
	while(1){
		xSemaphoreTake(timer, portMAX_DELAY);
		TickType_t initial = xTaskGetTickCount();
		xSemaphoreTake(timer, portMAX_DELAY);
		TickType_t final = xTaskGetTickCount();
		int timeInMillis = (final - initial)*portTICK_PERIOD_MS;
		if(timeInMillis > 300){
			SemaphoreHandle_t myHandle = NULL;
			if(timer == timer_p_u || timer == timer_p_d){
				myHandle = passenger_semaphore;
			}else{
				myHandle = driver_semaphore;
			}
			if(currTask != NULL && currTask == myHandle){
				xSemaphoreGive(currTask);
			}
		}
	}
}


int main(){
	SYSCTL_RCGCGPIO_R |= 0x13;
	while((SYSCTL_PRGPIO_R & 0x13)==0);	// Enable clock for PORTA, PORTC and PORTF
	GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTA_CR_R = 0xFF;
	GPIO_PORTA_DEN_R  = 0xFF;
	GPIO_PORTA_DIR_R  = 0x00;
	GPIO_PORTA_PDR_R = 0xFF;
	
	GPIO_PORTB_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTB_CR_R = 0xFF;
	GPIO_PORTB_DEN_R  = 0xFF;
	GPIO_PORTB_DIR_R  = 0x00;
	GPIO_PORTB_PDR_R = 0xFF;
	
	GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTE_CR_R = 0xFF;
	GPIO_PORTE_DEN_R = 0x3E; 
	GPIO_PORTE_DIR_R = 0x30;
	GPIO_PORTE_PDR_R = 0x0E;
	
	
	Set_Bit(NVIC_EN0_R, 0); // enable interrupt for port A
	Set_Bit(NVIC_EN0_R,1); // enable inttrupt for port B
	Set_Bit(NVIC_EN0_R,4); // enable intrupt for port E
	
	// Set Port B interrupt priority to 7 
	NVIC_PRI0_R = 0xE0E0;

	// Set Port E interrupt priority to 7
	NVIC_PRI1_R = 0xE0;
	
	buttonAinit(PF2);
	buttonAinit(PF3);
	buttonAinit(PF6);
	buttonAinit(PF7);
	
	buttonBinit(PF2);
	buttonBinit(PF3);
	buttonBinit(PF5);
	buttonBinit(PF6);
	buttonBinit(PF7);
	
	buttonEinit(PF1);
	buttonEinit(PF2);
	buttonEinit(PF3);
	
	msgs = xQueueCreate(1, sizeof(char));
	driver_semaphore = xSemaphoreCreateCounting(2,0);
	passenger_semaphore = xSemaphoreCreateCounting(2,0);
	motor = xSemaphoreCreateMutex();
	
	timer_d_u = xSemaphoreCreateCounting(2,0);
	timer_d_d = xSemaphoreCreateCounting(2,0);
	timer_p_u = xSemaphoreCreateCounting(2,0);
	timer_p_d = xSemaphoreCreateCounting(2,0);
	
	xTaskCreate(window_manager,"driver",240,(void*)(driver_semaphore),2,NULL);
	xTaskCreate(window_manager,"passenger",240,(void*)(passenger_semaphore),2,NULL);
	
	xTaskCreate(time_counter,"driverUP",240,(void*)(timer_d_u),3,NULL);
	xTaskCreate(time_counter,"driverDWN",240,(void*)(timer_d_d),3,NULL);
	xTaskCreate(time_counter,"passUP",240,(void*)(timer_p_u),3,NULL);
	xTaskCreate(time_counter,"passDWN",240,(void*)(timer_p_d),3,NULL);
	vTaskStartScheduler();
	
	return 0;
}