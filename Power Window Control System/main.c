/*
Priorities Map:

sporadic tasks (jamming , locking , limits) -> 4 (highest)
driver tasks when called form passeneger tasks -> 3 
driver and passeneger tasks when called from normal polling task -> 2
polling (button task) -> 1 (lowest)


NB: ofcourse hardware interrupts preempts all the above tasks 

*/



//needed Libraries
#include "inc/hw_memmap.h"
#include <stdbool.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOSConfig.h>
#include "tm4c123gh6pm.h"
#include "timers.h"
#include "lcd.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "semphr.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "inc/hw_gpio.h"
#include "driverlib/pwm.h"
#include "motor.h"

//defines needed in code
#define DEBOUNCE_DELAY_MS 50
#define LONG_PRESS_DELAY_MS 1000


//strings to be printed on lcd to display the current state and any actions encountered
static const char *pctextforlcd1 = "driver up auto";
static const char *pctextforlcd2 = "driver up manu";
static const char *pctextforlcd3 = "driver down auto";
static const char *pctextforlcd4 = "driver down manu";
static const char *pctextforlcd5 = "pass up auto";
static const char *pctextforlcd6 = "pass up manu";
static const char *pctextforlcd7 = "pass down auto";
static const char *pctextforlcd8 = "pass down manu";

//global variables needed to check for jamming , locking window , window limits 
static volatile int jamDetected = 0;
static volatile int windowLock = 0;
static volatile bool upperLimitReached = false;
static volatile bool lowerLimitReached = false;

// function declarations
static void vlcdWrite(void *pvParameters);
void portCinit();
void portEinit();
void buttonTask(void *pvParameters);
void drivUpAuto(void *pvParameters);
void drivDownAuto(void *pvParameters);
void drivUpManu(void *pvParameters);
void drivDownManu(void *pvParameters);
void passUpAuto(void *pvParameters);
void passDownAuto(void *pvParameters);
void passUpManu(void *pvParameters);
void passDownManu(void *pvParameters);
void setUpperLimit(void *pvParameters);
void setLowerLimit(void *pvParameters);
void setWindowLock(void *pvParameters);
void setJamDetected(void *pvParameters);


//Tasks Handles for all tasks (needed to delete tasks) 
TaskHandle_t xdrivUpAutoHandle;
TaskHandle_t xdrivUpManuHandle;
TaskHandle_t xdrivDownAutoHandle;
TaskHandle_t xdrivDownManuHandle;
TaskHandle_t xpassUpAutoHandle;
TaskHandle_t xpassUpManuHandle;
TaskHandle_t xpassDownAutoHandle;
TaskHandle_t xpassDownManuHandle;


//Binarysemaphores needed for (sporadic) tasks that occur because of an interrupt
SemaphoreHandle_t xupperLimitReachedSemaphore;
SemaphoreHandle_t xlowerLimitReachedSemaphore;
SemaphoreHandle_t xwindowLockSemaphore;
SemaphoreHandle_t xjamDetectedSemaphore;


/*Queue needed for tasks to pass handles between one another
(first task passes its handle to second task in queue so that the second task can know which task to delete  )*/
QueueHandle_t xQueue;

//---------------------------------------------------------------------////---------------------------------------------------------------------//
//---------------------------------------------------------------------////---------------------------------------------------------------------//

/*Mutex needed to manage pieces of code that can only be performed by one task at a time 
(in our case it is used so that push buttons of driver can only be read one at a time from different tasks 
to deny multiple unneeded calls to driver tasks)*/
SemaphoreHandle_t xDriverUpButtonMutex;
SemaphoreHandle_t xDriverDownButtonMutex;
//---------------------------------------------------------------------////---------------------------------------------------------------------//
//---------------------------------------------------------------------////---------------------------------------------------------------------//



//Task handle that is passed into the queue for task to task communication
TaskHandle_t xTaskHandlePassed;

int main(){

	// Create a mutex semaphores for driver buttons
    xDriverUpButtonMutex = xSemaphoreCreateMutex();
	xDriverDownButtonMutex = xSemaphoreCreateMutex();
	
	// create queue 
	xQueue = xQueueCreate(1, sizeof(TaskHandle_t));
	
	
	// Create the binary semaphores
    xupperLimitReachedSemaphore = xSemaphoreCreateBinary();
	xlowerLimitReachedSemaphore = xSemaphoreCreateBinary();
	xwindowLockSemaphore  = xSemaphoreCreateBinary();
	xjamDetectedSemaphore = xSemaphoreCreateBinary();
	
	//initlaize PORT C (push buttons for driver and passeneger buttons)
	portCinit();
	
	//initialize PORT E (buttons for jamming , window lock , upper and lower limit switches)
	portEinit();
	
	// initialize LCD 
	LCD_init();
	LCD_Clear();//clear LCD
	motorinit();//init motor 

	// Create the sporadic tasks (will be blocked initially because binary semaphore is empty / not given)
	// have highest priorities
    xTaskCreate(setUpperLimit,  "setupper", 128, NULL, 4, NULL);
	xTaskCreate(setLowerLimit,  "setlower", 128, NULL, 4, NULL);
	xTaskCreate(setWindowLock,  "setwindowlock", 128, NULL, 4, NULL);
	xTaskCreate(setJamDetected, "setjamdetected", 128, NULL, 4, NULL);

	
  // Create the button task (continious tasks that polls on driver and passenger buttons to check if clicked on
  xTaskCreate( buttonTask, "Button Task", 128, NULL, 1, NULL );

	vTaskStartScheduler();//start scheduler
		
}



/*
This tasks polls on every button belonging to driver and passenger and detect if the button was 
pressed and released instantly (Automatic Mode) or was pressed for a while (2 seconds)(Manual Mode)
*/

void buttonTask(void *pvParameters)
{	
    // Define variables to keep track of button states and times
		TickType_t xLastWakeTime;
    const TickType_t xDelay = pdMS_TO_TICKS(1);//delay of 1 ms
    xLastWakeTime = xTaskGetTickCount();
	  int pc4_pressed = 0;
    int pc5_pressed = 0;
    int pc4_long_pressed = 0;
    int pc5_long_pressed = 0;
		 int pc6_pressed = 0;
    int pc7_pressed = 0;
    int pc6_long_pressed = 0;
    int pc7_long_pressed = 0;
		
		while (1){
			
      //Take Mutex of driver up button to read its state (blocked if mutex is with another task later on)
			if (xSemaphoreTake(xDriverUpButtonMutex, portMAX_DELAY) == pdTRUE){
				//check if driver up button was pressed
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
            if (!pc4_pressed) {
                pc4_pressed = 1;
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));//delay for debounce
                if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
                    // PC4 button is still pressed
                    xLastWakeTime = xTaskGetTickCount();//start stopwatch
                    while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        vTaskDelay(xDelay);//delay and stay here IF button is still pressed and wait for 1 seconds
                    }
                    if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												// PC4 button has been pressed for 2 seconds (Manual Mode)
												// create driver task to move window up manually and give higher priority to preempt the current task
												xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 2, &xdrivUpManuHandle );
												LCD_Clear();											
										} 
										else {
											// PC4 button has been pressed and released almost immediately (Automatic Mode)
											// create driver task (with handler) to move window up automatically and give higher priority to preempt the current task
											xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 2, &xdrivUpAutoHandle );
											LCD_Clear();
                    }
                }
                pc4_pressed = 0;
            }
						
        }	
				pc4_pressed = 0;
				//Release mutex after finished reading the button state				
				xSemaphoreGive(xDriverUpButtonMutex);
			}
				
			//Take Mutex of driver down button to read its state (blocked if mutex is with another task later on)
			if (xSemaphoreTake(xDriverDownButtonMutex, portMAX_DELAY) == pdTRUE){	
					//check if driver down button was pressed
					if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
							if (!pc5_pressed) {
									pc5_pressed = 1;
									vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));//delay for debounce
									if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
											// PC5 button is still pressed
											xLastWakeTime = xTaskGetTickCount();//start stopwatch
											while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
													vTaskDelay(xDelay);//delay and stay here IF button is still pressed and wait for 2 seconds
											}
											if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
													// PC5 button has been pressed for 2 seconds (Manual Mode)
													// create driver task (with handler) to move window down Manually and give higher priority to preempt the current task
													xTaskCreate( drivDownManu, "driverDownManu", 128, NULL, 2, &xdrivDownManuHandle );
													LCD_Clear();
												
											} else {
													// PC5 button has been pressed and released almost immediately (Automatic Mode)
													// create driver task (with handler) to move window down automatically and give higher priority to preempt the current task
													xTaskCreate( drivDownAuto, "driverDownAuto", 128, NULL, 2, &xdrivDownAutoHandle );
													LCD_Clear();
											}
									}
									pc5_pressed = 0;
							}
							
					}
					pc5_pressed = 0;
					//Release mutex after finished reading the button state
					xSemaphoreGive(xDriverDownButtonMutex);
				}
					
				
			
			//check if passenger up button was pressed (same as driver)
			if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) {
					if (!pc6_pressed) {
							pc6_pressed = 1;
							vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
							if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) {
									// PC6 button is still pressed
									xLastWakeTime = xTaskGetTickCount();
									while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											vTaskDelay(xDelay);
									}
									if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											// PC6 button has been pressed for 2 seconds (Manual Mode)
											xTaskCreate( passUpManu, "passUpManu", 128, NULL, 2, &xpassUpManuHandle );
											LCD_Clear();
											pc6_long_pressed = 1;
										
									} else {
											// PC6 button has been pressed and released almost immediately (Automatic Mode)
											xTaskCreate( passUpAuto, "passUpAuto", 128, NULL, 2, &xpassUpAutoHandle );
											LCD_Clear();											
									}
							}
							pc6_pressed = 0;
					}
					
			}	
			pc6_pressed = 0;	

			//check if passenger down button was pressed (same as driver)
			if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) {
					if (!pc7_pressed) {
							pc7_pressed = 1;
							vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
							if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) {
									// PC7 button is still pressed
									xLastWakeTime = xTaskGetTickCount();
									while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											vTaskDelay(xDelay);
									}
									if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											// PC7 button has been pressed for 2 seconds (Manual Mode)
											//vlcdWrite((void*)pctextforlcd8);
											xTaskCreate( passDownManu, "passDownAuto", 128, NULL, 2, &xpassDownManuHandle );
											LCD_Clear();
											pc7_long_pressed = 1;
										
									} else {
											// PC7 button has been pressed and released almost immediately (Automatic Mode)
											//vlcdWrite((void*)pctextforlcd7);
											xTaskCreate( passDownAuto, "passDownManu", 128, NULL, 2, &xpassDownAutoHandle );
											LCD_Clear();
									}
							}
							pc7_pressed = 0;
					}
					
			}	
			pc7_pressed = 0;					
	}

}







//DRIVER TASKS

// This Task is needed to move the window Up automatically by Driver

void drivUpAuto(void *pvParameters)
{
	LCD_Clear();
	TaskHandle_t xReceivedHandle;// handle needed if this task preempted the passneger actions (beacuse driver have the higher action priority) 
	
	//check if queue contained a handler for a passeneger task to delete it (Because i dont want to return to a passneger task if i was called form one)
	if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
			vTaskDelete(xReceivedHandle);//delete passeneger task
	}
	vlcdWrite((void*)pctextforlcd1);//print on LCD driver up auto
	
	//keep moving window up as long as not upper limit detected , no jamming and no window lock engaged
	while((!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//TODO:turn motor uppp
		motor_forward();
		lowerLimitReached = false;//beacuse window moved up
	}
	
	//if window lock engaged stop motor from turning and display
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xdrivUpAutoHandle);//delete task 
	}
	
	// if jamming was detected
	if(jamDetected == 1){
		//TODO:implement jam detection protocol (stop motor , move down for 0.5 sec)
		motor_stop();
		TickType_t xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_backward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xdrivUpAutoHandle);//delete task
		
	}
	
	// if upper limit is reached (stop motor)
	if(upperLimitReached){
		//TODO:stop motor
		motor_stop();
		LCD_PrintLn(1,"Upper limit");
		LCD_Clear();
		vTaskDelete(xdrivUpAutoHandle);//delete task
	}


	vTaskDelete(xdrivUpAutoHandle);//defensive deletion (precautionary)
}


// This Task is needed to move the window UP Manually by Driver 

void drivUpManu(void *pvParameters)
{	
	LCD_Clear();
	TaskHandle_t xReceivedHandle;
	if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
			vTaskDelete(xReceivedHandle);
	}
	vlcdWrite((void*)pctextforlcd2);
	
	//Extra step : keep moving motor as long as button is pressed because this is manual mode
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && (!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//TODO:turn motor uppp
		motor_forward();
		lowerLimitReached = false;
	}
	
	//Extra step : if button is released (stop motor)
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)){
		motor_stop();
		LCD_PrintLn(1,"button release");
		LCD_Clear();
		vTaskDelete(xdrivUpManuHandle);
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xdrivUpManuHandle);
	}
	if(jamDetected == 1){
		//TODO: implement jam detection protocol
		motor_stop();
		TickType_t xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_backward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xdrivUpManuHandle);
	}
	if(upperLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Upper limit");
		LCD_Clear();
		vTaskDelete(xdrivUpManuHandle);
	}

	vTaskDelete(xdrivUpManuHandle);
}




// This Task is needed to move the window Down Automatically by Driver

void drivDownAuto(void *pvParameters)
{	
	LCD_Clear();
	TaskHandle_t xReceivedHandle;
	if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
			vTaskDelete(xReceivedHandle);
	}
	vlcdWrite((void*)pctextforlcd3);
	while((!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){//ask if jam detected needed here??
		//TODO: Turn motor downnn
		motor_backward();
		upperLimitReached = false;
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xdrivDownAutoHandle);
	}
	if(jamDetected == 1){
		motor_stop();
		TickType_t xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_forward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xdrivDownAutoHandle);
	}
	if(lowerLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Lower limit");
		LCD_Clear();
		vTaskDelete(xdrivDownAutoHandle);
	}

	vTaskDelete(xdrivDownAutoHandle);
}




// This Task is needed to move the window Down Manually by Driver

void drivDownManu(void *pvParameters)
{
	LCD_Clear();
	TaskHandle_t xReceivedHandle;
	if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
			vTaskDelete(xReceivedHandle);
	}	
	vlcdWrite((void*)pctextforlcd4);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && (!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//TODO: Turn motor downnn
		motor_backward();
		upperLimitReached = false;
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)){
		motor_stop();
		LCD_PrintLn(1,"button release");
		LCD_Clear();
		vTaskDelete(xdrivDownManuHandle);
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xdrivDownManuHandle);
	}
	if(jamDetected == 1){
		motor_stop();
		TickType_t xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_forward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xdrivDownManuHandle);
	}
	if(lowerLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Lower limit");
		LCD_Clear();
		vTaskDelete(xdrivDownManuHandle);
	}

	vTaskDelete(xdrivDownManuHandle);
}













//PASSENGER TASKS

// This Task is needed to move the window Up Automatically by Passenger
void passUpAuto(void *pvParameters)
{
	// Define variables to keep track of button states and times
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(1);
	int pc4_pressed = 0;
	int pc5_pressed = 0;
	int pc4_long_pressed = 0;
	int pc5_long_pressed = 0;
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd5);
	while((!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//TODO: Turn motor uppp
		motor_forward();
		lowerLimitReached = false;
		
		//Extra Step : Poll on Driver Buttons to check if pressed and preempt the current task to go to driver of pressed
		// beacuse driver's actions have a higher priority than the passeneger's actions
		
		//take Mutex to read buttons (here mutex was needed becasue any delays here caused the task to block and if a button was pressed
		// the polling task (button task) may read button and so the driver task will be performed twice so we needed mutex to lock polling on button).
		//same polling mechanism as the (button Task) with extra steps explained below
		if (xSemaphoreTake(xDriverUpButtonMutex, portMAX_DELAY) == pdTRUE){
			if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
					if (!pc4_pressed) {
							pc4_pressed = 1;
							vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
							if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
									// PC4 button is still pressed
									xLastWakeTime = xTaskGetTickCount();
									while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											vTaskDelay(xDelay);
									}
									if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
										// PC4 button has been pressed for 2 seconds (Manual Mode)
										LCD_Clear();
										// retrieve the current tasks's Handle and pass it to the queue so that the current task can be deleted from driver task
										// in order not to return to it again after finishing driver task
										xTaskHandlePassed = xTaskGetCurrentTaskHandle();
										xQueueSend(xQueue, &xTaskHandlePassed, 0);
										xSemaphoreGive(xDriverUpButtonMutex);//give up mutex 
										//create driver task and give higher priority than current task (3) to preempt the current task
										xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 3, &xdrivUpManuHandle );
										LCD_Clear();//should never reach here
										
									} 
									else {
										// PC4 button has been pressed and released almost immediately (Automatic Mode)
										LCD_Clear();
										// retrieve the current tasks's Handle and pass it to the queue so that the current task can be deleted from driver task
										// in orer not to return to it again after finishing driver task										
										xTaskHandlePassed = xTaskGetCurrentTaskHandle();
										xQueueSend(xQueue, &xTaskHandlePassed, 0);
										xSemaphoreGive(xDriverUpButtonMutex);//give up mutex 
										//create driver task and give higher priority than current task (3) to preempt the current task
										xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 3, &xdrivUpAutoHandle );
										LCD_Clear();//should never reach here
									}
							}
							pc4_pressed = 0;
					}
							
			 }
			xSemaphoreGive(xDriverUpButtonMutex);//give up mutex
		 }
		if (xSemaphoreTake(xDriverDownButtonMutex, portMAX_DELAY) == pdTRUE){	
				if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
						if (!pc5_pressed) {
								pc5_pressed = 1;
								vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
								if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
										// PC5 button is still pressed
										xLastWakeTime = xTaskGetTickCount();
										while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												vTaskDelay(xDelay);
										}
										if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												// PC5 button has been pressed for 2 seconds (Manual Mode)
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);
												xTaskCreate( drivDownManu, "driverDownManu", 128, NULL, 3, &xdrivDownManuHandle );
												LCD_Clear();
											
										} else {
												// PC5 button has been pressed and released almost immediately (Automatic Mode)
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);										
												xTaskCreate( drivDownAuto, "driverDownAuto", 128, NULL, 3, &xdrivDownAutoHandle );
												LCD_Clear();
										}
								}
								pc5_pressed = 0;
						}
						
				}
				pc5_pressed = 0;
				xSemaphoreGive(xDriverDownButtonMutex);
			}
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xpassUpAutoHandle);
	}
	if(jamDetected == 1){
		motor_stop();
		xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_backward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xpassUpAutoHandle);
	}
	if(upperLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Upper limit");
		LCD_Clear();
		vTaskDelete(xpassUpAutoHandle);
	}

	vTaskDelete(xpassUpAutoHandle);
}





void passUpManu(void *pvParameters)
{
	// Define variables to keep track of button states and times
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(1);
	int pc4_pressed = 0;
	int pc5_pressed = 0;
	int pc4_long_pressed = 0;
	int pc5_long_pressed = 0;
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd6);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) && (!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//TODO:turn motor uppp
		motor_forward();
		lowerLimitReached = false;
		
		//Extra Steps
		if (xSemaphoreTake(xDriverUpButtonMutex, portMAX_DELAY) == pdTRUE){
			if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
					if (!pc4_pressed) {
							pc4_pressed = 1;
							vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
							if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
									// PC4 button is still pressed
									xLastWakeTime = xTaskGetTickCount();
									while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											vTaskDelay(xDelay);
									}
									if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											// PC4 button has been pressed for 2 seconds (Manual Mode)
											LCD_Clear();
											xTaskHandlePassed = xTaskGetCurrentTaskHandle();
											xQueueSend(xQueue, &xTaskHandlePassed, 0);
											xSemaphoreGive(xDriverUpButtonMutex);
											xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 3, &xdrivUpManuHandle );
											LCD_Clear();
										
									} 
									else {
										// PC4 button has been pressed and released almost immediately (Automatic Mode)
										LCD_Clear();
										xTaskHandlePassed = xTaskGetCurrentTaskHandle();
										xQueueSend(xQueue, &xTaskHandlePassed, 0);
										xSemaphoreGive(xDriverUpButtonMutex);
										xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 3, &xdrivUpAutoHandle );
										LCD_Clear();
									}
							}
							pc4_pressed = 0;
					}
							
			 }
			xSemaphoreGive(xDriverUpButtonMutex);
		 }
		if (xSemaphoreTake(xDriverDownButtonMutex, portMAX_DELAY) == pdTRUE){	
				if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
						if (!pc5_pressed) {
								pc5_pressed = 1;
								vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
								if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
										// PC5 button is still pressed
										xLastWakeTime = xTaskGetTickCount();
										while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												vTaskDelay(xDelay);
										}
										if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												// PC5 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd4);
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);
												xTaskCreate( drivDownManu, "driverDownManu", 128, NULL, 3, &xdrivDownManuHandle );
												LCD_Clear();
												//pc5_long_pressed = 1;
											
										} else {
												// PA1 button has been pressed and released almost immediately (auto)
												//vlcdWrite((void*)pctextforlcd3);
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);										
												xTaskCreate( drivDownAuto, "driverDownAuto", 128, NULL, 3, &xdrivDownAutoHandle );
												LCD_Clear();
										}
								}
								pc5_pressed = 0;
						}
						
				}
				pc5_pressed = 0;
				xSemaphoreGive(xDriverDownButtonMutex);
			}
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)){
		motor_stop();
		LCD_PrintLn(1,"button release");
		LCD_Clear();
		vTaskDelete(xpassUpManuHandle);
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xpassUpManuHandle);
	}
	if(jamDetected == 1){
		motor_stop();
		xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_backward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xpassUpManuHandle);
	}
	if(upperLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Upper limit");
		LCD_Clear();
		vTaskDelete(xpassUpManuHandle);
	}
	vTaskDelete(xpassUpManuHandle);
}





void passDownAuto(void *pvParameters)
{
	// Define variables to keep track of button states and times
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(1);
	int pc4_pressed = 0;
	int pc5_pressed = 0;
	int pc4_long_pressed = 0;
	int pc5_long_pressed = 0;
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd7);
	while((!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){//ask if jam detected needed here??
		//TODO: Turn motor downnn
		motor_backward();
		upperLimitReached = false;
		
		//Extra steps
		if (xSemaphoreTake(xDriverUpButtonMutex, portMAX_DELAY) == pdTRUE){
			if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
					if (!pc4_pressed) {
							pc4_pressed = 1;
							vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
							if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
									// PC4 button is still pressed
									xLastWakeTime = xTaskGetTickCount();
									while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											vTaskDelay(xDelay);
									}
									if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											// PA0 button has been pressed for 2 seconds (manu)
											//vlcdWrite((void*)pctextforlcd2);
											LCD_Clear();
											xTaskHandlePassed = xTaskGetCurrentTaskHandle();
											xQueueSend(xQueue, &xTaskHandlePassed, 0);
											xSemaphoreGive(xDriverUpButtonMutex);
											xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 3, &xdrivUpManuHandle );
											LCD_Clear();
											//pc4_long_pressed = 1;
										
									} 
									else {
										// PA0 button has been pressed and released almost immediately (auto)
										//vlcdWrite((void*)pctextforlcd1);
										LCD_Clear();
										xTaskHandlePassed = xTaskGetCurrentTaskHandle();
										xQueueSend(xQueue, &xTaskHandlePassed, 0);
										xSemaphoreGive(xDriverUpButtonMutex);
										xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 3, &xdrivUpAutoHandle );
										LCD_Clear();
									}
							}
							pc4_pressed = 0;
					}
							
			 }
			xSemaphoreGive(xDriverUpButtonMutex);
		 }
		if (xSemaphoreTake(xDriverDownButtonMutex, portMAX_DELAY) == pdTRUE){	
				if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
						if (!pc5_pressed) {
								pc5_pressed = 1;
								vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
								if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
										// PC5 button is still pressed
										xLastWakeTime = xTaskGetTickCount();
										while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												vTaskDelay(xDelay);
										}
										if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												// PC5 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd4);
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);
												xTaskCreate( drivDownManu, "driverDownManu", 128, NULL, 3, &xdrivDownManuHandle );
												LCD_Clear();
												//pc5_long_pressed = 1;
											
										} else {
												// PA1 button has been pressed and released almost immediately (auto)
												//vlcdWrite((void*)pctextforlcd3);
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);										
												xTaskCreate( drivDownAuto, "driverDownAuto", 128, NULL, 3, &xdrivDownAutoHandle );
												LCD_Clear();
										}
								}
								pc5_pressed = 0;
						}
						
				}
				pc5_pressed = 0;
				xSemaphoreGive(xDriverDownButtonMutex);
			}
		
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xpassDownAutoHandle);
	}
	if(jamDetected == 1){
		motor_stop();
		xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_forward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();
		vTaskDelete(xpassDownAutoHandle);
	}
	if(lowerLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Lower limit");
		LCD_Clear();
		vTaskDelete(xpassDownAutoHandle);
	}

	vTaskDelete(xpassDownAutoHandle);
}





void passDownManu(void *pvParameters)
{
	// Define variables to keep track of button states and times
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(1);
	//xLastWakeTime = xTaskGetTickCount();
	int pc4_pressed = 0;
	int pc5_pressed = 0;
	int pc4_long_pressed = 0;
	int pc5_long_pressed = 0;
	LCD_Clear();	
	vlcdWrite((void*)pctextforlcd8);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) && (!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//TODO: Turn motor downnn
		motor_backward();
		upperLimitReached = false;	
		
		//Extra Steps
		if (xSemaphoreTake(xDriverUpButtonMutex, portMAX_DELAY) == pdTRUE){
			if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
					if (!pc4_pressed) {
							pc4_pressed = 1;
							vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
							if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
									// PC4 button is still pressed
									xLastWakeTime = xTaskGetTickCount();
									while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											vTaskDelay(xDelay);
									}
									if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
											// PA0 button has been pressed for 2 seconds (manu)
											//vlcdWrite((void*)pctextforlcd2);
											LCD_Clear();
											xTaskHandlePassed = xTaskGetCurrentTaskHandle();
											xQueueSend(xQueue, &xTaskHandlePassed, 0);
											xSemaphoreGive(xDriverUpButtonMutex);
											xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 3, &xdrivUpManuHandle );
											LCD_Clear();
											//pc4_long_pressed = 1;
										
									} 
									else {
										// PA0 button has been pressed and released almost immediately (auto)
										//vlcdWrite((void*)pctextforlcd1);
										LCD_Clear();
										xTaskHandlePassed = xTaskGetCurrentTaskHandle();
										xQueueSend(xQueue, &xTaskHandlePassed, 0);
										xSemaphoreGive(xDriverUpButtonMutex);
										xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 3, &xdrivUpAutoHandle );
										LCD_Clear();
									}
							}
							pc4_pressed = 0;
					}
							
			 }
			xSemaphoreGive(xDriverUpButtonMutex);
		 }
		if (xSemaphoreTake(xDriverDownButtonMutex, portMAX_DELAY) == pdTRUE){	
				if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
						if (!pc5_pressed) {
								pc5_pressed = 1;
								vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
								if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
										// PC5 button is still pressed
										xLastWakeTime = xTaskGetTickCount();
										while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												vTaskDelay(xDelay);
										}
										if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												// PC5 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd4);
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);
												xTaskCreate( drivDownManu, "driverDownManu", 128, NULL, 3, &xdrivDownManuHandle );
												LCD_Clear();
												//pc5_long_pressed = 1;
											
										} else {
												// PA1 button has been pressed and released almost immediately (auto)
												//vlcdWrite((void*)pctextforlcd3);
												LCD_Clear();
												xTaskHandlePassed = xTaskGetCurrentTaskHandle();
												xQueueSend(xQueue, &xTaskHandlePassed, 0);
												xSemaphoreGive(xDriverDownButtonMutex);										
												xTaskCreate( drivDownAuto, "driverDownAuto", 128, NULL, 3, &xdrivDownAutoHandle );
												LCD_Clear();
										}
								}
								pc5_pressed = 0;
						}
						
				}
				pc5_pressed = 0;
				xSemaphoreGive(xDriverDownButtonMutex);
			}		
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)){
		motor_stop();
		LCD_PrintLn(1,"button release");
		LCD_Clear();
		vTaskDelete(xpassDownManuHandle);
	}
	if(windowLock == 1){
		motor_stop();
		LCD_PrintLn(1,"Window lock");
		LCD_Clear();
		vTaskDelete(xpassDownManuHandle);
	}
	if(jamDetected == 1){
		motor_stop();
		xLastWakeTime =  xTaskGetTickCount();	
		LCD_PrintLn(1,"Jam detected");
		while((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(1000)){
			motor_forward();
		}
		motor_stop();
		jamDetected = 0;
		LCD_Clear();;
		vTaskDelete(xpassDownManuHandle);
	}
	if(lowerLimitReached){
		motor_stop();
		LCD_PrintLn(1,"Lower limit");
		LCD_Clear();
		vTaskDelete(xpassDownManuHandle);
	}
	vTaskDelete(xpassDownManuHandle);
}

















//Sporadic Tasks To change state of jamming , locking and limits

void setUpperLimit(void *pvParameters)
{
	xSemaphoreTake(xupperLimitReachedSemaphore, 0);//defensive mechanism
  // Wait for the semaphore
	while(1){
  xSemaphoreTake(xupperLimitReachedSemaphore, portMAX_DELAY);//semaphore is given in ISR
		
	//change upper limit flag to true to indicate upper limit reached	
	upperLimitReached = true;
	}
}


void setLowerLimit(void *pvParameters)
{

  xSemaphoreTake(xlowerLimitReachedSemaphore, 0);//defensive mechanism
	// Wait for the semaphore
	while(1){
  xSemaphoreTake(xlowerLimitReachedSemaphore, portMAX_DELAY);//semaphore is given in ISR
	
	//change lower limit flag to true to indicate lower limit reached	
	lowerLimitReached = true;
	}
}


void setWindowLock(void *pvParameters)
{

	xSemaphoreTake(xwindowLockSemaphore, 0);//defensive mechanism
	// Wait for the semaphore
	while(1){
		xSemaphoreTake(xwindowLockSemaphore, portMAX_DELAY);//semaphore is given in ISR
		
		//toggle window lock accordingly
		if (windowLock == 1){
				windowLock = 0;
		}
		else 
			windowLock = 1;
	}
}


void setJamDetected(void *pvParameters)
{
	
	xSemaphoreTake(xjamDetectedSemaphore, 0);
	// Wait for the semaphore
	while(1){
		xSemaphoreTake(xjamDetectedSemaphore, portMAX_DELAY);//semaphore is given in ISR
		
		//change jam detected flag to 1 to indicate jam was detected and initiate jamming protocol
		jamDetected = 1;
	}
 
}

















//not needed anymore (call lcd print directly)
static void vlcdWrite(void *pvParameters){
	
	char *pclcdprint = (char*)pvParameters;
	LCD_PrintLn(0,pclcdprint);
}



// Interrupt handler for the 4 interrupt buttons (jamming , locking , upper limit , lower limit)
void PortE_ISR_Handler(void)
{
    uint32_t interrupt_status = GPIOIntStatus(GPIO_PORTE_BASE, true); // Get the interrupt status of the GPIO port E
	
		//initalize higherpriortytaskwoken to false that would be changed to true if semaphore giving  unblocked a task with higher priority than 
		//current working task (in our case we gave these tasks highest priority (4) to preempt any other task)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		//check which pin in PORT E caused the interrupt
    if(interrupt_status & GPIO_PIN_1) // Check if Pin 1 caused the interrupt
    {
        // Pin 1 was pressed (Upper Limit Reached)
				//pass semaphore to appropriate task to unblock it
				xSemaphoreGiveFromISR(xupperLimitReachedSemaphore, &xHigherPriorityTaskWoken);
    }

    if(interrupt_status & GPIO_PIN_2) // Check if Pin 2 caused the interrupt
    {
        // Pin 2 was pressed (lower Limit reached)
				//pass semaphore to appropriate task to unblock it
				xSemaphoreGiveFromISR(xlowerLimitReachedSemaphore, &xHigherPriorityTaskWoken);
    }

    if(interrupt_status & GPIO_PIN_3) // Check if Pin 3 caused the interrupt
    {
        // Pin 3 was pressed (window Lock Engaged)
				//pass semaphore to appropriate task to unblock it
				xSemaphoreGiveFromISR(xwindowLockSemaphore, &xHigherPriorityTaskWoken);
    }
    }

    if(interrupt_status & GPIO_PIN_4) // Check if Pin 4 caused the interrupt
    {
        // Pin 4 was pressed (jamming Detected)
				//pass semaphore to appropriate task to unblock it
				xSemaphoreGiveFromISR(xjamDetectedSemaphore, &xHigherPriorityTaskWoken);
    }
		GPIOIntClear(GPIO_PORTE_BASE, interrupt_status); // Clear the interrupt status of the GPIO port E
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //force context switch 
}









//PORT C initializations
void portCinit(){
	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);//input
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//pull-up


}








//PORT E initializations
void portEinit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable the GPIO port E

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);//input
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); //pull-up

    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_FALLING_EDGE); // Set Pins 1 to 4 as interrupts on falling edge
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4); // Enable interrupts for Pins 1 to 4
	  // Register the ISR in the vector table
    IntRegister(INT_GPIOE, PortE_ISR_Handler);

    // Set the priority of the interrupt
    IntPrioritySet(INT_GPIOE, 255);

    // Enable the interrupt in the NVIC
    IntEnable(INT_GPIOE);
	
		//enable cpu interrupts
		IntMasterEnable();
}







