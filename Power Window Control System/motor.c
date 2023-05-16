
#include "motor.h"
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

void motorinit() {
    // Initialize GPIO pins for L298N control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, ENA_PIN | IN1_PIN | IN2_PIN | ENB_PIN | IN3_PIN | IN4_PIN);
}




void motor_forward() {
    // Drive motors forward
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN3_PIN, IN1_PIN | IN3_PIN);
    GPIOPinWrite(GPIO_PORTA_BASE, IN2_PIN | IN4_PIN, 0);
}



void motor_backward() {
    // Drive motors backward
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN3_PIN, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, IN2_PIN | IN4_PIN, IN2_PIN | IN4_PIN);
}



void motor_stop() {
    // Stop motors
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN, 0);
}