/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>

/* LCD header file */
#include "displayls013b7dh03config.h"

#include "main.h"

/* Header files required for display */
#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

/* Push Button PB0 connected to pin PF6 */
#define PB0_PORT gpioPortF
#define PB0_PIN 6

/* Push Button PB1 connected to pin PF7 */
#define PB1_PORT gpioPortF
#define PB1_PIN 7

/* Infrared Sensor port and pin */
#define IR_1_PORT gpioPortD
#define IR_1_PIN 10
#define IR_2_PORT gpioPortD
#define IR_2_PIN 11

/* Vibration Sensor port and pin */
#define VIBRATION_PORT gpioPortD
#define VIBRATION_PIN 12

/* Buzzer port and pin */
#define BUZZER_PORT gpioPortA
#define BUZZER_PIN 3

void gpioInit();
void gpioIntEnable();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

/* Functions required for display */
void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);

#endif /* SRC_GPIO_H_ */
