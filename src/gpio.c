/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>


#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5

CORE_DECLARE_IRQ_STATE;

void gpioInit()
{
	/* LED configuration */
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	/* PB0 passkey confirmation button configuration */
	GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInputPull, true);

	/* Infrared Sensor gpio configuration */
	GPIO_PinModeSet(IR_1_PORT, IR_1_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(IR_2_PORT, IR_2_PIN, gpioModeInputPull, true);

	/* Vibration Sensor gpio configuration */
	GPIO_PinModeSet(VIBRATION_PORT, VIBRATION_PIN, gpioModeInputPull, true);

	/* Buzzer gpio configuration */
	GPIO_PinModeSet(BUZZER_PORT, BUZZER_PIN, gpioModePushPull, false);
}

void gpioIntEnable()
{
	/* Configuring PB0 and PB1 for rising and falling edge and enabling its interrupt */
	GPIO_IntConfig(PB0_PORT, PB0_PIN, true, true, true);
	GPIO_IntConfig(PB1_PORT, PB1_PIN, true, true, true);

	/* Infrared Sensor configured for falling edge and enabling its interrupt */
	GPIO_IntConfig(IR_1_PORT, IR_1_PIN, false, true, true);
	GPIO_IntConfig(IR_2_PORT, IR_2_PIN, false, true, true);

	/* Vibration Sensor configured for falling edge and enabling its interrupt */
	GPIO_IntConfig(VIBRATION_PORT, VIBRATION_PIN, false, true, true);

	/* Enabling GPIO in NVIC */
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

void GPIO_EVEN_IRQHandler(void)
{
	CORE_ENTER_CRITICAL();
	uint32_t reason = GPIO_IntGet();

	/* Clearing all interrupts */
	GPIO_IntClear(reason);

	// if interrupt came from PB0, send gecko external signal 0x10 for button press
	// pin 6, so 7th bit set (0x40)
	if(reason & 0x40)
		gecko_external_signal(PB0_FLAG);

	// IR1 pin 10, so 11th bit set
	if(reason & 0x400)
		gecko_external_signal(IR1_FLAG);

	// VIBRATION pin 12, so 13th bit set
	if(reason & 0x1000)
		gecko_external_signal(VIB_FLAG);

	CORE_EXIT_CRITICAL();
}

void GPIO_ODD_IRQHandler(void)
{
	CORE_ENTER_CRITICAL();
	uint32_t reason = GPIO_IntGet();

	/* Clearing all interrupts */
	GPIO_IntClear(reason);

	// if interrupt came from PB1, send gecko external signal 0x11 for button press
	// pin 7, so 8th bit set (0x80)
	if(reason & 0x80)
		gecko_external_signal(PB1_FLAG);

	// IR2 pin 11, so 12th bit set
	if(reason & 0x800)
		gecko_external_signal(IR2_FLAG);

	CORE_EXIT_CRITICAL();
}

//void buzzerSetToggle()
//{
//	gecko_cmd_hardware_set_soft_timer(1 * 32768, DISPLAY_UPDATE, 0);
//}

void buzzerSetOn()
{
	GPIO_PinOutSet(BUZZER_PORT, BUZZER_PIN);
}

void buzzerSetOff()
{
	GPIO_PinOutClear(BUZZER_PORT, BUZZER_PIN);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpioEnableDisplay()
{
	/* Enabling temperature sensor and display pin */
	GPIO_PinOutSet(LCD_PORT_DISP_SEL, LCD_PIN_DISP_SEL);
}

void gpioSetDisplayExtcomin(bool high)
{
	/* Setting EXTCOMIN pin based on boolean value - to toggle */
	if (high == true)
		GPIO_PinOutSet(LCD_PORT_EXTCOMIN, LCD_PIN_EXTCOMIN);

	else
		GPIO_PinOutClear(LCD_PORT_EXTCOMIN, LCD_PIN_EXTCOMIN);
}
