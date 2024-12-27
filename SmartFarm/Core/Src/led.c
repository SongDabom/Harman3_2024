/*
 * led.c
 *
 *  Created on: Aug 5, 2024
 *      Author: heso9
 */

#include "main.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

uint32_t ADCVal[1];  // ADC DMA buffer
uint32_t LDR_Val;
/*
volatile int lux;  // Set lux as global variable
char dum[40];	//dum for sprintf
*/
int Illuminance(uint32_t LDR_Val)
{
    // Convert analog value to voltage
    float Vout = LDR_Val * 5.0 / 4095.0; // 12-bit ADC, Ref 5V Vin
    // Convert voltage to resistance of LDR
    float RLDR = 10000.0 * Vout / (5.0 - Vout); // Rref = 10k, Vin = 5V
    // Convert resistance to illuminance (lux)
    int LightValue = 3000000 / RLDR;
    return LightValue; // Return the calculated illuminance value
}




