/*
 * myLib.c
 *
 *  Created on: Apr 24, 2024
 *      Author: SAMSUNG
 */


#include "main.h"
//extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
int __io_putchar(int ch)
{
   HAL_UART_Transmit(&huart2, &ch, 1, 10);
   return ch;
}
void Outs(char *s)
{
   /*while(1)
   {
      if(*s != 0) { __io_putchar(*s); s++;}
      else break;
   } */
   //while(*s) __io_putchar(*s++);
   while(*s) (*s != '\r') ? __io_putchar(*s++) : (__io_putchar(*s++), __io_putchar('\n'));         //enable Enter key
}

int GetAdcValue()
{
   /* HAL_ADC_Start (&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    return HAL_ADC_GetValue(&hadc1);
    */
}

void usDelay(int us) // micro-second based Delay
{
	volatile uint32_t t1 = htim2.Instance->CNT;
	while((htim2.Instance->CNT -t1) < us);
}

double Dist()
{

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, 0);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, 1);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, 0);
	//	Wait for burst
	usDelay(200);
	//	Wait for Echo Low

	//	Start count
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) != 1);
	volatile uint32_t t1 = htim2.Instance->CNT;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) != 0);
	volatile uint32_t t2 = htim2.Instance->CNT;

	int dt = t2 - t1;	//	micro-second sonic speed

	usDelay(60000);	//	minimum delay for next time

	double Dist = 0.0001715 * dt;

	return Dist;
}

void ProgramStart()
{
    printf("\033[2J");         //screen clear
     printf("\033[1;1H");         // move cursor pos to [1:1]
     printf("Program started ");
     Wait(1);
     printf("\033[2J");         //screen clear
}

void Wait(int o)
{
   printf("Press Blue button to continue\r\n");
   while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));      //B1 == 0
}
