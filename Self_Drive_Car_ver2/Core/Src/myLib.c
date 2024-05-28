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

   while(*s) (*s != '\r') ? __io_putchar(*s++) : (__io_putchar(*s++), __io_putchar('\n'));         //enable Enter key
}

void ProgramStart(){
   printf("\033[2J");   // screen clear
   printf("\033[1;1H");   // Move cursor pos to (1,1)
   printf("Program Started ");
   Wait(1);
   printf("\033[2J");   // screen clear
}

void Wait(int o)
{
	   printf("Press Blue button to continue\r\n");
	   while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));  // (B1 == 0) if pressed

}


void usDelay(int us) // micro-second based Delay
{
	volatile uint32_t t1 = htim2.Instance->CNT;
	while((htim2.Instance->CNT -t1) < us);
}

double usDist1()
{
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, 0);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, 1);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, 0);
	//	Wait for burst
	usDelay(200);
	//	Wait for Echo Low

	//	Start count
	while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) != 1);
	volatile uint32_t t1 = htim2.Instance->CNT;
	while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) != 0);
	volatile uint32_t t2 = htim2.Instance->CNT;

	int dt = t2 - t1;	//	micro-second sonic speed
	usDelay(60000);	//	minimum delay for next time

	double Dist = 0.0001715 * dt;

	return Dist;
}

double usDist2()
{
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, 0);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, 1);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, 0);
	//	Wait for burst
	usDelay(200);
	//	Wait for Echo Low

	//	Start count
	while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) != 1);
	volatile uint32_t t1 = htim2.Instance->CNT;
	while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) != 0);
	volatile uint32_t t2 = htim2.Instance->CNT;

	int dt = t2 - t1;	//	micro-second sonic speed
	usDelay(60000);	//	minimum delay for next time

	double Dist = 0.0001715 * dt;

	return Dist;
}

double usDist3()
{

	HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, 0);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, 1);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, 0);
	//	Wait for burst
	usDelay(200);
	//	Wait for Echo Low

	//	Start count
	while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) != 1);
	volatile uint32_t t1 = htim2.Instance->CNT;
	while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) != 0);
	volatile uint32_t t2 = htim2.Instance->CNT;

	int dt = t2 - t1;	//	micro-second sonic speed
	usDelay(60000);	//	minimum delay for next time
	double Dist = 0.0001715 * dt;

	return Dist;
}

double usDist4()
{

	HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, 0);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, 1);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, 0);
	//	Wait for burst
	usDelay(200);
	//	Wait for Echo Low

	//	Start count
	while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) != 1);
	volatile uint32_t t1 = htim2.Instance->CNT;
	while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) != 0);
	volatile uint32_t t2 = htim2.Instance->CNT;

	int dt = t2 - t1;	//	micro-second sonic speed
	usDelay(60000);	//	minimum delay for next time
	double Dist = 0.0001715 * dt;

	return Dist;
}

double usDist5()
{

	HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, 0);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, 1);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, 0);
	//	Wait for burst
	usDelay(200);
	//	Wait for Echo Low

	//	Start count
	while(HAL_GPIO_ReadPin(ECHO5_GPIO_Port, ECHO5_Pin) != 1);
	volatile uint32_t t1 = htim2.Instance->CNT;
	while(HAL_GPIO_ReadPin(ECHO5_GPIO_Port, ECHO5_Pin) != 0);
	volatile uint32_t t2 = htim2.Instance->CNT;

	int dt = t2 - t1;	//	micro-second sonic speed
	usDelay(60000);	//	minimum delay for next time
	double Dist = 0.0001715 * dt;

	return Dist;
}


