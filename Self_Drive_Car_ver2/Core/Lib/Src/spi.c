/*
 * spi.c
 *
 *  Created on: Nov 6, 2022
 *      Author: hussamaldean
 */


#include "spi.h"

#include "stm32f4xx.h"


void OLED_SPI_Pins_Init()
{
    // Enable clock for GPIOA, GPIOB, and GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // Set PC7 to alternate function mode (SPI2_SCK)
    GPIOC->MODER &= ~(GPIO_MODER_MODE7);  // Clear MODER bits for PC7
    GPIOC->MODER |= GPIO_MODER_MODE7_1;   // Set PC7 to alternate function mode (10)

    // Set PC3 to alternate function mode (SPI2_MOSI)
    GPIOC->MODER &= ~(GPIO_MODER_MODE3);  // Clear MODER bits for PC3
    GPIOC->MODER |= GPIO_MODER_MODE3_1;   // Set PC3 to alternate function mode (10)

    // Set PA12 as output (RST)
    GPIOA->MODER &= ~(GPIO_MODER_MODE12); // Clear MODER bits for PA12
    GPIOA->MODER |= GPIO_MODER_MODE12_0;  // Set PA12 to output mode (01)

    // Set PB8 as output (CS)
    GPIOB->MODER &= ~(GPIO_MODER_MODE8);  // Clear MODER bits for PB8
    GPIOB->MODER |= GPIO_MODER_MODE8_0;   // Set PB8 to output mode (01)

    // Set PB9 as output (DC)
    GPIOB->MODER &= ~(GPIO_MODER_MODE9);  // Clear MODER bits for PB9
    GPIOB->MODER |= GPIO_MODER_MODE9_0;   // Set PB9 to output mode (01)

    // Select AF5 (SPI2) for PC7 and PC3
    GPIOC->AFR[1] &= ~(0xF << 4);          // Clear AFR bits for PC7
    GPIOC->AFR[1] |= (0x5 << 4);           // Set PC7 to AF5 (SPI2_SCK)

    GPIOC->AFR[0] &= ~(0xF << 12);         // Clear AFR bits for PC3
    GPIOC->AFR[0] |= (0x5 << 12);          // Set PC3 to AF5 (SPI2_MOSI)
}

void OLED_SPI_Configure()
{
    // Enable clock access to SPI2 module
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // Set clock to fPCLK/2
    SPI2->CR1 &= ~(SPI_CR1_BR);
    SPI2->CR1 |= (0 << 3); // BR[2:0] = 000 (fPCLK/2)

    // Enable full duplex
    SPI2->CR1 &= ~(SPI_CR1_RXONLY);

    // Set MSB first
    SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);

    // Set mode to MASTER
    SPI2->CR1 |= (SPI_CR1_MSTR);

    // Set 8-bit data mode
    SPI2->CR1 &= ~(SPI_CR1_DFF);

    // Select software slave management by setting SSM=1 and SSI=1
    SPI2->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

    // Enable SPI module
    SPI2->CR1 |= (SPI_CR1_SPE);
}

void OLED_SPI_Write(char *data, uint32_t size)
{
    uint32_t i = 0;

    while (i < size)
    {
        // Wait until TXE is set
        while (!(SPI2->SR & SPI_SR_TXE)) {}

        // Write the data to the data register
        SPI2->DR = (uint8_t)data[i];
        i++;
    }
    // Wait until TXE is set
    while (!(SPI2->SR & SPI_SR_TXE)) {}

    // Wait for BUSY flag to reset
    while ((SPI2->SR & SPI_SR_BSY)) {}

    // Clear OVR flag
    (void)SPI2->DR;
    (void)SPI2->SR;
}

void OLED_Select(void)
{
    GPIOB->BSRR = GPIO_BSRR_BR8; // Set PB8 low (Select)
}

void OLED_Deselect(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS8; // Set PB8 high (Deselect)
}

void OLED_DataMode()
{
    GPIOB->BSRR = GPIO_BSRR_BS9;
}

void OLED_CommMode()
{
    GPIOB->BSRR = GPIO_BSRR_BR9;
}

