/*
 * main.h
 *
 *  Created on: 29 nov 2021
 *      Author: Federico
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "msp.h"
#include "msprf24.h"
#include <inttypes.h>

#define CYCLE_PERIOD            15000U
#define MIN_DUTY_CYCLE    975U
#define MAX_DUTY_CYCLE    1275U
#define AVE_DUTY_CYCLE    ((MIN_DUTY_CYCLE + MAX_DUTY_CYCLE) / 2)

#define PWM_REGISTER_WHEELS     TIMER_A_CAPTURECOMPARE_REGISTER_2
#define PWM_REGISTER_POWER      TIMER_A_CAPTURECOMPARE_REGISTER_3

#define setDutyCycle(timer, out_register, dutyCycle) \
        Timer_A_setCompareValue(timer, out_register, dutyCycle)

#define MISO_PORT   GPIO_PORT_P1
#define MOSI_PORT   GPIO_PORT_P1
#define SCK_PORT    GPIO_PORT_P1
#define CSN_PORT    GPIO_PORT_P6
#define CE_PORT     GPIO_PORT_P6
#define IRQ_PORT    GPIO_PORT_P3

#define MISO_PIN    GPIO_PIN7
#define MOSI_PIN    GPIO_PIN6
#define SCK_PIN     GPIO_PIN5
#define CSN_PIN     GPIO_PIN7
#define CE_PIN      GPIO_PIN6
#define IRQ_PIN     GPIO_PIN0

#define SPI_COMM        EUSCI_B0_BASE

#define RF_PACKET_SIZE  4U

void spi_init();
uint8_t spi_transfer(uint8_t inb);
uint16_t spi_transfer16(uint16_t inw);

#endif /* MAIN_H_ */