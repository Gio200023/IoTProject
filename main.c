#include "main.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msprf24.h"
#include "nrf_userconfig.h"
#include <math.h>

const float ADC_SLOPE = 190.9859f;

const Timer_A_PWMConfig pwmConfigA0_wheels = {
   TIMER_A_CLOCKSOURCE_SMCLK,           //clock source
   TIMER_A_CLOCKSOURCE_DIVIDER_4,       //divider
   CYCLE_PERIOD,                        //cycle period
   PWM_REGISTER,   //output compare register
   TIMER_A_OUTPUTMODE_TOGGLE_SET,       //output mode
   AVE_DUTY_CYCLE                 //duty cycle
};

const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        3000000,                                                    // SMCLK = DCO = 3MHZ
        1000000,                                                    // SPICLK = 500khz
        EUSCI_B_SPI_MSB_FIRST,                                      // MSB First
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                   // High polarity
        EUSCI_B_SPI_3PIN                                            // 3Wire SPI Mode
};

int16_t accelData[3];

uint8_t spi_transfer(uint8_t inb)
{
    UCB0TXBUF = inb;
    while ( !(UCB0IFG & UCRXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
        ;
    return UCB0RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
    uint16_t retw;
    uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

    UCB0TXBUF = inw8[1];
    while ( !(UCB0IFG & UCRXIFG) )
        ;
    retw8[1] = UCB0RXBUF;
    UCB0TXBUF = inw8[0];
    while ( !(UCB0IFG & UCRXIFG) )
        ;
    retw8[0] = UCB0RXBUF;
    return retw;
}

void spi_init() {
    GPIO_setAsPeripheralModuleFunctionOutputPin(MOSI_PORT, MOSI_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(SCK_PORT, SCK_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(MISO_PORT, MISO_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    SPI_initMaster(EUSCI_B0_BASE, &spiMasterConfig);
    SPI_enableModule(EUSCI_B0_BASE);
}

void adc_init() {
    /* Configures Pin 6.1, 4.0 and 4.2 as ADC input */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A14, A13, A11)  with repeat)
         * with internal 2.5v reference */
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
    ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM2,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
     *  is complete and enabling conversions */
    ADC14_enableInterrupt(ADC_INT2);

    /* Enabling Interrupts */
    Interrupt_enableInterrupt(INT_ADC14);

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
}

void comm_init() {
    uint8_t addr[] = { 't', 'e', 's', 't', '0'};
    rf_crc              = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
    rf_addr_width       = 5;
    rf_speed_power      = RF24_SPEED_MAX | RF24_POWER_MAX;
    rf_channel          = 119;

    msprf24_init();
    //msprf24_set_config(RF24_MASK_RX_DR | RF24_MASK_TX_DS);

    Interrupt_enableInterrupt(INT_PORT3);

    msprf24_set_pipe_packetsize(0, RF_PACKET_SIZE);
    msprf24_open_pipe(0, 1);

    w_tx_addr(addr);
    w_rx_addr(0, addr);

    return;
}

/**
 * main.c
 */
void main(void)
{
    WDT_A_holdTimer();

	adc_init();
	comm_init();

	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);

    Interrupt_enableMaster();

    while(1){
        if(rf_irq & RF24_IRQ_FLAGGED) {
            msprf24_get_irq_reason();
        }
        if(rf_irq & RF24_IRQ_TXFAILED) {
            msprf24_irq_clear(RF24_IRQ_TXFAILED);
        }
        PCM_gotoLPM3();
    }
}

void processAccelData(uint16_t *result) {
    accelData[2] = accelData[2] == 0 ? 1 : accelData[2];

    float roll =    atan2f(accelData[0], accelData[2]) + PI_QUARTER;
    float pitch =   atan2f(accelData[1], accelData[2]) + PI_QUARTER;

    roll =  roll>PI_HALF ? PI_HALF : roll;
    roll =  roll<0 ? 0 : roll;

    pitch = pitch>PI_HALF ? PI_HALF : pitch;
    pitch = pitch<0 ? 0 : pitch;

    result[0] = MIN_DUTY_CYCLE + ADC_SLOPE * roll;
    result[1] = MIN_DUTY_CYCLE + ADC_SLOPE * pitch;

    return;
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    /* ADC_MEM2 conversion completed */
    if(status & ADC_INT2)
    {
        uint16_t dataSend[2] = {0};

        accelData[0] = ADC14_getResult(ADC_MEM0) - ((ADC_ACCEL_MINUS_G + ADC_ACCEL_PLUS_G) >> 1);
        accelData[1] = ADC14_getResult(ADC_MEM1) - ((ADC_ACCEL_MINUS_G + ADC_ACCEL_PLUS_G) >> 1);
        accelData[2] = ADC14_getResult(ADC_MEM2) - ((ADC_ACCEL_MINUS_G + ADC_ACCEL_PLUS_G) >> 1);

        processAccelData(dataSend);

        w_tx_payload(RF_PACKET_SIZE, (uint8_t*) dataSend);
        msprf24_activate_tx();

        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN6);

    }
}




















