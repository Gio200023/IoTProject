#include "main.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "msprf24.h"
#include "nrf_userconfig.h"
#include <math.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <ti/grlib/grlib.h>
#include <string.h>

typedef enum { JOYSTICK, ACCELEROMETER } ADC_INPUT;

ADC_INPUT adc_source = JOYSTICK;

/* Graphic library context */
Graphics_Context g_sContext;
const float ADC_ACCEL_SLOPE = 954.9f;
const float ADC_JOYSTICK_SLOPE = 3.9f;


//NB this is the same configuration as the LCD, because they use the same EUSCI (B0)
const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        48000000,                                                   // SMCLK = DCO = 48MHZ
        8000000,                                                   // SPICLK = 8MHZ
        EUSCI_B_SPI_MSB_FIRST,                                      // MSB First
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                   // High polarity
        EUSCI_B_SPI_3PIN                                            // 3Wire SPI Mode
};

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

uint8_t adc_configure_joystick() {
    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
         * with internal 2.5v reference */
    while(!ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true));// return 1;
    ADC14_setResolution(ADC_8BIT);
    ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_enableInterrupt(ADC_INT1);
    ADC14_disableInterrupt(ADC_INT2);
    Interrupt_enableInterrupt(INT_ADC14);

    return 0;
}

uint8_t adc_configure_accel() {
    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A14, A13, A11)  with repeat)
         * with internal 2.5v reference */
    while(!ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true));// return 1;
    ADC14_setResolution(ADC_14BIT);
    ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureConversionMemory(ADC_MEM2,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_enableInterrupt(ADC_INT2);
    ADC14_disableInterrupt(ADC_INT1);
    Interrupt_enableInterrupt(INT_ADC14);

    return 0;
}

void adc_init() {
    /* Configures Pin 6.1, 4.0 and 4.2 as ADC input */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    switch(adc_source) {
    case JOYSTICK: adc_configure_joystick(); break;
    case ACCELEROMETER: adc_configure_accel(); break;
    }

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
    rf_channel          = RF_CHANNEL;

    msprf24_init();

    Interrupt_enableInterrupt(INT_PORT3);

    msprf24_set_pipe_packetsize(RF_PIPE, RF_PACKET_SIZE);
    msprf24_open_pipe(RF_PIPE, 1);    // set packet size and open pipe with autoack (6 available, we just use one)

    w_tx_addr(addr);        // set transmission address
    w_rx_addr(0, addr);     // set receive address (used for ACK)

    return;
}


void graphicsInit()
{
    char s[14] = {0};
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
        Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
        Graphics_clearDisplay(&g_sContext);

        switch(adc_source) {
        case JOYSTICK: strcpy(s, "Joystick"); break;
        case ACCELEROMETER: strcpy(s, "Accelerometer"); break;
        }
        Graphics_drawStringCentered(&g_sContext,
                                        (int8_t *)s,
                                        AUTO_STRING_LENGTH,
                                        64,
                                        30,
                                        OPAQUE_TEXT);

}

void hw_init() {
    WDT_A_holdTimer();

    /* Set the core voltage level to VCORE1 */
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    /* Set 2 flash wait states for Flash bank 0 and 1*/
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    /* Initializes Clock System */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

}

void GpioInit() {
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);

    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN1);

    Interrupt_enableInterrupt(INT_PORT5);
}

/**
 * main.c
 */
void main(void)
{
    hw_init();
	adc_init();
	comm_init();
	GpioInit();
    graphicsInit();

    Interrupt_enableMaster();

    while(1){
        if(rf_irq & RF24_IRQ_FLAGGED) {
            msprf24_get_irq_reason();
        }
        if(rf_irq & RF24_IRQ_TXFAILED) {
            msprf24_irq_clear(rf_irq);
        } else if (rf_irq & RF24_IRQ_FLAGGED) {
            msprf24_irq_clear(rf_irq);
        }
        PCM_gotoLPM3();
    }
}

void processAccelData(int16_t accelData[3], uint16_t *result) {
    accelData[0] -= (ADC_ACCEL_MAX_VALUE / 2);
    accelData[1] -= (ADC_ACCEL_MAX_VALUE / 2);
    accelData[2] -= (ADC_ACCEL_MAX_VALUE / 2);

    accelData[2] = accelData[2] == 0 ? 1 : accelData[2];

    float roll =    atan2f(accelData[0], accelData[2]);
    float pitch =   atan2f(accelData[1], accelData[2]);

    roll =  roll>PI_SIXTH ? PI_SIXTH : roll;
    roll =  roll<-PI_SIXTH ? -PI_SIXTH : roll;

    pitch = pitch>PI_SIXTH ? PI_SIXTH : pitch;
    pitch = pitch<-PI_SIXTH ? -PI_SIXTH : pitch;

    result[0] = MIN_DUTY_CYCLE + ADC_ACCEL_SLOPE * (roll + PI_SIXTH);
    result[1] = MIN_DUTY_CYCLE + ADC_ACCEL_SLOPE * (pitch + PI_SIXTH);

    return;
}

void processJoystickData(int16_t joyData[2], uint16_t *result) {
    result[0] = MIN_DUTY_CYCLE + ADC_JOYSTICK_SLOPE * joyData[0];
    result[1] = MIN_DUTY_CYCLE + ADC_JOYSTICK_SLOPE * joyData[1];

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
    if(status & ADC_INT2) {
        uint16_t dataSend[2] = {0};
        int16_t adcData[3] = {0};

        adcData[0] = ADC14_getResult(ADC_MEM0);
        adcData[1] = ADC14_getResult(ADC_MEM1);
        adcData[2] = ADC14_getResult(ADC_MEM2);
        processAccelData(adcData, dataSend);

        w_tx_payload(RF_PACKET_SIZE, (uint8_t*) dataSend);
        msprf24_activate_tx();

        if(msprf24_is_alive()) GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
        else GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
    } else if(status & ADC_INT1) {
        uint16_t dataSend[2] = {0};
        int16_t adcData[2] = {0};

        adcData[0] = ADC14_getResult(ADC_MEM0);
        adcData[1] = ADC14_getResult(ADC_MEM1);
        processJoystickData(adcData, dataSend);

        w_tx_payload(RF_PACKET_SIZE, (uint8_t*) dataSend);
        msprf24_activate_tx();

        if(msprf24_is_alive()) GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
        else GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
    }
}

void PORT5_IRQHandler(void)
{
    uint16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
    if(status & GPIO_PIN1) {
        char s[15] = {0};

        ADC14_disableConversion();
        switch(adc_source) {
        case ACCELEROMETER:
            strcpy(s, "Joystick");
            adc_configure_joystick();
            adc_source = JOYSTICK;
            break;
        case JOYSTICK:
            strcpy(s, "Accelerometer");
            adc_configure_accel();
            adc_source = ACCELEROMETER;
            break;
        }
        ADC14_enableConversion();
        ADC14_toggleConversionTrigger();
        Graphics_clearDisplay(&g_sContext);
        Graphics_drawStringCentered(&g_sContext,
                                        (int8_t *)s,
                                        AUTO_STRING_LENGTH,
                                        64,
                                        30,
                                        OPAQUE_TEXT);

    }
}




















