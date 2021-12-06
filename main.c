#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "main.h"
#include "msprf24.h"
#include "nrf_userconfig.h"

const Timer_A_PWMConfig pwmConfigA0_wheels = {
   TIMER_A_CLOCKSOURCE_SMCLK,           //clock source
   TIMER_A_CLOCKSOURCE_DIVIDER_4,       //divider
   CYCLE_PERIOD,                        //cycle period
   PWM_REGISTER_WHEELS,   //output compare register
   TIMER_A_OUTPUTMODE_TOGGLE_SET,       //output mode
   AVE_DUTY_CYCLE                 //duty cycle
};

const Timer_A_PWMConfig pwmConfigA0_power = {
   TIMER_A_CLOCKSOURCE_SMCLK,           //clock source
   TIMER_A_CLOCKSOURCE_DIVIDER_4,       //divider
   CYCLE_PERIOD,                        //cycle period
   PWM_REGISTER_POWER,   //output compare register
   TIMER_A_OUTPUTMODE_TOGGLE_SET,       //output mode
   AVE_DUTY_CYCLE                 //duty cycle
};

const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,                              // SMCLK Clock Source
        3000000,                                                    // SMCLK = DCO = 3MHZ
        1000000,                                                     // SPICLK = 500khz
        EUSCI_B_SPI_MSB_FIRST,                                      // MSB First
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                   // High polarity
        EUSCI_B_SPI_3PIN                                            // 3Wire SPI Mode
};

const Timer_A_UpModeConfig timerConfigA1 = {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_4,
        5*CYCLE_PERIOD,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
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

void timer_init() {
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA0_wheels);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA0_power);

    Timer_A_configureUpMode(TIMER_A1_BASE, &timerConfigA1);
    //Timer_A_enableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

void comm_init() {
    uint8_t addr[] = { 't', 'e', 's', 't', '0'};
    rf_crc              = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
    rf_addr_width       = 5;
    rf_speed_power      = RF24_SPEED_MAX | RF24_POWER_MAX;
    rf_channel          = 119;

    msprf24_init();

    Interrupt_enableInterrupt(INT_PORT3);

    msprf24_set_pipe_packetsize(0, RF_PACKET_SIZE);
    msprf24_open_pipe(0, 1); // pipe 0, autoACK (1)

    w_rx_addr(0, addr);

    msprf24_activate_rx();

    return;
}

uint8_t check=0;
uint16_t wheels_duty= AVE_DUTY_CYCLE, power_duty=AVE_DUTY_CYCLE;

void main(void)
{
    uint8_t buf[RF_PACKET_SIZE];
    uint8_t data[RF_PACKET_SIZE]={85};

    WDT_A_holdTimer();

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    timer_init();
    comm_init();
    Interrupt_enableMaster();

    while(!msprf24_is_alive());

    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

    while (1) {
        if (rf_irq & RF24_IRQ_FLAGGED) {
            msprf24_get_irq_reason(); //motivo interrupt
        }
        if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending()) {
            r_rx_payload(RF_PACKET_SIZE, buf);
            msprf24_irq_clear(RF24_IRQ_RX);
            TIMER_A1->R = 0; //set the timer counter to 0

            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

            uint16_t w_d, p_d;

            w_d = *((uint16_t*)buf);
            p_d = *((uint16_t*)buf+1);

            w_d = w_d>MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : w_d;
            w_d = w_d<MIN_DUTY_CYCLE ? MIN_DUTY_CYCLE : w_d;

            p_d = p_d>MAX_DUTY_CYCLE ? MAX_DUTY_CYCLE : p_d;
            p_d = p_d<MIN_DUTY_CYCLE ? MIN_DUTY_CYCLE : p_d;

            if( w_d > wheels_duty + 10 || w_d < wheels_duty - 10){

                wheels_duty = w_d;
                setDutyCycle(TIMER_A0_BASE, PWM_REGISTER_WHEELS, wheels_duty);

            }

            if( p_d > power_duty + 10 || p_d < power_duty - 10 ){

                power_duty = p_d;
                setDutyCycle(TIMER_A0_BASE, PWM_REGISTER_POWER, power_duty);

            }
        }
        PCM_gotoLPM3();
    }
}

void TA1_0_IRQHandler(void){

    uint32_t status;

    status = Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    setDutyCycle(TIMER_A0_BASE,PWM_REGISTER_WHEELS, AVE_DUTY_CYCLE);
    setDutyCycle(TIMER_A0_BASE,PWM_REGISTER_POWER, MIN_DUTY_CYCLE);

}
