#pragma PERSISTENT(boot_flag)
unsigned char boot_flag = 0;  // Boot flag that persists across resets

#include <msp430.h> // MSP430FR2355
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Prototip de functie pentru ajustarea software a frecventei DCO
void Software_Trim();

// Definirea constantelor pentru frecventa si pini
#define MCLK_FREQ_MHZ 2
#define SMCLK_FREQ 2000000  // 2 MHz
#define LED     BIT0        // LED on P1.0 to indicate status
#define BTN1    BIT3        // BTN1 (not used now)
#define BTN2    BIT1        // BTN2 (not used now)

// Variables for signal generation
volatile unsigned int signal_type = 0;  // 0 for sine, 1 for triangle, 2 for chainsaw
volatile unsigned long frequency = 1000; // Default frequency (1 kHz)

// Initializarea ceasului si a sistemului de alimentare
void clock_init(void) {
    PM5CTL0 &= ~LOCKLPM5;  // Deblocheaza pini I/O
    WDTCTL = WDTPW | WDTHOLD;   // Opreste watchdog timer
    FRCTL0 = FRCTLPW | NWAITS_2;

    __bis_SR_register(SCG0);  // Dezactiveaza temporar MCLK
    CSCTL3 = SELREF__REFOCLK; // Seteaza sursa de referinta la REFOCLK
    // Configurare DCO cu ajustare automata
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_1;
    CSCTL2 = FLLD_0 + 60;
    __delay_cycles(3);
    __bic_SR_register(SCG0); // Reactiveaza MCLK
    Software_Trim();          // Ajustare software a DCO
    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
}

// UART initialization function
void uart_init(void) {
    P4SEL0 |= BIT2 | BIT3;   // Set P4.2 and P4.3 for UART TX/RX
    P4SEL1 &= ~(BIT2 | BIT3);
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL_2;   // Use SMCLK as clock source

    UCA1BR0 = 0x03;
    UCA1BR1 = 0x00;
    UCA1MCTLW = 0x0241;

    UCA1CTLW0 &= ~UCSWRST;   // Start UART
    UCA1IE |= UCRXIE;        // Enable RX interrupt
}

// UART send function to transmit strings
void uart_send(char *str) {
    while (*str) {
        while (!(UCA1IFG & UCTXIFG));  // Wait for TX buffer to be ready
        UCA1TXBUF = *str++;
    }
}

// Function to set frequency based on user input
void set_frequency(void) {
    // Here you can implement logic to set frequency through UART commands
    // For now, just an example:
    uart_send("Set frequency (Hz): ");
    // This could be done by reading the user input and parsing it into `frequency`
}

// Function to generate sine wave
void generate_sine_wave(void) {
    // Implement sine wave generation logic here
    // Using DAC to output the sine wave values
}

// Function to generate triangle wave
void generate_triangle_wave(void) {
    // Implement triangle wave generation logic here
    // Using DAC to output the triangle wave values
}

// Function to generate chainsaw wave
void generate_chainsaw_wave(void) {
    // Implement chainsaw wave generation logic here
    // Using DAC to output the chainsaw wave values
}

// Timer setup for continuous signal generation
void setup_timer(void) {
    TB0CTL = TBSSEL__SMCLK | MC__CONTINUOUS | TBCLR | TBIE;  // Continuous mode, SMCLK as source
}

// Timer interrupt service routine for signal generation
#pragma vector=TIMER0_B1_VECTOR
__interrupt void Timer_B_ISR(void) {
    if (TB0IV == TB0IV_TBIFG) {
        // Generate the appropriate signal based on the signal type
        switch (signal_type) {
            case 0:
                generate_sine_wave();
                break;
            case 1:
                generate_triangle_wave();
                break;
            case 2:
                generate_chainsaw_wave();
                break;
            default:
                break;
        }
    }
}

// Main function for signal generation
int main(void) {
    WDTCTL = WDTPW | WDTHOLD; // Stop the watchdog timer
    __delay_cycles(10000);  // Short delay for stability
    uart_init();             // Initialize UART
    setup_timer();           // Setup Timer B for continuous operation

    // Signal generation loop
    while (1) {
        set_frequency();  // Get frequency from UART (you can implement frequency change logic here)

        // Based on user input, set the signal type (for simplicity, we keep this fixed for now)
        // For example, UART could be used to change the signal type
        // Here we use signal_type = 0 (sine), 1 (triangle), or 2 (chainsaw)
        uart_send("Generating signal...\n\r");

        __bis_SR_register(GIE);  // Enable global interrupts
    }
}

// Timer interrupt for waveform generation (example for sine, triangle, or chainsaw waves)
void generate_sine_wave(void) {
    // Example: Generate a sine wave by outputting DAC values
    // Implement sine wave values by creating a sine lookup table or using math (sin() function)
    unsigned int sine_value = (unsigned int)(2047 + 2047 * sin(2 * 3.14159 * TB0R / (SMCLK_FREQ / frequency)));  // Example sine calculation
    // Output to DAC (using the appropriate register and method for your setup)
    DAC12_0DAT = sine_value;
}

void generate_triangle_wave(void) {
    // Example: Generate a triangle wave
    static int triangle_value = 0;
    static int direction = 1;
    if (direction) {
        triangle_value += 50;  // Increase value (upward slope)
        if (triangle_value >= 4095) direction = 0;  // Switch direction at max value
    } else {
        triangle_value -= 50;  // Decrease value (downward slope)
        if (triangle_value <= 0) direction = 1;  // Switch direction at min value
    }
    DAC12_0DAT = triangle_value;  // Output to DAC
}

void generate_chainsaw_wave(void) {
    static unsigned int chainsaw_value = 0;
    chainsaw_value += 10;
    if (chainsaw_value >= 4095) chainsaw_value = 0;  // Reset after max value
    DAC12_0DAT = chainsaw_value;  // Output to DAC
}
