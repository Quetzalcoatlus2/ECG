#include <msp430.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Clock and pin definitions
#define MCLK_FREQ_MHZ 8          // Change from 16 to 8
#define SMCLK_FREQ 8000000       // Change from 16000000 to 8000000
#define LED BIT0            // LED on P1.0 for status indication

// Waveform types
#define WAVE_SINE       0
#define WAVE_TRIANGLE   1
#define WAVE_SAWTOOTH   2
#define WAVE_SQUARE     3

// DAC configuration
#define DAC_PIN BIT0       // P1.0 for DAC output
#define DAC_PORT_DIR P1DIR
#define DAC_PORT_SEL0 P1SEL0
#define DAC_PORT_SEL1 P1SEL1

// Signal parameters
volatile uint8_t signal_type = WAVE_SINE;  // Default: sine wave
volatile uint32_t frequency = 1000;        // Default: 1 kHz
volatile uint16_t amplitude = 4095;        // Full scale (12-bit DAC)
volatile uint16_t offset = 2048;           // Midpoint

// Sine wave lookup table (256 points for one cycle)
#define SINE_TABLE_SIZE 256
uint16_t sine_table[SINE_TABLE_SIZE];

// UART buffer definitions
#define UART_BUF_SIZE 64
char uart_rx_buf[UART_BUF_SIZE];
volatile uint8_t uart_rx_index = 0;

// Forward declarations
void Software_Trim(void);
void clock_init(void);
void uart_init(void);
void dac_init(void);
void timer_init(void);
void generate_sine_table(void);
void uart_send(const char *str);
void set_frequency(uint32_t freq);
void process_command(void);

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    clock_init();               // Initialize clock system
    uart_init();                // Initialize UART
    generate_sine_table();      // Generate lookup table for sine wave
    dac_init();                 // Initialize DAC
    timer_init();               // Initialize timer for waveform generation

    // Send initial welcome message
    uart_send("MSP430 Signal Generator Ready\r\n");

    // Replace your current main loop with this:
    while (1) {
        // Check for incoming UART data
        if (UCA1IFG & UCRXIFG) {
            char c = UCA1RXBUF;  // Get received character
            
            // Echo character back
            while (!(UCA1IFG & UCTXIFG));
            UCA1TXBUF = c;
            
            if (c == '\r' || c == '\n') {
                // End of command, process it
                uart_rx_buf[uart_rx_index] = '\0';
                process_command();
                uart_rx_index = 0;
            }
            else if (uart_rx_index < UART_BUF_SIZE - 1) {
                // Store character in buffer
                uart_rx_buf[uart_rx_index++] = c;
            }
        }
    }
}

// Initialize clock system
void clock_init(void) {
    // Disable the GPIO power-on default high-impedance mode
    PM5CTL0 &= ~LOCKLPM5;
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    FRCTL0 = FRCTLPW | NWAITS_1; // Set FRAM wait states (reduced for 8MHz)

    __bis_SR_register(SCG0);               // Disable FLL
    CSCTL3 = SELREF__REFOCLK;              // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3; // Range = 8MHz
    CSCTL2 = FLLD_0 + 243;                 // DCODIV = 8MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);               // Enable FLL
    Software_Trim();                       // Software Trim to get the best DCOFTRIM value
    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // ACLK = 32768Hz
}

// TX-only UART initialization
void uart_init(void) {
    // Configure LED for visual feedback
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // Basic clock setup - simple and reliable
    PM5CTL0 &= ~LOCKLPM5;       // Enable GPIO

    // Configure UART pins (P4.2 = RX, P4.3 = TX)
    P4SEL0 |= BIT2 | BIT3;
    P4SEL1 &= ~(BIT2 | BIT3);

    // Configure UART for 115200 baud @ 8MHz
    UCA1CTLW0 = UCSWRST;                    // Put UART in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;             // SMCLK as clock source
    UCA1BR0 = 4;                            // 8MHz/115200/16 = 4.34
    UCA1BR1 = 0;
    UCA1MCTLW = 0x5551 | UCOS16;            // Different modulation pattern
    UCA1CTLW0 &= ~UCSWRST;                  // Initialize UART
}

// Initialize DAC (12-bit)
void dac_init(void) {
    // Configure DAC pins
    P1SEL0 |= BIT0;                         // Select DAC function for P1.0
    P1SEL1 |= BIT0;

    // Configure DAC module
    SAC0DAC = DACSREF_1 | DACLSEL_0;        // Use Vcc as ref, load from DAC0DAT
    SAC0DAT = 0;                            // Initialize DAC value to 0
    SAC0OA = NMUXEN | PMUXEN | OAEN;        // Enable OA with inputs
    SAC0PGA = MSEL_1;                       // Buffer mode (gain = 1)
    SAC0OA |= SACEN;                        // Enable SAC
}

// Initialize timer for waveform generation
void timer_init(void) {
    // Configure Timer B0 for waveform generation
    TB0CTL = TBCLR;                         // Clear timer
    TB0CTL |= TBSSEL__SMCLK;                // SMCLK as clock source
    TB0CTL |= MC__UP;                       // Up mode

    // Calculate the timer period for the desired frequency
    set_frequency(frequency);

    // Enable timer interrupt
    TB0CCTL0 = CCIE;                        // TBCCR0 interrupt enabled
}

// Generate sine lookup table
void generate_sine_table(void) {
    unsigned int i; // Changed from int to unsigned int
    for (i = 0; i < SINE_TABLE_SIZE; i++) {
        // Convert sine (-1 to 1) to DAC value (0 to 4095)
        double angle = 2.0 * M_PI * i / SINE_TABLE_SIZE;
        sine_table[i] = (uint16_t)(2047.5 + 2047.5 * sin(angle));
    }
}

// Simple polling-based UART send function
void uart_send(const char *str) {
    while (*str) {
        while (!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = *str++;
    }
}

// Set frequency of generated waveform
void set_frequency(uint32_t freq) {
    if (freq < 1) freq = 1;
    if (freq > SMCLK_FREQ/256) freq = SMCLK_FREQ/256;  // Limit max frequency

    frequency = freq;

    // Calculate period for timer
    uint16_t period = SMCLK_FREQ / (frequency * 256);  // 256 points per waveform
    TB0CCR0 = period - 1;

    // Convert frequency value to string first, then combine
    char freq_str[32];
    char freq_val[16];
    
    // Use simple itoa approach rather than sprintf for the value
    uint32_t temp = frequency;
    uint8_t i = 0;
    
    // Handle special case of zero
    if (temp == 0) {
        freq_val[i++] = '0';
    } else {
        // Convert number to string, building it in reverse
        uint8_t digits[10];  // Max 10 digits for uint32_t
        uint8_t digit_count = 0;
        
        while (temp > 0) {
            digits[digit_count++] = temp % 10;
            temp /= 10;
        }
        
        // Place digits in correct order
        while (digit_count > 0) {
            freq_val[i++] = '0' + digits[--digit_count];
        }
    }
    
    // Null-terminate the value string
    freq_val[i] = '\0';
    
    // Combine into final message
    strcpy(freq_str, "Frequency set to: ");
    strcat(freq_str, freq_val);
    strcat(freq_str, " Hz\r\n");
    
    uart_send(freq_str);
}

// Add this function to process commands
void process_command(void) {
    char cmd[16] = {0};
    uint32_t freq = 0;
    
    // Parse command string
    sscanf(uart_rx_buf, "%15s %lu", cmd, &freq);
    
    if (strcmp(cmd, "SINE") == 0) {
        signal_type = WAVE_SINE;
        uart_send("Setting waveform: Sine\r\n");
        if (freq > 0) {
            set_frequency(freq);
        }
    }
    else if (strcmp(cmd, "TRIANGLE") == 0) {
        signal_type = WAVE_TRIANGLE;
        uart_send("Setting waveform: Triangle\r\n");
        if (freq > 0) {
            set_frequency(freq);
        }
    }
    else if (strcmp(cmd, "SAW") == 0) {
        signal_type = WAVE_SAWTOOTH;
        uart_send("Setting waveform: Sawtooth\r\n");
        if (freq > 0) {
            set_frequency(freq);
        }
    }
    else if (strcmp(cmd, "SQUARE") == 0) {
        signal_type = WAVE_SQUARE;
        uart_send("Setting waveform: Square\r\n");
        if (freq > 0) {
            set_frequency(freq);
        }
    }
    else if (strcmp(cmd, "HELP") == 0) {
        uart_send("Available commands:\r\n");
        uart_send("  SINE <freq>     - Generate sine wave\r\n");
        uart_send("  TRIANGLE <freq> - Generate triangle wave\r\n");
        uart_send("  SAW <freq>      - Generate sawtooth wave\r\n");
        uart_send("  SQUARE <freq>   - Generate square wave\r\n");
        uart_send("  HELP            - Show this help\r\n");
    }
    else {
        uart_send("Unknown command. Type HELP for available commands.\r\n");
    }
}

// Timer B0 interrupt service routine for waveform generation
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) {
    static uint8_t phase = 0;
    static uint16_t triangle_val = 0;
    static uint8_t triangle_dir = 1;
    uint16_t dac_value = 0;

    // Generate waveform based on current type
    switch (signal_type) {
        case WAVE_SINE:
            dac_value = sine_table[phase];
            break;

        case WAVE_TRIANGLE:
            dac_value = triangle_val;
            if (triangle_dir) {
                triangle_val += 16;  // Increment for rising edge
                if (triangle_val >= amplitude) {
                    triangle_val = amplitude;
                    triangle_dir = 0;
                }
            } else {
                triangle_val -= 16;  // Decrement for falling edge
                if (triangle_val <= 0 || triangle_val > amplitude) {
                    triangle_val = 0;
                    triangle_dir = 1;
                }
            }
            break;

        case WAVE_SAWTOOTH:
            dac_value = (phase * amplitude) / 255;
            break;

        case WAVE_SQUARE:
            dac_value = (phase < 128) ? amplitude : 0;
            break;

        default:
            dac_value = offset;
            break;
    }

    // Output to DAC
    SAC0DAT = dac_value;

    // Update phase for next cycle
    phase = (phase + 1) & 0xFF;  // Wrap around at 256
}

// Functia Software_Trim pentru stabilizarea frecventei DCO
void Software_Trim(void) {
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do {
        CSCTL0 = 0x100;
        do {
            CSCTL7 &= ~DCOFFG;
        } while (CSCTL7 & DCOFFG);

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);
        while ((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;
        csCtl1Read = CSCTL1;

        oldDcoTap = newDcoTap;
        newDcoTap = csCtl0Read & 0x01ff;
        dcoFreqTrim = (csCtl1Read & 0x0070) >> 4;

        if(newDcoTap < 256) {
            newDcoDelta = 256 - newDcoTap;
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256))
                endLoop = 1;
            else {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim << 4);
            }
        } else {
            newDcoDelta = newDcoTap - 256;
            if(oldDcoTap < 256)
                endLoop = 1;
            else {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim << 4);
            }
        }

        if(newDcoDelta < bestDcoDelta) {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }
    } while(endLoop == 0);

    // Se seteaza valorile optime pentru DCO
    CSCTL0 = csCtl0Copy;
    CSCTL1 = csCtl1Copy;
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));
}
