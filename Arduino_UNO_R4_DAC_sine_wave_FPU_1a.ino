/*  Arduino UNO R4 Minima code for DAC12 demonstration - SINE wave output
 *
 *  Susan Parker - 19th August 2023.
 *
 * This code is "AS IS" without warranty or liability. 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

// With thanks to @KurtE for the pointers how to get this code to use the RA4M1 Floating Point Unit
// Using the FPU for sinf() gives 6.5uS in the loop(); instead of 290uS software code exercution time

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100

// Low Power Mode Control - See datasheet section 10
#define SYSTEM 0x40010000 // System Registers
#define SYSTEM_SBYCR   ((volatile unsigned short *)(SYSTEM + 0xE00C))      // Standby Control Register
#define SYSTEM_MSTPCRA ((volatile unsigned int   *)(SYSTEM + 0xE01C))      // Module Stop Control Register A

#define MSTP 0x40040000 // Module Registers
#define MSTP_MSTPCRB   ((volatile unsigned int   *)(MSTP   + 0x7000))      // Module Stop Control Register B
#define MSTPB2   2  // CAN0
#define MSTPB8   8  // IIC1
#define MSTPB9   9  // IIC0
#define MSTPB18 18  // SPI1
#define MSTPB19 19  // SPI0
#define MSTPB22 22  // SCI9
#define MSTPB29 29  // SCI2
#define MSTPB30 30  // SCI1
#define MSTPB31 31  // SCI0

#define MSTP_MSTPCRC   ((volatile unsigned int   *)(MSTP + 0x7004))        // Module Stop Control Register C
#define MSTP_MSTPCRD   ((volatile unsigned int   *)(MSTP + 0x7008))        // Module Stop Control Register D
#define MSTPD2   2  // AGT1   - Asynchronous General Purpose Timer 1 Module
#define MSTPD3   3  // AGT0   - Asynchronous General Purpose Timer 0 Module
#define MSTPD5   5  // GPT320 and GPT321 General 32 bit PWM Timer Module
#define MSTPD6   6  // GPT162 to GPT167 General 16 bit PWM Timer Module
#define MSTPD14 14  // POEG   - Port Output Enable for GPT Module Stop
#define MSTPD16 16  // ADC140 - 14-Bit A/D Converter Module
#define MSTPD19 19  // DAC8   -  8-Bit D/A Converter Module
#define MSTPD20 20  // DAC12  - 12-Bit D/A Converter Module
#define MSTPD29 29  // ACMPLP - Low-Power Analog Comparator Module
#define MSTPD31 31  // OPAMP  - Operational Amplifier Module

// The Mode Control bits are read as 1, the write value should be 1.
// Bit value 0: Cancel the module-stop state 
// Bit value 1: Enter the module-stop state.


// ====  Asynchronous General Purpose Timer (AGT) =====
#define AGTBASE 0x40084000 
#define AGT0_AGTCR    ((volatile unsigned char  *)(AGTBASE + 0x008))  // AGT Control Register

// 12-Bit D/A Converter
#define DACBASE 0x40050000          // DAC Base - DAC output on A0 (P014 AN09 DAC)
#define DAC12_DADR0    ((volatile unsigned short *)(DACBASE + 0xE000))      // D/A Data Register 0 
#define DAC12_DACR     ((volatile unsigned char  *)(DACBASE + 0xE004))      // D/A Control Register
#define DAC12_DADPR    ((volatile unsigned char  *)(DACBASE + 0xE005))      // DADR0 Format Select Register
#define DAC12_DAADSCR  ((volatile unsigned char  *)(DACBASE + 0xE006))      // D/A A/D Synchronous Start Control Register
#define DAC12_DAVREFCR ((volatile unsigned char  *)(DACBASE + 0xE007))      // D/A VREF Control Register

// =========== Ports ============
// 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15)
#define PORTBASE 0x40040000 /* Port Base */

#define P000PFS 0x0800  // Port 0 Pin Function Select Register
#define PFS_P000PFS ((volatile unsigned int *)(PORTBASE + P000PFS))            // 
#define PFS_P001PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 1 * 4))) // 
#define PFS_P002PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 2 * 4))) // 
#define PFS_P003PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 3 * 4))) // 
#define PFS_P004PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 4 * 4))) // 
#define PFS_P005PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 5 * 4))) // 
#define PFS_P006PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 6 * 4))) // 
#define PFS_P007PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 7 * 4))) // 
#define PFS_P008PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 8 * 4))) // 
// #define PFS_P009PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 9 * 4))) // Does not exist
#define PFS_P010PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (10 * 4))) // 
#define PFS_P011PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (11 * 4))) // 
#define PFS_P012PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (12 * 4))) // 
#define PFS_P013PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (13 * 4))) // 
#define PFS_P014PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (14 * 4))) // A0 / DAC12
#define PFS_P015PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (15 * 4))) // 

#define PFS_P100PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843))   // 8 bits - A5
#define PFS_P101PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 1 * 4))) // A4
#define PFS_P102PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 2 * 4))) // D5
#define PFS_P103PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 3 * 4))) // D4
#define PFS_P104PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 4 * 4))) // D3
#define PFS_P105PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 5 * 4))) // D2
#define PFS_P106PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 6 * 4))) // D6
#define PFS_P107PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 7 * 4))) // D7
#define PFS_P108PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 8 * 4))) // SWDIO
#define PFS_P109PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 9 * 4))) // D11 / MOSI
#define PFS_P110PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (10 * 4))) // D12 / MISO
#define PFS_P111PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (11 * 4))) // D13 / SCLK
#define PFS_P112PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (12 * 4))) // D10 / CS
#define PFS_P300PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3))            // SWCLK (P300)
#define PFS_P301PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (01 * 4))) // D0 / RxD (P301)
#define PFS_P302PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (02 * 4))) // D1 / TxD (P302) 
#define PFS_P303PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (03 * 4))) // D9
#define PFS_P304PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (04 * 4))) // D8


// Quick loop DAC output test - with sine/cosine calls

void setup()
  {                                                 
  *PFS_P107PFS_BY = 0x04;               // Set D7 output low - DAC time flag pin

  setup_dac();

  *AGT0_AGTCR = 0;                     // disable Millis counter, delay etc. don't want this Interrupt messing up output stability

  Serial.begin(115200);      // The interrupts for the USB serial are already in place before setup() starts
  while (!Serial){};         // Note: USB serial cannot be used for serial comms when running fast IRQs - diagnostics only.
  }

uint16_t loop_count = 0;               // One 0 to 4095 ramp takes 2.7mS
int sampleRate = 24000;                // Not used yet

#define DOUBLE_PI 6.283185f
#define QUAD_PI   DOUBLE_PI * 2f

const float double_pi  = 6.283185f;
const float sine_range = 4096.00f;
const float sine_fraction = double_pi / sine_range;

float sine_amplitude = 2047.9f;    // Sine is +- this value
float sine_frequency =  100.0f;    // Desired frequency
float sine_input     =    0.0f;
float sine_value     =    0.0f;
float cosine_value   =    0.0f;
uint16_t result_sine = 0;
uint16_t result_cos  = 0;
uint32_t freq_time_accumulate = 0;
uint32_t freq_time_delta = 0x1 << 24;  // This value gives c. 300Hz in fast loop()
uint16_t freq_step_index = 0;

#define SINE_RANGE_BITS 12
#define SINE_RANGE_SHIFT 32 - SINE_RANGE_BITS
#define SINE_RANGE_SIZE 4096.0f
#define SINE_FRACTION_MULT 0.087890625f

void loop(void)                        // 4.26uS or 6.52uS total loop time (without D6 D7 flags)
  {
  *PFS_P107PFS_BY = 0x05;              // Set D7 output high    - takes c.  83nS

  freq_time_accumulate += freq_time_delta;   // Core DDS frequency-synthesis incriment

  freq_step_index = (uint16_t)(freq_time_accumulate >> SINE_RANGE_SHIFT);  // Index into table lookup

  sine_input = (double_pi / ( SINE_RANGE_SIZE / (float)freq_step_index ) );

  *PFS_P106PFS_BY = 0x05;               // Set D6 output high    - takes c.  83nS

  sine_value   = sinf(sine_input);        // One segment of sinf() rising positive takes c. 2.2uS; the rest c. 4.3uS 

  cosine_value = cosf(sine_input);        // ... both together between c. 4.3uS and 10uS 

  *PFS_P106PFS_BY = 0x04;               // Set D6 output low     - takes c.  83nS

  result_sine = (((int)(  sine_value * sine_amplitude)) + 2048);
  result_cos  = (((int)(cosine_value * sine_amplitude)) + 2048);  

  *DAC12_DADR0 = result_cos;          // DAC update            - takes c. 210nS 

  *PFS_P107PFS_BY = 0x04;              // Set D7 output low     - takes c.  83nS
  }


void setup_dac(void)       // Note make sure ADC is stopped before setup DAC
  {
  *MSTP_MSTPCRD &= ~(0x01 << MSTPD20);  // Enable DAC12 module
  *DAC12_DADPR    = 0x00;               // DADR0 Format Select Register - Set right-justified format
//  *DAC12_DAADSCR  = 0x80;               // D/A A/D Synchronous Start Control Register - Enable
  *DAC12_DAADSCR  = 0x00;               // D/A A/D Synchronous Start Control Register - Default
// 36.3.2 Notes on Using the Internal Reference Voltage as the Reference Voltage
  *DAC12_DAVREFCR = 0x00;               // D/A VREF Control Register - Write 0x00 first - see 36.2.5
  *DAC12_DADR0    = 0x0000;             // D/A Data Register 0 
   delayMicroseconds(10);               // Needed delay - see datasheet
  *DAC12_DAVREFCR = 0x01;               // D/A VREF Control Register - Select AVCC0/AVSS0 for Vref
  *DAC12_DACR     = 0x5F;               // D/A Control Register - 
   delayMicroseconds(5);                // 
  *DAC12_DADR0    = 0x0800;             // D/A Data Register 0 
  *PFS_P014PFS   = 0x00000000;          // Port Mode Control - Make sure all bits cleared
  *PFS_P014PFS  |= (0x1 << 15);         // ... use as an analog pin
  }
