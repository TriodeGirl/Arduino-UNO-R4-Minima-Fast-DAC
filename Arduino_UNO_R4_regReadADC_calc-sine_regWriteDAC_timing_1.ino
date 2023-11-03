/*  Arduino UNO R4 test code for fast non-blocking ADC analog-read, and DAC analog-write operation
 *  With fast sine calculation of frequency from ADC value
 *
 *  Susan Parker - 2nd November 2023.
 *    Realtime sine calc with Paul Stoffregen's 11th order Taylor Series Approximation
 *    https://www.pjrc.com/high-precision-sine-wave-synthesis-using-taylor-series/
 *    Note: I have condensed the Taylor Series code to a single set of inline asm calls
 *
 * This code is "AS IS" without warranty or liability. 
 * There may be glitches, etc. please let me know if you find any; thanks.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100

// Low Power Mode Control - See datasheet section 10
#define SYSTEM 0x40010000 // System Registers
#define SYSTEM_SBYCR   ((volatile unsigned short *)(SYSTEM + 0xE00C))      // Standby Control Register
#define SYSTEM_MSTPCRA ((volatile unsigned int   *)(SYSTEM + 0xE01C))      // Module Stop Control Register A

#define MSTP 0x40040000 // Module Registers
#define MSTP_MSTPCRB   ((volatile unsigned int   *)(MSTP + 0x7000))      // Module Stop Control Register B
#define MSTPB2   2 // CAN0
#define MSTPB8   8 // IIC1
#define MSTPB9   9 // IIC0
#define MSTPB18 18 // SPI1
#define MSTPB19 19 // SPI0
#define MSTPB22 22 // SCI9
#define MSTPB29 29 // SCI2
#define MSTPB30 30 // SCI1
#define MSTPB31 31 // SCI0

#define MSTP_MSTPCRC   ((volatile unsigned int   *)(MSTP + 0x7004))      // Module Stop Control Register C
#define MSTP_MSTPCRD   ((volatile unsigned int   *)(MSTP + 0x7008))      // Module Stop Control Register D
#define MSTPD2   2 // AGT1   - Asynchronous General Purpose Timer 1 Module
#define MSTPD3   3 // AGT0   - Asynchronous General Purpose Timer 0 Module
#define MSTPD5   5 // GPT320 and GPT321 General 32 bit PWM Timer Module
#define MSTPD6   6 // GPT162 to GPT167 General 16 bit PWM Timer Module
#define MSTPD14 14 // POEG   - Port Output Enable for GPT Module Stop
#define MSTPD16 16 // ADC140 - 14-Bit A/D Converter Module
#define MSTPD19 19 // DAC8   -  8-Bit D/A Converter Module
#define MSTPD20 20 // DAC12  - 12-Bit D/A Converter Module
#define MSTPD29 29 // ACMPLP - Low-Power Analog Comparator Module
#define MSTPD31 31 // OPAMP  - Operational Amplifier Module

// The Mode Control bits are read as 1, the write value should be 1.
// Bit value 0: Cancel the module-stop state 
// Bit value 1: Enter the module-stop state.

// ====  Asynchronous General Purpose Timer (AGT) =====
#define AGTBASE 0x40084000 
#define AGT0_AGTCR    ((volatile unsigned char  *)(AGTBASE + 0x008))  // AGT Control Register

// =========== ADC14 ============
// 35.2 Register Descriptions
#define ADCBASE 0x40050000 /* ADC Base */

#define ADC140_ADCSR   ((volatile unsigned short *)(ADCBASE + 0xC000)) // A/D Control Register
#define ADC140_ADANSA0 ((volatile unsigned short *)(ADCBASE + 0xC004)) // A/D Channel Select Register A0
#define ADC140_ADANSA1 ((volatile unsigned short *)(ADCBASE + 0xC006)) // A/D Channel Select Register A1
#define ADC140_ADADS0  ((volatile unsigned short *)(ADCBASE + 0xC008)) // A/D-Converted Value Addition/Average Channel Select Register 0
#define ADC140_ADADS1  ((volatile unsigned short *)(ADCBASE + 0xC00A)) // A/D-Converted Value Addition/Average Channel Select Register 1
#define ADC140_ADCER   ((volatile unsigned short *)(ADCBASE + 0xC00E)) // A/D Control Extended Register 
#define ADC140_ADSTRGR ((volatile unsigned short *)(ADCBASE + 0xC010)) // A/D Conversion Start Trigger Select Register
#define ADC140_ADEXICR ((volatile unsigned short *)(ADCBASE + 0xC012)) // A/D Conversion Extended Input Control Register
#define ADC140_ADANSB0 ((volatile unsigned short *)(ADCBASE + 0xC014)) // A/D Channel Select Register B0
#define ADC140_ADANSB1 ((volatile unsigned short *)(ADCBASE + 0xC016)) // A/D Channel Select Register B1
#define ADC140_ADTSDR  ((volatile unsigned short *)(ADCBASE + 0xC01A)) // A/D conversion result of temperature sensor output
#define ADC140_ADOCDR  ((volatile unsigned short *)(ADCBASE + 0xC01C)) // A/D result of internal reference voltage
#define ADC140_ADRD    ((volatile unsigned short *)(ADCBASE + 0xC01E)) // A/D Self-Diagnosis Data Register

#define ADC140_ADDR00 ((volatile unsigned short *)(ADCBASE + 0xC020))      // A1 (P000 AN00 AMP+)
#define ADC140_ADDR01 ((volatile unsigned short *)(ADCBASE + 0xC020 +  2)) // A2 (P001 AN01 AMP-) 
#define ADC140_ADDR02 ((volatile unsigned short *)(ADCBASE + 0xC020 +  4)) // A3 (P002 AN02 AMPO) 
#define ADC140_ADDR05 ((volatile unsigned short *)(ADCBASE + 0xC020 + 10)) // Aref (P010 AN05 VrefH0)
#define ADC140_ADDR09 ((volatile unsigned short *)(ADCBASE + 0xC020 + 18)) // A0 (P014 AN09 DAC)
#define ADC140_ADDR21 ((volatile unsigned short *)(ADCBASE + 0xC040 + 10)) // A4 (P101 AN21 SDA) 
#define ADC140_ADDR22 ((volatile unsigned short *)(ADCBASE + 0xC040 + 12)) // A5 (P100 AN20 SCL) 

#define ADC140_ADHVREFCNT ((volatile unsigned char  *)(ADCBASE + 0xC08A)) // A/D High-Potential/Low-Potential Reference Voltage Control Register
#define ADC140_ADADC      ((volatile unsigned char  *)(ADCBASE + 0xC00C)) // A/D-Converted Value Addition/Average Count Select Register

#define ADC140_ADSSTR00 ((volatile unsigned char *)(ADCBASE + 0xC0E0))      // AN00 A/D Sampling State Register

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
#define PFS_P013PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (13 * 4))) // N/C
#define PFS_P014PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (14 * 4))) // N/A
#define PFS_P015PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (15 * 4))) // N/A

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


// === Local Defines

// #define ADC_EXT_AREF         // Use external ADC Aref source
// #define ADC_AVARAGE          // Enable 4x averaging i.e. four sucessive conversions
 #define ADSSTR00               // Enable Sampling State Register change from default
 #define ADSSTR00_VAL 0x08      // A/D Sampling State Register 0 - Default is 0x0D

 #define SINE_DAC


bool tick_tock = false;

void setup()
  {
  Serial.begin(115200);
//  while (!Serial){};

  setup_adc();
  setup_dac();

  *ADC140_ADCSR   |= (0x01 << 15);   // Start an ADC conversion
  delayMicroseconds(10);
  *AGT0_AGTCR = 0;                     // disable Millis counter, delay etc. don't want this Interrupt messing up output stability
  }
  
void loop()
  {
          int32_t out, a , b;
          int32_t result_taylor_sine;
   static int32_t sine_amp_local = 0x7FFFFFFF;
  static uint32_t freq_time_accumulate = 0;
//  static uint32_t freq_time_delta = 0x1 << 24;  //  
  static uint32_t freq_time_delta = 0x1 << 18;  // This value gives 
          int16_t result_sine;

  int32_t angle, sum, p1, p2, p3, p5, p7, p9, p11, term; 
  uint32_t ph;
  uint16_t analog_read_value;
  uint16_t analog_write_value;

  *PFS_P103PFS_BY = 0x05;      // Pulse on D4 to trigger scope 
  *PFS_P103PFS_BY = 0x04;      // Each Port Output bit clear or set takes c. 83nS 

//  *PFS_P107PFS_BY = 0x05;      //  
  analog_read_value = *ADC140_ADDR00;    // adcValue = analogRead(analogPin); // Internal 16bit register read = c. 123nS 
  *ADC140_ADCSR |= (0x01 << 15);  // Next ADC conversion = write to register c. 300nS
//  *PFS_P107PFS_BY = 0x04;      //  

#ifdef SINE_DAC

#ifdef ADC_RIGHT_JUST
  freq_time_accumulate += (uint32_t)analog_read_value << 16;   // Core DDS frequency-synthesis incriment
#else
  freq_time_accumulate += (uint32_t)analog_read_value << 14;   // Core DDS frequency-synthesis incriment
#endif

  if (freq_time_accumulate >= 0xC0000000u || freq_time_accumulate < 0x40000000u)
    {
    angle = (int32_t)freq_time_accumulate; // valid from -90 to +90 degrees
    } 
  else
    {
    angle = (int32_t)(0x7FFFFFFFu - freq_time_accumulate);
    }
  term = angle << 1;
	asm volatile("smmulr %0, %1, %2" : "=r" (p1) : "r" (term), "r" (1686629713));
	asm volatile("smmulr %0, %1, %2" : "=r" (term) : "r" (p1), "r" (p1));
  p2 = term << 3;
	asm volatile("smmulr %0, %1, %2" : "=r" (term) : "r" (p2), "r" (p1));
  p3 = term << 3;
  term = p1 << 1;
	asm volatile("smmlsr %0, %2, %3, %1" : "=r" (sum) : "r" (term), "r" (p3), "r" (1431655765));
	asm volatile("smmulr %0, %1, %2" : "=r" (term) : "r" (p3), "r" (p2));
  p5 = term << 1;
	asm volatile("smmlar %0, %2, %3, %1" : "=r" (sum) : "r" (sum), "r" (p5), "r" (286331153));
	asm volatile("smmulr %0, %1, %2" : "=r" (p7) : "r" (p5), "r" (p2));
	asm volatile("smmlsr %0, %2, %3, %1" : "=r" (sum) : "r" (sum), "r" (p7), "r" (54539267));
/*  
// Uncommet this for 25bit precision calculation
	asm volatile("smmulr %0, %1, %2" : "=r" (p9) : "r" (p7), "r" (p2));
	asm volatile("smmlar %0, %2, %3, %1" : "=r" (sum) : "r" (sum), "r" (p9), "r" (6059919));
	asm volatile("smmulr %0, %1, %2" : "=r" (p11) : "r" (p9), "r" (p2));
	asm volatile("smmlsr %0, %2, %3, %1" : "=r" (sum) : "r" (sum), "r" (p11), "r" (440721));
*/

  result_taylor_sine = sum << 1;   

//	asm volatile("smmulr %0, %1, %2" : "=r" (out) : "r" (result_taylor_sine), "r" (sine_amp_local));
//  result_sine = (int16_t)(out >> 19);

#ifdef DAC_RIGHT_JUST
  result_sine = (int16_t)(result_taylor_sine >> 20);   // 32 - 12 = 20
  *DAC12_DADR0 = result_sine + 2048;
#else
  result_sine = (int16_t)(result_taylor_sine >> 16);   // 
  *DAC12_DADR0 = result_sine + 32767;
#endif

#else
	  
#ifdef ADC_RIGHT_JUST
  analog_read_value = analog_read_value >> 2;  // 
  analog_write_value = (~analog_read_value & 0x0FFF);  // do this outside the DAC timed window
#else
  analog_write_value = (~analog_read_value);  // do this outside the DAC timed window
#endif

  if(tick_tock == true)
    {
    *PFS_P107PFS_BY = 0x05;      // Set HIGH
    *DAC12_DADR0 = analog_read_value;  // 
    *PFS_P107PFS_BY = 0x04;      // Read plus Set LOW = c. 250nS
    tick_tock = false;
    }
  else
    {
    *PFS_P107PFS_BY = 0x05;      // Set HIGH
    *DAC12_DADR0 = analog_write_value;  // 
    *PFS_P107PFS_BY = 0x04;      // Read plus Set LOW = c. 250nS
    tick_tock = true;
    }
#endif

//  delayMicroseconds(1000);  // cant use delay() because AGT0 is stopped
  }

void setup_adc(void)
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD16));  // Enable ADC140 module
#ifdef ADC_EXT_AREF
  *ADC140_ADHVREFCNT = 0x01;         // Set External Aref = analogReference(AR_EXTERNAL);      
#endif
#ifdef ADC_RIGHT_JUST
  *ADC140_ADCER = 0x06;              // 14 bit mode, right justified
#else
  *ADC140_ADCER = 0x8006;            // 14 bit mode, left-justified
//  *ADC140_ADCER = 0x8000;            // 12 bit mode, left-justified
#endif
  *ADC140_ADANSA0 |= (0x01 << 0);    // Selected ANSA00 = A1 as DAC is on A0
#ifdef ADC_AVARAGE
  *ADC140_ADADC    = 0x83;           // Average mode - 4x and b7 to enable averaging
  *ADC140_ADADS0  |= (0x01 << 0);    // Enable Averaging for ANSA00 channel
#endif
#ifdef ADSSTR00
  *ADC140_ADSSTR00 = ADSSTR00_VAL;   // A/D Sampling State Register 0 - Default is 0x0D
#endif
  }

void setup_dac(void)       // Note make sure ADC is stopped before setup DAC
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD20));  // Enable DAC12 module
#ifdef DAC_RIGHT_JUST
  *DAC12_DADPR    = 0x00;        // DADR0 Format Select Register - Set right-justified format
#else
  *DAC12_DADPR    = 0x80;        // DADR0 Format Select Register - Set left-justified format i.e. 16 bit format, 4 LSBs not used
#endif
//  *DAC12_DAADSCR  = 0x80;        // D/A A/D Synchronous Start Control Register - Enable
  *DAC12_DAADSCR  = 0x00;        // D/A A/D Synchronous Start Control Register - Default
// 36.3.2 Notes on Using the Internal Reference Voltage as the Reference Voltage
  *DAC12_DAVREFCR = 0x00;        // D/A VREF Control Register - Write 0x00 first - see 36.2.5
  *DAC12_DADR0    = 0x0000;      // D/A Data Register 0 
   delayMicroseconds(10);        
  *DAC12_DAVREFCR = 0x01;        // D/A VREF Control Register - Select AVCC0/AVSS0 for Vref
//  *DAC12_DAVREFCR = 0x03;        // D/A VREF Control Register - Select Internal reference voltage/AVSS0
//  *DAC12_DAVREFCR = 0x06;        // D/A VREF Control Register - Select External Vref; set VREFH&L pins used for LEDs
  *DAC12_DACR     = 0x5F;        // D/A Control Register - 
   delayMicroseconds(5);         // 
  *DAC12_DADR0    = 0x0800;      // D/A Data Register 0 
  *PFS_P014PFS   = 0x00000000;   // Make sure all cleared
  *PFS_P014PFS  |= (0x1 << 15);  // Port Mode Control - Used as an analog pin
  }

