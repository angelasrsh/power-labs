//#############################################################################
//
// FILE:   adc_ex1_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
//! \addtogroup bitfield_example_list
//! <h1>ADC ePWM Triggering</h1>
//!
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A1 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults - A sequence of analog-to-digital conversion samples from
//!   pin A1. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
//
//
// 
// C2000Ware v6.00.01.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "f28x_project.h"

//
// Defines
//
#define RESULTS_BUFFER_SIZE     256

//
// Globals
//
uint16_t adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t index;                              // Index into result buffer
volatile uint16_t bufferFull;                // Flag to indicate buffer is full
const float pwmduty = 0.25f; 
const float pwmfreq = 10e3f; //sampling frequency?
enum countermodes {up = 0, down = 1, updown = 2};
enum countermodes countingtype = updown;
float Vout = 0;
float Vin = 0;
float iL = 0;


//
// Function Prototypes
//
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
__interrupt void adcA1ISR(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    InitSysCtrl();

    //
    // Initialize GPIO
    //
    InitGpio();

    //
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();

    //
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt 1
    EDIS;

    //
    // Configure the ADC and power it up
    //
    initADC();

    //
    // Configure the ePWM
    //
    initEPWM();

    //
    // Setup the ADC for ePWM triggered conversions on channel 1
    //
    initADCSOC();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;  // Enable group 1 interrupts

    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    //
    // Initialize results buffer
    //
    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults[index] = 0;
    }

    index = 0;
    bufferFull = 0;

    //
    // Enable PIE interrupt
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //
    // Sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    //
    // Take conversions indefinitely in loop
    //
    while(1)
    {
        //
        // Start ePWM
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 0;   // Unfreeze, and enter up count mode

        //
        // Wait while ePWM causes ADC conversions, which then cause interrupts,
        // which fill the results buffer, eventually setting the bufferFull
        // flag
        //
        while(!bufferFull)
        {
        }
        bufferFull = 0; //clear the buffer full flag

        //
        // Stop ePWM
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 0;    // Disable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 3;   // Freeze counter

        //
        // Software breakpoint. At this point, conversion results are stored in
        // adcAResults.
        //
        // Hit run again to get updated conversions.
        //
        ESTOP0;
    }
}

//
// initADC - Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Setup VREF as internal
    //
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);

    EALLOW;

    //
    // Set ADCCLK divider to /4
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC and then delay for 1 ms
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    DELAY_US(1000);
}

//
// initEPWM - Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{

    EALLOW;
    // Configure GPIO12 as EPWM7A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1; 

    // Configure GPIO13 as EPWM7B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1; 
    


    // Clock Setup
    // Note: Assumes SYSCLK is 150MHz. If SYSCLK is 200MHz, HSPCLKDIV=1 means TBCLK is 100MHz.
    // The calculations below assume an effective TBCLK of 75MHz (75e6).
    EPwm7Regs.TBCTL.bit.CLKDIV = 0;     // /1
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = 0;  // /2 (Default)

    EPwm7Regs.TBCTL.bit.CTRMODE = countingtype; 

    switch (countingtype) {
        case(up):
            // Freq = TBCLK / (TBPRD + 1) -> TBPRD = (TBCLK / Freq) - 1
            EPwm7Regs.TBPRD = (uint16_t)((150000000.0f / pwmfreq) - 1);
            
            // CMPA Logic
            EPwm7Regs.CMPA.bit.CMPA = (uint16_t)((EPwm7Regs.TBPRD + 1) * pwmduty);

            // -- Channel A (Active High) --
            // High at Zero (Start), Low at Compare Match
            EPwm7Regs.AQCTLA.bit.ZRO = 2; // 2 = Set High
            EPwm7Regs.AQCTLA.bit.CAU = 1; // 1 = Clear Low
            EPwm7Regs.AQCTLA.bit.PRD = 0; // Do Nothing

            // -- Channel B (Complementary / Inverted) --
            // Low at Zero (Start), High at Compare Match
            EPwm7Regs.AQCTLB.bit.ZRO = 1; // 1 = Clear Low
            EPwm7Regs.AQCTLB.bit.CAU = 2; // 2 = Set High
            EPwm7Regs.AQCTLB.bit.PRD = 0; // Do Nothing
            break;

        case(down):
            // Freq = TBCLK / (TBPRD + 1)
            EPwm7Regs.TBPRD = (uint16_t)((150000000.0f / pwmfreq) - 1);
            
            // CMPA Logic (Inverted for Down count)
            // Note: For Down count, smaller CMPA = Longer Duty if Active High.
            EPwm7Regs.CMPA.bit.CMPA = (uint16_t)(EPwm7Regs.TBPRD - ((EPwm7Regs.TBPRD + 1) * pwmduty));

            // -- Channel A (Active High) --
            // High at Period (Start), Low at Compare Match (Down)
            EPwm7Regs.AQCTLA.bit.PRD = 2; // Set High
            EPwm7Regs.AQCTLA.bit.CAD = 1; // Clear Low
            EPwm7Regs.AQCTLA.bit.ZRO = 0; // Do Nothing

            // -- Channel B (Complementary / Inverted) --
            // Low at Period (Start), High at Compare Match (Down)
            EPwm7Regs.AQCTLB.bit.PRD = 1; // Clear Low
            EPwm7Regs.AQCTLB.bit.CAD = 2; // Set High
            EPwm7Regs.AQCTLB.bit.ZRO = 0; // Do Nothing






            break;

        case(updown):
            // Freq = TBCLK / (2 * TBPRD)
            EPwm7Regs.TBPRD = (uint16_t)(15000000.0f / pwmfreq); 
            
            EPwm7Regs.CMPA.bit.CMPA = (uint16_t)(EPwm7Regs.TBPRD * pwmduty);

            // -- Channel A (Center-Aligned Active High) --
            // High when counter < CMPA
            EPwm7Regs.AQCTLA.bit.CAU = 1; // Clear Low when counting Up past CMPA
            EPwm7Regs.AQCTLA.bit.CAD = 2; // Set High when counting Down past CMPA
            EPwm7Regs.AQCTLA.bit.ZRO = 0; // Do nothing at Zero
            EPwm7Regs.AQCTLA.bit.PRD = 0; // Do nothing at Period

            // -- Channel B (Complementary / Inverted) --
            // Low when counter < CMPA
            EPwm7Regs.AQCTLB.bit.CAU = 2; // Set High when counting Up past CMPA
            EPwm7Regs.AQCTLB.bit.CAD = 1; // Clear Low when counting Down past CMPA
            EPwm7Regs.AQCTLB.bit.ZRO = 0;
            EPwm7Regs.AQCTLB.bit.PRD = 0;

            break;
    }

    uint16_t dbValue = (uint16_t)(100.0e-9f/(1/150.0e6f)); 

    // 1. Set the Delay Values
    EPwm7Regs.DBRED.bit.DBRED = dbValue; // Rising Edge Delay
    EPwm7Regs.DBFED.bit.DBFED = dbValue; // Falling Edge Delay

    // 2. Configure DBCTL (Dead-Band Control)
    // Bit 1-0: OUT_MODE = 3 (RED and FED both enabled)
    // Bit 3-2: POLSEL = 2   (Active High Complementary: B is inverted A)
    // Bit 5-4: IN_MODE = 0  (EPWM7A is the source for both RED and FED)
    EPwm7Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm7Regs.DBCTL.bit.POLSEL = 2;
    EPwm7Regs.DBCTL.bit.IN_MODE = 0;









    EDIS;
    /*
    EALLOW;

    // EPwm1Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    // EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC on up-count
    // EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event

    // EPwm1Regs.CMPA.bit.CMPA = 0x0800;   // Set compare A value to 2048 counts

    // EPwm1Regs.TBCTL.bit.CTRMODE = 3;    // Freeze counter

    //code I wrote
    EPwm7Regs.TBCTL.bit.CLKDIV = 0; //no predivider
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = 1; //counter divides by 2 (which is default)
    EPwm7Regs.TBCTL.bit.CTRMODE = countingtype;//set ctrmode equal to the enum defined globally  

    switch (countingtype) {
        case(up):
            EPwm7Regs.TBPRD = (uint16_t)(75e6 / pwmfreq);
            EPwm7Regs.CMPA.bit.CMPA = (uint16_t)((EPwm1Regs.TBPRD+1) * pwmdutyduty);
            EPwm7Regs.AQCTLA.bit.CAD = 0;
            EPwm7Regs.AQCTLA.bit.CAU = 1;
            EPwm7Regs.AQCTLA.bit.PRD = 0;
            EPwm7Regs.AQCTLA.bit.ZRO = 0b10;
            break;
        case(down):
            EPwm7Regs.TBPRD = (uint16_t)(75e6 / pwmfreq);
            EPwm7Regs.CMPA.bit.CMPA = (uint16_t)(EPwm1Regs.TBPRD - (EPwm1Regs.TBPRD+1) * pwmdutyduty);
            EPwm7Regs.AQCTLA.bit.CAD = 1;
            EPwm7Regs.AQCTLA.bit.CAU = 0;
            EPwm7Regs.AQCTLA.bit.PRD = 0b10;
            EPwm7Regs.AQCTLA.bit.ZRO = 0;
            break;
        case(updown):
            EPwm7Regs.TBPRD = (uint16_t)(37500000 / pwmfreq);
            EPwm7Regs.CMPA.bit.CMPA = (uint16_t)(EPwm1Regs.TBPRD * pwmdutyduty);
            EPwm7Regs.AQCTLA.bit.CAD = 1;
            EPwm7Regs.AQCTLA.bit.CAU = 0;
            EPwm7Regs.AQCTLA.bit.PRD = 0;
            EPwm7Regs.AQCTLA.bit.ZRO = 1;
            break;
    }
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=1;



    EDIS;*/
}

//
// initADCSOC - Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Select the channels to convert and the end of conversion flag
    //
    EALLOW;

    // AdcaRegs.ADCSOCCTL.bit.CHSEL = 1;     // SOC0 will convert pin A1
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Trigger on ePWM1 SOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2; // End of SOC2 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared


    EPwm7Regs.ETSEL.bit.SOCAEN = 1; //generate enable 
    EPwm7Regs.ETSEL.bit.SOCASEL = 3; //
    EPwm7Regs.ETPS.bit.SOCAPRD = 1; //first event
    EPwm7Regs.ETPS.bit.INTPRD = 1; // int at first event
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0; //set divider to 4
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = 1;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 0x00E; // (100 ns / 150Mhz = ACPS)
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x11; //epwm7

    // SOC1
    AdcaRegs.ADCSOC1CTL.bit.CHSEL  = 1;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 0x00E; // (100 ns / 150Mhz = ACPS)
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x11; //epwm7

    //SOC2
    AdcaRegs.ADCSOC2CTL.bit.CHSEL  = 1;
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 0x00E; // (100 ns / 150Mhz = ACPS)
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x11; //epwm7


    EDIS;
}

//
// adcA1ISR - ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{



    // //
    // // Add the latest result to the buffer
    // // ADCRESULT0 is the result register of SOC0
    // adcAResults[index++] = AdcaResultRegs.ADCRESULT0;

    // //
    // // Set the bufferFull flag if the buffer is full
    // //
    // if(RESULTS_BUFFER_SIZE <= index)
    // {
    //     index = 0;
    //     bufferFull = 1;
    // }

    // //
    // // Clear the interrupt flag
    // //
    // AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    // //
    // // Check if overflow has occurred
    // //
    // if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    // {

        //lab 2 
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        
        // ADC parameters
        float V_adcpin = 3.3;           // ADC full-scale voltage (V)
        uint16_t adc_bits = 12;            // Number of ADC bits
        uint16_t resolution = 4095; // 2^N - 1 = 2^12 - 1

        // Voltage divider resistances
        float R_top = 100; //ki
        float R_bottom = 138;

        // Step 2: Back-compute supply voltage accounting for voltage divider
        Vout = (float)((AdcaResultRegs.ADCRESULT0)*(3.3/resolution)) * ((R_top + R_bottom)/ (R_bottom));
        Vin = (float)((AdcaResultRegs.ADCRESULT1)*(3.3/resolution)) * ((R_top + R_bottom)/ (R_bottom));
        iL = (float)((AdcaResultRegs.ADCRESULT2)*(3.3/resolution));
    // }

    // //
    // // Acknowledge the interrupt
    // //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of File
//
