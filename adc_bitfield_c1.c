// adc module
// cpu1 - adc module A , cpu2 - adc module C
// ADC module A - pin A0, A1
// ADC module C - pin C3, C5
// Mock analog signals

// GPIO22 and 52 - rectangular waveforms
// 50Hz waveforms - triangular waveform (grid)
// CPU timer 1 - configured to toggle at 10ms delay


// RC circuit ot these gpio pins : 2.2 Kohm and 2.22 microF and 3.3microF
// Trigger

// ADC module A will be triggered by epwm1
// ADC module C will be triggered by epwm4




#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"

//global variables
     // feed to control

__interrupt void timer1_isr(void) {
// Execute control algorithms
    // Toggle GPIO 52
    GpioDataRegs.GPATOGGLE.bit.GPIO22=1;
    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;


    // Cleanup
    // Clear flag in timer module
    CpuTimer1Regs.TCR.bit.TIF = 1;
}


// ============ Control loop parameters — set/tune for your hardware ============

// ADC scaling
//#define ADC_VREF        3.3f      // ADC reference voltage (V) — confirm for your board
//#define ADC_MAX_COUNT   4095.0f   // 12-bit ADC, 0-4095

// Sensor scaling — SET THESE from your actual divider / sensor circuit
//#define VSENSE_GAIN     3.0f      // (R1+R2)/R2 of your Vout divider — volts-at-Vout per volt-at-ADC-pin
//#define ISENSE_GAIN     1.0f      // amps per volt out of your current sensor
//#define ISENSE_OFFSET   1.65f     // sensor's zero-current output (0 if unipolar/shunt referenced to GND)

// Setpoint
float VOUT_REF =       1.2f  ;    // desired regulated output voltage (V)

// PI gains — START SMALL, see tuning notes below
float KP     =         0.02f;
float KI        =      0.0005f;
float error, duty; // defined globaly for watchin in debug window
// Duty limits — protect the hardware, leave dead-time margin
#define DUTY_MIN        0.05f
#define DUTY_MAX        0.85f

// Software overcurrent backstop (real protection = HW trip-zone, see note below)
#define I_LIMIT         5.0f      // amps

// ============ Persistent control state ============
float volt, curr;           // now float, not double — native to the C28x's 32-bit FPU
float integral = 0.0f;
uint16_t fault_flag = 0;

__interrupt void adca1_isr(void)
{
    uint16_t volt_raw, curr_raw;
    

    // 1. Read the conversion results
    volt_raw = AdcaResultRegs.ADCRESULT0;   // SOC0 — A0, output voltage
    curr_raw = AdcaResultRegs.ADCRESULT1;   // SOC1 — A1, inductor/output current

    // 2. Raw counts -> physical units
    volt = ((float)volt_raw *3.3/4095); //* ADC_VREF * VSENSE_GAIN;
    curr = ((float)curr_raw*3.3 / 4095) ;//* ADC_VREF - ISENSE_OFFSET) * ISENSE_GAIN;

    if (curr > I_LIMIT)
    {
        // Software fault path — coarse backstop only, see note below
        fault_flag = 1;
        integral   = DUTY_MIN;
        EPwm1Regs.CMPA.bit.CMPA = (Uint16)(DUTY_MIN * EPwm1Regs.TBPRD);
    }
    else
    {
        fault_flag = 0;

        // 3. PI control law
        error = VOUT_REF - volt;

        integral += KI * error;
        if (integral > DUTY_MAX) integral = DUTY_MAX;
        if (integral < DUTY_MIN) integral = DUTY_MIN;

        duty = KP * error + integral;
        if (duty > DUTY_MAX) duty = DUTY_MAX;
        if (duty < DUTY_MIN) duty = DUTY_MIN;

        // 4. Update the compare register — shadow-loaded at next CTR=0, glitch-free
        EPwm1Regs.CMPA.bit.CMPA = (Uint16)(duty * EPwm1Regs.TBPRD);
    }

    // 5. Clear flags (unchanged from your version)
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}
//
// Main
//
void main(void)
{
      // Initialize System
    InitSysCtrl();
        // Disable interrupts and clear them at the CPU
    // Initialize PIE module
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;


    // Initialize PIE Vector Table
    InitPieVectTable();

    // Set frequency of ePWM clock (<100 Mhz)
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
    EDIS;

    // Assign ePWM modules to CPUs
    EALLOW;
    DevCfgRegs.CPUSEL0.bit.EPWM1 = 0;
    DevCfgRegs.CPUSEL0.bit.EPWM4 = 1;
    EDIS;

    // Assign ADC Modules to CPUs
    DevCfgRegs.CPUSEL11.bit.ADC_A=0;
    DevCfgRegs.CPUSEL11.bit.ADC_C=1;


 // Enable clock to Timer 1
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    EDIS;

    // Enable the clocks to ePWM modules 1 
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    EDIS;

    // Enable clock to ADC module A
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;
    EDIS;

    // Stop the timer counters for ePWM 1 
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    
    // Configure Timer 1
    // Stop the timers
    CpuTimer1Regs.TCR.bit.TSS = 1;

    // Load the period value
    // Timer 1 = 0.01 second interval
    CpuTimer1Regs.PRD.all = 2000000 - 1;

    // Load optional pre-scale value
    CpuTimer1Regs.TPR.all = 0;

    // Set the mode of running
    CpuTimer1Regs.TCR.bit.FREE = 1;
    
    // Enable timer interrupts
    CpuTimer1Regs.TCR.bit.TIE = 1;

    // Reload period and prescale
    CpuTimer1Regs.TCR.bit.TRB = 1;

    // Configure the ePWM modules 1 and 2
    // Time base sub-module
    // 100 MHz. TBCLK = 100 / (2 * 1) = 50 Mhz
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm1Regs.TBCTL.bit.PRDLD = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;  // up count mode

    // Period register
    EPwm1Regs.TBPRD = 5000 - 1;         // 10 kHz sawtooth

     // Counter-compare sub-module
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;

    EPwm1Regs.CMPA.bit.CMPA = 1999;

    // Action qualifier sub-module
    // ePWM1 - A output to be fed to the upper device of a leg
    // B output to be fed to the lower device of a leg
    // TBCTR = 0, A will be high, B will be low (ZRO - AQCTLA/B)
    // TBCTR = CMPA. A will be low and B will be high (CAU - AQCTLA/B)
    EPwm1Regs.AQCTLA.bit.ZRO = 2;
    EPwm1Regs.AQCTLB.bit.ZRO = 2;       // due to POLSEL in DBCTL
    EPwm1Regs.AQCTLA.bit.CAU = 1;
    EPwm1Regs.AQCTLB.bit.CAU = 1;       // due to POLSEL in DBCTL

    // Dead-band sub-module
    // DBCTL register
    EPwm1Regs.DBCTL.bit.IN_MODE = 1;  // A - rising edge, B - falling edge
    EPwm1Regs.DBCTL.bit.POLSEL = 2;     // falling edge (B) is inverted - change AQCTLB
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;   // both delay generators are enabled

    // DBRED/DBFED registers
    // 2 microsecond delay
    EPwm1Regs.DBRED.all = 100;
    EPwm1Regs.DBFED.all = 100;

    // Configure the Event trigger sub-module
    EPwm1Regs.ETSEL.bit.SOCAEN= 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 1; // when TBCTR = 0
    EPwm1Regs.ETPS.bit.SOCAPRD= 1;  // every PWM cycle

    // CONFIGURE THE ADC MODULE 

    // divide the clock ADCCLK<=50MHZ
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE=6;
    

    // Powerup the ADC module
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ=1;

    // Set mode and resolution of ADC using library functionad
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    


    // configure the channels---Its esentaly about taking the analog inputs
    

    AdcaRegs.ADCSOC0CTL.bit.CHSEL=0;        // A0 selected
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL=5;      // ePWM1 SocA
    AdcaRegs.ADCSOC0CTL.bit.ACQPS=99;     // 500ms S&H time

    AdcaRegs.ADCSOC1CTL.bit.CHSEL=1;        // A1 selected
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL=5;      // ePWM1 SocA
    AdcaRegs.ADCSOC1CTL.bit.ACQPS=99;     // 500ms S&H time

    

    //configure the interrupt such that when all the conversion is completed INTR gets generated
    
    //SOC from ePWM1 SOCA-A0 (SOC0) and then A1(SOC1)
    // a0 is sampled after that its cconverted
    // while a0 is converted, a1 is sampled
    // after a0 is converted, a1 is converted
    // after every conversion, there is an EOC
    // EOC0 and then EOC1
    // ADCINT1 can be after EOC1

    AdcaRegs.ADCINTSEL1N2.bit.INT1E=1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT=0; // continuous mode  is off
     AdcaRegs.ADCINTSEL1N2.bit.INT1SEL=1;
    EDIS;




    // Register ISRs for Timer 1
    EALLOW;
    PieVectTable.TIMER1_INT = &timer1_isr;
    EDIS;

    // Register ISRs for ADC
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    EDIS;

    // configure PIE module
    // INT1.1 - ADCAINT1
    PieCtrlRegs.PIEIER1.bit.INTx1=1;



    
    // Enable global interrupts and configure IER
    // Enable Timer 1
   IER |= 0x1000U;   // Timer 1 (INT13)
IER |= 0x0001U;   // ADC Group 1 (INT1) — needed for adca1_isr
EINT;


    // Configure GPIO0 to GPIO3 as ePWM functionality
    EALLOW;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
    EDIS;

    EALLOW;
    // Configure GPIO 22 and GPIO 52 as output pins
    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;
    EDIS;



    // Start the timers
    CpuTimer1Regs.TCR.bit.TSS = 0;

    // Start the timer counters for ePWM 1 and 2
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;


    // Infinite loop
    while (1) {}

}

//
// End of File
//
