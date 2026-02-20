
//#############################################################################
//
// FILE:    wsepwm.c (Wolfspeed Enhanced Pulse-Width Modulation - WS EPWM)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Control the enhanced pulse-width modulation (EPWM) outputs of the
//  controller such as controlling switching frequency, modulation index, and
//  deadtime.
//
//#############################################################################

//
// Included Files
//
#include "wsepwm.h"

//*****************************************************************************
//
//! Set initial operating parameters such as switching frequency, dead time,
//! fundamental frequency, modulation factor, EPWM angles, etc.
//!
//! \return None.
//
//*****************************************************************************
void initOperatingParameters(void) {
    //
    // Default Operating Conditions
    //
    switchingFreq       = DEFAULTSWITCHINGFREQ;     // [Hz] default switching frequency
    deadTime            = DEFAULTDEADTIME;          // [ns] default dead time
    fundamentalFreq     = DEFAULTFUNDAMENTALFREQ;   // [Hz] default fundamental frequency
    modulationFactor    = DEFAULTMODFACTOR;         // [unitless, 0-1] default modulation factor

    //
    // Phase Angles
    //
    epwmAngles[0]       = 0;                        // [rad] U (0 deg)
    epwmAngles[1]       = 4.0*PI/3.0;               // [rad] V (-120 deg = +240 deg)
    epwmAngles[2]       = 2.0*PI/3.0;               // [rad] W (-240 deg = +120 deg)

    //
    // Set EPWM Clock Parameters (timebase period, deadtime, counter compare)
    // Values are dependent on above operating conditions so cannot be set in SysConfig
    //
    updateClockCycleParameters();
}

//*****************************************************************************
//
//! Update EPWM counter compare value based on current phase angle and
//! modulation factor.
//!
//! \param base is the EPWM base to update.
//! \param epwmIndividualAngle is the current EPWM phase angle.
//!
//! \return None.
//
//*****************************************************************************
void updateIndividualOpenLoopSinePwm(uint32_t base, float epwmIndividualAngle) {
    // Sine Wave Math
    float Sine = (modulationFactor*sinf(epwmIndividualAngle)+1.0)/2.0;

    // Update Counter Compare Value
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, (uint16_t)(Sine*epwmTimebasePeriod));
}

//*****************************************************************************
//
//! Update EPWM counter compare values of all phases in open-loop sine PWM
//! control mode based on current phase angles and modulation factor.
//!
//! \return None.
//
//*****************************************************************************
void updateOpenLoopSinePwms(void) {
    updateIndividualOpenLoopSinePwm(myEPWM1_BASE, epwmAngles[0]);
    updateIndividualOpenLoopSinePwm(myEPWM2_BASE, epwmAngles[1]);
    updateIndividualOpenLoopSinePwm(myEPWM3_BASE, epwmAngles[2]);
}

//*****************************************************************************
//
//! Update EPWM clock parameters. This needs to be performed whenever the
//! switching frequency and/or fundamental frequency is changed (most
//! commonly after CAN communication).
//!
//! \return None.
//
//*****************************************************************************
void updateClockCycleParameters(void) {
    // [rad] phase step
    angleStep           = PI2 / (switchingFreq/fundamentalFreq);

    // [clock cycles] EPWM time step
    epwmTimebasePeriod  = SYSCLKFREQ/switchingFreq/2;

    // [unitless] # of switching periods in {RMSAVERAGE} # of fundamental periods
    rmsAvgBins          = switchingFreq*RMSAVERAGE/fundamentalFreq;
}

//*****************************************************************************
//
//! Set time base period, dead time, and default counter compare value for all
//! used EPWMs. Users can change these values through CAN interface, so these
//! values cannot be hard-coded in SysConfig interface. The initialization
//! ("init") performs the same operation as when EPWM settings are updated by
//! the user through the CAN interface, so init simply calls the same setup
//! function. Use the "init" naming convention so users recognize that EPWMs
//! are being initialized when reading main function.
//!
//! \return None.
//
//*****************************************************************************
void initEpwm(void) {
    setAllEpwm();
}

//*****************************************************************************
//
//! Set time base period, dead time, and default counter compare value for all
//! used EPWMs. Users can change these values through CAN interface, so these
//! values cannot be hard-coded in SysConfig interface.
//!
//! \return None.
//
//*****************************************************************************
void setAllEpwm(void) {
    //
    // Set ePWM Specific parameters (phase shift, dead time, etc.)
    //
    setIndividualEpwm(myEPWM1_BASE);
    setIndividualEpwm(myEPWM2_BASE);
    setIndividualEpwm(myEPWM3_BASE);
}

//*****************************************************************************
//
//! Set time base period, dead time, and default counter compare value for each
//! EPWM. Users can change these values through CAN interface, so these values
//! cannot be hard-coded in SysConfig interface.
//!
//! \param EPWM base is the base of the EPWM being modified
//!
//! \return None.
//
//*****************************************************************************
void setIndividualEpwm(uint32_t base) {
    //
    // Set EPWM Period
    //
    EPWM_setTimeBasePeriod(base, epwmTimebasePeriod);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, epwmTimebasePeriod*0.5); //start with 50%

    //
    // Set Dead Time
    //
    EPWM_setRisingEdgeDelayCount(base, deadTime/10);
    EPWM_setFallingEdgeDelayCount(base, deadTime/10);
}

//
// End of File
//
