
//#############################################################################
//
// FILE:    wsgpio.c (Wolfspeed General Purpose Input/Output - WS GPIO)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Control general purpose input/output (GPIO) pin functions. Supports
//  reading/controlling various digital inputs/outputs for operating functions
//  such as LEDs or enabling external relays.
//
//#############################################################################

//
// Included Files
//
#include "wsgpio.h"

//*****************************************************************************
//
//! Turn on the general-purpose red LED.
//!
//! \return None.
//
//*****************************************************************************
void enableOnBoardRedLed(void) {
    GPIO_writePin(LED_RED_PIN,1);
}

//*****************************************************************************
//
//! Turn off the general-purpose red LED.
//!
//! \return None.
//
//*****************************************************************************
void disableOnBoardRedLed(void) {
    GPIO_writePin(LED_RED_PIN,0);
}

//*****************************************************************************
//
//! Toggle the general-purpose red LED.
//!
//! \return None.
//
//*****************************************************************************
void toggleOnBoardRedLed(void) {
    GPIO_togglePin(LED_RED_PIN);
}

//*****************************************************************************
//
//! Turn on the general-purpose yellow LED.
//!
//! \return None.
//
//*****************************************************************************
void enableOnBoardYellowLed(void) {
    GPIO_writePin(LED_YELLOW_PIN,1);
}

//*****************************************************************************
//
//! Turn off the general-purpose yellow LED.
//!
//! \return None.
//
//*****************************************************************************
void disableOnBoardYellowLed(void) {
    GPIO_writePin(LED_YELLOW_PIN,0);
}

//*****************************************************************************
//
//! Toggle the general-purpose yellow LED.
//!
//! \return None.
//
//*****************************************************************************
void toggleOnBoardYellowLed(void) {
    GPIO_togglePin(LED_YELLOW_PIN);
}

//*****************************************************************************
//
//! Turn on the general-purpose green LED.
//!
//! \return None.
//
//*****************************************************************************
void enableOnBoardGreenLed(void) {
    GPIO_writePin(LED_GREEN_PIN,1);
}

//*****************************************************************************
//
//! Turn off the general-purpose green LED.
//!
//! \return None.
//
//*****************************************************************************
void disableOnBoardGreenLed(void) {
    GPIO_writePin(LED_GREEN_PIN,0);
}

//*****************************************************************************
//
//! Toggle the general-purpose green LED.
//!
//! \return None.
//
//*****************************************************************************
void toggleOnBoardGreenLed(void) {
    GPIO_togglePin(LED_GREEN_PIN);
}

//*****************************************************************************
//
//! Turn on the first controlCARD red LED.
//!
//! \return None.
//
//*****************************************************************************
void enableControlCardLed1(void) {
    GPIO_writePin(LED_CONTROLCARD1,0);
}

//*****************************************************************************
//
//! Turn off the first controlCARD red LED.
//!
//! \return None.
//
//*****************************************************************************
void disableControlCardLed1(void) {
    GPIO_writePin(LED_CONTROLCARD1,1);
}

//*****************************************************************************
//
//! Toggle the first controlCARD red LED.
//!
//! \return None.
//
//*****************************************************************************
void toggleControlCardLed1(void) {
    GPIO_togglePin(LED_CONTROLCARD1);
}

//*****************************************************************************
//
//! Turn on the second controlCARD red LED.
//!
//! \return None.
//
//*****************************************************************************
void enableControlCardLed2(void) {
    GPIO_writePin(LED_CONTROLCARD2,0);
}

//*****************************************************************************
//
//! Turn off the second controlCARD red LED.
//!
//! \return None.
//
//*****************************************************************************
void disableControlCardLed2(void) {
    GPIO_writePin(LED_CONTROLCARD2,1);
}

//*****************************************************************************
//
//! Toggle the second controlCARD red LED.
//!
//! \return None.
//
//*****************************************************************************
void toggleControlCardLed2(void) {
    GPIO_togglePin(LED_CONTROLCARD2);
}

//*****************************************************************************
//
//! Turn on all general-purpose LEDs (red, yellow, green - not controlCARD
//! LEDs).
//!
//! \return None.
//
//*****************************************************************************
void enableOnBoardLeds(void) {
    enableOnBoardRedLed();
    enableOnBoardYellowLed();
    enableOnBoardGreenLed();
}

//*****************************************************************************
//
//! Turn off all general-purpose LEDs (red, yellow, green - not controlCARD
//! LEDs).
//!
//! \return None.
//
//*****************************************************************************
void disableOnBoardLeds(void) {
    disableOnBoardRedLed();
    disableOnBoardYellowLed();
    disableOnBoardGreenLed();
}

//*****************************************************************************
//
//! Toggle all general-purpose LEDs (red, yellow, green - not controlCARD
//! LEDs).
//!
//! \return None.
//
//*****************************************************************************
void toggleOnBoardLeds(void) {
    toggleOnBoardRedLed();
    toggleOnBoardYellowLed();
    toggleOnBoardGreenLed();
}

//*****************************************************************************
//
//! Enable PWM output of all gate drivers in hardware.
//!
//! \return None.
//
//*****************************************************************************
void enableGateDrivers(void) {
    GPIO_writePin(GATEDRIVE_DISABLE, 0);
}

//*****************************************************************************
//
//! Disable PWM output of all gate drivers in hardware.
//!
//! \return None.
//
//*****************************************************************************
void disableGateDrivers(void) {
    GPIO_writePin(GATEDRIVE_DISABLE, 1);
}

//*****************************************************************************
//
//! Get status of gate drive logic enable/disable pin.
//!
//! \return 1 if gate drivers are enabled, 0 if gate drivers are disabled.
//
//*****************************************************************************
uint16_t getGateDrivers(void) {
    //
    // Get Gate Driver Control Pin Status
    //
    uint16_t gateDriveStatus = GPIO_readPinDataRegister(GATEDRIVE_DISABLE);

    //
    // Invert Result To Match Expected Return Values
    //
    if (gateDriveStatus == 0)
        return 1;
    else
        return 0;
}

//*****************************************************************************
//
//! Enable/disable PWM output of all gate drivers in hardware depending on the
//! function input parameters.
//!
//! \param setVal is the control parameter for the gate drivers. Value of 1
//! enables the gate drivers. All other values disable the gate drivers.
//!
//! \return None.
//
//*****************************************************************************
void setGateDrivers(uint16_t setVal) {
    if (setVal == 1)
        enableGateDrivers();
    else
        disableGateDrivers();
}

//*****************************************************************************
//
//! Enable high-current output #1. Named "RELAY" since driving relays were the
//! initial motivation for including high-current outputs.
//!
//! \return None.
//
//*****************************************************************************
void enableRelay1(void) {
    GPIO_writePin(RELAY1_OUT,1);
}

//*****************************************************************************
//
//! Disable high-current output #1. Named "RELAY" since driving relays were the
//! initial motivation for including high-current outputs.
//!
//! \return None.
//
//*****************************************************************************
void disableRelay1(void) {
    GPIO_writePin(RELAY1_OUT,0);
}

//*****************************************************************************
//
//! Get status of high-current output #1. Named "RELAY" since driving relays
//! were the initial motivation for including high-current outputs.
//!
//! \return 1 if high-current output #1 is enabled, 0 if high-current output #1
//! is disabled.
//
//*****************************************************************************
uint16_t getRelay1(void) {
    return GPIO_readPinDataRegister(RELAY1_OUT);
}

//*****************************************************************************
//
//! Enable/disable high-current output #1 depending on the function input
//! parameters. Named "RELAY" since driving relays were the initial motivation
//! for including high-current outputs.
//!
//! \param setVal is the control parameter for the high-current output #1.
//! Value of 1 enables the high-current output #1. All other values disable
//! the high-current output #1.
//!
//! \return None.
//
//*****************************************************************************
void setRelay1(uint16_t setVal) {
    if (setVal == 1)
        enableRelay1();
    else
        disableRelay1();
}

//*****************************************************************************
//
//! Enable high-current output #2. Named "RELAY" since driving relays were the
//! initial motivation for including high-current outputs.
//!
//! \return None.
//
//*****************************************************************************
void enableRelay2(void) {
    GPIO_writePin(RELAY2_OUT,1);
}

//*****************************************************************************
//
//! Disable high-current output #1. Named "RELAY" since driving relays were the
//! initial motivation for including high-current outputs.
//!
//! \return None.
//
//*****************************************************************************
void disableRelay2(void) {
    GPIO_writePin(RELAY2_OUT,0);
}

//*****************************************************************************
//
//! Get status of high-current output #2. Named "RELAY" since driving relays
//! were the initial motivation for including high-current outputs.
//!
//! \return 1 if high-current output #2 is enabled, 0 if high-current output #2
//! is disabled.
//
//*****************************************************************************
uint16_t getRelay2(void) {
    return GPIO_readPinDataRegister(RELAY2_OUT);
}

//*****************************************************************************
//
//! Enable/disable high-current output #1 depending on the function input
//! parameters. Named "RELAY" since driving relays were the initial motivation
//! for including high-current outputs.
//!
//! \param setVal is the control parameter for the high-current output #1.
//! Value of 1 enables the high-current output #1. All other values disable
//! the high-current output #1.
//!
//! \return None.
//
//*****************************************************************************
void setRelay2(uint16_t setVal) {
    if (setVal == 1)
        enableRelay2();
    else
        disableRelay2();
}

//*****************************************************************************
//
//! Enable high-current output #1 and #2. Named "RELAY" since driving relays
//! were the initial motivation for including high-current outputs.
//!
//! \return None.
//
//*****************************************************************************
void enableRelays(void) {
    enableRelay1();
    enableRelay2();
}

//*****************************************************************************
//
//! Disable high-current output #1 and #2. Named "RELAY" since driving relays
//! were the initial motivation for including high-current outputs.
//!
//! \return None.
//
//*****************************************************************************
void disableRelays(void) {
    disableRelay1();
    disableRelay2();
}

//*****************************************************************************
//
//! Turn on spare GPIO output #1.
//!
//! \return None.
//
//*****************************************************************************
void enableSpareGpioOutput1(void) {
    GPIO_writePin(SPARE_GPIO_OUTPUT1,1);
}

//*****************************************************************************
//
//! Turn off spare GPIO output #1.
//!
//! \return None.
//
//*****************************************************************************
void disableSpareGpioOutput1(void) {
    GPIO_writePin(SPARE_GPIO_OUTPUT1,0);
}

//*****************************************************************************
//
//! Toggle spare GPIO output #1.
//!
//! \return None.
//
//*****************************************************************************
void toggleSpareGpioOutput1(void) {
    GPIO_togglePin(SPARE_GPIO_OUTPUT1);
}

//*****************************************************************************
//
//! Get status of spare GPIO output #1.
//!
//! \return 1 if spare GPIO output #1 is enabled, 0 if spare GPIO output #1 is
//! disabled.
//
//*****************************************************************************
uint16_t getSpareGpioOutput1(void) {
    return GPIO_readPinDataRegister(SPARE_GPIO_OUTPUT1);
}

//*****************************************************************************
//
//! Enable/disable spare GPIO output #1 depending on the function input
//! parameters.
//!
//! \param setVal is the control parameter for spare GPIO output #1. All other
//! values disable spare GPIO output #1
//!
//! \return None.
//
//*****************************************************************************
void setSpareGpioOutput1(uint16_t setVal) {
    if (setVal == 1)
        enableSpareGpioOutput1();
    else
        disableSpareGpioOutput1();
}

//*****************************************************************************
//
//! Turn on spare GPIO output #2.
//!
//! \return None.
//
//*****************************************************************************
void enableSpareGpioOutput2(void) {
    GPIO_writePin(SPARE_GPIO_OUTPUT2,1);
}

//*****************************************************************************
//
//! Turn off spare GPIO output #2.
//!
//! \return None.
//
//*****************************************************************************
void disableSpareGpioOutput2(void) {
    GPIO_writePin(SPARE_GPIO_OUTPUT2,0);
}

//*****************************************************************************
//
//! Toggle spare GPIO output #2.
//!
//! \return None.
//
//*****************************************************************************
void toggleSpareGpioOutput2(void) {
    GPIO_togglePin(SPARE_GPIO_OUTPUT2);
}

//*****************************************************************************
//
//! Get status of spare GPIO output #2.
//!
//! \return 1 if spare GPIO output #2 is enabled, 0 if spare GPIO output #2 is
//! disabled.
//
//*****************************************************************************
uint16_t getSpareGpioOutput2(void) {
    return GPIO_readPinDataRegister(SPARE_GPIO_OUTPUT2);
}

//*****************************************************************************
//
//! Enable/disable spare GPIO output #2 depending on the function input
//! parameters.
//!
//! \param setVal is the control parameter for spare GPIO output #2. All other
//! values disable spare GPIO output #2
//!
//! \return None.
//
//*****************************************************************************
void setSpareGpioOutput2(uint16_t setVal) {
    if (setVal == 1)
        enableSpareGpioOutput2();
    else
        disableSpareGpioOutput2();
}

//*****************************************************************************
//
//! Get status of spare GPIO input #1.
//!
//! \return 1 if spare GPIO input #1 is enabled, 0 if spare GPIO input #1 is
//! disabled.
//
//*****************************************************************************
uint16_t getSpareGpioInput1(void) {
    return GPIO_readPin(SPARE_GPIO_INPUT1);
}

//*****************************************************************************
//
//! Get status of spare GPIO input #2.
//!
//! \return 1 if spare GPIO input #2 is enabled, 0 if spare GPIO input #2 is
//! disabled.
//
//*****************************************************************************
uint16_t getSpareGpioInput2(void) {
    return GPIO_readPin(SPARE_GPIO_INPUT2);
}

//*****************************************************************************
//
//! Get status of EPWM base trip zone fault.
//!
//! \param base is the EPWM base which is being checked for trip zone fault.
//!
//! \return 1 if EPWM base trip zone is faulted, 0 if EPWM base trip zone is
//!     not faulted.
//
//*****************************************************************************
uint16_t getTripZoneFault(uint32_t base) {
    if (EPWM_getTripZoneFlagStatus(base) != 0)
        return 1;
    return 0;
}

//*****************************************************************************
//
//! Get status of Phase U fault.
//!
//! \return 1 if Phase U is faulted, 0 if Phase U is not faulted.
//
//*****************************************************************************
uint16_t getFaultU(void) {
    uint16_t faultStatus = ADC_getPPBEventStatus(myADC2_BASE, IU_PPB);

    if (faultStatus == 0)
        return 0;
    else
        return 1;
}

//*****************************************************************************
//
//! Get status of Phase V fault.
//!
//! \return 1 if Phase V is faulted, 0 if Phase V is not faulted.
//
//*****************************************************************************
uint16_t getFaultV(void) {
    uint16_t faultStatus = ADC_getPPBEventStatus(myADC2_BASE, IV_PPB);

    if (faultStatus == 0)
        return 0;
    else
        return 1;
}

//*****************************************************************************
//
//! Get status of Phase W fault.
//!
//! \return 1 if Phase W is faulted, 0 if Phase W is not faulted.
//
//*****************************************************************************
uint16_t getFaultW(void) {
    uint16_t faultStatus = ADC_getPPBEventStatus(myADC2_BASE, IW_PPB);

    if (faultStatus == 0)
        return 0;
    else
        return 1;
}

//*****************************************************************************
//
//! Clear gate driver faults.
//!
//! \return None.
//
//*****************************************************************************
void clearGateDriverFaults(void) {
    //
    // Clear Post-Processing Block (PPB) Faults
    //
    ADC_clearPPBEventStatus(myADC2_BASE, IU_PPB, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));
    ADC_clearPPBEventStatus(myADC2_BASE, IV_PPB, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));
    ADC_clearPPBEventStatus(myADC2_BASE, IW_PPB, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO));

    //
    // Acknowledge this interrupt to receive more interrupts from group 10
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);

    //
    // Ensure PPB Faults are Cleared Before Clearing Trip Zone Faults
    //
    DEVICE_DELAY_US(10);

    //
    // Clear Trip Zone Faults
    //
    EPWM_clearTripZoneFlag(myEPWM1_BASE, (EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST | EPWM_TZ_INTERRUPT));
    EPWM_clearTripZoneFlag(myEPWM2_BASE, (EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST | EPWM_TZ_INTERRUPT));
    EPWM_clearTripZoneFlag(myEPWM3_BASE, (EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST | EPWM_TZ_INTERRUPT));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// End of file
//
