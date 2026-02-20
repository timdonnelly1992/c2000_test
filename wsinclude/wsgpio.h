
//#############################################################################
//
// FILE:    wsgpio.h (Wolfspeed General Purpose Input/Output - WS GPIO)
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
// Header Guards
//
#ifndef WSGPIO
#define WSGPIO

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"

//
// Function Prototypes - LEDs
//
void enableOnBoardRedLed(void);
void disableOnBoardRedLed(void);
void toggleOnBoardRedLed(void);
void enableOnBoardYellowLed(void);
void disableOnBoardYellowLed(void);
void toggleOnBoardYellowLed(void);
void enableOnBoardGreenLed(void);
void disableOnBoardGreenLed(void);
void toggleOnBoardGreenLed(void);
void enableControlCardLed1(void);
void disableControlCardLed1(void);
void toggleControlCardLed1(void);
void enableControlCardLed2(void);
void disableControlCardLed2(void);
void toggleControlCardLed2(void);
void enableOnBoardLeds(void);
void disableOnBoardLeds(void);
void toggleOnBoardLeds(void);

//
// Function Prototypes - Gate Drivers Enable/Disable
//
void enableGateDrivers(void);
void disableGateDrivers(void);
uint16_t getGateDrivers(void);
void setGateDrivers(uint16_t setVal);

//
// Function Prototypes - High-Current Outputs (Relays)
//
void enableRelay1(void);
void disableRelay1(void);
uint16_t getRelay1(void);
void setRelay1(uint16_t setVal);
void enableRelay2(void);
void disableRelay2(void);
uint16_t getRelay2(void);
void setRelay2(uint16_t setVal);
void enableRelays(void);
void disableRelays(void);

//
// Function Prototypes - Spare GPIOs
//
void enableSpareGpioOutput1(void);
void disableSpareGpioOutput1(void);
void toggleSpareGpioOutput1(void);
uint16_t getSpareGpioOutput1(void);
void setSpareGpioOutput1(uint16_t setVal);
void enableSpareGpioOutput2(void);
void disableSpareGpioOutput2(void);
void toggleSpareGpioOutput2(void);
uint16_t getSpareGpioOutput2(void);
void setSpareGpioOutput2(uint16_t setVal);
uint16_t getSpareGpioInput1(void);
uint16_t getSpareGpioInput2(void);

//
// Function Prototypes - Faults
//
uint16_t getTripZoneFault(uint32_t base);
uint16_t getFaultU(void);
uint16_t getFaultV(void);
uint16_t getFaultW(void);
void clearGateDriverFaults(void);

#endif
//
// End of file
//
