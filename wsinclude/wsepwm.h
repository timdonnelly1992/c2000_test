
//#############################################################################
//
// FILE:    wsepwm.h (Wolfspeed Enhanced Pulse-Width Modulation - WS EPWM)
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
// Header Guards
//
#ifndef WSEPWM
#define WSEPWM

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"
#include "wsinclude/wsdefine.h"

//
// Global Variables
//
extern float modulationFactor;
extern float switchingFreq;
extern float angleStep;
extern float epwmAngles[NUMPHASES];
extern uint16_t epwmTimebasePeriod;
extern uint16_t deadTime;
extern uint16_t fundamentalFreq;
extern uint16_t rmsAvgBins;

//
// Function Prototypes
//
void updateIndividualOpenLoopSinePwm(uint32_t base, float epwmIndividualAngle);
void updateOpenLoopSinePwms(void);
void updateClockCycleParameters(void);
void initEpwm(void);
void setAllEpwm(void);
void setIndividualEpwm(uint32_t base);
void initOperatingParameters(void);

#endif
//
// End of File
//
