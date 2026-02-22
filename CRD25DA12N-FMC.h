
//#############################################################################
//
// FILE:    CRD25DA12N-FMC.h (Wolfspeed Primary CRD25DA12N-FMC Header)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Header for main function of CRD25DA12N-FMC reference design code. Define
//  variables and functions used in main.
//
//#############################################################################

//
// Header Guards
//
#ifndef WSMAIN
#define WSMAIN

//
// Include Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"                           // standard C math functions
#include "wsinclude/wsgpio.h"               // GPIO pin control
#include "wsinclude/wssensor.h"             // voltage & current sensors
#include "wsinclude/wscan.h"                // CAN communication
#include "wsinclude/wsepwm.h"               // Enhanced PWM control
#include "CRD25DA12N-FMC_shared.h"          // CLA shared functions/variables

//
// Globals
//
float       angleStep;                      // [radians] phase angle step size
float       modulationFactor;               // [unitless, 0-1] output sine wave modulation factor
float       epwmAngles[NUMPHASES];          // [radians] phase angles
float       currentUBuffer[BUFFERSIZE];     // [A] Iu measurement buffer for plotting vs time
float       currentVBuffer[BUFFERSIZE];     // [A] Iv measurement buffer for plotting vs time
float       currentWBuffer[BUFFERSIZE];     // [A] Iw measurement buffer for plotting vs time
float       sensor2Sums[NUMAVGSENSORS]  = {0,0,0,0,0,0};    // [A^2, V^2] cumulative sum of sensor mesaurements squared
float       sensorRms[NUMAVGSENSORS];       // [A, V] RMS value of sensor measurement buffers
float       switchingFreq;                  // [Hz] switching frequency
uint16_t    deadTime;                       // [ns] dead time
uint16_t    fundamentalFreq;                // [Hz] fundamental/line frequency
uint16_t    rmsAvgBins;                     // [-] # of switching periods in RMSAVERAGE # of fund periods
uint16_t    bufferIndex;                    // [-] index counter for tracking ADC storage buffer
uint16_t    epwmTimebasePeriod;             // [s] Enhanced PWM (EPWM) period
uint16_t    rmsIndex = 0;                   // [-] index counter for tracking number of RMS data points
uint16_t    voltageSampleCounter;           // [-] decimation counter for voltage ADC sampling
float       idCurrentMeas;                  // [A] measured d-axis current
float       iqCurrentMeas;                  // [A] measured q-axis current
float       idCurrentErr;                   // [A] d-axis current error
float       iqCurrentErr;                   // [A] q-axis current error
float       vdCommand;                      // [V] d-axis controller output (pre-normalized)
float       vqCommand;                      // [V] q-axis controller output (pre-normalized)
float       idIntegrator;                   // [V] d-axis current PI integrator state
float       iqIntegrator;                   // [V] q-axis current PI integrator state
float       pllTheta;                       // [radians] PLL estimated electrical angle
float       pllOmega;                       // [rad/s] PLL estimated electrical frequency
float       pllVq;                          // [V] PLL q-axis voltage error signal
float       pllIntegrator;                  // [rad/s] PLL integral state
float       pllFreqErrHz;                   // [Hz] PLL estimated frequency error from nominal
uint16_t    pllLockCounter;                 // [-] lock detector debounce counter
uint16_t    pllLocked;                      // [0/1] PLL lock status
uint16_t    pllError;                       // [0/1] PLL validity fault flag for control
float       vdVoltageMeas;                  // [V] measured d-axis voltage
float       vqVoltageMeas;                  // [V] measured q-axis voltage
float       vdVoltageRef;                   // [V] d-axis voltage reference
float       vqVoltageRef;                   // [V] q-axis voltage reference
float       vdVoltageErr;                   // [V] d-axis voltage error
float       vqVoltageErr;                   // [V] q-axis voltage error
float       voltageKp;                      // [A/V] outer voltage-loop proportional gain
float       voltageKi;                      // [A/V/s] outer voltage-loop integral gain
float       vdVoltageIntegrator;            // [A] d-axis voltage-loop integrator
float       vqVoltageIntegrator;            // [A] q-axis voltage-loop integrator
float       idRefFromVoltageLoop;           // [A] inner-loop d-axis reference from voltage loop
float       iqRefFromVoltageLoop;           // [A] inner-loop q-axis reference from voltage loop
float       vdcVoltageRef;                  // [V] DC-bus voltage reference for active rectifier mode
float       vdcVoltageErr;                  // [V] DC-bus voltage control error
float       dcVoltageKp;                    // [A/V] DC-bus loop proportional gain
float       dcVoltageKi;                    // [A/V/s] DC-bus loop integral gain
float       vdcVoltageIntegrator;           // [A] DC-bus loop integrator state
float       idRefFromDcVoltageLoop;         // [A] d-axis current reference generated by DC-bus loop
uint16_t    manualRelayCommand;             // [0/1] 0=open relays, 1=close relays
uint16_t    manualRelay1Command;            // [0/1] manual command for relay 1
uint16_t    manualRelay2Command;            // [0/1] manual command for relay 2

//
// CLA Task 1 Variables - CPU to CLA
//
#pragma DATA_SECTION(epwmTimebasePeriod,"CpuToCla1MsgRAM");
uint16_t epwmTimebasePeriod;                // [clock cycles] Enhanced PWM (EPWM) period
#pragma DATA_SECTION(modulationFactor,"CpuToCla1MsgRAM");
float modulationFactor;                     // [unitless, 0-1]
#pragma DATA_SECTION(angleStep,"CpuToCla1MsgRAM");
float angleStep;                            // [radians] phase angle increment per switching period
#pragma DATA_SECTION(epwmAngles,"CpuToCla1MsgRAM");
float epwmAngles[NUMPHASES];                // [radians] phase angles
#pragma DATA_SECTION(electricalAngle,"CpuToCla1MsgRAM");
float electricalAngle;                      // [radians] electrical angle for dq transform
#pragma DATA_SECTION(phaseCurrentU,"CpuToCla1MsgRAM");
float phaseCurrentU;                        // [A] phase-U current sample
#pragma DATA_SECTION(phaseCurrentV,"CpuToCla1MsgRAM");
float phaseCurrentV;                        // [A] phase-V current sample
#pragma DATA_SECTION(phaseCurrentW,"CpuToCla1MsgRAM");
float phaseCurrentW;                        // [A] phase-W current sample
#pragma DATA_SECTION(phaseVoltageU,"CpuToCla1MsgRAM");
float phaseVoltageU;                        // [V] phase-U voltage sample
#pragma DATA_SECTION(phaseVoltageV,"CpuToCla1MsgRAM");
float phaseVoltageV;                        // [V] phase-V voltage sample
#pragma DATA_SECTION(phaseVoltageW,"CpuToCla1MsgRAM");
float phaseVoltageW;                        // [V] phase-W voltage sample
#pragma DATA_SECTION(dcBusVoltageMeas,"CpuToCla1MsgRAM");
float dcBusVoltageMeas;                     // [V] measured DC-link voltage
#pragma DATA_SECTION(idRefCurrent,"CpuToCla1MsgRAM");
float idRefCurrent;                         // [A] d-axis current reference
#pragma DATA_SECTION(iqRefCurrent,"CpuToCla1MsgRAM");
float iqRefCurrent;                         // [A] q-axis current reference
#pragma DATA_SECTION(currentKp,"CpuToCla1MsgRAM");
float currentKp;                            // [V/A] proportional gain for dq current loop
#pragma DATA_SECTION(currentKi,"CpuToCla1MsgRAM");
float currentKi;                            // [V/A/s] integral gain for dq current loop
#pragma DATA_SECTION(currentControlEnable,"CpuToCla1MsgRAM");
uint16_t currentControlEnable;              // [0/1] enable CLA current-controller math
#pragma DATA_SECTION(controlMode,"CpuToCla1MsgRAM");
uint16_t controlMode;                       // [0=open,1=current,2=ac-voltage+current,3=dc-voltage(active rectifier)+current]
#pragma DATA_SECTION(angleSourceSelect,"CpuToCla1MsgRAM");
uint16_t angleSourceSelect;                 // [0/1] 0=open-loop angle, 1=PLL angle
#pragma DATA_SECTION(controlPeriodSec,"CpuToCla1MsgRAM");
float controlPeriodSec;                     // [s] control period (normally 1/switchingFreq)
#pragma DATA_SECTION(pllKp,"CpuToCla1MsgRAM");
float pllKp;                                // [rad/s/V] SRF-PLL proportional gain
#pragma DATA_SECTION(pllKi,"CpuToCla1MsgRAM");
float pllKi;                                // [rad/s^2/V] SRF-PLL integral gain
#pragma DATA_SECTION(pllOmegaNom,"CpuToCla1MsgRAM");
float pllOmegaNom;                          // [rad/s] nominal grid electrical frequency

//
// CLA Task 1 Variables - CLA to CPU
//
#pragma DATA_SECTION(idCurrentMeas,"Cla1ToCpuMsgRAM");
float idCurrentMeas;                        // [A] measured d-axis current
#pragma DATA_SECTION(iqCurrentMeas,"Cla1ToCpuMsgRAM");
float iqCurrentMeas;                        // [A] measured q-axis current
#pragma DATA_SECTION(idCurrentErr,"Cla1ToCpuMsgRAM");
float idCurrentErr;                         // [A] d-axis error
#pragma DATA_SECTION(iqCurrentErr,"Cla1ToCpuMsgRAM");
float iqCurrentErr;                         // [A] q-axis error
#pragma DATA_SECTION(vdCommand,"Cla1ToCpuMsgRAM");
float vdCommand;                            // [V] d-axis voltage command (pre-normalized)
#pragma DATA_SECTION(vqCommand,"Cla1ToCpuMsgRAM");
float vqCommand;                            // [V] q-axis voltage command (pre-normalized)
#pragma DATA_SECTION(idIntegrator,"Cla1ToCpuMsgRAM");
float idIntegrator;                         // [V] d-axis current PI integrator state
#pragma DATA_SECTION(iqIntegrator,"Cla1ToCpuMsgRAM");
float iqIntegrator;                         // [V] q-axis current PI integrator state
#pragma DATA_SECTION(vdVoltageMeas,"Cla1ToCpuMsgRAM");
float vdVoltageMeas;                        // [V] measured d-axis voltage
#pragma DATA_SECTION(vqVoltageMeas,"Cla1ToCpuMsgRAM");
float vqVoltageMeas;                        // [V] measured q-axis voltage
#pragma DATA_SECTION(vdVoltageRef,"CpuToCla1MsgRAM");
float vdVoltageRef;                         // [V] d-axis voltage reference
#pragma DATA_SECTION(vqVoltageRef,"CpuToCla1MsgRAM");
float vqVoltageRef;                         // [V] q-axis voltage reference
#pragma DATA_SECTION(vdVoltageErr,"Cla1ToCpuMsgRAM");
float vdVoltageErr;                         // [V] d-axis voltage error
#pragma DATA_SECTION(vqVoltageErr,"Cla1ToCpuMsgRAM");
float vqVoltageErr;                         // [V] q-axis voltage error
#pragma DATA_SECTION(voltageKp,"CpuToCla1MsgRAM");
float voltageKp;                            // [A/V] outer voltage-loop proportional gain
#pragma DATA_SECTION(voltageKi,"CpuToCla1MsgRAM");
float voltageKi;                            // [A/V/s] outer voltage-loop integral gain
#pragma DATA_SECTION(vdVoltageIntegrator,"Cla1ToCpuMsgRAM");
float vdVoltageIntegrator;                  // [A] d-axis voltage-loop integrator
#pragma DATA_SECTION(vqVoltageIntegrator,"Cla1ToCpuMsgRAM");
float vqVoltageIntegrator;                  // [A] q-axis voltage-loop integrator
#pragma DATA_SECTION(idRefFromVoltageLoop,"Cla1ToCpuMsgRAM");
float idRefFromVoltageLoop;                 // [A] inner-loop d-axis ref generated by voltage PI
#pragma DATA_SECTION(iqRefFromVoltageLoop,"Cla1ToCpuMsgRAM");
float iqRefFromVoltageLoop;                 // [A] inner-loop q-axis ref generated by voltage PI
#pragma DATA_SECTION(vdcVoltageRef,"CpuToCla1MsgRAM");
float vdcVoltageRef;                        // [V] DC-bus voltage reference
#pragma DATA_SECTION(vdcVoltageErr,"Cla1ToCpuMsgRAM");
float vdcVoltageErr;                        // [V] DC-bus voltage control error
#pragma DATA_SECTION(dcVoltageKp,"CpuToCla1MsgRAM");
float dcVoltageKp;                          // [A/V] DC-bus loop proportional gain
#pragma DATA_SECTION(dcVoltageKi,"CpuToCla1MsgRAM");
float dcVoltageKi;                          // [A/V/s] DC-bus loop integral gain
#pragma DATA_SECTION(vdcVoltageIntegrator,"Cla1ToCpuMsgRAM");
float vdcVoltageIntegrator;                 // [A] DC-bus loop integrator state
#pragma DATA_SECTION(idRefFromDcVoltageLoop,"Cla1ToCpuMsgRAM");
float idRefFromDcVoltageLoop;               // [A] d-axis current reference from DC-bus loop
#pragma DATA_SECTION(pllTheta,"Cla1ToCpuMsgRAM");
float pllTheta;                             // [radians] PLL estimated electrical angle
#pragma DATA_SECTION(pllOmega,"Cla1ToCpuMsgRAM");
float pllOmega;                             // [rad/s] PLL estimated electrical frequency
#pragma DATA_SECTION(pllVq,"Cla1ToCpuMsgRAM");
float pllVq;                                // [V] PLL q-axis voltage error signal
#pragma DATA_SECTION(pllIntegrator,"Cla1ToCpuMsgRAM");
float pllIntegrator;                        // [rad/s] PLL integral state
#pragma DATA_SECTION(pllFreqErrHz,"Cla1ToCpuMsgRAM");
float pllFreqErrHz;                         // [Hz] PLL frequency error from nominal
#pragma DATA_SECTION(pllLockCounter,"Cla1ToCpuMsgRAM");
uint16_t pllLockCounter;                    // [-] lock detector debounce counter
#pragma DATA_SECTION(pllLocked,"Cla1ToCpuMsgRAM");
uint16_t pllLocked;                         // [0/1] PLL lock status
#pragma DATA_SECTION(pllError,"Cla1ToCpuMsgRAM");
uint16_t pllError;                          // [0/1] PLL validity fault flag for control

//
// Interrupt Function Prototypes
//
__interrupt void epwm1ISR(void);
__interrupt void cla1Isr1(void);
void setCurrentControllerMode(uint16_t enable);
void setCurrentControllerReferences(float idRef, float iqRef);
void setCurrentControllerGains(float kp, float ki);
void setControlMode(uint16_t mode);
void setVoltageControllerReferences(float vdRef, float vqRef);
void setVoltageControllerGains(float kp, float ki);
void setDcVoltageControllerReference(float vdcRef);
void setDcVoltageControllerGains(float kp, float ki);
void setAngleSourceMode(uint16_t mode);
void setPllGains(float kp, float ki);
void setPllNominalFrequency(float gridFreqHz);

#endif
//
// End of file
//
