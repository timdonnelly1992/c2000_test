
//#############################################################################
//
// FILE:    wscan.c (Wolfspeed Controller Area Network - WS CAN)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Controller Area Network (CAN) interface functions. Supports communicating
//  with the Wolfspeed CRD25DA12N-FMC reference design to remotely control
//  parameters such as switching frequency and dead time.
//
//#############################################################################

//
// Included Files
//
#include "wscan.h"

//*****************************************************************************
//
//! Transmit inverter control parameters via CAN. This allows user to monitor/
//! confirm all the operating parameters. Also indicate to the user if a fault
//! has occurred.
//!
//! \return None.
//
//*****************************************************************************
void sendCanControls(void) {
    //
    // Initialize Variables
    //
    uint16_t packetData[CANMESSAGELENGTH];

    //
    // Get Operating Parameters in Expected Units
    //
    uint16_t canFsw     = switchingFreq / 1e3;          // [kHz] switching frequency
    uint16_t canMf      = (modulationFactor * 1000);    // [0-1000] modulation factor
    uint16_t canTdead   = deadTime;                     // [ns] dead time
    uint16_t canFfund   = fundamentalFreq;              // [Hz] fundamental frequency

    //
    // Get Gate Driver Fault Status
    //
    uint16_t gateLogic  = getGateDrivers();
    uint16_t faultU     = getFaultU();
    uint16_t faultV     = getFaultV();
    uint16_t faultW     = getFaultW();

    //
    // Indicate Fault with Red LED, Ensure Gate Drivers are Disabled
    //
    if (faultU == 1 || faultV == 1 || faultW == 1) {
        enableOnBoardRedLed();
        disableGateDrivers();
    } else {
        disableOnBoardRedLed();
    }

    //
    // Get GPIO Status
    //
    uint16_t relay1     = getRelay1();
    uint16_t relay2     = getRelay2();
    uint16_t gpioOut1   = getSpareGpioOutput1();
    uint16_t gpioOut2   = getSpareGpioOutput2();
    uint16_t gpioIn1    = getSpareGpioInput1();
    uint16_t gpioIn2    = getSpareGpioInput2();
    uint16_t resetFault = 0;

    //
    // Concatenate Data Values into 8-Byte Message
    //
    packetData[0]       = (uint16_t)canFsw;
    packetData[1]       = (uint16_t)(canMf>>2);
    packetData[2]       = (uint16_t)(canMf<<6 | canTdead >>6);
    packetData[3]       = (uint16_t)(canTdead<<2 | canFfund>>8);
    packetData[4]       = (uint16_t)(canFfund);
    packetData[5]       = (uint16_t)(gpioOut1 << 6 | gpioOut2 << 5 | gpioIn1 << 4 | gateLogic << 2 | relay1 << 1  | relay2 );
    packetData[6]       = (uint16_t)(resetFault << 7 | faultU <<6 | faultV <<5 | faultW << 4 | gpioIn2 << 3);
    packetData[7]       = 0;

    //
    // Send Controller CAN Message
    //
    CAN_sendMessage(myCAN0_BASE, CANOBJECTTXCONTROL, CANMESSAGELENGTH, packetData);
}

//*****************************************************************************
//
//! Check that input parameter from user is within predetermined bounds. Bound
//! the value if it exceeds the minimum or maximum.
//!
//! \param val is a user input.
//! \param valMin is a predetermine acceptable minimum of the user input, val.
//! \param valMax is a predetermine acceptable maximum of the user input, val.
//!
//! \return The user input bounded by the predetermined minimum/maximum.
//
//*****************************************************************************
uint16_t validateIndividualOperatingParameter(uint16_t val, uint16_t valMin, uint16_t valMax) {
    // Check User Input Is Above Minimum
    if (val < valMin)
        return valMin;

    // Check User Input Is Below Maximum
    if (val > valMax)
        return valMax;

    // Value Is Already Within Bounds
    return val;
}

//*****************************************************************************
//
//! Receive inverter control parameters via CAN. Update inverter control
//! settings to operate using the new control parameters.
//!
//! \param packetData is the CAN message received on the CAN bus which requires
//!     decoding.
//!
//! \return None.
//
//*****************************************************************************
void receiveCanControls(uint16_t *packetData) {
    //
    // Read CAN Message Bytes
    //
    uint16_t canFsw     = (packetData[0])&0x000000FF;
    uint16_t canMf      = ((packetData[1]<<2) | (packetData[2]>>6))&0x000003FF;
    uint16_t canTdead   = (((packetData[2]&0x3F)<<6) | (packetData[3]>>2))&0x00000FFF;
    uint16_t canFfund   = (((packetData[3]&0x03)<<8) | packetData[4])&0x000003FF;
    uint16_t gpio1      = (packetData[5] & 0x40)>>6;
    uint16_t gpio2      = (packetData[5] & 0x20)>>5;
    uint16_t gateLogic  = (packetData[5] & 0x04)>>2;
    uint16_t relay1     = (packetData[5] & 0x02)>>1;
    uint16_t relay2     = packetData[5] & 0x01;
    uint16_t resetFault = (packetData[6] & 0x80)>>7;

    //
    // Set Gate Drivers
    //
    setGateDrivers(gateLogic);

    //
    // Clear Faults
    //
    if (resetFault == 1)
        clearGateDriverFaults();

    //
    // Set GPIO Outputs
    //
    setRelay1(relay1);
    setRelay2(relay2);
    setSpareGpioOutput1(gpio1);
    setSpareGpioOutput2(gpio2);

    //
    // Validate Operating Parameters
    //
    canFsw      = validateIndividualOperatingParameter(canFsw, MINSWITCHINGFREQ, MAXSWITCHINGFREQ);
    canMf       = validateIndividualOperatingParameter(canMf, MINMODFACTOR, MAXMODFACTOR);
    canTdead    = validateIndividualOperatingParameter(canTdead, MINDEADTIME, MAXDEADTIME);
    canFfund    = validateIndividualOperatingParameter(canFfund, MINFUNDAMENTALFREQ, MAXFUNDAMENTALFREQ);

    //
    // Convert units out of base
    //
    switchingFreq       = (float)canFsw*1.0e3;  // [Hz]
    modulationFactor    = canMf / 1000.0;       // [0-1]
    deadTime            = canTdead;             // [ns]
    fundamentalFreq     = canFfund;             // [Hz]
}

//*****************************************************************************
//
//! Receive inverter control parameters via CAN which cannot fit in the normal
//! control CAN message. Update inverter settings to operate using the new
//! control parameters. Currently this CAN packet only includes the
//! overcurrent trip current, but can include more in future revisions.
//!
//! \param packetData is the CAN message received on the CAN bus which requires
//!     decoding.
//!
//! \return None.
//
//*****************************************************************************
void receiveCanAdditional(uint16_t *packetData) {
    uint16_t canIoc     = (packetData[0])&0x000000FF;

    setOvercurrentProtection(canIoc);
}

//*****************************************************************************
//
//! Transmit feedback temperatures and spare measurements via CAN. Spare
//! measurements include resolver (not implemented yet), overcurrent trip
//! current, spare ADC1, and spare ADC2.
//!
//! \param ntcTemp is the measured negative temperature coefficient (NTC)
//!     temperature sensor measurement [Kelvin].
//!
//! \return None.
//
//*****************************************************************************
void sendCanTemperature(float ntcTemp) {
    uint16_t msgDataTemp[CANMESSAGELENGTH];

    uint16_t adcMeas1 = getSpareAdc1();
    uint16_t adcMeas2 = getSpareAdc2();
    float resolverMeas = getResolver();

    msgDataTemp[0] = (uint16_t)ntcTemp>>8;
    msgDataTemp[1] = (uint16_t)ntcTemp;

    msgDataTemp[2] = (uint16_t)resolverMeas>>8;
    msgDataTemp[3] = (uint16_t)resolverMeas;

    msgDataTemp[4] = (uint16_t)adcMeas1>>8;
    msgDataTemp[5] = (uint16_t)adcMeas1;

    msgDataTemp[6] = (uint16_t)adcMeas2>>8;
    msgDataTemp[7] = (uint16_t)adcMeas2;

    //Send Temperature CAN Message
    CAN_sendMessage(myCAN0_BASE, CANOBJECTTXTEMP, CANMESSAGELENGTH, msgDataTemp);
}

//*****************************************************************************
//
//! Transmit feedback RMS currents via CAN.
//!
//! \param sensorRms is the phase RMS currents [A] and voltages [V]. Array
//!     indexes are [Iu, Iv, Iw, Vu, Vv, Vw].
//!
//! \return None.
//
//*****************************************************************************
void sendCanCurrent(float *currentRms) {
    // Initialize Variables
    uint16_t msgDataCurrent[CANMESSAGELENGTH];

    // Phase U Current
    msgDataCurrent[0] = (int16_t)currentRms[0]>>8;
    msgDataCurrent[1] = (int16_t)currentRms[0];

    // Phase V Current
    msgDataCurrent[2] = (int16_t)currentRms[1]>>8;
    msgDataCurrent[3] = (int16_t)currentRms[1];

    // Phase W Current
    msgDataCurrent[4] = (int16_t)currentRms[2]>>8;
    msgDataCurrent[5] = (int16_t)currentRms[2];

    // Overcurrent Trip Current
    uint16_t overcurrentVal = getOvercurrentTripValue();
    msgDataCurrent[6] = (int16_t)overcurrentVal>>8;
    msgDataCurrent[7] = (int16_t)overcurrentVal;

    // Send Current CAN Message
    CAN_sendMessage(myCAN0_BASE, CANOBJECTTXCURRENT, CANMESSAGELENGTH, msgDataCurrent);
}

//*****************************************************************************
//
//! Transmit feedback RMS/DC voltages via CAN.
//!
//! \param sensorRms is the phase RMS currents [A] and voltages [V]. Array
//!     indexes are [Iu, Iv, Iw, Vu, Vv, Vw].
//! \param voltageDc is the measured DC voltage [V].
//!
//! \return None.
//
//*****************************************************************************
void sendCanVoltage(float *voltageRms, float voltageDc) {
    // Initialize Variables
    uint16_t msgDataVoltage[CANMESSAGELENGTH];

    // Phase U Voltage
    msgDataVoltage[0] = (int16_t)voltageRms[3]>>8;
    msgDataVoltage[1] = (int16_t)voltageRms[3];

    // Phase V Voltage
    msgDataVoltage[2] = (int16_t)voltageRms[4]>>8;
    msgDataVoltage[3] = (int16_t)voltageRms[4];

    // Phase W Voltage
    msgDataVoltage[4] = (int16_t)voltageRms[5]>>8;
    msgDataVoltage[5] = (int16_t)voltageRms[5];

    // DC Voltage
    msgDataVoltage[6] = (int16_t)voltageDc>>8;
    msgDataVoltage[7] = (int16_t)voltageDc;

    // Send Voltage CAN Message
    CAN_sendMessage(myCAN0_BASE, CANOBJECTTXVOLTAGE, CANMESSAGELENGTH, msgDataVoltage);
}

//*****************************************************************************
//
//! Transmit feedback temperatures, RMS currents, and RMS/DC voltages via CAN.
//!
//! \param ntcTemp is the measured negative temperature coefficient (NTC)
//!     temperature sensor measurement [Kelvin].
//! \param sensorRms is the phase RMS currents [A] and voltages [V]. Array
//!     indexes are [Iu, Iv, Iw, Vu, Vv, Vw].
//! \param voltageDc is the measured DC voltage [V].
//!
//! \return None.
//
//*****************************************************************************
void sendCanSensors() {
    // Force Measurement SOCs
    forceVoltageSocDC();
    forceNtcModuleSoc();

    // Get DC Voltage and Module Temperature
    float voltageDc = getVoltageDc();
    float ntcTemp   = getModuleTempK();

    // Calculate RMS Currents
    calculateSensorRms();

    // Send Temperature CAN Message
    sendCanTemperature(ntcTemp);

    // Sent Current CAN Message
    sendCanCurrent(sensorRms);

    // Send Voltage CAN Message
    sendCanVoltage(sensorRms,voltageDc);
}

//
// End of file
//
