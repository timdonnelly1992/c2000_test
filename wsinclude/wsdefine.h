
//#############################################################################
//
// FILE:    wsdefine.h (Wolfspeed Definitions - WS Define)
//
// TITLE:   CRD25DA12N-FMC 25 kW Controller Code
//
// AUTHOR:  WOLFSPEED
//
// PURPOSE:
//  Define constant values used throughout firmware.
//
//#############################################################################

//
// Header Guards
//
#ifndef WSDEFINE
#define WSDEFINE

//
// Math Parameters
//
#define PI                      3.141592654f    // pi
#define PI2                     6.283185308f    // 2*pi
#define SQRT3                   1.732050808f    // sqrt(3)

//
// System Parameters
//
#define SYSCLKFREQ              120e6           // [Hz] TMS320f280039C clock frequency
#define SYSCLKPERIOD            8.3333e-9f      // [s] 1/SYSCLKFREQ
#define ADCMAXVALUE             4095            // [-] maximum measured value of ADC (2^12-1)
#define ADCMIDVALUE             2047            // [-] middle of ADC range ((2^12-1)/2)

//
// Default Operating Parameters
//
#define DEFAULTSWITCHINGFREQ    20e3            // [Hz] default switching frequency
#define DEFAULTFUNDAMENTALFREQ  60              // [Hz] default fundamental frequency
#define DEFAULTMODFACTOR        0.01f           // [unitless, 0-1] default modulation factor
#define DEFAULTDEADTIME         200             // [ns] default dead time
#define DEFAULTOVERCURRENT      150             // [A] default overcurrent trip value

//
// Control Loop Limits
//
#define VOLTAGE_LOOP_INT_LIMIT_A            15.0f           // [A] outer voltage-loop integrator clamp
#define VOLTAGE_LOOP_IREF_LIMIT_A           15.0f           // [A] outer loop current-reference clamp
#define VDC_LOOP_INT_LIMIT_A                15.0f           // [A] DC-bus loop integrator clamp (active rectifier mode)
#define VDC_MIN_FOR_MODULATION_V            50.0f           // [V] minimum Vdc used for modulation normalization
#define CURRENT_LOOP_VCMD_MAX_ABS_V         400.0f          // [V] hard clamp on dq voltage commands
#define CURRENT_LOOP_DECOUPLING_L_H         0.001f          // [H] equivalent filter/phase inductance used for wL decoupling
#define CURRENT_LOOP_VOLTAGE_FF_GAIN        1.0f            // [-] dq voltage feedforward gain

//
// PLL Validity Thresholds
//
#define PLL_LOCK_VQ_ABS_MAX_V               20.0f           // [V] max |Vq| for PLL lock detect
#define PLL_LOCK_FREQ_ERR_MAX_HZ            5.0f            // [Hz] max |f_est-f_nom| for PLL lock detect
#define PLL_LOCK_COUNT_ON                   200u            // [samples] consecutive-good count to assert lock
#define PLL_LOCK_COUNT_OFF                  50u             // [samples] consecutive-bad count to clear lock

//
// Minimum/Maximum Operating Parameters
//
#define MINSWITCHINGFREQ        10u             // [kHz] minimum switching frequency
#define MAXSWITCHINGFREQ        100u            // [kHz] maximum switching frequency
#define MINFUNDAMENTALFREQ      50u             // [Hz] minimum fundamental frequency
#define MAXFUNDAMENTALFREQ      1000u           // [Hz] maximum fundamental frequency
#define MINMODFACTOR            0u              // [unitless, 0-1000] minimum modulation factor
#define MAXMODFACTOR            1000u           // [unitless, 0-1000] maximum modulation factor
#define MINDEADTIME             100u            // [ns] minimum dead time
#define MAXDEADTIME             2000u           // [ns] maximum dead time

//
// Buffer/Size Parameters
//
#define BUFFERSIZE              200             // current/voltage storage buffer size
#define RMSAVERAGE              3               // number of periods to average the calculated rms currents
#define ZEROCALSIZE             3               // number of times to average the sensor offsets
#define NUMPHASES               3               // number of output phases
#define NUMAVGSENSORS           7               // number of sensor that need RMS calculation (3 currents; 4 voltages)

//
// Indexes
//
#define INDEXCURRENTU           0               // array index to store phase U current
#define INDEXCURRENTV           1               // array index to store phase V current
#define INDEXCURRENTW           2               // array index to store phase W current
#define INDEXVOLTAGEU           3               // array index to store phase U voltage
#define INDEXVOLTAGEV           4               // array index to store phase V voltage
#define INDEXVOLTAGEW           5               // array index to store phase W voltage
#define INDEXVOLTAGEDC          6               // array index to store DC voltage

//
// CAN Parameters
//
#define CANOBJECTTXCONTROL      1               // CAN object number for transmitting control feedback
#define CANOBJECTRXCONTROL      2               // CAN object number for receiving control parameters
#define CANOBJECTTXTEMP         3               // CAN object number for transmitting temperature feedback
#define CANOBJECTTXCURRENT      4               // CAN object number for transmitting current feedback
#define CANOBJECTTXVOLTAGE      5               // CAN object number for transmitting voltage feedback
#define CANOBJECTRXADD          6               // CAN object number for receiving additional control parameters
#define CANMESSAGELENGTH        8               // CAN message length in bytes

//
// Phase Alias
//
#define SENSORU                 0               // alias number for phase U sensors
#define SENSORV                 1               // alias number for phase V sensors
#define SENSORW                 2               // alias number for phase W sensors
#define SENSORDC                3               // alias number for DC sensors

#endif
//
// End of File
//
