//*****************************************************************************
//*****************************************************************************
//  FILENAME: TX_TIMEOUT.h
//   Version: 2.6, Updated on 2009/7/10 at 10:46:29
//  Generated by PSoC Designer 5.0.985.0
//
//  DESCRIPTION: Timer16 User Module C Language interface file
//               for the 22/24/27/29xxx PSoC family of devices
//-----------------------------------------------------------------------------
//  Copyright (c) Cypress MicroSystems 2000-2004. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************

#include <m8c.h>

#pragma fastcall16 TX_TIMEOUT_EnableInt
#pragma fastcall16 TX_TIMEOUT_DisableInt
#pragma fastcall16 TX_TIMEOUT_Start
#pragma fastcall16 TX_TIMEOUT_Stop
#pragma fastcall16 TX_TIMEOUT_wReadTimer                // Read  DR0
#pragma fastcall16 TX_TIMEOUT_wReadTimerSaveCV          // Read  DR0      
#pragma fastcall16 TX_TIMEOUT_WritePeriod               // Write DR1
#pragma fastcall16 TX_TIMEOUT_wReadCompareValue         // Read  DR2
#pragma fastcall16 TX_TIMEOUT_WriteCompareValue         // Write DR2

// The following symbols are deprecated.
// They may be omitted in future releases
//
#pragma fastcall16 wTX_TIMEOUT_ReadCounter              // Read  DR0 "Obsolete"
#pragma fastcall16 wTX_TIMEOUT_CaptureCounter           // Read  DR0 "Obsolete"
#pragma fastcall16 wTX_TIMEOUT_ReadTimer                // Read  DR0 (Deprecated)
#pragma fastcall16 wTX_TIMEOUT_ReadTimerSaveCV          // Read  DR0 (Deprecated)
#pragma fastcall16 wTX_TIMEOUT_ReadCompareValue         // Read  DR2 (Deprecated)


//-------------------------------------------------
// Prototypes of the TX_TIMEOUT API.
//-------------------------------------------------

extern void TX_TIMEOUT_EnableInt(void);                           // Proxy 1
extern void TX_TIMEOUT_DisableInt(void);                          // Proxy 1
extern void TX_TIMEOUT_Start(void);                               // Proxy 1
extern void TX_TIMEOUT_Stop(void);                                // Proxy 1
extern WORD TX_TIMEOUT_wReadTimer(void);                          // Proxy 1
extern WORD TX_TIMEOUT_wReadTimerSaveCV(void);                    // Proxy 2
extern void TX_TIMEOUT_WritePeriod(WORD wPeriod);                 // Proxy 1
extern WORD TX_TIMEOUT_wReadCompareValue(void);                   // Proxy 1
extern void TX_TIMEOUT_WriteCompareValue(WORD wCompareValue);     // Proxy 1

// The following functions are deprecated.
// They may be omitted in future releases
//
extern WORD wTX_TIMEOUT_ReadCompareValue(void);       // Deprecated
extern WORD wTX_TIMEOUT_ReadTimerSaveCV(void);        // Deprecated
extern WORD wTX_TIMEOUT_ReadCounter(void);            // Obsolete
extern WORD wTX_TIMEOUT_ReadTimer(void);              // Deprecated
extern WORD wTX_TIMEOUT_CaptureCounter(void);         // Obsolete


//--------------------------------------------------
// Constants for TX_TIMEOUT API's.
//--------------------------------------------------

#define TX_TIMEOUT_CONTROL_REG_START_BIT       ( 0x01 )
#define TX_TIMEOUT_INT_REG_ADDR                ( 0x0e1 )
#define TX_TIMEOUT_INT_MASK                    ( 0x02 )


//--------------------------------------------------
// Constants for TX_TIMEOUT user defined values
//--------------------------------------------------

#define TX_TIMEOUT_PERIOD                      ( 0xf0 )
#define TX_TIMEOUT_COMPARE_VALUE               ( 0x0 )


//-------------------------------------------------
// Register Addresses for TX_TIMEOUT
//-------------------------------------------------

#pragma ioport  TX_TIMEOUT_COUNTER_LSB_REG: 0x020          //Count register LSB
BYTE            TX_TIMEOUT_COUNTER_LSB_REG;
#pragma ioport  TX_TIMEOUT_COUNTER_MSB_REG: 0x024          //Count register MSB
BYTE            TX_TIMEOUT_COUNTER_MSB_REG;
#pragma ioport  TX_TIMEOUT_PERIOD_LSB_REG:  0x021          //Period register LSB
BYTE            TX_TIMEOUT_PERIOD_LSB_REG;
#pragma ioport  TX_TIMEOUT_PERIOD_MSB_REG:  0x025          //Period register MSB
BYTE            TX_TIMEOUT_PERIOD_MSB_REG;
#pragma ioport  TX_TIMEOUT_COMPARE_LSB_REG: 0x022          //Compare register LSB
BYTE            TX_TIMEOUT_COMPARE_LSB_REG;
#pragma ioport  TX_TIMEOUT_COMPARE_MSB_REG: 0x026          //Compare register MSB
BYTE            TX_TIMEOUT_COMPARE_MSB_REG;
#pragma ioport  TX_TIMEOUT_CONTROL_LSB_REG: 0x023          //Control register LSB
BYTE            TX_TIMEOUT_CONTROL_LSB_REG;
#pragma ioport  TX_TIMEOUT_CONTROL_MSB_REG: 0x027          //Control register MSB
BYTE            TX_TIMEOUT_CONTROL_MSB_REG;
#pragma ioport  TX_TIMEOUT_FUNC_LSB_REG:    0x120          //Function register LSB
BYTE            TX_TIMEOUT_FUNC_LSB_REG;
#pragma ioport  TX_TIMEOUT_FUNC_MSB_REG:    0x124          //Function register MSB
BYTE            TX_TIMEOUT_FUNC_MSB_REG;
#pragma ioport  TX_TIMEOUT_INPUT_LSB_REG:   0x121          //Input register LSB
BYTE            TX_TIMEOUT_INPUT_LSB_REG;
#pragma ioport  TX_TIMEOUT_INPUT_MSB_REG:   0x125          //Input register MSB
BYTE            TX_TIMEOUT_INPUT_MSB_REG;
#pragma ioport  TX_TIMEOUT_OUTPUT_LSB_REG:  0x122          //Output register LSB
BYTE            TX_TIMEOUT_OUTPUT_LSB_REG;
#pragma ioport  TX_TIMEOUT_OUTPUT_MSB_REG:  0x126          //Output register MSB
BYTE            TX_TIMEOUT_OUTPUT_MSB_REG;
#pragma ioport  TX_TIMEOUT_INT_REG:       0x0e1            //Interrupt Mask Register
BYTE            TX_TIMEOUT_INT_REG;


//-------------------------------------------------
// TX_TIMEOUT Macro 'Functions'
//-------------------------------------------------

#define TX_TIMEOUT_Start_M \
   ( TX_TIMEOUT_CONTROL_LSB_REG |=  TX_TIMEOUT_CONTROL_REG_START_BIT )

#define TX_TIMEOUT_Stop_M  \
   ( TX_TIMEOUT_CONTROL_LSB_REG &= ~TX_TIMEOUT_CONTROL_REG_START_BIT )

#define TX_TIMEOUT_EnableInt_M   \
   M8C_EnableIntMask(  TX_TIMEOUT_INT_REG, TX_TIMEOUT_INT_MASK )

#define TX_TIMEOUT_DisableInt_M  \
   M8C_DisableIntMask( TX_TIMEOUT_INT_REG, TX_TIMEOUT_INT_MASK )


// end of file TX_TIMEOUT.h


