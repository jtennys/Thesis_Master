//*****************************************************************************
//*****************************************************************************
//  FILENAME: RECEIVE_2.h
//   Version: 3.3, Updated on 2009/7/10 at 10:46:15
//  Generated by PSoC Designer 5.0.985.0
//
//  DESCRIPTION:  RX8 User Module C Language interface file for the
//                22/24/25/26/27xxx PSoC family of devices.
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************


// include the global header file
#include <m8c.h>

#define RECEIVE_2_RXBUF_ENABLE 1

//-------------------------------------------------
// Prototypes of the RECEIVE_2 API.
//-------------------------------------------------

#if ( RECEIVE_2_RXBUF_ENABLE )
extern char RECEIVE_2_aRxBuffer[];
extern BYTE RECEIVE_2_bRxCnt;
extern BYTE RECEIVE_2_fStatus;
#endif

// Create pragmas to support proper argument and return value passing
#pragma fastcall16  RECEIVE_2_EnableInt
#pragma fastcall16  RECEIVE_2_DisableInt
#pragma fastcall16  RECEIVE_2_Start
#pragma fastcall16  RECEIVE_2_Stop
#pragma fastcall16  RECEIVE_2_bReadRxData
#pragma fastcall16  RECEIVE_2_bReadRxStatus

#pragma fastcall16  RECEIVE_2_cGetChar
#pragma fastcall16  RECEIVE_2_cReadChar
#pragma fastcall16  RECEIVE_2_iReadChar

#if ( RECEIVE_2_RXBUF_ENABLE )
#pragma fastcall16  RECEIVE_2_CmdReset
#pragma fastcall16  RECEIVE_2_bCmdCheck
#pragma fastcall16  RECEIVE_2_bErrCheck
#pragma fastcall16  RECEIVE_2_bCmdLength
#pragma fastcall16  RECEIVE_2_szGetParam
#pragma fastcall16  RECEIVE_2_szGetRestOfParams
#endif

//-------------------------------------------------
// Prototypes of the RECEIVE_2 API.
//-------------------------------------------------
extern void  RECEIVE_2_EnableInt(void);
extern void  RECEIVE_2_DisableInt(void);
extern void  RECEIVE_2_Start(BYTE bParity);
extern void  RECEIVE_2_Stop(void);
extern BYTE  RECEIVE_2_bReadRxData(void);
extern BYTE  RECEIVE_2_bReadRxStatus(void);

// High level RX functions
extern CHAR         RECEIVE_2_cGetChar(void);
extern CHAR         RECEIVE_2_cReadChar(void);
extern INT          RECEIVE_2_iReadChar(void);

#if ( RECEIVE_2_RXBUF_ENABLE )
extern void   RECEIVE_2_CmdReset(void);
extern BYTE   RECEIVE_2_bCmdCheck(void);
extern BYTE   RECEIVE_2_bErrCheck(void);
extern BYTE   RECEIVE_2_bCmdLength(void);
extern char * RECEIVE_2_szGetParam(void);
extern char * RECEIVE_2_szGetRestOfParams(void);
#endif

// Old function call names, do not use.
// These names will be removed in a future release.
#pragma fastcall16 bRECEIVE_2_ReadRxData
#pragma fastcall16 bRECEIVE_2_ReadRxStatus
extern BYTE bRECEIVE_2_ReadRxData(void);
extern BYTE bRECEIVE_2_ReadRxStatus(void);

//-------------------------------------------------
// Constants for RECEIVE_2 API's.
//-------------------------------------------------

//------------------------------------
// Receiver Interrupt masks
//------------------------------------
#define RECEIVE_2_INT_REG_ADDR                 ( 0x0e1 )
#define RECEIVE_2_bINT_MASK                    ( 0x08 )

//------------------------------------
// Receiver Parity masks
//------------------------------------
#define  RECEIVE_2_PARITY_NONE         0x00
#define  RECEIVE_2_PARITY_EVEN         0x02
#define  RECEIVE_2_PARITY_ODD          0x06

//------------------------------------
//  Receiver Status Register masks
//------------------------------------
#define  RECEIVE_2_RX_ACTIVE           0x10
#define  RECEIVE_2_RX_COMPLETE         0x08
#define  RECEIVE_2_RX_PARITY_ERROR     0x80
#define  RECEIVE_2_RX_OVERRUN_ERROR    0x40
#define  RECEIVE_2_RX_FRAMING_ERROR    0x20
#define  RECEIVE_2_RX_NO_ERROR         0xE0

#define  RECEIVE_2_RX_NO_DATA         0x01

#define  RECEIVE_2_RX_BUF_ERROR           0xF0  // Mask for any Rx that may occur.
#define  RECEIVE_2_RX_BUF_OVERRUN         0x10  // This indicates the software buffer has
                                                           // been over run.
#define  RECEIVE_2_RX_BUF_CMDTERM         0x01  // Command terminator has been received.

// Old defines, will be removed in future release
#define  RX8_PARITY_NONE         0x00
#define  RX8_PARITY_EVEN         0x02
#define  RX8_PARITY_ODD          0x06
#define  RX8_RX_ACTIVE           0x10
#define  RX8_RX_COMPLETE         0x08
#define  RX8_RX_PARITY_ERROR     0x80
#define  RX8_RX_OVERRUN_ERROR    0x40
#define  RX8_RX_FRAMING_ERROR    0x20
#define  RX8_RX_NO_ERROR         0xE0

//-------------------------------------------------
// Register Addresses for RECEIVE_2
//-------------------------------------------------
#pragma ioport  RECEIVE_2_CONTROL_REG:  0x02f              // Control register
BYTE            RECEIVE_2_CONTROL_REG;
#pragma ioport  RECEIVE_2_RX_SHIFT_REG: 0x02c              // RX Shift Register register
BYTE            RECEIVE_2_RX_SHIFT_REG;
#pragma ioport  RECEIVE_2_RX_BUFFER_REG:    0x02e              // RX Buffer Register
BYTE            RECEIVE_2_RX_BUFFER_REG;
#pragma ioport  RECEIVE_2_FUNC_REG: 0x12c                  // Function register
BYTE            RECEIVE_2_FUNC_REG;
#pragma ioport  RECEIVE_2_INPUT_REG:    0x12d              // Input register
BYTE            RECEIVE_2_INPUT_REG;
#pragma ioport  RECEIVE_2_OUTPUT_REG:   0x12e              // Output register
BYTE            RECEIVE_2_OUTPUT_REG;

// end of file RECEIVE_2.h

