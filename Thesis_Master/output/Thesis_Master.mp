  ROM 48% full. 7741 bytes used (does not include absolute areas).
  RAM 21% full. 163 bytes used (does not include stack usage).

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0190   0429 =   1065. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0190  LoadConfigTBL_pc_listener_Bank0
       01A3  LoadConfigTBL_pc_listener_Bank1
       01BC  UnloadConfigTBL_pc_listener_Bank0
       01C3  UnloadConfigTBL_pc_listener_Bank1
       01DD  LoadConfigTBL_receiver_config_Bank0
       01F0  LoadConfigTBL_receiver_config_Bank1
       0209  UnloadConfigTBL_receiver_config_Bank0
       0210  UnloadConfigTBL_receiver_config_Bank1
       022A  LoadConfigTBL_transmitter_config_Bank0
       029F  LoadConfigTBL_transmitter_config_Bank1
       0340  LoadConfigTBL_transmitter_config_Ordered
       03E9  ReloadConfigTBL_transmitter_config_Bank0
       03FC  ReloadConfigTBL_transmitter_config_Bank1
       0415  UnloadConfigTBL_transmitter_config_Bank0
       041C  UnloadConfigTBL_transmitter_config_Bank1
       0436  UnloadConfigTBL_Total_Bank0
       0457  UnloadConfigTBL_Total_Bank1
       04B8  __ctype
       05B9  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                        func_lit   05B9   0002 =      2. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       05B9  __func_lit_start
       05BB  __func_lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     psoc_config   05BB   01CF =    463. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       05BB  _LoadConfigInit
       05BB  LoadConfigInit
       05C8  _LoadConfig_transmitter_config
       05C8  LoadConfig_transmitter_config
       05EB  _ReloadConfig_transmitter_config
       05EB  ReloadConfig_transmitter_config
       060C  UnloadConfig_transmitter_config
       060C  _UnloadConfig_transmitter_config
       0631  LoadConfig_pc_listener
       0631  _LoadConfig_pc_listener
       0678  _UnloadConfig_pc_listener
       0678  UnloadConfig_pc_listener
       06BC  _LoadConfig_receiver_config
       06BC  LoadConfig_receiver_config
       06ED  UnloadConfig_receiver_config
       06ED  _UnloadConfig_receiver_config
       0725  UnloadConfig_Total
       0725  _UnloadConfig_Total
       078A  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   078A   062B =   1579. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       078A  _TX_TIMEOUT_EnableInt
       078A  TX_TIMEOUT_EnableInt
       078E  _TX_TIMEOUT_DisableInt
       078E  TX_TIMEOUT_DisableInt
       0792  TX_TIMEOUT_Start
       0792  _TX_TIMEOUT_Start
       0796  _TX_TIMEOUT_Stop
       0796  TX_TIMEOUT_Stop
       079A  _TX_TIMEOUT_WritePeriod
       079A  TX_TIMEOUT_WritePeriod
       07A0  _TX_TIMEOUT_WriteCompareValue
       07A0  TX_TIMEOUT_WriteCompareValue
       07A6  wTX_TIMEOUT_ReadCompareValue
       07A6  TX_TIMEOUT_wReadCompareValue
       07A6  _wTX_TIMEOUT_ReadCompareValue
       07A6  _TX_TIMEOUT_wReadCompareValue
       07AC  _wTX_TIMEOUT_ReadCounter
       07AC  _TX_TIMEOUT_wReadTimerSaveCV
       07AC  _wTX_TIMEOUT_ReadTimerSaveCV
       07AC  wTX_TIMEOUT_ReadTimerSaveCV
       07AC  TX_TIMEOUT_wReadTimerSaveCV
       07AC  wTX_TIMEOUT_ReadCounter
       07E2  wTX_TIMEOUT_CaptureCounter
       07E2  _TX_TIMEOUT_wReadTimer
       07E2  TX_TIMEOUT_wReadTimer
       07E2  wTX_TIMEOUT_ReadTimer
       07E2  _wTX_TIMEOUT_ReadTimer
       07E2  _wTX_TIMEOUT_CaptureCounter
       07EA  _TX_REPEATER_ISR
       07EB  _TX_REPEATER_EnableInt
       07EB  TX_REPEATER_EnableInt
       07EF  TX_REPEATER_DisableInt
       07EF  _TX_REPEATER_DisableInt
       07F3  _TX_REPEATER_SetTxIntMode
       07F3  TX_REPEATER_SetTxIntMode
       0805  _TX_REPEATER_Start
       0805  TX_REPEATER_Start
       080A  _TX_REPEATER_Stop
       080A  TX_REPEATER_Stop
       080E  TX_REPEATER_SendData
       080E  _TX_REPEATER_SendData
       0811  TX_REPEATER_bReadTxStatus
       0811  _TX_REPEATER_bReadTxStatus
       0811  _bTX_REPEATER_ReadTxStatus
       0811  bTX_REPEATER_ReadTxStatus
       0824  TX_REPEATER_PutSHexByte
       0824  _TX_REPEATER_PutSHexByte
       0837  TX_REPEATER_PutSHexInt
       0837  _TX_REPEATER_PutSHexInt
       083E  _TX_REPEATER_PutChar
       083E  TX_REPEATER_PutChar
       0846  _TX_REPEATER_PutString
       0846  TX_REPEATER_PutString
       0858  _TX_REPEATER_Write
       0858  TX_REPEATER_Write
       0880  TX_REPEATER_CWrite
       0880  _TX_REPEATER_CWrite
       08A8  _TX_REPEATER_CPutString
       08A8  TX_REPEATER_CPutString
       08BA  _TX_REPEATER_PutCRLF
       08BA  TX_REPEATER_PutCRLF
       08C3  _TRANSMIT_ISR
       08C4  TRANSMIT_EnableInt
       08C4  _TRANSMIT_EnableInt
       08C8  _TRANSMIT_DisableInt
       08C8  TRANSMIT_DisableInt
       08CC  _TRANSMIT_SetTxIntMode
       08CC  TRANSMIT_SetTxIntMode
       08DE  TRANSMIT_Start
       08DE  _TRANSMIT_Start
       08E3  _TRANSMIT_Stop
       08E3  TRANSMIT_Stop
       08E7  _TRANSMIT_SendData
       08E7  TRANSMIT_SendData
       08EA  _TRANSMIT_bReadTxStatus
       08EA  TRANSMIT_bReadTxStatus
       08EA  bTRANSMIT_ReadTxStatus
       08EA  _bTRANSMIT_ReadTxStatus
       08FD  _TRANSMIT_PutSHexByte
       08FD  TRANSMIT_PutSHexByte
       0910  _TRANSMIT_PutSHexInt
       0910  TRANSMIT_PutSHexInt
       0917  _TRANSMIT_PutChar
       0917  TRANSMIT_PutChar
       091F  _TRANSMIT_PutString
       091F  TRANSMIT_PutString
       0931  _TRANSMIT_Write
       0931  TRANSMIT_Write
       0959  TRANSMIT_CWrite
       0959  _TRANSMIT_CWrite
       0981  TRANSMIT_CPutString
       0981  _TRANSMIT_CPutString
       0993  TRANSMIT_PutCRLF
       0993  _TRANSMIT_PutCRLF
       099C  _RX_TIMEOUT_EnableInt
       099C  RX_TIMEOUT_EnableInt
       09A0  _RX_TIMEOUT_DisableInt
       09A0  RX_TIMEOUT_DisableInt
       09A4  RX_TIMEOUT_Start
       09A4  _RX_TIMEOUT_Start
       09A8  _RX_TIMEOUT_Stop
       09A8  RX_TIMEOUT_Stop
       09AC  _RX_TIMEOUT_WritePeriod
       09AC  RX_TIMEOUT_WritePeriod
       09B2  _RX_TIMEOUT_WriteCompareValue
       09B2  RX_TIMEOUT_WriteCompareValue
       09B8  RX_TIMEOUT_wReadCompareValue
       09B8  _RX_TIMEOUT_wReadCompareValue
       09B8  _wRX_TIMEOUT_ReadCompareValue
       09B8  wRX_TIMEOUT_ReadCompareValue
       09BE  _wRX_TIMEOUT_ReadCounter
       09BE  RX_TIMEOUT_wReadTimerSaveCV
       09BE  wRX_TIMEOUT_ReadTimerSaveCV
       09BE  wRX_TIMEOUT_ReadCounter
       09BE  _RX_TIMEOUT_wReadTimerSaveCV
       09BE  _wRX_TIMEOUT_ReadTimerSaveCV
       09F4  RX_TIMEOUT_wReadTimer
       09F4  wRX_TIMEOUT_CaptureCounter
       09F4  _wRX_TIMEOUT_ReadTimer
       09F4  _RX_TIMEOUT_wReadTimer
       09F4  _wRX_TIMEOUT_CaptureCounter
       09F4  wRX_TIMEOUT_ReadTimer
       09FC  _RECEIVE_ISR
       0A70  RECEIVE_EnableInt
       0A70  _RECEIVE_EnableInt
       0A74  _RECEIVE_DisableInt
       0A74  RECEIVE_DisableInt
       0A78  _RECEIVE_Start
       0A78  RECEIVE_Start
       0A7D  _RECEIVE_Stop
       0A7D  RECEIVE_Stop
       0A81  bRECEIVE_ReadRxData
       0A81  _bRECEIVE_ReadRxData
       0A81  _RECEIVE_bReadRxData
       0A81  RECEIVE_bReadRxData
       0A84  _RECEIVE_bReadRxStatus
       0A84  RECEIVE_bReadRxStatus
       0A84  _bRECEIVE_ReadRxStatus
       0A84  bRECEIVE_ReadRxStatus
       0A87  _RECEIVE_cGetChar
       0A87  RECEIVE_cGetChar
       0A8F  RECEIVE_cReadChar
       0A8F  _RECEIVE_cReadChar
       0AA6  _RECEIVE_iReadChar
       0AA6  RECEIVE_iReadChar
       0ABC  RECEIVE_CmdReset
       0ABC  _RECEIVE_CmdReset
       0AD2  RECEIVE_bCmdCheck
       0AD2  _RECEIVE_bCmdCheck
       0ADA  _RECEIVE_bErrCheck
       0ADA  RECEIVE_bErrCheck
       0AE5  _RECEIVE_bCmdLength
       0AE5  RECEIVE_bCmdLength
       0AEB  _RECEIVE_szGetParam
       0AEB  RECEIVE_szGetParam
       0B4A  _RECEIVE_szGetRestOfParams
       0B4A  RECEIVE_szGetRestOfParams
       0B55  _COMP_SERIAL_TX_ISR
       0B56  _COMP_SERIAL_RX_ISR
       0BCA  COMP_SERIAL_EnableInt
       0BCA  _COMP_SERIAL_EnableInt
       0BD1  COMP_SERIAL_DisableInt
       0BD1  _COMP_SERIAL_DisableInt
       0BD8  _COMP_SERIAL_SetTxIntMode
       0BD8  COMP_SERIAL_SetTxIntMode
       0BEA  COMP_SERIAL_Start
       0BEA  _COMP_SERIAL_Start
       0BF3  _COMP_SERIAL_Stop
       0BF3  COMP_SERIAL_Stop
       0BFA  _COMP_SERIAL_SendData
       0BFA  COMP_SERIAL_SendData
       0BFD  _bCOMP_SERIAL_ReadTxStatus
       0BFD  bCOMP_SERIAL_ReadTxStatus
       0BFD  COMP_SERIAL_bReadTxStatus
       0BFD  _COMP_SERIAL_bReadTxStatus
       0C00  bCOMP_SERIAL_ReadRxData
       0C00  COMP_SERIAL_bReadRxData
       0C00  _COMP_SERIAL_bReadRxData
       0C00  _bCOMP_SERIAL_ReadRxData
       0C03  COMP_SERIAL_bReadRxStatus
       0C03  bCOMP_SERIAL_ReadRxStatus
       0C03  _COMP_SERIAL_bReadRxStatus
       0C03  _bCOMP_SERIAL_ReadRxStatus
       0C06  COMP_SERIAL_TxIntMode
       0C06  _COMP_SERIAL_TxIntMode
       0C19  COMP_SERIAL_IntCntl
       0C19  _COMP_SERIAL_IntCntl
       0C44  COMP_SERIAL_PutSHexByte
       0C44  _COMP_SERIAL_PutSHexByte
       0C57  _COMP_SERIAL_PutSHexInt
       0C57  COMP_SERIAL_PutSHexInt
       0C5E  _COMP_SERIAL_PutChar
       0C5E  COMP_SERIAL_PutChar
       0C66  _COMP_SERIAL_cGetChar
       0C66  COMP_SERIAL_cGetChar
       0C6E  COMP_SERIAL_cReadChar
       0C6E  _COMP_SERIAL_cReadChar
       0C85  COMP_SERIAL_iReadChar
       0C85  _COMP_SERIAL_iReadChar
       0C9B  _COMP_SERIAL_PutString
       0C9B  COMP_SERIAL_PutString
       0CAD  COMP_SERIAL_Write
       0CAD  _COMP_SERIAL_Write
       0CD5  _COMP_SERIAL_CWrite
       0CD5  COMP_SERIAL_CWrite
       0D01  _COMP_SERIAL_CPutString
       0D01  COMP_SERIAL_CPutString
       0D13  _COMP_SERIAL_PutCRLF
       0D13  COMP_SERIAL_PutCRLF
       0D1C  COMP_SERIAL_CmdReset
       0D1C  _COMP_SERIAL_CmdReset
       0D32  _COMP_SERIAL_bCmdCheck
       0D32  COMP_SERIAL_bCmdCheck
       0D3A  COMP_SERIAL_bErrCheck
       0D3A  _COMP_SERIAL_bErrCheck
       0D45  COMP_SERIAL_bCmdLength
       0D45  _COMP_SERIAL_bCmdLength
       0D4B  COMP_SERIAL_szGetParam
       0D4B  _COMP_SERIAL_szGetParam
       0DAA  COMP_SERIAL_szGetRestOfParams
       0DAA  _COMP_SERIAL_szGetRestOfParams
       0DB5  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0DB5   1218 =   4632. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0DB5  __text_start
       0DB5  _main
       0DD1  _pingModule
       0EC0  _assignID
       0FB6  _clearConfig
       10AF  _sayHello
       1105  _validTransmission
       11F9  _decodeTransmission
       174C  _servoInstruction
       17EF  _longServoInstruction
       18A0  _configToggle
       197A  _unloadAllConfigs
       1984  _unloadConfig
       19B3  _busListen
       1A0D  _initializeSlaves
       1B4B  _xmitWait
       1B6A  _TX_TIMEOUT_ISR
       1B80  _RX_TIMEOUT_ISR
       1B96  Dispatch_INTERRUPT_10
       1BC0  Dispatch_INTERRUPT_9
       1BDF  __divmod_16X16_16
       1C2D  __divmodu_16X16_16
       1C38  __i_div16_block_util
       1C71  _atoi
       1DB2  _itoa
       1F8B  _isdigit
       1FAC  _isspace
       1FDD  __xidata_end
       1FCD  __text_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0080   014E =    334. bytes (abs,ovr,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0080  __Start
       014B  bGetPowerSetting
       014B  _bGetPowerSetting

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   0000   0005 =      5. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  _RECEIVE_fStatus
       0000  RECEIVE_fStatus
       0001  RECEIVE_bRxCnt
       0001  _RECEIVE_bRxCnt
       0002  NO_SHADOW
       0002  _NO_SHADOW
       0002  ACTIVE_CONFIG_STATUS
       0003  COMP_SERIAL_fStatus
       0003  _COMP_SERIAL_fStatus
       0004  _COMP_SERIAL_bRxCnt
       0004  COMP_SERIAL_bRxCnt

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                 COMP_SERIAL_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0005  COMP_SERIAL_aRxBuffer
       0005  _COMP_SERIAL_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     RECEIVE_RAM   0000   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0045  _RECEIVE_aRxBuffer
       0045  RECEIVE_aRxBuffer

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   008F   0009 =      9. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       008F  __rZ
       0090  __rY
       0091  __rX
       0092  __r5
       0093  __r4
       0094  __r3
       0095  __r2
       0096  __r1
       0097  __r0

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0000   0013 =     19. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __data_start
       0085  _PARAM
       0098  _NUM_MODULES
       009A  _STATE
       009C  _TIMEOUT
       00A0  _COMMAND_DESTINATION
       00A1  _COMMAND_SOURCE
       00A2  _COMMAND_TYPE

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
./obj/main.o	[ main.c ]
<library>	[ tx_timeout.asm, tx_repeaterint.asm, tx_repeater.asm, transmitint.asm, transmit.asm, rx_timeout.asm ]
[ receiveint.asm, receive.asm, psocdynamicint.asm, psocconfigtbl.asm, psocconfig.asm, comp_serialint.asm ]
[ comp_serial.asm, rZ.asm, rY.asm, rX.asm, r5.asm, r4.asm, r3.asm, r2.asm, r1.asm, r0.asm, divmod_16x16_16_util.asm ]
[ divmodu_16x16_16_util.asm, i_div16_block_util.asm, atoi.s, itoa.s, isdigit.s, isspace.s, ctype.s ]


User Base Address Definitions

lit = 0x190
data = 0x0
SSCParmBlk:0x00F7.0x00FF
InterruptRAM = 0x0
virtual_registers = 0x8f
transmitter_config_RAM = 0xa3
pc_listener_RAM
receiver_config_RAM = 0xa3

