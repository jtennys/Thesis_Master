  ROM 49% full. 7885 bytes used (does not include absolute areas).
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
                     psoc_config   05BB   01D8 =    472. bytes (rel,con,rom)

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
       06F3  UnloadConfig_receiver_config
       06F3  _UnloadConfig_receiver_config
       072E  UnloadConfig_Total
       072E  _UnloadConfig_Total
       0793  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   0793   062F =   1583. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0793  _TX_TIMEOUT_EnableInt
       0793  TX_TIMEOUT_EnableInt
       0797  _TX_TIMEOUT_DisableInt
       0797  TX_TIMEOUT_DisableInt
       079B  TX_TIMEOUT_Start
       079B  _TX_TIMEOUT_Start
       079F  _TX_TIMEOUT_Stop
       079F  TX_TIMEOUT_Stop
       07A3  _TX_TIMEOUT_WritePeriod
       07A3  TX_TIMEOUT_WritePeriod
       07A9  _TX_TIMEOUT_WriteCompareValue
       07A9  TX_TIMEOUT_WriteCompareValue
       07AF  wTX_TIMEOUT_ReadCompareValue
       07AF  TX_TIMEOUT_wReadCompareValue
       07AF  _wTX_TIMEOUT_ReadCompareValue
       07AF  _TX_TIMEOUT_wReadCompareValue
       07B5  _wTX_TIMEOUT_ReadCounter
       07B5  _TX_TIMEOUT_wReadTimerSaveCV
       07B5  _wTX_TIMEOUT_ReadTimerSaveCV
       07B5  wTX_TIMEOUT_ReadTimerSaveCV
       07B5  TX_TIMEOUT_wReadTimerSaveCV
       07B5  wTX_TIMEOUT_ReadCounter
       07EB  wTX_TIMEOUT_CaptureCounter
       07EB  _TX_TIMEOUT_wReadTimer
       07EB  TX_TIMEOUT_wReadTimer
       07EB  wTX_TIMEOUT_ReadTimer
       07EB  _wTX_TIMEOUT_ReadTimer
       07EB  _wTX_TIMEOUT_CaptureCounter
       07F3  _TX_REPEATER_ISR
       07F4  _TX_REPEATER_EnableInt
       07F4  TX_REPEATER_EnableInt
       07F8  TX_REPEATER_DisableInt
       07F8  _TX_REPEATER_DisableInt
       07FC  _TX_REPEATER_SetTxIntMode
       07FC  TX_REPEATER_SetTxIntMode
       080E  _TX_REPEATER_Start
       080E  TX_REPEATER_Start
       0813  _TX_REPEATER_Stop
       0813  TX_REPEATER_Stop
       0817  TX_REPEATER_SendData
       0817  _TX_REPEATER_SendData
       081A  TX_REPEATER_bReadTxStatus
       081A  _TX_REPEATER_bReadTxStatus
       081A  _bTX_REPEATER_ReadTxStatus
       081A  bTX_REPEATER_ReadTxStatus
       082D  TX_REPEATER_PutSHexByte
       082D  _TX_REPEATER_PutSHexByte
       0840  TX_REPEATER_PutSHexInt
       0840  _TX_REPEATER_PutSHexInt
       0847  _TX_REPEATER_PutChar
       0847  TX_REPEATER_PutChar
       084F  _TX_REPEATER_PutString
       084F  TX_REPEATER_PutString
       0861  _TX_REPEATER_Write
       0861  TX_REPEATER_Write
       0889  TX_REPEATER_CWrite
       0889  _TX_REPEATER_CWrite
       08B1  _TX_REPEATER_CPutString
       08B1  TX_REPEATER_CPutString
       08C3  _TX_REPEATER_PutCRLF
       08C3  TX_REPEATER_PutCRLF
       08CC  _TRANSMIT_ISR
       08CD  TRANSMIT_EnableInt
       08CD  _TRANSMIT_EnableInt
       08D1  _TRANSMIT_DisableInt
       08D1  TRANSMIT_DisableInt
       08D5  _TRANSMIT_SetTxIntMode
       08D5  TRANSMIT_SetTxIntMode
       08E7  TRANSMIT_Start
       08E7  _TRANSMIT_Start
       08EC  _TRANSMIT_Stop
       08EC  TRANSMIT_Stop
       08F0  _TRANSMIT_SendData
       08F0  TRANSMIT_SendData
       08F3  _TRANSMIT_bReadTxStatus
       08F3  TRANSMIT_bReadTxStatus
       08F3  bTRANSMIT_ReadTxStatus
       08F3  _bTRANSMIT_ReadTxStatus
       0906  _TRANSMIT_PutSHexByte
       0906  TRANSMIT_PutSHexByte
       0919  _TRANSMIT_PutSHexInt
       0919  TRANSMIT_PutSHexInt
       0920  _TRANSMIT_PutChar
       0920  TRANSMIT_PutChar
       0928  _TRANSMIT_PutString
       0928  TRANSMIT_PutString
       093A  _TRANSMIT_Write
       093A  TRANSMIT_Write
       0962  TRANSMIT_CWrite
       0962  _TRANSMIT_CWrite
       098A  TRANSMIT_CPutString
       098A  _TRANSMIT_CPutString
       099C  TRANSMIT_PutCRLF
       099C  _TRANSMIT_PutCRLF
       09A5  _RX_TIMEOUT_EnableInt
       09A5  RX_TIMEOUT_EnableInt
       09A9  _RX_TIMEOUT_DisableInt
       09A9  RX_TIMEOUT_DisableInt
       09AD  RX_TIMEOUT_Start
       09AD  _RX_TIMEOUT_Start
       09B1  _RX_TIMEOUT_Stop
       09B1  RX_TIMEOUT_Stop
       09B5  _RX_TIMEOUT_WritePeriod
       09B5  RX_TIMEOUT_WritePeriod
       09BB  _RX_TIMEOUT_WriteCompareValue
       09BB  RX_TIMEOUT_WriteCompareValue
       09C1  RX_TIMEOUT_wReadCompareValue
       09C1  _RX_TIMEOUT_wReadCompareValue
       09C1  _wRX_TIMEOUT_ReadCompareValue
       09C1  wRX_TIMEOUT_ReadCompareValue
       09C7  _wRX_TIMEOUT_ReadCounter
       09C7  RX_TIMEOUT_wReadTimerSaveCV
       09C7  wRX_TIMEOUT_ReadTimerSaveCV
       09C7  wRX_TIMEOUT_ReadCounter
       09C7  _RX_TIMEOUT_wReadTimerSaveCV
       09C7  _wRX_TIMEOUT_ReadTimerSaveCV
       09FD  RX_TIMEOUT_wReadTimer
       09FD  wRX_TIMEOUT_CaptureCounter
       09FD  _wRX_TIMEOUT_ReadTimer
       09FD  _RX_TIMEOUT_wReadTimer
       09FD  _wRX_TIMEOUT_CaptureCounter
       09FD  wRX_TIMEOUT_ReadTimer
       0A05  _RECEIVE_ISR
       0A7D  RECEIVE_EnableInt
       0A7D  _RECEIVE_EnableInt
       0A81  _RECEIVE_DisableInt
       0A81  RECEIVE_DisableInt
       0A85  _RECEIVE_Start
       0A85  RECEIVE_Start
       0A8A  _RECEIVE_Stop
       0A8A  RECEIVE_Stop
       0A8E  bRECEIVE_ReadRxData
       0A8E  _bRECEIVE_ReadRxData
       0A8E  _RECEIVE_bReadRxData
       0A8E  RECEIVE_bReadRxData
       0A91  _RECEIVE_bReadRxStatus
       0A91  RECEIVE_bReadRxStatus
       0A91  _bRECEIVE_ReadRxStatus
       0A91  bRECEIVE_ReadRxStatus
       0A94  _RECEIVE_cGetChar
       0A94  RECEIVE_cGetChar
       0A9C  RECEIVE_cReadChar
       0A9C  _RECEIVE_cReadChar
       0AB3  _RECEIVE_iReadChar
       0AB3  RECEIVE_iReadChar
       0AC9  RECEIVE_CmdReset
       0AC9  _RECEIVE_CmdReset
       0ADF  RECEIVE_bCmdCheck
       0ADF  _RECEIVE_bCmdCheck
       0AE7  _RECEIVE_bErrCheck
       0AE7  RECEIVE_bErrCheck
       0AF2  _RECEIVE_bCmdLength
       0AF2  RECEIVE_bCmdLength
       0AF8  _RECEIVE_szGetParam
       0AF8  RECEIVE_szGetParam
       0B57  _RECEIVE_szGetRestOfParams
       0B57  RECEIVE_szGetRestOfParams
       0B62  _COMP_SERIAL_TX_ISR
       0B63  _COMP_SERIAL_RX_ISR
       0BD7  COMP_SERIAL_EnableInt
       0BD7  _COMP_SERIAL_EnableInt
       0BDE  COMP_SERIAL_DisableInt
       0BDE  _COMP_SERIAL_DisableInt
       0BE5  _COMP_SERIAL_SetTxIntMode
       0BE5  COMP_SERIAL_SetTxIntMode
       0BF7  COMP_SERIAL_Start
       0BF7  _COMP_SERIAL_Start
       0C00  _COMP_SERIAL_Stop
       0C00  COMP_SERIAL_Stop
       0C07  _COMP_SERIAL_SendData
       0C07  COMP_SERIAL_SendData
       0C0A  _bCOMP_SERIAL_ReadTxStatus
       0C0A  bCOMP_SERIAL_ReadTxStatus
       0C0A  COMP_SERIAL_bReadTxStatus
       0C0A  _COMP_SERIAL_bReadTxStatus
       0C0D  bCOMP_SERIAL_ReadRxData
       0C0D  COMP_SERIAL_bReadRxData
       0C0D  _COMP_SERIAL_bReadRxData
       0C0D  _bCOMP_SERIAL_ReadRxData
       0C10  COMP_SERIAL_bReadRxStatus
       0C10  bCOMP_SERIAL_ReadRxStatus
       0C10  _COMP_SERIAL_bReadRxStatus
       0C10  _bCOMP_SERIAL_ReadRxStatus
       0C13  COMP_SERIAL_TxIntMode
       0C13  _COMP_SERIAL_TxIntMode
       0C26  COMP_SERIAL_IntCntl
       0C26  _COMP_SERIAL_IntCntl
       0C51  COMP_SERIAL_PutSHexByte
       0C51  _COMP_SERIAL_PutSHexByte
       0C64  _COMP_SERIAL_PutSHexInt
       0C64  COMP_SERIAL_PutSHexInt
       0C6B  _COMP_SERIAL_PutChar
       0C6B  COMP_SERIAL_PutChar
       0C73  _COMP_SERIAL_cGetChar
       0C73  COMP_SERIAL_cGetChar
       0C7B  COMP_SERIAL_cReadChar
       0C7B  _COMP_SERIAL_cReadChar
       0C92  COMP_SERIAL_iReadChar
       0C92  _COMP_SERIAL_iReadChar
       0CA8  _COMP_SERIAL_PutString
       0CA8  COMP_SERIAL_PutString
       0CBA  COMP_SERIAL_Write
       0CBA  _COMP_SERIAL_Write
       0CE2  _COMP_SERIAL_CWrite
       0CE2  COMP_SERIAL_CWrite
       0D0E  _COMP_SERIAL_CPutString
       0D0E  COMP_SERIAL_CPutString
       0D20  _COMP_SERIAL_PutCRLF
       0D20  COMP_SERIAL_PutCRLF
       0D29  COMP_SERIAL_CmdReset
       0D29  _COMP_SERIAL_CmdReset
       0D3F  _COMP_SERIAL_bCmdCheck
       0D3F  COMP_SERIAL_bCmdCheck
       0D47  COMP_SERIAL_bErrCheck
       0D47  _COMP_SERIAL_bErrCheck
       0D52  COMP_SERIAL_bCmdLength
       0D52  _COMP_SERIAL_bCmdLength
       0D58  COMP_SERIAL_szGetParam
       0D58  _COMP_SERIAL_szGetParam
       0DB7  COMP_SERIAL_szGetRestOfParams
       0DB7  _COMP_SERIAL_szGetRestOfParams
       0DC2  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0DC2   129B =   4763. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0DC2  __text_start
       0DC2  _main
       0DF9  _pingModule
       0EE8  _assignID
       0FDE  _clearConfig
       10D7  _sayHello
       112E  _validTransmission
       1222  _decodeTransmission
       1798  _servoInstruction
       183B  _longServoInstruction
       18EC  _configToggle
       19CB  _unloadAllConfigs
       19D5  _unloadConfig
       1A04  _busListen
       1A5E  _initializeSlaves
       1BDB  _xmitWait
       1BFA  _TX_TIMEOUT_ISR
       1C10  _RX_TIMEOUT_ISR
       1C26  Dispatch_INTERRUPT_10
       1C50  Dispatch_INTERRUPT_9
       1C6F  __divmod_16X16_16
       1CBD  __divmodu_16X16_16
       1CC8  __i_div16_block_util
       1D01  _atoi
       1E42  _itoa
       201B  _isdigit
       203C  _isspace
       206D  __xidata_end
       205D  __text_end

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

