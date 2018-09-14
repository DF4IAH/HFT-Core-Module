/*
 * task_AX5243.h
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */

#ifndef TASK_AX5243_H_
#define TASK_AX5243_H_

#include "stm32l4xx_hal.h"
#include "main.h"


#if 0
/* NEW CODE */

#if 0
# define USART_SPI_AX                                         USARTC1
#else
# define SPI_AX                                               SPIC
#endif

#ifndef IOPORT_MODE_FALLING
# define IOPORT_MODE_FALLING                                  0x02
#endif
#endif


#define C_PR1200_CALL_LENGTH                                  6

#define C_AX_PRW_LENGTH                                       253
#define C_SPI_AX_BUFFER_LENGTH                                512
#define C_SPI_AX_RANGE_NOT_SET                                0x10


typedef enum SPI_AX_TRPT_STATE_ENUM {

  SPI_AX_TRPT_STATE_DISABLED                                  = 0x00,
  SPI_AX_TRPT_STATE_ENA_ADDR,
  SPI_AX_TRPT_STATE_ENA_DATA_WRITE,
  SPI_AX_TRPT_STATE_ENA_DATA_READ,
  SPI_AX_TRPT_STATE_COMPLETE,

  SPI_AX_TRPT_STATE_END                                       = 0x7f,
  SPI_AX_TRPT_STATE_ERROR                                     = 0xff,

} SPI_AX_TRPT_STATE_t;


typedef enum AX_SET_REGISTERS_MODULATION_ENUM {

  AX_SET_REGISTERS_MODULATION_NONE                            = 0x00,
  AX_SET_REGISTERS_MODULATION_NO_CHANGE                       = 0x01,
  AX_SET_REGISTERS_MODULATION_FSK                             = 0x11,
  AX_SET_REGISTERS_MODULATION_PR1200                          = 0x21,
  AX_SET_REGISTERS_MODULATION_POCSAG                          = 0x71,
  AX_SET_REGISTERS_MODULATION_ANALOG_FM                       = 0x81,
  AX_SET_REGISTERS_MODULATION_INVALIDATE                      = 0x100,

} AX_SET_REGISTERS_MODULATION_t;

typedef enum AX_SET_REGISTERS_VARIANT_ENUM {

  AX_SET_REGISTERS_VARIANT_NONE                               = 0x00,
  AX_SET_REGISTERS_VARIANT_NO_CHANGE                          = 0x01,
  AX_SET_REGISTERS_VARIANT_TX                                 = 0x11,
  AX_SET_REGISTERS_VARIANT_RX                                 = 0x21,
  AX_SET_REGISTERS_VARIANT_RX_WOR,
  AX_SET_REGISTERS_VARIANT_RX_CONT,
  AX_SET_REGISTERS_VARIANT_RX_CONT_SINGLEPARAMSET,
  AX_SET_REGISTERS_VARIANT_INVALIDATE                         = 0x100,

} AX_SET_REGISTERS_VARIANT_t;

typedef enum AX_SET_REGISTERS_POWERMODE_ENUM {

  AX_SET_REGISTERS_POWERMODE_POWERDOWN                        = 0x00,
  AX_SET_REGISTERS_POWERMODE_DEEPSLEEP                        = 0x01,
  AX_SET_REGISTERS_POWERMODE_STANDBY                          = 0x05,
  AX_SET_REGISTERS_POWERMODE_FIFO                             = 0x06,
  AX_SET_REGISTERS_POWERMODE_SYNTHRX                          = 0x08,
  AX_SET_REGISTERS_POWERMODE_FULLRX                           = 0x09,
  AX_SET_REGISTERS_POWERMODE_WOR                              = 0x0B,
  AX_SET_REGISTERS_POWERMODE_SYNTHTX                          = 0x0C,
  AX_SET_REGISTERS_POWERMODE_FULLTX                           = 0x0D,
  AX_SET_REGISTERS_POWERMODE_NONE                             = 0x100,

} AX_SET_REGISTERS_POWERMODE_t;

/* find AX_SET_TX_RX_MODE_t in main.h, also */
#ifndef DEFINED_AX_SET_TX_RX_MODE
typedef enum AX_SET_TX_RX_MODE_ENUM {

  AX_SET_TX_RX_MODE_OFF                                       = 0x00,
  AX_SET_TX_RX_MODE_APRS_TX                                   = 0x31,
  AX_SET_TX_RX_MODE_APRS_RX_WOR,
  AX_SET_TX_RX_MODE_APRS_RX_CONT,
  AX_SET_TX_RX_MODE_APRS_RX_CONT_SINGLEPARAMSET,
  AX_SET_TX_RX_MODE_POCSAG_TX                                 = 0x71,
  AX_SET_TX_RX_MODE_POCSAG_RX_WOR,
  AX_SET_TX_RX_MODE_POCSAG_RX_CONT,
  AX_SET_TX_RX_MODE_POCSAG_RX_CONT_SINGLEPARAMSET,

} AX_SET_TX_RX_MODE_t;
#define DEFINED_AX_SET_TX_RX_MODE
#endif

/* find AX_SET_MON_MODE_t in main.h, also */
#ifndef DEFINED_AX_SET_MON_MODE
typedef enum AX_SET_MON_MODE_ENUM {

  AX_SET_MON_MODE_OFF                                         = 0x00,

  AX_SET_MON_MODE_APRS_RX_WOR                                 = AX_SET_TX_RX_MODE_APRS_RX_WOR,
  AX_SET_MON_MODE_APRS_RX_CONT                                = AX_SET_TX_RX_MODE_APRS_RX_CONT,
  AX_SET_MON_MODE_APRS_RX_CONT_SINGLEPARAMSET                 = AX_SET_TX_RX_MODE_APRS_RX_CONT_SINGLEPARAMSET,

  AX_SET_MON_MODE_POCSAG_RX_WOR                               = AX_SET_TX_RX_MODE_POCSAG_RX_WOR,
  AX_SET_MON_MODE_POCSAG_RX_CONT                              = AX_SET_TX_RX_MODE_POCSAG_RX_CONT,
  AX_SET_MON_MODE_POCSAG_RX_CONT_SINGLEPARAMSET               = AX_SET_TX_RX_MODE_POCSAG_RX_CONT_SINGLEPARAMSET,

} AX_SET_MON_MODE_t;
#define DEFINED_AX_SET_MON_MODE
#endif


typedef enum AX_FIFO_CMD_ENUM {

  AX_FIFO_CMD_NOP                                             = 0x00,
  AX_FIFO_CMD_ASK_COHERENT                                    = 0x01,
  AX_FIFO_CMD_CLEAR_FIFO_ERROR                                = 0x02,
  AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS                       = 0x03,
  AX_FIFO_CMD_COMMIT                                          = 0x04,
  AX_FIFO_CMD_ROLLBACK                                        = 0x05

} AX_FIFO_CMD_t;

typedef enum AX_FIFO_DATA_CMD_ENUM {

  AX_FIFO_DATA_CMD_NOP_TX                                     = 0x00,
  AX_FIFO_DATA_CMD_RSSI_RX                                    = 0x31,
  AX_FIFO_DATA_CMD_TXCTRL_TX                                  = 0x3C,
  AX_FIFO_DATA_CMD_FREQOFFS_RX                                = 0x52,
  AX_FIFO_DATA_CMD_ANTRSSI2_RX                                = 0x55,
  AX_FIFO_DATA_CMD_REPEATDATA_TX                              = 0x62,
  AX_FIFO_DATA_CMD_TIMER_RX                                   = 0x70,
  AX_FIFO_DATA_CMD_RFFREQOFFS_RX                              = 0x73,
  AX_FIFO_DATA_CMD_DATARATE_RX                                = 0x74,
  AX_FIFO_DATA_CMD_ANTRSSI3_RX                                = 0x75,
  AX_FIFO_DATA_CMD_DATA_TX_RX                                 = 0xE1,
  AX_FIFO_DATA_CMD_TXPWR_TX                                   = 0xFD,

} AX_FIFO_DATA_CMD_t;

typedef enum AX_FIFO_DATA_FLAGS_TX_BF_ENUM {

  AX_FIFO_DATA_FLAGS_TX_PKTSTART                              = 0x01,
  AX_FIFO_DATA_FLAGS_TX_PKTEND                                = 0x02,
  AX_FIFO_DATA_FLAGS_TX_RESIDUE                               = 0x04,
  AX_FIFO_DATA_FLAGS_TX_NOCRC                                 = 0x08,
  AX_FIFO_DATA_FLAGS_TX_RAW                                   = 0x10,
  AX_FIFO_DATA_FLAGS_TX_UNENC                                 = 0x20,

} AX_FIFO_DATA_FLAGS_TX_BF_t;

typedef enum AX_FIFO_DATA_FLAGS_RX_BF_ENUM {

  AX_FIFO_DATA_FLAGS_RX_PKTSTART                              = 0x01,
  AX_FIFO_DATA_FLAGS_RX_PKTEND                                = 0x02,
  AX_FIFO_DATA_FLAGS_RX_RESIDUE                               = 0x04,
  AX_FIFO_DATA_FLAGS_RX_CRCFAIL                               = 0x08,
  AX_FIFO_DATA_FLAGS_RX_ADDRFAIL                              = 0x10,
  AX_FIFO_DATA_FLAGS_RX_SIZEFAIL                              = 0x20,
  AX_FIFO_DATA_FLAGS_RX_ABORT                                 = 0x40,

} AX_FIFO_DATA_FLAGS_RX_BF_t;

typedef enum AX_DECODER_POCSAG_ENUM {

  AX_DECODER_POCSAG__NONE                                     = 0x00,
  AX_DECODER_POCSAG__FLUSH,
  AX_DECODER_POCSAG__SYNC,
  AX_DECODER_POCSAG__IDLE,
  AX_DECODER_POCSAG__ADDRESS,
  AX_DECODER_POCSAG__DATA,

} AX_DECODER_POCSAG_t;


typedef enum AX_FIFO_RX_FSM_ENUM {

  AX_FIFO_RX_FSM__START                                       = 0x00,
  AX_FIFO_RX_FSM__STOP                                        = 0x0f,

  AX_FIFO_RX_FSM_TIMER_1                                      = 0x11,
  AX_FIFO_RX_FSM_TIMER_2,
  AX_FIFO_RX_FSM_TIMER_3,

  AX_FIFO_RX_FSM_RSSI_1                                       = 0x21,

  AX_FIFO_RX_FSM_ANTRSSI2_1                                   = 0x31,
  AX_FIFO_RX_FSM_ANTRSSI2_2,

  AX_FIFO_RX_FSM_ANTRSSI3_1                                   = 0x41,
  AX_FIFO_RX_FSM_ANTRSSI3_2,
  AX_FIFO_RX_FSM_ANTRSSI3_3,

  AX_FIFO_RX_FSM_RFFREQOFFS_1                                 = 0x51,
  AX_FIFO_RX_FSM_RFFREQOFFS_2,
  AX_FIFO_RX_FSM_RFFREQOFFS_3,

  AX_FIFO_RX_FSM_FREQOFFS_1                                   = 0x61,
  AX_FIFO_RX_FSM_FREQOFFS_2,

  AX_FIFO_RX_FSM_DATARATE_1                                   = 0x71,
  AX_FIFO_RX_FSM_DATARATE_2,
  AX_FIFO_RX_FSM_DATARATE_3,

  AX_FIFO_RX_FSM_DATA_1                                       = 0x81,
  AX_FIFO_RX_FSM_DATA_2,
  AX_FIFO_RX_FSM_DATA_3,

  AX_FIFO_RX_FSM__FAIL_STATE                                  = 0xf1,
  AX_FIFO_RX_FSM__FAIL_CMD                                    = 0xf2,

} AX_FIFO_RX_FSM_t;


typedef enum AX_POCSAG_CODES_ENUM {                                             // MSB   S A A A  A A A A     A A A A  A A A A     A A A F  F C C C     C C C C  C C C P   LSB - S: Addr/Data, A: Addr, F: Function bits, C: Check, P: even Parity

  AX_POCSAG_CODES_PREAMBLE                                    = 0xaaaaaaaaUL,   // MSB   1 0 1 0  1 0 1 0     1 0 1 0  1 0 1 0     1 0 1 0  1 0 1 0     1 0 1 0  1 0 1 0   LSB
  AX_POCSAG_CODES_SYNCWORD                                    = 0x7cd215d8UL,   // MSB   0 1 1 1  1 1 0 0     1 1 0 1  0 0 1 0     0 0 0 1  0 1 0 1     1 1 0 1  1 0 0 0   LSB  == HI-address: 0x1f348  concat  0b000           = 0xf9a40
  AX_POCSAG_CODES_IDLEWORD                                    = 0x7a89c197UL,   // MSB   0 1 1 1  1 0 1 0     1 0 0 0  1 0 0 1     1 1 0 0  0 0 0 1     1 0 0 1  0 1 1 1   LSB  == HI-address: 0x1eb27  concat  0b000 .. 0b111  = 0xf5938 .. 0xf593f

} AX_POCSAG_CODES_t;

typedef enum AX_POCSAG_CW1_ENUM {

  AX_POCSAG_CW1_IS_ADDR                                       = 0,
  AX_POCSAG_CW1_IS_MSG                                        = 1,

} AX_POCSAG_CW1_t;

typedef enum AX_POCSAG_CW2_ENUM {

  AX_POCSAG_CW2_MODE0_NUMERIC                                 = 0,
  AX_POCSAG_CW2_MODE1_TONE                                    = 1,
  AX_POCSAG_CW2_MODE2_ACTIVATION                              = 2,
  AX_POCSAG_CW2_MODE3_ALPHANUM                                = 3,

} AX_POCSAG_CW2_t;

typedef enum AX_POCSAG_SKYPER_ACTIVATION_ARY_ENUM {

  AX_POCSAG_SKYPER_ACTIVATION_ARY_SHIFT                       = 0,
  AX_POCSAG_SKYPER_ACTIVATION_ARY_MASK,
  AX_POCSAG_SKYPER_ACTIVATION_ARY_OFFSET,

} AX_POCSAG_SKYPER_ACTIVATION_ARY_t;

typedef enum AX_POCSAG_SKYPER_RIC_ENUM {

  AX_POCSAG_SKYPER_RIC_CLOCK                                  = 2504,
  AX_POCSAG_SKYPER_RIC_RUBRICS                                = 4512,
  AX_POCSAG_SKYPER_RIC_NEWS                                   = 4520,

} AX_POCSAG_SKYPER_RIC_t;

typedef enum AX_POCSAG_IS_SKYPER_SPECIAL_ENUM {

  AX_POCSAG_IS_SKYPER_SPECIAL_NO                              = 0,
  AX_POCSAG_IS_SKYPER_SPECIAL_RUBRIC,
  AX_POCSAG_IS_SKYPER_SPECIAL_NEWS,

} AX_POCSAG_IS_SKYPER_SPECIAL_t;


typedef struct AX_POCSAG_DECODER_DATA {

  uint8_t                   badDecode;                        // No successful decode (0 bit error and 1 bit error)

  uint8_t                   badParity;                        // Parity not even (0 bit error)
  uint8_t                   badCheck;                         // Check failed (0 bit error)

  uint8_t                   isAddr;                           // Decoded word is an address
  uint8_t                   isData;                           // Decoded word contains data
  uint32_t                  addrData;                         // Shared data or an address
  uint8_t                   functionBits;                     // Two additional bits for sub-address or Skyper signaling
  uint8_t                   invertedBit;                      // One bit error correction: 0-31 inverted bit. 32 means no correction involved

} AX_POCSAG_DECODER_DATA_t;

typedef struct AX_RX_FIFO_MEAS {

  uint32_t                  timer;

  int8_t                    rssi;                             // Sign reversal at 0x40 offset
  int16_t                   antRssi2;
  int32_t                   antRssi3;

  int32_t                   rfFrqOffs;
  int16_t                   frqOffs;

  uint32_t                  dataRate;

} AX_RX_FIFO_MEAS_t;



#if 0
/* ISR routines */

/* PORTC Pin3 - AX5243 IRQ */
ISR(PORTC_INT0_vect, ISR_BLOCK);

void spi_ax_ISR_setFlags(uint8_t flags);
void isr_spi_ax_fifo_readMessage(void);
#endif


# if 0
inline static uint8_t s_strGetHex(const char* str);
inline static uint8_t s_strGetDec(const char* str, int* o_val);
# endif


//AX_POCSAG_CW2_t ax_pocsag_analyze_msg_tgtFunc_get(const char* msg, uint16_t msgLen);
//uint8_t spi_ax_pocsag_calc_evenParity(uint32_t par);
//uint32_t spi_ax_pocsag_calc_checkAndParity(uint32_t codeword_in);
//uint8_t spi_ax_pocsag_getBcd(char c);
//char spi_ax_pocsag_getReversedNumChar(uint8_t nibble);
//char spi_ax_pocsag_getReversedAlphaChar(uint8_t reversedIn);
//uint32_t spi_ax_pocsag_get20Bits(const char* tgtMsg, uint16_t tgtMsgLen, AX_POCSAG_CW2_t tgtFunc, uint16_t msgBitIdx);
//char spi_ax_pocsag_getNumeric(const uint32_t* rcv20Bits, uint8_t rcv20BitsCnt, uint8_t msgNumIdx);
//char spi_ax_pocsag_getAlphanum(const uint32_t* rcv20Bits, uint8_t rcv20BitsCnt, uint8_t msgAlphaIdx);

//void spi_ax_pocsag_address_tone(uint8_t individual_RIC_address, uint32_t address, uint8_t fktBits);
//void spi_ax_pocsag_address_numeric(uint8_t individual_RIC_address, uint32_t address, uint8_t fktBits, uint32_t* dataAry, uint8_t dataCnt);
//void spi_ax_pocsag_address_alphanum(uint8_t individual_RIC_address, uint32_t address, uint8_t fktBits, uint32_t* dataAry, uint8_t dataCnt, AX_POCSAG_IS_SKYPER_SPECIAL_t skyperSpecial);
//void spi_ax_pocsag_address_skyper_activation(uint8_t individual_RIC_address, uint32_t address, uint8_t fktBits, uint32_t* dataAry, uint8_t dataCnt);

//uint16_t spi_ax_pocsag_skyper_RIC2ActivationString(char* outBuf, uint16_t outBufSize, uint32_t RIC);
//uint16_t spi_ax_pocsag_skyper_TimeString(char* outBuf, uint16_t outBufSize, struct calendar_date* calDat);
//uint16_t spi_ax_pocsag_skyper_RubricString_Encode(char* outBuf, uint16_t outBufSize, uint8_t rubricNumber, const char* rubricLabel, uint16_t rubricLabelLen);
//uint16_t spi_ax_pocsag_skyper_RubricString_Decode(char* outBuf, uint16_t outBufSize, const char* pocsagSkyperRubricMsg, uint16_t pocsagSkyperRubricMsgLen);
//uint16_t spi_ax_pocsag_skyper_NewsString_Encode(char* outBuf, uint16_t outBufSize, uint8_t rubricNumber, uint8_t newsNumber, const char* newsString, uint16_t newsStringLen);
//uint16_t spi_ax_pocsag_skyper_NewsString_Decode(char* outBuf, uint16_t outBufSize, const char* pocsagSkyperNewsMsg, uint16_t pocsagSkyperNewsMsgLen);

//void spi_ax_pocsag_wordDecoder(AX_POCSAG_DECODER_DATA_t* l_pocsagData, uint32_t pocsagWord, uint8_t pocsagWordCnt);
//void spi_ax_pocsag_messageDecoder(uint32_t address, uint8_t functionBits, uint32_t* dataAry, uint8_t dataCnt);

//status_code_t spi_ax_transport(uint8_t isProgMem, const char* packet);

//status_code_t spi_ax_sync2Powerdown(void);
//void spi_ax_setPower_dBm(float dBm);
//void spi_ax_setPwrMode(AX_SET_REGISTERS_POWERMODE_t powerState);
//void spi_ax_setRegisters(uint8_t doReset, AX_SET_REGISTERS_MODULATION_t modulation, AX_SET_REGISTERS_VARIANT_t variant, AX_SET_REGISTERS_POWERMODE_t powerState);

//uint32_t spi_ax_calcFrequency_Mhz2Regs(float f_mhz);
//float spi_ax_calcFrequency_Regs2MHz(uint32_t vco_regval);
//void spi_ax_setFrequency2Regs(uint8_t chan, uint8_t isFreqB);
//void spi_ax_doRanging(void);
uint8_t spi_ax_vco_select(uint32_t reg_freq, uint8_t force);
uint8_t spi_ax_selectVcoFreq(uint8_t isFreqB);
void spi_ax_util_FIFO_waitFree(uint8_t neededSpace);
void spi_ax_setRxMode_by_MonMode(void);

void spi_ax_initRegisters_FSK(void);
void spi_ax_initRegisters_FSK_Tx(void);
void spi_ax_initRegisters_FSK_Rx(void);
void spi_ax_init_FSK_Tx(void);
void spi_ax_init_FSK_Rx(void);

void spi_ax_initRegisters_PR1200(void);
void spi_ax_initRegisters_PR1200_Tx(void);
void spi_ax_initRegisters_PR1200_Rx(void);
void spi_ax_initRegisters_PR1200_Rx_WoR(void);
void spi_ax_initRegisters_PR1200_Rx_cont(void);
void spi_ax_initRegisters_PR1200_Rx_cont_SingleParamSet(void);
void spi_ax_init_PR1200_Tx(void);
void spi_ax_run_PR1200_Tx_FIFO_APRS(const char addrAry[][C_PR1200_CALL_LENGTH], const uint8_t* ssidAry, uint8_t addrCnt, const char* aprsMsg, uint8_t aprsMsgLen);
void spi_ax_util_PR1200_Tx_FIFO_Flags(uint8_t count);
void spi_ax_util_PR1200_Tx_FIFO_AddressField(const char addrAry[][C_PR1200_CALL_LENGTH], const uint8_t* ssidAry, uint8_t addrCnt);
void spi_ax_util_PR1200_Tx_FIFO_InformationField(const char* aprsMsg, uint8_t aprsMsgLen);
void spi_ax_init_PR1200_Rx(AX_SET_REGISTERS_POWERMODE_t powerMode);

void spi_ax_initRegisters_POCSAG(void);
void spi_ax_initRegisters_POCSAG_Tx(void);
void spi_ax_initRegisters_POCSAG_Rx(void);
void spi_ax_initRegisters_POCSAG_Rx_WoR(void);
void spi_ax_initRegisters_POCSAG_Rx_cont(void);
void spi_ax_init_POCSAG_Tx(void);
void spi_ax_run_POCSAG_Tx_FIFO_Msg(uint32_t pocsagTgtRIC, AX_POCSAG_CW2_t pocsagTgtFunc, const char* pocsagTgtMsg, uint8_t pocsagTgtMsgLen);
void spi_ax_init_POCSAG_Rx(AX_SET_REGISTERS_POWERMODE_t powerMode);
void spi_ax_run_POCSAG_Tx_single_Msg(const char msgBuf, uint16_t msgBufLen);
void spi_ax_util_POCSAG_Tx_FIFO_Preamble(void);
int8_t spi_ax_util_POCSAG_Tx_FIFO_Batches(uint32_t pocsagTargetRIC, AX_POCSAG_CW2_t tgtFunc, const char* pocsagMsg, uint8_t pocsagMsgLen);

void spi_ax_initRegisters_AnlogFM(void);
void spi_ax_initRegisters_AnlogFM_Tx(void);
void spi_ax_initRegisters_AnlogFM_Rx(void);
void spi_ax_init_AnalogFM_Tx(void);
void spi_ax_init_AnalogFM_Rx(void);

void spi_ax_setTxRxMode(AX_SET_TX_RX_MODE_t mode);
uint8_t spi_ax_doProcess_RX_messages(const uint8_t* buf, uint8_t msgLen);
void spi_ax_Rx_FIFO_DataProcessor(AX_SET_MON_MODE_t monMode, const uint8_t* dataBuf, uint16_t dataLen);

//void init_spi_ax5243(void);

//void task_spi_ax(void);


/* Debugging */
#if 0
void spi_ax_test_monitor_levels(void);
void spi_ax_test_Rx_FIFO(void);

void spi_ax_test_Analog_FM_Tx(void);
void spi_ax_test_Analog_FM_Rx(void);

void spi_ax_test_PR1200_Tx(void);
void spi_ax_test_PR1200_Tx_FIFO_Lev2_minimal_AddressField(void);
void spi_ax_test_PR1200_Tx_FIFO_Lev2_minimal(void);
void spi_ax_test_PR1200_Rx(void);

void spi_ax_test_POCSAG(void);
void spi_ax_test_POCSAG_Tx(void);
void spi_ax_test_POCSAG_Rx(void);
#endif


/*
 * ITU Recommendations:             @see https://www.itu.int/dms_pubrec/itu-r/rec/m/R-REC-M.584-2-199711-I!!PDF-E.pdf
 *
 * Technical details for POCSAG receiption:   @see http://akafunk.faveve.uni-stuttgart.de/pocsag/
 * QRG = 439.9875 MHz (12.5 kHz channel spacing
 * Modulation = FSK with  +/- 4.0 kHz
 * '0' = Higher frequency
 * '1' = Lower  frequency
 *
 * Message:                   @see http://www.qsl.net/db0avh/code.html
 *
 * Bit No.:    Function:          Remarks:
 *
 * 1           Codeword explanation     0 = Address, 1 = Message
 * 2-21        Address (2-19)       Mode (20-21) / Message (2-21)
 * 22-31       Checksum
 * 32          Parity (even)
 *
 * Preamble-SW-CWCW-CWCW-CWCW-CWCW-CWCW-CWCW-CWCW-CWCW-SW-CWCW-CWCW-CWCW ...
 *
 * Preamble >= 288 x '10'  equals to 18x 0xAAAAAAAA
 *
 * Checksum:  Each codeword contains 21 information bits. They are the coefficients of a polynomial (x^30 .. X^10). The remaining coefficients of the polynomial are being zeroed.
 *       That polynomial has to be divided (modulo 2) with the generator polynomial (x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + x^0).
 *       The checksum bits are defined to be the resulting bits from the remaining coefficients (x^9 .. x^0) after the division.
 *
 * Characters & Numbers:
 *
 *                                          Character table
 *
 *                                          Numeric      Alpha
 *                                                    7   0   0   0 0 1 1 1 1
 *                        Bit                         6   0   0   1 1 0 0 1 1
 *                        4 3 2 1                     5   0   1   0 1 0 1 0 1
 *
 *                        0 0 0 0              0         NUL DLE ' '0 ß P ' p
 *                        0 0 0 1              1         SOH  DC  ! 1 A Q a q
 *                        0 0 1 0              2         STX  DC  " 2 B R b r
 *                        0 0 1 1              3         ETX  DC  # 3 C S c s
 *                        0 1 0 0              4         EOT  DC  $ 4 D T d t
 *                        0 1 0 1              5         ENQ NAK  % 5 E U e u
 *                        0 1 1 0              6         ACK SYN  & 6 F V f v
 *                        0 1 1 1              7         BEL ETB  ` 7 G W g w
 *                        1 0 0 0              8         BS  CAN  ( 8 H X h x
 *                        1 0 0 1              9         HT  EM   ) 9 I Y i y
 *                        1 0 1 0          - SPARE -     LF  SUB  * : J Z j z
 *                        1 0 1 1              U         VT  ESC  + ; K Ä k ä
 *                        1 1 0 0             ' '        FF  FS   , < L Ö l ö
 *                        1 1 0 1              -         CR  GS   - = M Ü m ü
 *                        1 1 1 0              ]         SO  RS   . > N ^ n ß
 *                        1 1 1 1              [         SI  US   / ? O _ O DEL
 */

 /* See also: https://www.codeproject.com/Articles/13189/POCSAG-Encoder and  http://www.codeproject.com/samples/bch2131.asp */


void ax5243TaskInit(void);
void ax5243TaskLoop(void);

#endif /* TASK_AX5243_H_ */
