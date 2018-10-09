/*
 * lib_LoRaWAN.h
 *
 *  Created on: 16.09.2018
 *      Author: DF4IAH
 */

#ifndef LIB_LORAWAN_H_
#define LIB_LORAWAN_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"


#define LoRaWAN_MsgLenMax             51
#define LoRaWAN_FRMPayloadMax         48


/* UTC --> GPS  leap seconds adjustment: +18 s as of 2018-07-01 */
#define GPS_UTC_LEAP_SECS             (+18)


/* Default settings */
#define  LORAWAN_CONFPACKS_DEFAULT    1
#define  LORAWAN_ADR_ENABLED_DEFAULT  0



/* Bit-mask for the loRaWANEventGroup */
typedef enum Lora_EGW_BM {
  Lora_EGW__QUEUE_IN                  = 0x00000001UL,
  Lora_EGW__DO_INIT                   = 0x00000010UL,
  Lora_EGW__DO_LINKCHECKREQ           = 0x00000100UL,
  Lora_EGW__DO_DEVICETIMEREQ          = 0x00000200UL,
  Lora_EGW__EXTI_DIO0                 = 0x00001000UL,
} Lora_EGW_BM_t;

/* Command types for the loraInQueue */
typedef enum LoraInQueueCmds {
  LoraInQueueCmds__NOP                = 0,
  LoraInQueueCmds__Init,
  LoraInQueueCmds__LinkCheckReq,
  LoraInQueueCmds__DeviceTimeReq,
  LoraInQueueCmds__ConfirmedPackets,
  LoraInQueueCmds__ADRset,
  LoraInQueueCmds__PwrRedDb,
  LoraInQueueCmds__DRset,
  LoraInQueueCmds__TrackMeApplUp,
  LoraInQueueCmds__LoRaBareFrequency,
  LoraInQueueCmds__LoRaBareRxEnable,
  LoraInQueueCmds__LoRaBareSend,
} LoraInQueueCmds_t;

/* Command types for the loraOutQueue */
typedef enum LoraOutQueueCmds {
  LoraOutQueueCmds__NOP                = 0,
  LoraOutQueueCmds__Connected,
} LoraOutQueueCmds_t;


typedef enum DataRates {
  DR0_SF12_125kHz_LoRa                = 0,
  DR1_SF11_125kHz_LoRa,
  DR2_SF10_125kHz_LoRa,
  DR3_SF9_125kHz_LoRa,
  DR4_SF8_125kHz_LoRa,
  DR5_SF7_125kHz_LoRa,
  DR6_SF7_250kHz_LoRa,
  DR7_50kHz_FSK,                                                                                // nRF905 "OGN Tracking Protocol" compatible?
} DataRates_t;


/* FSM states of the loRaWANLoRaWANTaskLoop */
typedef enum Fsm_States {
  Fsm_NOP                             = 0,

  Fsm_RX1                             = 0x01,
  Fsm_RX2,
  Fsm_JoinRequestRX1,
  Fsm_JoinRequestRX2,

  Fsm_TX                              = 0x09,

  Fsm_MAC_Decoder                     = 0x10,

  Fsm_MAC_Proc,

  Fsm_MAC_JoinRequest,
  Fsm_MAC_JoinAccept,

  Fsm_MAC_LinkCheckReq,
  Fsm_MAC_LinkCheckAns,

  Fsm_MAC_LinkADRReq,
  Fsm_MAC_LinkADRAns,

  Fsm_MAC_DutyCycleReq,
  Fsm_MAC_DutyCycleAns,

  Fsm_MAC_RXParamSetupReq,
  Fsm_MAC_RXParamSetupAns,

  Fsm_MAC_DevStatusReq,
  Fsm_MAC_DevStatusAns,

  Fsm_MAC_NewChannelReq,
  Fsm_MAC_NewChannelAns,

  Fsm_MAC_RXTimingSetupReq,
  Fsm_MAC_RXTimingSetupAns,

  Fsm_MAC_TxParamSetupReq,
  Fsm_MAC_TxParamSetupAns,

  Fsm_MAC_DlChannelReq,
  Fsm_MAC_DlChannelAns,

  Fsm_MAC_DeviceTimeReq,
  Fsm_MAC_DeviceTimeAns,


  Fsm_Bare_SetTrxMode                 = 0x80,
} Fsm_States_t;



/* --- LoRaBare --- */

typedef struct LoRaBareCtx {
  /* Transceiver settings */
  uint8_t                             sxMode;                                                   // Mode of the SX transceiver chip
  uint8_t                             spreadingFactor;                                          // Spreading factor to be used
  uint8_t                             pwrred;                                                   // Power reduction in dB from +14dBm
  float                               frequencyMHz;                                             // Frequency in use for current TX or RX
} LoRaBareCtx_t;



/* --- LoRaWAN --- */

typedef enum LoRaWAN_CalcMIC_JOINREQUEST {
  MIC_JOINREQUEST                     = 0x01,
  MIC_DATAMESSAGE                     = 0x11,
} LoRaWAN_CalcMIC_VARIANT_t;


typedef enum CurrentWindow {
  CurWin_none                         = 0,
  CurWin_RXTX1,
  CurWin_RXTX2
} CurrentWindow_t;

/* LoRaWAN RX windows */
typedef enum LoRaWAN_RX_windows {
  LORAWAN_BALANCING_AT_MOST_MS        =  250,
  LORAWAN_FRQ_JUMP_PREPARE_MS         =  100,                                                   // Last good value:  100
  LORAWAN_RX_PREPARE_MS               =  50,                                                    // Last good value:   50
  LORAWAN_EU868_MAX_TX_DURATION_MS    = 1990,                                                   // EU-868 M=59 / N=51 @ 250 bits/sec

  LORAWAN_EU868_DELAY1_MS             = 1000,
  LORAWAN_EU868_DELAY2_MS             = 2000,

  LORAWAN_EU868_JOIN_ACCEPT_DELAY1_MS = 5000,
  LORAWAN_EU868_JOIN_ACCEPT_DELAY2_MS = 6000,
} LoRaWAN_RX_windows_t;


/* MHDR */
typedef enum LoRaWAN_MTypeBF {
  JoinRequest                         = 0,
  JoinAccept                          = 1,
  UnconfDataUp                        = 2,
  UnconfDataDn                        = 3,
  ConfDataUp                          = 4,
  ConfDataDn                          = 5,
  RejoinReq                           = 6,
  Proprietary                         = 7
} LoRaWAN_MTypeBF_t;

typedef enum LoRaWAN_MajorBF {
  LoRaWAN_R1                          = 0,
} LoRaWAN_MajorBF_t;

#define LoRaWAN_MHDR_MType_SHIFT      5
#define LoRaWAN_MHDR_Major_SHIFT      0
#define MHDR_SIZE                     (sizeof(uint8_t))

typedef enum LoRaWANVersion {
  LoRaWANVersion_10                   = 0,
  LoRaWANVersion_11                   = 1,
} LoRaWANVersion_t;


/* FCtl */
#define LoRaWAN_FCtl_ADR_SHIFT        7
#define LoRaWAN_FCtl_ADRACKReq_SHIFT  6
#define LoRaWAN_FCtl_ACK_SHIFT        5
#define LoRaWAN_FCtl_ClassB_SHIFT     4
#define LoRaWAN_FCtl_FPending_SHIFT   4
#define LoRaWAN_FCtl_FOptsLen_SHIFT   0


/* FPort */
#define FPort_TrackMeAppl_Default     1


/* LoRaWAN direction of transmission - Up: device --> gateway / Dn: device <-- gateway */
typedef enum LoRaWANctxDir {
  Up                                  = 0,
  Dn                                  = 1
} LoRaWANctxDir_t;


typedef enum LoRaWANMAC_CID {
  ResetInd_UP                         = 0x01,                                                   // ABP devices only, LW 1.1
  ResetConf_DN                        = 0x01,                                                   // ABP devices only, LW 1.1

  LinkCheckReq_UP                     = 0x02,
  LinkCheckAns_DN                     = 0x02,

  LinkADRReq_DN                       = 0x03,
  LinkADRAns_UP                       = 0x03,

  DutyCycleReq_DN                     = 0x04,
  DutyCycleAns_UP                     = 0x04,

  RXParamSetupReq_DN                  = 0x05,
  RXParamSetupAns_UP                  = 0x05,

  DevStatusReq_DN                     = 0x06,
  DevStatusAns_UP                     = 0x06,

  NewChannelReq_DN                    = 0x07,
  NewChannelAns_UP                    = 0x07,

  RXTimingSetupReq_DN                 = 0x08,
  RXTimingSetupAns_UP                 = 0x08,

  TxParamSetupReq_DN                  = 0x09,
  TxParamSetupAns_UP                  = 0x09,

  DlChannelReq_DN                     = 0x0A,
  DlChannelAns_UP                     = 0x0A,

  RekeyInd_UP                         = 0x0B,
  RekeyConf_DN                        = 0x0B,

  ADRParamSetupReq_DN                 = 0x0C,
  ADRParamSetupAns_UP                 = 0x0C,

  DeviceTimeReq_UP                    = 0x0D,
  DeviceTimeAns_DN                    = 0x0D,

  ForceRejoinReq_DN                   = 0x0E,

  RejoinParamSetupReq_DN              = 0x0F,
  RejoinParamSetupAns_UP              = 0x0F,
} LoRaWANMAC_CID_t;

typedef enum ChMaskCntl {
  ChMaskCntl__appliesTo_1to16         = 0,
  ChMaskCntl__allChannelsON           = 6,
} ChMaskCntl_t;


/* MIC calculation structures */
typedef struct FRMPayloadBlockA_Up {
  uint8_t                             variant;
  uint8_t                             _noUse[4];
  uint8_t                             Dir;
  uint8_t                             DevAddr[4];
  uint8_t                             FCntUp[4];
  uint8_t                             _pad;
  uint8_t                             idx;
} FRMPayloadBlockA_Up_t;

#ifdef LORAWAN_1V02
typedef struct FRMPayloadBlockA_Dn {
  uint8_t                             variant;
  uint8_t                             _noUse[4];
  uint8_t                             Dir;
  uint8_t                             DevAddr[4];
  uint8_t                             FCntDn[4];
  uint8_t                             _pad;
  uint8_t                             idx;
} FRMPayloadBlockA_Dn_t;
#endif

typedef struct MICBlockB0_Up {
  uint8_t                             variant;
  uint8_t                             _noUse[4];
  uint8_t                             Dir;
  uint8_t                             DevAddr[4];
  uint8_t                             FCntUp[4];
  uint8_t                             _pad;
  uint8_t                             len;
} MICBlockB0_Up_t;

#ifdef LORAWAN_1V1
typedef struct MICBlockB1_Up {
  uint8_t                             variant;
  uint8_t                             ConfFCnt[2];
  uint8_t                             TxDr;
  uint8_t                             TxCh;
  uint8_t                             Dir;
  uint8_t                             DevAddr[4];
  uint8_t                             FCntUp[4];
  uint8_t                             _pad;
  uint8_t                             len;
} MICBlockB1_Up_t;
#endif

#ifdef LORAWAN_1V02
typedef struct MICBlockB0_Dn {
  uint8_t                             variant;
  uint8_t                             _noUse[4];
  uint8_t                             Dir;
  uint8_t                             DevAddr[4];
  uint8_t                             FCntDn[4];
  uint8_t                             _pad;
  uint8_t                             len;
} MICBlockB0_Dn_t;
#elif LORAWAN_1V1
typedef struct MICBlockB0_Dn {
  uint8_t                             variant;
  uint8_t                             ConfFCnt[2];
  uint8_t                             _noUse[2];
  uint8_t                             Dir;
  uint8_t                             DevAddr[4];
  uint8_t                             FCntDn[4];
  uint8_t                             _pad;
  uint8_t                             len;
} MICBlockB0_Dn_t;
#endif


/* Memory assignment of the non-volatile memory section of the timer */
typedef struct LoRaWANctxBkpRam {
  /* Skip first entries */
  uint32_t                            _skip[16];

  /* CRC */
  uint32_t                            LoRaWANcrc;

  /* Counters */
  uint32_t                            FCntUp;
#ifdef LORAWAN_1V02
  uint32_t                            FCntDwn;
#elif LORAWAN_1V1
  uint32_t                            NFCntDwn;
#endif
  uint32_t                            AFCntDwn;

  uint32_t                            n8_n8_DLSettings8_RxDelay8;
  uint32_t                            CFList_03_02_01_00;
  uint32_t                            CFList_07_06_05_04;
  uint32_t                            CFList_11_10_09_08;
  uint32_t                            CFList_15_14_13_12;

  uint32_t                            _end;
} LoRaWANctxBkpRam_t;


/* Context information */
typedef struct LoRaWANctx {
  /* Non-volatile counters */
  volatile LoRaWANctxBkpRam_t*        bkpRAM;

  /* FSM */
  volatile Fsm_States_t               FsmState;

  /* Device specific */
  uint8_t                             DevEUI_LE[8];
  volatile uint8_t                    DevAddr_LE[4];                                            // JOIN-ACCEPT
  volatile uint8_t                    DevNonce_LE[2];                                           // JOIN-REQUEST, REJOIN-REQUEST - V1.02: random value
  volatile uint8_t                    Home_NetID_LE[3];                                         // JOIN-ACCEPT
  volatile uint64_t                   BootTime_UTC_ms;                                          // UTC time of xTaskGetTickCount() == 0 (init of the Free-RTOS) in ms

  /* Network / MAC specific */
  LoRaWANVersion_t                    LoRaWAN_ver;
#ifdef LORAWAN_1V1
  uint8_t                             NwkKey_1V1[16];                                           // Network root key (for OTA devices)
#endif
  volatile uint8_t                    NwkSKey[16];                                              // Session key - JOIN-ACCEPT (derived from: AppKey)
#ifdef LORAWAN_1V1
  uint8_t                             FNwkSKey_1V1[16];                                         // JOIN-ACCEPT
  uint8_t                             FNwkSIntKey_1V1[16];                                      // JOIN-ACCEPT (derived from: NwkKey)
  uint8_t                             SNwkSIntKey_1V1[16];                                      // JOIN-ACCEPT (derived from: NwkKey)
  uint8_t                             NwkSEncKey_1V1[16];                                       // JOIN-ACCEPT (derived from: NwkKey)
  uint8_t                             JSIntKey_1V1[16];                                         // (for OTA devices)
  uint8_t                             JSEncKey_1V1[16];                                         // (for OTA devices)
#endif
  volatile uint8_t                    NetID_LE[3];                                              // JOIN-RESPONSE
  volatile uint8_t                    RXDelay;                                                  // JOIN-ACCEPT
  volatile uint8_t                    CFList[16];                                               // JOIN-ACCEPT - EU-868: Freq Ch3[2:0], Freq Ch4[2:0], Freq Ch5[2:0], Freq Ch6[2:0], Freq Ch7[2:0], CFListType[0]
  volatile uint8_t                    Ch_Selected;                                              // Fixed or randomly assigned channel
  volatile float                      Ch_FrequenciesUplink_MHz[16];                             // Default / MAC:NewChannelReq
  volatile float                      Ch_FrequenciesDownlink_MHz[16];                           // Default / MAC:NewChannelReq / MAC:DlChannelReq
  volatile DataRates_t                Ch_DataRateTX_min[16];
  volatile DataRates_t                Ch_DataRateTX_max[16];
  volatile DataRates_t                Ch_DataRateTX_Selected[16];
  volatile float                      GatewayPpm;                                               // PPM value to adjust RX for remote gateway crystal drift

  /* MAC communicated data */
  volatile uint8_t                    ConfirmedPackets_enabled;                                 // Setting: global MType
  volatile uint8_t                    ADR_enabled;                                              // Setting: global ADR
  volatile uint8_t                    TX_MAC_Len;                                               // MAC list to be sent at next transmission
  volatile uint8_t                    TX_MAC_Buf[16];                                           // MAC list to be sent at next transmission
  volatile uint8_t                    LinkCheck_Ppm_SNR;                                        // MAC: LinkCheckAns
  volatile uint8_t                    LinkCheck_GW_cnt;                                         // MAC: LinkCheckAns
  volatile uint8_t                    LinkADR_TxPowerReduction_dB;                              // MAC: LinkADRReq
  volatile DataRates_t                LinkADR_DataRate_TX1;                                     // MAC: LinkADRReq
  volatile uint8_t                    LinkADR_DataRate_RX1_DRofs;                               // JoinAccept
  volatile DataRates_t                LinkADR_DataRate_RXTX2;                                   // MAC: LinkADRReq
  volatile uint16_t                   LinkADR_ChannelMask;                                      // MAC: LinkADRReq
  volatile uint8_t                    LinkADR_ChannelMask_OK;                                   // Last channel mask setting whether valid
  volatile uint8_t                    LinkADR_NbTrans;                                          // MAC: LinkADRReq - unconfirmed up packets multiple transmissions
  volatile ChMaskCntl_t               LinkADR_ChMaskCntl;                                       // MAC: LinkADRReq
  volatile uint8_t                    DutyCycle_MaxDutyCycle;                                   // MAC: DutyCycleReq
  volatile uint8_t                    TxParamSetup_MaxEIRP_dBm;                                 // MAC: TxParamSetupReq
  volatile uint16_t                   TxParamSetup_UplinkDwellTime_ms;                          // MAC: TxParamSetupReq
  volatile uint16_t                   TxParamSetup_DownlinkDwellTime_ms;                        // MAC: TxParamSetupReq
  volatile uint8_t                    Channel_DataRateValid;                                    // MAC: NewChannelReq
  volatile uint8_t                    Channel_FrequencyValid;                                   // MAC: NewChannelReq, DlChannelReq
  volatile uint8_t                    Channel_FrequencyExists;                                  // MAC: DlChannelReq

  /* Join Server specific */
#ifdef LORAWAN_1V1
  uint8_t                             JoinEUI_LE_1V1[8];                                        // JOIN-REQUEST, REJOIN-REQUEST
  volatile uint8_t                    JoinNonce_LE_1V1[4];                                      // JOIN-ACCEPT
  volatile uint8_t                    ServerNonce_LE_1V1[3];
#endif

  /* Application specific */
  uint8_t                             AppEUI_LE[8];
  uint8_t                             AppKey[16];                                               // Application root key (for OTA devices)
  volatile uint8_t                    AppNonce_LE[3];                                           // JOIN-RESPONSE
  volatile uint8_t                    AppSKey[16];                                              // Session key - JOIN-ACCEPT (derived from: AppKey)

  /* Current transmission */
  volatile uint32_t                   TsEndOfTx;                                                // Timestamp of last TX end
  volatile CurrentWindow_t            Current_RXTX_Window;                                      // RX/TX1, RX/TX2 or none
  volatile LoRaWANctxDir_t            Dir;                                                      // 0:up, 1:down
  volatile LoRaWAN_MTypeBF_t          MHDR_MType;                                               // Message type in use
  volatile LoRaWAN_MajorBF_t          MHDR_Major;                                               // Major release to use
  volatile uint8_t                    FCtrl_ADR;
  volatile uint8_t                    FCtrl_ADRACKReq;
  volatile uint8_t                    FCtrl_ACK;
  volatile uint8_t                    FCtrl_ClassB;
  volatile uint8_t                    FCtrl_FPending;
  volatile uint8_t                    FPort_absent;                                             // 1: FPort field absent
  volatile uint8_t                    FPort;
  volatile float                      FrequencyMHz;                                             // This value is taken for the TRX when turned on
  volatile uint8_t                    SpreadingFactor;                                          // This value is taken for the TRX when turned on

  /* Last radio measurements */
  volatile float                      CrystalPpm;                                               // PPM value to correct local device crystal drift
  volatile uint32_t                   LastIqBalTimeUs;                                          // Last point of system time when procedure was done
  volatile uint32_t                   LastIqBalDurationUs;                                      // Duration of balancing procedure
  volatile int8_t                     LastIqBalTemp;                                            // Temperature degC when last I/Q balancing took place
  volatile int16_t                    LastRSSIDbm;
  volatile int16_t                    LastPacketStrengthDBm;
  volatile int8_t                     LastPacketSnrDb;
  volatile float                      LastFeiHz;
  volatile float                      LastFeiPpm;
} LoRaWANctx_t;


typedef struct LoRaWAN_TX_Message {
  /* Ready for transport section */
  volatile uint8_t                    msg_encoded_EncDone;
  volatile uint8_t                    msg_encoded_Len;
  volatile uint8_t                    msg_encoded_Buf[LoRaWAN_MsgLenMax];

  /* Prepare section */
#ifdef LORAWAN_1V1
  uint8_t                             msg_prep_FOpts_Encoded[16];
  //
#endif
  uint8_t                             msg_prep_FRMPayload_Len;
  uint8_t                             msg_prep_FRMPayload_Buf[LoRaWAN_FRMPayloadMax];
  uint8_t                             msg_prep_FRMPayload_Encoded[LoRaWAN_FRMPayloadMax];
} LoRaWAN_TX_Message_t;

typedef struct LoRaWAN_RX_Message {
  /* Received data section */
  volatile uint8_t                    msg_encoded_Len;
  volatile uint8_t                    msg_encoded_Buf[LoRaWAN_MsgLenMax];

  /* parted section */
  uint8_t                             msg_parted_MHDR;
  uint8_t                             msg_parted_MType;
  uint8_t                             msg_parted_Major;
  //
  uint8_t                             msg_parted_DevAddr[4];
  //
  uint8_t                             msg_parted_FCtrl;
  uint8_t                             msg_parted_FCtrl_ADR;
  uint8_t                             msg_parted_FCtrl_ACK;
  uint8_t                             msg_parted_FCtrl_FPending;
  uint8_t                             msg_parted_FCtrl_FOptsLen;
  //
  uint8_t                             msg_parted_FCntDwn[2];
  //
  uint8_t                             msg_parted_FOpts_Buf[16];
  //
  uint8_t                             msg_parted_FPort_absent;
  uint8_t                             msg_parted_FPort;
  //
  uint8_t                             msg_parted_FRMPayload_Len;
  uint8_t                             msg_parted_FRMPayload_Buf[LoRaWAN_FRMPayloadMax];
  uint8_t                             msg_parted_FRMPayload_Encoded[LoRaWAN_FRMPayloadMax];
  //
  uint8_t                             msg_parted_MIC_Buf[4];
  uint8_t                             msg_parted_MIC_Valid;
} LoRaWAN_RX_Message_t;


/* TrackMeApp */
typedef struct TrackMeApp_up {
  /* TTN Mapper entities */
  float                               latitude_deg;
  float                               longitude_deg;
  float                               altitude_m;
  uint16_t                            accuracy_10thM;

  /* Motion vector entities */
  uint16_t                            course_deg;
  float                               speed_m_s;
  float                               vertspeed_m_s;

  /* Weather entities */
  int16_t                             temp_100th_C;
  uint16_t                            humitidy_1000th;
  uint32_t                            baro_Pa;

  /* System entities */
  uint16_t                            vbat_mV;
  int32_t                             ibat_uA;
} TrackMeApp_up_t;

typedef struct TrackMeApp_down {
  /* TBD */
} TrackMeApp_down_t;


/* LoraliveApp */
typedef struct LoraliveApp_up {
  uint8_t                             voltage_32_v;

  /* Big endian */
  uint8_t                             dust025_10_hi;
  uint8_t                             dust025_10_lo;

  /* Big endian */
  uint8_t                             dust100_10_hi;
  uint8_t                             dust100_10_lo;

  char  id;

  union {
    struct loraliveApp_len6 {
    } l6;

    struct loraliveApp_len8 {
      int8_t                          temperature;
      uint8_t                         humidity_100;
    } l8;

    struct loraliveApp_len14 {
      uint8_t                         latitude_1000_sl24;
      uint8_t                         latitude_1000_sl16;
      uint8_t                         latitude_1000_sl08;
      uint8_t                         latitude_1000_sl00;

      uint8_t                         longitude_1000_sl24;
      uint8_t                         longitude_1000_sl16;
      uint8_t                         longitude_1000_sl08;
      uint8_t                         longitude_1000_sl00;
    } l14;

    struct loraliveApp_len16 {
      int8_t                          temperature;
      uint8_t                         humidity_100;

      uint8_t                         latitude_1000_sl24;
      uint8_t                         latitude_1000_sl16;
      uint8_t                         latitude_1000_sl08;
      uint8_t                         latitude_1000_sl00;

      uint8_t                         longitude_1000_sl24;
      uint8_t                         longitude_1000_sl16;
      uint8_t                         longitude_1000_sl08;
      uint8_t                         longitude_1000_sl00;
    } l16;
  } u;
} LoraliveApp_up_t;

typedef struct LoraliveApp_down {
  /* TBD */
} LoraliveApp_down_t;



/* Functions */

uint8_t GET_BYTE_OF_WORD(uint32_t word, uint8_t pos);


void LoRaWANctx_applyKeys_trackMeApp(void);

uint8_t LoRaWAN_calc_randomChannel(LoRaWANctx_t* ctx);
float LoRaWAN_calc_Channel_to_MHz(LoRaWANctx_t* ctx, uint8_t channel, LoRaWANctxDir_t dir, uint8_t dflt);

void LoRaWAN_MAC_Queue_Push(const uint8_t* macAry, uint8_t cnt);
void LoRaWAN_MAC_Queue_Pull(uint8_t* macAry, uint8_t cnt);
void LoRaWAN_MAC_Queue_Reset(void);

void loRaWANLoraTaskInit(void);
void loRaWANLoraTaskLoop(void);


#ifdef __cplusplus
}

#endif

#endif /* LIB_LORAWAN_H_ */
