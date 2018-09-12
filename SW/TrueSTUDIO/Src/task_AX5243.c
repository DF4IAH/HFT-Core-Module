/*
 * task_AX5243.c
 *
 *  Created on: 10.09.2018
 *      Author: DF4IAH
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32l496xx.h"

//#include "stm32l4xx_hal_def.h"
//#include "stm32l4xx_hal_gpio.h"
#include "usb.h"
#include "bus_spi.h"

#include "task_AX5243.h"


extern EventGroupHandle_t   extiEventGroupHandle;
extern osSemaphoreId        usbToHostBinarySemHandle;
extern EventGroupHandle_t   spiEventGroupHandle;

extern ENABLE_MASK_t        g_enableMsk;
extern MON_MASK_t           g_monMsk;

extern uint8_t              spi3TxBuffer[SPI3_BUFFERSIZE];
extern uint8_t              spi3RxBuffer[SPI3_BUFFERSIZE];


/* BLOCK of DEFINITIONS */

//# define  AX_RUN_VCO2_APRS_TX     1U

//# define  AX_TEST_VCO1_BANDENDS   1U
//# define  AX_TEST_VCO1_FSK_TX     1U
//# define  AX_TEST_VCO1_FSK_RX     1U
//# define  AX_TEST_VCO1_POCSAG_TX    1U
//# define  AX_TEST_VCO1_POCSAG_RX    1U

//# define  AX_TEST_VCO2_BANDENDS   1U
//# define  AX_TEST_VCO2_ANALOG_FM_TX 1U
//# define  AX_TEST_VCO2_ANALOG_FM_RX 1U
//# define  AX_TEST_VCO2_PR1200_TX    1U
//# define  AX_TEST_VCO2_PR1200_RX    1U


#if defined(AX_TEST_VCO1_BANDENDS) | defined(AX_TEST_VCO1_FSK_TX)   | defined(AX_TEST_VCO1_FSK_RX)      | defined(AX_TEST_VCO1_FSK_RX)    | defined(AX_TEST_VCO1_POCSAG_RX) | \
  defined(AX_TEST_VCO2_BANDENDS) | defined(AX_TEST_VCO2_ANALOG_FM_TX) | defined(AX_TEST_VCO2_ANALOG_FM_RX)  | defined(AX_TEST_VCO2_PR1200_TX) | defined(AX_TEST_VCO2_PR1200_RX)
# ifndef AX_TEST
# define AX_TEST 1U
# endif
#endif


static uint8_t              s_ax_enable                       = 0U;  // FLASH
static uint8_t              s_ax_aprs_enable                  = 0U;  // FLASH
static uint8_t              s_ax_pocsag_enable                = 0U;  // FLASH
static uint8_t              s_ax_pocsag_beacon_secs           = 0U;  // FLASH
static AX_SET_TX_RX_MODE_t  s_ax_set_tx_rx_mode               = AX_SET_TX_RX_MODE_OFF;
static AX_SET_MON_MODE_t    s_ax_set_mon_mode                 = AX_SET_MON_MODE_OFF;  // FLASH
//static struct spi_device    s_ax_spi_device_conf              = { 0 };
static  uint8_t             s_ax_spi_packet_buffer[C_SPI_AX_BUFFER_LENGTH]  = { 0U };
static  uint8_t             s_ax_spi_rx_buffer[C_SPI_AX_BUFFER_LENGTH]      = { 0U };
static  uint16_t            s_ax_spi_rx_buffer_idx            = 0U;
static  uint8_t             s_ax_spi_rx_fifo_doService        = 0U;
static  uint32_t            s_ax_spi_freq_chan[2]             = { 0UL };
static  uint8_t             s_ax_spi_range_chan[2]            = { 0U };
static  uint8_t             s_ax_spi_vcoi_chan[2]             = { 0U };
static  int8_t              s_ax_spi_rx_bgnd_rssi             = 0x96;  // -170 dBm
static  AX_RX_FIFO_MEAS_t   s_ax_rx_fifo_meas                 = { 0 };

static uint8_t              s_ax_pocsag_chime_enable          = 0U;
static uint32_t             s_ax_pocsag_individual_ric        = 0UL;

static const uint16_t       s_ax_pwr_ary[C_AX_PRW_LENGTH]     = {
  0x0000UL,
  0x000aUL,

  // -10.0 .. - 9.1
    170,    173,    175,    178,    180,    182,    184,    186,    188,    189,

  // - 9.0 .. - 8.1
    191,    193,    194,    196,    198,    199,    201,    203,    205,    207,

  // - 8.0 .. - 7.1
    209,    211,    214,    216,    219,    221,    224,    227,    230,    233,

  // - 7.0 .. - 6.1
    236,    239,    243,    246,    250,    253,    257,    260,    264,    267,

  // - 6.0 .. - 5.1
    271,    275,    278,    282,    285,    289,    292,    296,    299,    303,

  // - 5.0 .. - 4.1
    306,    309,    313,    317,    320,    324,    327,    331,    335,    338,

  // - 4.0 .. - 3.1
    342,    346,    350,    354,    358,    362,    366,    370,    374,    379,

  // - 3.0 .. - 2.1
    383,    387,    392,    397,    401,    406,    411,    416,    421,    426,

  // - 2.0 .. - 1.1
    431,    436,    441,    447,    452,    457,    462,    467,    471,    476,

  // - 1.0 .. - 0.1
    480,    484,    488,    492,    495,    499,    502,    506,    510,    514,

  //   0.0 .. + 0.9
    519,    524,    529,    535,    540,    547,    553,    559,    566,    573,

  // + 1.0 .. + 1.9
    580,    587,    594,    602,    609,    616,    624,    632,    640,    648,

  // + 2.0 .. + 2.1
    656,    664,    673,    681,    690,    699,    708,    718,    727,    737,

  // + 3.0 .. + 3.9
    747,    757,    768,    778,    789,    800,    812,    824,    836,    849,

  // + 4.0 .. + 4.9
    862,    876,    889,    903,    917,    930,    942,    954,    965,    974,

  // + 5.0 .. + 5.9
    982,    988,    993,    997,   1001,   1004,   1008,   1012,   1016,   1022,

  // + 6.0 .. + 6.9
   1030,   1039,   1051,   1064,   1078,   1095,   1112,   1131,   1151,   1171,

  // + 7.0 .. + 7.9
   1193,   1215,   1238,   1261,   1284,   1307,   1329,   1350,   1369,   1388,

  // + 8.0 .. + 8.9
   1404,   1419,   1431,   1443,   1454,   1466,   1477,   1489,   1503,   1518,

  // + 9.0 .. + 9.9
   1536,   1556,   1579,   1604,   1630,   1657,   1685,   1712,   1740,   1766,

  // +10.0 .. +10.9
   1792,   1816,   1839,   1861,   1884,   1907,   1931,   1956,   1984,   2014,

  // +11.0 .. +11.9
   2048,   2085,   2126,   2169,   2215,   2263,   2313,   2363,   2414,   2465,

  // +12.0 .. +12.9
   2516,   2566,   2616,   2666,   2717,   2770,   2824,   2881,   2941,   3004,

  // +13.0 .. +13.9
   3072,   3144,   3220,   3298,   3378,   3459,   3540,   3619,   3696,   3770,

  // +14.0 .. +14.9
   3840,   3905,   3963,   4015,   4058,   4093,   4117,   4131,   4132,   4120,

  // +15.0
   4095
};

static uint8_t              s_ax_pocsag_news_idx              = 0;

static uint8_t              s_ax_pocsag_activation_code_len;
static const uint8_t        s_ax_pocsag_activation_code[][3]  = {
  { 0, 7, 0x32},
  { 0, 7, 0x22},
  { 0, 7, 0x35},
  { 0, 7, 0x33},
  { 0, 7, 0x33},
  { 0, 7, 0x34},
  { 0, 7, 0x34},
  { 0, 7, 0x34}
};


/* Forward declarations */
#ifdef AX_TEST
static void s_spi_test_start_testBox(void);
#endif


static
uint8_t getCurrent_POCSAG_TimeSlot(void)
{
  /* Get up-to-date global data */
  const uint64_t l_seconds10 = getRunTimeCounterValue() / 100ULL;

  /* Each TimeSlot takes 6.4 secs */
  return (uint8_t) ((l_seconds10 >> 6) & 0x0f);
}

#if 0

/* ISR routines */

/* PORTC Pin3 - AX5243 IRQ */
ISR(PORTC_INT0_vect, ISR_BLOCK)
{
  uint8_t processed = 0U;

  /* IRQREQUEST: check which request is on */
  spi_ax_transport(0U, "< 0c R2 >");                           // RD address 0x0C: IRQREQUEST
  uint16_t ax_spi_irq_request = (uint16_t)g_ax_spi_packet_buffer[0] << 8 | g_ax_spi_packet_buffer[1];

  /* Reason(s) for interrupt */

  /* IRQRQFIFONOTEMPTY */
  if (ax_spi_irq_request & _BV(0)) {
    uint8_t fifoRssiCmd[2] = { 0x32 };                            // Own variant of RSSI FIFO message

    /* Get RSSI */
    spi_ax_transport(0U, "< 40 R1 >");                         // RD Address 0x40: RSSI
    fifoRssiCmd[1] = g_ax_spi_packet_buffer[0];

    /* Get local adjusted copies */
    int16_t l_ax_spi_rx_rssi    = (int8_t) (fifoRssiCmd[1]);
    int16_t l_ax_spi_rx_bgnd_rssi = g_ax_spi_rx_bgnd_rssi;

    /* 64 dB offset correction */
    l_ax_spi_rx_rssi    -= 64;
    l_ax_spi_rx_bgnd_rssi -= 64;

    /* Decrease background RSSI each time to avoid blocking */
    if (-170 < l_ax_spi_rx_bgnd_rssi) {
      --g_ax_spi_rx_bgnd_rssi;
    }

    /* Abort packet when signal strength indicator falls near background noise and at least some WORDS were detected */
    if ((g_ax_spi_rx_buffer_idx >= 0x10) && (l_ax_spi_rx_rssi < (l_ax_spi_rx_bgnd_rssi + 12))) {
      /* FRMMODE abort frame */
      spi_ax_transport(0U, "< 92 07 >");                       // WR address 0x12: FRAMING - CRCMODE: none, FRMMODE: Raw, Pattern Match 0x06, FABORT
    }

    /* Concatenate FIFO message to local buffer */
    isr_spi_ax_fifo_readMessage();

    /* Concatenate RSSI to local buffer */
    memcpy(g_ax_spi_rx_buffer + g_ax_spi_rx_buffer_idx, fifoRssiCmd, 2);
    g_ax_spi_rx_buffer_idx += 2;

    /* Inform front-end to process the data */
    g_ax_spi_rx_fifo_doService = 1U;
    processed = 1U;
  }

  /* IRQRFIFOERROR */
  if (ax_spi_irq_request & _BV(4)) {
    /* FIFOCMD / FIFOSTAT */
    spi_ax_transport(0U, "< a8 03 >");                         // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS
    processed = 1U;
  }

  /* IRQRQPLLUNLOCK */
  if (ax_spi_irq_request & _BV(5)) {
    __asm volatile( "nop" );
    processed = 1U;
  }

  /* RADIO EVENT = REVMDONE */
  if (ax_spi_irq_request & _BV(6)) {
    const uint8_t fifoDoneCmd = 0x01;
    memcpy(g_ax_spi_rx_buffer + g_ax_spi_rx_buffer_idx, &fifoDoneCmd, 1);
    g_ax_spi_rx_buffer_idx++;

    /* Inform front-end to process the data */
    g_ax_spi_rx_fifo_doService = 1U;
    processed = 1U;
  }

  if (!processed) {
    /* unknown request */
    __asm volatile( "nop" );
  }
}

void spi_ax_ISR_setFlags(uint8_t flags)
{
  if (!flags) {
    irqflags_t flags = cpu_irq_save();

    /* Disable AX5243 interrupts */
    {
      /* FIFOCMD / FIFOSTAT */
      spi_ax_transport(0U, "< a8 03 >");                       // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

      /* IRQMASK: set to none */
      spi_ax_transport(0U, "< 86 00 00 >");                      // WR address 0x06: IRQMASK - clear all flags
    }

    /* Turn off MCU PORTC interrupts */
    PORTC_INTCTRL = PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;

    cpu_irq_restore(flags);

  } else {
    /* PORTC Pin3 --> INT0 */
    irqflags_t flags = cpu_irq_save();

    /* PORTC Pin3 INPUT */
    ioport_set_pin_dir(AX_IRQ_PIN, IOPORT_DIR_INPUT);

    /* PORTC Pin3 PULLHIGH, Level: low active */
    ioport_set_pin_mode(AX_IRQ_PIN, IOPORT_MODE_PULLUP | IOPORT_MODE_INVERT_PIN);

    /* Interrupt priority levels for INT1 and INT0 = Low */
    PORTC_INTCTRL = PORT_INT1LVL_OFF_gc | PORT_INT0LVL_LO_gc;

    /* Map PORTC Pin3 --> INT0 */
    PORTC_INT0MASK = _BV(3);
    PORTC_INT1MASK = 0;

    /* Enable AX5243 interrupts */
    {
      /* FIFOCMD / FIFOSTAT */
      spi_ax_transport(0U, "< a8 03 >");                       // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

      /* PINFUNCIRQ */
      spi_ax_transport(0U, "< a4 43 >");                       // WR address 0x24: PINFUNCIRQ - set to IRQ source, no PULLUP = !0x80, inverse = 0x40

      /* RADIOEVENTMASK */
      spi_ax_transport(0U, "< 88 00 01 >");                      // WR address 0x08: RADIOEVENTMASK - set to not REVMRXPARAMSETCHG !0x08, not REVMRADIOSTATECHG !0x04, REVMDONE 0x01

      /* IRQMASK */
      spi_ax_transport(0U, "< 86 00 51 >");                      // WR address 0x06: IRQMASK - set to Bit06:IRQMRADIOCTRL 0x040, !Bit05:IRQRQPLLUNLOCK !0x020, Bit04:IRQMFIFOERROR 0x010, Bit0:IRQMFIFONOTEMPTY 0x001
    }

    /* Clear any pending INT1 / INT0 interrupts */
    PORTC_INTFLAGS = 0b00000011;

    /* Activation of all interrupts, again */
    cpu_irq_restore(flags);
  }
}

void isr_spi_ax_fifo_readMessage(void)
{
  /* Read the length of the message within the FIFO */
  spi_ax_transport(0U, "< 2a R2 >");                           // RD address 0x2A: FIFOCOUNT
  uint16_t msgLen = (uint16_t)g_ax_spi_packet_buffer[0] << 8 | g_ax_spi_packet_buffer[1];
  if ((g_ax_spi_rx_buffer_idx + msgLen) > 256) {
    msgLen = 256 - g_ax_spi_rx_buffer_idx;
  }

  /* Fetch the AX5243 FIFO content */
  uint8_t fifoDataAddr = 0x29;
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, &fifoDataAddr, 1);
  spi_read_packet(&SPI_AX, g_ax_spi_rx_buffer + g_ax_spi_rx_buffer_idx, msgLen);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  g_ax_spi_rx_buffer_idx += msgLen;
}
#endif

static uint8_t sel_u8_from_u32(uint32_t in_u32, uint8_t sel)
{
  return 0xff & (in_u32 >> (sel << 3));
}

static void spi_ax_select_device(void)
{
#if 0
  /* clear PORT C4 */
  PORTC_OUTCLR = 0x10;
#endif
}

#if 0
static void spi_ax_deselect_device(void)
{
#if 0
  /* set   PORT C4 */
  PORTC_OUTSET = 0x10;
#endif
}
#endif

static uint8_t strGetHex(const char* str)
{
  unsigned int ret = 0;

  sscanf(str, "%02x", &ret);
  return (uint8_t)ret;
}

static uint8_t strGetDec(const char* str, int* o_val)
{
  return (uint8_t) sscanf(str, "%d", o_val);
}

static void spi_ax_xtal_waitReady(void)
{
  /*  Wait until crystal oscillator is ready */
  //do {
    //spi_ax_transport(0U, "< 1d R1 >");                       // RD Address 0x1D: XTALSTATUS
  //} while (!(g_ax_spi_packet_buffer[0] & 0x01));                      // Bit 0: XTALRUN
}


AX_POCSAG_CW2_t ax_pocsag_analyze_msg_tgtFunc_get(const char* msg, uint16_t msgLen)
{
  if (!msg || !msgLen) {
    return AX_POCSAG_CW2_MODE1_TONE;
  }

  for (uint16_t idx = 0; idx < msgLen; idx++) {
    char c = *(msg + idx);

    if (('0' <= c) && (c <= '9')) {
      continue;
    }

    if ((c == ' ') || (c == '-') || (toupper(c) == 'U') || (c == '[') || (c == ']') || (c == '(') || (c == ')')) {
      continue;
    }

    return AX_POCSAG_CW2_MODE3_ALPHANUM;
  }

  return AX_POCSAG_CW2_MODE0_NUMERIC;
}

uint8_t spi_ax_pocsag_calc_evenParity(uint32_t par)
{
  par ^= par >> 0x01;
  par ^= par >> 0x02;
  par ^= par >> 0x04;
  par ^= par >> 0x08;
  par ^= par >> 0x10;
  par &= 1;

  return (uint8_t) par;
}

uint32_t spi_ax_pocsag_calc_checkAndParity(uint32_t codeword_in)
{
  const uint32_t inPoly   = 0xfffff800UL & codeword_in;
  const uint32_t genPoly    = 0x769UL;                            // generating polynomial   x10 + x9 + x8 + x6 + x5 + x3 + 1
  uint32_t cw_calc      = inPoly;

  // Calculate the BCH check
  for (uint8_t idx = 31; idx >= 11; idx--) {
    if (cw_calc  & (1UL <<  idx)) {
      volatile uint32_t genPolyShifted = genPoly << (idx - 10);
      cw_calc ^= genPolyShifted;
      (void)genPolyShifted;
    }
  }
  cw_calc &= 0x000007feUL;

  /* Data with BCH check */
  cw_calc |= inPoly;

  /* Calculate for Even Parity */
  uint8_t par = spi_ax_pocsag_calc_evenParity(cw_calc);

  /* Data with 31:21 BCH and Parity */
  cw_calc |= par;

  return cw_calc;
}

uint8_t spi_ax_pocsag_getBcd(char c)
{
  uint8_t u8;

  switch (c) {
    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
    case 0x38:
    case 0x39:
      u8 = c - 0x30;
      break;

    case 0x23:                                        // #
    case 0x2a:                                        // *
      u8 = 0xa;
      break;

    case 0x55:                                        // U
    case 0x75:                                        // u
      u8 = 0xb;
      break;

    case 0x20:                                        // ' '
      u8 = 0xc;
      break;

    case 0x2d:                                        // -
      u8 = 0xd;
      break;

    case 0x5d:                                        // ]
    case 0x29:                                        // )
      u8 = 0xe;
      break;

    case 0x5b:                                        // [
    case 0x28:                                        // (
      u8 = 0xf;
      break;

    default:
      u8 = 0xc;                                     // ' '
  }
  return u8;
}

char spi_ax_pocsag_getReversedNumChar(uint8_t nibble)
{
  char c;

  switch (nibble) {
    case 0b0000:
      c = 0x30;
      break;

    case 0b1000:
      c = 0x31;
      break;

    case 0b0100:
      c = 0x32;
      break;

    case 0b1100:
      c = 0x33;
      break;

    case 0b0010:
      c = 0x34;
      break;

    case 0b1010:
      c = 0x35;
      break;

    case 0b0110:
      c = 0x36;
      break;

    case 0b1110:
      c = 0x37;
      break;

    case 0b0001:
      c = 0x38;
      break;

    case 0b1001:
      c = 0x39;
      break;

    case 0b0101:                                      // #
      c = 0x23;
      break;

    case 0b1101:                                      // U
      c = 0x55;
      break;

    case 0b0011:                                      // ' '
      c = 0x20;
      break;

    case 0b1011:                                      // -
      c = 0x2d;
      break;

    case 0b0111:                                      // ]
      c = 0x5d;
      break;

    case 0b1111:                                      // [
      c = 0x5b;
      break;

    default:
      c = 0x20;                                     // ' '
  }
  return c;
}

char spi_ax_pocsag_getReversedAlphaChar(uint8_t reversedIn)
{
  char c = 0;

  /* Reverse 7-bit to get ASCII */
  for (uint8_t idx = 0; idx < 7; idx++) {
    /* Take LSB */
    uint8_t bit = reversedIn & 0x01;

    /* Move bit to reversed target */
    c <<= 1;
    c  |= bit;
    reversedIn >>= 1;
  }
  return c;
}

uint32_t spi_ax_pocsag_get20Bits(const char* tgtMsg, uint16_t tgtMsgLen, AX_POCSAG_CW2_t tgtFunc, uint16_t msgBitIdx)
{
  uint32_t msgWord = 0UL;
  uint16_t msgByteIdx;
  uint16_t msgByteIdxLast = 65535;
  uint8_t  byteBitPos;
  uint8_t  byte = 0;

  for (uint16_t bitCnt = 20; bitCnt; bitCnt--, msgBitIdx++) {
    if (tgtFunc == AX_POCSAG_CW2_MODE0_NUMERIC) {
      /* Calculate source byte and bit position of byte */
      msgByteIdx  = msgBitIdx >> 2;
      byteBitPos  = msgBitIdx & 0x3;

      /* Get the byte as NUMERIC - NUMERIC_SPACE when message exhausted */
      if (msgByteIdxLast != msgByteIdx) {
        msgByteIdxLast = msgByteIdx;
        byte = (msgByteIdx < tgtMsgLen) ?  spi_ax_pocsag_getBcd(*(tgtMsg + msgByteIdx)) : 0xc;
      }

    } else {
      /* Calculate source byte and bit position of byte */
      msgByteIdx  = msgBitIdx / 7;
      byteBitPos  = msgBitIdx % 7;

      /* Get the byte as ALPHA - NULL when message exhausted */
      if (msgByteIdxLast != msgByteIdx) {
        msgByteIdxLast = msgByteIdx;
        byte = (msgByteIdx < tgtMsgLen) ?  (uint8_t) *(tgtMsg + msgByteIdx) : 0;
      }
    }

    msgWord <<= 1;
    if (byte & (1U << byteBitPos)) {
      msgWord |= 0b1;
    }
  }
  return msgWord;
}

char spi_ax_pocsag_getNumeric(const uint32_t* rcv20Bits, uint8_t rcv20BitsCnt, uint8_t msgNumIdx)
{
  uint8_t rcv20BitsIdx  = msgNumIdx / 5;
  uint8_t innerPos    = 4 - (msgNumIdx % 5);
  uint8_t mvCnt     = innerPos << 2;
  char numChar      = ' ';

  if (rcv20BitsIdx < rcv20BitsCnt) {
    /* Get nibble slice */
    uint8_t nibble = (*(rcv20Bits + rcv20BitsIdx) >> mvCnt) & 0x0f;
    numChar = spi_ax_pocsag_getReversedNumChar(nibble);
  }
  return numChar;
}

char spi_ax_pocsag_getAlphanum(const uint32_t* rcv20Bits, uint8_t rcv20BitsCnt, uint8_t msgAlphaIdx)
{
  uint8_t   rcv20BitsStartWordPos = (msgAlphaIdx * 7) / 20;
  uint8_t   rcv20BitsStartBitPos  = (msgAlphaIdx * 7) - (rcv20BitsStartWordPos * 20);

  /* Check for overflow */
  if ((rcv20BitsStartBitPos > 13) && ((rcv20BitsStartWordPos + 1) >= rcv20BitsCnt)) {
    return ' ';
  }

  /* Preload with MSB 20 bits */
  uint32_t charHiPad = *(rcv20Bits + rcv20BitsStartWordPos);

  /* Decide whether word concatenation is needed */
  if (rcv20BitsStartBitPos <= 13) {
    /* No overlapping */
    charHiPad >>= (13 - rcv20BitsStartBitPos);

  } else {
    /* Complete with LSB part when overlapping */

    /* Load LSB part */
    uint32_t charLoPad = *(rcv20Bits + rcv20BitsStartWordPos + 1);

    /* Move upper part to right position */
    charHiPad <<= (rcv20BitsStartBitPos - 13);

    /* Move lower part to right position */
    charLoPad >>= (33 - rcv20BitsStartBitPos);

    /* Combine into MSB pad */
    charHiPad  |= charLoPad;
  }

  /* Get character of this 7-bit field */
  return spi_ax_pocsag_getReversedAlphaChar((uint8_t) (charHiPad & 0x0000007fUL));
}


void spi_ax_pocsag_address_tone(uint8_t is_RIC_individual, uint32_t address, uint8_t fktBits)
{
  char isMyChar = is_RIC_individual ?  '*' : ' ';
  char isMyBeep = is_RIC_individual ?  0x07 : ' ';
  int  dbgLen;
  char dbgBuf[256];

  dbgLen = (uint8_t) sprintf(dbgBuf, "\r\n>>> TS=%1X %c Address=%8lu FktBits=0x%02x  (TONE   )%c%c%c\r\n", getCurrent_POCSAG_TimeSlot(), isMyChar, address, fktBits, isMyBeep, isMyBeep, isMyBeep);
  usbLogLen(dbgBuf, dbgLen);
}

void spi_ax_pocsag_address_numeric(uint8_t is_RIC_individual, uint32_t address, uint8_t fktBits, uint32_t* dataAry, uint8_t dataCnt)
{
  char isMyChar = is_RIC_individual ?  '*' : ' ';
  char isMyBeep = is_RIC_individual ?  0x07 : ' ';
  uint8_t digitIdx = 0;
  int  dbgLen;
  char dbgBuf[256];

  dbgLen = (uint8_t) sprintf(dbgBuf, "\r\n>>> TS=%1X %c Address=%8lu FktBits=0x%02x  (NUMERIC):", getCurrent_POCSAG_TimeSlot(), isMyChar, address, fktBits);
  usbLogLen(dbgBuf, dbgLen);

  for (uint8_t dataCntIdx = 0; dataCntIdx < dataCnt; dataCntIdx++) {
    for (uint8_t sliceIdx = 0; sliceIdx < 5; sliceIdx++, digitIdx++) {
      dbgBuf[0] = spi_ax_pocsag_getNumeric(dataAry, dataCnt, digitIdx);
      usbLogLen(dbgBuf, 1);
    }
  }

  dbgLen = (uint8_t) sprintf(dbgBuf, "%c\r\n", isMyBeep);
  usbLogLen(dbgBuf, dbgLen);
}

void spi_ax_pocsag_address_alphanum(uint8_t is_RIC_individual, uint32_t address, uint8_t fktBits, uint32_t* dataAry, uint8_t dataCnt, AX_POCSAG_IS_SKYPER_SPECIAL_t skyperSpecial)
{
  const char* typeStr = 0;
  char isMyChar = is_RIC_individual ?  '*' : ' ';
  char isMyBeep = is_RIC_individual ?  0x07 : ' ';
  int  dbgLen;
  char dbgBuf[256];
  char l_buf[256];

  switch (skyperSpecial) {
    case 1:
      typeStr = "RUBRIC ";
    break;

    case 2:
      typeStr = "NEWS   ";
    break;

    default:
      typeStr = "ALPHA  ";
  }

  dbgLen = (uint8_t) sprintf(dbgBuf, "\r\n>>> TS=%1X %c Address=%8lu FktBits=0x%02x  (%s):", getCurrent_POCSAG_TimeSlot(), isMyChar, address, fktBits, typeStr);
  usbLogLen(dbgBuf, dbgLen);
  dbgLen = 0;

  uint8_t alphaCnt = (dataCnt * 20) / 7;
  if (alphaCnt >= sizeof(dbgBuf)) {
    /* Buffer overflow */
    return;
  }

  /* Get text data */
  for (uint8_t alphaIdx = 0; alphaIdx < alphaCnt; alphaIdx++) {
    dbgBuf[dbgLen++] = spi_ax_pocsag_getAlphanum(dataAry, dataCnt, alphaIdx);
  }

  switch (skyperSpecial) {
    case 1:
      /* Decode Rubric content */
      dbgLen = spi_ax_pocsag_skyper_RubricString_Decode(l_buf, sizeof(l_buf), dbgBuf, dbgLen);
      usbLogLen(l_buf, dbgLen);
    break;

    case 2:
      /* Decode News content */
      dbgLen = spi_ax_pocsag_skyper_NewsString_Decode(l_buf, sizeof(l_buf), dbgBuf, dbgLen);
      usbLogLen(l_buf, dbgLen);
    break;

    default:
      usbLogLen(dbgBuf, dbgLen);
  }

  dbgLen = (uint8_t) sprintf(dbgBuf, "%c\r\n", isMyBeep);
  usbLogLen(dbgBuf, dbgLen);
}

void spi_ax_pocsag_address_skyper_activation(uint8_t is_RIC_individual, uint32_t address, uint8_t fktBits, uint32_t* dataAry, uint8_t dataCnt)
{
  char isMyChar = is_RIC_individual ?  '*' : ' ';
  char isMyBeep = is_RIC_individual ?  0x07 : ' ';
  int  dbgLen;
  char dbgBuf[256];

  dbgLen = (uint8_t) sprintf(dbgBuf, "\r\n>>> TS=%1X %c Address=%8lu FktBits=0x%02x  (ACTIVTN):", getCurrent_POCSAG_TimeSlot(), isMyChar, address, fktBits);
  usbLogLen(dbgBuf, dbgLen);

  dbgLen = (uint8_t) sprintf(dbgBuf, " TODO! %c%c%c\r\n", isMyBeep, isMyBeep, isMyBeep);
  usbLogLen(dbgBuf, dbgLen);
}


uint16_t spi_ax_pocsag_skyper_RIC2ActivationString(char* outBuf, uint16_t outBufSize, uint32_t RIC)
{
  uint16_t outLen = 0;

  /* Sanity checks */
  if (!outBuf || outBufSize <= s_ax_pocsag_activation_code_len) {
    return 0;
  }

  for (uint16_t codeIdx = 0; codeIdx < s_ax_pocsag_activation_code_len; codeIdx++) {
    outBuf[outLen++] = (((RIC
              >> s_ax_pocsag_activation_code[codeIdx][AX_POCSAG_SKYPER_ACTIVATION_ARY_SHIFT])
              &  s_ax_pocsag_activation_code[codeIdx][AX_POCSAG_SKYPER_ACTIVATION_ARY_MASK])
              +  s_ax_pocsag_activation_code[codeIdx][AX_POCSAG_SKYPER_ACTIVATION_ARY_OFFSET])
              &  0x7f;                              // ASCII-7bit
  }
  outBuf[outLen] = 0;

  return outLen;
}

const char          PM_POCSAG_SKYPER_TIME[]         = "%02d%02d%02d   %02d%02d%02d";
//PROGMEM_DECLARE(const char, PM_POCSAG_SKYPER_TIME[]);
#if 0
uint16_t spi_ax_pocsag_skyper_TimeString(char* outBuf, uint16_t outBufSize, struct calendar_date* calDat)
{
  /* Sanity checks */
  if (!outBuf || outBufSize <= 15) {
    return 0;
  }

  /* Put calendar information into Skyper time message form */
  return (uint16_t) snprintf(outBuf, outBufSize, PM_POCSAG_SKYPER_TIME, calDat->hour, calDat->minute, calDat->second, calDat->date + 1, calDat->month + 1, calDat->year % 100);
}
#endif

const char          PM_POCSAG_SKYPER_RUBRIC_ENC[]     = "1%c%c";
//PROGMEM_DECLARE(const char, PM_POCSAG_SKYPER_RUBRIC_ENC[]);
uint16_t spi_ax_pocsag_skyper_RubricString_Encode(char* outBuf, uint16_t outBufSize, uint8_t rubricNumber, const char* rubricLabel, uint16_t rubricLabelLen)
{
  /* Sanity checks */
  if (!outBuf || outBufSize < 4) {
    return 0;
  }

  /* Put rubric label into Skyper rubric message form */
  uint16_t outLen = snprintf(outBuf, outBufSize, PM_POCSAG_SKYPER_RUBRIC_ENC, ' ' + (rubricNumber - 1), ' ' + 10);

  char* outBuf_ptr    = outBuf + outLen;
  const char* read_ptr  = rubricLabel;
  for (uint16_t idx = 0; idx < rubricLabelLen; idx++) {
    *(outBuf_ptr++) = *(read_ptr++) + 1;

    /* Target buffer runs out of space */
    if (++outLen >= (outBufSize - 1)) {
      break;
    }
  }
  *outBuf_ptr = 0;
  return outLen;
}

const char          PM_POCSAG_SKYPER_RUBRIC_DEC[]     = "Rubric=%02d, MaxEntries=%02d, Name: ";
//PROGMEM_DECLARE(const char, PM_POCSAG_SKYPER_RUBRIC_DEC[]);
uint16_t spi_ax_pocsag_skyper_RubricString_Decode(char* outBuf, uint16_t outBufSize, const char* pocsagSkyperRubricMsg, uint16_t pocsagSkyperRubricMsgLen)
{
  /* Sanity checks */
  if (!outBuf || outBufSize < 8 || pocsagSkyperRubricMsgLen < 3) {
    return 0;
  }

  uint8_t rubricNumber  = (pocsagSkyperRubricMsg[1] - ' ') + 1;
  uint8_t rubricSize    = pocsagSkyperRubricMsg[2] - ' ';
  uint16_t outLen     = snprintf(outBuf, outBufSize, PM_POCSAG_SKYPER_RUBRIC_DEC, rubricNumber, rubricSize);

  char* outBuf_ptr    = outBuf + outLen;
  const char* read_ptr  = pocsagSkyperRubricMsg + 3;
  for (uint16_t idx = 3; idx < pocsagSkyperRubricMsgLen; idx++) {
    *(outBuf_ptr++) = *(read_ptr++) - 1;

    /* Target buffer runs out of space */
    if (++outLen >= (outBufSize - 1)) {
      break;
    }
  }
  *outBuf_ptr = 0;
  return outLen;
}

const char          PM_POCSAG_SKYPER_NEWS_ENC[]       = "%c%c";
//PROGMEM_DECLARE(const char, PM_POCSAG_SKYPER_NEWS_ENC[]);
uint16_t spi_ax_pocsag_skyper_NewsString_Encode(char* outBuf, uint16_t outBufSize, uint8_t rubricNumber, uint8_t newsNumber, const char* newsString, uint16_t newsStringLen)
{
  /* Sanity checks */
  if (!outBuf || outBufSize < 2) {
    return 0;
  }

  /* Put news into Skyper news message form */
  uint16_t outLen = snprintf(outBuf, outBufSize, PM_POCSAG_SKYPER_NEWS_ENC, ' ' + (rubricNumber - 1), ' ' + newsNumber);

  char* outBuf_ptr    = outBuf + outLen;
  const char* read_ptr  = newsString;
  for (uint16_t idx = 0; idx < newsStringLen; idx++) {
    *(outBuf_ptr++) = *(read_ptr++) + 1;

    /* Target buffer runs out of space */
    if (++outLen >= (outBufSize - 1)) {
      break;
    }
  }
  *outBuf_ptr = 0;
  return outLen;
}

const char          PM_POCSAG_SKYPER_NEWS_DEC[]       = "Rubric=%02d, News#=%02d, Text: ";
//PROGMEM_DECLARE(const char, PM_POCSAG_SKYPER_NEWS_DEC[]);
uint16_t spi_ax_pocsag_skyper_NewsString_Decode(char* outBuf, uint16_t outBufSize, const char* pocsagSkyperNewsMsg, uint16_t pocsagSkyperNewsMsgLen)
{
  /* Sanity checks */
  if (!outBuf || outBufSize < 8 || pocsagSkyperNewsMsgLen < 2) {
    return 0;
  }

  uint8_t rubricNumber  = (pocsagSkyperNewsMsg[0] - ' ') + 1;
  uint8_t newsNumber    = pocsagSkyperNewsMsg[1] - ' ';
  uint16_t outLen     = snprintf(outBuf, outBufSize, PM_POCSAG_SKYPER_NEWS_DEC, rubricNumber, newsNumber);

  char* outBuf_ptr    = outBuf + outLen;
  const char* read_ptr  = pocsagSkyperNewsMsg + 2;
  for (uint16_t idx = 2; idx < pocsagSkyperNewsMsgLen; idx++) {
    *(outBuf_ptr++) = *(read_ptr++) - 1;

    /* Target buffer runs out of space */
    if (++outLen >= (outBufSize - 1)) {
      break;
    }
  }
  *outBuf_ptr = 0;
  return outLen;
}


void spi_ax_pocsag_wordDecoder(AX_POCSAG_DECODER_DATA_t* l_pocsagData, uint32_t pocsagWord, uint8_t pocsagWordCnt)
{
  uint8_t tryBits = 32;

  /* Sanity check */
  if (!l_pocsagData) {
    return;
  }

  /* Preparations */
  l_pocsagData->badDecode   = 1U;                               // No decoding data result
  l_pocsagData->badParity   = 1U;                               // Parity check on non-modified data failed
  l_pocsagData->badCheck    = 1U;                               // Check on non-modified data failed
  l_pocsagData->isAddr    = 0U;
  l_pocsagData->isData    = 0U;
  l_pocsagData->addrData    = 0UL;
  l_pocsagData->functionBits  = 0;

  /* Outer loop for one bit error corrections */
  do {
    l_pocsagData->invertedBit = tryBits;

    /* Try this pattern */
    uint32_t l_pocsagWord = pocsagWord;
    if ((0x00 <= tryBits) && (tryBits < 0x20)) {
      l_pocsagWord ^= (1UL << tryBits);
    }

    /* Inner loop for CODEWORD under test */
    do {
      /* Parity check */
      uint8_t  parCalc   = spi_ax_pocsag_calc_evenParity(l_pocsagWord & 0xfffffffe);
      uint8_t  parData   = (l_pocsagWord & 0x00000001) != 0;
      uint8_t  l_badParity = (parCalc != parData);
      if (tryBits == 0x20) {
        l_pocsagData->badParity = l_badParity;
      }
      if (l_badParity) {
        break;
      }

      /* Check calculation */
      uint32_t  chkCalc   = spi_ax_pocsag_calc_checkAndParity(l_pocsagWord);
      uint8_t    l_badCheck  = (chkCalc != l_pocsagWord);
      if (tryBits == 0x20) {
        l_pocsagData->badCheck = l_badCheck;

        if (l_badCheck && !l_badParity) {
          /* At least 2 bit errors detected - abort */
          return;
        }
      }
      if (l_badCheck) {
        break;
      }

      /* Address / Data assignment */
      l_pocsagData->badDecode = 0U;
      if (!(l_pocsagWord & 0x80000000UL)) {
        /* Address - HI-part concatenated by LO-part */
        l_pocsagData->isAddr  = 1U;
        l_pocsagData->addrData  = ((l_pocsagWord >> 10) & 0x001ffff8UL) | ((pocsagWordCnt >> 1) & 0b111);

        /* Function bits */
        l_pocsagData->functionBits = (uint8_t) ((l_pocsagWord >> 11) & 0b11);

      } else {
        /* Data */
        l_pocsagData->isData  = 1U;
        l_pocsagData->addrData  = ((l_pocsagWord >> 11) & 0x000fffffUL);
      }
      return;
    } while (0U);
  } while (tryBits--);
}

void spi_ax_pocsag_messageDecoder(uint32_t address, uint8_t functionBits, uint32_t* dataAry, uint8_t dataCnt)
{
  uint8_t isSkyper = 1U;

  if (isSkyper) {
    uint32_t l_ax_pocsag_individual_ric = s_ax_pocsag_individual_ric;

    switch (address) {
      case AX_POCSAG_SKYPER_RIC_CLOCK:
      {
        if (functionBits == AX_POCSAG_CW2_MODE0_NUMERIC) {
          /* Time information */
          spi_ax_pocsag_address_numeric(0U, address, functionBits, dataAry, dataCnt);      // TODO: DEBUGGING ONLY
        }
      }
      break;

      case AX_POCSAG_SKYPER_RIC_RUBRICS:
      {
        if (functionBits == AX_POCSAG_CW2_MODE3_ALPHANUM) {
          /* Store list of Rubrics */
          spi_ax_pocsag_address_alphanum(0U, address, functionBits, dataAry, dataCnt, AX_POCSAG_IS_SKYPER_SPECIAL_RUBRIC);   // TODO: DEBUGGING ONLY
        }
      }
      break;

      case AX_POCSAG_SKYPER_RIC_NEWS:
      {
        if (functionBits == AX_POCSAG_CW2_MODE3_ALPHANUM) {
          /* Store News into rubrics */
          spi_ax_pocsag_address_alphanum(0U, address, functionBits, dataAry, dataCnt, AX_POCSAG_IS_SKYPER_SPECIAL_NEWS);   // TODO: DEBUGGING ONLY
        }
      }
      break;

      case 0:
      {
        /* Drop non-valid address */
      }
      break;

      default:
      {
        uint8_t is_RIC_individual = address == l_ax_pocsag_individual_ric;
        switch (functionBits) {
          case AX_POCSAG_CW2_MODE2_ACTIVATION:
          {
            /* Check activation */
            spi_ax_pocsag_address_skyper_activation(is_RIC_individual, address, functionBits, dataAry, dataCnt);
          }
          break;

          case AX_POCSAG_CW2_MODE0_NUMERIC:
          {
            /* Numeric decoder */
            spi_ax_pocsag_address_numeric(is_RIC_individual, address, functionBits, dataAry, dataCnt);
          }
          break;

          case AX_POCSAG_CW2_MODE1_TONE:
          {
            /* Beep */
            spi_ax_pocsag_address_tone(is_RIC_individual, address, functionBits);
          }
          break;

          case AX_POCSAG_CW2_MODE3_ALPHANUM:
          {
            /* Alphanum decoder */
            spi_ax_pocsag_address_alphanum(is_RIC_individual, address, functionBits, dataAry, dataCnt, AX_POCSAG_IS_SKYPER_SPECIAL_NO);
          }
          break;
        }  // switch (functionBits)
      }
    }  // switch (address)
  }  // if (isSkyper)
}


#if 0
status_code_t spi_ax_transport(uint8_t isProgMem, const char* packet)
{
  uint8_t           l_axIdx               = 0;
  uint16_t          l_fmtIdx              = 0;
  SPI_AX_TRPT_STATE_t     l_state               = SPI_AX_TRPT_STATE_DISABLED;
  char            l_msg_buf[C_SPI_AX_BUFFER_LENGTH];
  const char          *l_fmtPtr             = isProgMem ?  l_msg_buf : packet;
  uint8_t            l_isRead              = 0U;
  uint8_t           l_adr0 = 0, l_adr1 = 0;
  status_code_t       sc;

  memset(l_msg_buf, 0, sizeof(l_msg_buf));
  memset(g_ax_spi_packet_buffer, 0, sizeof(g_ax_spi_packet_buffer));

  if (isProgMem) {
    snprintf(l_msg_buf, sizeof(l_msg_buf), packet);
  }

  do {
    switch (l_state) {
      case SPI_AX_TRPT_STATE_DISABLED:
      {
        if (*(l_fmtPtr + l_fmtIdx++) == '<') {
          l_adr0 = 0; l_adr1 = 0;

          /* Skip mandatory space */
          if (*(l_fmtPtr + l_fmtIdx++) != ' ') {
            l_state = SPI_AX_TRPT_STATE_ERROR;
            break;
          }

          spi_select_device(&SPI_AX, &g_ax_spi_device_conf);

          /* Check if read symbol for status request comes first */
          if (*(l_fmtPtr + l_fmtIdx) == 'R') {
            l_state = SPI_AX_TRPT_STATE_ENA_DATA_READ;
            break;

          } else {
            l_state = SPI_AX_TRPT_STATE_ENA_ADDR;
          }

        } else {
          l_state = SPI_AX_TRPT_STATE_ERROR;
          break;
        }
      }
      break;

      case SPI_AX_TRPT_STATE_ENA_ADDR:
      {
        l_adr0 = s_strGetHex(l_fmtPtr + l_fmtIdx); l_fmtIdx++; l_fmtIdx++;

        /* Build packet */
        g_ax_spi_packet_buffer[l_axIdx++] = l_adr0;

        l_isRead = (l_adr0 & 0x80) ?  0U : 1U;

        /* Long address format */
        if ((l_adr0 & 0x7f) >= 0b01110000) {
          /* Skip mandatory space */
          if (*(l_fmtPtr + l_fmtIdx++) != ' ') {
            l_state = SPI_AX_TRPT_STATE_ERROR;
            break;
          }

          l_adr1 = s_strGetHex(l_fmtPtr + l_fmtIdx); l_fmtIdx++; l_fmtIdx++;

          /* Append to packet */
          g_ax_spi_packet_buffer[l_axIdx++] = l_adr1;
        }

        /* Skip mandatory space */
        if (*(l_fmtPtr + l_fmtIdx++) != ' ') {
          l_state = SPI_AX_TRPT_STATE_ERROR;
          break;
        }

        /* Send read address */
        if (l_isRead) {
          sc = spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, l_axIdx);
          if (sc != STATUS_OK) {
            return sc;
          }

          l_axIdx = 0;
          l_state = SPI_AX_TRPT_STATE_ENA_DATA_READ;

        } else {
          l_state = SPI_AX_TRPT_STATE_ENA_DATA_WRITE;
        }
      }
      break;

      case SPI_AX_TRPT_STATE_ENA_DATA_WRITE:
      {
        char c = tolower(*(l_fmtPtr + l_fmtIdx));
        if (c == '>') {
          sc = spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, l_axIdx);
          if (sc != STATUS_OK) {
            return sc;
          }

          l_axIdx = 0;
          l_state = SPI_AX_TRPT_STATE_COMPLETE;
          break;

        } else if ((('0' <= c) && (c <= '9')) || (('a' <= c) && (c <= 'f'))) {
          uint8_t data = s_strGetHex(l_fmtPtr + l_fmtIdx); l_fmtIdx++; l_fmtIdx++;

          /* Append to packet */
          g_ax_spi_packet_buffer[l_axIdx++] = data;

          /* Skip mandatory space */
          if (*(l_fmtPtr + l_fmtIdx++) != ' ') {
            l_state = SPI_AX_TRPT_STATE_ERROR;
            break;
          }

        } else {
          /* Proof: trying to read from a write address ? */
          l_state = SPI_AX_TRPT_STATE_ERROR;
          break;
        }
      }
      break;

      case SPI_AX_TRPT_STATE_ENA_DATA_READ:
      {
        int   cnt = 0;
        uint8_t len;

        /* Skip mandatory read symbol */
        if (*(l_fmtPtr + l_fmtIdx++) != 'R') {
          l_state = SPI_AX_TRPT_STATE_ERROR;
          break;
        }

        len = s_strGetDec(l_fmtPtr + l_fmtIdx, &cnt); l_fmtIdx += ++len;

        /* Execute read */
        sc = spi_read_packet(&SPI_AX, g_ax_spi_packet_buffer, cnt);
        if (sc != STATUS_OK) {
          return sc;
        }

        /* Expect deselect symbol */
        if (*(l_fmtPtr + l_fmtIdx++) == '>') {
          l_state = SPI_AX_TRPT_STATE_COMPLETE;

        } else {
          l_state = SPI_AX_TRPT_STATE_ERROR;
          break;
        }
      }
      break;

      case SPI_AX_TRPT_STATE_COMPLETE:
      {
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
        l_state = SPI_AX_TRPT_STATE_END;
      }
      break;

      case SPI_AX_TRPT_STATE_ERROR:
      default:
      {
        /* ERROR in format string */
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

        //return 0U;
        while (1U)
          __asm volatile( "nop" );
      }
    }
  } while (l_state != SPI_AX_TRPT_STATE_END);

  return STATUS_OK;
}
#endif


#if 0
status_code_t spi_ax_sync2Powerdown(void)
{
  /* @see AND9347/D, page 12: Preparation 1. and 2. */
  uint16_t cntr = 0;

  /* SEL line and MISO check */
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
  delay_us(1);
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);

  while (--cntr && !ioport_get_pin_level(AX_MISO_PIN)) {
    barrier();
  }

  if (!cntr) {
    /* Timeout */
    return ERR_TIMEOUT;
  }

  /* Set RESET with Powerdown mode */
  spi_ax_transport(0U, "< 82 80 >");                         // WR address 0x02: PWRMODE - RESET, PWRMODE=Powerdown

  /* Clear RESET and go to Powerdown */
  spi_ax_transport(0U, "< 82 00 >");                         // WR address 0x02: PWRMODE - RESET, PWRMODE=Powerdown

  /* Ready to program register file */
  /* @see AND9347/D, page 12: Preparation 3. */
  return STATUS_OK;
}

static void s_spi_ax_xtal_waitReady(void)
{
  /*  Wait until crystal oscillator is ready */
  do {
    spi_ax_transport(0U, "< 1d R1 >");                       // RD Address 0x1D: XTALSTATUS
  } while (!(g_ax_spi_packet_buffer[0] & 0x01));                      // Bit 0: XTALRUN
}
#endif

void spi_ax_setPower_dBm(float dBm)
{
  uint16_t pwrReg; (void) pwrReg;
  uint8_t pwrIdx;

  if (dBm > 15.f) {
    dBm = 15.f;

  } else if (dBm < -20.f) {
    dBm = -10.2f;

  } else if (dBm < -10.f) {
    dBm = -10.1f;
  }

  pwrIdx = (uint8_t) ((dBm + 10.2f) * 10.f + 0.5f);
  pwrReg = s_ax_pwr_ary[pwrIdx];

  /* Set power register value */
#if 0
  s_ax_spi_packet_buffer[0] = 0xF1;                             // WR Address 0x16A: TXPWRCOEFFB
  s_ax_spi_packet_buffer[1] = 0x6A;
  s_ax_spi_packet_buffer[2] = (uint8_t) (pwrReg >> 8);
  s_ax_spi_packet_buffer[3] = (uint8_t) (pwrReg  & 0xff);

  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 4)
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif
}

void spi_ax_setPwrMode(AX_SET_REGISTERS_POWERMODE_t powerState)
{
  /* XOEN and REFEN when oscillator active power modes */
  powerState &= 0x0f;
  if (powerState & 0b1100) {
    powerState |= 0x60;
  }

  /* Prepare packet */
#if 0
  g_ax_spi_packet_buffer[0] = 0x02 | 0x80;                            // WR address 0x02: PWRMODE
  g_ax_spi_packet_buffer[1] = (uint8_t)powerState;

  /* Send packet */
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif
}

void spi_ax_setRegisters(uint8_t doReset, AX_SET_REGISTERS_MODULATION_t modulation, AX_SET_REGISTERS_VARIANT_t variant, AX_SET_REGISTERS_POWERMODE_t powerState)
{
  static AX_SET_REGISTERS_MODULATION_t s_modulation = AX_SET_REGISTERS_MODULATION_NONE;
  static AX_SET_REGISTERS_VARIANT_t    s_variant    = AX_SET_REGISTERS_VARIANT_NONE;
  static AX_SET_REGISTERS_POWERMODE_t  s_powerState = AX_SET_REGISTERS_POWERMODE_NONE;
  //status_code_t sc;

  /* Do not access the registers when not enabled */
  if (!s_ax_enable) {
    return;
  }

  /* Allow to reset to unknown state */
  if (modulation == AX_SET_REGISTERS_MODULATION_INVALIDATE) {
    s_modulation = AX_SET_REGISTERS_MODULATION_NONE;
  }
  if (variant == AX_SET_REGISTERS_VARIANT_INVALIDATE) {
    s_variant = AX_SET_REGISTERS_VARIANT_NONE;
  }

  /* Sanity check */
  if (powerState == AX_SET_REGISTERS_POWERMODE_NONE) {
    return;
  }

  /* Entering DEEPSLEEP - all register contents are lost */
  if (powerState == AX_SET_REGISTERS_POWERMODE_DEEPSLEEP) {
    //spi_ax_ISR_setFlags(0x00);

    s_modulation  = AX_SET_REGISTERS_MODULATION_NONE;
    s_variant   = AX_SET_REGISTERS_VARIANT_NONE;
    s_powerState  = powerState;

    spi_ax_setPwrMode(powerState);
    return;
  }

  if (doReset) {
    //spi_ax_ISR_setFlags(0x00);

    s_modulation  = AX_SET_REGISTERS_MODULATION_NONE;
    s_variant   = AX_SET_REGISTERS_VARIANT_NONE;
    s_powerState  = AX_SET_REGISTERS_POWERMODE_POWERDOWN;

    //sc = spi_ax_sync2Powerdown();
    //if (sc != STATUS_OK) {
    //  s_ax_enable = 0U;
    //}
  }

  /* Copy when NO_CHANGE */
  {
    if ((modulation == AX_SET_REGISTERS_MODULATION_NO_CHANGE) && (s_modulation != AX_SET_REGISTERS_MODULATION_NONE)) {
      modulation = s_modulation;
    }

    if ((variant == AX_SET_REGISTERS_VARIANT_NO_CHANGE)     && (s_variant != AX_SET_REGISTERS_VARIANT_NONE)) {
      variant = s_variant;
    }
  }

  /* No change, leave */
  if ((s_modulation == modulation) && (s_variant == variant) && (s_powerState == powerState)) {
    return;
  }

  /* Register file modifications */
  if (((modulation != AX_SET_REGISTERS_MODULATION_INVALIDATE) && (s_modulation != modulation)) ||
    ((variant    != AX_SET_REGISTERS_VARIANT_INVALIDATE)    && (s_variant    != variant   ))) {

    /* Go to POWERDOWN mode for register file modifications */
    if (s_powerState != AX_SET_REGISTERS_POWERMODE_POWERDOWN) {
      //spi_ax_ISR_setFlags(0x00);
      s_powerState  = AX_SET_REGISTERS_POWERMODE_POWERDOWN;
      spi_ax_setPwrMode(s_powerState);
    }

    /* Overwrite with modulation settings */
    switch (modulation) {
      case AX_SET_REGISTERS_MODULATION_FSK:
      {
        if (s_modulation != modulation) {
          s_modulation  = AX_SET_REGISTERS_MODULATION_FSK;
          s_variant   = AX_SET_REGISTERS_VARIANT_NONE;
        }

        /* Overwrite with variant settings */
        if (s_variant != variant) {
          s_variant  = variant;

          switch (variant) {
            case AX_SET_REGISTERS_VARIANT_TX:
            {
              spi_ax_initRegisters_FSK();
              spi_ax_initRegisters_FSK_Tx();
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX:
            {
              spi_ax_initRegisters_FSK();
              spi_ax_initRegisters_FSK_Rx();
            }
            break;

            default:
            {
              /* Nothing to be set */
            }
          }  // switch (variant)
        }  // if (s_variant != variant)
      }  // case AX_SET_REGISTERS_MODULATION_FSK
      break;

      case AX_SET_REGISTERS_MODULATION_PR1200:
      {
        if (s_modulation != modulation) {
          s_modulation  = AX_SET_REGISTERS_MODULATION_PR1200;
          s_variant   = AX_SET_REGISTERS_VARIANT_NONE;
        }

        /* Overwrite with variant settings */
        if (s_variant != variant) {
          s_variant = variant;

          switch (variant) {
            case AX_SET_REGISTERS_VARIANT_TX:
            {
              spi_ax_initRegisters_PR1200();
              spi_ax_initRegisters_PR1200_Tx();
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX_WOR:
            {
              spi_ax_initRegisters_PR1200();
              spi_ax_initRegisters_PR1200_Rx();
              spi_ax_initRegisters_PR1200_Rx_WoR();
              //spi_ax_ISR_setFlags(0x01);
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX_CONT:
            case AX_SET_REGISTERS_VARIANT_RX:
            {
              spi_ax_initRegisters_PR1200();
              spi_ax_initRegisters_PR1200_Rx();
              spi_ax_initRegisters_PR1200_Rx_cont();
              //spi_ax_ISR_setFlags(0x01);
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX_CONT_SINGLEPARAMSET:
            {
              spi_ax_initRegisters_PR1200();
              spi_ax_initRegisters_PR1200_Rx();
              spi_ax_initRegisters_PR1200_Rx_cont();
              spi_ax_initRegisters_PR1200_Rx_cont_SingleParamSet();
              //spi_ax_ISR_setFlags(0x01);
            }
            break;

            default:
            {
              /* Nothing to be set */
            }
          }  // switch (variant)
        }  // if (s_variant != variant)
      }  // case AX_SET_REGISTERS_MODULATION_PR1200
      break;

      case AX_SET_REGISTERS_MODULATION_POCSAG:
      {
        if (s_modulation != modulation) {
          s_modulation  = AX_SET_REGISTERS_MODULATION_POCSAG;
          s_variant   = AX_SET_REGISTERS_VARIANT_NONE;
        }

        /* Overwrite with variant settings */
        if (s_variant != variant) {
          s_variant = variant;

          switch (variant) {
            case AX_SET_REGISTERS_VARIANT_TX:
            {
              spi_ax_initRegisters_POCSAG();
              spi_ax_initRegisters_POCSAG_Tx();
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX_WOR:
            {
              spi_ax_initRegisters_POCSAG();
              spi_ax_initRegisters_POCSAG_Rx();
              spi_ax_initRegisters_POCSAG_Rx_WoR();
              //spi_ax_ISR_setFlags(0x01);
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX_CONT:
            case AX_SET_REGISTERS_VARIANT_RX:
            {
              spi_ax_initRegisters_POCSAG();
              spi_ax_initRegisters_POCSAG_Rx();
              spi_ax_initRegisters_POCSAG_Rx_cont();
              //spi_ax_ISR_setFlags(0x01);
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX_CONT_SINGLEPARAMSET:
            {
              spi_ax_initRegisters_POCSAG();
              spi_ax_initRegisters_POCSAG_Rx();
              spi_ax_initRegisters_POCSAG_Rx_cont();
              //spi_ax_initRegisters_POCSAG_Rx_cont_SingleParamSet();
              //spi_ax_ISR_setFlags(0x01);
            }
            break;

            default:
            {
              /* Nothing to be set */
            }
          }  // switch (variant)
        }  // if (s_variant != variant)
      }
      break;

      case AX_SET_REGISTERS_MODULATION_ANALOG_FM:
      {
        if (s_modulation != modulation) {
          s_modulation  = AX_SET_REGISTERS_MODULATION_ANALOG_FM;
          s_variant   = AX_SET_REGISTERS_VARIANT_NONE;
        }

        /* Overwrite with variant settings */
        if (s_variant != variant) {
          s_variant  = variant;

          switch (variant) {
            case AX_SET_REGISTERS_VARIANT_TX:
            {
              spi_ax_initRegisters_AnlogFM();
              spi_ax_initRegisters_AnlogFM_Tx();
            }
            break;

            case AX_SET_REGISTERS_VARIANT_RX:
            case AX_SET_REGISTERS_VARIANT_RX_CONT:
            case AX_SET_REGISTERS_VARIANT_RX_WOR:
            case AX_SET_REGISTERS_VARIANT_RX_CONT_SINGLEPARAMSET:
            {
              spi_ax_initRegisters_AnlogFM();
              spi_ax_initRegisters_AnlogFM_Rx();
            }
            break;

            default:
            {
              /* Nothing to be set */
            }
          }  // switch (variant)
        }  // if (s_variant != variant)
      }
      break;

      default:
      {
        /* Nothing to be set */
      }
    }  // switch (modulation)
  }


  /* Finally enter desired powerState */
  switch (powerState) {
    case AX_SET_REGISTERS_POWERMODE_WOR:
    case AX_SET_REGISTERS_POWERMODE_DEEPSLEEP:
    case AX_SET_REGISTERS_POWERMODE_POWERDOWN:
    {
      if (s_powerState != powerState) {
        s_powerState  = powerState;
        spi_ax_setPwrMode(powerState);
      }
      return;
    }
    break;

    case AX_SET_REGISTERS_POWERMODE_FIFO:
    case AX_SET_REGISTERS_POWERMODE_STANDBY:
    {
      if (s_powerState != powerState) {
        s_powerState  = powerState;
        spi_ax_setPwrMode(s_powerState);
        spi_ax_xtal_waitReady();  // T_xtal
      }
      return;
    }
    break;

    case AX_SET_REGISTERS_POWERMODE_SYNTHRX:
    {
      switch (s_powerState) {
        case AX_SET_REGISTERS_POWERMODE_POWERDOWN:
        {
          s_powerState  = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          spi_ax_xtal_waitReady();  // T_xtal

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_STANDBY:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHRX:
        {
          // unused
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_FULLRX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          // ?
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHTX:
        case AX_SET_REGISTERS_POWERMODE_FULLTX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          // ?

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth
        }
        break;

        default: { }
      }  // switch (s_powerState)
    }
    break;

    case AX_SET_REGISTERS_POWERMODE_FULLRX:
    {
      switch (s_powerState) {
        case AX_SET_REGISTERS_POWERMODE_POWERDOWN:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          spi_ax_xtal_waitReady();  // T_xtal

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth

          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLRX;
          spi_ax_setPwrMode(s_powerState);
          //osDelay(5);  // T_rx_rssi
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_STANDBY:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth

          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLRX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(5);  // T_rx_rssi
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHRX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLRX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(5);  // T_rx_rssi
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_FULLRX:
        {
          // unused
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHTX:
        case AX_SET_REGISTERS_POWERMODE_FULLTX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          // ?

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHRX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth

          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLRX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(5);  // T_rx_rssi
        }
        break;

        default: { }
      }  // switch (s_powerState)
    }
    break;

    case AX_SET_REGISTERS_POWERMODE_SYNTHTX:
    {
      switch (s_powerState) {
        case AX_SET_REGISTERS_POWERMODE_POWERDOWN:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          spi_ax_xtal_waitReady();  // T_xtal

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_STANDBY:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHRX:
        case AX_SET_REGISTERS_POWERMODE_FULLRX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          // ?

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHTX:
        {
          // unused
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_FULLTX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLTX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(1);  // T_tx_on
        }
        break;

        default: { }
      }  // switch (s_powerState)
    }
    break;

    case AX_SET_REGISTERS_POWERMODE_FULLTX:
    {
      switch (s_powerState) {
        case AX_SET_REGISTERS_POWERMODE_POWERDOWN:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          spi_ax_xtal_waitReady();  // T_xtal

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth

          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLTX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(1);  // T_tx_on
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_STANDBY:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth

          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLTX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(1);  // T_tx_on
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHRX:
        case AX_SET_REGISTERS_POWERMODE_FULLRX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_STANDBY;
          spi_ax_setPwrMode(s_powerState);
          // ?

          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          //delay_us(40);  // T_synth

          s_powerState = AX_SET_REGISTERS_POWERMODE_FULLTX;
          spi_ax_setPwrMode(s_powerState);
          osDelay(1);  // T_tx_on
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_SYNTHTX:
        {
          s_powerState = AX_SET_REGISTERS_POWERMODE_SYNTHTX;
          spi_ax_setPwrMode(s_powerState);
          // ?
        }
        break;

        case AX_SET_REGISTERS_POWERMODE_FULLTX:
        {
          // unused
        }
        break;

        default: { }
      }  // switch (s_powerState)
    }
    break;

    default: { }
  }  // switch (powerState)
}


uint32_t spi_ax_calcFrequency_Mhz2Regs(float f_mhz)
{
  const float xtal_hz = 16E+6f;                               // XTAL = 16 MHz
  const float reg_per_mhz = (1LL << 24) * 1E+6f / xtal_hz;
  return (uint32_t) (0.5f + f_mhz * reg_per_mhz);
}

float spi_ax_calcFrequency_Regs2MHz(uint32_t vco_regval)
{
  const float xtal_hz = 16E+6f;                               // XTAL = 16 MHz
  const float reg_per_mhz = (1LL << 24) * 1E+6f / xtal_hz;
  return vco_regval / reg_per_mhz;
}

void spi_ax_setFrequency2Regs(uint8_t chan, uint8_t isFreqB)
{
  uint32_t f_reg;  (void) f_reg;

  if (chan > 2) {
    /* ERROR */
    return;
  }
  f_reg = s_ax_spi_freq_chan[chan];

  /* Make range value non-valid again */
  s_ax_spi_range_chan[chan] = C_SPI_AX_RANGE_NOT_SET;

  /* Prepare packet */
#if 0
  g_ax_spi_packet_buffer[0] = (isFreqB ?  0x3C : 0x34) | 0x80;                // WR Address 0x34 or 0x3C
  g_ax_spi_packet_buffer[1] = (uint8_t) (f_reg >> 24) & 0xff;
  g_ax_spi_packet_buffer[2] = (uint8_t) (f_reg >> 16) & 0xff;
  g_ax_spi_packet_buffer[3] = (uint8_t) (f_reg >>  8) & 0xff;
  g_ax_spi_packet_buffer[4] = (uint8_t) (f_reg      ) & 0xff;

  /* Send packet */
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 5);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif
}

void spi_ax_doRanging(void)
{
  static uint32_t sf_ax_spi_freq_chan[2] = { 0x28, 0x28 };

  if ((0x00 <= s_ax_spi_range_chan[0]) && (s_ax_spi_range_chan[0] <= 0x0f)  &&  (s_ax_spi_freq_chan[0] == s_ax_spi_freq_chan[0])  &&
      (0x00 <= s_ax_spi_range_chan[1]) && (s_ax_spi_range_chan[1] <= 0x0f)  &&  (s_ax_spi_freq_chan[1] == s_ax_spi_freq_chan[1])) {

    spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

    /* Recall the ranging values for both frequencies FREQA and FREQB */
    for (uint8_t idx = 0; idx < 2; idx++) {
      uint8_t regAdr = idx ?  0x3b : 0x33;  (void) regAdr;                        // WR Address 0x33: PLLRANGINGA  or  Address 0x3b: PLLRANGINGB
      uint8_t val = s_ax_spi_range_chan[idx];  (void) val;

#if 0
      s_ax_spi_packet_buffer[0] = regAdr | 0x80;
      s_ax_spi_packet_buffer[1] = val  & 0x0f;
      spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
      spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
      spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif
    }

  } else {
    /* Do automatic ranging */

    /* POWERMODE STANDBY */
    spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_FSK, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_STANDBY);

    /* MODULATION */
    //spi_ax_transport(0U, "< 90 08 >");                         // WR address 0x10: MODULATION - 08: FSK

    /* Default 100kHz loop BW for ranging */
    /* PLLLOOP */
    //spi_ax_transport(0U, "< b0 09 >");                         // WR address 0x30: PLLLOOP - DIRECT: Bypass External Filter Pin, FLT: Internal Loop Filter x1 --> BW = 100 kHz for I_CP = 68 �A

    /* PLLCPI */
    //spi_ax_transport(0U, "< b1 08 >");                         // WR address 0x31: PLLCPI - PLLCPI: Charge pump current in multiples of 8.5 킕 --> I_CP = 68 �A

    /* FSKDEV */
    //spi_ax_transport(0U, "< f1 61 00 00 00 >");                      // WR address 0x161: FSKDEV - off

    for (uint8_t idx = 0; idx < 2; idx++) {
      if ((0x10 <= s_ax_spi_range_chan[idx]) || (sf_ax_spi_freq_chan[idx] != s_ax_spi_freq_chan[idx]))
      {
        uint8_t regAdr = idx ?  0x3b : 0x33;  (void) regAdr;                      // WR Address 0x33: PLLRANGINGA  or  Address 0x3b: PLLRANGINGB

        /* Switch to VCO1 or VCO2 as needed */
        (void) spi_ax_vco_select(s_ax_spi_freq_chan[idx], 1U);

        /* Command message */
#if 0
        g_ax_spi_packet_buffer[0] = regAdr | 0x80;
        g_ax_spi_packet_buffer[1] = 0x18;                       // Bit 4: RNGSTART, ranging start value = 8
        spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
        spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif

        /* Wait until ranging has ended */
#if 0
        do {
          /* Request message */
          g_ax_spi_packet_buffer[0] = regAdr;
          spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
          spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 1);
          spi_read_packet(&SPI_AX, g_ax_spi_packet_buffer, 1);
          spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
        } while (g_ax_spi_packet_buffer[0] & _BV(4));                 // Bit 4: RNGSTATUS
#endif

        /* Ranging value to be stored - when error occurs store that result, also */
        s_ax_spi_range_chan[idx]  = s_ax_spi_packet_buffer[0];
        sf_ax_spi_freq_chan[idx]  = s_ax_spi_freq_chan[idx];

        /* When ranging has failed set default ranging value to keep VCO stable */
#if 0
        if (g_ax_spi_packet_buffer[0] & _BV(5)) {
          /* Command message */
          g_ax_spi_packet_buffer[0] = regAdr | 0x80;
          g_ax_spi_packet_buffer[1] = 0x08;                     // Ranging default value = 8
          spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
          spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
          spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
        }
#endif
      }
    }

    /* VCOI Manual calibration when GPADC13 is attached to the VCO control voltage, available at FILT in external loop filter mode */
    #if defined(AX_GPADC13_ENABLED)
    {
      spi_ax_initRegisters_PR1200_Tx();

      /* MODULATION */
      spi_ax_transport(0U, "< 90 08 >");                       // WR address 0x10: MODULATION - 08: FSK

      /* FSKDEV */
      spi_ax_transport(0U, "< f1 61 00 00 00 >");                    // WR address 0x161: FSKDEV - off

      /* PLLLOOP */
      {
        spi_ax_transport(0U, "< 30 R1 >");                     // RD address 0x38: PLLLOOP - 04: FILTEN
        g_ax_spi_packet_buffer[1] = 0x04 | g_ax_spi_packet_buffer[0];
        g_ax_spi_packet_buffer[0] = 0x30 | 0x80;

        spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
        spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);             // Write back with modified value
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
      }

      /* 0xF35 */
      {
        spi_ax_transport(0U, "< 7f 35 R1 >");                    // RD address 0xF35 (RX/TX) - Set to 0x10 for reference frequencies (crystal or TCXO) less than 24.8 MHz (fXTALDIV = 1), or to 0x11 otherwise (fXTALDIV = 2)
        uint8_t new_0xF35 = 0x80 | g_ax_spi_packet_buffer[0];

        if (0x02 & (uint8_t)~new_0xF35) {
          ++new_0xF35;
        }

        g_ax_spi_packet_buffer[0] = 0xFF;
        g_ax_spi_packet_buffer[1] = 0x35;
        g_ax_spi_packet_buffer[2] = new_0xF35;

        spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
        spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 3);             // Write back with modified value
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
      }

      spi_ax_transport(0U, "< 82 6c >");                       // WR Address 0x02: PWRMODE - XOEN, REFEN, PWRMODE=SYNTH_TX

      {
        /* PLLVCOI */
        spi_ax_transport(0U, "< 71 80 R1 >");                    // RD address 0x180: PLLVCOI
        uint8_t vcoi_save = g_ax_spi_packet_buffer[0];

        uint8_t dropFirstCalIdx = 2;
        for (uint8_t chanIdx = 0; chanIdx < 2; chanIdx++) {
          g_ax_spi_vcoi_chan[chanIdx] = 0;

          if (g_ax_spi_range_chan[chanIdx] & 0x20) {
            continue;
          }

          g_ax_spi_packet_buffer[0] = 0x33 | 0x80;                  // WR Address 0x33: PLLRANGINGA
          g_ax_spi_packet_buffer[1] = g_ax_spi_range_chan[0] & 0x0f;

          spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
          spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
          spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

          spi_ax_setFrequency2Regs(chanIdx, chanIdx == 1 ?  1U : 0U);

          do {
            g_ax_spi_vcoi_chan[chanIdx] = s_spi_ax_cal_vcoi();
          } while (--dropFirstCalIdx);
          dropFirstCalIdx = 1;
        }

        /* Revert to initial setting */
        g_ax_spi_packet_buffer[0] = 0x71 | 0x80;                    // WR Address 0x180: PLLVCOI
        g_ax_spi_packet_buffer[1] = 0x80;
        g_ax_spi_packet_buffer[2] = vcoi_save;

        spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
        spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
      }
    }
    #endif  // VCOI Calibration

    /* POWERMODE POWERDOWN */
    spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);
  }
}

uint8_t spi_ax_vco_select(uint32_t reg_freq, uint8_t force)
{
  float f_Mhz = spi_ax_calcFrequency_Regs2MHz(reg_freq);
  uint8_t curVco2 = 0U;
  uint8_t modified = 0U;

  /* Read VCO settings */
  {
    /* PLLVCODIV */
#if 0
    spi_ax_transport(0U, "< 32 R1 >");                         // RD address 0x32: PLLVCODIV - VCO2INT, VCOSEL, RFDIV
    curVco2 = (g_ax_spi_packet_buffer[0] & 0x20) ?  1U : 0U;
#endif
  }

  /* VCO1 ranges abt. 380 MHz ... 540 MHz (RFDIV=1)  and  760 MHz ... 1080 MHz (RFDIV=0) */
  if (f_Mhz >= 760.f) {
    modified = 1U;

    /* PLLVCODIV - mind you: check 0xF34, also */
    //spi_ax_transport(0U, "< b2 00 >");                         // WR address 0x32: PLLVCODIV - no RFDIV

    /* 0xF34 */
    //spi_ax_transport(0U, "< ff 34 08 >");                        // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise
  }

  else if (f_Mhz >= 380.f) {
    /* Select VCO1 */
    if (curVco2 || force) {
      modified = 1U;

      /* PLLVCODIV - mind you: check 0xF34, also */
      //spi_ax_transport(0U, "< b2 04 >");                       // WR address 0x32: PLLVCODIV - RFDIV

      /* 0xF34 */
      //spi_ax_transport(0U, "< ff 34 28 >");                      // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise
    }
  }

  else {
    /* Select VCO2 */
    if (!curVco2 || force) {
      modified = 1U;

      /* PLLVCODIV - mind you: check 0xF34, also */
      //spi_ax_transport(0U, "< b2 34 >");                       // WR address 0x32: PLLVCODIV - VCO2INT, VCOSEL, RFDIV

      /* 0xF34 */
      //spi_ax_transport(0U, "< ff 34 28 >");                      // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise
    }
  }

  return modified;
}

uint8_t spi_ax_selectVcoFreq(uint8_t isFreqB)
{
  /* Switch to VCO1 or VCO2 as needed */
  uint8_t modified = spi_ax_vco_select(s_ax_spi_freq_chan[isFreqB ?  1 : 0], 0U);

  /* PLLLOOP */
#if 0
  spi_ax_transport(0U, "< 30 R1 >");                           // RD address 0x30: PLLLOOP
  g_ax_spi_packet_buffer[1] = (isFreqB ?  (g_ax_spi_packet_buffer[0] | 0x80)          // Set   FREQB
                     :  (g_ax_spi_packet_buffer[0] & 0x7f));        // Clear FREQB
  g_ax_spi_packet_buffer[0] = 0x30 | 0x80;                          // WR address 0x30: PLLLOOP

  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 2);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif

  return modified;
}

void spi_ax_util_FIFO_waitFree(uint8_t neededSpace)
{
  uint16_t fifoFree;  (void) fifoFree;

#if 0
  do {
    /* FIFOFREE */
    spi_ax_transport(0U, "< 2c R2 >");                         // RD address 0x2C: FIFOFREE
    fifoFree = 0x1ff & (((uint16_t)g_ax_spi_packet_buffer[0] << 8) | g_ax_spi_packet_buffer[1]);
  } while (fifoFree < neededSpace);
#endif
}

void spi_ax_setRxMode_by_MonMode(void)
{
  /* Leave when AX5243 is disabled */
  if (!s_ax_enable) {
    s_ax_set_mon_mode = AX_SET_MON_MODE_OFF;
    return;
  }

  AX_SET_MON_MODE_t l_ax_set_mon_mode = s_ax_set_mon_mode;

  switch (l_ax_set_mon_mode) {
    case AX_SET_MON_MODE_APRS_RX_WOR:
    case AX_SET_MON_MODE_APRS_RX_CONT:
    case AX_SET_MON_MODE_APRS_RX_CONT_SINGLEPARAMSET:
      spi_ax_setTxRxMode((AX_SET_TX_RX_MODE_t)l_ax_set_mon_mode);
      break;

    case AX_SET_MON_MODE_POCSAG_RX_WOR:
    case AX_SET_MON_MODE_POCSAG_RX_CONT:
    case AX_SET_MON_MODE_POCSAG_RX_CONT_SINGLEPARAMSET:
      spi_ax_setTxRxMode((AX_SET_TX_RX_MODE_t)l_ax_set_mon_mode);
      break;

    case AX_SET_MON_MODE_OFF:
    default:
      spi_ax_setTxRxMode(AX_SET_TX_RX_MODE_OFF);
  }
}


void spi_ax_initRegisters_FSK(void)
{
  /* MODULATION */
#if 0
  spi_ax_transport(0U, "< 90 08 >");                           // WR address 0x10: MODULATION - 08: FSK

  /* ENCODING */
  spi_ax_transport(0U, "< 91 00 >");                           // WR address 0x11: ENCODING - normal

  /* FRAMING */
  spi_ax_transport(0U, "< 92 00 >");                           // WR address 0x12: FRAMING - off

  /* PINFUNCTCXO_EN */
  #if 0
  spi_ax_transport(0U, "< a6 05 >");                           // WR address 0x26: PINFUNCTCXO_EN - Use TCXO_EN pin as DAC output
  #else
  spi_ax_transport(0U, "< a6 00 >");                           // WR address 0x26: PINFUNCTCXO_EN - Set to output '0'
  #endif

  /* WAKEUPXOEARLY */
  spi_ax_transport(0U, "< ee 01 >");                           // WR address 0x6E: WAKEUPXOEARLY - 1 LPOSC cycle wake-up time before receiver is started


  /* IFFREQ */
  spi_ax_transport(0U, "< f1 00 00 cd >");                         // WR address 0x100: IFFREQ - IFFREQ: 3,128 Hz (f_xtal = 16 MHz)

  /* DECIMATION */
  spi_ax_transport(0U, "< f1 02 25 >");                          // WR address 0x102: DECIMATION - DECIMATION: 37d, f_BASEBAND = 27,027.03 Hz

  /* RXDATARATE */
  spi_ax_transport(0U, "< f1 03 00 b4 2e >");                        // WR address 0x103: RXDATARATE - 1,200 bit/s

  /* MAXDROFFSET */
  spi_ax_transport(0U, "< f1 06 00 00 00 >");                        // WR address 0x106: MAXDROFFSET - off

  /* MAXRFOFFSET */
  spi_ax_transport(0U, "< f1 09 80 07 5f >");                        // WR address 0x109: MAXRFOFFSET - FREQOFFSCORR: Correct frequency offset at the first LO if this bit is one, MAXRFOFFSET: +/- 1,799.6 Hz

  /* FSKDMAX */
  spi_ax_transport(0U, "< f1 0c 03 f3 >");                         // WR address 0x10C: FSKDMAX - FSKDMAX: +1,011d (should be +640d?, seems to be moved by abt. 1.5x up)

  /* FSKDMIN */
  spi_ax_transport(0U, "< f1 0e ff 0d >");                         // WR address 0x10E: FSKDMIN - FSKDMIN: -243d (should be -640d?, seems to be moved by abt. 1.5x up)

  /* AMPLFILTER */
  spi_ax_transport(0U, "< f1 15 00 >");                          // WR address 0x115: AMPLFILTER - AMPLFILTER: filter bypassed.

  #if 0
  /* FREQUENCYLEAK */
  spi_ax_transport(0U, "< f1 16 00 >");                          // WR address 0x116: FREQUENCYLEAK
  #endif

  /* RXPARAMSETS */
  spi_ax_transport(0U, "< f1 17 f4 >");                          // WR address 0x117: RXPARAMSETS - RXPS0: 0, RXPS1: 1, RXPS2: 3, RXPS3: 3

  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 e8 >");                          // WR address 0x120: AGCGAIN0 - AGCATTACK0: 8, AGCDECAY0: 14

  /* AGCTARGET0 */
  spi_ax_transport(0U, "< f1 21 84 >");                          // WR address 0x121: AGCTARGET0 - average AGC magnitude = 304

  /* TIMEGAIN0 */
  spi_ax_transport(0U, "< f1 24 ba >");                          // WR address 0x124: TIMEGAIN0

  /* DRGAIN0  0xB4 */
  spi_ax_transport(0U, "< f1 25 b4 >");                          // WR address 0x125: DRGAIN0

  /* PHASEGAIN0 */
  spi_ax_transport(0U, "< f1 26 c3 >");                          // WR address 0x126: PHASEGAIN0

  /* FREQGAINA0 */
  spi_ax_transport(0U, "< f1 27 0f >");                          // WR address 0x127: FREQGAINA0

  /* FREQGAINB0 */
  spi_ax_transport(0U, "< f1 28 1f >");                          // WR address 0x128: FREQGAINB0

  /* FREQGAINC0 */
  spi_ax_transport(0U, "< f1 29 0a >");                          // WR address 0x129: FREQGAINC0

  /* FREQGAIND0 */
  spi_ax_transport(0U, "< f1 2a 0a >");                          // WR address 0x12A: FREQGAIND0

  /* AMPLGAIN0 */
  spi_ax_transport(0U, "< f1 2b 06 >");                          // WR address 0x12B: AMPLGAIN0

  /* FREQDEV0 */
  spi_ax_transport(0U, "< f1 2c 00 00 >");                         // WR address 0x12C: FREQDEV0

  /* BBOFFSRES0 */
  spi_ax_transport(0U, "< f1 2f 00 >");                          // WR address 0x12F: BBOFFSRES0


  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 e8 >");                          // WR address 0x130: AGCGAIN1

  /* AGCTARGET1 */
  spi_ax_transport(0U, "< f1 31 84 >");                          // WR address 0x131: AGCTARGET1

  /* AGCAHYST1 */
  spi_ax_transport(0U, "< f1 32 00 >");                          // WR address 0x132: AGCAHYST1

  /* AGCMINMAX1 */
  spi_ax_transport(0U, "< f1 33 00 >");                          // WR address 0x133: AGCMINMAX1

  /* TIMEGAIN1 */
  spi_ax_transport(0U, "< f1 34 b8 >");                          // WR address 0x134: TIMEGAIN1

  /* DRGAIN1 */
  spi_ax_transport(0U, "< f1 35 b3 >");                          // WR address 0x135: DRGAIN1

  /* PHASEGAIN1 */
  spi_ax_transport(0U, "< f1 36 c3 >");                          // WR address 0x136: PHASEGAIN1

  /* FREQGAINA1 */
  spi_ax_transport(0U, "< f1 37 0f >");                          // WR address 0x137: FREQGAINA1

  /* FREQGAINB1 */
  spi_ax_transport(0U, "< f1 38 1f >");                          // WR address 0x138: FREQGAINB1

  /* FREQGAINC1 */
  spi_ax_transport(0U, "< f1 39 0a >");                          // WR address 0x139: FREQGAINC1

  /* FREQGAIND1 */
  spi_ax_transport(0U, "< f1 3a 0a >");                          // WR address 0x13A: FREQGAIND1

  /* AMPLGAIN1 */
  spi_ax_transport(0U, "< f1 3b 06 >");                          // WR address 0x13B: AMPLGAIN1

  /* FREQDEV1 */
  spi_ax_transport(0U, "< f1 3c 00 4b >");                         // WR address 0x13C: FREQDEV1

  /* FOURFSK1 */
  spi_ax_transport(0U, "< f1 3e 16 >");                          // WR address 0x13E: FOURFSK1

  /* BBOFFSRES1 */
  spi_ax_transport(0U, "< f1 3f 00 >");                          // WR address 0x13F: BBOFFSRES1


  /* AGCGAIN3 */
  spi_ax_transport(0U, "< f1 50 ff >");                          // WR address 0x150: AGCGAIN3

  /* AGCTARGET3 */
  spi_ax_transport(0U, "< f1 51 84 >");                          // WR address 0x151: AGCTARGET3

  /* AGCAHYST3 */
  spi_ax_transport(0U, "< f1 52 00 >");                          // WR address 0x152: AGCAHYST3

  /* AGCMINMAX3 */
  spi_ax_transport(0U, "< f1 53 00 >");                          // WR address 0x153: AGCMINMAX3

  /* TIMEGAIN3 */
  spi_ax_transport(0U, "< f1 54 b8 >");                          // WR address 0x154: TIMEGAIN3

  /* DRGAIN3 */
  spi_ax_transport(0U, "< f1 55 b3 >");                          // WR address 0x155: DRGAIN3

  /* PHASEGAIN3 */
  spi_ax_transport(0U, "< f1 56 c3 >");                          // WR address 0x156: PHASEGAIN3

  /* FREQGAINA3 */
  spi_ax_transport(0U, "< f1 57 0f >");                          // WR address 0x157: FREQGAINA3

  /* FREQGAINB3 */
  spi_ax_transport(0U, "< f1 58 1f >");                          // WR address 0x158: FREQGAINB3

  /* FREQGAINC3 */
  spi_ax_transport(0U, "< f1 59 0d >");                          // WR address 0x159: FREQGAINC3

  /* FREQGAIND3 */
  spi_ax_transport(0U, "< f1 5a 0d >");                          // WR address 0x15A: FREQGAIND3

  /* AMPLGAIN3 */
  spi_ax_transport(0U, "< f1 5b 06 >");                          // WR address 0x15B: AMPLGAIN3

  /* FREQDEV3 */
  spi_ax_transport(0U, "< f1 5c 00 4b >");                         // WR address 0x15C: FREQDEV3

  /* FOURFSK3 */
  spi_ax_transport(0U, "< f1 5e 16 >");                          // WR address 0x15E: FOURFSK3

  /* BBOFFSRES3 */
  spi_ax_transport(0U, "< f1 5f 00 >");                          // WR address 0x15F: BBOFFSRES3 - not used


  /* MODCFGF */
  spi_ax_transport(0U, "< f1 60 00 >");                          // WR address 0x160: MODCFGF - FREQSHAPE: External Loop Filter (BT = 0.0)

  /* FSKDEV */
  spi_ax_transport(0U, "< f1 61 00 02 0c >");                        // WR address 0x161: FSKDEV - FSKDEV: 500 Hz @ fxtal = 16 MHz.


  /* MODCFGA */
  spi_ax_transport(0U, "< f1 64 05 >");                          // WR address 0x164: MODCFGA - AMPLSHAPE, TXDIFF

  /* TXRATE */
  spi_ax_transport(0U, "< f1 65 00 04 ea >");                        // WR address 0x165: TXRATE - TXRATE: 1,200 bit/s

  /* TXPWRCOEFFA */
  spi_ax_transport(0U, "< f1 68 00 00 >");                         // WR address 0x168: TXPWRCOEFFA - no correction

  /* TXPWRCOEFFB */
  spi_ax_transport(0U, "< f1 6a 00 00 >");                         // WR address 0x16A: TXPWRCOEFFB - off

  /* TXPWRCOEFFC */
  spi_ax_transport(0U, "< f1 6c 00 00 >");                         // WR address 0x16C: TXPWRCOEFFC - no correction

  /* TXPWRCOEFFD */
  spi_ax_transport(0U, "< f1 6e 00 00 >");                         // WR address 0x16E: TXPWRCOEFFD - no correction

  /* TXPWRCOEFFE */
  spi_ax_transport(0U, "< f1 70 00 00 >");                         // WR address 0x170: TXPWRCOEFFE - no correction


  /* PLLVCOI */
  spi_ax_transport(0U, "< f1 80 99 >");                          // WR address 0x180: PLLVCOI - 10 킕 * 153 = 1,53 mA

  /* PLLRNGCLK */
  spi_ax_transport(0U, "< f1 83 03 >");                          // WR address 0x183: PLLRNGCLK - PLLRNGCLK: 7,812 Hz ? (@see Table 148 in AND9347/D)

  /* BBTUNE */
  spi_ax_transport(0U, "< f1 88 0f >");                          // WR address 0x188: BBTUNE - BBTUNE: 15 (?)

  /* BBOFFSCAP */
  spi_ax_transport(0U, "< f1 89 77 >");                          // WR address 0x189: BBOFFSCAP - CAPINTB: 7, CAPINTA: 7


  /* TMGTXBOOST */
  spi_ax_transport(0U, "< f2 20 32 >");                          // WR address 0x220: TMGTXBOOST - TMGTXBOOSTE = 1, TMGTXBOOSTM = 18 --> 36 탎

  /* TMGTXSETTLE */
  spi_ax_transport(0U, "< f2 21 14 >");                          // WR address 0x221: TMGTXSETTLE - TMGTXSETTLEE = 0, TMGTXSETTLEM = 20 --> 20 탎

  /* TMGRXBOOST */
  spi_ax_transport(0U, "< f2 23 32 >");                          // WR address 0x223: TMGRXBOOST - TMGRXBOOSTE = 1, TMGRXBOOSTM = 18 --> 36 탎

  /* TMGRXSETTLE */
  spi_ax_transport(0U, "< f2 24 14 >");                          // WR address 0x224: TMGRXSETTLE - TMGRXSETTLEE = 0, TMGRXSETTLEM = 20 --> 20 탎

  /* TMGRXOFFSACQ */
  spi_ax_transport(0U, "< f2 25 00 >");                          // WR address 0x225: TMGRXOFFSACQ - TMGRXOFFSACQE = 0, TMGRXOFFSACQM = 0 --> 0 탎

  /* TMGRXCOARSEAGC */
  spi_ax_transport(0U, "< f2 26 73 >");                          // WR address 0x226: TMGRXCOARSEAGC - TMGRXCOARSEAGCE = 3, TMGRXCOARSEAGCM = 19 --> 152 탎 (Bits)  @see PKTMISCFLAGS.RXAGC

  /* TMGRXRSSI */
  spi_ax_transport(0U, "< f2 28 03 >");                          // WR address 0x228: TMGRXRSSI - TMGRXAGCE = 0, TMGRXAGCM = 3 --> 3 탎 (Bits)  @see PKTMISCFLAGS.RXRSSI

  /* TMGRXPREAMBLE1 */
  //spi_ax_transport(0U, "< f2 29 00 >");                          // WR address 0x229: TMGRXPREAMBLE1 - TMGRXPREAMBLE1 timeout = none

  /* TMGRXPREAMBLE2 */
  spi_ax_transport(0U, "< f2 2a 17 >");                          // WR address 0x22A: TMGRXPREAMBLE2 - TMGRXPREAMBLE2 timeout = 7 * 2^1 = 14 bits

  /* TMGRXPREAMBLE3 */
  //spi_ax_transport(0U, "< f2 2b 00 >");                          // WR address 0x22B: TMGRXPREAMBLE3 - TMGRXPREAMBLE3 timeout = none

  /* RSSIREFERENCE */
  spi_ax_transport(0U, "< f2 2c f8 >");                          // WR address 0x22C: RSSIREFERENCE - RSSI Offset, this register adds a constant offset to the computed RSSI value. It is used to compensate for board effects.

  /* RSSIABSTHR */
  spi_ax_transport(0U, "< f2 2d dd >");                          // WR address 0x22D: RSSIABSTHR - RSSIABSTHR > -35 (-64) = -99 dBm (BUSY)

  /* BGNDRSSITHR */
  spi_ax_transport(0U, "< f2 2f 00 >");                          // WR address 0x22F: BGNDRSSITHR - BGNDRSSITHR: off


  /* PKTMISCFLAGS */
  spi_ax_transport(0U, "< f2 31 00 >");                          // WR address 0x231: PKTMISCFLAGS - no BGND RSSI, RXAGC CLK, RXRSSI CLK clock sources: 1 탎


  /* LPOSCCONFIG */
  spi_ax_transport(0U, "< f3 10 12 >");                          // WR address 0x310: LPOSCCONFIG - LPOSC CALIBF 0x10, LPOSC FAST 0x02, (LPOSC ENA 0x01)

  /* LPOSCKFILT */
  spi_ax_transport(0U, "< f3 12 01 5d >");                         // WR address 0x312: LPOSCKFILT - LPOSCKFILT <= 1,398 (lower value gives lower jitter; 1/4 th taken: 349)

  /* LPOSCREF */
  spi_ax_transport(0U, "< f3 14 61 a8 >");                         // WR address 0x314: LPOSCREF - LPOSCREF = 25,000

  /* LPOSCFREQ */
  spi_ax_transport(0U, "< f3 16 00 00 >");                         // WR address 0x316: LPOSCFREQ - no manual tune, done by automatic


  #if 1
  /* DACVALUE */
  spi_ax_transport(0U, "< f3 30 00 0c >");                         // WR address 0x330: DACVALUE - DACSHIFT = 12 bit

  /* DACCONFIG */
  spi_ax_transport(0U, "< f3 32 83 >");                          // WR address 0x332: DACCONFIG - DACPWM, DACINPUT=TRKFREQUENCY
  #endif


  /* 0xF10 - XTALOSC*/
  spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10: XTALOSC

  /* 0xF11 - XTALAMPL */
  spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11: XTALAMPL
#endif
}

void spi_ax_initRegisters_FSK_Tx(void)
{
#if 0
  /* PLLLOOP */
  spi_ax_transport(0U, "< b0 0a >");                           // WR address 0x30: PLLLOOP - DIRECT 0x08, no FILTEN, FLT 02: Internal Loop Filter x2 (BW = 200 kHz for ICP = 272 mA)

  /* PLLLOOPBOOST */
  spi_ax_transport(0U, "< b8 0b >");                           // WR address 0x38: PLLLOOPBOOST - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5 (BW = 500 kHz for ICP = 1.7 mA)
#endif
}

void spi_ax_initRegisters_FSK_Rx(void)
{
#if 0
  /* PLLLOOP */
  spi_ax_transport(0U, "< b0 0a >");                           // WR address 0x30: PLLLOOP - DIRECT 0x08, no FILTEN, FLT 02: Internal Loop Filter x2 (BW = 200 kHz for ICP = 272 mA)

  /* PLLLOOPBOOST */
  spi_ax_transport(0U, "< b8 0b >");                           // WR address 0x38: PLLLOOPBOOST - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5 (BW = 500 kHz for ICP = 1.7 mA)
#endif
}

void spi_ax_init_FSK_Tx(void)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* FSK */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(433.9250);            // VCO1 (internal without ext. L) with RFDIV --> VCORA = 0x09
    /*
    Radiometrix TXL2/RXL2 - 16kbps bi-phase FSK
    433.925MHz - CHAN0
    433.285MHz - CHAN1
    433.605MHz - CHAN2
    434.245MHz - CHAN3
    434.565MHz - CHAN4
    */

    /* POCSAG */
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(439.9875);            // VCO1 (internal without ext. L) with RFDIV --> VCORB = 0x09

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the transmitter settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_FSK, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  #if 1
    /* Set VCO-PLL to FREQA - 433.9250 MHz */
    (void) spi_ax_selectVcoFreq(0U);
  #else
    /* Set VCO-PLL to FREQB - 439.9875 MHz */
    (void) spi_ax_selectVcoFreq(1U);
  #endif

  /* Set power level */
  spi_ax_setPower_dBm(-10.0f);

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the transmitter */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLTX);
}

void spi_ax_init_FSK_Rx(void)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* FSK */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(433.9250);            // VCO1 (internal without ext. L) with RFDIV --> VCORA = 0x09
    /*
    Radiometrix TXL2/RXL2 - 16kbps bi-phase FSK
    433.925MHz - CHAN0
    433.285MHz - CHAN1
    433.605MHz - CHAN2
    434.245MHz - CHAN3
    434.565MHz - CHAN4
    */

    /* POCSAG */
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(439.9875);            // VCO1 (internal without ext. L) with RFDIV --> VCORB = 0x09

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the receiver settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_FSK, AX_SET_REGISTERS_VARIANT_RX_WOR, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  #if 0
    /* Set VCO-PLL to FREQA - 433.9250 MHz */
    (void) spi_ax_selectVcoFreq(0U);
  #else
    /* Set VCO-PLL to FREQB - 439.9875 MHz */
    (void) spi_ax_selectVcoFreq(1U);
  #endif

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the wake-on-radio receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_WOR);
}


void spi_ax_initRegisters_PR1200(void)
{
#if 0
  /* MODULATION */
  spi_ax_transport(0U, "< 90 0a >");                           // WR address 0x10: MODULATION - 0A: AFSK

  /* ENCODING */
  spi_ax_transport(0U, "< 91 03 >");                           // WR address 0x11: ENCODING -  NRZI = ENC DIFF 0x02  &  ENC INV 0x01

  /* FRAMING */
  /* When FRMMODE: "Raw, Soft Bits (0b001 << 1)" is selected, do address 0xF72 also */
  spi_ax_transport(0U, "< 92 14 >");                           // WR address 0x12: FRAMING - CRCMODE: CCITT (16 bit) 0x10 (right for AX.25), FRMMODE: HDLC [1] 0x04      --> with automatic ending CRC, Flag and 16x '1's, 2xSPACE, off
  //spi_ax_transport(0U, "< 92 10 >");                           // WR address 0x12: FRAMING - CRCMODE: CCITT (16 bit) 0x10 (right for AX.25), FRMMODE: Raw 0x00         --> no automatic ending Flag, 2xSPACE, off
  //spi_ax_transport(0U, "< 92 12 >");                           // WR address 0x12: FRAMING - CRCMODE: CCITT (16 bit) 0x10 (right for AX.25), FRMMODE: Raw, Soft Bits 0x02    --> no automatic ending Flag, 2xSPACE, off
  //spi_ax_transport(0U, "< 92 16 >");                           // WR address 0x12: FRAMING - CRCMODE: CCITT (16 bit) 0x10 (right for AX.25), FRMMODE: Raw, Pattern Match 0x06  --> no automatic ending Flag, 2xSPACE, off
  //spi_ax_transport(0U, "< 92 04 >");                           // WR address 0x12: FRAMING - CRCMODE: none 0x00, FRMMODE: HDLC [1] 0x04
  //spi_ax_transport(0U, "< 92 24 >");                           // WR address 0x12: FRAMING - CRCMODE: CRC-16 0x20 (wrong for AX.25), FRMMODE: HDLC [1] 0x04

  /* CRCINIT */
  spi_ax_transport(0U, "< 94 ff ff ff ff >");                        // WR address 0x14: CRCINIT
#endif

  /*WAKEUPTIMER */
  //spi_ax_transport(0U, "< 68 R2 >");                           // RD address 0x68: WAKEUPTIMER
  //uint16_t wutNext = ((uint16_t)s_ax_spi_packet_buffer[0] << 8) | g_ax_spi_packet_buffer[1];
  //wutNext += 1024;

#if 0
  g_ax_spi_packet_buffer[0] = 0x6A | 0x80;                          // WR address 0x6A: WAKEUP - 100 ms later
  g_ax_spi_packet_buffer[1] = wutNext >> 8;
  g_ax_spi_packet_buffer[2] = wutNext & 0xff;
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 3);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif

  /* WAKEUPFREQ */
  //spi_ax_transport(0U, "< ec 04 00 >");                          // WR address 0x6C: WAKEUPFREQ - every 100 ms = 1024d


  /* PINFUNCTCXO_EN */
  #if 0
  spi_ax_transport(0U, "< a6 05 >");                           // WR address 0x26: PINFUNCTCXO_EN - Use TCXO_EN pin as DAC output
  #else
  //spi_ax_transport(0U, "< a6 00 >");                           // WR address 0x26: PINFUNCTCXO_EN - Set to output '0'
  #endif

#if 0
  /* WAKEUPXOEARLY */
  spi_ax_transport(0U, "< ee 01 >");                           // WR address 0x6E: WAKEUPXOEARLY - 1 LPOSC cycle wake-up time before receiver is started


  /* IFFREQ */
  spi_ax_transport(0U, "< f1 00 00 cd >");                         // WR address 0x100: IFFREQ - IFFREQ: 3,128 Hz (f_xtal = 16 MHz)

  /* DECIMATION */
  spi_ax_transport(0U, "< f1 02 25 >");                          // WR address 0x102: DECIMATION - DECIMATION: 37d, f_BASEBAND = 27,027.03 Hz

  /* RXDATARATE */
  spi_ax_transport(0U, "< f1 03 00 b4 2e >");                        // WR address 0x103: RXDATARATE - 1,200 bit/s

  /* MAXDROFFSET */
  spi_ax_transport(0U, "< f1 06 00 00 00 >");                        // WR address 0x106: MAXDROFFSET - off

  /* MAXRFOFFSET */
  spi_ax_transport(0U, "< f1 09 80 04 ea >");                        // WR address 0x109: MAXRFOFFSET - FREQOFFSCORR: Correct frequency offset at the first LO 0x80, MAXRFOFFSET: +/- 1,200 Hz

  /* FSKDMAX */
  spi_ax_transport(0U, "< f1 0c 03 f3 >");                         // WR address 0x10C: FSKDMAX - FSKDMAX: +1,011d (should be +640d?, seems to be moved by abt. 1.5x up)

  /* FSKDMIN */
  spi_ax_transport(0U, "< f1 0e ff 0d >");                         // WR address 0x10E: FSKDMIN - FSKDMIN: -243d (should be -640d?, seems to be moved by abt. 1.5x up)

  /* AMPLFILTER */
  spi_ax_transport(0U, "< f1 15 00 >");                          // WR address 0x115: AMPLFILTER - AMPLFILTER: filter bypassed.

  #if 0
  /* FREQUENCYLEAK */
  spi_ax_transport(0U, "< f1 16 00 >");                          // WR address 0x116: FREQUENCYLEAK
  #endif

  /* RXPARAMSETS */
  spi_ax_transport(0U, "< f1 17 f4 >");                          // WR address 0x117: RXPARAMSETS - RXPS0: 0, RXPS1: 1, RXPS2: 3, RXPS3: 3

  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 e8 >");                          // WR address 0x120: AGCGAIN0 - AGCATTACK0: 8, AGCDECAY0: 14

  /* AGCTARGET0 */
  spi_ax_transport(0U, "< f1 21 84 >");                          // WR address 0x121: AGCTARGET0 - average AGC magnitude = 304

  /* TIMEGAIN0 */
  spi_ax_transport(0U, "< f1 24 ba >");                          // WR address 0x124: TIMEGAIN0

  /* DRGAIN0  0xB4 */
  spi_ax_transport(0U, "< f1 25 b4 >");                          // WR address 0x125: DRGAIN0

  /* PHASEGAIN0 */
  spi_ax_transport(0U, "< f1 26 c3 >");                          // WR address 0x126: PHASEGAIN0

  /* FREQGAINA0 */
  spi_ax_transport(0U, "< f1 27 0f >");                          // WR address 0x127: FREQGAINA0

  /* FREQGAINB0 */
  spi_ax_transport(0U, "< f1 28 1f >");                          // WR address 0x128: FREQGAINB0

  /* FREQGAINC0 */
  spi_ax_transport(0U, "< f1 29 0a >");                          // WR address 0x129: FREQGAINC0

  /* FREQGAIND0 */
  spi_ax_transport(0U, "< f1 2a 0a >");                          // WR address 0x12A: FREQGAIND0

  /* AMPLGAIN0 */
  spi_ax_transport(0U, "< f1 2b 06 >");                          // WR address 0x12B: AMPLGAIN0

  /* FREQDEV0 */
  spi_ax_transport(0U, "< f1 2c 00 00 >");                         // WR address 0x12C: FREQDEV0

  /* BBOFFSRES0 */
  spi_ax_transport(0U, "< f1 2f 00 >");                          // WR address 0x12F: BBOFFSRES0

  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 e8 >");                          // WR address 0x130: AGCGAIN1

  /* AGCTARGET1 */
  spi_ax_transport(0U, "< f1 31 84 >");                          // WR address 0x131: AGCTARGET1

  /* AGCAHYST1 */
  spi_ax_transport(0U, "< f1 32 00 >");                          // WR address 0x132: AGCAHYST1

  /* AGCMINMAX1 */
  spi_ax_transport(0U, "< f1 33 00 >");                          // WR address 0x133: AGCMINMAX1

  /* TIMEGAIN1 */
  spi_ax_transport(0U, "< f1 34 b8 >");                          // WR address 0x134: TIMEGAIN1

  /* DRGAIN1 */
  spi_ax_transport(0U, "< f1 35 b3 >");                          // WR address 0x135: DRGAIN1

  /* PHASEGAIN1 */
  spi_ax_transport(0U, "< f1 36 c3 >");                          // WR address 0x136: PHASEGAIN1

  /* FREQGAINA1 */
  spi_ax_transport(0U, "< f1 37 0f >");                          // WR address 0x137: FREQGAINA1

  /* FREQGAINB1 */
  spi_ax_transport(0U, "< f1 38 1f >");                          // WR address 0x138: FREQGAINB1

  /* FREQGAINC1 */
  spi_ax_transport(0U, "< f1 39 0a >");                          // WR address 0x139: FREQGAINC1

  /* FREQGAIND1 */
  spi_ax_transport(0U, "< f1 3a 0a >");                          // WR address 0x13A: FREQGAIND1

  /* AMPLGAIN1 */
  spi_ax_transport(0U, "< f1 3b 06 >");                          // WR address 0x13B: AMPLGAIN1

  /* FREQDEV1 */
  spi_ax_transport(0U, "< f1 3c 00 4b >");                         // WR address 0x13C: FREQDEV1

  /* FOURFSK1 */
  spi_ax_transport(0U, "< f1 3e 16 >");                          // WR address 0x13E: FOURFSK1

  /* BBOFFSRES1 */
  spi_ax_transport(0U, "< f1 3f 00 >");                          // WR address 0x13F: BBOFFSRES1


  /* AGCGAIN3 */
  spi_ax_transport(0U, "< f1 50 ff >");                          // WR address 0x150: AGCGAIN3

  /* AGCTARGET3 */
  spi_ax_transport(0U, "< f1 51 84 >");                          // WR address 0x151: AGCTARGET3

  /* AGCAHYST3 */
  spi_ax_transport(0U, "< f1 52 00 >");                          // WR address 0x152: AGCAHYST3

  /* AGCMINMAX3 */
  spi_ax_transport(0U, "< f1 53 00 >");                          // WR address 0x153: AGCMINMAX3

  /* TIMEGAIN3 */
  spi_ax_transport(0U, "< f1 54 b8 >");                          // WR address 0x154: TIMEGAIN3

  /* DRGAIN3 */
  spi_ax_transport(0U, "< f1 55 b3 >");                          // WR address 0x155: DRGAIN3

  /* PHASEGAIN3 */
  spi_ax_transport(0U, "< f1 56 c3 >");                          // WR address 0x156: PHASEGAIN3

  /* FREQGAINA3 */
  spi_ax_transport(0U, "< f1 57 0f >");                          // WR address 0x157: FREQGAINA3

  /* FREQGAINB3 */
  spi_ax_transport(0U, "< f1 58 1f >");                          // WR address 0x158: FREQGAINB3

  /* FREQGAINC3 */
  spi_ax_transport(0U, "< f1 59 0d >");                          // WR address 0x159: FREQGAINC3

  /* FREQGAIND3 */
  spi_ax_transport(0U, "< f1 5a 0d >");                          // WR address 0x15A: FREQGAIND3

  /* AMPLGAIN3 */
  spi_ax_transport(0U, "< f1 5b 06 >");                          // WR address 0x15B: AMPLGAIN3

  /* FREQDEV3 */
  spi_ax_transport(0U, "< f1 5c 00 4b >");                         // WR address 0x15C: FREQDEV3

  /* FOURFSK3 */
  spi_ax_transport(0U, "< f1 5e 16 >");                          // WR address 0x15E: FOURFSK3

  /* BBOFFSRES3 */
  spi_ax_transport(0U, "< f1 5f 00 >");                          // WR address 0x15F: BBOFFSRES3 - not used

  /* MODCFGF */
  spi_ax_transport(0U, "< f1 60 00 >");                          // WR address 0x160: MODCFGF - FREQSHAPE: External Loop Filter (BT = 0.0)

  /* FSKDEV */
  spi_ax_transport(0U, "< f1 61 00 02 0c >");                        // WR address 0x161: FSKDEV - FSKDEV: 500 Hz @ fxtal = 16 MHz.


  /* MODCFGA */
  spi_ax_transport(0U, "< f1 64 05 >");                          // WR address 0x164: MODCFGA - AMPLSHAPE, TXDIFF

  /* TXRATE */
  spi_ax_transport(0U, "< f1 65 00 04 ea >");                        // WR address 0x165: TXRATE - TXRATE: 1,200 bit/s

  /* TXPWRCOEFFA */
  spi_ax_transport(0U, "< f1 68 00 00 >");                         // WR address 0x168: TXPWRCOEFFA - no correction

  /* TXPWRCOEFFB */
  spi_ax_transport(0U, "< f1 6a 00 00 >");                         // WR address 0x16A: TXPWRCOEFFB - off

  /* TXPWRCOEFFC */
  spi_ax_transport(0U, "< f1 6c 00 00 >");                         // WR address 0x16C: TXPWRCOEFFC - no correction

  /* TXPWRCOEFFD */
  spi_ax_transport(0U, "< f1 6e 00 00 >");                         // WR address 0x16E: TXPWRCOEFFD - no correction

  /* TXPWRCOEFFE */
  spi_ax_transport(0U, "< f1 70 00 00 >");                         // WR address 0x170: TXPWRCOEFFE - no correction


  /* PLLVCOI */
  spi_ax_transport(0U, "< f1 80 99 >");                          // WR address 0x180: PLLVCOI - 10 킕 * 153 = 1,53 mA

  /* PLLRNGCLK */
  spi_ax_transport(0U, "< f1 83 03 >");                          // WR address 0x183: PLLRNGCLK - PLLRNGCLK: 7,812 Hz ? (@see Table 148 in AND9347/D)

  /* BBTUNE */
  spi_ax_transport(0U, "< f1 88 0f >");                          // WR address 0x188: BBTUNE - BBTUNE: 15 (?)

  /* BBOFFSCAP */
  spi_ax_transport(0U, "< f1 89 77 >");                          // WR address 0x189: BBOFFSCAP - CAPINTB: 7, CAPINTA: 7


  /* PKTADDRCFG */
  spi_ax_transport(0U, "< f2 00 20 >");                          // WR address 0x200: PKTADDRCFG - !MSB_FIRST, !CRC_SKIP_FIRST, FEC SYNC DIS = 0x20, ADDR POS = 0x00

  /* PKTLENCFG */
  spi_ax_transport(0U, "< f2 01 00 >");                          // WR address 0x201: PKTLENCFG - none

  /* PKTLENOFFSET */
  spi_ax_transport(0U, "< f2 02 08 >");                          // WR address 0x202: PKTLENOFFSET - receiver_length = Length-Field +8 bytes, in case Length-Field is sent. (Remarks by DF4IAH: reason unknown)

  /* PKTMAXLEN */
  spi_ax_transport(0U, "< f2 03 ff >");                          // WR address 0x203: PKTMAXLEN - PKTMAXLEN = 255


  /* MATCH0PAT */
  spi_ax_transport(0U, "< f2 10 aa cc aa cc >");                     // WR address 0x210: MATCH0PAT - not in use
  //spi_ax_transport(0U, "< f2 10 7e 00 00 00 >");                     // WR address 0x210: MATCH0PAT - flag = 0x7E

  /* MATCH0LEN */
  spi_ax_transport(0U, "< f2 14 00 >");                          // WR address 0x214: MATCH0LEN - not in use
  //spi_ax_transport(0U, "< f2 14 87 >");                          // WR address 0x214: MATCH0LEN - MATCH0LEN = 8, MATCH0RAW: Select whether Match Unit 0 operates on decoded (after Manchester, Descrambler etc.) (if 0), or on raw received bits (if 1)

  /* MATCH0MIN */
  spi_ax_transport(0U, "< f2 15 00 >");                          // WR address 0x215: MATCH0MIN - not in use

  /* MATCH0MAX */
  spi_ax_transport(0U, "< f2 16 1f >");                          // WR address 0x216: MATCH0MAX - not in use
  //spi_ax_transport(0U, "< f2 16 07 >");                          // WR address 0x216: MATCH0MAX - MATCH0MAX = 7

  /* MATCH1PAT */
  spi_ax_transport(0U, "< f2 18 7e 7e >");                         // WR address 0x218: MATCH1PAT - MATCH1PAT = 0x7E 0x7E

  /* MATCH1LEN */
  spi_ax_transport(0U, "< f2 1c 8a >");                          // WR address 0x21C: MATCH1LEN - MATCH1LEN = 11, MATCH1RAW

  /* MATCH1MAX */
  spi_ax_transport(0U, "< f2 1e 0a >");                          // WR address 0x21E: MATCH1MAX - MATCH1MAX = 10


  /* TMGTXBOOST */
  spi_ax_transport(0U, "< f2 20 32 >");                          // WR address 0x220: TMGTXBOOST - TMGTXBOOSTE = 1, TMGTXBOOSTM = 18 --> 36 탎

  /* TMGTXSETTLE */
  spi_ax_transport(0U, "< f2 21 14 >");                          // WR address 0x221: TMGTXSETTLE - TMGTXSETTLEE = 0, TMGTXSETTLEM = 20 --> 20 탎

  /* TMGRXBOOST */
  spi_ax_transport(0U, "< f2 23 32 >");                          // WR address 0x223: TMGRXBOOST - TMGRXBOOSTE = 1, TMGRXBOOSTM = 18 --> 36 탎

  /* TMGRXSETTLE */
  spi_ax_transport(0U, "< f2 24 14 >");                          // WR address 0x224: TMGRXSETTLE - TMGRXSETTLEE = 0, TMGRXSETTLEM = 20 --> 20 탎

  /* TMGRXOFFSACQ */
  spi_ax_transport(0U, "< f2 25 00 >");                          // WR address 0x225: TMGRXOFFSACQ - TMGRXOFFSACQE = 0, TMGRXOFFSACQM = 0 --> 0 탎

  /* TMGRXCOARSEAGC */
  spi_ax_transport(0U, "< f2 26 73 >");                          // WR address 0x226: TMGRXCOARSEAGC - TMGRXCOARSEAGCE = 3, TMGRXCOARSEAGCM = 19 --> 152 탎 (Bits)  @see PKTMISCFLAGS.RXAGC

  /* TMGRXRSSI */
  //spi_ax_transport(0U, "< f2 28 02 >");                          // WR address 0x228: TMGRXRSSI - TMGRXRSSIE = 0, TMGRXRSSIM = 2 --> 2 탎 (Bits)  @see PKTMISCFLAGS.RXRSSI (for datarates < 2 kbit possible)
  spi_ax_transport(0U, "< f2 28 03 >");                          // WR address 0x228: TMGRXRSSI - TMGRXRSSIE = 0, TMGRXRSSIM = 3 --> 3 탎 (Bits)  @see PKTMISCFLAGS.RXRSSI (fall-back strategy)

  /* TMGRXPREAMBLE1 */
  //spi_ax_transport(0U, "< f2 29 00 >");                          // WR address 0x229: TMGRXPREAMBLE1 - TMGRXPREAMBLE1 timeout = none

  /* TMGRXPREAMBLE2 */
  spi_ax_transport(0U, "< f2 2a 17 >");                          // WR address 0x22A: TMGRXPREAMBLE2 - TMGRXPREAMBLE2 timeout = 7 * 2^1 = 14 bits

  /* TMGRXPREAMBLE3 */
  //spi_ax_transport(0U, "< f2 2b 00 >");                          // WR address 0x22B: TMGRXPREAMBLE3 - TMGRXPREAMBLE3 timeout = none

  /* RSSIREFERENCE */
  spi_ax_transport(0U, "< f2 2c f8 >");                          // WR address 0x22C: RSSIREFERENCE - RSSI Offset, this register adds a constant offset to the computed RSSI value. It is used to compensate for board effects.

  /* RSSIABSTHR */
  spi_ax_transport(0U, "< f2 2d dd >");                          // WR address 0x22D: RSSIABSTHR - RSSIABSTHR >  -36 (-64 offset binary) = -100 dBm (BUSY)
  //spi_ax_transport(0U, "< f2 2d 85 >");  // testing function of BGNDRSSITHR

  /* BGNDRSSITHR */
  //spi_ax_transport(0U, "< f2 2f 00 >");                          // WR address 0x22F: BGNDRSSITHR - off
  //spi_ax_transport(0U, "< f2 2f 01 >");                          // WR address 0x22F: BGNDRSSITHR - BGNDRSSITHR: 1 dB above the BGNDRSSI
  spi_ax_transport(0U, "< f2 2f 30 >");  // testing


  #if 0
  /* PKTMISCFLAGS - handeld by  PR1200_Rx_WoR() and PR1200_Rx_cont() */
  spi_ax_transport(0U, "< f2 31 00 >");                          // WR address 0x231: PKTMISCFLAGS - no BGND RSSI !0x04, RXAGC CLK, RXRSSI CLK clock sources: 1 탎
  #endif

  /* LPOSCCONFIG */
  spi_ax_transport(0U, "< f3 10 13 >");                          // WR address 0x310: LPOSCCONFIG - LPOSC CALIBF 0x10, LPOSC FAST 0x02, (LPOSC ENA 0x01)

  /* LPOSCKFILT */
  spi_ax_transport(0U, "< f3 12 01 5d >");                         // WR address 0x312: LPOSCKFILT - LPOSCKFILT <= 1,398 (lower value gives lower jitter; 1/4 th taken: 349)

  /* LPOSCREF */
  spi_ax_transport(0U, "< f3 14 61 a8 >");                         // WR address 0x314: LPOSCREF - LPOSCREF = 25,000

  /* LPOSCFREQ */
  spi_ax_transport(0U, "< f3 16 00 00 >");                         // WR address 0x316: LPOSCFREQ - no manual tune, done by automatic


  /* PKTCHUNKSIZE */
  spi_ax_transport(0U, "< f2 30 0c >");                          // WR address 0x230: PKTCHUNKSIZE - PKTCHUNKSIZE: 224 bytes

  /* PKTSTOREFLAGS */
  spi_ax_transport(0U, "< f2 32 17 >");                          // WR address 0x232: PKTSTOREFLAGS - ST RSSI, ST RFOFFS, ST FOFFS, ST TIMER

  /* PKTACCEPTFLAGS */
  spi_ax_transport(0U, "< f2 33 28 >");                          // WR address 0x233: PKTACCEPTFLAGS - ACCPT LRGP 0x20, not ACCPT SZF !0x10, ACCPT ADDRF 0x08, not ACCPT CRCF !0x04, not ACCPT ABRT !0x02, not ACCPT RESIDUE !0x01
  //spi_ax_transport(0U, "< f2 33 2c >");  // Test


  #if 1
  /* DACVALUE */
  spi_ax_transport(0U, "< f3 30 00 0c >");                         // WR address 0x330: DACVALUE - DACSHIFT = 12 bit

  /* DACCONFIG */
  spi_ax_transport(0U, "< f3 32 83 >");                          // WR address 0x332: DACCONFIG - DACPWM, DACINPUT=TRKFREQUENCY
  #endif


  /* 0xF10 - XTALOSC*/
  spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10: XTALOSC

  /* 0xF11 - XTALAMPL */
  spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11: XTALAMPL
#endif
}

void spi_ax_initRegisters_PR1200_Tx(void)
{
#if 0
  /* PLLLOOP */
  spi_ax_transport(0U, "< b0 0a >");                           // WR address 0x30: PLLLOOP - DIRECT 0x08, no FILTEN, FLT 02: Internal Loop Filter x2 (BW = 200 kHz for ICP = 272 mA)

  /* PLLCPI */
  spi_ax_transport(0U, "< b1 10 >");                           // WR address 0x31: PLLCPI

  /* PLLVCODIV - mind you: check 0xF34, also */
  spi_ax_transport(0U, "< b2 34 >");                           // WR address 0x32: PLLVCODIV - VCO2INT, VCOSEL, RFDIV

  /* PLLLOOPBOOST */
  spi_ax_transport(0U, "< b8 0b >");                           // WR address 0x38: PLLLOOPBOOST - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5 (BW = 500 kHz for ICP = 1.7 mA)


  /* AFSKSPACE - TX */
  spi_ax_transport(0U, "< f1 10 00 24 >");                         // WR address 0x110: AFSKSPACE - AFSKSPACE: 2,200 Hz = 36d

  /* AFSKMARK - TX */
  spi_ax_transport(0U, "< f1 12 00 14 >");                         // WR address 0x112: AFSKMARK - AFSKMARK: 1,200 Hz = 20d

  #if 0
  /* AFSKCTRL - RX (& TX?) */
  spi_ax_transport(0U, "< f1 14 06 >");                          // WR address 0x114: AFSKCTRL - AFSKSHIFT: 6
  #endif


  /* MODCFGF */
  spi_ax_transport(0U, "< f1 60 00 >");                          // WR address 0x160: MODCFGF - FREQSHAPE: External Loop Filter (BT = 0.0)

  /* FSKDEV - AFSK */
  spi_ax_transport(0U, "< f1 61 00 08 cc >");                        // WR address 0x161: FSKDEV - FSKDEV: +/-2,500 Hz @ fxtal = 16 MHz = 2,252d
  //spi_ax_transport(0U, "< f1 61 00 06 af >");                      // WR address 0x161: FSKDEV - FSKDEV: +/-1,900 Hz @ fxtal = 16 MHz = 1,711d
  //spi_ax_transport(0U, "< f1 61 00 06 28 >");                      // WR address 0x161: FSKDEV - FSKDEV: +/-1,750 Hz @ fxtal = 16 MHz = 1,576d
  //spi_ax_transport(0U, "< f1 61 00 05 47 >");                      // WR address 0x161: FSKDEV - FSKDEV: +/-1,500 Hz @ fxtal = 16 MHz = 1,351d
  //spi_ax_transport(0U, "< f1 61 00 04 66 >");                      // WR address 0x161: FSKDEV - FSKDEV: +/-1,250 Hz @ fxtal = 16 MHz = 1,126d
  //spi_ax_transport(0U, "< f1 61 00 03 85 >");                      // WR address 0x161: FSKDEV - FSKDEV: +/-1,000 Hz @ fxtal = 16 MHz =   901d

  /* MODCFGA */
  spi_ax_transport(0U, "< f1 64 05 >");                          // WR address 0x164: MODCFGA - AMPLSHAPE, TXDIFF

  /* TXRATE */
  spi_ax_transport(0U, "< f1 65 00 04 ea >");                        // WR address 0x165: TXRATE - TXRATE: 1,200 bit/s


  /* XTALCAP */
  spi_ax_transport(0U, "< f1 84 0c >");                          // WR address 0x184: XTALCAP - DF4IAH: adjusted from 0x08 to 0x0c


  /* 0xF00 */
  spi_ax_transport(0U, "< ff 00 0f >");                          // WR address 0xF00 (RX/TX) - Set to 0x0F

  /* 0xF0C */
  spi_ax_transport(0U, "< ff 0c 00 >");                          // WR address 0xF0C (RX/TX) - Keep the default 0x00

  /* 0xF0D = REF */
  spi_ax_transport(0U, "< ff 0d 03 >");                          // WR address 0xF0D: REF (RX/TX) - Set to 0x03

  /* 0xF10 - XTALOSC*/
  spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10 (RX/TX) - Set to 0x04 if a TCXO is used. If a crystal is used, set to 0x0D if the reference frequency (crystal or TCXO) is more than 43 MHz, or to 0x03 otherwise

  /* 0xF11 - XTALAMPL */
  spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11 (RX/TX) - Set to 0x07 if a crystal is connected to CLK16P/CLK16N, or 0x00 if a TCXO is used

  /* 0xF18 */
  spi_ax_transport(0U, "< ff 18 06 >");                          // WR address 0xF18 (RX/TX)

  /* 0xF1C */
  spi_ax_transport(0U, "< ff 1c 07 >");                          // WR address 0xF1C (RX/TX) - Set to 0x07

  /* 0xF34 */
  spi_ax_transport(0U, "< ff 34 28 >");                          // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise

  /* 0xF35 */
  spi_ax_transport(0U, "< ff 35 10 >");                          // WR address 0xF35 (RX/TX) - Set to 0x10 for reference frequencies (crystal or TCXO) less than 24.8 MHz (fXTALDIV = 1), or to 0x11 otherwise (fXTALDIV = 2)

  /* 0xF44 */
  spi_ax_transport(0U, "< ff 44 25 >");                          // WR address 0xF44 (RX/TX)
#endif
}

void spi_ax_initRegisters_PR1200_Rx(void)
{
#if 0
  /* PLLLOOP */
  spi_ax_transport(0U, "< b0 0a >");                           // WR address 0x30: PLLLOOP - DIRECT 0x08, no FILTEN, FLT 02: Internal Loop Filter x2 (BW = 200 kHz for ICP = 272 mA)

  /* PLLCPI */
  spi_ax_transport(0U, "< b1 10 >");                           // WR address 0x31: PLLCPI

  /* PLLVCODIV - mind you: check 0xF34, also */
  spi_ax_transport(0U, "< b2 34 >");                           // WR address 0x32: PLLVCODIV - VCO2INT, VCOSEL, RFDIV

  /* PLLLOOPBOOST */
  spi_ax_transport(0U, "< b8 0b >");                           // WR address 0x38: PLLLOOPBOOST - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5 (BW = 500 kHz for ICP = 1.7 mA)


  /* AFSKSPACE - RX */
  spi_ax_transport(0U, "< f1 10 01 4d >");                         // WR address 0x110: AFSKSPACE - AFSKSPACE: 2,200 Hz

  /* AFSKMARK - RX */
  spi_ax_transport(0U, "< f1 12 00 b6 >");                         // WR address 0x112: AFSKMARK - AFSKMARK: 1,200 Hz

  /* AFSKCTRL */
  spi_ax_transport(0U, "< f1 14 06 >");                          // WR address 0x114: AFSKCTRL - AFSKSHIFT: 6

  /* XTALCAP */
  spi_ax_transport(0U, "< f1 84 0c >");                          // WR address 0x184: XTALCAP - DF4IAH: adjusted from 0x08 to 0x0c

  /* 0xF00 */
  spi_ax_transport(0U, "< ff 00 0f >");                          // WR address 0xF00 (RX/TX) - Set to 0x0F

  /* 0xF0C */
  spi_ax_transport(0U, "< ff 0c 00 >");                          // WR address 0xF0C (RX/TX) - Keep the default 0x00

  /* 0xF0D = REF */
  spi_ax_transport(0U, "< ff 0d 03 >");                          // WR address 0xF0D: REF (RX/TX) - Set to 0x03

  /* 0xF10 - XTALOSC*/
  spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10 (RX/TX) - Set to 0x04 if a TCXO is used. If a crystal is used, set to 0x0D if the reference frequency (crystal or TCXO) is more than 43 MHz, or to 0x03 otherwise

  /* 0xF11 - XTALAMPL */
  spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11 (RX/TX) - Set to 0x07 if a crystal is connected to CLK16P/CLK16N, or 0x00 if a TCXO is used

  /* 0xF18 */
  spi_ax_transport(0U, "< ff 18 06 >");                          // WR address 0xF18 (RX/TX)

  /* 0xF1C */
  spi_ax_transport(0U, "< ff 1c 07 >");                          // WR address 0xF1C (RX/TX) - Set to 0x07

  /* 0xF21 */
  spi_ax_transport(0U, "< ff 21 68 >");                          // WR address 0xF21 (RX)

  /* 0xF22 */
  spi_ax_transport(0U, "< ff 22 ff >");                          // WR address 0xF22 (RX)

  /* 0xF23 */
  spi_ax_transport(0U, "< ff 23 84 >");                          // WR address 0xF23 (RX)

  /* 0xF26 */
  spi_ax_transport(0U, "< ff 26 98 >");                          // WR address 0xF26 (RX)

  /* 0xF34 */
  spi_ax_transport(0U, "< ff 34 28 >");                          // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise

  /* 0xF35 */
  spi_ax_transport(0U, "< ff 35 10 >");                          // WR address 0xF35 (RX/TX) - Set to 0x10 for reference frequencies (crystal or TCXO) less than 24.8 MHz (fXTALDIV = 1), or to 0x11 otherwise (fXTALDIV = 2)

  /* 0xF44 */
  spi_ax_transport(0U, "< ff 44 25 >");                          // WR address 0xF44 (RX/TX)

  /* 0xF72 */
  spi_ax_transport(0U, "< ff 72 00 >");                          // WR address 0xF72 (RX) - Set to 0x06 if the framing mode is set to "Raw, Soft Bits" (register FRAMING), or to 0x00 otherwise
#endif
}

void spi_ax_initRegisters_PR1200_Rx_WoR(void)
{
#if 0
  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 83 >");                          // WR address 0x120: AGCGAIN0 - AGCDECAY0 = 8  (f_3db=621 Hz), AGCATTACK0 = 3  (f_3db=18,660 Hz)

  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 83 >");                          // WR address 0x130: AGCGAIN1 - AGCDECAY1 = 8  (f_3db=621 Hz), AGCATTACK1 = 3  (f_3db=18,660 Hz)


  /* TMGRXAGC */
  spi_ax_transport(0U, "< f2 27 54 >");                          // WR address 0x227: TMGRXAGC - TMGRXAGCE = 5, TMGRXAGCM = 4 --> 128 탎 (Bits)  @see PKTMISCFLAGS.RXRSSI

  /* TMGRXPREAMBLE1 */
  spi_ax_transport(0U, "< f2 29 34 >");                          // WR address 0x229: TMGRXPREAMBLE1 - TMGRXPREAMBLEE = 3, TMGRXPREAMBLEM = 4 --> 32 Bits

  /* PKTMISCFLAGS */
  spi_ax_transport(0U, "< f2 31 05 >");                          // WR address 0x231: PKTMISCFLAGS - BGND RSSI 0x04, !RXAGC CLK, RXRSSI CLK clock sources: Bit clock
#endif
}

void spi_ax_initRegisters_PR1200_Rx_cont(void)
{
#if 0
  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 e8 >");                          // WR address 0x120: AGCGAIN0 - AGCDECAY0 = 14  (f_3db=10 Hz), AGCATTACK0 = 8  (f_3db=621 Hz)

  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 e8 >");                          // WR address 0x130: AGCGAIN1 - AGCDECAY1 = 14  (f_3db=10 Hz), AGCATTACK1 = 8  (f_3db=621 Hz)


  /* TMGRXAGC */
  spi_ax_transport(0U, "< f2 27 00 >");                          // WR address 0x227: TMGRXAGC - not used

  /* TMGRXPREAMBLE1 */
  spi_ax_transport(0U, "< f2 29 00 >");                          // WR address 0x229: TMGRXPREAMBLE1 - not used

  /* PKTMISCFLAGS */
  //spi_ax_transport(0U, "< f2 31 00 >");                          // WR address 0x231: PKTMISCFLAGS - no BGND RSSI !0x04, RXAGC CLK, RXRSSI CLK clock sources: 1 탎
  spi_ax_transport(0U, "< f2 31 04 >");                          // WR address 0x231: PKTMISCFLAGS - BGND RSSI 0x04, RXAGC CLK, RXRSSI CLK clock sources: 1 탎
#endif
}

void spi_ax_initRegisters_PR1200_Rx_cont_SingleParamSet(void)
{
#if 0
  /* RXPARAMSETS */
  spi_ax_transport(0U, "< f1 17 ff >");                          // WR address 0x117: RXPARAMSETS

  /* FREQDEV3 */
  spi_ax_transport(0U, "< f1 5c 00 >");                          // WR address 0x15C: FREQDEV3

  /* AGCGAIN3 */
  spi_ax_transport(0U, "< f1 50 e8 >");                          // WR address 0x150: AGCGAIN3
#endif
}

void spi_ax_init_PR1200_Tx(void)
{
  /* Syncing and sending reset command, then setting the packet radio values for transmission */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* APRS  */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.8000);            // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05

    /* Burst-Aussendungen fuer Steuerungszwecke */
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9250);            // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05

    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the transmitter settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_PR1200, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  #if 1
    /* Set VCO-PLL to FREQA - 144.800 MHz */
    (void) spi_ax_selectVcoFreq(0U);
  #else
    /* Set VCO-PLL to FREQB - 144.925 MHz */
    (void) spi_ax_selectVcoFreq(1U);
  #endif

  /* Set power level */
  spi_ax_setPower_dBm(-10.0f);

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the transmitter */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLTX);
}

void spi_ax_run_PR1200_Tx_FIFO_APRS(const char addrAry[][C_PR1200_CALL_LENGTH], const uint8_t* ssidAry, uint8_t addrCnt, const char* aprsMsg, uint8_t aprsMsgLen)
{
  /* Enter an APRS UI frame */

  /* 1 - Flags */
  spi_ax_util_PR1200_Tx_FIFO_Flags(50);                           // 333 ms

  /* 2 - Address field, Control and PID */
  spi_ax_util_PR1200_Tx_FIFO_AddressField(addrAry, ssidAry, addrCnt);

  /* 3 - Information field - APRS data */
  spi_ax_util_PR1200_Tx_FIFO_InformationField(aprsMsg, aprsMsgLen);
}

void spi_ax_util_PR1200_Tx_FIFO_Flags(uint8_t count)
{
  uint16_t idx = 0;  (void) idx;

  /* Wait until enough space for next batch is available */
  spi_ax_util_FIFO_waitFree(5 + count);

#if 0
  g_ax_spi_packet_buffer[idx++] = 0xA9;                           // WR address 0x29: FIFODATA  (SPI AX address keeps constant)
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_CMD_REPEATDATA_TX;

  /* Setting RAW to one causes the DATA to bypass the framing mode, but still pass through the encoder */
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_FLAGS_TX_RAW | AX_FIFO_DATA_FLAGS_TX_NOCRC | AX_FIFO_DATA_FLAGS_TX_PKTSTART;

  g_ax_spi_packet_buffer[idx++] = count;
  g_ax_spi_packet_buffer[idx++] = 0b01111110;                         // The AX25 'Flag'
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, idx);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  /* FIFO do a COMMIT */
  spi_ax_transport(0U, "< a8 04 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_COMMIT
#endif
}

void spi_ax_util_PR1200_Tx_FIFO_AddressField(const char addrAry[][C_PR1200_CALL_LENGTH], const uint8_t* ssidAry, uint8_t addrCnt)
{
  uint16_t idx = 0;  (void) idx;

  /* Sanity check */
  if (addrCnt > 4) {
    return;
  }

  /* Wait until enough space for next batch is available */
  spi_ax_util_FIFO_waitFree(4 + addrCnt * 7 + 2);

#if 0
  g_ax_spi_packet_buffer[idx++] = 0xA9;                           // WR address 0x29: FIFODATA  (SPI AX address keeps constant)
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_CMD_DATA_TX_RX;
  g_ax_spi_packet_buffer[idx++] = 0;                              // Dummy entry for now
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_FLAGS_TX_PKTSTART;               // FIFO flag byte

  for (uint8_t addrIdx = 0; addrIdx < addrCnt; addrIdx++) {
    const char* addrStr = &addrAry[addrIdx][0];
    uint8_t ssid = 0x0f & ssidAry[addrIdx];
    uint8_t strLen = strnlen(addrStr, C_PR1200_CALL_LENGTH);

    for (uint8_t addrStrIdx = 0; addrStrIdx < C_PR1200_CALL_LENGTH; addrStrIdx++) {
      uint8_t c = addrStrIdx < strLen ?  toupper((char)*(addrStr + addrStrIdx)) : ' ';
      g_ax_spi_packet_buffer[idx++] = (c << 1)  | 0;                  // Address: dest. string
    }

    uint8_t pf = (addrIdx == 1) ?  1 : 0;
    uint8_t addrEnd = (addrIdx == addrCnt - 1) ?  1 : 0;
    uint8_t val = (pf << 7) | (0b11 << 5) | (ssid << 1) | addrEnd;              // Address: dest. SSID
    g_ax_spi_packet_buffer[idx++] = val;
  }

  g_ax_spi_packet_buffer[idx++] = (0b0 << 4) |  0b11;                     // Control: UI frame with no Poll bit set
  g_ax_spi_packet_buffer[idx++] = 0xf0;                           // PID

  /* Set length for FIFO DATA command */
  g_ax_spi_packet_buffer[    2] = idx - 3;                          // Length

  /* FIFO data enter */
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, idx);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  /* FIFO do a COMMIT */
  spi_ax_transport(0U, "< a8 04 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_COMMIT
#endif
}

void spi_ax_util_PR1200_Tx_FIFO_InformationField(const char* aprsMsg, uint8_t aprsMsgLen)
{
  uint16_t idx = 0;  (void) idx;

  /* Sanity checks */
  if (!aprsMsg || (aprsMsgLen > 80)) {
    return;
  }

  /* Wait until enough space for next batch is available */
  spi_ax_util_FIFO_waitFree(4 + aprsMsgLen);

#if 0
  g_ax_spi_packet_buffer[idx++] = 0xA9;                           // WR address 0x29: FIFODATA  (SPI AX address keeps constant)
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_CMD_DATA_TX_RX;
  g_ax_spi_packet_buffer[idx++] = 0;                              // Dummy entry for now
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_FLAGS_TX_PKTEND;               // FIFO flag byte

  for (uint8_t msgStrIdx = 0; msgStrIdx < aprsMsgLen; msgStrIdx++) {
    g_ax_spi_packet_buffer[idx++] = *(aprsMsg + msgStrIdx);                 // Info: APRS data
  }

  /* Set length for FIFO DATA command */
  g_ax_spi_packet_buffer[    2] = idx - 3;                          // Length

  /* FIFO data enter */
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, idx);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  /* FIFO do a COMMIT */
  spi_ax_transport(0U, "< a8 04 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_COMMIT
#endif
}

void spi_ax_init_PR1200_Rx(AX_SET_REGISTERS_POWERMODE_t powerMode)
{
  /* Syncing and sending reset command, then setting the packet radio values for transmission */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* APRS  */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.8000);            // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05

    /* Burst-Aussendungen fuer Steuerungszwecke */
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9250);            // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05

    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the receiver settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_PR1200, AX_SET_REGISTERS_VARIANT_RX_WOR, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  #if 1
    /* Set VCO-PLL to FREQA - 144.800 MHz */
    (void) spi_ax_selectVcoFreq(0U);
  #else
    /* Set VCO-PLL to FREQB - 144.925 MHz */
    (void) spi_ax_selectVcoFreq(1U);
  #endif

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the wake-on-radio receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, powerMode);
}


void spi_ax_initRegisters_POCSAG(void)
{
#if 0
  /* MODULATION */
  spi_ax_transport(0U, "< 90 08 >");                           // WR address 0x10: MODULATION - 08: FSK

  /* ENCODING */
  spi_ax_transport(0U, "< 91 01 >");                           // WR address 0x11: ENCODING -  ENC INV 0x01

  /* FRAMING */
  /* When FRMMODE: "Raw, Soft Bits (0b001 << 1)" is selected, do address 0xF72 also */
  spi_ax_transport(0U, "< 92 06 >");                           // WR address 0x12: FRAMING - CRCMODE: none, FRMMODE: Raw, Pattern Match 0x06

  #if 0
  /* CRCINIT */
  spi_ax_transport(0U, "< 94 ff ff ff ff >");                        // WR address 0x14: CRCINIT
  #endif


  /* PINFUNCSYSCLK */
  spi_ax_transport(0U, "< a1 82 >");                           // WR address 0x21: PINFUNCSYSCLK - Set to 'Z'

  /* PINFUNCDCLK */
  spi_ax_transport(0U, "< a2 82 >");                           // WR address 0x22: PINFUNCDCLK - Set to 'Z'

  /* PINFUNCDATA */
  spi_ax_transport(0U, "< a3 82 >");                           // WR address 0x23: PINFUNCDATA - Set to 'Z'

  /* PINFUNCIRQ */
  spi_ax_transport(0U, "< a4 03 >");                           // WR address 0x24: PINFUNCIRQ - Set to Interrupt request

  /* PINFUNCANTSEL */
  spi_ax_transport(0U, "< a5 05 >");                           // WR address 0x25: PINFUNCANTSEL - Set to DAC output

  /* PINFUNCPWRAMP / PINFUNCTCXO_EN */
  spi_ax_transport(0U, "< a6 02 >");                           // WR address 0x26: PINFUNCTCXO_EN - Set to 'Z'


  // --

  /*WAKEUPTIMER */
  spi_ax_transport(0U, "< 68 R2 >");                           // RD address 0x68: WAKEUPTIMER
  uint16_t wutNext = ((uint16_t)g_ax_spi_packet_buffer[0] << 8) | g_ax_spi_packet_buffer[1];
  wutNext += 1024;

  g_ax_spi_packet_buffer[0] = 0x6A | 0x80;                          // WR address 0x6A: WAKEUP - 100 ms later
  g_ax_spi_packet_buffer[1] = wutNext >> 8;
  g_ax_spi_packet_buffer[2] = wutNext & 0xff;
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 3);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  /* WAKEUPFREQ */
  spi_ax_transport(0U, "< ec 04 00 >");                          // WR address 0x6C: WAKEUPFREQ - every 100 ms = 1024d

  // --

  /* WAKEUPXOEARLY */
  spi_ax_transport(0U, "< ee 01 >");                           // WR address 0x6E: WAKEUPXOEARLY - 1 LPOSC cycle wake-up time before receiver is started


  /* IFFREQ */
  spi_ax_transport(0U, "< f1 00 02 66 >");                         // WR address 0x100: IFFREQ - IFFREQ: 9,369 Hz (f_xtal = 16 MHz)

  /* DECIMATION */
  spi_ax_transport(0U, "< f1 02 19 >");                          // WR address 0x102: DECIMATION - DECIMATION: 37d, f_BASEBAND = 27,027.03 Hz

  /* RXDATARATE */
  spi_ax_transport(0U, "< f1 03 01 0a aa >");                        // WR address 0x103: RXDATARATE - 1,200 bit/s

  /* MAXDROFFSET */
  spi_ax_transport(0U, "< f1 06 00 00 00 >");                        // WR address 0x106: MAXDROFFSET - off

  /* MAXRFOFFSET */
  spi_ax_transport(0U, "< f1 09 80 08 31 >");                        // WR address 0x109: MAXRFOFFSET - FREQOFFSCORR: Correct frequency offset at the first LO 0x80, MAXRFOFFSET: +/- 1,200 Hz

  /* FSKDMAX */
  spi_ax_transport(0U, "< f1 0c 06 55 >");                         // WR address 0x10C: FSKDMAX - FSKDMAX: +1,011d (should be +640d?, seems to be moved by abt. 1.5x up)

  /* FSKDMIN */
  spi_ax_transport(0U, "< f1 0e f9 ab >");                         // WR address 0x10E: FSKDMIN - FSKDMIN: -243d (should be -640d?, seems to be moved by abt. 1.5x up)

  /* AMPLFILTER */
  spi_ax_transport(0U, "< f1 15 00 >");                          // WR address 0x115: AMPLFILTER - AMPLFILTER: filter bypassed.

  #if 0
  /* FREQUENCYLEAK */
  spi_ax_transport(0U, "< f1 16 00 >");                          // WR address 0x116: FREQUENCYLEAK
  #endif

  /* RXPARAMSETS */
  spi_ax_transport(0U, "< f1 17 f4 >");                          // WR address 0x117: RXPARAMSETS - RXPS0: 0, RXPS1: 1, RXPS2: 3, RXPS3: 3

  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 e8 >");                          // WR address 0x120: AGCGAIN0 - AGCATTACK0: 8, AGCDECAY0: 14

  /* AGCTARGET0 */
  spi_ax_transport(0U, "< f1 21 84 >");                          // WR address 0x121: AGCTARGET0 - average AGC magnitude = 304

  /* TIMEGAIN0 */
  spi_ax_transport(0U, "< f1 24 8b >");                          // WR address 0x124: TIMEGAIN0

  /* DRGAIN0  0xB4 */
  spi_ax_transport(0U, "< f1 25 85 >");                          // WR address 0x125: DRGAIN0

  /* PHASEGAIN0 */
  spi_ax_transport(0U, "< f1 26 c3 >");                          // WR address 0x126: PHASEGAIN0

  /* FREQGAINA0 */
  spi_ax_transport(0U, "< f1 27 0f >");                          // WR address 0x127: FREQGAINA0

  /* FREQGAINB0 */
  spi_ax_transport(0U, "< f1 28 1f >");                          // WR address 0x128: FREQGAINB0

  /* FREQGAINC0 */
  spi_ax_transport(0U, "< f1 29 0a >");                          // WR address 0x129: FREQGAINC0

  /* FREQGAIND0 */
  spi_ax_transport(0U, "< f1 2a 0a >");                          // WR address 0x12A: FREQGAIND0

  /* AMPLGAIN0 */
  spi_ax_transport(0U, "< f1 2b 06 >");                          // WR address 0x12B: AMPLGAIN0

  /* FREQDEV0 */
  spi_ax_transport(0U, "< f1 2c 00 00 >");                         // WR address 0x12C: FREQDEV0

  /* BBOFFSRES0 */
  spi_ax_transport(0U, "< f1 2f 00 >");                          // WR address 0x12F: BBOFFSRES0

  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 e8 >");                          // WR address 0x130: AGCGAIN1

  /* AGCTARGET1 */
  spi_ax_transport(0U, "< f1 31 84 >");                          // WR address 0x131: AGCTARGET1

  /* AGCAHYST1 */
  spi_ax_transport(0U, "< f1 32 00 >");                          // WR address 0x132: AGCAHYST1

  /* AGCMINMAX1 */
  spi_ax_transport(0U, "< f1 33 00 >");                          // WR address 0x133: AGCMINMAX1

  /* TIMEGAIN1 */
  spi_ax_transport(0U, "< f1 34 89 >");                          // WR address 0x134: TIMEGAIN1

  /* DRGAIN1 */
  spi_ax_transport(0U, "< f1 35 84 >");                          // WR address 0x135: DRGAIN1

  /* PHASEGAIN1 */
  spi_ax_transport(0U, "< f1 36 c3 >");                          // WR address 0x136: PHASEGAIN1

  /* FREQGAINA1 */
  spi_ax_transport(0U, "< f1 37 0f >");                          // WR address 0x137: FREQGAINA1

  /* FREQGAINB1 */
  spi_ax_transport(0U, "< f1 38 1f >");                          // WR address 0x138: FREQGAINB1

  /* FREQGAINC1 */
  spi_ax_transport(0U, "< f1 39 0a >");                          // WR address 0x139: FREQGAINC1

  /* FREQGAIND1 */
  spi_ax_transport(0U, "< f1 3a 0a >");                          // WR address 0x13A: FREQGAIND1

  /* AMPLGAIN1 */
  spi_ax_transport(0U, "< f1 3b 06 >");                          // WR address 0x13B: AMPLGAIN1

  /* FREQDEV1 */
  spi_ax_transport(0U, "< f1 3c 02 55 >");                         // WR address 0x13C: FREQDEV1

  /* FOURFSK1 */
  spi_ax_transport(0U, "< f1 3e 16 >");                          // WR address 0x13E: FOURFSK1

  /* BBOFFSRES1 */
  spi_ax_transport(0U, "< f1 3f 00 >");                          // WR address 0x13F: BBOFFSRES1


  /* AGCGAIN3 */
  spi_ax_transport(0U, "< f1 50 ff >");                          // WR address 0x150: AGCGAIN3

  /* AGCTARGET3 */
  spi_ax_transport(0U, "< f1 51 84 >");                          // WR address 0x151: AGCTARGET3

  /* AGCAHYST3 */
  spi_ax_transport(0U, "< f1 52 00 >");                          // WR address 0x152: AGCAHYST3

  /* AGCMINMAX3 */
  spi_ax_transport(0U, "< f1 53 00 >");                          // WR address 0x153: AGCMINMAX3

  /* TIMEGAIN3 */
  spi_ax_transport(0U, "< f1 54 88 >");                          // WR address 0x154: TIMEGAIN3

  /* DRGAIN3 */
  spi_ax_transport(0U, "< f1 55 83 >");                          // WR address 0x155: DRGAIN3

  /* PHASEGAIN3 */
  spi_ax_transport(0U, "< f1 56 c3 >");                          // WR address 0x156: PHASEGAIN3

  /* FREQGAINA3 */
  spi_ax_transport(0U, "< f1 57 0f >");                          // WR address 0x157: FREQGAINA3

  /* FREQGAINB3 */
  spi_ax_transport(0U, "< f1 58 1f >");                          // WR address 0x158: FREQGAINB3

  /* FREQGAINC3 */
  spi_ax_transport(0U, "< f1 59 0d >");                          // WR address 0x159: FREQGAINC3

  /* FREQGAIND3 */
  spi_ax_transport(0U, "< f1 5a 0d >");                          // WR address 0x15A: FREQGAIND3

  /* AMPLGAIN3 */
  spi_ax_transport(0U, "< f1 5b 06 >");                          // WR address 0x15B: AMPLGAIN3

  /* FREQDEV3 */
  spi_ax_transport(0U, "< f1 5c 02 55 >");                         // WR address 0x15C: FREQDEV3

  /* FOURFSK3 */
  spi_ax_transport(0U, "< f1 5e 16 >");                          // WR address 0x15E: FOURFSK3

  /* BBOFFSRES3 */
  spi_ax_transport(0U, "< f1 5f 00 >");                          // WR address 0x15F: BBOFFSRES3 - not used

  /* MODCFGF */
  spi_ax_transport(0U, "< f1 60 00 >");                          // WR address 0x160: MODCFGF - FREQSHAPE: External Loop Filter (BT = 0.0)

  /* FSKDEV */
  spi_ax_transport(0U, "< f1 61 00 10 62 >");                        // WR address 0x161: FSKDEV - FSKDEV: +/-2,500 Hz @ fxtal = 16 MHz = 2,252d


  /* MODCFGA */
  spi_ax_transport(0U, "< f1 64 05 >");                          // WR address 0x164: MODCFGA - AMPLSHAPE, TXDIFF

  /* TXRATE */
  spi_ax_transport(0U, "< f1 65 00 04 ea >");                        // WR address 0x165: TXRATE - TXRATE: 1,200 bit/s

  /* TXPWRCOEFFA */
  spi_ax_transport(0U, "< f1 68 00 00 >");                         // WR address 0x168: TXPWRCOEFFA - no correction

  /* TXPWRCOEFFB */
  spi_ax_transport(0U, "< f1 6a 00 aa >");                         // WR address 0x16A: TXPWRCOEFFB - -10 dBm

  /* TXPWRCOEFFC */
  spi_ax_transport(0U, "< f1 6c 00 00 >");                         // WR address 0x16C: TXPWRCOEFFC - no correction

  /* TXPWRCOEFFD */
  spi_ax_transport(0U, "< f1 6e 00 00 >");                         // WR address 0x16E: TXPWRCOEFFD - no correction

  /* TXPWRCOEFFE */
  spi_ax_transport(0U, "< f1 70 00 00 >");                         // WR address 0x170: TXPWRCOEFFE - no correction


  /* PLLVCOI */
  spi_ax_transport(0U, "< f1 80 99 >");                          // WR address 0x180: PLLVCOI - 10 킕 * 153 = 1,53 mA

  /* PLLRNGCLK */
  spi_ax_transport(0U, "< f1 83 03 >");                          // WR address 0x183: PLLRNGCLK - PLLRNGCLK: 7,812 Hz ? (@see Table 148 in AND9347/D)

  /* BBTUNE */
  spi_ax_transport(0U, "< f1 88 0f >");                          // WR address 0x188: BBTUNE - BBTUNE: 15 (?)

  /* BBOFFSCAP */
  spi_ax_transport(0U, "< f1 89 77 >");                          // WR address 0x189: BBOFFSCAP - CAPINTB: 7, CAPINTA: 7


  /* PKTADDRCFG */
  spi_ax_transport(0U, "< f2 00 a0 >");                          // WR address 0x200: PKTADDRCFG - MSB_FIRST, !CRC_SKIP_FIRST, FEC SYNC DIS = 0x20, ADDR POS = 0x00

  /* PKTLENCFG */
  spi_ax_transport(0U, "< f2 01 f0 >");                          // WR address 0x201: PKTLENCFG - arbitrary length packets

  /* PKTLENOFFSET */
  spi_ax_transport(0U, "< f2 02 00 >");                          // WR address 0x202: PKTLENOFFSET - not in use

  /* PKTMAXLEN */
  spi_ax_transport(0U, "< f2 03 ff >");                          // WR address 0x203: PKTMAXLEN - arbitrary length packets


  /* MATCH0PAT (32 bits) */
  spi_ax_transport(0U, "< f2 10 1b a8 4b 3e >");                     // WR address 0x210: MATCH0PAT - POCSAG SYNC word   (POCSAG SYNCWORD = 0x7CD215D8  MSB / LSB reversed)

  /* MATCH0LEN */
  spi_ax_transport(0U, "< f2 14 1f >");                          // WR address 0x214: MATCH0LEN = 32  (all bits of MATCH0PAT are used)

  /* MATCH0MIN */
  spi_ax_transport(0U, "< f2 15 00 >");                          // WR address 0x215: MATCH0MIN - not in use

  /* MATCH0MAX */
  spi_ax_transport(0U, "< f2 16 1f >");                          // WR address 0x216: MATCH0MAX - MATCH0MAX = 32

  /* MATCH1PAT (16 bits) */
  spi_ax_transport(0U, "< f2 18 55 55 >");                         // WR address 0x218: MATCH1PAT - MATCH1PAT = 0x55 0x55  (POCSAG Preamble 0xAAAA is sent MSB, but MATCHxPAT is LSB)

  /* MATCH1LEN */
  spi_ax_transport(0U, "< f2 1c 0f >");                          // WR address 0x21C: MATCH1LEN - not MATCH1RAW, MATCH1LEN = 16  (all bits of MATCH1PAT are used)

  /* MATCH1MIN */
  spi_ax_transport(0U, "< f2 1d 00 >");                          // WR address 0x21D: MATCH1MIN - not in use

  /* MATCH1MAX */
  spi_ax_transport(0U, "< f2 1e 0f >");                          // WR address 0x21E: MATCH1MAX - MATCH1MAX = 16


  /* TMGTXBOOST */
  spi_ax_transport(0U, "< f2 20 32 >");                          // WR address 0x220: TMGTXBOOST - TMGTXBOOSTE = 1, TMGTXBOOSTM = 18 --> 36 탎

  /* TMGTXSETTLE */
  spi_ax_transport(0U, "< f2 21 14 >");                          // WR address 0x221: TMGTXSETTLE - TMGTXSETTLEE = 0, TMGTXSETTLEM = 20 --> 20 탎

  /* TMGRXBOOST */
  spi_ax_transport(0U, "< f2 23 32 >");                          // WR address 0x223: TMGRXBOOST - TMGRXBOOSTE = 1, TMGRXBOOSTM = 18 --> 36 탎

  /* TMGRXSETTLE */
  spi_ax_transport(0U, "< f2 24 14 >");                          // WR address 0x224: TMGRXSETTLE - TMGRXSETTLEE = 0, TMGRXSETTLEM = 20 --> 20 탎

  /* TMGRXOFFSACQ */
  spi_ax_transport(0U, "< f2 25 00 >");                          // WR address 0x225: TMGRXOFFSACQ - TMGRXOFFSACQE = 0, TMGRXOFFSACQM = 0 --> 0 탎

  /* TMGRXCOARSEAGC */
  spi_ax_transport(0U, "< f2 26 73 >");                          // WR address 0x226: TMGRXCOARSEAGC - TMGRXCOARSEAGCE = 3, TMGRXCOARSEAGCM = 19 --> 152 탎 (Bits)  @see PKTMISCFLAGS.RXAGC

  /* TMGRXRSSI */
  spi_ax_transport(0U, "< f2 28 03 >");                          // WR address 0x228: TMGRXRSSI - TMGRXRSSIE = 0, TMGRXRSSIM = 3 --> 3 탎 (Bits)  @see PKTMISCFLAGS.RXRSSI (fall-back strategy)

  /* TMGRXPREAMBLE1 */
  spi_ax_transport(0U, "< f2 29 00 >");                          // WR address 0x229: TMGRXPREAMBLE1 - TMGRXPREAMBLE1 timeout = none

  /* TMGRXPREAMBLE2 */
  spi_ax_transport(0U, "< f2 2a 35 >");                          // WR address 0x22A: TMGRXPREAMBLE2 - TMGRXPREAMBLE2 timeout = 7 * 2^1 = 14 bits

  /* TMGRXPREAMBLE3 */
  spi_ax_transport(0U, "< f2 2b 00 >");                          // WR address 0x22B: TMGRXPREAMBLE3 - TMGRXPREAMBLE3 timeout = none

  /* RSSIREFERENCE */
  spi_ax_transport(0U, "< f2 2c f6 >");                          // WR address 0x22C: RSSIREFERENCE - RSSI Offset, this register adds a constant offset to the computed RSSI value. It is used to compensate for board effects.

  /* RSSIABSTHR */
  //spi_ax_transport(0U, "< f2 2d ed >");                          // WR address 0x22D: RSSIABSTHR - RSSIABSTHR >  - 26 dBm (BUSY)
  //spi_ax_transport(0U, "< f2 2d a4 >");                          // WR address 0x22D: RSSIABSTHR - RSSIABSTHR >  - 93 dBm (BUSY)
  spi_ax_transport(0U, "< f2 2d 85 >");                          // WR address 0x22D: RSSIABSTHR - RSSIABSTHR >  -122 dBm (BUSY)

  /* BGNDRSSITHR */
  spi_ax_transport(0U, "< f2 2f 00 >");                          // WR address 0x22F: BGNDRSSITHR - off


  /* PKTCHUNKSIZE */
  spi_ax_transport(0U, "< f2 30 0c >");                          // WR address 0x230: PKTCHUNKSIZE - PKTCHUNKSIZE: 224 bytes

  /* PKTSTOREFLAGS */
  spi_ax_transport(0U, "< f2 32 01 >");                          // WR address 0x232: PKTSTOREFLAGS - not ST RSSI !0x10, not ST RFOFFS !0x04, not ST FOFFS !0x02, ST TIMER 0x01

  /* PKTACCEPTFLAGS */
  spi_ax_transport(0U, "< f2 33 3c >");                          // WR address 0x233: PKTACCEPTFLAGS - ACCPT LRGP 0x20 (arbitrary length packets), ACCPT SZF 0x10, ACCPT ADDRF 0x08, ACCPT CRCF 0x04, not ACCPT ABRT !0x02, not ACCPT RESIDUE !0x01


  /* LPOSCCONFIG */
  spi_ax_transport(0U, "< f3 10 13 >");                          // WR address 0x310: LPOSCCONFIG - LPOSC CALIBF 0x10, LPOSC FAST 0x02, (LPOSC ENA 0x01)

  /* LPOSCKFILT */
  spi_ax_transport(0U, "< f3 12 01 5d >");                         // WR address 0x312: LPOSCKFILT - LPOSCKFILT <= 1,398 (lower value gives lower jitter; 1/4 th taken: 349)

  /* LPOSCREF */
  spi_ax_transport(0U, "< f3 14 61 a8 >");                         // WR address 0x314: LPOSCREF - LPOSCREF = 25,000

  /* LPOSCFREQ */
  spi_ax_transport(0U, "< f3 16 00 00 >");                         // WR address 0x316: LPOSCFREQ - no manual tune, done by automatic


  /* DACVALUE */
  spi_ax_transport(0U, "< f3 30 00 0c >");                         // WR address 0x330: DACVALUE - DACSHIFT = 12 bit

  /* DACCONFIG */
  spi_ax_transport(0U, "< f3 32 81 >");                          // WR address 0x332: DACCONFIG


  /* 0xF0D = REF */
  spi_ax_transport(0U, "< ff 0d 03 >");                          // WR address 0xF0D: REF (RX/TX) - Set to 0x03

  /* 0xF10 - XTALOSC*/
  spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10 (RX/TX) - Set to 0x04 if a TCXO is used. If a crystal is used, set to 0x0D if the reference frequency (crystal or TCXO) is more than 43 MHz, or to 0x03 otherwise

  /* 0xF11 - XTALAMPL */
  spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11 (RX/TX) - Set to 0x07 if a crystal is connected to CLK16P/CLK16N, or 0x00 if a TCXO is used

  /* 0xF1C */
  spi_ax_transport(0U, "< ff 1c 07 >");                          // WR address 0xF1C (RX/TX) - Set to 0x07
}

void spi_ax_initRegisters_POCSAG_Tx(void)
{
  /* PLLLOOP */
  spi_ax_transport(0U, "< b0 0b >");                           // WR address 0x30: PLLLOOP

  /* PLLCPI */
  spi_ax_transport(0U, "< b1 10 >");                           // WR address 0x31: PLLCPI

  /* PLLVCODIV - mind you: check 0xF34, also */
  spi_ax_transport(0U, "< b2 24 >");                           // WR address 0x32: PLLVCODIV - VCO2INT, RFDIV

  /* PLLLOOPBOOST */
  //spi_ax_transport(0U, "< b8 0b >");                           // WR address 0x38: PLLLOOPBOOST - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5 (BW = 500 kHz for ICP = 1.7 mA)


  /* MODCFGF */
  //spi_ax_transport(0U, "< f1 60 00 >");                          // WR address 0x160: MODCFGF - FREQSHAPE: External Loop Filter (BT = 0.0)

  /* FSKDEV */
  //spi_ax_transport(0U, "< f1 61 00 10 62 >");                      // WR address 0x161: FSKDEV - FSKDEV: +/-4,000 Hz @ fxtal = 16 MHz

  /* MODCFGA */
  //spi_ax_transport(0U, "< f1 64 05 >");                          // WR address 0x164: MODCFGA - AMPLSHAPE, TXDIFF

  /* TXRATE */
  //spi_ax_transport(0U, "< f1 65 00 04 ea >");                      // WR address 0x165: TXRATE - TXRATE: 1,200 bit/s


  /* XTALCAP */
  spi_ax_transport(0U, "< f1 84 0c >");                          // WR address 0x184: XTALCAP


  /* 0xF00 */
  spi_ax_transport(0U, "< ff 00 0f >");                          // WR address 0xF00:

  /* 0xF0C */
  //spi_ax_transport(0U, "< ff 0c 00 >");                          // WR address 0xF0C (RX/TX) - Keep the default 0x00

  /* 0xF0D = REF */
  //spi_ax_transport(0U, "< ff 0d 03 >");                          // WR address 0xF0D: REF (RX/TX) - Set to 0x03

  /* 0xF10 - XTALOSC*/
  //spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10 (RX/TX) - Set to 0x04 if a TCXO is used. If a crystal is used, set to 0x0D if the reference frequency (crystal or TCXO) is more than 43 MHz, or to 0x03 otherwise

  /* 0xF11 - XTALAMPL */
  //spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11 (RX/TX) - Set to 0x07 if a crystal is connected to CLK16P/CLK16N, or 0x00 if a TCXO is used

  /* 0xF18 */
  spi_ax_transport(0U, "< ff 18 06 >");                          // WR address 0xF18 (RX/TX)

  /* 0xF1C */
  //spi_ax_transport(0U, "< ff 1c 07 >");                          // WR address 0xF1C (RX/TX) - Set to 0x07

  /* 0xF34 */
  //spi_ax_transport(0U, "< ff 34 28 >");                          // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise

  /* 0xF35 */
  //spi_ax_transport(0U, "< ff 35 10 >");                          // WR address 0xF35 (RX/TX) - Set to 0x10 for reference frequencies (crystal or TCXO) less than 24.8 MHz (fXTALDIV = 1), or to 0x11 otherwise (fXTALDIV = 2)

  /* 0xF44 */
  //spi_ax_transport(0U, "< ff 44 25 >");                          // WR address 0xF44 (RX/TX)

  /* 0xF21 */
  //spi_ax_transport(0U, "< ff 21 68 >");                          // WR address 0xF21 (RX)

  /* 0xF22 */
  //spi_ax_transport(0U, "< ff 22 ff >");                          // WR address 0xF22 (RX)

  /* 0xF23 */
  //spi_ax_transport(0U, "< ff 23 84 >");                          // WR address 0xF23 (RX)

  /* 0xF26 */
  //spi_ax_transport(0U, "< ff 26 98 >");                          // WR address 0xF26 (RX)

  /* 0xF34 */
  //spi_ax_transport(0U, "< ff 34 28 >");                          // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise

  /* 0xF35 */
  //spi_ax_transport(0U, "< ff 35 10 >");                          // WR address 0xF35 (RX/TX) - Set to 0x10 for reference frequencies (crystal or TCXO) less than 24.8 MHz (fXTALDIV = 1), or to 0x11 otherwise (fXTALDIV = 2)

  /* 0xF44 */
  //spi_ax_transport(0U, "< ff 44 25 >");                          // WR address 0xF44 (RX/TX)
#endif
}

void spi_ax_initRegisters_POCSAG_Rx(void)
{
#if 0
  /* PLLLOOP */
  spi_ax_transport(0U, "< b0 0b >");                           // WR address 0x30: PLLLOOP - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5

  /* PLLCPI */
  spi_ax_transport(0U, "< b1 10 >");                           // WR address 0x31: PLLCPI

  /* PLLVCODIV - mind you: check 0xF34, also */
  spi_ax_transport(0U, "< b2 24 >");                           // WR address 0x32: PLLVCODIV - VCO2INT, RFDIV

  /* PLLLOOPBOOST */
  //spi_ax_transport(0U, "< b8 0b >");                           // WR address 0x38: PLLLOOPBOOST - DIRECT 0x08, no FILTEN, FLT 03: Internal Loop Filter x5 (BW = 500 kHz for ICP = 1.7 mA)


  /* XTALCAP */
  spi_ax_transport(0U, "< f1 84 0c >");                          // WR address 0x184: XTALCAP


  /* 0xF00 */
  spi_ax_transport(0U, "< ff 00 0f >");                          // WR address 0xF00:

  /* 0xF0C */
  //spi_ax_transport(0U, "< ff 0c 00 >");                          // WR address 0xF0C (RX/TX) - Keep the default 0x00

  /* 0xF0D = REF */
  //spi_ax_transport(0U, "< ff 0d 03 >");                          // WR address 0xF0D: REF (RX/TX) - Set to 0x03

  /* 0xF10 - XTALOSC*/
  //spi_ax_transport(0U, "< ff 10 03 >");                          // WR address 0xF10 (RX/TX) - Set to 0x04 if a TCXO is used. If a crystal is used, set to 0x0D if the reference frequency (crystal or TCXO) is more than 43 MHz, or to 0x03 otherwise

  /* 0xF11 - XTALAMPL */
  //spi_ax_transport(0U, "< ff 11 07 >");                          // WR address 0xF11 (RX/TX) - Set to 0x07 if a crystal is connected to CLK16P/CLK16N, or 0x00 if a TCXO is used

  /* 0xF18 */
  spi_ax_transport(0U, "< ff 18 02 >");                          // WR address 0xF18 (RX/TX) - Differs between RX and TX

  /* 0xF1C */
  //spi_ax_transport(0U, "< ff 1c 07 >");                          // WR address 0xF1C (RX/TX) - Set to 0x07

  /* 0xF21 */
  //spi_ax_transport(0U, "< ff 21 68 >");                          // WR address 0xF21 (RX)

  /* 0xF22 */
  //spi_ax_transport(0U, "< ff 22 ff >");                          // WR address 0xF22 (RX)

  /* 0xF23 */
  //spi_ax_transport(0U, "< ff 23 84 >");                          // WR address 0xF23 (RX)

  /* 0xF26 */
  //spi_ax_transport(0U, "< ff 26 98 >");                          // WR address 0xF26 (RX)

  /* 0xF34 */
  //spi_ax_transport(0U, "< ff 34 28 >");                          // WR address 0xF34 (RX/TX) - Set to 0x28 if RFDIV in register PLLVCODIV is set, or to 0x08 otherwise

  /* 0xF35 */
  //spi_ax_transport(0U, "< ff 35 10 >");                          // WR address 0xF35 (RX/TX) - Set to 0x10 for reference frequencies (crystal or TCXO) less than 24.8 MHz (fXTALDIV = 1), or to 0x11 otherwise (fXTALDIV = 2)

  /* 0xF44 */
  //spi_ax_transport(0U, "< ff 44 25 >");                          // WR address 0xF44 (RX/TX)

  /* 0xF72 */
  //spi_ax_transport(0U, "< ff 72 00 >");                          // WR address 0xF72 (RX) - Set to 0x06 if the framing mode is set to "Raw, Soft Bits" (register FRAMING), or to 0x00 otherwise
#endif
}

void spi_ax_initRegisters_POCSAG_Rx_WoR(void)
{
#if 0
  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 83 >");                          // WR address 0x120: AGCGAIN0 - AGCDECAY0 = 14  (f_3db=10 Hz), AGCATTACK0 = 8  (f_3db=621 Hz)

  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 83 >");                          // WR address 0x130: AGCGAIN1 - AGCDECAY1 = 14  (f_3db=10 Hz), AGCATTACK1 = 8  (f_3db=621 Hz)


  /* TMGRXAGC */
  spi_ax_transport(0U, "< f2 27 54 >");                          // WR address 0x227: TMGRXAGC

  /* TMGRXPREAMBLE1 */
  spi_ax_transport(0U, "< f2 29 18 >");                          // WR address 0x229: TMGRXPREAMBLE1

  /* PKTMISCFLAGS */
  spi_ax_transport(0U, "< f2 31 05 >");                          // WR address 0x231: PKTMISCFLAGS
#endif
}

void spi_ax_initRegisters_POCSAG_Rx_cont(void)
{
#if 0
  /* AGCGAIN0 */
  spi_ax_transport(0U, "< f1 20 e8 >");                          // WR address 0x120: AGCGAIN0 - AGCDECAY0 = 14  (f_3db=10 Hz), AGCATTACK0 = 8  (f_3db=621 Hz)

  /* AGCGAIN1 */
  spi_ax_transport(0U, "< f1 30 e8 >");                          // WR address 0x130: AGCGAIN1 - AGCDECAY1 = 14  (f_3db=10 Hz), AGCATTACK1 = 8  (f_3db=621 Hz)


  /* TMGRXAGC */
  spi_ax_transport(0U, "< f2 27 00 >");                          // WR address 0x227: TMGRXAGC - not used

  /* TMGRXPREAMBLE1 */
  spi_ax_transport(0U, "< f2 29 00 >");                          // WR address 0x229: TMGRXPREAMBLE1 - not used

  /* PKTMISCFLAGS */
  //spi_ax_transport(0U, "< f2 31 00 >");                          // WR address 0x231: PKTMISCFLAGS - no BGND RSSI !0x04, RXAGC CLK, RXRSSI CLK clock sources: 1 탎
  spi_ax_transport(0U, "< f2 31 04 >");                          // WR address 0x231: PKTMISCFLAGS - BGND RSSI 0x04, RXAGC CLK, RXRSSI CLK clock sources: 1 탎
#endif
}

void spi_ax_init_POCSAG_Tx(void)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* POCSAG */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.9250);            // VCO1 (internal with    ext. L) with RFDIV --> VCORB = 0x05
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(439.9875);            // VCO1 (internal without ext. L) with RFDIV --> VCORB = 0x09

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load transmitter settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_POCSAG, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Switch to VCO-B - set VCO-PLL to FREQB - 439.9875 MHz */
  (void) spi_ax_selectVcoFreq(1U);

  /* Set power level */
  spi_ax_setPower_dBm(-10.0f);

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the transmitter */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLTX);
}

int8_t spi_ax_run_POCSAG_Tx_FIFO_Msg(uint32_t pocsagTgtRIC, AX_POCSAG_CW2_t pocsagTgtFunc, const char* pocsagTgtMsg, uint8_t pocsagTgtMsgLen)
{
  /* Enter a POCSAG message */
  int8_t ret;

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* 1 - Flags */
  spi_ax_util_POCSAG_Tx_FIFO_Preamble();                            // 576 bits of 1/0 pattern

  /* 2 - Target RIC, message to be sent */
  ret = spi_ax_util_POCSAG_Tx_FIFO_Batches(pocsagTgtRIC, pocsagTgtFunc, pocsagTgtMsg, pocsagTgtMsgLen);

  /* 3 - Delay until message is sent */
#if 0
  do {
    /* RADIOSTATE */
    spi_ax_transport(0U, "< 1c R1 >");                         // RD Address 0x1C: RADIOSTATE - IDLE
  } while ((g_ax_spi_packet_buffer[0] & 0x0f) != 0);
#endif

  return ret;
}

void spi_ax_util_POCSAG_Tx_FIFO_Preamble(void)
{
  uint16_t idx = 0;  (void) idx;

  /* Wait until enough space for next batch is available */
  spi_ax_util_FIFO_waitFree(4 + 18 * 4);

#if 0
  g_ax_spi_packet_buffer[idx++] = 0xA9;                           // WR address 0x29: FIFODATA  (SPI AX address keeps constant)
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_CMD_REPEATDATA_TX;
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_FLAGS_TX_PKTSTART;               // FIFO flag byte
  g_ax_spi_packet_buffer[idx++] = 18 * 4;                           // PREAMBLE length: 576 bits = 18 words
  g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(AX_POCSAG_CODES_PREAMBLE, 0);       // 1/0 pattern

  /* FIFO data enter */
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, idx);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
#endif
}

int8_t spi_ax_util_POCSAG_Tx_FIFO_Batches(uint32_t tgtRIC, AX_POCSAG_CW2_t tgtFunc, const char* tgtMsg, uint8_t tgtMsgLen)
{
  //const uint32_t tgtAddrHi  = calc_BitReverse(18, tgtRIC >> 3);
  uint32_t tgtAddrHi  = tgtRIC >> 3;
  uint8_t  tgtAddrLo  = tgtRIC & 0x7;
  uint16_t msgBitIdx  = 0U;
  uint8_t  batchIdx   = 0U;
  uint8_t  inMsg      = 0U;
  uint8_t  msgDone    = 0U;

  /* Sanity checks */
  {
    if (!tgtRIC) {
      return -1;
    }

    switch (tgtFunc) {
      case AX_POCSAG_CW2_MODE0_NUMERIC:
        if (!tgtMsg || !tgtMsgLen || (tgtMsgLen > 40)) {
          return -2;
        }
      break;

      case AX_POCSAG_CW2_MODE1_TONE:
        {
          /* Any data is silently ignored */
        }
      break;

      case AX_POCSAG_CW2_MODE2_ACTIVATION:
        if (!tgtMsg || !tgtMsgLen || (tgtMsgLen > 10)) {                // TODO: fix me - correct maximum size, here.
          return -2;
        }
      break;

      case AX_POCSAG_CW2_MODE3_ALPHANUM:
        if (!tgtMsg || !tgtMsgLen || ((tgtMsgLen > 80) && (tgtRIC != AX_POCSAG_SKYPER_RIC_NEWS))) {
          return -2;
        }
      break;

      default:
        return -3;
    }
  }

  /* Process message as much batches it needs - each batch is enqueued into the transmitter FIFO */
  do {
    uint8_t idx = 0;  (void) idx;

    /* Wait until enough space for next batch is available */
    spi_ax_util_FIFO_waitFree(4 + 4 + 8 * (4 + 4));                     // FIFO_cmd + SYNCWORD + 8 batches * (2 words)

#if 0
    g_ax_spi_packet_buffer[idx++] = 0xA9;                         // WR address 0x29: FIFODATA  (SPI AX address keeps constant)
    g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_CMD_DATA_TX_RX;
    g_ax_spi_packet_buffer[idx++] = 0;                            // Dummy entry for now
    g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_FLAGS_TX_PKTSTART | AX_FIFO_DATA_FLAGS_TX_NOCRC; // FIFO flag byte

    /* SYNC */
    g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(AX_POCSAG_CODES_SYNCWORD, 3);
    g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(AX_POCSAG_CODES_SYNCWORD, 2);
    g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(AX_POCSAG_CODES_SYNCWORD, 1);
    g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(AX_POCSAG_CODES_SYNCWORD, 0);
#endif

    /* Frames */
    for (uint8_t frIdx = 0; frIdx < 8; frIdx++) {
      for (uint8_t cwIdx = 0; cwIdx < 2; cwIdx++) {
        uint32_t pad;  (void) pad;

        /* WORD */
        if (!inMsg && !msgDone) {
          if (frIdx < tgtAddrLo) {
            pad = AX_POCSAG_CODES_IDLEWORD;

          } else {
            uint32_t addrCW;

            inMsg = 1U;
            addrCW  = tgtAddrHi     << 13;
            addrCW |= (uint32_t)tgtFunc << 11;
            pad   = spi_ax_pocsag_calc_checkAndParity(addrCW);

            /* No message content when TONE is sent */
            if (tgtFunc == AX_POCSAG_CW2_MODE1_TONE) {
              msgDone = 1U;
            }
          }

        } else if (inMsg && !msgDone) {
          uint32_t msgCW;

          msgCW   = spi_ax_pocsag_get20Bits(tgtMsg, tgtMsgLen, tgtFunc, msgBitIdx) << 11;
          msgCW    |= 0x80000000UL;
          msgBitIdx  += 20;
          pad     = spi_ax_pocsag_calc_checkAndParity(msgCW);

          /* Check for message end */
          switch (tgtFunc) {
            case AX_POCSAG_CW2_MODE0_NUMERIC:
              if (tgtMsgLen <= (msgBitIdx >> 2)) {
                msgDone = 1U;
              }
            break;

            #if 0                                 // TODO: to be implemented
            case AX_POCSAG_CW2_MODE2_ACTIVATION:
            break;
            #endif

            case AX_POCSAG_CW2_MODE3_ALPHANUM:
              if (tgtMsgLen <= (msgBitIdx / 7)) {
                msgDone = 1U;
              }
            break;

            default:
              msgDone = 1U;
          }

        } else if (msgDone) {
          inMsg = 0U;
          pad   = AX_POCSAG_CODES_IDLEWORD;

          /* Break transmission after data + IDLEWORD has been sent - leave the loops */
          cwIdx = 2;
          frIdx = 8;
        }

#if 0
        g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(pad, 3);
        g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(pad, 2);
        g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(pad, 1);
        g_ax_spi_packet_buffer[idx++] = s_sel_u8_from_u32(pad, 0);
#endif
      }  // for (cwIdx)
    }  // for (frIdx)

#if 0
    /* Set length for FIFO DATA command */
    g_ax_spi_packet_buffer[    2] = idx - 3;                        // Length

    /* FIFO data enter */
    spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
    spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, idx);
    spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

    /* FIFO do a COMMIT */
    spi_ax_transport(0U, "< a8 04 >");                         // WR address 0x28: FIFOCMD - AX_FIFO_CMD_COMMIT
#endif

    batchIdx++;
  } while (!msgDone || inMsg);

  return 0;
}

void spi_ax_send_POCSAG_Msg(uint32_t pocsagTgtRIC, AX_POCSAG_CW2_t pocsagTgtFunc, const char* pocsagTgtMsg, uint8_t pocsagTgtMsgLen)
{
  /* Send message */
  spi_ax_run_POCSAG_Tx_FIFO_Msg(pocsagTgtRIC, pocsagTgtFunc, pocsagTgtMsg, pocsagTgtMsgLen);

  /* Wait until message in FIFO is sent and transmitter switches off */
#if 0
  do {
    /* FIFOSTAT */
    spi_ax_transport(0U, "< 28 R1 >");
  } while (!(g_ax_spi_packet_buffer[0] & 0x01));
  do {
    /* RADIOSTATE */
    spi_ax_transport(0U, "< 1c R1 >");                       // RD Address 0x1C: RADIOSTATE - IDLE
  } while ((g_ax_spi_packet_buffer[0] & 0x0f) != 0);
#endif
}

void spi_ax_init_POCSAG_Rx(AX_SET_REGISTERS_POWERMODE_t powerMode)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* FSK */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(433.9250);            // VCO1 (internal without ext. L) with RFDIV --> VCORA = 0x09
    /*
    Radiometrix TXL2/RXL2 - 16kbps bi-phase FSK
    433.925MHz - CHAN0
    433.285MHz - CHAN1
    433.605MHz - CHAN2
    434.245MHz - CHAN3
    434.565MHz - CHAN4
    */

    /* POCSAG */
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(439.9875);            // VCO1 (internal without ext. L) with RFDIV --> VCORB = 0x09

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the receiver settings */
  if (powerMode == AX_SET_REGISTERS_POWERMODE_FULLRX) {
    spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_POCSAG, AX_SET_REGISTERS_VARIANT_RX_CONT, AX_SET_REGISTERS_POWERMODE_STANDBY);
  } else if (powerMode == AX_SET_REGISTERS_POWERMODE_WOR) {
    spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_POCSAG, AX_SET_REGISTERS_VARIANT_RX_WOR, AX_SET_REGISTERS_POWERMODE_STANDBY);
  }

  /* Switch to VCO-B - set VCO-PLL to FREQB - 439.9875 MHz */
  (void) spi_ax_selectVcoFreq(1U);

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Change PowerMode */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, powerMode);
}


void spi_ax_initRegisters_AnlogFM(void)
{
  /* MODULATION */
  //spi_ax_transport(0U, "< 90 0b >");                           // WR address 0x10: MODULATION - 0B: Analog FM
}

void spi_ax_initRegisters_AnlogFM_Tx(void)
{
#if 0
  /* PINFUNCDATA */
  spi_ax_transport(0U, "< a3 04 >");                           // WR address 0x23: PINFUNCDATA - DATA Input/Output Modem Data: enables continuous TX operation, rather than powering up the PA only if there is committed FIFO data. This is similar to wire mode, except that no data is read from the pin in FM mode.

  /* PINFUNCTCXO_EN */
  spi_ax_transport(0U, "< a6 00 >");                           // WR address 0x26: PINFUNCTCXO_EN - Output '0'

  /* FSKDEV */
  spi_ax_transport(0U, "< f1 61 00 c0 07 >");                        // WR address 0x161: FSKDEV - GPADC13, enable sign extension and offset (=midcode) subtraction, fdeviation = � 65 kHz [max / min ADC value gives fdeviation = � fxtal / 2^(AX5043_FSKDEV0[2:0]+1), allowed values are 0..7

  /* GPADCCTRL */
  spi_ax_transport(0U, "< f3 00 06 >");                          // WR address 0x300: GPADCCTRL - continuous sampling of GPADC13

  /* GPADCPERIOD */
  spi_ax_transport(0U, "< f3 01 07 >");                          // WR address 0x301: GPADCPERIOD - Fs = fxtal/32/GPADCPERIOD  5 gives 100 kHz @ fxtal = 16 MHz. This determines the sampling rate, TXRATE has no meaning in FM mode.

  /* DACVALUE */
  spi_ax_transport(0U, "< f3 30 00 00 >");                         // WR address 0x330: DACVALUE - off

  /* DACCONFIG */
  spi_ax_transport(0U, "< f3 32 80 >");                          // WR address 0x332: DACCONFIG - DACPWM, output DACVALUE

  /* 0xF18 */
  spi_ax_transport(0U, "< ff 18 06 >");                          // WR address 0xF18 (RX/TX) - ? (is set to 0x06, explicit named for using Analog FM)
#endif
}

void spi_ax_initRegisters_AnlogFM_Rx(void)
{
#if 0
  /* PINFUNCTCXO_EN */
  spi_ax_transport(0U, "< a6 05 >");                           // WR address 0x26: PINFUNCTCXO_EN - Use TCXO_EN pin as DAC output

  /* IFFREQ */
  spi_ax_transport(0U, "< f1 00 06 66 >");                         // WR address 0x100: IFFREQ - 25 kHz (f_xtal = 16 MHz)

  /* MAXDROFFSET */
  spi_ax_transport(0U, "< f1 06 00 00 00 >");                        // WR address 0x106: MAXDROFFSET - off

  /* MAXRFOFFSET */
  spi_ax_transport(0U, "< f1 09 80 cc cc >");                        // WR address 0x109: MAXRFOFFSET - track at LO1, max 50 kHz @ f_xtal = 16 MHz

  /* FREQUENCYLEAK */
  spi_ax_transport(0U, "< f1 16 04 >");                          // WR address 0x116: FREQUENCYLEAK - FREQUENCYGAINB0 + 2, prevents the demodulator AFC loop from tracking static frequency offsets

  /* RXPARAMSETS */
  spi_ax_transport(0U, "< f1 17 00 >");                          // WR address 0x117: RXPARAMSETS - only use receiver parameter set 0

  /* TIMEGAIN0 */
  spi_ax_transport(0U, "< f1 24 00 >");                          // WR address 0x124: TIMEGAIN0 - disable bit timing recovery, which would only add jitter

  /* DRGAIN0 */
  spi_ax_transport(0U, "< f1 25 00 >");                          // WR address 0x125: DRGAIN0 - off

  /* FREQGAINA0 */
  spi_ax_transport(0U, "< f1 27 a0 >");                          // WR address 0x127: FREQGAINA0 - off

  /* FREQGAINB0 */
  spi_ax_transport(0U, "< f1 28 02 >");                          // WR address 0x128: FREQGAINB0 - bandwidth of "inner" AFC loop used for FM demodulation. f_3dB = 0.115*BR. This is the fastest setting available

  /* FREQGAINC0 */
  spi_ax_transport(0U, "< f1 29 1f >");                          // WR address 0x129: FREQGAINC0 - off

  /* FREQGAIND0 */
  spi_ax_transport(0U, "< f1 2a 08 >");                          // WR address 0x12A: FREQGAIND0 - bandwidth of "outer" AFC loop (tracking frequency mismatch), 78 Hz @ BR = 100 kbps, f_xtal = 16 MHz

  /* DACVALUE */
  spi_ax_transport(0U, "< f3 30 00 0c >");                         // WR address 0x330: DACVALUE - DACSHIFT = 12 bit. This gives maximum volume, downshifting further gives smaller volume

  /* DACCONFIG */
  spi_ax_transport(0U, "< f3 32 83 >");                          // WR address 0x332: DACCONFIG - DACPWM, output TRKFREQ (= demodulated signal) on DAC

  /* 0xF18 */
  spi_ax_transport(0U, "< ff 18 06 >");                          // WR address 0xF18 (RX/TX) - ? (is set to 0x06, explicit named for using Analog FM)
#endif
}

void spi_ax_init_AnalogFM_Tx(void)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* Burst-Aussendungen fuer Steuerungszwecke - lower and upper frequencies */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.9245);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9255);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the transmitter settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_ANALOG_FM, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  (void) spi_ax_selectVcoFreq(0U);

  /* Set power level */
  spi_ax_setPower_dBm(-10.0f);

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                         // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the transmitter */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLTX);
}

void spi_ax_init_AnalogFM_Rx(void)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* Burst-Aussendungen fuer Steuerungszwecke - lower and upper frequencies */
    s_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.9245);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05
    s_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9255);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* @see AND9347/D, page 12: Preparation 4. */
    /* Auto ranging and storing */
    spi_ax_doRanging();
  }

  /* Load the receiver settings */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_ANALOG_FM, AX_SET_REGISTERS_VARIANT_RX_CONT, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  (void) spi_ax_selectVcoFreq(0U);

  /* FIFOCMD / FIFOSTAT */
  //spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  /* Enabling the receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLRX);
}


#if defined(AX_GPADC13_ENABLED)
static void spi_ax_adcCtrlSet(uint8_t val)
{
  g_ax_spi_packet_buffer[0] = 0xF3;                             // WR Address 0x300: GPADCCTRL
  g_ax_spi_packet_buffer[1] = 0x30;
  g_ax_spi_packet_buffer[2] = val;

  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 3);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
}

static uint8_t spi_ax_adcCtrlGet(void)
{
  /* GPADCCTRL */
  spi_ax_transport(0U, "< 73 30 R1 >");                          // RD address 0x300: GPADCCTRL
  return g_ax_spi_packet_buffer[0];
}

static uint16_t spi_ax_adcValGet(void)
{
  /* GPADC13VALUE */
  spi_ax_transport(0U, "< 73 08 R2 >");                          // RD address 0x308: GPADC13VALUE

  uint16_t val = g_ax_spi_packet_buffer[0] & 0x03;
  val <<= 8;
  val  |= g_ax_spi_packet_buffer[1];
  return val;
}

inline
static void spi_ax_adcConvertionWait(void)
{
  do { }  while (spi_ax_adcCtrlGet() & 0x80);                         // GPADCCTRL - BUSY Conversion ongoing when 1
}

static uint16_t spi_axr_getVcoTuneVoltage(void)
{
    int16_t tuneVoltage = 0;

  /* Prepare the ADC */
    uint8_t cnt = 64;
    do {
    spi_ax_adcCtrlSet(0x84);                                // GPADCCTRL - BUSY When writing 1, a single conversion is started - GPADC13 Enable Sampling GPADC1?GPADC3
        spi_ax_adcConvertionWait();
    } while (--cnt);

    cnt = 32;
    do {
    spi_ax_adcCtrlSet(0x84);                                // GPADCCTRL - BUSY When writing 1, a single conversion is started - GPADC13 Enable Sampling GPADC1?GPADC3
        spi_ax_adcConvertionWait();
    tuneVoltage += spi_ax_adcValGet();
    } while (--cnt);

    return tuneVoltage;
}

/* REMARKS: the AX5243 has got an automatic VCOI adjustment built in - in contrast this function does the setting manually with the help of the GPADC13 */
static uint8_t s_spi_ax_cal_vcoi(void)
{
    uint8_t vcoiRet = 0;
    uint16_t vmin = 0xffff;
    uint16_t vmax = 0x0000;

    for (uint8_t vcoiCurrentIdx = 0x40; vcoiCurrentIdx; ) {
        --vcoiCurrentIdx;
        uint8_t vcoiNew = 0x80 | vcoiCurrentIdx;                        // VCOIE - Enable manual VCOI

    /* PLLVCOI */
    g_ax_spi_packet_buffer[0] = 0xf1;                           // WR address 0x180: PLLVCOI
    g_ax_spi_packet_buffer[1] = 0x80;
    g_ax_spi_packet_buffer[2] = vcoiNew;

    spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
    spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 3);                 // Write back with modified value
    spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

        /* Clear STICKY LOCK */
    spi_ax_transport(0U, "< 33 R1 >");                         // RD Address 0x33: PLLRANGINGA

        uint16_t curTuneVoltage = spi_axr_getVcoTuneVoltage();

        /* Clear STICKY LOCK again */
    spi_ax_transport(0U, "< 33 R1 >");                         // RD Address 0x33: PLLRANGINGA

        //global_var[vcoiCurrentIdx] = curTuneVoltage;                      // For debugging VCO_TuneVoltage = f(VCOI)
        if (curTuneVoltage > vmax) {
            vmax = curTuneVoltage;
    }

        if (curTuneVoltage < vmin) {
            vmin = curTuneVoltage;

            /* Check whether the PLL is still locked */
      spi_ax_transport(0U, "< 33 R1 >");                       // RD Address 0x33: PLLRANGINGA
      uint8_t curPllrangingA = g_ax_spi_packet_buffer[0];

      /* Both flags STICKY LOCK and PLL LOCK are set */
            if (!(0xc0 & ~curPllrangingA)) {
                vcoiRet = vcoiCurrentIdx;
      }
        }
    }

  /* Security checks */
    if (!vcoiRet || (vmax >= 0xFF00) || (vmin < 0x0100) || ((vmax - vmin) < 0x6000)) {
        return 0;
  }
    return vcoiRet;
}
#endif


void spi_ax_setTxRxMode(AX_SET_TX_RX_MODE_t txRxMode)
{
  static AX_SET_TX_RX_MODE_t lastTxRxMode = AX_SET_TX_RX_MODE_OFF;

  if (lastTxRxMode != txRxMode) {
    lastTxRxMode  = txRxMode;

    /* FIFOCMD / FIFOSTAT */
    //spi_ax_transport(0U, "< a8 03 >");                               // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

    switch (txRxMode) {
      case AX_SET_TX_RX_MODE_APRS_RX_WOR:
        spi_ax_init_PR1200_Rx(AX_SET_REGISTERS_POWERMODE_WOR);
      break;

      case AX_SET_TX_RX_MODE_APRS_RX_CONT:
      case AX_SET_TX_RX_MODE_APRS_RX_CONT_SINGLEPARAMSET:
        spi_ax_init_PR1200_Rx(AX_SET_REGISTERS_POWERMODE_FULLRX);
      break;

      case AX_SET_TX_RX_MODE_APRS_TX:
        spi_ax_init_PR1200_Tx();
      break;


      case AX_SET_TX_RX_MODE_POCSAG_RX_WOR:
        spi_ax_init_POCSAG_Rx(AX_SET_REGISTERS_POWERMODE_WOR);
      break;

      case AX_SET_TX_RX_MODE_POCSAG_RX_CONT:
      case AX_SET_TX_RX_MODE_POCSAG_RX_CONT_SINGLEPARAMSET:
        spi_ax_init_POCSAG_Rx(AX_SET_REGISTERS_POWERMODE_FULLRX);
      break;

      case AX_SET_TX_RX_MODE_POCSAG_TX:
        spi_ax_init_POCSAG_Tx();
      break;


      default:
        spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_INVALIDATE, AX_SET_REGISTERS_VARIANT_INVALIDATE, AX_SET_REGISTERS_POWERMODE_DEEPSLEEP);
    }
  }
}

uint8_t spi_ax_doProcess_RX_messages(const uint8_t* buf, uint8_t msgLen)
{
  uint8_t   msgCnt      = 0;
  uint16_t  msgPos      = 0;
  uint8_t   fifoCmd     = 0;
  uint8_t   fifoDataLen   = 0;

  /* Process each FIFO message */
  while (msgPos < msgLen) {
    fifoCmd = *(buf + msgPos++);

    switch (fifoCmd) {
      case 0b00000001:                                    // TRX change: Own CMD for TRX mode change
      {
        __asm volatile( "nop" );
      }
      break;

      case 0b00110001:                                    // RSSI: Receive Signal Strength Indicator
      case 0b00110010:                                    // RSSI: Own CMD for RSSI
      {
        s_ax_rx_fifo_meas.rssi = *(buf + msgPos++);

        msgCnt++;
      }
      break;

      case 0b01010010:                                    // FREQOFFS: Frequency Offset
      {
        uint8_t fifoFrqOffsHi = *(buf + msgPos++);
        uint8_t fifoFrqOffsLo = *(buf + msgPos++);

        s_ax_rx_fifo_meas.frqOffs = (int16_t) ((uint16_t)fifoFrqOffsHi << 8 | fifoFrqOffsLo);

        msgCnt++;
      }
      break;

      case 0b01010101:                                    // ANTRSSI2: Background Noise Calculation RSSI
      {
        uint8_t fifoAntRssi2Hi = *(buf + msgPos++);
        uint8_t fifoAntRssi2Lo = *(buf + msgPos++);

        s_ax_rx_fifo_meas.antRssi2 = (int16_t) ((uint16_t)fifoAntRssi2Hi << 8 | fifoAntRssi2Lo);

        msgCnt++;
      }
      break;

      case 0b01110000:                                    // TIMER: Timestamp
      {
        uint8_t fifoTimerHi  = *(buf + msgPos++);
        uint8_t fifoTimerMid = *(buf + msgPos++);
        uint8_t fifoTimerLo  = *(buf + msgPos++);

        s_ax_rx_fifo_meas.timer = (uint32_t)fifoTimerHi << 16 | (uint32_t)fifoTimerMid << 8 | fifoTimerLo;

        msgCnt++;
      }
      break;

      case 0b01110011:                                    // RFFREQOFFS: RF Frequency Offset
      {
        uint8_t fifoRfFrqOffsHi  = *(buf + msgPos++);
        uint8_t fifoRfFrqOffsMid = *(buf + msgPos++);
        uint8_t fifoRfFrqOffsLo  = *(buf + msgPos++);

        s_ax_rx_fifo_meas.rfFrqOffs = (int32_t) ((uint32_t)fifoRfFrqOffsHi << 16 | (uint32_t)fifoRfFrqOffsMid << 8 | fifoRfFrqOffsLo);

        msgCnt++;
      }
      break;

      case 0b01110100:                                    // DATARATE: Datarate
      {
        uint8_t fifoDataRateHi  = *(buf + msgPos++);
        uint8_t fifoDataRateMid = *(buf + msgPos++);
        uint8_t fifoDataRateLo  = *(buf + msgPos++);

        s_ax_rx_fifo_meas.dataRate = (uint32_t)fifoDataRateHi << 16 | (uint32_t)fifoDataRateMid << 8 | fifoDataRateLo;

        msgCnt++;
      }
      break;

      case 0b01110101:                                    // ANTRSSI3: Antenna Selection RSSI
      {
        uint8_t fifoAntRssi3Hi  = *(buf + msgPos++);
        uint8_t fifoAntRssi3Mid = *(buf + msgPos++);
        uint8_t fifoAntRssi3Lo  = *(buf + msgPos++);

        s_ax_rx_fifo_meas.antRssi3 = (int32_t) ((uint32_t)fifoAntRssi3Hi << 16 | (uint32_t)fifoAntRssi3Mid << 8 | fifoAntRssi3Lo);

        msgCnt++;
      }
      break;

      case 0b11100001:
      {
        fifoDataLen = *(buf + msgPos++);

        AX_SET_MON_MODE_t l_ax_set_mon_mode = s_ax_set_mon_mode;

        spi_ax_Rx_FIFO_DataProcessor(l_ax_set_mon_mode, buf + msgPos, fifoDataLen);
        msgPos += fifoDataLen;

        msgCnt++;
      }
      break;

      default:
      {
        // Nothing to do
        __asm volatile( "nop" );
      }
    }  // switch(fifoCmd)
  }  // while (msgPos < msgLen)

  return msgCnt;
}

void spi_ax_Rx_FIFO_DataProcessor(AX_SET_MON_MODE_t monMode, const uint8_t* dataBuf, uint16_t dataLen)
{
  #define S_POCSAG_DATA_SIZE  32

  static uint8_t  s_pocsagState             = AX_DECODER_POCSAG__NONE;
  static uint32_t s_pocsagFIFOWord            = 0UL;
  static uint8_t  s_pocsagFIFOWordLen           = 0;
  static uint8_t  s_pocsagWordCtr             = 0;
  static uint32_t s_pocsagData_Addr           = 0UL;
  static uint8_t  s_pocsagData_FunctionBits       = 0;
  static uint32_t s_pocsagData_Data[S_POCSAG_DATA_SIZE] = { 0UL };
  static uint8_t  s_pocsagData_DataCnt          = 0;

  /* Sanity check */
  if (!dataLen) {
    return;
  }

  switch (monMode) {
    /* APRS / PR1200 UI-Frames */
    case AX_SET_MON_MODE_APRS_RX_CONT:
    case AX_SET_MON_MODE_APRS_RX_CONT_SINGLEPARAMSET:
    case AX_SET_MON_MODE_APRS_RX_WOR:
    {
      __asm volatile( "nop" );
    }
    break;

    /* POCSAG */
    case AX_SET_MON_MODE_POCSAG_RX_CONT:
    case AX_SET_MON_MODE_POCSAG_RX_CONT_SINGLEPARAMSET:
    case AX_SET_MON_MODE_POCSAG_RX_WOR:
    {
      uint8_t           status        = *dataBuf;
      uint16_t          dataPos       = 1U;
      uint32_t          pocsagWord      = 0UL;
      uint8_t           pocsagWordBytePos = 0;
      AX_POCSAG_DECODER_DATA_t  l_pocsagData;
      //const char*         l_pocsagStateStr  = 0;

      if (status & AX_FIFO_DATA_FLAGS_RX_PKTSTART) {
        /* Process last data */
        if (s_pocsagData_DataCnt) {
          /* Decode the previous message */
          spi_ax_pocsag_messageDecoder(s_pocsagData_Addr, s_pocsagData_FunctionBits, s_pocsagData_Data, s_pocsagData_DataCnt);

          /* Reset data */
          s_pocsagData_Addr     = 0UL;
          s_pocsagData_FunctionBits = 0;
          s_pocsagData_DataCnt    = 0;
        }

        s_pocsagFIFOWord  = 0UL;
        s_pocsagFIFOWordLen = 0;
        s_pocsagWordCtr   = 0;
        s_pocsagState   = AX_DECODER_POCSAG__SYNC;
      }

      /* Pull FIFO data first */
      while (pocsagWordBytePos < s_pocsagFIFOWordLen) {
        /* Transfer data */
        pocsagWord <<= 8;
        pocsagWord  |= s_pocsagFIFOWord & 0xff;

        /* Adjust */
        pocsagWordBytePos++;
        s_pocsagFIFOWord >>= 8;
      }
      s_pocsagFIFOWordLen = 0;

      /* Process the data buffer */
      while ((dataPos + 4) < dataLen) {
        /* Fill the rest from the buffer for a complete WORD */
        while (pocsagWordBytePos < 4) {
          /* Transfer data */
          pocsagWord <<= 8;
          pocsagWord  |= *(dataBuf + dataPos++) & 0xff;

          /* Adjust */
          pocsagWordBytePos++;
        }
        pocsagWordBytePos = 0;

        /* Restart receiver at once to catch the next SYNCWORD */
        if ((pocsagWord == 0x55555555UL) || (pocsagWord == 0xaaaaaaaaUL)) {
          /* Interrupt disabled actions */
          {
#if 0
            /* FRMMODE abort frame */
            spi_ax_transport(0U, "< 92 07 >");                   // WR address 0x12: FRAMING - CRCMODE: none, FRMMODE: Raw, Pattern Match 0x06, FABORT

            /* FIFOCMD / FIFOSTAT */
            spi_ax_transport(0U, "< a8 03 >");                   // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

            g_ax_spi_rx_buffer_idx = 0;
            memset(g_ax_spi_rx_buffer, 0, C_SPI_AX_BUFFER_LENGTH);
#endif
          }

          /* DEBUG WORDs */
          #if 0
          int16_t l_rssi    = g_ax_rx_fifo_meas.rssi; l_rssi    -= 64;
          int16_t l_bgnd_rssi = g_ax_spi_rx_bgnd_rssi;  l_bgnd_rssi -= 64;
          uint8_t len = (uint8_t) sprintf(g_prepare_buf, "\tstatus = 0x%02x, WORD = 0x%08lx, RSSI = %-4ddBm, bgnd_RSSI = %-4ddBm, ***ABORT_FRAME***\r\n",
                          status, pocsagWord,
                          l_rssi, l_bgnd_rssi);
          udi_write_tx_buf(g_prepare_buf, len, 0U);
          #endif

          /* Make current data non-valid */
          s_pocsagFIFOWord  = 0UL;
          s_pocsagFIFOWordLen = 0;
          s_pocsagWordCtr   = 0;
          s_pocsagState   = AX_DECODER_POCSAG__NONE;

          return;
        }

        /* WORD decoder */
        spi_ax_pocsag_wordDecoder(&l_pocsagData, pocsagWord, s_pocsagWordCtr);

        #if 0
        /* DEBUG state strings */
        switch (s_pocsagState) {
          case AX_DECODER_POCSAG__NONE:
            l_pocsagStateStr = "NONE ";
          break;

          case AX_DECODER_POCSAG__SYNC:
            l_pocsagStateStr = "SYNC ";
          break;

          case AX_DECODER_POCSAG__IDLE:
            l_pocsagStateStr = "IDLE ";
          break;

          case AX_DECODER_POCSAG__ADDRESS:
            l_pocsagStateStr = "ADDR ";
          break;

          case AX_DECODER_POCSAG__DATA:
            l_pocsagStateStr = "DATA ";
          break;

          case AX_DECODER_POCSAG__FLUSH:
            l_pocsagStateStr = "FLUSH";
          break;

          default:
            l_pocsagStateStr = "";
        }

        /* DEBUG WORDs */
        int16_t l_rssi    = g_ax_rx_fifo_meas.rssi; l_rssi    -= 64;
        int16_t l_bgnd_rssi = g_ax_spi_rx_bgnd_rssi;  l_bgnd_rssi -= 64;
        uint8_t len = (uint8_t) sprintf(g_prepare_buf, "\tDecoder=%s\tstatus = 0x%02x, WORD = 0x%08lx, RSSI = %-4ddBm, bgnd_RSSI = %-4ddBm, %c (EC=%c)\r\n",
                        l_pocsagStateStr,
                        status, pocsagWord,
                        l_rssi, l_bgnd_rssi,
                        l_pocsagData.badDecode ?  '-' : '+',
                        (!l_pocsagData.badDecode && (l_pocsagData.invertedBit != 32) ?  '1' : (l_pocsagData.badDecode ?  '-' : '0')));
        udi_write_tx_buf(g_prepare_buf, len, 0U);
        #endif

        if (!l_pocsagData.badDecode) {
          if (l_pocsagData.isAddr) {
            /* Known addresses of these WORDs - HI-part */
            switch (l_pocsagData.addrData >> 3) {
              case ((AX_POCSAG_CODES_SYNCWORD >> 13) & 0x0007ffffUL):
                s_pocsagState = AX_DECODER_POCSAG__SYNC;
              break;

              case ((AX_POCSAG_CODES_IDLEWORD >> 13) & 0x0007ffffUL):
                s_pocsagState = AX_DECODER_POCSAG__IDLE;
              break;

              default:
                s_pocsagState = AX_DECODER_POCSAG__ADDRESS;
            }  // switch(pocsagWord)

          } else if (l_pocsagData.isData) {
            if (s_pocsagState == AX_DECODER_POCSAG__ADDRESS) {
              s_pocsagState = AX_DECODER_POCSAG__DATA;

            } else if (s_pocsagState == AX_DECODER_POCSAG__DATA) {
              // keep state

            } else {
              s_pocsagState = AX_DECODER_POCSAG__NONE;
            }
          }  // else if (l_pocsagData.isData)

        } else {
          s_pocsagState = AX_DECODER_POCSAG__FLUSH;

          /* Get RSSI and take that as background RSSI */
          {
#if 0
            int16_t   l_cur_bgnd_rssi;
            int16_t   l_mean_bgnd_rssi;

            spi_ax_transport(0U, "< 40 R1 >");                 // RD Address 0x40: RSSI
            l_cur_bgnd_rssi = (int8_t) (g_ax_spi_packet_buffer[0]);

            l_mean_bgnd_rssi = s_ax_spi_rx_bgnd_rssi;

            /* 64 dB offset correction */
            l_cur_bgnd_rssi   -= 64;
            l_mean_bgnd_rssi  -= 64;

            /* Calculate mean value with weight 7 and 1 */
            l_mean_bgnd_rssi  *= 7;
            l_mean_bgnd_rssi  += l_cur_bgnd_rssi;
            l_mean_bgnd_rssi >>= 3;

            /* 64 dB offset encoding */
            l_mean_bgnd_rssi += 64;

            s_ax_spi_rx_bgnd_rssi = (int8_t) (l_mean_bgnd_rssi & 0xff);
#endif
          }
        }  // if (l_pocsagData.badDecode)


        switch (s_pocsagState) {
          case AX_DECODER_POCSAG__ADDRESS:
          {
            /* Process last data */
            if (s_pocsagData_DataCnt) {
              /* Decode the previous message */
              spi_ax_pocsag_messageDecoder(s_pocsagData_Addr, s_pocsagData_FunctionBits, s_pocsagData_Data, s_pocsagData_DataCnt);

              /* Reset data */
              s_pocsagData_Addr     = 0UL;
              s_pocsagData_FunctionBits = 0;
              s_pocsagData_DataCnt    = 0;
            }

            /* New address follows */
            s_pocsagData_Addr     = l_pocsagData.addrData;
            s_pocsagData_FunctionBits = l_pocsagData.functionBits;
          }
          break;

          case AX_DECODER_POCSAG__DATA:
          {
            /* Store data for processing */
            if (s_pocsagData_DataCnt < S_POCSAG_DATA_SIZE) {
              s_pocsagData_Data[s_pocsagData_DataCnt++] = l_pocsagData.addrData;

            } else {
              if (s_pocsagData_DataCnt) {
                /* When message is to long for buffer do break message */
                spi_ax_pocsag_messageDecoder(s_pocsagData_Addr, s_pocsagData_FunctionBits, s_pocsagData_Data, s_pocsagData_DataCnt);

                /* Reset data */
                s_pocsagData_Addr     = 0UL;
                s_pocsagData_FunctionBits = 0;
                s_pocsagData_DataCnt    = 0;
              }

              s_pocsagState = AX_DECODER_POCSAG__NONE;
            }
          }
          break;

          case AX_DECODER_POCSAG__FLUSH:
          {
            if (s_pocsagData_DataCnt) {
              /* Flush the data as much it is usable */
              spi_ax_pocsag_messageDecoder(s_pocsagData_Addr, s_pocsagData_FunctionBits, s_pocsagData_Data, s_pocsagData_DataCnt);

              /* Reset data */
              s_pocsagData_Addr     = 0UL;
              s_pocsagData_FunctionBits = 0;
              s_pocsagData_DataCnt    = 0;
            }

            s_pocsagState = AX_DECODER_POCSAG__NONE;
          }
          break;

          case AX_DECODER_POCSAG__IDLE:
          {
            if (s_pocsagData_DataCnt) {
              /* Decode the previous message */
              spi_ax_pocsag_messageDecoder(s_pocsagData_Addr, s_pocsagData_FunctionBits, s_pocsagData_Data, s_pocsagData_DataCnt);

              /* Reset data */
              s_pocsagData_Addr     = 0UL;
              s_pocsagData_FunctionBits = 0;
              s_pocsagData_DataCnt    = 0;
            }
          }
          break;

          case AX_DECODER_POCSAG__SYNC:
          {
            /* Process last data */
            if (s_pocsagData_DataCnt) {
              /* Decode the previous message */
              spi_ax_pocsag_messageDecoder(s_pocsagData_Addr, s_pocsagData_FunctionBits, s_pocsagData_Data, s_pocsagData_DataCnt);

              /* Reset data */
              s_pocsagData_Addr     = 0UL;
              s_pocsagData_FunctionBits = 0;
              s_pocsagData_DataCnt    = 0;
            }

            s_pocsagWordCtr = 0;
          }

          case AX_DECODER_POCSAG__NONE:
          default:
          {
            /* Reset data */
            s_pocsagData_Addr     = 0UL;
            s_pocsagData_FunctionBits = 0;
            s_pocsagData_DataCnt    = 0;
          }
        }  // switch (s_pocsagState)

        s_pocsagWordCtr++;
      }  // while ((dataPos + 4) < dataLen)

      /* Put sliced data into the FIFO word */
      if (dataPos < dataLen) {
        s_pocsagFIFOWord  = 0UL;
        s_pocsagFIFOWordLen = 0;
        do {
          /* Push into MSB */
          s_pocsagFIFOWord >>= 8;
          s_pocsagFIFOWord  |= ((uint32_t) ((*(dataBuf + dataPos++)) & 0xff)) << 24;

          s_pocsagFIFOWordLen++;
        } while (dataPos < dataLen);

        s_pocsagFIFOWord >>= ((4 - s_pocsagFIFOWordLen) << 3);
        return;
      }
    }
    break;

    default:
    {
      __asm volatile( "nop" );
    }
  }
}


const char          PM_SPI_INIT_AX5243_01[]       = "\r\nSPI  AX5243: AX5243 VHF/UHF transceiver\r\n";
const char          PM_SPI_INIT_AX5243_02[]       = "SPI  AX5243:  Silicon-Rev: 0x%02X\r\n";
const char          PM_SPI_INIT_AX5243_03[]       = "SPI  AX5243:  --> INIT success.\r\n";
const char          PM_SPI_INIT_AX5243_04[]       = "SPI  AX5243:  --> device not found on board.\r\n";
//PROGMEM_DECLARE(const char, PM_SPI_INIT_AX5243_01[]);
//PROGMEM_DECLARE(const char, PM_SPI_INIT_AX5243_02[]);
//PROGMEM_DECLARE(const char, PM_SPI_INIT_AX5243_03[]);
//PROGMEM_DECLARE(const char, PM_SPI_INIT_AX5243_04[]);
static void init_spi_ax5243(void)
{
  //status_code_t sc;
  int  dbgLen;
  char dbgBuf[128];

  dbgLen = snprintf(dbgBuf, sizeof(dbgBuf), PM_SPI_INIT_AX5243_01);
  usbLogLen(dbgBuf, min(dbgLen, sizeof(dbgLen)));

  do {
    /* Reset AX5243 and go to Powerdown mode */
#if 0
    sc = spi_ax_sync2Powerdown();
    if (sc != STATUS_OK) {
      break;
    }
#endif

    /* AX5243: read Silicon-Rev byte */
#if 0
    sc = spi_ax_transport(0U, "< 00 R1 >");                        // RD Address 0x00: REVISION
    if (sc != STATUS_OK) {
      break;
    }
#endif
    uint8_t rev = 0x99;  //g_ax_spi_packet_buffer[0];

    dbgLen = snprintf(dbgBuf, sizeof(dbgLen), PM_SPI_INIT_AX5243_02, rev);
    usbLogLen(dbgBuf, min(dbgLen, sizeof(dbgBuf)));
    osDelay(2);

    /* Known chip revision */
    if (rev == 0x51) {
      dbgLen = snprintf(dbgBuf, sizeof(dbgLen), PM_SPI_INIT_AX5243_03);
      usbLogLen(dbgBuf, min(dbgLen, sizeof(dbgBuf)));
      osDelay(2);
      s_ax_enable = 1U;
      return;
    }
  } while (0U);

  dbgLen = snprintf(dbgBuf, sizeof(dbgLen), PM_SPI_INIT_AX5243_04);
  usbLogLen(dbgBuf, min(dbgLen, sizeof(dbgBuf)));
  osDelay(2);

  /* No chip on board */
  s_ax_enable = 0U;
}


void spi_init(void) {
  /* Set init level */
#if 0
  ioport_set_pin_mode(AX_SEL,  (uint8_t) (IOPORT_INIT_HIGH | IOPORT_SRL_ENABLED));      // SEL  inactive
  ioport_set_pin_mode(AX_CLK,  (uint8_t) (IOPORT_INIT_HIGH | IOPORT_SRL_ENABLED));      // CLK  inactive
  ioport_set_pin_mode(AX_MOSI, (uint8_t) (IOPORT_INIT_HIGH | IOPORT_SRL_ENABLED));      // MOSI inactive

  /* Set port directions */
  ioport_set_pin_dir(AX_SEL,      IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(AX_CLK,      IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(AX_MOSI,     IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(AX_IRQ,      IOPORT_DIR_INPUT);
  ioport_set_pin_dir(AX_MISO,     IOPORT_DIR_INPUT);

  /* Set input pulling resistors */
  ioport_set_pin_mode(AX_IRQ,     IOPORT_MODE_PULLUP);
  ioport_set_pin_mode(AX_MISO,    IOPORT_MODE_BUSKEEPER);
#endif
}

void spi_start(void) {
#if 0
  g_ax_spi_device_conf.id = AX_SEL;

  spi_master_init(&SPI_AX);
  spi_master_setup_device(&SPI_AX, &g_ax_spi_device_conf, SPI_MODE_0, 10000000, 0);     // max. 10 MHz (100 ns) when AX in POWERDOWN mode
  spi_enable(&SPI_AX);

  init_spi_ax5243();


  #ifdef AX_TEST
  /* Frequency settings */
  {
    #if defined(AX_RUN_VCO2_APRS_TX)
      /* Default setting for the application: APRS */
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_PR1200, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* APRS  */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.8000);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05

      /* Burst-Aussendungen fuer Steuerungszwecke */
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9250);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05


    #elif defined(AX_TEST_VCO1_BANDENDS)
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_FSK, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* VCO A/B settings */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(400.0000);          // VCO1 (internal without ext. L) with RFDIV --> VCORA = 0x0e
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(525.0000);          // VCO1 (internal without ext. L) with RFDIV --> VCORB = 0x02

    #elif defined(AX_TEST_VCO1_FSK_TX) | defined(AX_TEST_VCO1_FSK_RX) | defined(AX_TEST_VCO1_POCSAG_TX) | defined(AX_TEST_VCO1_POCSAG_RX)
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_FSK, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* FSK */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(433.9250);          // VCO1 (internal without ext. L) with RFDIV --> VCORA = 0x09
      /*
      Radiometrix TXL2/RXL2 - 16kbps bi-phase FSK
      433.925MHz - CHAN0
      433.285MHz - CHAN1
      433.605MHz - CHAN2
      434.245MHz - CHAN3
      434.565MHz - CHAN4
      */

      /* POCSAG */
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(439.9875);          // VCO1 (internal without ext. L) with RFDIV --> VCORB = 0x09

    #elif defined(AX_TEST_VCO2_BANDENDS)
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_FSK, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* VCO A/B settings */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(137.0000);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x0e
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(149.0000);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x00

    #elif defined(AX_TEST_VCO2_ANALOG_FM_RX)
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_ANALOG_FM, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* APRS */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.8000);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05

      /* DB0ZH */
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(145.6250);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x04

    #elif defined(AX_TEST_VCO2_ANALOG_FM_TX)
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_ANALOG_FM, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* Burst-Aussendungen fuer Steuerungszwecke - lower and upper frequencies */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.9245);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9255);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05

    #elif defined(AX_TEST_VCO2_PR1200_TX) | defined(AX_TEST_VCO2_PR1200_RX)
      /* Syncing and sending reset command, then setting the default values */
      spi_ax_setRegisters(1U, AX_SET_REGISTERS_MODULATION_PR1200, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

      /* APRS  */
      g_ax_spi_freq_chan[0] = spi_ax_calcFrequency_Mhz2Regs(144.8000);          // VCO2 (internal with    ext. L) with RFDIV --> VCORA = 0x05

      /* Burst-Aussendungen fuer Steuerungszwecke */
      g_ax_spi_freq_chan[1] = spi_ax_calcFrequency_Mhz2Regs(144.9250);          // VCO2 (internal with    ext. L) with RFDIV --> VCORB = 0x05

    #else
      #error "A FREQA / FREQB pair has to be set."
    #endif

    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* Auto ranging and storing */
    spi_ax_doRanging();

    #ifndef AX_RUN_VCO2_APRS_TX
      #if 0
        while (1U) { __asm volatile( "nop" ); }
      #endif
    #endif
  }

  /* TEST BOX */
  s_spi_test_start_testBox();
  #endif
#endif
}

void task_spi_ax(void)
{
#if 0
  /* When doService is flagged by the ISR do read the FIFO */
  if (g_ax_spi_rx_fifo_doService) {
    uint8_t bufDbg[C_SPI_AX_BUFFER_LENGTH];
    uint8_t buf[C_SPI_AX_BUFFER_LENGTH];
    uint8_t lenDbg = 0;
    uint8_t len = 0;

    /* IRQ disabled section */
    {
      irqflags_t flags = cpu_irq_save();

      #if 0
      lenDbg += doHexdump((char*)(bufDbg + lenDbg), g_ax_spi_rx_buffer, g_ax_spi_rx_buffer_idx);
      #endif

      /* Copy chunk to decoder buffer */
      len = g_ax_spi_rx_buffer_idx;
      memcpy(buf, g_ax_spi_rx_buffer, len);

      /* Free ISR buffer */
      g_ax_spi_rx_buffer_idx = 0;
      memset(g_ax_spi_rx_buffer, 0, sizeof(g_ax_spi_rx_buffer));

      /* Reset doService flag */
      g_ax_spi_rx_fifo_doService = 0U;

      cpu_irq_restore(flags);
    }

    #if 1
    udi_write_tx_buf((char*)bufDbg, lenDbg, 0U);
    #endif

    /* Decode the POCSAG data */
    spi_ax_doProcess_RX_messages(buf, len);
  }
#endif
}


/* Debugging */

#ifdef AX_TEST

static void spi_test_start_testBox(void)
{
  //  AX_TEST_ANALOG_FM_TX
  #if defined(AX_TEST_VCO2_ANALOG_FM_TX)
  spi_ax_test_Analog_FM_Tx();

  // AX_TEST_ANALOG_FM_RX
  #elif defined(AX_TEST_VCO2_ANALOG_FM_RX)
  spi_ax_test_Analog_FM_Rx();


  // AX_TEST_PR1200_TX
  #elif defined(AX_TEST_VCO2_PR1200_TX)
  spi_ax_test_PR1200_Tx();

  // AX_TEST_PR1200_RX
  #elif defined(AX_TEST_VCO2_PR1200_RX)
  spi_ax_test_PR1200_Rx();


  // AX_TEST_POCSAG_TX
  #elif defined(AX_TEST_VCO1_POCSAG_TX)
  spi_ax_test_POCSAG_Tx();

  // AX_TEST_POCSAG_RX
  #elif defined(AX_TEST_VCO1_POCSAG_RX)
  spi_ax_test_POCSAG_Rx();
  #endif
}

void spi_ax_test_monitor_levels(void)
{
  volatile uint8_t curRssi = 0;
  volatile uint8_t curBgndRssi = 0;
  volatile uint8_t curAgcCounter = 0;
  volatile uint16_t curTrkAmpl = 0;
  volatile uint32_t curTrkRfFreq = 0;
  volatile uint16_t curTrkFreq = 0;

  while (1U) {
    /* RSSI, BGNDRSSI */
    spi_ax_transport(0U, "< 40 R2 >");                       // RD Address 0x40: RSSI, BGNDRSSI
    curRssi     = g_ax_spi_packet_buffer[0];
    curBgndRssi   = g_ax_spi_packet_buffer[1];

    /* AGCCOUNTER */
    spi_ax_transport(0U, "< 43 R1 >");                       // RD Address 0x43: AGCCOUNTER
    curAgcCounter = g_ax_spi_packet_buffer[0];

    /* TRKAMPL */
    spi_ax_transport(0U, "< 48 R2 >");                       // RD Address 0x48: TRKAMPL
    curTrkAmpl    = ((uint16_t)g_ax_spi_packet_buffer[0] << 8) | g_ax_spi_packet_buffer[1];

    /* TRKRFFREQ */
    spi_ax_transport(0U, "< 4D R3 >");                       // RD Address 0x4D: TRKRFFREQ
    curTrkRfFreq  = (((uint32_t)g_ax_spi_packet_buffer[0] & 0x0f) << 24) | ((uint32_t)g_ax_spi_packet_buffer[1] << 8) | g_ax_spi_packet_buffer[2];

    /* TRKFREQ */
    spi_ax_transport(0U, "< 50 R2 >");                       // RD Address 0x50: TRKFREQ
    curTrkFreq    = ((uint16_t)g_ax_spi_packet_buffer[0] << 8) | g_ax_spi_packet_buffer[1];

    /* View debugger auto variables*/
    __asm volatile( "nop" );

    (void) curRssi;
    (void) curBgndRssi;
    (void) curAgcCounter;
    (void) curTrkAmpl;
    (void) curTrkRfFreq;
    (void) curTrkFreq;
  }
}

void spi_ax_test_Rx_FIFO(void)
{
  uint8_t       l_curRssi           = 0;
  uint8_t       l_curBgndRssi         = 0;
  uint8_t       l_agcCounter          = 0;
  uint8_t       l_fifo_stat           = 0;
  uint16_t      l_fifo_count          = 0;
  AX_FIFO_RX_FSM_t  l_fifo_state          = AX_FIFO_RX_FSM__START;
  AX_FIFO_DATA_CMD_t  l_fifo_cmd            = AX_FIFO_DATA_CMD_NOP_TX;
  uint32_t      l_fifo_lastPacket_time      = 0UL;
  uint8_t       l_fifo_lastPacket_rssi      = 0U;
  uint8_t       l_fifo_lastPacket_rssi_ant2   = 0U;
  uint8_t       l_fifo_lastPacket_bgndnoise   = 0U;
  uint32_t      l_fifo_lastPacket_frfreqoffset  = 0UL;
  uint16_t      l_fifo_lastPacket_freqoffset  = 0U;
  uint32_t      l_fifo_lastPacket_datarate    = 0UL;
  uint8_t       l_fifo_lastPacket_dataLen   = 0U;
  uint8_t       l_fifo_lastPacket_dataStatus  = 0U;
  uint8_t       l_fifo_lastPacket_dataMsg[C_SPI_AX_BUFFER_LENGTH];

  /* Receive loop */
  do {
    /* Wait for a packet */
    do {
      /* RSSI, BGNDRSSI */
      spi_ax_transport(0U, "< 40 R4 >");                       // RD Address 0x40: RSSI, BGNDRSSI
      l_curRssi     = g_ax_spi_packet_buffer[0];
      l_curBgndRssi   = g_ax_spi_packet_buffer[1];
      l_agcCounter    = g_ax_spi_packet_buffer[3];

      #if 1
      /* IRQREQUEST0 */
      spi_ax_transport(0U, "< 0d R1 >");                       // RD address 0x0d: IRQREQUEST0

      #else
      /* FIFOSTAT - does not work in WoR mode */
      spi_ax_transport(0U, "< 28 R1 >");                       // RD address 0x28: FIFOSTAT
      #endif
    } while (!(g_ax_spi_packet_buffer[0] & 0x01));

    l_fifo_stat = g_ax_spi_packet_buffer[0];

    /* FIFOCOUNT */
    spi_ax_transport(0U, "< 2a R2 >");                         // RD address 0x28: FIFOCOUNT
    l_fifo_count = ((uint16_t)(g_ax_spi_packet_buffer[0]) << 8) | g_ax_spi_packet_buffer[1];

    /* Reset FSM to START state */
    l_fifo_state = AX_FIFO_RX_FSM__START;

    /* Pull messages from the FIFO */
    for (uint16_t idx = 0; idx < l_fifo_count; idx++) {
      /* FIFODATA */
      spi_ax_transport(0U, "< 29 R1 >");                       // RD address 0x29: FIFODATA

      switch (l_fifo_state) {
        case AX_FIFO_RX_FSM__START:
        {
          l_fifo_cmd = (AX_FIFO_DATA_CMD_t)g_ax_spi_packet_buffer[0];
          switch (l_fifo_cmd) {
            case AX_FIFO_DATA_CMD_TIMER_RX:
            l_fifo_state = AX_FIFO_RX_FSM_TIMER_1;
            break;

            case AX_FIFO_DATA_CMD_RSSI_RX:
            l_fifo_state = AX_FIFO_RX_FSM_RSSI_1;
            break;

            case AX_FIFO_DATA_CMD_ANTRSSI2_RX:
            l_fifo_state = AX_FIFO_RX_FSM_ANTRSSI2_1;
            break;

            case AX_FIFO_DATA_CMD_ANTRSSI3_RX:
            l_fifo_state = AX_FIFO_RX_FSM_ANTRSSI3_1;
            break;

            case AX_FIFO_DATA_CMD_RFFREQOFFS_RX:
            l_fifo_state = AX_FIFO_RX_FSM_RFFREQOFFS_1;
            break;

            case AX_FIFO_DATA_CMD_FREQOFFS_RX:
            l_fifo_state = AX_FIFO_RX_FSM_FREQOFFS_1;
            break;

            case AX_FIFO_DATA_CMD_DATARATE_RX:
            l_fifo_state = AX_FIFO_RX_FSM_DATARATE_1;
            break;

            case AX_FIFO_DATA_CMD_DATA_TX_RX:
            l_fifo_state = AX_FIFO_RX_FSM_DATA_1;
            break;

            default:
            {
              l_fifo_state = AX_FIFO_RX_FSM__FAIL_CMD;
            }
          }
        }
        break;


        case AX_FIFO_RX_FSM_TIMER_1:
        l_fifo_lastPacket_time  = (uint32_t)g_ax_spi_packet_buffer[0] << 16;
        l_fifo_state = AX_FIFO_RX_FSM_TIMER_2;
        break;

        case AX_FIFO_RX_FSM_TIMER_2:
        l_fifo_lastPacket_time |= (uint32_t)g_ax_spi_packet_buffer[0] <<  8;
        l_fifo_state = AX_FIFO_RX_FSM_TIMER_3;
        break;

        case AX_FIFO_RX_FSM_TIMER_3:
        l_fifo_lastPacket_time |= (uint32_t)g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_RSSI_1:
        l_fifo_lastPacket_rssi = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_ANTRSSI2_1:
        l_fifo_lastPacket_rssi = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM_ANTRSSI2_2;
        break;

        case AX_FIFO_RX_FSM_ANTRSSI2_2:
        l_fifo_lastPacket_bgndnoise = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_ANTRSSI3_1:
        l_fifo_lastPacket_rssi = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM_ANTRSSI3_2;
        break;

        case AX_FIFO_RX_FSM_ANTRSSI3_2:
        l_fifo_lastPacket_rssi_ant2 = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM_ANTRSSI3_3;
        break;

        case AX_FIFO_RX_FSM_ANTRSSI3_3:
        l_fifo_lastPacket_bgndnoise = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_RFFREQOFFS_1:
        l_fifo_lastPacket_frfreqoffset  = (uint32_t)g_ax_spi_packet_buffer[0] << 16;
        l_fifo_state = AX_FIFO_RX_FSM_RFFREQOFFS_2;
        break;

        case AX_FIFO_RX_FSM_RFFREQOFFS_2:
        l_fifo_lastPacket_frfreqoffset |= (uint32_t)g_ax_spi_packet_buffer[0] <<  8;
        l_fifo_state = AX_FIFO_RX_FSM_RFFREQOFFS_3;
        break;

        case AX_FIFO_RX_FSM_RFFREQOFFS_3:
        l_fifo_lastPacket_frfreqoffset |= (uint32_t)g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_FREQOFFS_1:
        l_fifo_lastPacket_freqoffset  = (uint32_t)g_ax_spi_packet_buffer[0] <<  8;
        l_fifo_state = AX_FIFO_RX_FSM_FREQOFFS_2;
        break;

        case AX_FIFO_RX_FSM_FREQOFFS_2:
        l_fifo_lastPacket_freqoffset |= (uint32_t)g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_DATARATE_1:
        l_fifo_lastPacket_datarate  = (uint32_t)g_ax_spi_packet_buffer[0] << 16;
        l_fifo_state = AX_FIFO_RX_FSM_DATARATE_2;
        break;

        case AX_FIFO_RX_FSM_DATARATE_2:
        l_fifo_lastPacket_datarate |= (uint32_t)g_ax_spi_packet_buffer[0] <<  8;
        l_fifo_state = AX_FIFO_RX_FSM_DATARATE_3;
        break;

        case AX_FIFO_RX_FSM_DATARATE_3:
        l_fifo_lastPacket_datarate |= (uint32_t)g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;


        case AX_FIFO_RX_FSM_DATA_1:
        l_fifo_lastPacket_dataLen = g_ax_spi_packet_buffer[0];
        l_fifo_state = AX_FIFO_RX_FSM_DATA_2;
        break;

        case AX_FIFO_RX_FSM_DATA_2:
        l_fifo_lastPacket_dataStatus = g_ax_spi_packet_buffer[0];
        l_fifo_lastPacket_dataLen--;
        l_fifo_state = AX_FIFO_RX_FSM_DATA_3;
        break;

        case AX_FIFO_RX_FSM_DATA_3:
        g_ax_spi_packet_buffer[0] = 0x29;                     // RD address 0x29: FIFODATA

        spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
        spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, 1);
        spi_read_packet(&SPI_AX, g_ax_spi_packet_buffer, l_fifo_lastPacket_dataLen);
        spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

        memcpy(l_fifo_lastPacket_dataMsg, g_ax_spi_packet_buffer, l_fifo_lastPacket_dataLen);
        memset(l_fifo_lastPacket_dataMsg + l_fifo_lastPacket_dataLen, 0, sizeof(l_fifo_lastPacket_dataMsg) - l_fifo_lastPacket_dataLen);

        idx += l_fifo_lastPacket_dataLen;
        l_fifo_state = AX_FIFO_RX_FSM__START;
        break;

        default:
        {
          l_fifo_state = AX_FIFO_RX_FSM__FAIL_STATE;
        }
      }
    }


    (void) l_fifo_stat;

    (void) l_curRssi;
    (void) l_curBgndRssi;
    (void) l_agcCounter;

    (void) l_fifo_lastPacket_time;
    (void) l_fifo_lastPacket_rssi;
    (void) l_fifo_lastPacket_rssi_ant2;
    (void) l_fifo_lastPacket_bgndnoise;
    (void) l_fifo_lastPacket_frfreqoffset;
    (void) l_fifo_lastPacket_freqoffset;
    (void) l_fifo_lastPacket_datarate;
    (void) l_fifo_lastPacket_dataLen;
    (void) l_fifo_lastPacket_dataStatus;
    (void) l_fifo_lastPacket_dataMsg;

    __asm volatile( "nop" );
  } while (1U);
}


void spi_ax_test_Analog_FM_Tx(void)
{
  uint8_t toggle = 0U;

  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_ANALOG_FM, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* Recall ranging values */
    spi_ax_doRanging();

    #if 0
    /* Set VCO-PLL to FREQB */
    (void) spi_ax_selectVcoFreq(1U);
    #else
    /* Set VCO-PLL to FREQA */
    (void) spi_ax_selectVcoFreq(0U);
    #endif
  }

  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLTX);

  /* Set power level */
  spi_ax_setPower_dBm(-20);


  for (uint16_t cnt = 0x2000; cnt; cnt--) {
    (void) spi_ax_selectVcoFreq(toggle);                          // Frequency is alternated  802x per second with the Debug code - 2106x per second with the Release code!
    toggle = !toggle;
  }

  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);
  __asm volatile( "nop" );
}

void spi_ax_test_Analog_FM_Rx(void)
{
  /* Syncing and sending reset command, then setting the default values */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_ANALOG_FM, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    /* FREQA <-- chan[0], FREQB <-- chan[1] */
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* Recall ranging values */
    spi_ax_doRanging();

    #if 0
    /* Set VCO-PLL to FREQB */
    (void) spi_ax_selectVcoFreq(1U);
    #else
    /* Set VCO-PLL to FREQA */
    (void) spi_ax_selectVcoFreq(0U);
    #endif
  }

  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLRX);

  /* Monitor loop inside */
  spi_ax_test_monitor_levels();
}


/* PR1200 TX */
void spi_ax_test_PR1200_Tx(void)
{ /* TEST: transmitting some packet */
  //uint8_t cmd = 0x59;
  //volatile uint8_t tmr01[3];
  //volatile uint8_t tmr02[3];

  spi_ax_init_PR1200_Tx();


  /* TIMER */
  /*
  // Convenient operation
  spi_ax_transport(0U, "< 59 R3 >");
  memcpy(tmr01, g_ax_spi_packet_buffer, 3);

  // Manual operation
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, &cmd, 1);                             // Read TIMER data
  spi_read_packet(&SPI_AX, tmr01, 3);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  // Direct enable line handling
  s_spi_ax_select_device();                                   // clear PORT_C4
  spi_write_packet(&SPI_AX, &cmd, 1);                             // Read TIMER data
  spi_read_packet(&SPI_AX, tmr01, 3);
  s_spi_ax_deselect_device();                                 // set   PORT_C4


  spi_ax_transport(0U, "< 59 R3 >");
  memcpy(tmr02, g_ax_spi_packet_buffer, 3);
  // Debug-code: 176 탎

  // Manual operation
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, &cmd, 1);                             // Read TIMER data
  spi_read_packet(&SPI_AX, tmr02, 3);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);
  // Debug-code: 17 탎

  / Direct enable line handling
  s_spi_ax_select_device();                                   // clear PORT_C4
  spi_write_packet(&SPI_AX, &cmd, 1);                             // Read TIMER data
  spi_read_packet(&SPI_AX, tmr02, 3);
  s_spi_ax_deselect_device();                                 // set   PORT_C4
  // Debug-code: 8 탎
  */

  char addrAry[][6] = { "APXFMS", "DF4IAH", "WIDE1", "WIDE2" };
  uint8_t ssidAry[] = { 0, 8, 1, 2 };
  char aprsMsg[]    = "!4928.39N/00836.88Ej OP: Uli, QTH: Ladenburg, LOC: JN49hl.";

  for (uint16_t count = 10; count; count--) {
    do {
      /* FIFOSTAT */
      spi_ax_transport(0U, "< 28 R1 >");
    } while (!(g_ax_spi_packet_buffer[0] & 0x01));

    #if 0
    /* Enter minimal AX.25 Level-2 packet */
    spi_ax_test_PR1200_Tx_FIFO_Lev2_minimal();
    osDelay(750);

    #else
    /* Enter an APRS UI frame */
    spi_ax_run_PR1200_Tx_FIFO_APRS(addrAry, ssidAry, 4, aprsMsg, strlen(aprsMsg));
    osDelay(1000);
    //osDelay(7500);
    #endif

  }

  /* FIFOCMD / FIFOSTAT */
  spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  do {
    /* RADIOSTATE */
    spi_ax_transport(0U, "< 1c R1 >");                         // RD Address 0x1C: RADIOSTATE - IDLE
  } while ((g_ax_spi_packet_buffer[0] & 0x0f) != 0);

  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  while (1U) {
    __asm volatile( "nop" );
  }
}

void spi_ax_test_PR1200_Tx_FIFO_Lev2_minimal_AddressField(void)
{
  uint16_t idx = 0;

  g_ax_spi_packet_buffer[idx++] = 0xA9;                         // WR address 0x29: FIFODATA  (SPI AX address keeps constant)
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_CMD_DATA_TX_RX;
  g_ax_spi_packet_buffer[idx++] = 0;                            // Dummy entry for now
  g_ax_spi_packet_buffer[idx++] = AX_FIFO_DATA_FLAGS_TX_PKTSTART | AX_FIFO_DATA_FLAGS_TX_PKTEND;
  g_ax_spi_packet_buffer[idx++] = ('D' << 1)  | 0;                    // Address: dest.       [A 1]
  g_ax_spi_packet_buffer[idx++] = ('F' << 1)  | 0;                    // Address: dest.       [A 2]
  g_ax_spi_packet_buffer[idx++] = ('4' << 1)  | 0;                    // Address: dest.       [A 3]
  g_ax_spi_packet_buffer[idx++] = ('I' << 1)  | 0;                    // Address: dest.       [A 4]
  g_ax_spi_packet_buffer[idx++] = ('A' << 1)  | 0;                    // Address: dest.       [A 5]
  g_ax_spi_packet_buffer[idx++] = ('H' << 1)  | 0;                    // Address: dest.       [A 6]
  g_ax_spi_packet_buffer[idx++] = ((0b0 << 7) | (0b11 << 5) | (0x2 << 1)  | 0);     // Address: dest.  SSID [A 7]

  g_ax_spi_packet_buffer[idx++] = ('D' << 1)  | 0;                    // Address: source      [A 8]
  g_ax_spi_packet_buffer[idx++] = ('F' << 1)  | 0;                    // Address: source      [A 9]
  g_ax_spi_packet_buffer[idx++] = ('4' << 1)  | 0;                    // Address: source      [A10]
  g_ax_spi_packet_buffer[idx++] = ('I' << 1)  | 0;                    // Address: source      [A11]
  g_ax_spi_packet_buffer[idx++] = ('A' << 1)  | 0;                    // Address: source      [A12]
  g_ax_spi_packet_buffer[idx++] = ('H' << 1)  | 0;                    // Address: source      [A13]
  g_ax_spi_packet_buffer[idx++] = ((0b1 << 7) | (0b11 << 5) | (0x1 << 1)  | 1);     // Address: source SSID [A14]

  g_ax_spi_packet_buffer[idx++] = ((0b0 << 4) |  0b11);                 // Control: UI frame with no Poll bit set
  g_ax_spi_packet_buffer[idx++] =  0xf0;                          // PID: No layer 3 protocol implemented

  g_ax_spi_packet_buffer[idx++] =  ':';
  g_ax_spi_packet_buffer[idx++] =  '-';
  g_ax_spi_packet_buffer[idx++] =  ')';
  g_ax_spi_packet_buffer[idx++] =  '\r';

  /* Set length for FIFO DATA command */
  g_ax_spi_packet_buffer[    2] = idx - 3;                        // Length

  /* FIFO data enter */
  spi_select_device(&SPI_AX, &g_ax_spi_device_conf);
  spi_write_packet(&SPI_AX, g_ax_spi_packet_buffer, idx);
  spi_deselect_device(&SPI_AX, &g_ax_spi_device_conf);

  /* FIFO do a COMMIT */
  spi_ax_transport(0U, "< a8 04 >");                         // WR address 0x28: FIFOCMD - AX_FIFO_CMD_COMMIT
}

void spi_ax_test_PR1200_Tx_FIFO_Lev2_minimal()
{
  /* Enter an APRS UI frame */

  /* 1 - Flags */
  spi_ax_util_PR1200_Tx_FIFO_Flags(35);                         // Minimal

  /* 2 - Address field, Control and PID */
  spi_ax_test_PR1200_Tx_FIFO_Lev2_minimal_AddressField();
}


/* PR1200 RX */
void spi_ax_test_PR1200_Rx(void)
{
  /* Syncing and sending reset command, then setting the packet radio values for receiving */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_PR1200, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* Recall ranging values */
    spi_ax_doRanging();

    #if 0
    /* Set VCO-PLL to FREQB */
    (void) spi_ax_selectVcoFreq(1U);
    #else
    /* Set VCO-PLL to FREQA */
    (void) spi_ax_selectVcoFreq(0U);
    #endif
  }

  /* FIFOCMD / FIFOSTAT */
  spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  #if 0
  /* Enable the receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_RX_WOR, AX_SET_REGISTERS_POWERMODE_WOR);

  #else
  /* Enable the receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_RX_CONT, AX_SET_REGISTERS_POWERMODE_FULLRX);
  #endif

  /* Receive loop */
  spi_ax_test_Rx_FIFO();
}


/* POCSAG TX */
void spi_ax_test_POCSAG_Tx(void)
{ /* TEST: transmitting some packet */
  /* Syncing and sending reset command, then setting the packet radio values for transmission */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_POCSAG, AX_SET_REGISTERS_VARIANT_TX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* Recall ranging values */
    spi_ax_doRanging();

    /* Switch to VCO-B */
    (void) spi_ax_selectVcoFreq(1U);
  }

  /* Enabling the transmitter */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_FULLTX);

  /* Set power level */
  spi_ax_setPower_dBm(-20);


  /* FIFOCMD / FIFOSTAT */
  spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  for (uint16_t count = 3; count; count--) {
    do {
      /* FIFOSTAT */
      spi_ax_transport(0U, "< 28 R1 >");
    } while (!(g_ax_spi_packet_buffer[0] & 0x01));

    /* Enter POCSAG preamble */
    spi_ax_util_POCSAG_Tx_FIFO_Preamble();

    /* Enter POCSAG batches with message to destination RIC */
    int32_t targetRIC = 2030000UL;
    const char msgBuf[] = "DF4IAH: This is a demonstration message to my  RIC 2030000  using 80 characters.";
    (void) spi_ax_util_POCSAG_Tx_FIFO_Batches(targetRIC, AX_POCSAG_CW2_MODE3_ALPHANUM, msgBuf, strlen(msgBuf));

    osDelay(2000);
    //osDelay(7500);
  }

  /* FIFOCMD / FIFOSTAT */
  spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  do {
    /* RADIOSTATE */
    spi_ax_transport(0U, "< 1c R1 >");                         // RD Address 0x1C: RADIOSTATE - IDLE
  } while ((g_ax_spi_packet_buffer[0] & 0x0f) != 0);

  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_NO_CHANGE, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  while (1U) {
    __asm volatile( "nop" );
  }
}

/* POCSAG RX */
void spi_ax_test_POCSAG_Rx(void)
{
  /* Syncing and sending reset command, then setting the packet radio values for receiving */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_POCSAG, AX_SET_REGISTERS_VARIANT_RX, AX_SET_REGISTERS_POWERMODE_POWERDOWN);

  /* Frequency settings */
  {
    spi_ax_setFrequency2Regs(0, 0U);
    spi_ax_setFrequency2Regs(1, 1U);

    /* Recall ranging values */
    spi_ax_doRanging();

    /* Set VCO-PLL to FREQB */
    (void) spi_ax_selectVcoFreq(1U);
  }

  /* FIFOCMD / FIFOSTAT */
  spi_ax_transport(0U, "< a8 03 >");                           // WR address 0x28: FIFOCMD - AX_FIFO_CMD_CLEAR_FIFO_DATA_AND_FLAGS

  #if 0
  /* Enable the receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_RX_WOR, AX_SET_REGISTERS_POWERMODE_WOR);

  #else
  /* Enable the receiver */
  spi_ax_setRegisters(0U, AX_SET_REGISTERS_MODULATION_NO_CHANGE, AX_SET_REGISTERS_VARIANT_RX_CONT, AX_SET_REGISTERS_POWERMODE_FULLRX);
  #endif

  /* Receive loop */
  spi_ax_test_Rx_FIFO();
}
#endif


uint8_t spiDetectAx5243(void)
{
  /* Request RD-address 0x000 SILICONREV */
  {
    uint8_t axVersion = 0;

    spi3TxBuffer[0] = SPI_RD_FLAG | 0x00;
    if (HAL_OK == spiProcessSpi3Msg(SPI3_AX, 2)) {
      axVersion = spi3RxBuffer[1];
    }

    if (axVersion != 0x51) {                                                                    // AX5243
      /* We can handle Version  0x51 (AX5243) only */
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static void ax5243Init(void)
{

}


/* Tasks */

void ax5243TaskInit(void)
{
  osDelay(500UL);
  ax5243Init();
}

void ax5243TaskLoop(void)
{
  const uint32_t  eachMs              = 250UL;
  static uint32_t sf_previousWakeTime = 0UL;

  if (!sf_previousWakeTime) {
    sf_previousWakeTime  = osKernelSysTick();
    sf_previousWakeTime -= sf_previousWakeTime % 1000UL;
    sf_previousWakeTime += 500UL;
  }

  osDelayUntil(&sf_previousWakeTime, eachMs);
}

