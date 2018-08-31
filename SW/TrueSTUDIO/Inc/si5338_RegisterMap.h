//Register map for use with AN428 (JumpStart)
//http://www.silabs.com/clocks 
//Copyright 2012 Silicon Laboratories
//#BEGIN_HEADER
//Date = Friday, August 31, 2018 11:19 PM
//File version = 3
//Software Name = ClockBuilder Desktop
//Software version = 6.4
//Software date = October 8, 2014
//Chip = Si5338
//Part Number = Si5338
//#END_HEADER
//Input Frequency (MHz) = 20.000000000
//Input Type = LVDS_LVPECL_HCSL
//P1 = 1
//Input Mux = FbClk
//FDBK Input Frequency (MHz) = 20.000000000
//FDBK Input Type = LVDS_LVPECL_HCSL
//P2 = 1
//FDBK Mux = NoClk
//PFD Input Frequency (MHz) = 20.000000000
//VCO Frequency (GHz) = 2.480000
//N = 124  (124.0000)
//Internal feedback enabled
//Output Clock 0
// Output Frequency (MHz) = 20.000000000
// Mux Selection = FDBK
// R = 1
//Output Clock 1
// Output Frequency (MHz) = 26.666666667
// Mux Selection = IDn
// MultiSynth = 93  (93.0000)
// R = 1
//Output Clock 2
// Output Frequency (MHz) = 13.333333333
// Mux Selection = IDn
// MultiSynth = 186  (186.0000)
// R = 1
//Output Clock 3
// Output Frequency (MHz) = 10.000000000
// Mux Selection = IDn
// MultiSynth = 248  (248.0000)
// R = 1
//Driver 0
// Enabled
// Powered on
// Output voltage = 3.30
// Output type = 3.3V CMOS on A and B
// Output state when disabled = StopLow
//Driver 1
// Enabled
// Powered on
// Output voltage = 3.30
// Output type = 3.3V LVDS
// Output state when disabled = StopLow
//Driver 2
// Enabled
// Powered on
// Output voltage = 3.30
// Output type = 3.3V LVDS
// Output state when disabled = StopLow
//Driver 3
// Enabled
// Powered on
// Output voltage = 3.30
// Output type = 3.3V LVDS
// Output state when disabled = StopLow
//Clock 0 phase inc/dec step size (ns) = 0.000
//Clock 1 phase inc/dec step size (ns) = 0.000
//Clock 2 phase inc/dec step size (ns) = 0.000
//Clock 3 phase inc/dec step size (ns) = 0.000
//Phase increment and decrement pin control is off
//Frequency increment and decrement pin control is off
//Frequency increment and decrement is disabled
//Initial phase offset 0 (ns) = 0.000
//Initial phase offset 1 (ns) = 0.000
//Initial phase offset 2 (ns) = 0.000
//Initial phase offset 3 (ns) = 0.000
//SSC is disabled

#define SI5338_NUM_REGS_MAX 350

typedef struct Reg_Data{
   unsigned char Reg_Addr;
   unsigned char Reg_Val;
   unsigned char Reg_Mask;
} Reg_Data;

const Reg_Data si5338_Reg_Store[SI5338_NUM_REGS_MAX] = {
    {  0,0x00,0x00},
    {  1,0x00,0x00},
    {  2,0x00,0x00},
    {  3,0x00,0x00},
    {  4,0x00,0x00},
    {  5,0x00,0x00},
    {  6,0x04,0x1D},
    {  7,0x00,0x00},
    {  8,0x70,0x00},
    {  9,0x0F,0x00},
    { 10,0x00,0x00},
    { 11,0x00,0x00},
    { 12,0x00,0x00},
    { 13,0x00,0x00},
    { 14,0x00,0x00},
    { 15,0x00,0x00},
    { 16,0x00,0x00},
    { 17,0x00,0x00},
    { 18,0x00,0x00},
    { 19,0x00,0x00},
    { 20,0x00,0x00},
    { 21,0x00,0x00},
    { 22,0x00,0x00},
    { 23,0x00,0x00},
    { 24,0x00,0x00},
    { 25,0x00,0x00},
    { 26,0x00,0x00},
    { 27,0x70,0x80},
    { 28,0x03,0xFF},
    { 29,0x20,0xFF},
    { 30,0xA0,0xFF},
    { 31,0x02,0xFF},
    { 32,0xC0,0xFF},
    { 33,0xC0,0xFF},
    { 34,0xC0,0xFF},
    { 35,0x00,0xFF},
    { 36,0x03,0x1F},
    { 37,0x06,0x1F},
    { 38,0x06,0x1F},
    { 39,0x06,0x1F},
    { 40,0x77,0xFF},
    { 41,0x0C,0x7F},
    { 42,0x23,0x3F},
    { 43,0x00,0x00},
    { 44,0x00,0x00},
    { 45,0x00,0xFF},
    { 46,0x00,0xFF},
    { 47,0x14,0x3F},
    { 48,0x49,0xFF},
    { 49,0x00,0xFF},
    { 50,0xC4,0xFF},
    { 51,0x07,0xFF},
    { 52,0x10,0xFF},
    { 53,0x00,0xFF},
    { 54,0x3C,0xFF},
    { 55,0x00,0xFF},
    { 56,0x00,0xFF},
    { 57,0x00,0xFF},
    { 58,0x00,0xFF},
    { 59,0x01,0xFF},
    { 60,0x00,0xFF},
    { 61,0x00,0xFF},
    { 62,0x00,0x3F},
    { 63,0x10,0xFF},
    { 64,0x80,0xFF},
    { 65,0x2C,0xFF},
    { 66,0x00,0xFF},
    { 67,0x00,0xFF},
    { 68,0x00,0xFF},
    { 69,0x00,0xFF},
    { 70,0x01,0xFF},
    { 71,0x00,0xFF},
    { 72,0x00,0xFF},
    { 73,0x00,0x3F},
    { 74,0x10,0xFF},
    { 75,0x00,0xFF},
    { 76,0x5B,0xFF},
    { 77,0x00,0xFF},
    { 78,0x00,0xFF},
    { 79,0x00,0xFF},
    { 80,0x00,0xFF},
    { 81,0x01,0xFF},
    { 82,0x00,0xFF},
    { 83,0x00,0xFF},
    { 84,0x00,0x3F},
    { 85,0x10,0xFF},
    { 86,0x00,0xFF},
    { 87,0x7A,0xFF},
    { 88,0x00,0xFF},
    { 89,0x00,0xFF},
    { 90,0x00,0xFF},
    { 91,0x00,0xFF},
    { 92,0x01,0xFF},
    { 93,0x00,0xFF},
    { 94,0x00,0xFF},
    { 95,0x00,0x3F},
    { 96,0x10,0x00},
    { 97,0x00,0xFF},
    { 98,0x3C,0xFF},
    { 99,0x00,0xFF},
    {100,0x00,0xFF},
    {101,0x00,0xFF},
    {102,0x00,0xFF},
    {103,0x01,0xFF},
    {104,0x00,0xFF},
    {105,0x00,0xFF},
    {106,0x80,0xBF},
    {107,0x00,0xFF},
    {108,0x00,0xFF},
    {109,0x00,0xFF},
    {110,0x40,0xFF},
    {111,0x00,0xFF},
    {112,0x00,0xFF},
    {113,0x00,0xFF},
    {114,0x40,0xFF},
    {115,0x00,0xFF},
    {116,0x80,0xFF},
    {117,0x00,0xFF},
    {118,0x40,0xFF},
    {119,0x00,0xFF},
    {120,0x00,0xFF},
    {121,0x00,0xFF},
    {122,0x40,0xFF},
    {123,0x00,0xFF},
    {124,0x00,0xFF},
    {125,0x00,0xFF},
    {126,0x00,0xFF},
    {127,0x00,0xFF},
    {128,0x00,0xFF},
    {129,0x00,0x0F},
    {130,0x00,0x0F},
    {131,0x00,0xFF},
    {132,0x00,0xFF},
    {133,0x00,0xFF},
    {134,0x00,0xFF},
    {135,0x00,0xFF},
    {136,0x00,0xFF},
    {137,0x00,0xFF},
    {138,0x00,0xFF},
    {139,0x00,0xFF},
    {140,0x00,0xFF},
    {141,0x00,0xFF},
    {142,0x00,0xFF},
    {143,0x00,0xFF},
    {144,0x00,0xFF},
    {145,0x00,0x00},
    {146,0xFF,0x00},
    {147,0x00,0x00},
    {148,0x00,0x00},
    {149,0x00,0x00},
    {150,0x00,0x00},
    {151,0x00,0x00},
    {152,0x00,0xFF},
    {153,0x00,0xFF},
    {154,0x00,0xFF},
    {155,0x00,0xFF},
    {156,0x00,0xFF},
    {157,0x00,0xFF},
    {158,0x00,0x0F},
    {159,0x00,0x0F},
    {160,0x00,0xFF},
    {161,0x00,0xFF},
    {162,0x00,0xFF},
    {163,0x00,0xFF},
    {164,0x00,0xFF},
    {165,0x00,0xFF},
    {166,0x00,0xFF},
    {167,0x00,0xFF},
    {168,0x00,0xFF},
    {169,0x00,0xFF},
    {170,0x00,0xFF},
    {171,0x00,0xFF},
    {172,0x00,0xFF},
    {173,0x00,0xFF},
    {174,0x00,0xFF},
    {175,0x00,0xFF},
    {176,0x00,0xFF},
    {177,0x00,0xFF},
    {178,0x00,0xFF},
    {179,0x00,0xFF},
    {180,0x00,0xFF},
    {181,0x00,0x0F},
    {182,0x00,0xFF},
    {183,0x00,0xFF},
    {184,0x00,0xFF},
    {185,0x00,0xFF},
    {186,0x00,0xFF},
    {187,0x00,0xFF},
    {188,0x00,0xFF},
    {189,0x00,0xFF},
    {190,0x00,0xFF},
    {191,0x00,0xFF},
    {192,0x00,0xFF},
    {193,0x00,0xFF},
    {194,0x00,0xFF},
    {195,0x00,0xFF},
    {196,0x00,0xFF},
    {197,0x00,0xFF},
    {198,0x00,0xFF},
    {199,0x00,0xFF},
    {200,0x00,0xFF},
    {201,0x00,0xFF},
    {202,0x00,0xFF},
    {203,0x00,0x0F},
    {204,0x00,0xFF},
    {205,0x00,0xFF},
    {206,0x00,0xFF},
    {207,0x00,0xFF},
    {208,0x00,0xFF},
    {209,0x00,0xFF},
    {210,0x00,0xFF},
    {211,0x00,0xFF},
    {212,0x00,0xFF},
    {213,0x00,0xFF},
    {214,0x00,0xFF},
    {215,0x00,0xFF},
    {216,0x00,0xFF},
    {217,0x00,0xFF},
    {218,0x00,0x00},
    {219,0x00,0x00},
    {220,0x00,0x00},
    {221,0x0D,0x00},
    {222,0x00,0x00},
    {223,0x00,0x00},
    {224,0xF4,0x00},
    {225,0xF0,0x00},
    {226,0x00,0x00},
    {227,0x00,0x00},
    {228,0x00,0x00},
    {229,0x00,0x00},
    {231,0x00,0x00},
    {232,0x00,0x00},
    {233,0x00,0x00},
    {234,0x00,0x00},
    {235,0x00,0x00},
    {236,0x00,0x00},
    {237,0x00,0x00},
    {238,0x14,0x00},
    {239,0x00,0x00},
    {240,0x00,0x00},
    {242,0x00,0x02},
    {243,0xF0,0x00},
    {244,0x00,0x00},
    {245,0x00,0x00},
    {247,0x00,0x00},
    {248,0x00,0x00},
    {249,0xA8,0x00},
    {250,0x00,0x00},
    {251,0x84,0x00},
    {252,0x00,0x00},
    {253,0x00,0x00},
    {254,0x00,0x00},
    {255, 1, 0xFF}, // set page bit to 1
    {  0,0x00,0x00},
    {  1,0x00,0x00},
    {  2,0x00,0x00},
    {  3,0x00,0x00},
    {  4,0x00,0x00},
    {  5,0x00,0x00},
    {  6,0x00,0x00},
    {  7,0x00,0x00},
    {  8,0x00,0x00},
    {  9,0x00,0x00},
    { 10,0x00,0x00},
    { 11,0x00,0x00},
    { 12,0x00,0x00},
    { 13,0x00,0x00},
    { 14,0x00,0x00},
    { 15,0x00,0x00},
    { 16,0x00,0x00},
    { 17,0x01,0x00},
    { 18,0x00,0x00},
    { 19,0x00,0x00},
    { 20,0x90,0x00},
    { 21,0x31,0x00},
    { 22,0x00,0x00},
    { 23,0x00,0x00},
    { 24,0x01,0x00},
    { 25,0x00,0x00},
    { 26,0x00,0x00},
    { 27,0x00,0x00},
    { 28,0x00,0x00},
    { 29,0x00,0x00},
    { 30,0x00,0x00},
    { 31,0x00,0xFF},
    { 32,0x00,0xFF},
    { 33,0x01,0xFF},
    { 34,0x00,0xFF},
    { 35,0x00,0xFF},
    { 36,0x90,0xFF},
    { 37,0x31,0xFF},
    { 38,0x00,0xFF},
    { 39,0x00,0xFF},
    { 40,0x01,0xFF},
    { 41,0x00,0xFF},
    { 42,0x00,0xFF},
    { 43,0x00,0x0F},
    { 44,0x00,0x00},
    { 45,0x00,0x00},
    { 46,0x00,0x00},
    { 47,0x00,0xFF},
    { 48,0x00,0xFF},
    { 49,0x01,0xFF},
    { 50,0x00,0xFF},
    { 51,0x00,0xFF},
    { 52,0x90,0xFF},
    { 53,0x31,0xFF},
    { 54,0x00,0xFF},
    { 55,0x00,0xFF},
    { 56,0x01,0xFF},
    { 57,0x00,0xFF},
    { 58,0x00,0xFF},
    { 59,0x00,0x0F},
    { 60,0x00,0x00},
    { 61,0x00,0x00},
    { 62,0x00,0x00},
    { 63,0x00,0xFF},
    { 64,0x00,0xFF},
    { 65,0x01,0xFF},
    { 66,0x00,0xFF},
    { 67,0x00,0xFF},
    { 68,0x90,0xFF},
    { 69,0x31,0xFF},
    { 70,0x00,0xFF},
    { 71,0x00,0xFF},
    { 72,0x01,0xFF},
    { 73,0x00,0xFF},
    { 74,0x00,0xFF},
    { 75,0x00,0x0F},
    { 76,0x00,0x00},
    { 77,0x00,0x00},
    { 78,0x00,0x00},
    { 79,0x00,0xFF},
    { 80,0x00,0xFF},
    { 81,0x00,0xFF},
    { 82,0x00,0xFF},
    { 83,0x00,0xFF},
    { 84,0x90,0xFF},
    { 85,0x31,0xFF},
    { 86,0x00,0xFF},
    { 87,0x00,0xFF},
    { 88,0x01,0xFF},
    { 89,0x00,0xFF},
    { 90,0x00,0xFF},
    { 91,0x00,0x0F},
    { 92,0x00,0x00},
    { 93,0x00,0x00},
    { 94,0x00,0x00},
    {255, 0, 0xFF} }; // set page bit to 0
//End of file
