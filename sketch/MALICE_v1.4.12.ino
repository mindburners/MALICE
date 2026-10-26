// ============================================================
// MINDBURNER MALICE – v1.4
// 6-voice polyphonic / monophonic synthesiser + 303-style step sequencer
// MALICE PCB: ESP32 DevKit + PCM5102A, 4 buttons + rotary encoder
// ESP32 Arduino core 2.x (legacy I2S driver)
// Alan G Watkins 2026
//
// TFT:   ST7735S 1.8" 160x128  SCK=18 MOSI=23 CS=15 DC=2 RST=4 BL=26
// MIDI:  GPIO16 (Serial2 RX)
// BTNs:  A=5  B=12  C=13  D=19
// ENC:   A=14  B=21  (internal pull-ups)   Push switch = 32
// I2S:   BCK=27  LCK=25  DIN=22
//
// Voice (x6): osc1 + osc2 (SAW / TRI / SQR / PULSE + 4 mixes, band-limited),
//             osc2 tune/fine, osc mix, square sub (level), analog drift,
//             pulse width + PWM (own triangle LFO), noise (white / pink /
//             brown / digital, colour, burst decay, oscillator mute)
//             -> drive (saturation) -> resonant TPT SVF low-pass, 12/24 dB
//             -> ADSR driving filter + amp -> mixed
// LFO (global): sine/tri/saw/square/S&H, 0.05-20 Hz, to pitch/filter/amp,
//             free-running or key-synced
// FX (stereo): Juno-style chorus I / II / I+II, then delay
//             CLEAN / TAPE (darkening, saturating, wow) / PING (ping-pong)
//             -> soft clip -> stereo out
// Sequencer:  16 steps x 8 patterns, note / accent / slide per step, length 1-16,
//             keyboard recording, MIDI clock / Start / Stop, saved automatically
//
// PARAMETER PAGES (OSC / PULSE / NOISE / FILTER / ENV / LFO / MOD / FX / VOICE):
//   A tap = next parameter            A hold = next page (repeats)
//   B = value up   C = value down (hold to repeat/accelerate)
//   B+C hold (or D hold) = PATCH page
//   A+B+C hold 2 s = RESTORE FACTORY: B = selected slot, A = all slots, C = cancel
//   A+B hold 1 s = SEQUENCER screen on / off
//   Encoder turn = change value (fast turns accelerate)
//   Encoder push = next parameter     push hold = next page
//   D tap = next page
// PATCH PAGE (30 slots):
//   A tap/hold or encoder = choose slot   B or push = LOAD…   C = SAVE…
//   confirm box: B or push = yes, C = no  |  B+C hold (or D tap) = back
// SEQUENCER SCREEN:
//   A tap / encoder push = next step or box (BPM PAT LEN REC CLR)   A hold = back
//   On a step:  encoder = note   B tap = note on / rest   B hold or D = accent / slide
//               C = play / stop
//   BPM / PAT / LEN: encoder or B/C   REC: B = arm, C = play   CLR: B = clear pattern
//
// v1.4:   POLI LP / POLI BP filter types (Polivoks-style overdriven, saturating filter).
//         New factory sounds 12 Screamer Lead and 20 Soviet Bass. A+B+C = restore factory.
//         30 patch slots (21-30: ten new factory sounds). FX page: Cho Rate (AUTO or 0.05-8 Hz).
//         MOD page: 2-slot mod matrix (WHEEL VELOCITY AFTERTOUCH LFO ENV KEY -> CUTOFF PEAK PITCH
//         OSC2 PITCH PULSE W OSC MIX LFO RATE VOLUME DLY MIX CHO DEPTH X-MOD). Channel aftertouch received.
//         Wheel Vib: AUTO (no vibrato when the wheel is routed in the matrix) / ON / OFF.
//         PW/XMOD page: X-Mod (osc 2 frequency-modulates osc 1, JX-8P style) and Sync (osc 1 to osc 2).
// v1.3:   Step sequencer (accent, slide, 8 patterns, recording, MIDI clock), CLK indicator.
// v1.2:   Noise types (WHITE / PINK / BROWN / DIGITAL). A hold = next page
//         again (works on 3-button boards). Saved patches kept.
// v1.1:   NOISE page, 20 patches / 20 factory sounds, engine ~2x faster.
// v1.0:   Rotary encoder and button D.
// v0.9:   FX page: stereo chorus and delay (CLEAN / TAPE / PING).
// v0.8:   SAW/TRI/SQR/PULSE waves (+ mixes), PULSE page with PWM.
// v0.7:   LFO page, patch slots, load/save confirmation.
// v0.6:   Parameter pages, osc2 controls, drive, 12/24 dB, drift.
// v0.5:   6-voice polyphony, per-patch MONO/POLY (CC126/127).
//
// CC map:  1 Mod wheel (vibrato)  5 Glide   7 Volume
//          70 Osc1 wave  80 Osc2 wave  81 Osc2 tune  76 Osc2 fine
//          82 Osc mix    78 Sub level  83 Drift
//          104 Pulse width  105 PWM depth  106 PWM rate  21 X-Mod  20 Sync
//          111 Noise level  112 Noise colour  113 Noise decay  114 Osc mute
//          115 Noise type
//          74 Cutoff  71 Resonance  77 Env amt  85 Drive  86 Slope
//          73 Attack  75 Decay  79 Sustain  72 Release
//          87 LFO wave  88 LFO rate  89 LFO>pitch  90 LFO>filter
//          102 LFO>amp  103 LFO sync
//          93 Chorus  94 Chorus rate  107 Delay type  108 Delay time  109 Feedback  110 Delay mix
//          120/123 All notes off   126 Mono mode   127 Poly mode
//          Program Change 0-29 = patch 1-30
//          MIDI Clock / Start / Stop = sequencer
// ============================================================

#pragma GCC optimize ("O3")              // compile this sketch for speed, not size
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <math.h>

// ============================================================
//  Hardware
// ============================================================
#define SCREEN_W  160
#define SCREEN_H  128

#define TFT_SCK   18
#define TFT_MOSI  23
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST    4
#define BL_PIN    26

#define MIDI_RX_PIN  16
#define MIDI_BAUD    31250

#define BTN_A_PIN   5
#define BTN_B_PIN  12
#define BTN_C_PIN  13
#define BTN_D_PIN  19    // default VSPI MISO – SPI is started without MISO

#define ENC_A_PIN     14
#define ENC_B_PIN     21
#define ENC_BTN_PIN   32
#define ENC_REVERSE   false    // set true if the encoder turns the wrong way
#define ENC_STEPS_PER_DETENT 4 // 4 for most EC11 encoders; 2 if each click moves two values

#define I2S_BCK_PIN   27
#define I2S_WS_PIN    25    // LCK on PCM5102A
#define I2S_DATA_PIN  22    // DIN on PCM5102A

#define CPU_REPORT    true  // print audio CPU load to Serial once a second

// ============================================================
//  Audio settings
// ============================================================
#define SAMPLE_RATE   44100
#define BLOCK_SIZE    64
#define FILTER_SUB    16       // filter coefficients updated every 16 samples
#define BEND_RANGE    2.0f
#define NUM_VOICES    6
#define POLY_GAIN     0.8f     // headroom for chords (soft clip catches peaks)
#define DRIFT_MAX_CENTS 15.0f

#define CHO_LEN       1024     // chorus buffer (23 ms, power of two)
#define DLY_LEN       26460    // delay buffer per channel (0.6 s)

static const float SR     = (float)SAMPLE_RATE;
static const float INV_SR = 1.0f / SR;
static const float PI_F   = 3.14159265f;  // single-precision pi (Arduino's PI is a slow double)

// Fast 2^x – accurate to ~0.2 cents, far quicker than powf() on the ESP32
static inline float fastExp2(float x) {
  int32_t xi = (int32_t)x;
  if ((float)xi > x) xi--;                // floor
  float f = x - (float)xi;
  float p = 1.0f + f * (0.6960656f + f * (0.2244943f + f * 0.0794402f));
  union { float f; int32_t i; } u;
  u.f = p;
  u.i += (int32_t)((uint32_t)xi << 23);
  return u.f;
}

// Fast tan(x) for the filter coefficient (accurate for 0 <= x <= 1.3)
static inline float fastTan(float x) {
  float x2 = x * x;
  return x * (15.0f - x2) / (15.0f - 6.0f * x2);
}

// ============================================================
//  Colours / layout (matches analyser style)
// ============================================================
#define COL_BLACK    ST77XX_BLACK
#define COL_WHITE    ST77XX_WHITE
#define COL_YELLOW   ST77XX_YELLOW
#define COL_CYAN     ST77XX_CYAN
#define COL_GREEN    ST77XX_GREEN
#define COL_RED      ST77XX_RED
#define COL_ORANGE   0xFD20
#define COL_GREY     0x7BEF
#define COL_DKGREY   0x2945
#define COL_SELBG    0x1082
#define HDR_BG       (PAGE_COLS[curPage])   // header colour follows the current page
#define HDR_FG       COL_BLACK

#define HDR_H         16
#define FOOTER_Y     108
#define HINT_TY      110
#define FOOTER_TY    119
#define LIST_Y        20
#define ROW_H         12
#define UI_REFRESH_MS 40

Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// ============================================================
//  Parameters
// ============================================================
// The first 33 parameters keep their original order and storage layout;
// the NOISE page parameters come after them and are stored separately.
enum ParamId : uint8_t {
  P_WAVE1, P_WAVE2, P_TUNE2, P_FINE2, P_MIX, P_SUB, P_DRIFT,        // OSC
  P_PW, P_PWMDEPTH, P_PWMRATE,                                       // PULSE
  P_CUTOFF, P_RESO, P_ENVAMT, P_DRIVE, P_SLOPE,                      // FILTER
  P_ATTACK, P_DECAY, P_SUSTAIN, P_RELEASE,                           // ENV
  P_LFOWAVE, P_LFORATE, P_LFOPITCH, P_LFOFILT, P_LFOAMP, P_LFOSYNC,  // LFO
  P_CHORUS, P_DLYTYPE, P_DLYTIME, P_DLYFB, P_DLYMIX,                 // FX
  P_MODE, P_GLIDE, P_VOLUME,                                         // VOICE
  P_NLEVEL, P_NCOLOR, P_NDECAY, P_NMUTE,                             // NOISE (v1.1)
   P_NTYPE,                                                           // NOISE (v1.2)
  P_DLYSYNC,                                                         // FX (v1.3 delay sync)
  P_LFODIV,                                                          // LFO (v1.3 tempo sync)
  P_CHORATE,                                                         // FX (v1.4 chorus rate)
  P_M1SRC, P_M1DST, P_M1AMT, P_M2SRC, P_M2DST, P_M2AMT,              // MOD matrix (v1.4)
  P_WHLVIB,                                                          // MOD: wheel vibrato AUTO / ON / OFF
  P_XMOD, P_SYNC,                                                    // PULSE page: cross-mod + hard sync (v1.4)
  P_MIDICH,                                                          // global
  P_MTUNE,                                                           // global: master tune (430..450 Hz)
  P_RANDOM,                                                          // action row on VOICE (not stored)
  P_REBOOT,                                                          // action row on VOICE (not stored)
  NUM_PARAMS
};

struct ParamDef {
  const char* name;
  uint8_t cc;       // 255 = no generic CC (Mode uses CC126/127)
  uint8_t maxVal;
  uint8_t defVal;
};

const ParamDef PARAMS[NUM_PARAMS] = {
  {"Osc1 Wave",  70,   7,   0},   // SAW TRI SQR PULSE + 4 mixes
  {"Osc2 Wave",  80,   7,   0},
  {"Osc2 Tune",  81,  24,  12},   // 12 = unison, 0 = -12 st, 24 = +12 st
  {"Osc2 Fine",  76, 127,  19},   // 0..50 cents
  {"Osc Mix",    82, 127,  64},   // 0 = osc1 only, 127 = osc2 only
  {"Sub Level",  78, 127,   0},
  {"Drift",      83, 127,  20},
  {"Width",     104, 127,   0},   // 0 = 50 % (square) .. 127 = 5 %
  {"PWM Depth", 105, 127,   0},
  {"PWM Rate",  106, 127,  50},
  {"Cutoff",     74, 127,  70},
  {"Resonance",  71, 127,  38},
  {"Env Amt",    77, 127,  64},
  {"Drive",      85, 127,  20},
  {"Slope",      86,   3,   0},   // 0 = 12 dB, 1 = 24 dB, 2 = POLI LP, 3 = POLI BP
  {"Attack",     73, 127,   3},
  {"Decay",      75, 127,  45},
  {"Sustain",    79, 127,  76},
  {"Release",    72, 127,  32},
  {"Wave",       87,   4,   0},   // SINE TRI SAW SQR S&H
  {"Rate",       88, 127,  64},   // 0.05 .. 20 Hz (64 = 1 Hz)
  {"To Pitch",   89, 127,   0},
  {"To Filter",  90, 127,   0},
  {"To Amp",    102, 127,   0},
  {"Sync",      103,   1,   0},   // 0 = FREE, 1 = KEY
  {"Chorus",     93,   3,   0},   // OFF I II I+II
  {"Delay",     107,   3,   0},   // OFF CLEAN TAPE PING
  {"Time",      108, 127,  81},   // 20 .. 580 ms (81 = 250 ms)
  {"Feedback",  109, 127,  40},
  {"Dly Mix",   110, 127,  25},
  {"Mode",      255,   1,   0},   // 0 = MONO, 1 = POLY
  {"Glide",       5, 127,   0},
  {"Volume",      7, 127, 100},
  {"Level",     111, 127,   0},   // noise level
  {"Colour",    112, 127, 100},   // DARK .. BRIGHT
  {"Decay",     113, 127,   0},   // 0 = HOLD, else burst 5 ms .. 2 s
  {"Oscs",      114,   1,   0},   // 0 = ON, 1 = MUTED (noise only)
    {"Type",      115,   3,   0},   // WHITE PINK BROWN DIGITAL
   {"Dly Sync",  116,   5,   0},   // OFF 1/4 1/8D 1/8 1/8T 1/16
  {"Rate Sync", 117,   9,   0},   // OFF 4BAR 2BAR 1BAR 1/2 1/4 1/8 1/16 1/4T 1/8T
  {"Cho Rate",   94, 127,   0},   // 0 = AUTO (classic I / II rates), 1..127 = 0.05 .. 8 Hz
  {"Source 1",  255,   6,   0},   // OFF WHEEL VELO AFTER LFO ENV KEY
  {"Dest 1",    255,  10,   0},   // CUTOFF PEAK PITCH OSC2 PW MIX LFO RATE AMP DLY MIX CHO DEPTH
  {"Amount 1",  255, 127,  64},   // 64 = 0, 0 = -100 %, 127 = +100 %
  {"Source 2",  255,   6,   0},
  {"Dest 2",    255,  10,   0},
  {"Amount 2",  255, 127,  64},
  {"Wheel Vib", 255,   2,   0},   // AUTO (off when WHEEL is a mod source) / ON / OFF
  {"X-Mod",      21, 127,   0},   // osc 2 frequency-modulates osc 1 (JX-8P style), 0 = off
  {"Sync",       20,   1,   0},   // OFF / ON: osc 1 hard-synced to osc 2
  {"MIDI Ch",   255,  16,   0},   // 0 = OMNI (global, not stored per patch)
  {"Tune A4",   255,  20,  10},   // 430..450 Hz, 10 = 440 Hz (global, not stored per patch)
  {"Random",    255,   1,   0},   // B = new random patch, C = undo
  {"Reboot",    255,   1,   0},   // B = reboot (asks first)
};

// Pages – each fits on screen without scrolling
struct PageDef { const char* name; uint8_t count; uint8_t ids[7]; };
const PageDef PAGES[] = {
  {"OSC",    7, { P_WAVE1, P_WAVE2, P_TUNE2, P_FINE2, P_MIX, P_SUB, P_DRIFT }},
  {"PW/XMOD", 5, { P_PW, P_PWMDEPTH, P_PWMRATE, P_XMOD, P_SYNC, 0, 0 }},
  {"NOISE",  5, { P_NLEVEL, P_NTYPE, P_NCOLOR, P_NDECAY, P_NMUTE, 0, 0 }},
  {"FILTER", 5, { P_CUTOFF, P_RESO, P_ENVAMT, P_DRIVE, P_SLOPE, 0, 0 }},
  {"ENV",    4, { P_ATTACK, P_DECAY, P_SUSTAIN, P_RELEASE, 0, 0, 0 }},
  {"LFO",    7, { P_LFOWAVE, P_LFORATE, P_LFODIV, P_LFOPITCH, P_LFOFILT, P_LFOAMP, P_LFOSYNC }},
  {"MOD",    7, { P_M1SRC, P_M1DST, P_M1AMT, P_M2SRC, P_M2DST, P_M2AMT, P_WHLVIB }},
  {"FX",     7, { P_CHORUS, P_CHORATE, P_DLYTYPE, P_DLYTIME, P_DLYSYNC, P_DLYFB, P_DLYMIX }},
   {"VOICE",  7, { P_MODE, P_GLIDE, P_VOLUME, P_MIDICH, P_MTUNE, P_RANDOM, P_REBOOT }},
};
#define NUM_PAGES 9

// Header colour for each page (black header text stays readable on all)
const uint16_t PAGE_COLS[NUM_PAGES] = {
  COL_YELLOW,   // OSC
  0x867F,       // PULSE  – sky blue
  0xC618,       // NOISE  – silver
  COL_ORANGE,   // FILTER
  COL_GREEN,    // ENV
  0xF81F,       // LFO    – magenta
  0x07FF,       // MOD    – cyan
  0xFB2C,       // FX     – coral
  0xB5DF,       // VOICE  – lavender
};

const char* const OSC_WAVE_NAMES[8]   = { "SAW", "TRI", "SQR", "PULSE",
                                          "SAW+SQR", "SAW+TRI", "TRI+SQR", "SAW+PUL" };
const char* const LFO_WAVE_NAMES[5]   = { "SINE", "TRI", "SAW", "SQR", "S&H" };
const char* const CHORUS_NAMES[4]     = { "OFF", "I", "II", "I+II" };
const char* const MOD_SRC_NAMES[7]    = { "OFF", "WHEEL", "VELOCITY", "AFTERTCH", "LFO", "ENV", "KEY" };
const char* const MOD_DST_NAMES[11]   = { "CUTOFF", "PEAK", "PITCH", "OSC2 PIT", "PULSE W", "OSC MIX", "LFO RATE", "VOLUME",
                                          "DLY MIX", "CHO DPTH", "X-MOD" };
const char* const DELAY_NAMES[4]      = { "OFF", "CLEAN", "TAPE", "PING" };
const char* const NOISE_TYPE_NAMES[4] = { "WHITE", "PINK", "BROWN", "DIGITAL" };
const char* const DSYNC_NAMES[6]      = { "OFF", "1/4", "1/8D", "1/8", "1/8T", "1/16" };
const float       DSYNC_BEATS[6]      = { 0.0f, 1.0f, 0.75f, 0.5f, 0.33333f, 0.25f };
const char* const LFO_DIV_NAMES[10]   = { "OFF", "4 BAR", "2 BAR", "1 BAR", "1/2", "1/4",
                                          "1/8", "1/16", "1/4T", "1/8T" };
const uint16_t    LFO_DIV_TICKS[10]   = { 0, 384, 192, 96, 48, 24, 12, 6, 16, 8 };   // clock ticks per cycle

// Chorus rate: 1..127 -> 0.05 .. 8 Hz (exponential)
static inline float choRateHz(uint8_t v) { return 0.05f * powf(160.0f, (float)(v - 1) / 126.0f); }

volatile uint8_t pv[NUM_PARAMS];          // written by UI + MIDI (core 1), read by audio (core 0)
volatile float   pModWheel = 0.0f;
volatile float   pBend     = 0.0f;
volatile float   pAfter    = 0.0f;            // channel aftertouch 0..1 (mod matrix source)

static inline float pf(uint8_t id) { return pv[id] / 127.0f; }

// Status published by the audio core for the display / deferred save
volatile int      gDisplayNote = 60;
volatile bool     gAnyGate     = false;
volatile uint32_t gCpuPct      = 0;       // worst-case audio load over the last second

volatile unsigned long lastMidiMs = 0;
volatile int16_t lastCcNum = -1;

// MIDI clock (written on core 1, read by the audio core and the display)
volatile uint32_t      clockTicks  = 0;
volatile uint32_t      clockBase   = 0;    // tick count at the last MIDI Start (CLK beat flash)
volatile unsigned long lastClockMs = 0;
volatile float         clockTickUs = 20833.0f; // smoothed time between clock ticks (120 BPM)
uint32_t               lastTickUs  = 0;

// ============================================================
//  Rotary encoder (interrupt-driven quadrature decoder)
// ============================================================
volatile int32_t encRaw  = 0;
volatile uint8_t encPrev = 0;
// Valid transitions of (previous AB, current AB) -> -1 / 0 / +1
const int8_t ENC_TABLE[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

void IRAM_ATTR encISR() {
  uint8_t s = (uint8_t)((digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN));
  encRaw += ENC_TABLE[(encPrev << 2) | s];
  encPrev = s;
}

// Whole detents turned since the last call (positive = clockwise)
int readEncoder() {
  static int32_t consumed = 0;
  int32_t diff = encRaw - consumed;
  int d = diff / ENC_STEPS_PER_DETENT;
  if (d) consumed += d * ENC_STEPS_PER_DETENT;
  return ENC_REVERSE ? -d : d;
}

// ============================================================
//  Note event queue (core 1 -> core 0), single producer / consumer
// ============================================================
#define EV_NOTE_ON   0
#define EV_NOTE_OFF  1
#define EV_ALL_OFF   2
#define EVQ_SIZE     64                      // power of two

struct NoteEvent { uint8_t type, note, vel; };
NoteEvent evq[EVQ_SIZE];
volatile uint8_t evHead = 0, evTail = 0;

void pushEvent(uint8_t type, uint8_t note, uint8_t vel) {
  uint8_t next = (evHead + 1) & (EVQ_SIZE - 1);
  if (next == evTail) return;                // queue full – drop (never in practice)
  evq[evHead].type = type;
  evq[evHead].note = note;
  evq[evHead].vel  = vel;
  __sync_synchronize();                      // data visible before head moves
  evHead = next;
}

// ============================================================
//  Sequencer data – 8 patterns of 16 steps, saved in flash (Preferences)
// ============================================================
#define SEQ_STEPS   16
#define SEQ_PATS     8
#define SEQ_MAGIC   0x4D
#define ST_ACC       1                       // step flag: accent
#define ST_SLD       2                       // step flag: slide into the next step

struct MStep { uint8_t note, flags; };       // note 0 = rest
struct MPat  { MStep st[SEQ_STEPS]; uint8_t len, pad; };
struct MSeq  { uint8_t magic, bpm, pad1, pad2; MPat pat[SEQ_PATS]; };
MSeq seq;
#define CHAIN_END (seq.pad1)                 // 0 = chain off, 1..7 = chain P1 to P2..P8

volatile bool    seqRunning   = false;
volatile bool    seqToggleReq = false;       // UI -> audio core: start / stop
volatile bool    seqRec       = false;       // recording armed
volatile bool    seqDirty     = false;       // needs saving
volatile bool    seqMidiStart = false;
volatile bool    seqMidiStop  = false;
volatile uint8_t gSeqPos      = 0;           // step just played (for the screen)
volatile uint8_t editPat      = 0;           // pattern shown / edited
volatile uint8_t queuedPat    = 0;           // pattern to play from the next pass
volatile uint8_t playPat      = 0;           // pattern playing now
Preferences prefs;

void seqDefaults() {
  memset(&seq, 0, sizeof(seq));
  seq.magic = SEQ_MAGIC;
  seq.bpm   = 120;
  for (int i = 0; i < SEQ_PATS; i++) seq.pat[i].len = SEQ_STEPS;
  // P1: an acid riff in C.  Flags: 1 = accent, 2 = slide, 3 = both
  static const uint8_t R1[SEQ_STEPS][2] = {
    {36,1},{36,0},{48,2},{36,0}, {0,0},{39,0},{36,1},{46,2},
    {48,0},{36,0},{43,1},{36,0}, {41,2},{43,0},{36,1},{0,0} };
  // P2: a minor bassline in A
  static const uint8_t R2[SEQ_STEPS][2] = {
    {33,1},{45,0},{33,0},{45,2}, {43,0},{33,0},{40,1},{33,0},
    {36,0},{48,2},{47,0},{45,0}, {33,1},{0,0},{31,0},{43,2} };
  for (int s = 0; s < SEQ_STEPS; s++) {
    seq.pat[0].st[s].note = R1[s][0]; seq.pat[0].st[s].flags = R1[s][1];
    seq.pat[1].st[s].note = R2[s][0]; seq.pat[1].st[s].flags = R2[s][1];
  }
}

void seqLoad() {
  prefs.begin("malice", false);
  size_t n = prefs.getBytes("seq", &seq, sizeof(seq));
  if (n != sizeof(seq) || seq.magic != SEQ_MAGIC || seq.bpm < 60 || seq.bpm > 200) {
    seqDefaults();
    return;
  }
  if (CHAIN_END > SEQ_PATS - 1) CHAIN_END = 0;
  for (int i = 0; i < SEQ_PATS; i++) {
    if (seq.pat[i].len < 1 || seq.pat[i].len > SEQ_STEPS) seq.pat[i].len = SEQ_STEPS;
    for (int s = 0; s < SEQ_STEPS; s++) {
      if (seq.pat[i].st[s].note > 127) seq.pat[i].st[s].note = 0;
      seq.pat[i].st[s].flags &= 3;
    }
  }
}

void seqSave() {
  seqDirty = false;
  prefs.putBytes("seq", &seq, sizeof(seq));
}

// ============================================================
//  Patch memory
// ============================================================
#define NUM_PATCHES      30
#define NUM_FACTORY      30                // a factory sound for every slot
#define PATCH_PARAMS     P_MIDICH          // params 0..37 stored per patch
#define MAIN_PARAMS      P_NLEVEL          // original 33 params – original layout
#define EXT_PARAMS       4                 // v1.1 noise params (level, colour, decay, mute)
#define EE_SIZE          4096
#define EE_MAGIC         0x4D              // 'M' for Mindburner
#define EE_VERSION       21                // v1.4: POLI, slots 21-30, chorus rate, mod matrix, wheel vib, X-Mod, P13, P27
#define EE_ADDR_MAGIC    0
#define EE_ADDR_VER      1
#define EE_ADDR_LAST     2
#define EE_ADDR_MIDICH   3
#define EE_ADDR_PATCHES  4
#define EE_PATCH_STRIDE  (MAIN_PARAMS + 1) // main params + checksum
#define EE_ADDR_EXT      700               // v1.1 noise params, stored separately
#define EE_EXT_STRIDE    (EXT_PARAMS + 1)  // noise params + checksum
#define EE_ADDR_EXT2     850               // v1.2 noise type: value + check byte
#define EE_EXT2_STRIDE   2
#define EE_ADDR_EXT3     900               // v1.3 delay sync: value + check byte
#define EE_ADDR_EXT4     950               // v1.3 LFO rate sync: value + check byte
#define EE_ADDR_MTUNE    1000              // master tune: value + check byte (global)
#define EE_ADDR_HI_OLD   1100              // early v1.4 builds kept slots 21-30 here (moved, see upgrade)
#define EE_ADDR_HI       2048              // v1.4: slots 21-30, whole patch + checksum, fixed 64-byte slots
#define EE_HI_STRIDE     64                // room for future parameters without moving again
#define EE_ADDR_EXT6     1600              // v1.4 mod matrix, slots 1-20: 6 values + checksum
#define EE_ADDR_EXT7     1760              // v1.4 wheel vibrato, slots 1-20: value + check byte
#define EE_ADDR_EXT8     1800              // v1.4 X-Mod + Sync, slots 1-20: 2 values + checksum
#define EE_ADDR_EXT5     1540              // v1.4 chorus rate, slots 1-20: value + check byte
#define DEFER_SAVE_MS    3000

// Waves: 0 SAW 1 TRI 2 SQR 3 PULSE 4 SAW+SQR 5 SAW+TRI 6 TRI+SQR 7 SAW+PUL
// Chorus: 0 OFF 1 I 2 II 3 I+II      Delay: 0 OFF 1 CLEAN 2 TAPE 3 PING
// Rows may omit the noise values at the end: omitted values are 0 (noise off, WHITE).
//  W1 W2 Tn2 Fin Mix Sub Drf | PW PWd PWr | Cut Res Env Drv Slp | Att Dec Sus Rel | LWv LRt LPi LFi LAm LSy | Cho DTy DTm DFb DMx | Mode Gld Vol | NLv NCo NDc NMu NTy
uint8_t FACTORY[NUM_FACTORY][PATCH_PARAMS] = {        // amounts fixed up at boot (fixFactoryMod)
  { 0, 0, 12, 19, 64,   0, 20,   0,  0, 50,   70,  38, 64, 20, 0,    3, 45,  76, 32,   0, 64,  0,  0,  0, 0,   0, 0,  81,  0,  0,   0,  0,  61 }, //  1 Classic
  { 0, 0, 12,  0,  0,   0, 10,   0,  0, 50,   30, 105, 95, 60, 1,    0, 40,   0, 20,   0, 64,  0,  0,  0, 0,   0, 2,  68, 45, 25,   0, 25, 127 }, //  2 Acid
  { 7, 3, 12, 20, 55,   0, 30,  40, 60, 75,   75,  45, 20, 15, 0,   30, 60, 110, 60,   0,100, 12,  0,  0, 0,   3, 2,  97, 25, 15,   1,  0,  32 }, //  3 Vox Humana
  { 0, 0, 24, 10, 50,   0, 15,   0,  0, 50,   40,  50, 90, 10, 0,    0, 35,   0, 35,   0, 64,  0,  0,  0, 0,   1, 3,  81, 45, 30,   1,  0, 127 }, //  4 Pluck
  { 2, 2, 19, 30, 64,   0, 25,   0,  0, 50,   75,  45, 40, 30, 0,    5, 60,  90, 40,   0,100, 18,  0,  0, 0,   0, 1,  97, 40, 25,   0, 35,  47 }, //  5 Square Lead
  { 0, 0, 12, 12, 64, 100, 20,   0,  0, 50,   25,  10, 50, 50, 1,    2, 40,  80, 25,   0, 64,  0,  0,  0, 0,   0, 0,  81,  0,  0,   0,  0,  90 }, //  6 Sub Bass
  { 0, 2, 12, 10, 50,  80, 15,   0,  0, 50,   35,  80, 20, 70, 1,    2, 40, 100, 20,   0, 80,  0, 70,  0, 1,   0, 0,  81,  0,  0,   0,  0,  70 }, //  7 Wobble Bass
  { 2, 2, 24,  0, 30,   0,  0,   0,  0, 50,   50, 100, 30, 20, 0,    0, 30,  40, 30,   4, 95,  0, 80,  0, 0,   1, 2,  97, 55, 35,   0,  0,  66 }, //  8 S&H Bleeps
  { 0, 3, 12, 25, 64,   0, 50,  40, 50, 50,   60,  15, 15,  0, 0,   60, 70, 100, 70,   1, 70,  8, 10,  0, 0,   2, 0,  81,  0,  0,   1,  0,  56 }, //  9 Strings
  { 1, 2, 24,  5, 40,   0, 10,   0,  0, 50,   60,  20, 40, 10, 0,    1, 60,  50, 50,   0, 90,  0,  0, 70, 0,   1, 3,  97, 30, 20,   1,  0,  98 }, // 10 Tremolo Keys
  { 0, 0, 12, 15, 64,   0, 25,   0,  0, 50,   45,  20, 80, 20, 0,   25, 50,  90, 30,   0, 64,  0,  0,  0, 0,   1, 0,  81,  0,  0,   1,  0,  46 }, // 11 Brass
  { 0, 2, 24, 10, 50,   0, 20,   0,  0, 50,   60, 110, 60, 45, 3,    2, 40,  70, 25,   1, 90, 15,  0,  0, 0,   0, 2,  97, 40, 25,   0, 30,  46 }, // 12 Screamer Lead (POLI BP)
  { 3, 3,  0,  8, 40,  70, 15,  40, 50, 60,   40,  30, 60, 30, 1,    0, 35,  50, 15,   0, 64,  0,  0,  0, 0,   0, 2,  81,110,  0,   0, 10, 105,    0,  0,  0, 0, 0,  0, 0, 0,   1, 8, 127 }, // 13 PWM Bass (wheel -> tape echo)
  { 7, 0, 12, 30, 64,   0, 50,  30, 60, 30,   50,  45, 20,  0, 0,   90, 90, 100, 90,   1, 30,  0, 45,  0, 0,   2, 2, 111, 45, 30,   1,  0, 107,   30, 70,  0, 0, 1 }, // 14 Evolving Pad (+ pink breath)
  { 7, 7,  0, 40, 60,   0, 30,  60, 90, 70,   80,  20, 30, 40, 0,    5, 50,  90, 40,   0, 64,  0,  0,  0, 0,   3, 0,  81,  0,  0,   0, 60,  67 }, // 15 Hoover
  { 1, 5, 24,  5, 30,   0, 15,   0,  0, 50,   60,  10, 50,  0, 0,    0, 70,  30, 45,   1, 80,  0,  0, 25, 0,   1, 0,  81,  0,  0,   1,  0, 127 }, // 16 Warm Keys
  { 0, 0, 12, 30, 64,   0, 60,   0,  0, 50,   85,  25, 30, 10, 0,    3, 60,  80, 35,   0, 85, 10,  0,  0, 0,   2, 3,  97, 45, 30,   0, 20,  65 }, // 17 Trance Lead
  { 0, 0, 12, 25, 64,  60, 30,   0,  0, 50,   45,  20, 25, 50, 1,    2, 60,  90, 25,   0, 40,  0, 15,  0, 0,   0, 0,  81,  0,  0,   0, 15,  66 }, // 18 Reese Bass
  { 1, 2, 24, 20, 40,   0, 10,   0,  0, 50,   75,  60, 70,  0, 0,    0, 45,   0, 50,   0, 64,  0,  0,  0, 0,   1, 3,  81, 50, 35,   1,  0, 127,   50,110, 25, 0, 0 }, // 19 Glass Pluck (+ chiff)
  { 0, 3, 12, 15, 40,  40, 20,  20,  0, 50,   35,  95, 85, 40, 2,    0, 35,  25, 20,   0, 64,  0,  0,  0, 0,   1, 2,  81, 30, 18,   0, 12,  80 }, // 20 Soviet Bass (POLI LP)
  { 0, 0, 12, 18, 60,   0, 25,   0,  0, 50,   55,  70, 60, 30, 2,   15, 60,  90, 35,   0, 64,  0,  0,  0, 0,   1, 0,  81,  0,  0,   1,  0,  70 }, // 21 Poli Brass
  { 0, 0, 12,  0,  0,   0, 10,   0,  0, 50,   28, 115,100, 60, 2,    0, 38,   0, 20,   0, 64,  0,  0,  0, 0,   0, 2,  68, 45, 25,   0, 25, 100 }, // 22 Acid Poli
  { 0, 2, 12, 12, 50,   0, 15,   0,  0, 50,   45, 100, 30, 40, 3,    0, 60,  90, 30,   1, 40,  0, 50,  0, 0,   0, 0,  81,  0,  0,   0,  0,  45 }, // 23 Talk Box (POLI BP + LFO)
  { 3, 3, 12, 15, 50,   0, 40,  30, 50, 30,   45,  15, 25,  0, 1,   80, 90, 110, 90,   0, 64,  0,  0,  0, 0,   1, 2,  97, 40, 30,   1,  0,  80,    0,  0,  0, 0, 0,  0, 0, 25 }, // 24 Dark Pad (slow swish)
  { 2, 2, 24,  0, 40,   0, 10,   0,  0, 50,   70,  40, 50, 10, 0,    0, 25,   0, 30,   0, 64,  0,  0,  0, 0,   0, 3,  81, 50, 35,   1,  0, 110,    0,  0,  0, 0, 0,  2, 0 }, // 25 Echo Bleep (1/8D)
  { 0, 0, 12,  0,  0,   0,  0,   0,  0, 50,   60,  80, 20,  0, 3,   90, 90, 110,100,   1, 20,  0, 40,  0, 0,   1, 0,  81,  0,  0,   1,  0, 110,  110, 90,  0, 1, 1,  0, 0 }, // 26 Cold Wind (noise)
  { 1, 0, 23, 38, 15,   0,  0,   0,  0, 50,  100,  20, 30, 20, 0,    0, 75,   0, 70,   0, 64,  0,  0,  0, 0,   0, 3,  81, 40, 25,   1,  0,  82,    0,  0,  0, 0, 0,  2, 0, 0,   1,10,127,   0, 0, 64,   0, 55, 0 }, // 27 Metal Bell (X-Mod; mod wheel = more clang)
  { 3, 2,  0,  5, 30,  90, 15,  30, 40, 40,   38,  30, 55, 20, 1,    0, 45,  40, 20,   0, 64,  0,  0,  0, 0,   1, 0,  81,  0,  0,   0,  0,  90 }, // 28 Juno Bass
  { 0, 0, 12, 22, 55,   0, 35,   0,  0, 50,   70,  15, 15,  0, 0,   60, 80, 110, 80,   1, 75,  4,  0,  0, 0,   2, 3,  81, 50, 35,   1,  0,  80,    0,  0,  0, 0, 0,  2, 0, 72 }, // 29 Ping-Pong Strings (1/8D, chorus II at 0.87 Hz)
  { 6, 1, 24,  5, 45,   0, 10,   0,  0, 50,   90,  10, 30,  0, 0,    0, 70,  40, 50,   0, 64,  0,  0,  0, 0,   2, 3,  81, 40, 30,   1,  0,  85 }, // 30 Glass Organ
};

uint8_t currentPatch = 0;
uint8_t loadedSnap[PATCH_PARAMS];

uint8_t savedMidiCh   = 0;
uint8_t savedMTune    = 10;
uint8_t savedLast     = 0;
uint8_t lastSeenCh    = 255;
uint8_t lastSeenPatch = 255;
unsigned long settingsChangeT = 0;

uint8_t patchChecksum(const uint8_t* p, uint8_t n) {
  uint8_t c = 0xA5;
  for (uint8_t i = 0; i < n; i++) {
    c = (uint8_t)((c << 1) | (c >> 7));
    c ^= p[i];
  }
  return c;
}

// Factory rows only list the parameters they use: give the mod matrix its OFF / 0 % defaults
void fixFactoryMod() {
  for (uint8_t s = 0; s < NUM_FACTORY; s++)
    for (uint8_t i = 0; i < 6; i++) {
      uint8_t* f = (uint8_t*)FACTORY[s];
      if (i == 2 || i == 5) { if (f[P_M1SRC + i] == 0) f[P_M1SRC + i] = 64; }
    }
}

void eeWritePatch(uint8_t slot, const uint8_t* p) {
  if (slot >= 20) {                                             // v1.4 slots 21-30: one block each
    int hi = EE_ADDR_HI + (slot - 20) * EE_HI_STRIDE;
    for (uint8_t i = 0; i < PATCH_PARAMS; i++) EEPROM.write(hi + i, p[i]);
    EEPROM.write(hi + PATCH_PARAMS, patchChecksum(p, PATCH_PARAMS));
    return;
  }
  int base = EE_ADDR_PATCHES + slot * EE_PATCH_STRIDE;          // main params (original layout)
  for (uint8_t i = 0; i < MAIN_PARAMS; i++) EEPROM.write(base + i, p[i]);
  EEPROM.write(base + MAIN_PARAMS, patchChecksum(p, MAIN_PARAMS));
  int ext = EE_ADDR_EXT + slot * EE_EXT_STRIDE;                 // v1.1 noise params
  for (uint8_t i = 0; i < EXT_PARAMS; i++) EEPROM.write(ext + i, p[MAIN_PARAMS + i]);
  EEPROM.write(ext + EXT_PARAMS, patchChecksum(p + MAIN_PARAMS, EXT_PARAMS));
  int ext2 = EE_ADDR_EXT2 + slot * EE_EXT2_STRIDE;              // v1.2 noise type
  EEPROM.write(ext2,     p[P_NTYPE]);
  EEPROM.write(ext2 + 1, (uint8_t)(p[P_NTYPE] ^ 0x5A));
  int ext3 = EE_ADDR_EXT3 + slot * 2;                           // v1.3 delay sync
  EEPROM.write(ext3,     p[P_DLYSYNC]);
  EEPROM.write(ext3 + 1, (uint8_t)(p[P_DLYSYNC] ^ 0x5A));
  int ext4 = EE_ADDR_EXT4 + slot * 2;                           // v1.3 LFO rate sync
  EEPROM.write(ext4,     p[P_LFODIV]);
  EEPROM.write(ext4 + 1, (uint8_t)(p[P_LFODIV] ^ 0x5A));
  int ext5 = EE_ADDR_EXT5 + slot * 2;                           // v1.4 chorus rate
  EEPROM.write(ext5,     p[P_CHORATE]);
  EEPROM.write(ext5 + 1, (uint8_t)(p[P_CHORATE] ^ 0x5A));
  int ext6 = EE_ADDR_EXT6 + slot * 7;                           // v1.4 mod matrix
  for (uint8_t i = 0; i < 6; i++) EEPROM.write(ext6 + i, p[P_M1SRC + i]);
  EEPROM.write(ext6 + 6, patchChecksum(p + P_M1SRC, 6));
  int ext7 = EE_ADDR_EXT7 + slot * 2;                           // v1.4 wheel vibrato
  EEPROM.write(ext7,     p[P_WHLVIB]);
  EEPROM.write(ext7 + 1, (uint8_t)(p[P_WHLVIB] ^ 0x5A));
  int ext8 = EE_ADDR_EXT8 + slot * 3;                           // v1.4 X-Mod + Sync
  EEPROM.write(ext8,     p[P_XMOD]);
  EEPROM.write(ext8 + 1, p[P_SYNC]);
  EEPROM.write(ext8 + 2, patchChecksum(p + P_XMOD, 2));
}

bool eeReadPatch(uint8_t slot, uint8_t* p) {
  if (slot >= 20) {
    int hi = EE_ADDR_HI + (slot - 20) * EE_HI_STRIDE;
    for (uint8_t i = 0; i < PATCH_PARAMS; i++) {
      p[i] = EEPROM.read(hi + i);
      if (p[i] > PARAMS[i].maxVal) return false;
    }
    return EEPROM.read(hi + PATCH_PARAMS) == patchChecksum(p, PATCH_PARAMS);
  }
  int base = EE_ADDR_PATCHES + slot * EE_PATCH_STRIDE;
  for (uint8_t i = 0; i < MAIN_PARAMS; i++) {
    p[i] = EEPROM.read(base + i);
    if (p[i] > PARAMS[i].maxVal) return false;
  }
  if (EEPROM.read(base + MAIN_PARAMS) != patchChecksum(p, MAIN_PARAMS)) return false;

  // Noise settings – patches saved before the NOISE page existed get noise off
  int  ext   = EE_ADDR_EXT + slot * EE_EXT_STRIDE;
  bool extOK = true;
  for (uint8_t i = 0; i < EXT_PARAMS; i++) {
    p[MAIN_PARAMS + i] = EEPROM.read(ext + i);
    if (p[MAIN_PARAMS + i] > PARAMS[MAIN_PARAMS + i].maxVal) extOK = false;
  }
  if (!extOK || EEPROM.read(ext + EXT_PARAMS) != patchChecksum(p + MAIN_PARAMS, EXT_PARAMS)) {
    for (uint8_t i = 0; i < EXT_PARAMS; i++) p[MAIN_PARAMS + i] = PARAMS[MAIN_PARAMS + i].defVal;
  }

  // Noise type – patches saved before v1.2 get WHITE (sounds as they did)
  int ext2 = EE_ADDR_EXT2 + slot * EE_EXT2_STRIDE;
  uint8_t t = EEPROM.read(ext2);
  p[P_NTYPE] = (t <= PARAMS[P_NTYPE].maxVal && EEPROM.read(ext2 + 1) == (uint8_t)(t ^ 0x5A)) ? t : 0;

  // Delay sync – patches saved before v1.3 get OFF (sound as they did)
  int ext3 = EE_ADDR_EXT3 + slot * 2;
  uint8_t ds = EEPROM.read(ext3);
  p[P_DLYSYNC] = (ds <= PARAMS[P_DLYSYNC].maxVal && EEPROM.read(ext3 + 1) == (uint8_t)(ds ^ 0x5A)) ? ds : 0;

  // LFO rate sync – older patches get OFF
  int ext4 = EE_ADDR_EXT4 + slot * 2;
  uint8_t ld = EEPROM.read(ext4);
  p[P_LFODIV] = (ld <= PARAMS[P_LFODIV].maxVal && EEPROM.read(ext4 + 1) == (uint8_t)(ld ^ 0x5A)) ? ld : 0;

  // Chorus rate – older patches get AUTO (sound as they did)
  int ext5 = EE_ADDR_EXT5 + slot * 2;
  uint8_t cr = EEPROM.read(ext5);
  p[P_CHORATE] = (cr <= PARAMS[P_CHORATE].maxVal && EEPROM.read(ext5 + 1) == (uint8_t)(cr ^ 0x5A)) ? cr : 0;

  // Mod matrix – older patches get both slots OFF
  int  ext6 = EE_ADDR_EXT6 + slot * 7;
  bool mOK  = true;
  for (uint8_t i = 0; i < 6; i++) {
    p[P_M1SRC + i] = EEPROM.read(ext6 + i);
    if (p[P_M1SRC + i] > PARAMS[P_M1SRC + i].maxVal) mOK = false;
  }
  if (!mOK || EEPROM.read(ext6 + 6) != patchChecksum(p + P_M1SRC, 6))
    for (uint8_t i = 0; i < 6; i++) p[P_M1SRC + i] = PARAMS[P_M1SRC + i].defVal;

  // Wheel vibrato – older patches get AUTO
  int ext7 = EE_ADDR_EXT7 + slot * 2;
  uint8_t wv = EEPROM.read(ext7);
  p[P_WHLVIB] = (wv <= PARAMS[P_WHLVIB].maxVal && EEPROM.read(ext7 + 1) == (uint8_t)(wv ^ 0x5A)) ? wv : 0;

  // X-Mod + Sync – older patches get both off
  int ext8 = EE_ADDR_EXT8 + slot * 3;
  p[P_XMOD] = EEPROM.read(ext8);
  p[P_SYNC] = EEPROM.read(ext8 + 1);
  if (p[P_XMOD] > 127 || p[P_SYNC] > 1 || EEPROM.read(ext8 + 2) != patchChecksum(p + P_XMOD, 2)) {
    p[P_XMOD] = 0; p[P_SYNC] = 0;
  }
  return true;
}

void loadPatch(uint8_t slot) {
  if (slot >= NUM_PATCHES) return;
  uint8_t p[PATCH_PARAMS];
  if (!eeReadPatch(slot, p)) memcpy(p, FACTORY[slot % NUM_FACTORY], PATCH_PARAMS);
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) pv[i] = p[i];
  memcpy(loadedSnap, p, PATCH_PARAMS);
  currentPatch = slot;
}

void savePatch(uint8_t slot) {
  uint8_t p[PATCH_PARAMS];
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) p[i] = pv[i];
  eeWritePatch(slot, p);
  EEPROM.write(EE_ADDR_LAST, slot);
  EEPROM.commit();
  memcpy(loadedSnap, p, PATCH_PARAMS);
  currentPatch  = slot;
  savedLast     = slot;
  lastSeenPatch = slot;
}

bool patchEdited() {
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) if (pv[i] != loadedSnap[i]) return true;
  return false;
}

void storageInit() {
  fixFactoryMod();
  EEPROM.begin(EE_SIZE);
  uint8_t oldVer = EEPROM.read(EE_ADDR_VER);
  if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC && oldVer >= 8 && oldVer <= 16) {
    eeWritePatch(12, FACTORY[12]);                    // v1.4.8: patch 13 gets wheel -> tape echo
    if (oldVer == 16) { EEPROM.write(EE_ADDR_VER, EE_VERSION); EEPROM.commit(); }
  }
  if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC && (oldVer == 14 || oldVer == 15)) {
    // v1.4.4 / v1.4.5: keep slots 21-30 (they had fewer parameters); new ones start off
    uint8_t oldN = (oldVer == 14) ? P_WHLVIB : P_XMOD;
    for (uint8_t s = 20; s < NUM_PATCHES; s++) {
      uint8_t p[PATCH_PARAMS];
      int  a  = EE_ADDR_HI + (s - 20) * EE_HI_STRIDE;
      bool ok = true;
      for (uint8_t i = 0; i < oldN; i++) { p[i] = EEPROM.read(a + i); if (p[i] > PARAMS[i].maxVal) ok = false; }
      if (!ok || EEPROM.read(a + oldN) != patchChecksum(p, oldN)) memcpy(p, FACTORY[s], PATCH_PARAMS);
      for (uint8_t i = oldN; i < PATCH_PARAMS; i++) p[i] = PARAMS[i].defVal;
      eeWritePatch(s, p);
    }
    EEPROM.write(EE_ADDR_VER, EE_VERSION);
    EEPROM.commit();
  }
  if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC && oldVer >= 10 && oldVer <= 13) {
    // Earlier v1.4 builds: move your slots 21-30 to their new home (mod matrix starts OFF)
    uint8_t oldN = (oldVer >= 12) ? 41 : 40;                              // params stored by that build
    for (uint8_t s = 20; s < NUM_PATCHES; s++) {
      uint8_t p[PATCH_PARAMS];
      memcpy(p, FACTORY[s], PATCH_PARAMS);
      int  a  = EE_ADDR_HI_OLD + (s - 20) * (oldN + 1);
      bool ok = true;
      for (uint8_t i = 0; i < oldN; i++) { p[i] = EEPROM.read(a + i); if (p[i] > PARAMS[i].maxVal) ok = false; }
      if (!ok || EEPROM.read(a + oldN) != patchChecksum(p, oldN)) memcpy(p, FACTORY[s], PATCH_PARAMS);
      else {
        if (oldN < 41) p[P_CHORATE] = 0;
        for (uint8_t i = 0; i < 6; i++) p[P_M1SRC + i] = PARAMS[P_M1SRC + i].defVal;
      }
      if (s == 28 && oldVer == 12) memcpy(p, FACTORY[28], PATCH_PARAMS);  // 29 tweak from v1.4.3
      eeWritePatch(s, p);
    }
    EEPROM.write(EE_ADDR_VER, EE_VERSION);
    EEPROM.commit();
  }
  if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC && oldVer >= 8 && oldVer <= 9) {
    // Upgrade: your patches 1-20 are kept; v1.3 boards get the new POLI sounds in 12 and 20
    if (oldVer == 8) { eeWritePatch(11, FACTORY[11]); eeWritePatch(19, FACTORY[19]); }
    for (uint8_t s = 20; s < NUM_PATCHES; s++) eeWritePatch(s, FACTORY[s]);   // new slots 21-30
    EEPROM.write(EE_ADDR_VER, EE_VERSION);
    EEPROM.commit();
  }
  if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC && oldVer >= 8 && oldVer <= 20) {
    eeWritePatch(26, FACTORY[26]);                    // v1.4.12: patch 27 = Metal Bell, wheel -> X-Mod
    EEPROM.write(EE_ADDR_VER, EE_VERSION);
    EEPROM.commit();
  }
  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC || EEPROM.read(EE_ADDR_VER) != EE_VERSION) {
    for (uint8_t s = 0; s < NUM_PATCHES; s++) eeWritePatch(s, FACTORY[s % NUM_FACTORY]);
    EEPROM.write(EE_ADDR_LAST, 0);
    EEPROM.write(EE_ADDR_MIDICH, 0);
    EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC);
    EEPROM.write(EE_ADDR_VER, EE_VERSION);
    EEPROM.commit();
  }
  uint8_t ch = EEPROM.read(EE_ADDR_MIDICH);
  pv[P_MIDICH] = (ch <= 16) ? ch : 0;
  savedMidiCh  = pv[P_MIDICH];
  uint8_t mt = EEPROM.read(EE_ADDR_MTUNE);
  pv[P_MTUNE] = (mt <= 20 && EEPROM.read(EE_ADDR_MTUNE + 1) == (uint8_t)(mt ^ 0xA5)) ? mt : 10;
  savedMTune  = pv[P_MTUNE];

  uint8_t last = EEPROM.read(EE_ADDR_LAST);
  if (last >= NUM_PATCHES) last = 0;
  loadPatch(last);
  savedLast = last;

  lastSeenCh    = pv[P_MIDICH];
  lastSeenPatch = currentPatch;
}

// Last-patch and MIDI-channel changes are written only after 3s of no
// further change AND while no note is held – flash writes stall both
// cores briefly, so this keeps them away from anything you're playing.
void serviceDeferredSave(unsigned long now) {
  uint8_t ch = pv[P_MIDICH];
  uint8_t mt = pv[P_MTUNE];
  static uint8_t lastSeenTune = 255;
  if (ch != lastSeenCh || currentPatch != lastSeenPatch || mt != lastSeenTune) {
    lastSeenCh = ch;
    lastSeenTune = mt;
    lastSeenPatch = currentPatch;
    settingsChangeT = now;
  }
  if ((ch != savedMidiCh || currentPatch != savedLast || mt != savedMTune) &&
      (now - settingsChangeT) >= DEFER_SAVE_MS && !gAnyGate) {
    EEPROM.write(EE_ADDR_MIDICH, ch);
    EEPROM.write(EE_ADDR_LAST, currentPatch);
    EEPROM.write(EE_ADDR_MTUNE, mt);
    EEPROM.write(EE_ADDR_MTUNE + 1, (uint8_t)(mt ^ 0xA5));
    EEPROM.commit();
    savedMidiCh = ch;
    savedMTune  = mt;
    savedLast   = currentPatch;
  }
}

// ============================================================
//  Shared helpers
// ============================================================
static inline float ccToTime(float v, float minT, float maxT) {
  return minT + v * v * (maxT - minT);
}

static inline float velCurve(uint8_t v) { return 0.3f + 0.7f * (v / 127.0f); }

static inline float lfoRateHz(float f)  { return 0.05f * powf(400.0f, f); }   // 0.05 .. 20 Hz
static inline float pulseWidth(float f) { return 0.5f - f * 0.45f; }         // 50 % .. 5 %
static inline float delaySecs(float f)  { return 0.02f + f * f * 0.56f; }    // 20 .. 580 ms

const char* const NOTE_NAMES[12] = {
  "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

void getNoteName(uint8_t note, char* buf) {
  if (note > 127) { strcpy(buf, "---"); return; }
  sprintf(buf, "%s%d", NOTE_NAMES[note % 12], (int)(note / 12) - 1);
}

void fmtTime(float t, char* out, size_t n) {
  if (t < 1.0f) snprintf(out, n, "%dms", (int)(t * 1000.0f + 0.5f));
  else          snprintf(out, n, "%.2fs", t);
}

void fmtHz(float hz, char* out, size_t n) {
  if (hz < 10.0f) snprintf(out, n, "%.2fHz", hz);
  else            snprintf(out, n, "%.1fHz", hz);
}

void formatParam(uint8_t id, uint8_t v, char* out, size_t n) {
  float f = v / 127.0f;
  int   pct = (int)(f * 100.0f + 0.5f);
  switch (id) {
    case P_WAVE1:
    case P_WAVE2:   snprintf(out, n, "%s", OSC_WAVE_NAMES[v < 8 ? v : 0]); break;
    case P_TUNE2:
      if (v == 12) snprintf(out, n, "0st");
      else         snprintf(out, n, "%+dst", (int)v - 12);
      break;
    case P_FINE2:   snprintf(out, n, "+%dc", (int)(f * 50.0f + 0.5f)); break;
    case P_MIX:     snprintf(out, n, "%d/%d", 100 - pct, pct); break;
    case P_PW:      snprintf(out, n, "%d%%", (int)(pulseWidth(f) * 100.0f + 0.5f)); break;
    case P_PWMRATE: fmtHz(lfoRateHz(f), out, n); break;
    case P_SUB:
    case P_DRIFT:
    case P_DRIVE:
    case P_PWMDEPTH:
    case P_LFOPITCH:
    case P_LFOFILT:
    case P_LFOAMP:
    case P_DLYFB:
    case P_DLYMIX:
    case P_NLEVEL:
      if (v == 0) snprintf(out, n, "OFF");
      else        snprintf(out, n, "%d%%", pct);
      break;
    case P_NTYPE:   snprintf(out, n, "%s", NOISE_TYPE_NAMES[v < 4 ? v : 0]); break;
    case P_NCOLOR:  snprintf(out, n, "%s", v < 43 ? "DARK" : (v < 86 ? "WARM" : "BRIGHT")); break;
    case P_NDECAY:
      if (v == 0) snprintf(out, n, "HOLD");
      else        fmtTime(ccToTime(f, 0.005f, 2.0f), out, n);
      break;
    case P_NMUTE:   snprintf(out, n, "%s", v ? "MUTED" : "ON"); break;
    case P_CUTOFF: {
      float hz = 20.0f * powf(1000.0f, f);
      if (hz >= 1000.0f) snprintf(out, n, "%.1fk", hz / 1000.0f);
      else               snprintf(out, n, "%dHz", (int)hz);
    } break;
    case P_SLOPE:   snprintf(out, n, "%s", (v == 0) ? "12dB" : (v == 1) ? "24dB" : (v == 2) ? "POLI LP" : "POLI BP"); break;
    case P_ATTACK:  fmtTime(ccToTime(f, 0.001f, 3.0f), out, n); break;
    case P_DECAY:   fmtTime(ccToTime(f, 0.005f, 4.0f), out, n); break;
    case P_RELEASE: fmtTime(ccToTime(f, 0.005f, 5.0f), out, n); break;
    case P_LFOWAVE: snprintf(out, n, "%s", LFO_WAVE_NAMES[v < 5 ? v : 0]); break;
     case P_LFORATE:
      if (pv[P_LFODIV]) snprintf(out, n, "SYNC");
      else              fmtHz(lfoRateHz(f), out, n);
      break;
    case P_LFODIV:  snprintf(out, n, "%s", LFO_DIV_NAMES[v < 10 ? v : 0]); break;
    case P_CHORATE:
      if (v == 0) snprintf(out, n, "AUTO");
      else        snprintf(out, n, "%.2fHz", choRateHz(v));
      break;
    case P_LFOSYNC: snprintf(out, n, "%s", v ? "KEY" : "FREE"); break;
    case P_CHORUS:  snprintf(out, n, "%s", CHORUS_NAMES[v < 4 ? v : 0]); break;
    case P_DLYTYPE: snprintf(out, n, "%s", DELAY_NAMES[v < 4 ? v : 0]); break;
     case P_DLYTIME:
      if (pv[P_DLYSYNC]) snprintf(out, n, "SYNC");
      else               snprintf(out, n, "%dms", (int)(delaySecs(f) * 1000.0f + 0.5f));
      break;
    case P_DLYSYNC: snprintf(out, n, "%s", DSYNC_NAMES[v < 6 ? v : 0]); break;
    case P_M1SRC: case P_M2SRC: snprintf(out, n, "%s", MOD_SRC_NAMES[v < 7 ? v : 0]); break;
    case P_M1DST: case P_M2DST: snprintf(out, n, "%s", MOD_DST_NAMES[v < 11 ? v : 0]); break;
    case P_SYNC:   snprintf(out, n, "%s", v ? "ON" : "OFF"); break;
    case P_XMOD:
      if (v == 0) snprintf(out, n, "OFF");
      else        snprintf(out, n, "%d%%", pct);
      break;
    case P_WHLVIB: snprintf(out, n, "%s", (v == 1) ? "ON" : (v == 2) ? "OFF" : "AUTO"); break;
    case P_M1AMT: case P_M2AMT: snprintf(out, n, "%+d%%", ((int)v - 64) * 100 / 63); break;
    case P_GLIDE:
      if (v == 0) snprintf(out, n, "OFF");
      else        fmtTime(ccToTime(f, 0.0f, 2.0f), out, n);
      break;
     case P_MODE:    snprintf(out, n, "%s", v ? "POLY" : "MONO"); break;
      case P_REBOOT:  snprintf(out, n, "B: go"); break;
    case P_RANDOM:  snprintf(out, n, "B: new"); break;
    case P_MTUNE:   snprintf(out, n, "%dHz", 430 + v); break;
    case P_MIDICH:
      if (v == 0) snprintf(out, n, "OMNI");
      else        snprintf(out, n, "%d", v);
      break;
    default: snprintf(out, n, "%d%%", pct); break;
  }
}

// ============================================================
//  MIDI (core 1) – notes go to the audio core via the event queue
// ============================================================
void handleCC(uint8_t cc, uint8_t val) {
  lastCcNum = cc;
  if (cc == 1)                 { pModWheel = val / 127.0f; return; }
  if (cc == 120 || cc == 123)  { pushEvent(EV_ALL_OFF, 0, 0); return; }
  if (cc == 126)               { pv[P_MODE] = 0; pushEvent(EV_ALL_OFF, 0, 0); return; }
  if (cc == 127)               { pv[P_MODE] = 1; pushEvent(EV_ALL_OFF, 0, 0); return; }
  for (uint8_t i = 0; i < NUM_PARAMS; i++) {
    if (PARAMS[i].cc == cc) {
      uint8_t mx = PARAMS[i].maxVal;
      // Full-range params take the CC directly; smaller ranges are scaled
      // (e.g. an on/off param switches at CC value 64)
      pv[i] = (mx == 127) ? val : (uint8_t)(((uint16_t)val * (mx + 1)) / 128);
      return;
    }
  }
}

void handleMessage(uint8_t status, uint8_t d1, uint8_t d2) {
  uint8_t ch  = status & 0x0F;
  uint8_t sel = pv[P_MIDICH];
  if (sel && ch != sel - 1) return;
  switch (status & 0xF0) {
    case 0x90:
      if (d2) pushEvent(EV_NOTE_ON, d1, d2);
      else    pushEvent(EV_NOTE_OFF, d1, 0);
      break;
    case 0x80: pushEvent(EV_NOTE_OFF, d1, 0); break;
    case 0xB0: handleCC(d1, d2); break;
    case 0xC0: if (d1 < NUM_PATCHES) loadPatch(d1); break;
    case 0xE0: pBend = ((int)((d2 << 7) | d1) - 8192) / 8192.0f * BEND_RANGE; break;
    case 0xD0: pAfter = d1 / 127.0f; break;
  }
}

uint8_t runStatus = 0, dataBuf[2], dataIdx = 0, dataNeed = 0;

void parseMidi(uint8_t b) {
  if (b >= 0xF8) {                             // real-time: clock, start, stop
       if (b == 0xF8) {
      uint32_t t  = micros();
      uint32_t dt = t - lastTickUs;
      lastTickUs  = t;
      if (dt > 2000 && dt < 125000) clockTickUs += ((float)dt - clockTickUs) * 0.05f;   // smoothed tempo
      clockTicks++;
      lastClockMs = millis();
    }
    else if (b == 0xFA) { clockBase = clockTicks; seqMidiStart = true; }
    else if (b == 0xFC) { seqMidiStop = true; }
    return;
  }
  lastMidiMs = millis();
  if (b >= 0xF0) { runStatus = 0; return; }    // sysex / common – ignore
  if (b & 0x80) {
    runStatus = b;
    dataIdx = 0;
    uint8_t t = b & 0xF0;
    dataNeed = (t == 0xC0 || t == 0xD0) ? 1 : 2;
    return;
  }
  if (!runStatus) return;
  dataBuf[dataIdx++] = b;
  if (dataIdx >= dataNeed) {
    handleMessage(runStatus, dataBuf[0], dataNeed == 2 ? dataBuf[1] : 0);
    dataIdx = 0;
  }
}

// ============================================================
//  DSP BUILDING BLOCKS
// ============================================================
static inline float polyBlep(float t, float dt) {
  if (t < dt)        { t /= dt;              return t + t - t * t - 1.0f; }
  if (t > 1.0f - dt) { t = (t - 1.0f) / dt;  return t * t + t + t + 1.0f; }
  return 0.0f;
}

// Band-limited oscillator: 0 SAW, 1 TRI, 2 SQR (50 %), 3 PULSE (width pw),
// 4 SAW+SQR, 5 SAW+TRI, 6 TRI+SQR, 7 SAW+PUL.
// The triangle is a gently-leaky integral of the band-limited square; when no
// wave needs the square, the triangle is kept in step from the phase instead.
struct Osc {
  float phase    = 0.0f;
  float triState = -1.0f;              // starts at the triangle's minimum (phase 0)

  inline float process(float dt, uint8_t wave, float pw) {
    // 50 % band-limited square – only when a wave needs it (SQR, TRI and mixes)
    float sq = 0.0f;
    bool needSq = (wave == 1 || wave == 2 || wave == 4 || wave == 5 || wave == 6);
    if (needSq) {
      sq = (phase < 0.5f) ? 1.0f : -1.0f;
      sq += polyBlep(phase, dt);
      float p2 = phase + 0.5f;
      if (p2 >= 1.0f) p2 -= 1.0f;
      sq -= polyBlep(p2, dt);
      triState = triState * 0.9999f + 4.0f * dt * sq;
    } else {
      // keep the triangle in step, so switching to TRI later is seamless
      triState = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
    }

    // Only compute the shapes the selected wave needs
    float saw = 0.0f, pul = 0.0f;
    if (wave == 0 || wave == 4 || wave == 5 || wave == 7)
      saw = 2.0f * phase - 1.0f - polyBlep(phase, dt);
    if (wave == 3 || wave == 7) {
      pul = (phase < pw) ? 1.0f : -1.0f;
      pul += polyBlep(phase, dt);                           // rising edge at 0
      float t2 = phase - pw;
      if (t2 < 0.0f) t2 += 1.0f;
      pul -= polyBlep(t2, dt);                              // falling edge at pw
      pul -= (2.0f * pw - 1.0f);                            // remove DC offset
    }

    float out;
    switch (wave) {
      case 0:  out = saw;                        break;     // SAW
      case 1:  out = triState;                   break;     // TRI
      case 2:  out = sq;                         break;     // SQR
      case 3:  out = pul;                        break;     // PULSE
      case 4:  out = 0.5f * (saw + sq);          break;     // SAW+SQR
      case 5:  out = 0.5f * (saw + triState);    break;     // SAW+TRI
      case 6:  out = 0.5f * (triState + sq);     break;     // TRI+SQR
      default: out = 0.5f * (saw + pul);         break;     // SAW+PUL
    }

    phase += dt;
    if (phase >= 1.0f) phase -= 1.0f;
    return out;
  }
};

// Band-limited square one octave below osc1 (fed half of osc1's increment)
struct SubOsc {
  float phase = 0.0f;
  inline float process(float dt) {
    float sq = (phase < 0.5f) ? 1.0f : -1.0f;
    sq += polyBlep(phase, dt);
    float p2 = phase + 0.5f;
    if (p2 >= 1.0f) p2 -= 1.0f;
    sq -= polyBlep(p2, dt);
    phase += dt;
    if (phase >= 1.0f) phase -= 1.0f;
    return sq;
  }
  // Keep time without generating (Sub Level = 0), so switching on stays click-free
  inline float skip(float dt) {
    phase += dt;
    if (phase >= 1.0f) phase -= 1.0f;
    return 0.0f;
  }
};

// TPT state-variable low-pass. g = tan(pi*fc/fs) computed by the caller
// so two stages (24 dB mode) can share one coefficient calculation.
struct SVF {
  float ic1 = 0, ic2 = 0, a1 = 0, a2 = 0, a3 = 0;
  float bp = 0;                            // band-pass output of the last sample (POLI BP)
  void setGK(float g, float k) {
    a1 = 1.0f / (1.0f + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
  }
  inline float lp(float x) {
    float v3 = x - ic2;
    float v1 = a1 * ic1 + a2 * v3;
    float v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2.0f * v1 - ic1;
    ic2 = 2.0f * v2 - ic2;
    bp  = v1;
    return v2;
  }
};

// Stage names prefixed ENV_ – plain DEC clashes with Arduino's DEC constant
struct Env {
  enum State { ENV_IDLE, ENV_ATT, ENV_DEC, ENV_SUS, ENV_REL } st = ENV_IDLE;
  float level = 0.0f;
  float attInc = 0.01f, decCoef = 0.999f, relCoef = 0.999f, sus = 0.6f;

  void trigger() { st = ENV_ATT; }
  void release() { if (st != ENV_IDLE && st != ENV_REL) st = ENV_REL; }

  inline float process() {
    switch (st) {
      case ENV_ATT:
        level += attInc;
        if (level >= 1.0f) { level = 1.0f; st = ENV_DEC; }
        break;
      case ENV_DEC:
        level = sus + (level - sus) * decCoef;
        if (level - sus < 0.0005f) st = ENV_SUS;
        break;
      case ENV_SUS:
        level += (sus - level) * 0.001f;
        break;
      case ENV_REL:
        level *= relCoef;
        if (level < 0.00005f) { level = 0.0f; st = ENV_IDLE; }
        break;
      default: break;
    }
    return level;
  }
};

// Cubic soft clipper: smooth, with no division (much cheaper on the ESP32)
static inline float softClip(float x) {
  if (x >  1.5f) return  1.0f;
  if (x < -1.5f) return -1.0f;
  return x - 0.148148f * x * x * x;         // x - (4/27)x^3
}

// Fast white noise, -1..1 (separate generator from drift and S&H)
static uint32_t noiseState = 0x9E3779B9;
static inline float whiteNoise() {
  noiseState ^= noiseState << 13;
  noiseState ^= noiseState >> 17;
  noiseState ^= noiseState << 5;
  return (float)(int32_t)noiseState * (1.0f / 2147483648.0f);
}

// Fast random (audio core only) for analog drift and S&H
static uint32_t rngState = 0x6D616C69;
static inline float rand01() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (rngState >> 8) * (1.0f / 16777216.0f);
}

// ============================================================
//  LFO state (core 0 only)
// ============================================================
float lfoPh      = 0.0f;
float lfoSH      = 0.0f;      // current sample & hold value
float lfoAmpPrev = 1.0f;      // for smooth tremolo ramps across a block
float pwmPh      = 0.0f;      // dedicated PWM triangle LFO

void lfoRetrigger() {
  if (pv[P_LFODIV]) return;              // tempo-synced: the clock sets the phase, not the notes
  lfoPh = 0.0f;
  lfoSH = rand01() * 2.0f - 1.0f;
}

// ============================================================
//  FX state (core 0 only)
// ============================================================
static float choBuf[CHO_LEN];          // mono chorus input history
uint16_t choW  = 0;
float    choPh = 0.0f;
float    lfoLast = 0.0f;                  // last block's LFO value (mod matrix -> LFO rate)

int16_t* dlyL  = nullptr;              // stereo delay lines (allocated in setup)
int16_t* dlyR  = nullptr;
bool     dlyOK = false;
uint32_t dlyW  = 0;
float    dlyTimeSm = 0.25f * SR;       // smoothed delay time in samples
float    tapeLpL = 0.0f, tapeLpR = 0.0f;
float    wowPh   = 0.0f;

// Read the chorus line d samples behind the write position (linear interp.)
static inline float choRead(float d) {
  float rp = (float)choW - d;
  if (rp < 0.0f) rp += CHO_LEN;
  int   ip = (int)rp;                       // rp is never negative, so this is floor()
  int   i0 = ip & (CHO_LEN - 1);
  float fr = rp - (float)ip;
  int   i1 = (i0 + 1) & (CHO_LEN - 1);
  return choBuf[i0] + (choBuf[i1] - choBuf[i0]) * fr;
}

// Read a delay line d samples behind the write position (linear interp.)
static inline float dlyRead(const int16_t* buf, float d) {
  float rp = (float)dlyW - d;
  if (rp < 0.0f) rp += DLY_LEN;
  int   i0 = (int)rp;
  float fr = rp - (float)i0;
  int   i1 = i0 + 1;
  if (i1 >= DLY_LEN) i1 = 0;
  return (buf[i0] + (buf[i1] - buf[i0]) * fr) * (1.0f / 32767.0f);
}

// ============================================================
//  VOICES (core 0 only)
// ============================================================
struct Voice {
  Osc    o1, o2;
  SubOsc sub;
  SVF    filt, filt2;
  Env    env;
  float    curNote    = 60.0f;
  float    targetNote = 60.0f;
  float    vel        = 1.0f;
  float    d1 = 0.0f, d2 = 0.0f;       // current drift (cents), osc1 / osc2
  float    t1 = 0.0f, t2 = 0.0f;       // drift targets
  float    nLp = 0.0f, nEnv = 0.0f;    // noise colour filter, noise burst envelope
  float    pk0 = 0.0f, pk1 = 0.0f, pk2 = 0.0f;   // pink noise filter states
  float    brn = 0.0f;                 // brown noise state
  float    dVal = 0.0f;                // digital noise held value
  uint8_t  dCnt = 0;                   // digital noise hold counter
  uint8_t  note       = 255;
  bool     gate       = false;
  bool     accent     = false;         // sequencer accent: opens the filter further
  uint32_t age        = 0;

  bool active() const { return gate || env.st != Env::ENV_IDLE; }

  void start(uint8_t n, float velocity, uint32_t stamp) {
    note = n; targetNote = n; vel = velocity;
    gate = true; age = stamp;
    env.trigger();                       // restarts from current level: no click
    nEnv = 1.0f;                         // restart the noise burst
  }
};

Voice    voices[NUM_VOICES];
uint32_t voiceStamp = 0;
uint8_t  lastMode   = 255;
bool     noteAccent = false;           // set by the sequencer for accented steps

// Mono note stack (voice 0) – last-note priority, legato
#define STACK_MAX 16
uint8_t monoStack[STACK_MAX];
uint8_t monoLen = 0;

void monoRemove(uint8_t n) {
  for (uint8_t i = 0; i < monoLen; i++) {
    if (monoStack[i] == n) {
      for (uint8_t j = i; j < monoLen - 1; j++) monoStack[j] = monoStack[j + 1];
      monoLen--;
      return;
    }
  }
}

void monoNoteOn(uint8_t n, uint8_t velByte) {
  bool wasEmpty = (monoLen == 0);
  monoRemove(n);
  if (monoLen >= STACK_MAX) {
    for (uint8_t j = 0; j < STACK_MAX - 1; j++) monoStack[j] = monoStack[j + 1];
    monoLen--;
  }
  monoStack[monoLen++] = n;
  if (wasEmpty) {
    voices[0].start(n, velCurve(velByte), ++voiceStamp);
    if (pv[P_LFOSYNC]) lfoRetrigger();     // key sync: restart on each new phrase
  } else {                                // legato: new pitch, no retrigger
    voices[0].note = n;
    voices[0].targetNote = n;
  }
  voices[0].accent = noteAccent;
  gDisplayNote = n;
}

void monoNoteOff(uint8_t n) {
  monoRemove(n);
  if (monoLen == 0) {
    voices[0].gate = false;
  } else {
    uint8_t top = monoStack[monoLen - 1];
    voices[0].note = top;
    voices[0].targetNote = top;
    gDisplayNote = top;
  }
}

void polyNoteOn(uint8_t n, uint8_t velByte) {
  int idx = -1;
  // 1. same note already sounding -> retrigger that voice
  for (int i = 0; i < NUM_VOICES; i++)
    if (voices[i].note == n && voices[i].active()) { idx = i; break; }
  // 2. oldest idle voice
  if (idx < 0) {
    uint32_t best = 0xFFFFFFFF;
    for (int i = 0; i < NUM_VOICES; i++)
      if (!voices[i].active() && voices[i].age < best) { best = voices[i].age; idx = i; }
  }
  // 3. oldest voice already releasing
  if (idx < 0) {
    uint32_t best = 0xFFFFFFFF;
    for (int i = 0; i < NUM_VOICES; i++)
      if (!voices[i].gate && voices[i].age < best) { best = voices[i].age; idx = i; }
  }
  // 4. oldest held voice (steal)
  if (idx < 0) {
    uint32_t best = 0xFFFFFFFF;
    for (int i = 0; i < NUM_VOICES; i++)
      if (voices[i].age < best) { best = voices[i].age; idx = i; }
  }
  voices[idx].start(n, velCurve(velByte), ++voiceStamp);
  voices[idx].accent = noteAccent;
  if (pv[P_LFOSYNC]) lfoRetrigger();       // key sync: restart on every note
  gDisplayNote = n;
}

void polyNoteOff(uint8_t n) {
  for (int i = 0; i < NUM_VOICES; i++)
    if (voices[i].gate && voices[i].note == n) voices[i].gate = false;
}

void allVoicesOff() {
  monoLen = 0;
  for (int i = 0; i < NUM_VOICES; i++) voices[i].gate = false;
}

// Play / release a note in the current mode (used by MIDI and the sequencer)
void synthOn(uint8_t n, uint8_t velByte) {
  if (pv[P_MODE]) polyNoteOn(n, velByte); else monoNoteOn(n, velByte);
}

void synthOff(uint8_t n) {
  if (pv[P_MODE]) polyNoteOff(n); else monoNoteOff(n);
}

// ============================================================
//  SEQUENCER ENGINE (core 0)
// ============================================================
uint8_t  seqPos       = 0;                 // next step to play
uint8_t  seqCur       = 0;                 // step just played
uint32_t seqSmp       = 0;                 // samples into the current step (internal tempo)
uint32_t seqTickAcc   = 0;                 // MIDI clock ticks into the current step
bool     seqFollow    = false;             // following MIDI clock
bool     seqWaitTick  = false;             // MIDI Start received: wait for the first tick
uint32_t seqWaitMs    = 0;
uint8_t  seqSounding  = 0;                 // note the sequencer is holding (0 = none)
uint8_t  seqLastFlags = 0;                 // flags of the step that note came from
bool     seqGateOff   = false;             // release the note half way through the step
bool     seqSlideGlide = false;            // slide in progress: glide even if Glide is off
int8_t   seqSkip      = -1;                // step just recorded ahead: don't play it twice
uint8_t  recHeld      = 0;                 // keys held while recording
int8_t   recLastStep  = -1;

static inline uint32_t seqStepSamples() {
  return (uint32_t)(SR * 60.0f / (float)seq.bpm / 4.0f);
}

bool seqFirst = false;                             // first step after starting

bool patEmpty(uint8_t i) {
  for (int s = 0; s < SEQ_STEPS; s++) if (seq.pat[i].st[s].note) return false;
  return true;
}

// Next pattern in the chain (P1..P(CHAIN_END+1)), skipping empty ones
uint8_t chainNext(uint8_t p) {
  uint8_t n = CHAIN_END + 1;
  for (uint8_t k = 1; k <= n; k++) {
    uint8_t q = (p >= n) ? (uint8_t)(k - 1) : (uint8_t)((p + k) % n);
    if (!patEmpty(q)) return q;
  }
  return p;
}

void seqDoStep() {
  uint8_t len = seq.pat[playPat].len;
  if (seqPos >= len) seqPos = 0;
  if (seqPos == 0 && !seqFirst) {                  // top of the pattern
    if (queuedPat != playPat)  playPat = queuedPat;                    // chosen with PAT
    else if (CHAIN_END)        playPat = queuedPat = chainNext(playPat); // chain
  }
  const MPat& p = seq.pat[playPat];
  if (seqPos >= p.len) seqPos = 0;

  if (seqSkip == (int8_t)seqPos) {                  // just recorded: already sounding live
    seqSkip = -1;
    if (seqSounding) { synthOff(seqSounding); seqSounding = 0; }
    seqLastFlags = 0; seqGateOff = false; seqSlideGlide = false;
  } else {
    MStep s = p.st[seqPos];
    bool slideIn = seqSounding && (seqLastFlags & ST_SLD);
    if (s.note) {
      noteAccent = (s.flags & ST_ACC) != 0;
      uint8_t vel = noteAccent ? 127 : 90;
      if (slideIn) {                                // legato: no retrigger, glide to the note
        seqSlideGlide = true;
        uint8_t old = seqSounding;
        synthOn(s.note, vel);
        if (old != s.note) synthOff(old);
      } else {
        seqSlideGlide = false;
        if (seqSounding) synthOff(seqSounding);
        synthOn(s.note, vel);
      }
      noteAccent   = false;
      seqSounding  = s.note;
      seqLastFlags = s.flags;
      seqGateOff   = !(s.flags & ST_SLD);           // slide steps hold into the next step
    } else {                                        // rest
      if (seqSounding) synthOff(seqSounding);
      seqSounding = 0; seqLastFlags = 0; seqGateOff = false; seqSlideGlide = false;
    }
  }
  seqCur  = seqPos;
  gSeqPos = seqPos;
  seqPos++;
  seqFirst = false;
}

void seqStart() {
  playPat = queuedPat = editPat;
  seqRunning = true;
  seqPos = 0; seqCur = 0; seqTickAcc = 0;
  seqSmp = seqStepSamples();                        // first step plays straight away
  seqSounding = 0; seqLastFlags = 0; seqGateOff = false; seqSlideGlide = false;
  seqSkip = -1; recHeld = 0; recLastStep = -1;
  seqFirst = true;
}

void seqStop() {
  seqRunning = false;
  if (seqSounding) synthOff(seqSounding);
  seqSounding = 0; seqGateOff = false; seqSlideGlide = false;
}

// Record a played note into the playing pattern, snapped to the nearest step
void seqRecordOn(uint8_t n, uint8_t vel) {
  MPat& p = seq.pat[playPat];
  uint8_t len = p.len;
  float ph = seqFollow ? (float)seqTickAcc / 6.0f : (float)seqSmp / (float)seqStepSamples();
  bool fwd = (ph >= 0.5f);
  uint8_t st = fwd ? (uint8_t)((seqCur + 1) % len) : seqCur;
  if (recHeld && recLastStep >= 0 && recLastStep != (int8_t)st)
    p.st[recLastStep].flags |= ST_SLD;              // played legato: slide into this note
  p.st[st].note  = n;
  p.st[st].flags = (vel >= 100) ? ST_ACC : 0;       // hard hits become accents
  if (fwd) seqSkip = (int8_t)st;
  recLastStep = (int8_t)st;
  recHeld++;
  seqDirty = true;
}

void seqRecordOff() {
  if (recHeld) recHeld--;
}

// Block-rate sequencer housekeeping: transport, timing, gate lengths
void seqBlock(uint32_t newTicks, bool clockLive) {
  if (seqToggleReq) { seqToggleReq = false; seqWaitTick = false; if (seqRunning) seqStop(); else seqStart(); }
  if (seqMidiStart) {
    seqMidiStart = false;
    // MIDI Start with a chain on: always begin at the top of the chain (P1, or the first
    // pattern with notes), so every Mindburner sequencer starts lined up
    if (CHAIN_END) editPat = patEmpty(0) ? chainNext(0) : 0;
    seqStart(); seqWaitTick = true; seqWaitMs = millis();
  }
  if (seqMidiStop)  { seqMidiStop = false; seqWaitTick = false; if (seqRunning) seqStop(); }
  if (!seqRunning) return;

  if (seqWaitTick) {                                // MIDI Start: step 1 on the first clock tick
    if (newTicks) {
      seqWaitTick = false;
      seqFollow = true;
      seqDoStep();
      seqTickAcc = newTicks - 1;
      while (seqTickAcc >= 6) { seqTickAcc -= 6; seqDoStep(); }
      return;
    }
    if (millis() - seqWaitMs < 250) return;
    seqWaitTick = false;                            // no clock arrived: internal tempo
    seqSmp = seqStepSamples();
  }

  seqFollow = clockLive;
  if (seqFollow) {                                  // MIDI clock: 6 ticks per 16th
    seqTickAcc += newTicks;
    while (seqTickAcc >= 6) { seqTickAcc -= 6; seqDoStep(); }
    seqSmp = seqTickAcc * seqStepSamples() / 6;     // keep the internal counter in step
  } else {                                          // internal tempo
    uint32_t len = seqStepSamples();
    seqSmp += BLOCK_SIZE;
    while (seqSmp >= len) { seqSmp -= len; seqDoStep(); }
  }

  if (seqGateOff && seqSounding) {                  // normal steps: half-step gate
    bool half = seqFollow ? (seqTickAcc >= 3) : (seqSmp >= seqStepSamples() / 2);
    if (half) { synthOff(seqSounding); seqSounding = 0; seqGateOff = false; }
  }
}

// Drain the event queue at the start of each audio block
void processEvents() {
  uint8_t mode = pv[P_MODE];
  if (mode != lastMode) {                 // mode switched (UI, CC or patch load)
    allVoicesOff();
    seqSounding = 0;
    lastMode = mode;
  }
  while (evTail != evHead) {
    __sync_synchronize();
    uint8_t t = evq[evTail].type;
    uint8_t n = evq[evTail].note;
    uint8_t v = evq[evTail].vel;
    evTail = (evTail + 1) & (EVQ_SIZE - 1);
    switch (t) {
      case EV_NOTE_ON:
        if (mode) polyNoteOn(n, v); else monoNoteOn(n, v);
        if (seqRec && seqRunning && !seqWaitTick) seqRecordOn(n, v);
        break;
      case EV_NOTE_OFF:
        if (mode) polyNoteOff(n); else monoNoteOff(n);
        if (seqRunning) seqRecordOff();
        break;
      case EV_ALL_OFF:
        allVoicesOff();
        seqSounding = 0;
        break;
    }
  }
}

// ============================================================
//  RENDER (core 0) – runs from internal RAM for speed
// ============================================================
float smoothCut = 0.55f;
float vibPhase  = 0.0f;
uint32_t lastTickSeen = 0;
static float mixBuf[BLOCK_SIZE];

void IRAM_ATTR renderBlock(int16_t *out) {
  // MIDI clock for this block (read the clock time before millis(), so the
  // other core updating it can never make the age wrap round to a huge number)
  uint32_t ct = clockTicks;
  uint32_t newTicks = ct - lastTickSeen;
  lastTickSeen = ct;
  unsigned long lastClk = lastClockMs;
  bool clockLive = (long)(millis() - lastClk) < 500;

  seqBlock(newTicks, clockLive);
  processEvents();

  // ── Shared control-rate values (once per block) ──
  float attInc  = 1.0f / (ccToTime(pf(P_ATTACK), 0.001f, 3.0f) * SR);
  float decCoef = expf(-5.0f / (ccToTime(pf(P_DECAY),   0.005f, 4.0f) * SR));
  float relCoef = expf(-5.0f / (ccToTime(pf(P_RELEASE), 0.005f, 5.0f) * SR));
  float sus     = pf(P_SUSTAIN);

  float glideT    = (pv[P_GLIDE] == 0) ? 0.0f : ccToTime(pf(P_GLIDE), 0.0f, 2.0f);
  float glideCoef = (glideT < 0.002f) ? 0.0f : expf(-5.0f * BLOCK_SIZE / (glideT * SR));
  if (glideCoef == 0.0f && seqSlideGlide)            // sequencer slide: 60 ms glide
    glideCoef = expf(-5.0f * BLOCK_SIZE / (0.06f * SR));

  // Mod wheel vibrato (independent of the main LFO)
  vibPhase += 5.5f * BLOCK_SIZE * INV_SR;
  if (vibPhase >= 1.0f) vibPhase -= 1.0f;
  // Wheel vibrato: AUTO = on unless the wheel is a mod matrix source, ON, OFF
  bool wheelRouted = (pv[P_M1SRC] == 1) || (pv[P_M2SRC] == 1);
  bool vibOn = (pv[P_WHLVIB] == 1) || (pv[P_WHLVIB] == 0 && !wheelRouted);
  float vib  = vibOn ? sinf(2.0f * PI_F * vibPhase) * pModWheel * 0.5f : 0.0f;
  float bend = pBend;

  // Main LFO – free-running, or locked to the tempo (MIDI clock, else sequencer BPM)
  uint8_t lDiv = (pv[P_LFODIV] < 10) ? pv[P_LFODIV] : 0;
  if (lDiv) {
    uint16_t perT  = LFO_DIV_TICKS[lDiv];
    float    tickS = clockLive ? clockTickUs * 1.0e-6f : 60.0f / ((float)seq.bpm * 24.0f);
    lfoPh += BLOCK_SIZE * INV_SR / (tickS * (float)perT);
    if (clockLive) {
      bool restart = false;                          // a new cycle starts on the clock grid
      for (uint32_t k = 0; k < newTicks; k++)
        if (((ct - newTicks + k - clockBase) % perT) == 0) restart = true;
      if (restart)              { lfoPh = 0.0f; lfoSH = rand01() * 2.0f - 1.0f; }
      else if (lfoPh > 0.999f)  lfoPh = 0.999f;      // wait for the clock to start the next cycle
    } else if (lfoPh >= 1.0f) {
      lfoPh -= 1.0f; lfoSH = rand01() * 2.0f - 1.0f;
    }
  } else {
    float lrMod = 0.0f;                              // mod matrix -> LFO rate (global sources only)
    for (int m = 0; m < 2; m++)
      if (pv[P_M1DST + m * 3] == 6) {
        uint8_t sr = pv[P_M1SRC + m * 3];
        float sv = (sr == 1) ? pModWheel : (sr == 3) ? pAfter : (sr == 4) ? lfoLast : 0.0f;
        lrMod += sv * ((int)pv[P_M1AMT + m * 3] - 64) / 63.0f * 3.0f;      // up to ±3 octaves
      }
    lfoPh += lfoRateHz(pf(P_LFORATE)) * fastExp2(lrMod) * BLOCK_SIZE * INV_SR;
    if (lfoPh >= 1.0f) { lfoPh -= 1.0f; lfoSH = rand01() * 2.0f - 1.0f; }
  }
  float lfo;
  switch (pv[P_LFOWAVE]) {
    case 0:  lfo = sinf(2.0f * PI_F * lfoPh); break;                             // sine
    case 1:  lfo = (lfoPh < 0.5f) ? (4.0f * lfoPh - 1.0f) : (3.0f - 4.0f * lfoPh); break; // triangle
    case 2:  lfo = 2.0f * lfoPh - 1.0f; break;                                   // saw (rising)
    case 3:  lfo = (lfoPh < 0.5f) ? 1.0f : -1.0f; break;                         // square
    default: lfo = lfoSH; break;                                                 // sample & hold
  }
  lfoLast = lfo;
  // ── Mod matrix: global sources now, per-voice sources inside the voice loop ──
  uint8_t mSrc[2] = { pv[P_M1SRC], pv[P_M2SRC] };
  uint8_t mDst[2] = { pv[P_M1DST], pv[P_M2DST] };
  float   mAmt[2] = { ((int)pv[P_M1AMT] - 64) / 63.0f, ((int)pv[P_M2AMT] - 64) / 63.0f };
  float   mGlob[2];
  for (int m = 0; m < 2; m++) {
    if (mAmt[m] > 1.0f) mAmt[m] = 1.0f;
    mGlob[m] = (mSrc[m] == 1) ? pModWheel : (mSrc[m] == 3) ? pAfter : (mSrc[m] == 4) ? lfo : 0.0f;
  }

  float lpd       = pf(P_LFOPITCH);
  float lfoPitch  = lfo * lpd * lpd * 12.0f;                 // up to ±12 semitones
  float lfoFilt   = lfo * pf(P_LFOFILT) * 4.0f;              // up to ±4 octaves
  float ampTarget = 1.0f - pf(P_LFOAMP) * 0.5f * (1.0f - lfo);

  // PWM: dedicated triangle LFO sweeping the pulse width
  pwmPh += lfoRateHz(pf(P_PWMRATE)) * BLOCK_SIZE * INV_SR;
  if (pwmPh >= 1.0f) pwmPh -= 1.0f;
  float pwmTri = (pwmPh < 0.5f) ? (4.0f * pwmPh - 1.0f) : (3.0f - 4.0f * pwmPh);
  float pw     = pulseWidth(pf(P_PW)) + pwmTri * pf(P_PWMDEPTH) * 0.45f;
  if (pw < 0.05f) pw = 0.05f;
  if (pw > 0.95f) pw = 0.95f;

  // Oscillators
  uint8_t w1     = pv[P_WAVE1];
  uint8_t w2     = pv[P_WAVE2];
  float tune2    = (float)((int)pv[P_TUNE2] - 12) + pf(P_FINE2) * 0.5f;   // semitones (fine: 0..50 c)
  float a4Hz     = 430.0f + (float)pv[P_MTUNE];                           // master tune (A4 pitch)
  float mix      = pf(P_MIX);
  float m1       = 1.0f - mix, m2 = mix;
  float subLvl   = pf(P_SUB);
  float oscScale = 1.0f - 0.2f * subLvl;
  float subGain  = 0.4f * subLvl;
  float driftC   = pf(P_DRIFT) * DRIFT_MAX_CENTS;
  bool  syncOn   = (pv[P_SYNC] != 0);

  // Noise: every type is scaled to the same basic level, then coloured by a
  // one-pole low-pass. Flat types (white, digital) get full level compensation
  // for darker colours, pink gets partial, brown (already dark) gets none.
  bool    oscMute  = (pv[P_NMUTE] != 0);
  bool    useNoise = (pv[P_NLEVEL] != 0);
  uint8_t nType    = pv[P_NTYPE];
  float   nLvl     = pf(P_NLEVEL);
  float   nCol     = pf(P_NCOLOR);
  float   nCoef    = 0.02f + nCol * nCol * 0.98f;
  float   lpComp   = 1.0f / sqrtf(nCoef / (2.0f - nCoef));
  float   comp     = (nType == 1) ? sqrtf(lpComp) : (nType == 2) ? 1.0f : lpComp;
  float   nGain    = nLvl * nLvl * (0.35f / 0.577f) * comp;
  bool    nBurst   = (pv[P_NDECAY] != 0);
  float   nDecCoef = nBurst ? expf(-5.0f / (ccToTime(pf(P_NDECAY), 0.005f, 2.0f) * SR)) : 1.0f;

  // Filter
  smoothCut += (pf(P_CUTOFF) - smoothCut) * 0.2f;
  float baseCut = 20.0f * powf(1000.0f, smoothCut);
  float envAmt  = pf(P_ENVAMT) * 6.0f;
  uint8_t fMode = pv[P_SLOPE] & 3;                   // 12dB / 24dB / POLI LP / POLI BP
  bool  slope24 = (fMode == 1);
  bool  poli    = (fMode >= 2);                       // Polivoks-style: overdriven, saturating 12 dB filter
  float k       = poli ? (2.0f - 2.05f * pf(P_RESO))  // can go past self-oscillation – the clipping holds it
                       : (2.0f - 1.95f * pf(P_RESO));
  float poliIn  = 2.2f + pf(P_DRIVE) * 4.0f;          // POLI always hits the filter hard
  bool  driveOn = (pv[P_DRIVE] != 0);
  float dg      = 1.0f + pf(P_DRIVE) * 7.0f;          // drive gain 1..8
  float dn      = 1.0f / softClip(dg);                // keep full-scale level steady

  bool  poly    = (pv[P_MODE] != 0);

  memset(mixBuf, 0, sizeof(mixBuf));
  bool anyGate = false;
  uint32_t newestAge = 0; float nVel = 0.0f, nEnv = 0.0f, nKey = 0.0f;   // newest voice (mod -> effects)

  // ── Voices ──
  for (int vi = 0; vi < NUM_VOICES; vi++) {
    Voice& v = voices[vi];
    v.env.attInc = attInc; v.env.decCoef = decCoef;
    v.env.relCoef = relCoef; v.env.sus = sus;
    if (v.gate) anyGate = true;
    else        v.env.release();
    if (!v.active()) continue;                     // idle voices cost nothing

    if (glideCoef == 0.0f) v.curNote = v.targetNote;
    else v.curNote = v.targetNote + (v.curNote - v.targetNote) * glideCoef;

    // Analog drift: each oscillator wanders slowly towards random targets
    if (driftC > 0.0f) {
      if (rand01() < 0.002f) v.t1 = (rand01() * 2.0f - 1.0f) * driftC;
      if (rand01() < 0.002f) v.t2 = (rand01() * 2.0f - 1.0f) * driftC;
      v.d1 += (v.t1 - v.d1) * 0.005f;
      v.d2 += (v.t2 - v.d2) * 0.005f;
    } else {
      v.d1 = v.d2 = v.t1 = v.t2 = 0.0f;
    }

    // Mod matrix for this voice: one value per slot, then summed per destination
    float mCut = 0.0f, mPeak = 0.0f, mPit = 0.0f, mO2 = 0.0f, mPw = 0.0f, mMix = 0.0f, mAmp = 0.0f, mXm = 0.0f;
    float mEnvCutAmt = 0.0f;
    for (int m = 0; m < 2; m++) {
      if (!mSrc[m] || mAmt[m] == 0.0f) continue;
      float sv = mGlob[m];
      if      (mSrc[m] == 2) sv = v.vel;
      else if (mSrc[m] == 5) sv = v.env.level;
      else if (mSrc[m] == 6) sv = (v.curNote - 60.0f) * (1.0f / 36.0f);
      float x = sv * mAmt[m];
      switch (mDst[m]) {
        case 0: if (mSrc[m] == 5) mEnvCutAmt += mAmt[m] * 4.0f; else mCut += x * 4.0f; break;  // ±4 octaves
        case 1: mPeak += x; break;
        case 2: mPit  += x * 12.0f; break;                     // ±12 semitones
        case 3: mO2   += x * 12.0f; break;
        case 4: mPw   += x * 0.45f; break;
        case 5: mMix  += x; break;
        case 7: mAmp  += x; break;
        case 10: mXm  += x; break;                     // X-MOD amount
        default: break;                                        // LFO RATE: handled above
      }
    }

    float note = v.curNote + bend + vib + lfoPitch + mPit;
    float n1   = note + v.d1 * 0.01f;
    float n2   = note + tune2 + mO2 + v.d2 * 0.01f;
    float dt1  = a4Hz * fastExp2((n1 - 69.0f) * (1.0f / 12.0f)) * INV_SR;
    float dt2  = a4Hz * fastExp2((n2 - 69.0f) * (1.0f / 12.0f)) * INV_SR;
    bool  useSub = (subGain > 0.0f);
    if (dt1 > 0.45f) dt1 = 0.45f;
    if (dt2 > 0.45f) dt2 = 0.45f;
    float dtSub = dt1 * 0.5f;
    if (v.age >= newestAge) {
      newestAge = v.age; nVel = v.vel; nEnv = v.env.level; nKey = (v.curNote - 60.0f) * (1.0f / 36.0f);
    }
    float xmv = pf(P_XMOD) + mXm;                    // mod matrix -> X-MOD
    if (xmv < 0.0f) xmv = 0.0f;
    if (xmv > 1.0f) xmv = 1.0f;
    float xmAmtV = xmv * xmv * 3.0f;
    float vvel  = v.vel;
    float envA  = envAmt + (v.accent ? 1.5f : 0.0f) + mEnvCutAmt;  // accent / mod matrix ENV -> CUTOFF
    float kv    = k - mPeak * 1.9f;                       // mod matrix -> PEAK
    if (kv < (poli ? -0.05f : 0.05f)) kv = poli ? -0.05f : 0.05f;
    if (kv > 2.0f) kv = 2.0f;
    float pwv   = pw + mPw;                               // mod matrix -> PULSE W
    if (pwv < 0.05f) pwv = 0.05f;
    if (pwv > 0.95f) pwv = 0.95f;
    float m2v   = m2 + mMix;                              // mod matrix -> OSC MIX
    if (m2v < 0.0f) m2v = 0.0f;
    if (m2v > 1.0f) m2v = 1.0f;
    float m1v   = 1.0f - m2v;
    float ampv  = 1.0f + mAmp;                            // mod matrix -> VOLUME
    if (ampv < 0.0f) ampv = 0.0f;
    if (ampv > 2.0f) ampv = 2.0f;
    vvel *= ampv;

    for (int s0 = 0; s0 < BLOCK_SIZE; s0 += FILTER_SUB) {
      // Filter coefficients follow the envelope and LFO every 16 samples
      float fc = baseCut * fastExp2(envA * v.env.level + lfoFilt + mCut);
      if (fc > 18000.0f) fc = 18000.0f;
      if (fc < 10.0f)    fc = 10.0f;
      float g = fastTan(PI_F * fc * INV_SR);
      v.filt.setGK(g, kv);
      if (slope24) v.filt2.setGK(g, 1.414f);       // second stage: no extra peak

      for (int i = s0; i < s0 + FILTER_SUB; i++) {
        float s = 0.0f;
        if (!oscMute) {
          // Osc 2 runs first: it is the X-Mod and Sync source for osc 1
          float p2 = v.o2.phase;
          float o2 = v.o2.process(dt2, w2, pwv);
          if (syncOn && v.o2.phase < p2) {            // hard sync: osc 2 restarts osc 1
            float ph = v.o2.phase * (dt1 / dt2);
            v.o1.phase = ph - (float)(int)ph;
          }
          float d1x = dt1;
          if (xmAmtV > 0.0f) {                        // X-Mod: osc 2 bends osc 1's frequency
            d1x = dt1 * (1.0f + xmAmtV * o2);
            if (d1x < 0.0f) d1x = -d1x;
            if (d1x > 0.45f) d1x = 0.45f;
          }
          float o1 = v.o1.process(d1x, w1, pwv);
          float sb = useSub ? v.sub.process(dtSub) : v.sub.skip(dtSub);   // stays phase-locked
          s = (o1 * m1v + o2 * m2v) * oscScale + sb * subGain;
        }
        if (useNoise) {
          float w = whiteNoise();
          float nraw;
          switch (nType) {
            case 1:                                  // PINK (Paul Kellet's economy filter)
              v.pk0 = 0.99765f * v.pk0 + w * 0.0990460f;
              v.pk1 = 0.96300f * v.pk1 + w * 0.2965164f;
              v.pk2 = 0.57000f * v.pk2 + w * 1.0526913f;
              nraw  = (v.pk0 + v.pk1 + v.pk2 + w * 0.1848f) * 0.36f;
              break;
            case 2:                                  // BROWN (leaky random walk)
              v.brn = (v.brn + w * 0.02f) * 0.998f;
              nraw  = v.brn * 3.15f;
              break;
            case 3:                                  // DIGITAL (stepped 1-bit)
              if (++v.dCnt >= 4) { v.dCnt = 0; v.dVal = (w >= 0.0f) ? 0.577f : -0.577f; }
              nraw  = v.dVal;
              break;
            default:                                 // WHITE
              nraw  = w;
              break;
          }
          v.nLp += (nraw - v.nLp) * nCoef;          // colour
          float nz = v.nLp * nGain;
          if (nBurst) { nz *= v.nEnv; v.nEnv *= nDecCoef; }
          s += nz;
        }
        if (poli) {
          // POLI: hot input, then integrators that saturate like the Polivoks op-amps
          float y = v.filt.lp(softClip(s * poliIn));
          v.filt.ic1 = 1.5f * softClip(v.filt.ic1 * 0.6667f);
          v.filt.ic2 = 1.5f * softClip(v.filt.ic2 * 0.6667f);
          s = (fMode == 3) ? v.filt.bp * 2.6f : y * 1.3f;
        } else {
        if (driveOn) s = softClip(s * dg) * dn;    // pre-filter saturation
        s = v.filt.lp(s);
        }
        if (slope24) {
          if (driveOn) s = softClip(s);            // inter-stage warmth
          s = v.filt2.lp(s);
        }
        mixBuf[i] += s * v.env.process() * vvel;
      }
    }
  }
  gAnyGate = anyGate || seqRunning;

  // ── FX settings for this block ──
  uint8_t choMode   = pv[P_CHORUS];
  float   choRate   = (choMode == 1) ? 0.513f : (choMode == 2) ? 0.863f : 9.75f;   // Juno rates
  float   choDepth  = ((choMode == 3) ? 0.0003f : 0.00185f) * SR;                  // samples
  if (pv[P_CHORATE]) {                              // manual rate: slow = deep 70s swish
    choRate  = choRateHz(pv[P_CHORATE]);
    float d  = (choRate < 1.0f) ? 0.0032f : (choRate < 3.0f) ? 0.0022f : 0.0010f;
    choDepth = ((choMode == 3) ? d * 0.6f : d) * SR;
  }
  float   choCentre = 0.0035f * SR;

  // Mod matrix -> effects (per-voice sources follow the newest note)
  float fxDly = 0.0f, fxCho = 0.0f;
  for (int m = 0; m < 2; m++) {
    if (!mSrc[m] || (mDst[m] != 8 && mDst[m] != 9)) continue;
    float sv = mGlob[m];
    if      (mSrc[m] == 2) sv = nVel;
    else if (mSrc[m] == 5) sv = nEnv;
    else if (mSrc[m] == 6) sv = nKey;
    if (mDst[m] == 8) fxDly += sv * mAmt[m]; else fxCho += sv * mAmt[m];
  }
  if (fxCho != 0.0f) {
    choDepth *= (1.0f + fxCho);                    // CHO DEPTH: 0 .. 2x
    if (choDepth < 0.0f) choDepth = 0.0f;
    if (choDepth > 0.0033f * SR) choDepth = 0.0033f * SR;   // stays inside the chorus line
  }

  uint8_t dType   = dlyOK ? pv[P_DLYTYPE] : 0;
  float   dTarget;
  if (pv[P_DLYSYNC]) {                             // tempo-synced: MIDI clock, else sequencer BPM
    float quarter = clockLive ? clockTickUs * 24.0e-6f : 60.0f / (float)seq.bpm;
    float t = quarter * DSYNC_BEATS[pv[P_DLYSYNC] < 6 ? pv[P_DLYSYNC] : 1];
    while (t > 0.59f) t *= 0.5f;                   // longer than the 0.6 s line: half time, still on the beat
    dTarget = t * SR;
  } else {
    dTarget = delaySecs(pf(P_DLYTIME)) * SR;
  }
  float   fb      = pf(P_DLYFB) * 0.9f;
  float   dMix    = pf(P_DLYMIX) + fxDly;            // mod matrix -> DLY MIX
  if (dMix < 0.0f) dMix = 0.0f;
  if (dMix > 1.0f) dMix = 1.0f;
  if (dType == 0) dlyTimeSm = dTarget;             // snap when off: no swoop on enable

  // ── Mix out: tremolo -> chorus -> delay -> soft clip ──
  float gOut  = pf(P_VOLUME) * (poly ? POLY_GAIN : 1.0f);
  float aStep = (ampTarget - lfoAmpPrev) / (float)BLOCK_SIZE;
  float amp   = lfoAmpPrev;

  for (int i = 0; i < BLOCK_SIZE; i++) {
    amp += aStep;
    float s = mixBuf[i] * gOut * amp;              // mono synth signal

    // Chorus (Juno-style: left and right modulated in opposite directions)
    choBuf[choW] = s;
    choW = (choW + 1) & (CHO_LEN - 1);
    float L = s, R = s;
    if (choMode) {
      choPh += choRate * INV_SR;
      if (choPh >= 1.0f) choPh -= 1.0f;
      float tri = (choPh < 0.5f) ? (4.0f * choPh - 1.0f) : (3.0f - 4.0f * choPh);
      L = (s + choRead(choCentre + choDepth * tri)) * 0.7f;
      R = (s + choRead(choCentre - choDepth * tri)) * 0.7f;
    }

    // Delay
    if (dlyOK) {
      if (dType == 0) {
        dlyL[dlyW] = 0;                            // keep the lines flushed while off
        dlyR[dlyW] = 0;
      } else {
        dlyTimeSm += (dTarget - dlyTimeSm) * 0.0005f;   // glide time changes smoothly
        float d = dlyTimeSm;
        if (dType == 2) {                          // tape wow
          wowPh += 0.7f * INV_SR;
          if (wowPh >= 1.0f) wowPh -= 1.0f;
          float wt = (wowPh < 0.5f) ? (4.0f * wowPh - 1.0f) : (3.0f - 4.0f * wowPh);
          d += wt * 25.0f;
        }
        float eL = dlyRead(dlyL, d);
        float eR = dlyRead(dlyR, d);
        float inL, inR;
        if (dType == 1) {                          // CLEAN stereo
          inL = L + eL * fb;
          inR = R + eR * fb;
        } else if (dType == 2) {                   // TAPE: darker, saturating repeats
          tapeLpL += (eL - tapeLpL) * 0.3f;
          tapeLpR += (eR - tapeLpR) * 0.3f;
          inL = L + softClip(tapeLpL * fb);
          inR = R + softClip(tapeLpR * fb);
        } else {                                   // PING: bounce between sides
          inL = 0.5f * (L + R) + eR * fb;
          inR = eL * fb;
        }
        dlyL[dlyW] = (int16_t)(softClip(inL) * 32767.0f);
        dlyR[dlyW] = (int16_t)(softClip(inR) * 32767.0f);
        L += eL * dMix;
        R += eR * dMix;
      }
      if (++dlyW >= DLY_LEN) dlyW = 0;
    }

    out[2 * i]     = (int16_t)(softClip(L) * 30000.0f);
    out[2 * i + 1] = (int16_t)(softClip(R) * 30000.0f);
  }
  lfoAmpPrev = ampTarget;
}

void audioTask(void *) {
  static int16_t buf[BLOCK_SIZE * 2];
  size_t written;
  const uint32_t blockUs = (uint32_t)(1000000.0f * BLOCK_SIZE / SAMPLE_RATE);   // ~1451 us
  uint32_t worstUs = 0, blocks = 0;
  for (;;) {
    uint32_t t0 = micros();
    renderBlock(buf);
    uint32_t dt = micros() - t0;
    if (dt > worstUs) worstUs = dt;
    if (++blocks >= 690) {              // about once a second
      gCpuPct = worstUs * 100 / blockUs;
      worstUs = 0;
      blocks = 0;
    }
    i2s_write(I2S_NUM_0, buf, sizeof(buf), &written, portMAX_DELAY);
  }
}

void setupI2S() {
  i2s_config_t cfg = {};
  cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate          = SAMPLE_RATE;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count        = 6;
  cfg.dma_buf_len          = BLOCK_SIZE;
  cfg.use_apll             = false;
  cfg.tx_desc_auto_clear   = true;
  cfg.fixed_mclk           = 0;
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = I2S_BCK_PIN;
  pins.ws_io_num    = I2S_WS_PIN;
  pins.data_out_num = I2S_DATA_PIN;
  pins.data_in_num  = I2S_PIN_NO_CHANGE;
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// Allocate the delay lines from the heap (two ~53 KB blocks)
void setupFX() {
  dlyL = (int16_t*)malloc(DLY_LEN * sizeof(int16_t));
  dlyR = (int16_t*)malloc(DLY_LEN * sizeof(int16_t));
  if (dlyL && dlyR) {
    memset(dlyL, 0, DLY_LEN * sizeof(int16_t));
    memset(dlyR, 0, DLY_LEN * sizeof(int16_t));
    dlyOK = true;
  } else {                                         // not enough memory: delay disabled
    free(dlyL); free(dlyR);
    dlyL = dlyR = nullptr;
    dlyOK = false;
  }
  memset(choBuf, 0, sizeof(choBuf));
}

// ============================================================
//  UI STATE
// ============================================================
enum UiMode : uint8_t { UI_PARAMS, UI_PATCH, UI_SEQ };
enum ConfirmState : uint8_t { CONF_NONE, CONF_LOAD, CONF_SAVE, CONF_FACTORY };

UiMode       uiMode       = UI_PARAMS;
ConfirmState confirmState = CONF_NONE;
bool         confirmDirty = false;
bool         uiNeedsFull  = true;
uint8_t      curPage      = 0;
uint8_t      curRow       = 0;
uint8_t      selParam     = P_WAVE1;
uint8_t      patchSel     = 0;
bool         rebootConfirm = false;       // reboot yes/no box open
bool         rebootDirty   = false;
void sayByeAndRestart();                  // defined with the splash

// Sequencer screen cursor: 0-15 = steps, then the BPM PAT LEN REC CLR boxes
#define SEQ_CELLS  (SEQ_STEPS + 6)
uint8_t seqCursor = 0;
uint8_t lastNote  = 36;                    // note given to a step switched on
unsigned long lastSeqEditT = 0;

// Short message shown in the parameter page footer (e.g. "Loaded P3")
char          toastMsg[24] = "";
unsigned long toastT       = 0;

unsigned long patchEnterT = 0;            // when the patch page opened

void enterPatchMode() {
  uiMode = UI_PATCH;
  patchSel = currentPatch;
  confirmState = CONF_NONE;
  uiNeedsFull = true;
  patchEnterT = millis();
}

// Patch-page audition: the live sound is kept here while another slot is previewed
uint8_t previewBak[PATCH_PARAMS];
bool    previewing = false;

void endPreview() {
  if (!previewing) return;
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) pv[i] = previewBak[i];
  previewing = false;
  uiNeedsFull = true;
}

// The sound you had before the last load, so a load can be undone
uint8_t lastSound[PATCH_PARAMS];
bool    haveLast = false;

void exitPatchMode() {
  endPreview();                              // leaving without loading: original sound back
  uiMode = UI_PARAMS;
  confirmState = CONF_NONE;
  uiNeedsFull = true;
}

void nextPage() {
  curPage = (curPage + 1) % NUM_PAGES;
  curRow = 0;
  selParam = PAGES[curPage].ids[curRow];
  uiNeedsFull = true;
}

void prevPage() {
  curPage = (curPage + NUM_PAGES - 1) % NUM_PAGES;
  curRow = 0;
  selParam = PAGES[curPage].ids[curRow];
  uiNeedsFull = true;
}

// ============================================================
//  BUTTONS + ENCODER (core 1)
// ============================================================
void nudgeParam(uint8_t id, int delta) {
  int v = (int)pv[id] + delta;
  if (v < 0) v = 0;
  if (v > PARAMS[id].maxVal) v = PARAMS[id].maxVal;
  pv[id] = (uint8_t)v;
}

struct Button {
  uint8_t pin;
  bool stable = false, lastRaw = false;
  bool pressedEdge = false, releasedEdge = false;
  unsigned long rawChangeT = 0, downT = 0, lastHeld = 0;

  Button(uint8_t p) : pin(p) {}

  void update(unsigned long now) {
    pressedEdge = releasedEdge = false;
    bool raw = (digitalRead(pin) == LOW);
    if (raw != lastRaw) { lastRaw = raw; rawChangeT = now; }
    if (raw != stable && (now - rawChangeT) >= 20) {
      stable = raw;
      if (stable) { pressedEdge = true; downT = now; }
      else        { releasedEdge = true; lastHeld = now - downT; }
    }
  }
  unsigned long heldFor(unsigned long now) const { return stable ? (now - downT) : 0; }

  // Value step for B/C. Single step fires on release (so the B+C combo
  // never nudges values); holding repeats, accelerating after 1.5s.
  // A member function so the Arduino IDE's auto-prototypes can't break it.
  void step(int dir, unsigned long now, unsigned long& repT, bool& repeated) {
    if (pressedEdge) { repeated = false; repT = now; }
    if (stable) {
      unsigned long h = now - downT;
      if (h >= 400 && (now - repT) >= 50) {
        repT = now;
        repeated = true;
        int amt = (h > 1500 && PARAMS[selParam].maxVal > 32) ? 4 : 1;
        nudgeParam(selParam, dir * amt);
      }
    }
    if (releasedEdge && !repeated) nudgeParam(selParam, dir);
  }

  // A genuine tap: pressed after 'since' and held at least 30 ms (ignores contact bounce)
  bool tapped(unsigned long since) const {
    return releasedEdge && downT > since && lastHeld >= 30;
  }

  // Press, then auto-repeat while held (sequencer boxes)
  bool repeatFire(unsigned long now, unsigned long& repT) {
    if (pressedEdge) { repT = now; return true; }
    if (stable && (now - downT) >= 400 && (now - repT) >= 60) { repT = now; return true; }
    return false;
  }
};

Button btnA(BTN_A_PIN), btnB(BTN_B_PIN), btnC(BTN_C_PIN), btnD(BTN_D_PIN), btnE(ENC_BTN_PIN);

// A: tap returns +1, hold returns -1 (then repeats every repeatMs)
int8_t readANav(unsigned long now, unsigned long repeatMs) {
  static bool holdActive = false;
  static unsigned long repT = 0;
  if (btnA.pressedEdge) holdActive = false;
  if (btnA.stable && btnA.heldFor(now) >= 500 && (!holdActive || (now - repT) >= repeatMs)) {
    holdActive = true;
    repT = now;
    return -1;
  }
  if (btnA.releasedEdge && !holdActive) return +1;
  return 0;
}

// ── Random patch (VOICE page) ──
uint8_t undoBuf[PATCH_PARAMS];
bool    haveUndo = false;

static int rr(int lo, int hi) { return lo + (int)(esp_random() % (uint32_t)(hi - lo + 1)); }

// A random patch built like a real subtractive sound: bass, lead, pad or pluck
void randomPatch() {
  uint8_t p[PATCH_PARAMS];
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) { p[i] = pv[i]; undoBuf[i] = pv[i]; }
  haveUndo = true;

  enum { R_BASS, R_LEAD, R_PAD, R_PLUCK };
  uint8_t type = (uint8_t)rr(0, 3);
  static const uint8_t BASS_WAVES[4] = { 0, 2, 3, 4 };       // SAW SQR PULSE SAW+SQR

  // Oscillators
  p[P_WAVE1] = (type == R_BASS) ? BASS_WAVES[rr(0, 3)] : rr(0, 7);
  p[P_WAVE2] = (rr(0, 2) == 0) ? rr(0, 7) : p[P_WAVE1];
  static const uint8_t TUNES[5] = { 12, 12, 12, 19, 24 };   // unison (mostly), +7, +12
  p[P_TUNE2] = (type == R_BASS && rr(0, 3) == 0) ? 0 : TUNES[rr(0, 4)];   // bass: sometimes -12
  p[P_FINE2] = rr(8, 40);
  p[P_MIX]   = rr(30, 70);
  p[P_SUB]   = (type == R_BASS) ? rr(40, 100) : (rr(0, 3) == 0 ? rr(10, 40) : 0);
  p[P_DRIFT] = rr(10, 40);

  // Pulse width – only matters for PULSE waves
  p[P_PW]       = rr(0, 60);
  p[P_PWMDEPTH] = rr(0, 1) ? rr(20, 70) : 0;
  p[P_PWMRATE]  = rr(30, 70);

  // Noise – mostly off; pads sometimes get a breath of pink noise
  p[P_NLEVEL] = 0; p[P_NMUTE] = 0; p[P_NDECAY] = 0;
  if (type == R_PAD && rr(0, 2) == 0) { p[P_NLEVEL] = rr(20, 40); p[P_NTYPE] = 1; p[P_NCOLOR] = rr(60, 100); }

  // Filter and envelope by type
  switch (type) {
    case R_BASS:
      p[P_CUTOFF] = rr(25, 55);  p[P_RESO] = rr(0, 2) ? rr(20, 60) : rr(80, 105);   // sometimes acid
      p[P_ENVAMT] = rr(50, 100); p[P_DRIVE] = rr(10, 60); p[P_SLOPE] = rr(0, 2) ? 1 : 0;
      p[P_ATTACK] = rr(0, 3);    p[P_DECAY] = rr(30, 55); p[P_SUSTAIN] = rr(30, 90); p[P_RELEASE] = rr(15, 35);
      break;
    case R_LEAD:
      p[P_CUTOFF] = rr(50, 85);  p[P_RESO] = rr(20, 60);  p[P_ENVAMT] = rr(30, 70);
      p[P_DRIVE]  = rr(10, 60);  p[P_SLOPE] = rr(0, 1);
      p[P_ATTACK] = rr(0, 10);   p[P_DECAY] = rr(40, 70); p[P_SUSTAIN] = rr(70, 110); p[P_RELEASE] = rr(25, 45);
      break;
    case R_PAD:
      p[P_CUTOFF] = rr(45, 80);  p[P_RESO] = rr(10, 40);  p[P_ENVAMT] = rr(10, 40);
      p[P_DRIVE]  = rr(0, 20);   p[P_SLOPE] = 0;
      p[P_ATTACK] = rr(50, 90);  p[P_DECAY] = rr(60, 90); p[P_SUSTAIN] = rr(90, 120); p[P_RELEASE] = rr(60, 95);
      break;
    default:                                                  // PLUCK
      p[P_CUTOFF] = rr(30, 60);  p[P_RESO] = rr(30, 80);  p[P_ENVAMT] = rr(70, 110);
      p[P_DRIVE]  = rr(0, 40);   p[P_SLOPE] = rr(0, 1);
      p[P_ATTACK] = rr(0, 2);    p[P_DECAY] = rr(25, 50); p[P_SUSTAIN] = rr(0, 20); p[P_RELEASE] = rr(25, 50);
      break;
  }

  // LFO – leads may get vibrato, pads a slow filter sweep
  p[P_LFOPITCH] = 0; p[P_LFOFILT] = 0; p[P_LFOAMP] = 0;
  if (type == R_LEAD && rr(0, 1)) { p[P_LFOWAVE] = 0; p[P_LFORATE] = rr(70, 85); p[P_LFOPITCH] = rr(15, 30); }
  if (type == R_PAD  && rr(0, 1)) { p[P_LFOWAVE] = rr(0, 1); p[P_LFORATE] = rr(25, 50); p[P_LFOFILT] = rr(15, 40); }

  // Effects
  p[P_CHORUS]  = (type == R_PAD) ? rr(1, 3) : (rr(0, 2) == 0 ? rr(1, 3) : 0);
  p[P_CHORATE] = (type == R_PAD && rr(0, 1)) ? rr(15, 45) : 0;
  p[P_DLYTYPE] = rr(0, 1) ? rr(1, 3) : 0;
  p[P_DLYTIME] = rr(60, 100);
  p[P_DLYFB]   = rr(20, 50);
  p[P_DLYMIX]  = p[P_DLYTYPE] ? rr(15, 35) : 0;

  for (uint8_t i = 0; i < PATCH_PARAMS; i++) pv[i] = p[i];
}

void undoRandom() {
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) pv[i] = undoBuf[i];
  haveUndo = false;
}

void handleParamButtons(unsigned long now) {
  if (rebootConfirm) {                                // reboot box: B or push = yes, C or A = no
    readEncoder();                                    // ignore turns while the box is open
    if (btnB.releasedEdge || btnE.releasedEdge) sayByeAndRestart();
    else if (btnC.releasedEdge || btnA.releasedEdge) {
      rebootConfirm = false;
      uiNeedsFull = true;
    }
    return;
  }
  uint8_t cnt = PAGES[curPage].count;

  // A: tap = next parameter, hold = next page (repeats) – works without D or the encoder
  int8_t nav = readANav(now, 700);
  if (nav > 0)      curRow = (curRow + 1) % cnt;
  else if (nav < 0) nextPage();

  // Encoder push: tap = next parameter, hold = previous page (repeats)
  static bool eHold = false;
  static unsigned long eRep = 0;
  if (btnE.pressedEdge) eHold = false;
  if (btnE.stable && btnE.heldFor(now) >= 600 && (!eHold || (now - eRep) >= 700)) {
    eHold = true;
    eRep = now;
    prevPage();
  }
  if (btnE.releasedEdge && !eHold) curRow = (curRow + 1) % PAGES[curPage].count;

  selParam = PAGES[curPage].ids[curRow];

  if (selParam == P_REBOOT) {                         // B opens the reboot box
    readEncoder();
    if (btnB.releasedEdge) { rebootConfirm = true; rebootDirty = true; }
    return;
  }

  if (selParam == P_RANDOM) {                         // B = new random patch, C = undo
    readEncoder();
    if (btnB.releasedEdge) {
      randomPatch();
      snprintf(toastMsg, sizeof(toastMsg), "New patch! C = undo");
      toastT = now;
    } else if (btnC.releasedEdge && haveUndo) {
      undoRandom();
      snprintf(toastMsg, sizeof(toastMsg), "Undone");
      toastT = now;
    }
    return;
  }

  // Encoder turn: change value (fast turning steps by 4)
  int det = readEncoder();
  if (det) {
    static unsigned long lastEnc = 0;
    int amt = ((now - lastEnc) < 40 && PARAMS[selParam].maxVal > 32) ? 4 : 1;
    lastEnc = now;
    nudgeParam(selParam, det * amt);
  }

  // Two buttons held together are a shortcut (patch page), not an edit:
  // stop B/C editing and put back the value from before the first press
  static uint8_t snapId = 0, snapVal = 0;
  static bool    chordSeen = false;
  if ((btnB.pressedEdge && !btnC.stable) || (btnC.pressedEdge && !btnB.stable)) {
    snapId  = selParam;                      // value before this edit began
    snapVal = pv[selParam];
  }
  bool chord = (btnB.stable && btnC.stable) || (btnD.stable && (btnB.stable || btnC.stable));
  if (chord && !chordSeen) {
    chordSeen = true;
    pv[snapId] = snapVal;                    // undo anything the first key already changed
  }
  if (chordSeen) {
    if (!btnB.stable && !btnC.stable) chordSeen = false;   // wait until both are let go
    return;
  }

  static unsigned long bRepT = 0, cRepT = 0;
  static bool bRep = false, cRep = false;
  btnB.step(+1, now, bRepT, bRep);
  btnC.step(-1, now, cRepT, cRep);
}

void auditionSlot(uint8_t slot);             // defined after serviceAudition()
void recallMine();

void handlePatchButtons(unsigned long now) {
  // Only presses that started on this page count – not the keys used to open it
  bool bTap = btnB.tapped(patchEnterT);
  bool cTap = btnC.tapped(patchEnterT);
  bool eTap = btnE.tapped(patchEnterT) && btnE.lastHeld < 600;   // a hold is not a tap

  // ── Restore factory box: B or push = selected slot, A = all slots, C = cancel ──
  if (confirmState == CONF_FACTORY) {
    readEncoder();
    bool aTap = btnA.tapped(patchEnterT);
    if (bTap || eTap) {
      eeWritePatch(patchSel, FACTORY[patchSel % NUM_FACTORY]);
      EEPROM.commit();
      if (patchSel == currentPatch) loadPatch(patchSel);
      snprintf(toastMsg, sizeof(toastMsg), "P%d restored", patchSel + 1);
      toastT = now;
      exitPatchMode();
    } else if (aTap) {
      for (uint8_t s = 0; s < NUM_PATCHES; s++) eeWritePatch(s, FACTORY[s % NUM_FACTORY]);
      EEPROM.commit();
      loadPatch(currentPatch);
      snprintf(toastMsg, sizeof(toastMsg), "All patches restored");
      toastT = now;
      exitPatchMode();
    } else if (cTap) {
      confirmState = CONF_NONE;
      uiNeedsFull = true;
    }
    return;
  }

  // ── Confirmation box open: push or B = yes, C = no ──
  if (confirmState != CONF_NONE) {
    readEncoder();                           // ignore turns while the box is open
    if (bTap || eTap) {
      if (confirmState == CONF_LOAD) {
        for (uint8_t i = 0; i < PATCH_PARAMS; i++) lastSound[i] = pv[i];   // keep the sound being replaced
        haveLast = true;
        loadPatch(patchSel);
        snprintf(toastMsg, sizeof(toastMsg), "Loaded P%d", patchSel + 1);
      } else {
        savePatch(patchSel);
        snprintf(toastMsg, sizeof(toastMsg), "Saved to P%d", patchSel + 1);
      }
      toastT = now;
      exitPatchMode();                       // back to the parameters
    } else if (cTap) {
      confirmState = CONF_NONE;              // cancelled – redraw the grid
      uiNeedsFull = true;
    }
    return;
  }

  // ── Slot selection: encoder or A ──
  int8_t nav = readANav(now, 250);
  int det = readEncoder() + nav;
  if (det) patchSel = (uint8_t)(((int)patchSel + det % NUM_PATCHES + NUM_PATCHES) % NUM_PATCHES);

  // Hold the encoder: back to your own sound (or, after a load, the sound you had before)
  static bool eHoldDone = false;
  if (btnE.pressedEdge) eHoldDone = false;
  if (btnE.stable && !eHoldDone && btnE.downT > patchEnterT && btnE.heldFor(now) >= 600) {
    eHoldDone = true;
    recallMine();
  }

  if (eTap) auditionSlot(patchSel);                                          // push: hear it
  else if (bTap) { endPreview(); confirmState = CONF_LOAD; confirmDirty = true; }
  else if (cTap) { endPreview(); confirmState = CONF_SAVE; confirmDirty = true; }  // saves the original
}

// Play a note briefly while editing with the sequencer stopped
uint8_t       audNote  = 0;
unsigned long audOffT  = 0;

void audition(uint8_t n) {
  if (seqRunning) return;
  if (audNote) pushEvent(EV_NOTE_OFF, audNote, 0);
  pushEvent(EV_NOTE_ON, n, 100);
  audNote = n;
  audOffT = millis() + 180;
}

void serviceAudition(unsigned long now) {
  if (audNote && (long)(now - audOffT) >= 0) {
    pushEvent(EV_NOTE_OFF, audNote, 0);
    audNote = 0;
  }
}

// Patch page: switch to the selected slot's sound (keeping the original) and play C3
void auditionSlot(uint8_t slot) {
  if (!previewing) {
    for (uint8_t i = 0; i < PATCH_PARAMS; i++) previewBak[i] = pv[i];
    previewing = true;
    uiNeedsFull = true;                      // header shows PREVIEW
  }
  uint8_t p[PATCH_PARAMS];
  if (!eeReadPatch(slot, p)) memcpy(p, FACTORY[slot % NUM_FACTORY], PATCH_PARAMS);
  for (uint8_t i = 0; i < PATCH_PARAMS; i++) pv[i] = p[i];
  if (seqRunning) return;                    // the sequencer is already playing it
  if (audNote) pushEvent(EV_NOTE_OFF, audNote, 0);
  pushEvent(EV_NOTE_ON, 48, 100);
  audNote = 48;
  audOffT = millis() + 700;
}

// Encoder hold on the patch page: hear your own sound again
void recallMine() {
  if (previewing) {                          // auditioning: back to the sound you were working on
    endPreview();
  } else if (haveLast) {                     // after a load: swap with the sound you had before
    for (uint8_t i = 0; i < PATCH_PARAMS; i++) { uint8_t t = pv[i]; pv[i] = lastSound[i]; lastSound[i] = t; }
    uiNeedsFull = true;
  } else {
    return;
  }
  if (seqRunning) return;                    // the sequencer is already playing it
  if (audNote) pushEvent(EV_NOTE_OFF, audNote, 0);
  pushEvent(EV_NOTE_ON, 48, 100);
  audNote = 48;
  audOffT = millis() + 700;
}

void seqEdited(unsigned long now) {
  seqDirty = true;
  lastSeqEditT = now;
}

// Step on the cursor of the pattern being edited: plain > accent > slide > both
void cycleStepFlags(unsigned long now) {
  MStep& s = seq.pat[editPat].st[seqCursor];
  if (!s.note) return;
  s.flags = (uint8_t)((s.flags + 1) & 3);
  seqEdited(now);
}

void handleSeqButtons(unsigned long now) {
  // A tap = next step or box, A hold = back
  int8_t nav = readANav(now, 200);
  if (nav > 0)      seqCursor = (uint8_t)((seqCursor + 1) % SEQ_CELLS);
  else if (nav < 0) seqCursor = (uint8_t)((seqCursor + SEQ_CELLS - 1) % SEQ_CELLS);

  // Encoder push = jump between the step grid and the boxes (back to the same step)
  static uint8_t lastStepCur = 0;
  if (btnE.releasedEdge) {
    if (seqCursor < SEQ_STEPS) { lastStepCur = seqCursor; seqCursor = SEQ_STEPS; }
    else                       seqCursor = lastStepCur;
  }

  // D hold = play / stop from anywhere; D tap = accent / slide (on a step)
  static bool dHold = false;
  if (btnD.pressedEdge) dHold = false;
  if (btnD.stable && !dHold && btnD.heldFor(now) >= 600) { dHold = true; seqToggleReq = true; }
  bool dTap = btnD.releasedEdge && !dHold;

  int det = readEncoder();
  static unsigned long bRepT = 0, cRepT = 0;
  static bool bLong = false;
  MPat& p = seq.pat[editPat];

  if (seqCursor < SEQ_STEPS) {                                // ── a step ──
    MStep& s = p.st[seqCursor];
    if (det) {                                                // encoder: note
      if (!s.note) s.note = lastNote;
      else {
        int n = (int)s.note + det;
        if (n < 24) n = 24;
        if (n > 96) n = 96;
        s.note = (uint8_t)n;
      }
      lastNote = s.note;
      audition(s.note);
      seqEdited(now);
    }
    if (btnB.pressedEdge) bLong = false;
    if (btnB.stable && !bLong && btnB.heldFor(now) >= 500) {  // B hold: accent / slide
      bLong = true;
      cycleStepFlags(now);
    }
    if (btnB.releasedEdge && !bLong) {                        // B tap: note on / rest
      if (s.note) { lastNote = s.note; s.note = 0; s.flags = 0; }
      else        { s.note = lastNote; audition(s.note); }
      seqEdited(now);
    }
       if (dTap) cycleStepFlags(now);                            // D tap: accent / slide
    if (btnC.pressedEdge) seqToggleReq = true;                // C: play / stop
  } else {                                                    // ── the boxes ──
    uint8_t box = seqCursor - SEQ_STEPS;
    bool bF = btnB.repeatFire(now, bRepT);
    bool cF = btnC.repeatFire(now, cRepT);
    int  d  = det + (bF ? 1 : 0) - (cF ? 1 : 0);
    if (box == 0) {                                           // BPM
      if (d) {
        int b = (int)seq.bpm + d;
        if (b < 60) b = 60;
        if (b > 200) b = 200;
        seq.bpm = (uint8_t)b;
        seqEdited(now);
      }
    } else if (box == 1) {                                    // PAT
      if (d) {
        editPat = (uint8_t)(((int)editPat + d % SEQ_PATS + SEQ_PATS) % SEQ_PATS);
        queuedPat = editPat;                                  // plays from the next pass
        if (!seqRunning) playPat = editPat;
      }
    } else if (box == 2) {                                    // LEN
      if (d) {
        int l = (int)p.len + d;
        if (l < 1) l = 1;
        if (l > SEQ_STEPS) l = SEQ_STEPS;
        p.len = (uint8_t)l;
        seqEdited(now);
      }
    } else if (box == 3) {                                    // REC
      if (btnB.pressedEdge) seqRec = !seqRec;
      if (btnC.pressedEdge) seqToggleReq = true;
    } else if (box == 4) {                                    // CLR
      if (btnB.pressedEdge) {
        for (int i = 0; i < SEQ_STEPS; i++) { p.st[i].note = 0; p.st[i].flags = 0; }
        seqEdited(now);
        snprintf(toastMsg, sizeof(toastMsg), "Cleared P%d", editPat + 1);
        toastT = now;
      }
      if (btnC.pressedEdge) seqToggleReq = true;
    } else {                                                  // CHN: OFF, >2 .. >8
      if (d) {
        int c = (int)CHAIN_END + d;
        if (c < 0) c = 0;
        if (c > SEQ_PATS - 1) c = SEQ_PATS - 1;
        CHAIN_END = (uint8_t)c;
        seqEdited(now);
      }
    }
  }
}

void handleButtons() {
  unsigned long now = millis();
  btnA.update(now); btnB.update(now); btnC.update(now); btnD.update(now); btnE.update(now);

  // A+B+C held for 2 s = restore factory patches (asks which first)
  static unsigned long tripleT = 0;
  static bool tripleLatch = false;
  if (tripleLatch) {
    if (!btnA.stable && !btnB.stable && !btnC.stable) tripleLatch = false;
    return;                                   // ignore releases until all are let go
  }
  if (btnA.stable && btnB.stable && btnC.stable && uiMode != UI_SEQ) {
    if (tripleT == 0) tripleT = now;
    if (now - tripleT >= 2000) {
      tripleLatch = true;
      tripleT = 0;
      if (uiMode != UI_PATCH) enterPatchMode();   // keeps the slot chosen on the patch page
      else { endPreview(); uiNeedsFull = true; }
      confirmState = CONF_FACTORY;
      confirmDirty = true;
    }
    return;
  }
  tripleT = 0;

  // A+B held for 1 s toggles the sequencer screen (saves the patterns on the way out)
  static unsigned long abT = 0;
  static bool abLatch = false;
  if (abLatch) {
    if (!btnA.stable && !btnB.stable) abLatch = false;
    return;                                   // swallow releases from the combo
  }
  if (btnA.stable && btnB.stable && !btnC.stable && uiMode != UI_PATCH) {
    if (abT == 0) abT = now;
    if (now - abT >= 1000) {
      abLatch = true;
      abT = 0;
      if (uiMode == UI_SEQ) {
        if (seqDirty) seqSave();
        uiMode = UI_PARAMS;
      } else {
        uiMode = UI_SEQ;
      }
      readEncoder();                          // drop any turns made meanwhile
      uiNeedsFull = true;
    }
    return;
  }
  abT = 0;

  // B+C held together toggles the patch page
  static unsigned long comboT = 0;
  static bool comboLatch = false;
  if (comboLatch) {
    if (!btnB.stable && !btnC.stable) comboLatch = false;
    return;                                   // swallow releases from the combo
  }
  if (btnB.stable && btnC.stable && uiMode != UI_SEQ) {
    if (comboT == 0) comboT = now;
    if (now - comboT >= 800) {
      comboLatch = true;
      comboT = 0;
      if (uiMode == UI_PARAMS) enterPatchMode();
      else                     exitPatchMode();
    }
    return;
  }
  comboT = 0;

  // D: params – tap = next page, hold = patch page; patch page – tap = back
  // (on the sequencer screen D sets accent / slide instead)
  if (uiMode != UI_SEQ && !rebootConfirm) {
    static bool dHoldFired = false;
    if (btnD.pressedEdge) dHoldFired = false;
    if (uiMode == UI_PARAMS && btnD.stable && !dHoldFired && btnD.heldFor(now) >= 600) {
      dHoldFired = true;
      enterPatchMode();
    }
    if (btnD.releasedEdge && !dHoldFired) {
      if (uiMode == UI_PARAMS) nextPage();
      else                     exitPatchMode();
    }
  }

  if (uiMode == UI_PARAMS)    handleParamButtons(now);
  else if (uiMode == UI_SEQ)  handleSeqButtons(now);
  else                        handlePatchButtons(now);
}

// ============================================================
//  SPLASH GRAPHICS
// ============================================================
// MALICE logo – 148 x 35, 1-bit, drawn with display.drawBitmap()
#define MALICE_LOGO_W 148
#define MALICE_LOGO_H 35
const uint8_t MALICE_LOGO[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x00, 0x00, 0x20, 0x00, 0x01, 0x80, 0x00, 0x42, 0x00, 0x00, 0x0C, 0x00, 0x80, 0x00, 0x40, 0x80, 0x00, 0x20,
  0x7F, 0x80, 0x00, 0x7F, 0x00, 0x01, 0x80, 0x00, 0xFE, 0x00, 0x00, 0x1C, 0x01, 0xFF, 0xFF, 0xC1, 0xFF, 0xFF, 0xE0,
  0x7F, 0xC0, 0x00, 0xFF, 0x00, 0x03, 0xC0, 0x00, 0xFE, 0x00, 0x00, 0x3C, 0x03, 0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xC0,
  0x3F, 0xC0, 0x01, 0xFF, 0x00, 0x07, 0xE0, 0x00, 0xFE, 0x00, 0x00, 0x7C, 0x07, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x80,
  0x1F, 0xE0, 0x03, 0xFF, 0x00, 0x07, 0xE0, 0x00, 0xFE, 0x00, 0x00, 0xFC, 0x0F, 0xFF, 0xFE, 0x0F, 0xFF, 0xFF, 0x00,
  0x0F, 0xF0, 0x03, 0xFF, 0x00, 0x0F, 0xF0, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x1F, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0x00,
  0x67, 0xF8, 0x07, 0xFF, 0x00, 0x0F, 0xF0, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x3F, 0xFF, 0xF8, 0x3F, 0xFF, 0xFE, 0x00,
  0x77, 0xF8, 0x0F, 0xFF, 0x00, 0x1F, 0xF8, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7F, 0xFC, 0x1F, 0xFF, 0x00, 0x1F, 0xF8, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7F, 0xFE, 0x1F, 0xFF, 0x00, 0x3F, 0xFC, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x80, 0x00, 0x00, 0x40, 0x00, 0x00,
  0x7F, 0xFE, 0x3F, 0xFF, 0x00, 0x3F, 0xFC, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00,
  0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0x7F, 0xFE, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00,
  0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x7F, 0xFE, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xC0,
  0x7F, 0x7F, 0xFF, 0x7F, 0x00, 0xFE, 0x7F, 0x00, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0x80,
  0x7F, 0x3F, 0xFF, 0x7F, 0x00, 0xFE, 0x7F, 0x80, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0x00,
  0x7F, 0x3F, 0xFE, 0x7F, 0x01, 0xFC, 0x3F, 0x80, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x1F, 0xFF, 0xFE, 0x00,
  0x7F, 0x1F, 0xFC, 0x7F, 0x01, 0xF8, 0x3F, 0xC0, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x3F, 0xFF, 0xFC, 0x00,
  0x7F, 0x0F, 0xF8, 0x7F, 0x03, 0xF8, 0x1F, 0xC0, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x3F, 0xFF, 0xF8, 0x00,
  0x7F, 0x07, 0xF8, 0x7F, 0x07, 0xF0, 0x0F, 0xE0, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x3F, 0xC0, 0x00, 0x00,
  0x7F, 0x07, 0xF0, 0x7F, 0x07, 0xFF, 0xFF, 0xE0, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00,
  0x7F, 0x03, 0xE0, 0x7F, 0x0F, 0xFF, 0xFF, 0xF0, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00,
  0x7F, 0x01, 0xC0, 0x7F, 0x0F, 0xFF, 0xFF, 0xF0, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0x80, 0x00, 0x0F, 0xC0, 0x00, 0x00,
  0x7F, 0x00, 0x80, 0x7F, 0x1F, 0xFF, 0xF7, 0xF8, 0xFE, 0x00, 0x01, 0xFC, 0x7F, 0xC0, 0x00, 0x1F, 0xC0, 0x00, 0x00,
  0x7F, 0x00, 0x00, 0x7F, 0x1F, 0xFF, 0xE3, 0xF8, 0xFF, 0xFF, 0xF9, 0xFC, 0x7F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0xC0,
  0x7F, 0x00, 0x00, 0x7F, 0x3F, 0xFF, 0xC3, 0xFC, 0xFF, 0xFF, 0xF1, 0xFC, 0x3F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0xC0,
  0x7F, 0x00, 0x00, 0x7E, 0x3F, 0x80, 0x01, 0xFC, 0xFF, 0xFF, 0xF1, 0xF8, 0x1F, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0x80,
  0x7F, 0x00, 0x00, 0x7C, 0x7F, 0x80, 0x01, 0xFE, 0xFF, 0xFF, 0xE1, 0xF0, 0x0F, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0x00,
  0x7E, 0x00, 0x00, 0x7C, 0x7F, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xC1, 0xE0, 0x07, 0xFF, 0xF8, 0x3F, 0xFF, 0xFE, 0x00,
  0x7C, 0x00, 0x00, 0x78, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x81, 0xE0, 0x03, 0xFF, 0xF0, 0x3F, 0xFF, 0xFC, 0x00,
  0x78, 0x00, 0x00, 0x70, 0xFE, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x01, 0xC0, 0x03, 0xFF, 0xE0, 0x7F, 0xFF, 0xF8, 0x00,
  0x70, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x60, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// Black outline behind the MALICE logo (logo shape grown by 2 px) so the
// white letters stay crisp over the face
const uint8_t MALICE_OUTLINE[] PROGMEM = {
  0xFF, 0xE0, 0x01, 0xFF, 0xC0, 0x07, 0xE0, 0x03, 0xFF, 0x80, 0x00, 0x7F, 0x07, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0xF0,
  0xFF, 0xF0, 0x03, 0xFF, 0xC0, 0x0F, 0xF0, 0x03, 0xFF, 0x80, 0x00, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xF0, 0x07, 0xFF, 0xC0, 0x1F, 0xF8, 0x03, 0xFF, 0x80, 0x01, 0xFF, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xF8, 0x0F, 0xFF, 0xC0, 0x1F, 0xF8, 0x03, 0xFF, 0x80, 0x03, 0xFF, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFC, 0x0F, 0xFF, 0xC0, 0x3F, 0xFC, 0x03, 0xFF, 0x80, 0x07, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFE, 0x1F, 0xFF, 0xC0, 0x3F, 0xFC, 0x03, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFE, 0x3F, 0xFF, 0xC0, 0x7F, 0xFE, 0x03, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0,
  0xFF, 0xFF, 0x7F, 0xFF, 0xC0, 0x7F, 0xFE, 0x03, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0x03, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0x03, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0x80,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC1, 0xFF, 0xFF, 0x83, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xF8, 0x00, 0x07, 0xF0, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC1, 0xFF, 0xFF, 0x83, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xC3, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x1F, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xE3, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0xE3, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0xF3, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xF3, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0xE0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFB, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0xC0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xFB, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0x80,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0xFF, 0xFF, 0xFE, 0x00,
  0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0xFF, 0xF0, 0x00, 0x00,
  0xFF, 0xDF, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xCF, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xC7, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xC3, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0,
  0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
  0xFF, 0x80, 0x01, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xFF, 0xF7, 0xF8, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
  0xFF, 0x00, 0x01, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xFF, 0xE7, 0xF8, 0x0F, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0x00,
  0xFE, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0xFF, 0xC7, 0xF0, 0x0F, 0xFF, 0xF9, 0xFF, 0xFF, 0xFE, 0x00,
  0xFC, 0x00, 0x01, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xF8, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// MALICE face – 93 x 128, 1-bit (skin areas only), drawn in dark red
#define MALICE_FACE_W 93
#define MALICE_FACE_H 128
#define COL_FACE_RED  0x6800
const uint8_t MALICE_FACE[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x00, 0xE0, 0x00, 0x08, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x70, 0x00, 0x08, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x38, 0x00, 0x08, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x18, 0x00, 0x08, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x1C, 0x00, 0x18, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x0C, 0x00, 0x18, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x86, 0x00, 0x10, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0xE7, 0x00, 0x10, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFB, 0x80, 0x31, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x33, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x67, 0x18, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xF8, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x13, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x78, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0xFC, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0xFE, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0xFF, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0xFF, 0x80, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0xFF, 0xC0, 0x7F, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0xFF, 0xC0, 0x7E, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
  0x00, 0x00, 0xFF, 0xE0, 0x7F, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
  0x00, 0x00, 0xFF, 0xE0, 0xFF, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xF8, 0x00,
  0x00, 0x00, 0xFF, 0xF0, 0xFF, 0x80, 0x00, 0x1F, 0xFF, 0xFF, 0xC0, 0x00,
  0x00, 0x00, 0x7F, 0xF0, 0xFF, 0xC6, 0x00, 0x03, 0xFF, 0xFC, 0x00, 0x00,
  0x00, 0x00, 0x7F, 0xF0, 0xFF, 0xE3, 0xC0, 0x00, 0xFF, 0xE0, 0x00, 0x00,
  0x00, 0x00, 0x7F, 0xF0, 0xFF, 0xE1, 0xF8, 0x00, 0x7F, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x7F, 0xF0, 0xFF, 0xF0, 0xFF, 0x00, 0x7F, 0x00, 0xC0, 0x00,
  0x00, 0x00, 0x3F, 0xF0, 0xFF, 0xF8, 0x3F, 0xC0, 0x7E, 0x07, 0x80, 0x00,
  0x00, 0x00, 0x3F, 0xF8, 0xFF, 0xFC, 0x00, 0x00, 0x7E, 0x3F, 0x80, 0x00,
  0x00, 0x00, 0x1F, 0xFC, 0xFF, 0xFC, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00,
  0x00, 0x00, 0x1F, 0xFE, 0xFF, 0xFE, 0x00, 0x01, 0xFC, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0x00, 0x03, 0xFC, 0x00, 0x20, 0x00,
  0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0x0F, 0x87, 0xFE, 0x00, 0x60, 0x00,
  0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x8F, 0xFE, 0x70, 0xE0, 0x00,
  0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0xFE, 0x7F, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x1F, 0xDF, 0xFF, 0xFF, 0x3F, 0xFE, 0x7F, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x1F, 0xE7, 0xFF, 0xFF, 0x7F, 0xFE, 0x7F, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x1F, 0xE3, 0xFF, 0xFF, 0xFF, 0xFE, 0x7F, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x1F, 0xF1, 0xFF, 0xFF, 0xFF, 0xFE, 0x7F, 0xE0, 0x00,
  0x00, 0x00, 0x00, 0x0F, 0xF0, 0xFF, 0xFF, 0xFF, 0xFE, 0x7F, 0xC0, 0x00,
  0x00, 0x00, 0x00, 0x0F, 0xF8, 0x3F, 0xFF, 0xFF, 0xFE, 0x7F, 0xC0, 0x00,
  0x00, 0x00, 0x00, 0x0F, 0xF8, 0x1F, 0xFF, 0xFF, 0xFE, 0x7F, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x0F, 0xFC, 0x0F, 0xFF, 0xFF, 0xFF, 0x7F, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xFE, 0x07, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xFE, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xFF, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xFF, 0x07, 0xFF, 0xF1, 0xFF, 0xFE, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xFF, 0x87, 0xFF, 0xFC, 0x1F, 0xFC, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0x87, 0xFF, 0xFE, 0x03, 0xFC, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xC7, 0xFF, 0xFF, 0x07, 0xFC, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xC7, 0xFF, 0xFF, 0x8F, 0xF8, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xFF, 0xE7, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0xE7, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3F, 0xF7, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF0, 0xEF, 0xE0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x00, 0x03, 0xE0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x01, 0xE0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xF8, 0x00, 0x01, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x03, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFC, 0x07, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x1E, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xC0, 0x1E, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xF8, 0x1C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xF8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// ============================================================
//  SPLASH – Mindburner intro, then MALICE face + logo
// ============================================================
// MALICE's eyes (rows of pixels on the face bitmap): y, x from, x to
const uint8_t MALICE_EYES[9][3] = {
  {68,45,46}, {69,46,49}, {70,47,52}, {71,48,55}, {72,50,57},   // left eye
  {71,80,81}, {72,77,80}, {73,74,80}, {74,74,79}                // right eye
};

void drawMaliceEyes(int16_t x0, uint16_t color) {
  for (int i = 0; i < 9; i++)
    display.drawFastHLine(x0 + MALICE_EYES[i][1], MALICE_EYES[i][0],
                          MALICE_EYES[i][2] - MALICE_EYES[i][1] + 1, color);
}

// Reboot: save, MALICE's eyes flash, "BYE...", restart
void sayByeAndRestart() {
  seqMidiStop = true;                                   // stop the sequencer
  pushEvent(EV_ALL_OFF, 0, 0);                          // let held notes release
  EEPROM.write(EE_ADDR_LAST, currentPatch);             // come back on the same patch
  EEPROM.write(EE_ADDR_MIDICH, pv[P_MIDICH]);
  EEPROM.write(EE_ADDR_MTUNE, pv[P_MTUNE]);
  EEPROM.write(EE_ADDR_MTUNE + 1, (uint8_t)(pv[P_MTUNE] ^ 0xA5));
  EEPROM.commit();
  if (seqDirty) seqSave();                              // keep sequencer edits

  const int FX = (SCREEN_W - MALICE_FACE_W) / 2;
  display.fillScreen(COL_BLACK);
  display.drawBitmap(FX, 0, MALICE_FACE, MALICE_FACE_W, MALICE_FACE_H, COL_FACE_RED);
  display.drawBitmap(6, 80, MALICE_OUTLINE, MALICE_LOGO_W, MALICE_LOGO_H, COL_BLACK);
  display.drawBitmap(6, 80, MALICE_LOGO,    MALICE_LOGO_W, MALICE_LOGO_H, COL_WHITE);
  delay(300);
  for (int f = 0; f < 2; f++) {
    drawMaliceEyes(FX, COL_RED);
    delay(120);
    drawMaliceEyes(FX, COL_FACE_RED);
    delay(120);
  }
  drawMaliceEyes(FX, COL_RED);                          // and they stay lit
  display.setTextSize(1); display.setTextColor(COL_GREY);
  display.setCursor(62, 119); display.print(F("BYE..."));
  delay(1500);
  ESP.restart();
}

void splash() {
  // ── Mindburner intro (same animation as the other Mindburner units) ──
  const uint16_t COL_MAGENTA_I = ST77XX_MAGENTA;
  const char* word = "MINDBURNER"; const int N = 10;
  const int FINAL_Y = 48, CHAR_W = 12, START_X = (SCREEN_W - N * CHAR_W) / 2;
  int finalX[N], finalY[N];
  for (int i = 0; i < N; i++) { finalX[i] = START_X + i * CHAR_W; finalY[i] = FINAL_Y; }
  randomSeed(esp_random());
  float px[N], py[N];
  for (int i = 0; i < N; i++) {
    int edge = (i * 7 + 3) % 4;
    switch (edge) {
      case 0:  px[i] = random(0, SCREEN_W); py[i] = -16;           break;
      case 1:  px[i] = random(0, SCREEN_W); py[i] = SCREEN_H + 4;  break;
      case 2:  px[i] = -16;          py[i] = random(0, SCREEN_H);  break;
      default: px[i] = SCREEN_W + 4; py[i] = random(0, SCREEN_H);  break;
    }
  }
  const uint16_t palette[] = { COL_YELLOW, COL_CYAN, COL_GREEN, COL_MAGENTA_I, COL_ORANGE,
                               COL_WHITE, COL_YELLOW, COL_CYAN, COL_GREEN, COL_MAGENTA_I };
  display.fillScreen(COL_BLACK);
  const int FRAMES = 40, FRAME_MS = 30;
  for (int f = 0; f <= FRAMES; f++) {
    float t  = (float)f / (float)FRAMES;
    float te = 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
    display.fillScreen(COL_BLACK);
    for (int i = 0; i < N; i++) {
      int cx = (int)(px[i] + (finalX[i] - px[i]) * te);
      int cy = (int)(py[i] + (finalY[i] - py[i]) * te);
      if (cx < -12 || cx > SCREEN_W || cy < -16 || cy > SCREEN_H) continue;
      char ch[2] = { word[i], 0 };
      display.setTextSize(2); display.setTextColor(palette[i]);
      display.setCursor(cx, cy); display.print(ch);
    }
    delay(FRAME_MS);
  }
  delay(600);
  for (int flash = 0; flash < 2; flash++) {
    display.setTextSize(2); display.setTextColor((flash % 2 == 0) ? COL_WHITE : COL_YELLOW);
    display.setCursor(START_X, FINAL_Y); display.print(F("MINDBURNER"));
    delay(180);
  }
  delay(400);

  // ── MALICE face + logo + version ──
  display.fillScreen(COL_BLACK);
  display.drawBitmap((SCREEN_W - MALICE_FACE_W) / 2, 0, MALICE_FACE,
                     MALICE_FACE_W, MALICE_FACE_H, COL_FACE_RED);
  display.drawBitmap(6, 80, MALICE_OUTLINE, MALICE_LOGO_W, MALICE_LOGO_H, COL_BLACK);
  display.drawBitmap(6, 80, MALICE_LOGO,    MALICE_LOGO_W, MALICE_LOGO_H, COL_WHITE);
  display.setTextSize(1); display.setTextColor(COL_GREY);
  display.setCursor(68, 119); display.print(F("v1.4"));

  // Eyes flash red twice
  const int EX = (SCREEN_W - MALICE_FACE_W) / 2;
  delay(500);
  for (int f = 0; f < 2; f++) {
    drawMaliceEyes(EX, COL_RED);
    delay(120);
    drawMaliceEyes(EX, COL_FACE_RED);
    delay(120);
  }
  delay(1000);
}

// ============================================================
//  DISPLAY – PARAMETER PAGES (core 1)
// ============================================================
int16_t lastDrawnVal[NUM_PARAMS];
uint8_t lastDrawnSel    = 255;
int     lastDrawnNote   = -1;
int     lastDrawnGate   = -1;
int     lastDrawnAct    = -1;
int16_t lastDrawnCc     = -2;
int     lastToastOn     = -1;
uint8_t lastDrawnPatch  = 255;
int     lastDrawnEdited = -1;
int     lastClkShown    = -1;
unsigned long lastUiTime = 0;

// CLK indicator: green while MIDI clock arrives, flashing white on each beat
void drawClockInd(int y) {
  bool live = (millis() - lastClockMs) < 500;
  int st = 0;
  if (live) st = ((clockTicks - clockBase) % 24 < 6) ? 2 : 1;
  if (st == lastClkShown) return;
  lastClkShown = st;
  display.fillRect(141, y, 19, 8, COL_BLACK);
  if (st) {
    display.setTextSize(1);
    display.setTextColor(st == 2 ? COL_WHITE : COL_GREEN);
    display.setCursor(142, y);
    display.print(F("CLK"));
  }
}

void drawParamsStatic() {
  display.fillScreen(COL_BLACK);
  display.fillRect(0, 0, SCREEN_W, HDR_H, HDR_BG);
  display.drawFastHLine(0, HDR_H, SCREEN_W, COL_WHITE);
  display.setTextSize(1); display.setTextColor(HDR_FG);
  display.setCursor(2, 4); display.print(PAGES[curPage].name);
  display.drawFastHLine(0, FOOTER_Y, SCREEN_W, COL_DKGREY);
  display.setTextColor(COL_GREY);
    display.setCursor(0, HINT_TY); display.print(F("Turn:val D:pg Dh:patch"));

  for (uint8_t i = 0; i < NUM_PARAMS; i++) lastDrawnVal[i] = -1;
  lastDrawnSel    = 255;
  lastDrawnNote   = -1;
  lastDrawnGate   = -1;
  lastDrawnAct    = -1;
  lastDrawnCc     = -2;
  lastToastOn     = -1;
  lastDrawnPatch  = 255;
  lastDrawnEdited = -1;
  lastClkShown    = -1;
}

void drawRow(uint8_t id, uint8_t row) {
  int y = LIST_Y + row * ROW_H;
  bool sel = (id == selParam);
  display.fillRect(0, y, SCREEN_W, ROW_H, sel ? COL_SELBG : COL_BLACK);
  if (sel) display.fillRect(0, y, 3, ROW_H, HDR_BG);

  display.setTextSize(1);
  display.setTextColor(sel ? COL_WHITE : COL_GREY);
  display.setCursor(6, y + 2); display.print(PARAMS[id].name);

  char vb[10];
  formatParam(id, pv[id], vb, sizeof(vb));
  display.setTextColor(sel ? COL_YELLOW : COL_CYAN);
  display.setCursor(66, y + 2); display.print(vb);

  if (PARAMS[id].maxVal == 127) {            // bar only for continuous 0-127 params
    int w = (int)pv[id] * 44 / 127;
    display.drawRect(110, y + 3, 46, 6, COL_DKGREY);
    if (w > 0) display.fillRect(111, y + 4, w, 4, sel ? COL_YELLOW : COL_CYAN);
  }
}

void renderParamsUI() {
  unsigned long now = millis();
  if (uiNeedsFull) { drawParamsStatic(); uiNeedsFull = false; }
  drawClockInd(HINT_TY);

  // ── Parameter rows for the current page ─────────────────
  const PageDef& pg = PAGES[curPage];
  for (uint8_t r = 0; r < pg.count; r++) {
    uint8_t id = pg.ids[r];
    bool selChanged = ((id == selParam) != (id == lastDrawnSel));
    bool force = (id == P_DLYTIME && lastDrawnVal[P_DLYSYNC] != pv[P_DLYSYNC]) ||  // Time shows SYNC
                 (id == P_LFORATE && lastDrawnVal[P_LFODIV]  != pv[P_LFODIV]);     // Rate shows SYNC
    if (selChanged || force || lastDrawnVal[id] != pv[id]) {
      drawRow(id, r);
      lastDrawnVal[id] = pv[id];
    }
  }
  lastDrawnSel = selParam;

  // ── Header: patch number + edited flag ──────────────────
  int edited = patchEdited() ? 1 : 0;
  if (currentPatch != lastDrawnPatch || edited != lastDrawnEdited) {
    display.fillRect(54, 2, 40, 13, HDR_BG);
    display.setTextSize(1); display.setTextColor(HDR_FG);
    display.setCursor(56, 4);
    display.print('P'); display.print(currentPatch + 1);
    if (edited) display.print('*');
    lastDrawnPatch  = currentPatch;
    lastDrawnEdited = edited;
  }

  // ── Header: MIDI activity, last note, gate ──────────────
  int act = ((now - lastMidiMs) < 80) ? 1 : 0;
  if (act != lastDrawnAct) {
    display.fillRect(98, 5, 7, 7, HDR_BG);
    if (act) display.fillRect(98, 5, 7, 7, HDR_FG);
    else     display.drawRect(98, 5, 7, 7, HDR_FG);
    lastDrawnAct = act;
  }
  int note = gDisplayNote;
  int gate = gAnyGate ? 1 : 0;
  if (note != lastDrawnNote || gate != lastDrawnGate) {
    display.fillRect(110, 2, 48, 13, HDR_BG);
    char nb[6]; getNoteName((uint8_t)note, nb);
    display.setTextSize(1); display.setTextColor(HDR_FG);
    display.setCursor(112, 4); display.print(nb);
    if (gate) display.fillRect(148, 4, 8, 8, HDR_FG);
    else      display.drawRect(148, 4, 8, 8, HDR_FG);
    lastDrawnNote = note;
    lastDrawnGate = gate;
  }

  // ── Footer: load/save message, else last CC received ────
  int16_t cc = lastCcNum;
  int toastOn = (toastMsg[0] && (now - toastT) < 1500) ? 1 : 0;
  if (cc != lastDrawnCc || toastOn != lastToastOn) {
    display.fillRect(0, FOOTER_TY, SCREEN_W, 9, COL_BLACK);
    display.setTextSize(1);
    display.setCursor(0, FOOTER_TY);
    if (toastOn) {
      display.setTextColor(COL_GREEN);
      display.print(toastMsg);
    } else if (cc >= 0) {
      display.setTextColor(COL_WHITE);
      display.print(F("CC")); display.print(cc); display.print(F(" -> "));
      const char* nm = nullptr;
      if (cc == 1) nm = "Vibrato";
      else if (cc == 120 || cc == 123) nm = "All off";
      else if (cc == 126) nm = "Mono";
      else if (cc == 127) nm = "Poly";
      else if (cc >= 111 && cc <= 115) nm = "Noise";
      else for (uint8_t i = 0; i < NUM_PARAMS; i++) if (PARAMS[i].cc == cc) { nm = PARAMS[i].name; break; }
      display.setTextColor(nm ? COL_CYAN : COL_GREY);
      display.print(nm ? nm : "unmapped");
    }
    lastDrawnCc = cc;
    lastToastOn = toastOn;
  }
}

// ============================================================
//  DISPLAY – PATCH PAGE (core 1)
// ============================================================
uint8_t lastDrawnPatchSel = 255;
uint8_t lastDrawnPatchCur = 255;

// 30 slots in a 6 x 5 grid of solid tiles – selected = yellow, live patch = green
void drawPatchBox(uint8_t s) {
  int x = 1 + (s % 6) * 26;
  int y = 20 + (s / 6) * 17;
  bool sel = (s == patchSel);
  bool cur = (s == currentPatch);

  display.fillRect(x, y, 25, 16, sel ? COL_YELLOW : (cur ? COL_GREEN : COL_DKGREY));
  display.setTextSize(2);
  display.setTextColor((sel || cur) ? COL_BLACK : COL_WHITE);
  display.setCursor(s < 9 ? x + 7 : x + 2, y + 1);
  display.print(s + 1);
}

void drawConfirm() {
  if (confirmState == CONF_FACTORY) {
    display.fillRect(10, 34, 140, 58, COL_BLACK);
    display.drawRect(10, 34, 140, 58, COL_ORANGE);
    display.drawRect(11, 35, 138, 56, COL_ORANGE);
    display.setTextSize(1);
    display.setTextColor(COL_WHITE);
    display.setCursor(18, 42); display.print(F("RESTORE FACTORY?"));
    display.setTextColor(COL_YELLOW);
    display.setCursor(18, 56); display.print(F("B: slot ")); display.print(patchSel + 1);
    display.setCursor(18, 68); display.print(F("A: all 30 slots"));
    display.setTextColor(COL_GREY);
    display.setCursor(18, 80); display.print(F("C: cancel"));
    return;
  }
  bool isSave = (confirmState == CONF_SAVE);
  uint16_t frame = isSave ? COL_RED : COL_GREEN;
  display.fillRect(10, 32, 140, 62, COL_BLACK);
  display.drawRect(10, 32, 140, 62, frame);
  display.drawRect(11, 33, 138, 60, frame);
  display.setTextSize(1);

  display.setTextColor(COL_WHITE);
  display.setCursor(18, 42);
  if (isSave) display.print(F("SAVE TO PATCH "));
  else        display.print(F("LOAD PATCH "));
  display.print(patchSel + 1); display.print('?');

  display.setCursor(18, 56);
  if (isSave) {
    display.setTextColor(COL_GREY);
    display.print(F("Overwrites slot ")); display.print(patchSel + 1);
  } else if (patchEdited()) {
    display.setTextColor(COL_ORANGE);
    display.print(F("Unsaved edits lost"));
  } else {
    display.setTextColor(COL_GREY);
    display.print(F("Replaces live sound"));
  }

  display.setTextColor(COL_YELLOW);
  display.setCursor(18, 76); display.print(F("B: YES"));
  display.setCursor(96, 76); display.print(F("C: NO"));
}

void renderPatchUI() {
  bool full = uiNeedsFull;

  if (full) {
    display.fillScreen(COL_BLACK);
    display.fillRect(0, 0, SCREEN_W, HDR_H, COL_CYAN);
    display.drawFastHLine(0, HDR_H, SCREEN_W, COL_WHITE);
    display.setTextSize(1); display.setTextColor(COL_BLACK);
    display.setCursor(2, 4); display.print(F("PATCH MEMORY"));
    if (previewing) { display.setCursor(110, 4); display.print(F("PREVIEW")); }
    display.drawFastHLine(0, FOOTER_Y, SCREEN_W, COL_DKGREY);
    display.setTextColor(COL_GREY);
      display.setCursor(0, HINT_TY);   display.print(F("Push:hear B:load C:save"));
     display.setCursor(0, FOOTER_TY); display.print(F("Hold push: my sound"));
    uiNeedsFull = false;
  }

  if (confirmState == CONF_NONE) {
    for (uint8_t s = 0; s < NUM_PATCHES; s++) {
      bool dirty = full ||
                   ((s == patchSel) != (s == lastDrawnPatchSel)) ||
                   ((s == currentPatch) != (s == lastDrawnPatchCur));
      if (dirty) drawPatchBox(s);
    }
    lastDrawnPatchSel = patchSel;
    lastDrawnPatchCur = currentPatch;
  } else if (confirmDirty || full) {
    if (full) {                                   // grid behind the box
      for (uint8_t s = 0; s < NUM_PATCHES; s++) drawPatchBox(s);
    }
    drawConfirm();
    confirmDirty = false;
  }
}

// ============================================================
//  DISPLAY – SEQUENCER SCREEN (core 1)
// ============================================================
#define SQ_GX      8                    // first column x
#define SQ_CW      9                    // column pitch (8 px wide + 1 px gap)
#define SQ_GY     19                    // grid top
#define SQ_GH     62                    // grid height
#define SQ_INFO_Y 86                    // selected step / box description
#define SQ_BOX_Y  97                    // BPM PAT LEN REC CLR boxes
#define SQ_HINT_Y 118

uint32_t lastColSig[SEQ_STEPS];
int32_t  lastInfoSig = -1, lastBoxSig = -1, lastHdrSig = -1;

// One step column: pitch block (red = accent), yellow line = slide,
// white bars above and below = cursor, lighter background = playhead
void drawSeqColumn(uint8_t c) {
  const MPat& p = seq.pat[editPat];
  int  x     = SQ_GX + c * SQ_CW;
  bool inLen = c < p.len;
  bool play  = seqRunning && editPat == playPat && c == gSeqPos;
  bool cur   = (c == seqCursor);
  uint16_t bg = !inLen ? COL_BLACK : play ? 0x4208 : ((c / 4) & 1) ? 0x18E3 : 0x2945;

  display.fillRect(x, SQ_GY - 2, SQ_CW - 1, 2, cur ? COL_WHITE : COL_BLACK);
  display.fillRect(x, SQ_GY + SQ_GH + 1, SQ_CW - 1, 2, cur ? COL_WHITE : COL_BLACK);
  display.fillRect(x, SQ_GY, SQ_CW - 1, SQ_GH, bg);
  if (!inLen) display.drawPixel(x + 3, SQ_GY + SQ_GH / 2, COL_DKGREY);

  const MStep& s = p.st[c];
  if (s.note) {
    int n = s.note;
    if (n < 24) n = 24;
    if (n > 96) n = 96;
    int y = SQ_GY + (96 - n) * (SQ_GH - 4) / 72;
    uint16_t col = !inLen ? COL_GREY : (s.flags & ST_ACC) ? COL_RED : COL_CYAN;
    display.fillRect(x, y, SQ_CW - 1, 4, col);
    if (s.flags & ST_SLD) {
      int ly = (y > SQ_GY + SQ_GH - 8) ? y - 2 : y + 5;
      display.drawFastHLine(x, ly, SQ_CW - 1, COL_YELLOW);
    }
  }
}

void drawSeqInfo() {
  display.fillRect(0, SQ_INFO_Y - 1, SCREEN_W, 10, COL_BLACK);
  display.setTextSize(1);
  display.setCursor(2, SQ_INFO_Y);
  if (seqCursor < SEQ_STEPS) {
    const MStep& s = seq.pat[editPat].st[seqCursor];
    display.setTextColor(COL_WHITE);
    display.print(F("STEP ")); display.print(seqCursor + 1);
    display.setCursor(56, SQ_INFO_Y);
    if (s.note) {
      char nb[6]; getNoteName(s.note, nb);
      display.setTextColor(COL_CYAN);   display.print(nb);
      display.setCursor(92, SQ_INFO_Y);
      display.setTextColor(COL_RED);    if (s.flags & ST_ACC) display.print(F("ACC"));
      display.setCursor(118, SQ_INFO_Y);
      display.setTextColor(COL_YELLOW); if (s.flags & ST_SLD) display.print(F("SLIDE"));
    } else {
      display.setTextColor(COL_GREY);   display.print(F("rest"));
    }
  } else {
    static const char* const DESC[6] = { "Tempo (BPM)", "Pattern to edit / play",
                                         "Pattern length (steps)", "Record from keyboard",
                                         "Clear this pattern", "Chain patterns from P1" };
    display.setTextColor(COL_GREY);
    display.print(DESC[seqCursor - SEQ_STEPS]);
  }
}

void drawSeqBoxes() {
  for (int i = 0; i < 6; i++) {
    int  x   = 1 + i * 26;
    bool sel = (seqCursor == SEQ_STEPS + i);
    uint16_t fill = (i == 3 && seqRec) ? COL_RED : COL_BLACK;
    display.fillRect(x, SQ_BOX_Y, 24, 13, fill);
    display.drawRect(x, SQ_BOX_Y, 24, 13, sel ? COL_YELLOW : COL_DKGREY);
    display.setTextSize(1);
    display.setTextColor(sel ? COL_YELLOW : COL_WHITE);
    display.setCursor(x + 3, SQ_BOX_Y + 3);
    if (i == 0)      display.print(seq.bpm);
    else if (i == 1) { display.print('P'); display.print(editPat + 1); }
    else if (i == 2) { display.print('L'); display.print(seq.pat[editPat].len); }
    else if (i == 3) display.print(F("REC"));
    else if (i == 4) display.print(F("CLR"));
    else if (CHAIN_END) { display.print('>'); display.print(CHAIN_END + 1); }
    else {
      if (!sel) display.setTextColor(COL_GREY);
      display.print(F("CHN"));
    }
  }
  // Hint for the cursor position (clear of the CLK indicator)
  display.fillRect(0, SQ_HINT_Y - 1, 140, 10, COL_BLACK);
  display.setTextColor(COL_GREY);
  display.setCursor(0, SQ_HINT_Y);
  if (seqCursor < SEQ_STEPS)            display.print(F("Enc:note B:on C:play"));
  else if (seqCursor < SEQ_STEPS + 3)   display.print(F("Enc or B/C: change"));
  else if (seqCursor == SEQ_STEPS + 3)  display.print(F("B:arm rec C:play"));
  else if (seqCursor == SEQ_STEPS + 4)  display.print(F("B:clear C:play"));
  else                                  display.print(F("Enc or B/C: chain"));
}

void renderSeqUI() {
  unsigned long now = millis();
  if (uiNeedsFull) {
    display.fillScreen(COL_BLACK);
    display.fillRect(0, 0, SCREEN_W, HDR_H, COL_RED);
    display.drawFastHLine(0, HDR_H, SCREEN_W, COL_WHITE);
    display.setTextSize(1); display.setTextColor(HDR_FG);
    display.setCursor(2, 4); display.print(F("SEQUENCER"));
    for (int c = 0; c < SEQ_STEPS; c++) lastColSig[c] = 0xFFFFFFFF;
    lastInfoSig = lastBoxSig = lastHdrSig = -1;
    lastClkShown = -1;
    uiNeedsFull = false;
  }

  // While chaining, the grid follows the pattern that is playing
  static uint8_t lastPlayPat = 255;
  if (seqRunning && CHAIN_END && playPat != lastPlayPat) editPat = playPat;
  lastPlayPat = playPat;

  // Step columns – redraw only those that changed
  const MPat& p = seq.pat[editPat];
  for (uint8_t c = 0; c < SEQ_STEPS; c++) {
    bool inLen = c < p.len;
    bool play  = seqRunning && editPat == playPat && c == gSeqPos;
    uint32_t sig = (uint32_t)p.st[c].note | ((uint32_t)p.st[c].flags << 8) |
                   ((uint32_t)inLen << 10) | ((uint32_t)play << 11) |
                   ((uint32_t)(c == seqCursor) << 12) | ((uint32_t)editPat << 13);
    if (sig != lastColSig[c]) { drawSeqColumn(c); lastColSig[c] = sig; }
  }

  // Info line
  int32_t infoSig = seqCursor;
  if (seqCursor < SEQ_STEPS)
    infoSig |= ((int32_t)p.st[seqCursor].note << 5) | ((int32_t)p.st[seqCursor].flags << 12);
  infoSig |= (int32_t)editPat << 16;
  if (infoSig != lastInfoSig) { drawSeqInfo(); lastInfoSig = infoSig; }

  // Boxes and hint
  int32_t boxSig = seqCursor | ((int32_t)seq.bpm << 5) | ((int32_t)editPat << 13) |
                   ((int32_t)p.len << 16) | ((int32_t)seqRec << 21) | ((int32_t)CHAIN_END << 22);
  if (boxSig != lastBoxSig) { drawSeqBoxes(); lastBoxSig = boxSig; }

  // Header: play state, pattern, recording, messages
  int toastOn = (toastMsg[0] && (now - toastT) < 1500) ? 1 : 0;
  int32_t hdrSig = (int32_t)seqRunning | ((int32_t)playPat << 1) | ((int32_t)queuedPat << 4) |
                   ((int32_t)seqRec << 7) | ((int32_t)toastOn << 8) | ((int32_t)editPat << 9) |
                   ((int32_t)(CHAIN_END != 0) << 12);
  if (hdrSig != lastHdrSig) {
    display.fillRect(64, 2, 94, 13, COL_RED);
    display.setTextSize(1); display.setTextColor(HDR_FG);
    display.setCursor(66, 4);
    if (toastOn) display.print(toastMsg);
    else if (seqRunning && CHAIN_END) {
      display.print(F("CHAIN P")); display.print(playPat + 1);
      if (seqRec) display.print(F(" REC"));
    }
    else if (seqRunning) {
      display.print(F("PLAY P")); display.print(playPat + 1);
      if (queuedPat != playPat) { display.print(F(">P")); display.print(queuedPat + 1); }
      if (seqRec) display.print(F(" REC"));
    } else {
      display.print(F("STOP P")); display.print(editPat + 1);
    }
    lastHdrSig = hdrSig;
  }

  drawClockInd(SQ_HINT_Y);
}

// Reboot yes/no box (drawn over the VOICE page)
void drawRebootBox() {
  display.fillRect(10, 32, 140, 62, COL_BLACK);
  display.drawRect(10, 32, 140, 62, COL_ORANGE);
  display.drawRect(11, 33, 138, 60, COL_ORANGE);
  display.setTextSize(1);
  display.setTextColor(COL_WHITE);
  display.setCursor(18, 42); display.print(F("REBOOT MALICE?"));
  display.setCursor(18, 56);
  if (patchEdited()) { display.setTextColor(COL_ORANGE); display.print(F("Unsaved edits lost")); }
  else               { display.setTextColor(COL_GREY);   display.print(F("Restarts the synth")); }
  display.setTextColor(COL_YELLOW);
  display.setCursor(18, 76); display.print(F("B: YES"));
  display.setCursor(96, 76); display.print(F("C: NO"));
}

void renderUI() {
  if (uiMode == UI_PATCH)     renderPatchUI();
  else if (uiMode == UI_SEQ)  renderSeqUI();
  else if (rebootConfirm) {
    if (rebootDirty) { drawRebootBox(); rebootDirty = false; }
  }
  else                        renderParamsUI();
}

// ============================================================
//  SETUP / LOOP
// ============================================================
// Save sequencer edits once things are quiet (stopped, no notes, 2 s since the last edit)
void serviceSeqSave(unsigned long now) {
  if (seqDirty && !seqRunning && !gAnyGate && (now - lastSeqEditT) >= 2000) seqSave();
}

void setup() {
  pinMode(BTN_A_PIN, INPUT_PULLUP);
  pinMode(BTN_B_PIN, INPUT_PULLUP);
  pinMode(BTN_C_PIN, INPUT_PULLUP);
  pinMode(BTN_D_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  encPrev = (uint8_t)((digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN));
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encISR, CHANGE);

  for (uint8_t i = 0; i < NUM_PARAMS; i++) {
    pv[i] = PARAMS[i].defVal;
    lastDrawnVal[i] = -1;
  }
  storageInit();                 // loads last patch + MIDI channel (factory presets on first boot)
  seqLoad();                     // sequencer patterns (starter riffs on first boot)
  smoothCut = pf(P_CUTOFF);
  selParam  = PAGES[curPage].ids[curRow];
  setupFX();                     // allocate delay lines before audio starts

  // Start SPI with only the pins the TFT uses. The display library would
  // otherwise claim GPIO19 (default MISO) and remove button D's pull-up.
  SPI.begin(TFT_SCK, -1, TFT_MOSI, -1);
  display.initR(INITR_BLACKTAB);
  display.setRotation(1);
  pinMode(BTN_D_PIN, INPUT_PULLUP);                // re-assert after display init
  splash();

  setupI2S();
  disableCore0WDT();             // the audio task owns core 0 – no idle-task watchdog
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 0);

  Serial2.begin(MIDI_BAUD, SERIAL_8N1, MIDI_RX_PIN, -1);
  Serial.begin(115200);          // CPU load report
  uiNeedsFull = true;
}

void loop() {
  while (Serial2.available()) parseMidi((uint8_t)Serial2.read());
  handleButtons();
  unsigned long now = millis();
  serviceDeferredSave(now);
  serviceAudition(now);
  serviceSeqSave(now);
  if (CPU_REPORT) {
    static unsigned long cpuT = 0;
    if (now - cpuT >= 1000) {
      cpuT = now;
      Serial.printf("Audio CPU: %lu%%\n", (unsigned long)gCpuPct);
    }
  }
  if (now - lastUiTime >= UI_REFRESH_MS) {
    lastUiTime = now;
    renderUI();
  }
}