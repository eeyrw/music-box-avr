#ifndef __WAVETABLE__
#define __WAVETABLE__
// Sample name: True Vibe C5
// Sample's base frequency: 522.9098067244921 Hz
// Sample's sample rate: 32000 Hz
#define WAVETABLE_LEN 55318
#define WAVETABLE_ATTACK_LEN 53789
#define WAVETABLE_LOOP_LEN 1529
#define WAVETABLE_ACTUAL_LEN 55319

#ifndef __ASSEMBLER__
#include <stdint.h>
extern const int8_t WaveTable[WAVETABLE_ACTUAL_LEN];
extern const uint16_t WaveTable_Increment[];
#else
.extern	WaveTable
.extern WaveTable_Increment
#endif

#endif