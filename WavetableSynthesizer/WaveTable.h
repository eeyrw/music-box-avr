#ifndef __WAVETABLE__
#define __WAVETABLE__
// Sample name: Square Wave C5
// Sample's base frequency: 524.6176589445292 Hz
// Sample's sample rate: 32000 Hz
#define WAVETABLE_LEN 5032
#define WAVETABLE_ATTACK_LEN 4971
#define WAVETABLE_LOOP_LEN 61
#define WAVETABLE_ACTUAL_LEN 5032

#ifndef __ASSEMBLER__
#include <stdint.h>
extern const int8_t WaveTable[WAVETABLE_ACTUAL_LEN];
extern const uint16_t WaveTable_Increment[];
#else
.extern	WaveTable
.extern WaveTable_Increment
#endif

#endif