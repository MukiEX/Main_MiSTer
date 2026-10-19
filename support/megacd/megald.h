#ifndef MEGALD_H
#define MEGALD_H

// LaserActive (Mega-LD / Sega PAC) support for the MegaCD core: the
// LaserDisc player.
//
// The core's PD6103A block (rtl/MCD/PD6103A.vhd) logs every sub-CPU write
// to the chip. This module drains that log, runs the player the writes
// command (a port of Nemesis's MCD::LD in ares, ares/md/mcd/megald.cpp),
// and sends the player's output registers back to the core. The disc
// itself is the CD drive (megacdd.cpp) in LaserDisc mode: its sector clock
// moves the player, and the player tells it how far each tick goes, mixes
// the analog audio in, and commits video frames (megald_video.cpp).

#include <stdint.h>
#include <vector>
#include "megald_audio.h"

struct ld_disc;

void ld_poll();
void ld_reset();                                  // core reset (full player power-on)
long ld_ms_since_seek();                          // -1 before the first seek
void ld_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));  // <root>/laseractive.log

// LaserDisc image, used by the CD drive (megacdd.cpp)
const ld_disc *ld_disc_mount(const char *path);   // NULL on failure
void ld_disc_unmount();
const ld_disc *ld_get_disc();                     // NULL if none

// CD drive hooks, called once per sector tick while a LaserDisc plays
int  ld_sector_advance();                         // sectors to move (may be 0 or negative)
void ld_sector_reached(int lba, int end_of_disc);
void ld_mix_audio(int16_t *sector, const int16_t *analog, int have_digital, int have_analog);  // 588 stereo samples, in place
void ld_audio_attach(const std::vector<ld_audio_seg> &digital);  // after the cue sheet is parsed

#endif
