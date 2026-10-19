#ifndef MEGALD_AUDIO_H
#define MEGALD_AUDIO_H

// LaserActive: read-ahead for the disc's audio.
//
// The core's CD audio FIFO holds about 29 ms, so a sector must go out within
// ~15 ms of the FIFO asking for it. Reading each sector from the SD card at
// that moment stalls behind the video reader's frame reads now and then, and
// the FIFO runs dry (a click). This keeps the source audio (the digital
// track's audio sectors and the analog soundtrack) read a couple of seconds
// ahead of the playing position, in big sequential reads on its own thread,
// so the drive tick only copies from memory. Mixing (faders, channel
// selection) still happens at the tick, so commands take effect at once.

#include <stdint.h>
#include <vector>

struct ld_audio_seg
{
	int start, end;   // LBAs [start, end) with audio here
	uint64_t off;     // archive offset of LBA start's 2352 bytes
};

void ld_audio_start(int fd, const std::vector<ld_audio_seg> &digital, const ld_audio_seg &analog);
void ld_audio_stop();

// One sector (588 stereo samples) of each source. dir is the way the disc is
// moving (sectors per tick; 0 for a still). Sets *have_* to whether the source
// has audio there. On a cache miss (a seek, start-up) it reads the card
// directly, as before.
void ld_audio_get(int lba, int dir, int16_t *digital, int *have_digital, int16_t *analog, int *have_analog);

// Called when a sector of audio goes to the core (time in ms). A gap of more
// than 29 ms (the FIFO's whole depth) means the core ran out: counted as late
// and logged (at most 20 events per 10 s, with the time since the last seek).
void ld_audio_sent(unsigned long ms, int lba);
void ld_audio_idle();   // not playing (stopped, paused, seeking): the next gap doesn't count

struct ld_audio_stats_t { uint32_t sectors, misses, chunk_loads, read_errors, late, worst_gap; };
void ld_audio_stats(ld_audio_stats_t *s);

#endif
