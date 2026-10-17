#ifndef MEGALD_MMI_H
#define MEGALD_MMI_H

// LaserActive disc images: .dscmmi (and the layout they share with .mmi).
//
// Both are ZIP archives of stored (uncompressed) entries. MediaInfo.json
// lists the disc sides ("media") and, for each, its streams:
//   Redbook  - the digital track, a .cue sheet whose files are archive entries
//   RawAudio - the analog audio, 16-bit little-endian stereo PCM at 44.1 kHz
//   RawVideo - the analog video (.qon in an .mmi). A .dscmmi replaces it with
//              a .dscv stream (see dscmmi's README), named in DSCMMI.json.
// Everything here only reads the archive's directory and small entries; the
// big entries are read in place through their offsets.

#include <stdint.h>
#include <string>
#include <vector>

struct mmi_entry
{
	std::string name;
	uint64_t off;    // offset of the entry's data in the archive
	uint64_t size;
};

struct ld_disc
{
	char path[1024];                 // full path of the archive
	int fd = -1;
	std::vector<mmi_entry> entries;

	std::string title;
	int media_count;
	int media_index;                 // the side in use
	std::string media_name;
	int side_no;
	bool clv;

	std::string redbook;             // entry names ("" if absent)
	std::string raw_audio;
	std::string raw_video;           // the .qon named by MediaInfo.json
	int lead_in, active, lead_out;   // video frames in each region

	// .dscv
	bool has_video;
	uint64_t dscv_off;               // archive offset of the .dscv entry
	uint32_t frame_count, frame_bytes, frame_stride, field1_row;
	uint64_t frames_off;             // archive offset of frame 0
	uint8_t pps[128];
	std::vector<uint32_t> vbi;       // 4 codes per frame (lines 16, 17, 279, 280); may be empty

	// RawAudio
	uint64_t audio_off, audio_size;
};

// Opens path and reads the side with index media_index (0 = first).
// On failure returns false and puts a short reason in err.
bool mmi_open(ld_disc *d, const char *path, int media_index, char *err, int errlen);
void mmi_close(ld_disc *d);
const mmi_entry *mmi_find(const ld_disc *d, const char *name);   // case-insensitive
bool mmi_read(const ld_disc *d, void *buf, size_t len, uint64_t off);  // archive offset
bool mmi_is_archive_name(const char *path);   // .dscmmi or .mmi extension

#endif
