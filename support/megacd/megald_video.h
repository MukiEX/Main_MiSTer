#ifndef MEGALD_VIDEO_H
#define MEGALD_VIDEO_H

// LaserActive video test (stage 2): plays the analog video of a .dscmmi
// file through the MegaCD core's DSC decoder.
//
// A reader thread copies frames into a ring of 8 slots in DDR3 at
// 0x30100000 (1 MB each); the core (rtl/LD/ld_video.sv) decodes one per
// 1/29.97 s into a framebuffer that the scaler shows. The core reports how
// many frames it has used, and the thread stays at most 7 frames ahead.

void ld_video_open(const char *path);  // "" closes
void ld_video_poll();                  // call often from the main loop

#endif
