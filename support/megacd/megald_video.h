#ifndef MEGALD_VIDEO_H
#define MEGALD_VIDEO_H

// LaserActive video: shows .dscv frames through the MegaCD core's DSC
// decoder (rtl/LD/ld_video.sv).
//
// Frames live in 16 slots of 512 KB in DDR3 at 0x30100000, which work as a
// cache: a reader thread loads the frames the player is likely to want next
// (ld_video_predict), and ld_video_request() commits a frame to the core as
// soon as its slot holds it. The core decodes each commit at once and puts
// it up at the start of a VDP frame, two VDP frames per picture (see
// ld_video.sv), and keeps the last picture up when nothing new is committed.

#include <stdint.h>

struct ld_disc;

int  ld_video_attach(const ld_disc *d);  // a disc with video was mounted; 0 if it can't be shown
void ld_video_detach();
int  ld_video_attached();

// Show this .dscv frame (index into the stream). Returns a request number,
// or -1 without video. A newer request replaces one still waiting for its
// frame to load.
int  ld_video_request(int frame);
// The request has reached the screen, or was replaced, or failed.
int  ld_video_request_done(int req);
// Load these next: frame + step * k, k = 1, 2, ...
void ld_video_predict(int frame, int step);
void ld_video_set_visible(int visible);

void ld_video_poll();                    // call often from the main loop

// Fine position of the LD picture under the VDP's: x in 1/256 LD columns,
// y in LD lines (laseractive.cfg video_x / video_y)
void ld_video_set_position(int x, int y);
int  ld_video_core_version();             // the core's LaserDisc protocol version (0: none)

#endif
