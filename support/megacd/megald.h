#ifndef MEGALD_H
#define MEGALD_H

// LaserActive (Mega-LD / Sega PAC) support for the MegaCD core.
// Step 1: PD6103A register logger.
//
// The core's PD6103A block (rtl/MCD/PD6103A.vhd) logs every sub-CPU write to
// its registers. This module drains that log over the HPS bus, writes it to
// <root>/laseractive_pd6103a.log, and lets you set the driven value of output
// registers from <root>/laseractive_out.txt without rebuilding anything.

#include <stdint.h>

void ld_poll();
void ld_reset();
void ld_set_output(uint8_t reg, uint8_t value);
void ld_set_log_reads(int enable);

#endif
