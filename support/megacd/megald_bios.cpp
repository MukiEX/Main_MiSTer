// LaserActive: which Sega PAC boot ROM the MegaLD core runs.
//
// The OSD's "LaserActive BIOS" option (status bits 65:64) picks one:
//   Auto       the disc's region (or the Region option, if it's forced)
//   Japan      boot_jp.rom
//   USA        boot_us.rom    (v1.04; boot_us102.rom if it's missing)
//   USA v1.02  boot_us102.rom (boot_us.rom if it's missing)
// all in games/MegaLD. If the file isn't there it falls back to cd_bios.rom
// in the disc's folder (the 1.x setup), then boot.rom, then the other
// region's ROM. The known dumps are recognised by CRC and named in
// laseractive.log.
//
// The core takes its console region from the BIOS header unless the Region
// option is forced, but the US v1.02 dump says "J" there, so after a known
// dump Main sends its real region on download index 6.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "../../user_io.h"
#include "../../file_io.h"
#include "../../menu.h"
#include "megacd.h"
#include "megald.h"

#define BIOS_SIZE        0x20000
#define REGION_IO_INDEX  6

enum { R_JP = 0, R_US = 1, R_EU = 2 };

struct known_bios
{
	uint32_t crc;
	int region;
	const char *name;
};

static const known_bios KNOWN[] = {
	{ 0x00EEDB3A, R_JP, "Sega PAC (Japan) v1.02, internal 0.98" },
	{ 0x3B10CF41, R_US, "Sega PAC (USA) v1.02, internal 0.98" },
	{ 0x50CD3D23, R_US, "Sega PAC (USA) v1.04" },
};

static char loaded_path[1024];
static int last_sel = -1, last_reg = -1;
static char cur_disc[1024];
static int cur_disc_region;

static uint32_t crc32_of(const uint8_t *p, size_t n)
{
	uint32_t c = 0xFFFFFFFFu;
	for (size_t i = 0; i < n; i++)
	{
		c ^= p[i];
		for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
	}
	return ~c;
}

// candidate files, most wanted first
static int candidates(const char *disc, int disc_region, char list[][1024], int max)
{
	int sel = user_io_status_get("[65:64]");
	int reg = user_io_status_get("[7:6]");     // 0 Auto, 1 JP, 2 US, 3 EU
	int want;
	if (sel == 1) want = R_JP;
	else if (sel >= 2) want = R_US;
	else if (reg == 1) want = R_JP;
	else if (reg == 2) want = R_US;
	else if ((disc_region & 2) && !(disc_region & 1)) want = R_US;   // a US-only disc
	else want = R_JP;                                                  // JP, both or unknown

	const char *home = HomeDir();
	int n = 0;
	auto add = [&](const char *dir, const char *name) {
		if (n < max) snprintf(list[n++], 1024, "%s/%s", dir, name);
	};
	if (want == R_JP) add(home, "boot_jp.rom");
	else if (sel == 3) { add(home, "boot_us102.rom"); add(home, "boot_us.rom"); }
	else { add(home, "boot_us.rom"); add(home, "boot_us102.rom"); }

	if (disc && *disc)
	{
		char dir[1024];
		snprintf(dir, sizeof(dir), "%s", disc);
		char *p = strrchr(dir, '/');
		if (p) { *p = 0; add(dir, "cd_bios.rom"); }
	}
	add(home, "boot.rom");

	if (want == R_JP) { add(home, "boot_us.rom"); add(home, "boot_us102.rom"); }
	else add(home, "boot_jp.rom");
	return n;
}

static void send_region(int region)
{
	uint8_t b[2] = { (uint8_t)region, 0 };
	user_io_set_index(REGION_IO_INDEX);
	user_io_set_download(1);
	user_io_file_tx_data(b, 2);
	user_io_set_download(0);
}

// Loads the BIOS for this disc ("" for none) if it isn't already in the core.
// reset: also reset the console and the player, as a power cycle would.
// Returns 0 if no BIOS file was found at all.
int ld_bios_load(const char *disc, int disc_region, int reset)
{
	snprintf(cur_disc, sizeof(cur_disc), "%s", disc ? disc : "");
	cur_disc_region = disc_region;
	last_sel = user_io_status_get("[65:64]");
	last_reg = user_io_status_get("[7:6]");

	static char list[8][1024];
	int n = candidates(disc, disc_region, list, 8);
	static uint8_t img[BIOS_SIZE];
	for (int i = 0; i < n; i++)
	{
		int len = FileLoad(list[i], img, BIOS_SIZE);
		if (len <= 0) continue;
		if (!strcmp(list[i], loaded_path)) return 1;   // already running

		if (reset)
		{
			user_io_status_set("[0]", 1);
			user_io_status_set("[0]", 0);
			mcd_reset();
		}
		if (!user_io_file_tx(list[i])) continue;
		snprintf(loaded_path, sizeof(loaded_path), "%s", list[i]);

		uint32_t crc = crc32_of(img, (size_t)len);
		const known_bios *k = NULL;
		for (const known_bios &x : KNOWN) if (x.crc == crc && len == BIOS_SIZE) k = &x;
		bool pac = len >= 0x130 && !memcmp(img + 0x120, "MEGA-LD BOOT ROM", 16);
		if (k)
		{
			send_region(k->region);
			ld_log("BIOS: %s (%s)", k->name, list[i]);
		}
		else if (pac)
		{
			ld_log("BIOS: an unrecognised Sega PAC boot ROM, CRC %08X (%s)", crc, list[i]);
		}
		else
		{
			char title[17] = {};
			if (len >= 0x130) for (int j = 0; j < 16; j++) title[j] = (img[0x120 + j] >= 32 && img[0x120 + j] < 127) ? img[0x120 + j] : '.';
			ld_log("BIOS: %s is not a LaserActive BIOS (\"%s\", CRC %08X); LaserDiscs need a Sega PAC boot ROM", list[i], title, crc);
			Info("Not a LaserActive BIOS:\nsee laseractive.log", 4000);
		}
		if (i > 0) ld_log("BIOS: %s was wanted but isn't there", list[0]);
		return 1;
	}
	ld_log("BIOS: none found (looked for %s and others)", n ? list[0] : "boot_jp.rom");
	Info("LaserActive BIOS not found!\nPut boot_jp.rom or boot_us.rom\nin games/MegaLD", 6000);
	return 0;
}

// The OSD's BIOS or Region option changed: switch BIOS if the choice changed.
void ld_bios_poll()
{
	if (last_sel < 0) return;   // nothing loaded yet
	int sel = user_io_status_get("[65:64]");
	int reg = user_io_status_get("[7:6]");
	if (sel == last_sel && reg == last_reg) return;
	ld_bios_load(cur_disc, cur_disc_region, 1);
}

// The core was loaded fresh: forget what was in it.
void ld_bios_forget()
{
	loaded_path[0] = 0;
}
