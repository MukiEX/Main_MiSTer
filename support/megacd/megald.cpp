// LaserActive (Mega-LD / Sega PAC) support for the MegaCD core.
// Step 1: PD6103A register logger. See megald.h.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/stat.h>

#include "../../file_io.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../hardware.h"
#include "megald.h"

// Core-specific HPS commands, answered by rtl/hps_ext.v in the MegaCD core.
#define UIO_LD_GET 0x50
#define UIO_LD_SET 0x51

#define LD_SET_OP_CONTROL 0
#define LD_SET_OP_OUTREGS 1

#define LD_LOG_FILE "laseractive_pd6103a.log"
#define LD_CFG_FILE "laseractive_out.txt"

static FILE *log_fp = NULL;
static int need_apply = 1;
static time_t cfg_mtime = 0;
static int log_reads = 0;

static uint8_t cfg_out_valid[32];
static uint8_t cfg_out_value[32];

static const char *root_path(const char *name)
{
	static char path[1024];
	snprintf(path, sizeof(path), "%s/%s", getRootDir(), name);
	return path;
}

static void log_open()
{
	if (log_fp) return;
	log_fp = fopen(root_path(LD_LOG_FILE), "a");
	if (log_fp)
	{
		fprintf(log_fp, "\n=== PD6103A log opened, t=%lu ms ===\n", GetTimer(0));
		fflush(log_fp);
	}
}

static void send_control(int clear_ovf)
{
	spi_uio_cmd_cont(UIO_LD_SET);
	spi_w(LD_SET_OP_CONTROL);
	spi_w((log_reads ? 1 : 0) | (clear_ovf ? 2 : 0));
	DisableIO();
}

void ld_set_output(uint8_t reg, uint8_t value)
{
	spi_uio_cmd_cont(UIO_LD_SET);
	spi_w(LD_SET_OP_OUTREGS);
	spi_w(((reg & 0x1F) << 8) | value);
	DisableIO();
}

void ld_set_log_reads(int enable)
{
	log_reads = enable;
	send_control(0);
}

void ld_reset()
{
	// The PD6103A block is reset along with the core, so driven output values
	// and the log-reads setting have to be sent again.
	need_apply = 1;
}

// Config file, re-read whenever it changes:
//   # comment
//   log_reads 1        log CPU reads as well as writes (can be very busy)
//   out 00 01          set the driven value of output register 0x00 to 0x01
static void load_config()
{
	struct stat st;
	const char *path = root_path(LD_CFG_FILE);
	if (stat(path, &st) != 0)
	{
		if (cfg_mtime) printf("LD: %s removed\n", path);
		cfg_mtime = 0;
		memset(cfg_out_valid, 0, sizeof(cfg_out_valid));
		return;
	}
	if (st.st_mtime == cfg_mtime) return;
	cfg_mtime = st.st_mtime;

	FILE *fp = fopen(path, "r");
	if (!fp) return;

	memset(cfg_out_valid, 0, sizeof(cfg_out_valid));
	log_reads = 0;

	char line[256];
	while (fgets(line, sizeof(line), fp))
	{
		unsigned reg, val;
		int flag;
		char *p = line;
		while (*p == ' ' || *p == '\t') p++;
		if (*p == '#' || *p == '\n' || *p == '\r' || !*p) continue;

		if (sscanf(p, "out %x %x", &reg, &val) == 2 && reg < 32 && val < 256)
		{
			cfg_out_valid[reg] = 1;
			cfg_out_value[reg] = (uint8_t)val;
		}
		else if (sscanf(p, "log_reads %d", &flag) == 1)
		{
			log_reads = flag ? 1 : 0;
		}
		else
		{
			printf("LD: ignoring config line: %s", p);
		}
	}
	fclose(fp);

	printf("LD: loaded %s\n", path);
	need_apply = 1;
}

static void apply_config()
{
	send_control(1);
	for (int i = 0; i < 32; i++)
	{
		if (cfg_out_valid[i]) ld_set_output(i, cfg_out_value[i]);
	}

	log_open();
	if (log_fp)
	{
		fprintf(log_fp, "--- t=%lu ms: config applied (log_reads=%d", GetTimer(0), log_reads);
		for (int i = 0; i < 32; i++) if (cfg_out_valid[i]) fprintf(log_fp, ", out %02X=%02X", i, cfg_out_value[i]);
		fprintf(log_fp, ") ---\n");
		fflush(log_fp);
	}
}

static void log_entry(uint16_t e)
{
	int is_read = (e >> 15) & 1;
	int is_out = (e >> 14) & 1;
	int reg = (e >> 9) & 0x1F;
	int alt = (e >> 8) & 1;
	uint8_t data = e & 0xFF;

	char bits[9];
	for (int i = 0; i < 8; i++) bits[i] = (data & (0x80 >> i)) ? '1' : '0';
	bits[8] = 0;

	fprintf(log_fp, "%c %s %02X = %02X  %s%s\n",
		is_read ? 'R' : 'W', is_out ? "out" : "in ", reg, data, bits,
		(is_read && alt) ? "  (snapshot/held)" : "");
}

void ld_poll()
{
	static uint32_t poll_timer = 0;
	static uint32_t cfg_timer = 0;

	if (!cfg_timer || CheckTimer(cfg_timer))
	{
		cfg_timer = GetTimer(1000);
		load_config();
	}

	if (need_apply)
	{
		need_apply = 0;
		apply_config();
	}

	if (poll_timer && !CheckTimer(poll_timer)) return;
	poll_timer = GetTimer(5);

	uint16_t st = spi_uio_cmd_cont(UIO_LD_GET);
	int count = st & 0x3FF;
	int ovf = (st >> 15) & 1;

	if (count > 512) count = 0; // not our core / garbage
	if (!count && !ovf)
	{
		DisableIO();
		return;
	}

	static uint16_t entries[512];
	for (int i = 0; i < count; i++) entries[i] = spi_w(0);
	DisableIO();

	log_open();
	if (!log_fp) return;

	fprintf(log_fp, "--- t=%lu ms, %d entr%s ---\n", GetTimer(0), count, count == 1 ? "y" : "ies");
	for (int i = 0; i < count; i++) log_entry(entries[i]);
	if (ovf)
	{
		fprintf(log_fp, "!!! log FIFO overflowed: some accesses were dropped !!!\n");
		send_control(1);
	}
	fflush(log_fp);
}
