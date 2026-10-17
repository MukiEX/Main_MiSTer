// LaserActive video test (stage 2). See megald_video.h.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include "../../user_io.h"
#include "../../file_io.h"
#include "../../menu.h"
#include "../../spi.h"
#include "../../hardware.h"
#include "../../shmem.h"
#include "megald_video.h"

// Core-specific HPS commands, answered by rtl/hps_ext.v in the MegaCD core.
#define UIO_LD_VSET 0x52
#define UIO_LD_VGET 0x53
#define LDV_MAGIC   0x4C44  // reply to UIO_LD_VGET from a core that has the player

#define LDV_OP_CONTROL 0    // word: bit0 enable, bit1 restart
#define LDV_OP_PPS     1    // words: {addr[6:0], data[7:0]} ...
#define LDV_OP_PARAMS  2    // words: frame_words[15:0], frame_words[17:16], field1_row
#define LDV_OP_WRCOUNT 3    // word: frames written to the ring

#define RING_ADDR  0x30100000u
#define RING_SLOTS 8
#define SLOT_SIZE  0x100000u

static int ldv_fd = -1;
static char ldv_name[1024];
static uint8_t ldv_pps[128];
static uint32_t frame_count, frame_bytes, frame_stride, field1_row;
static uint64_t frames_pos;   // file offset of frame 0
static uint8_t *ring_map = NULL;
static uint8_t *stage = NULL;

static pthread_t reader;
static int reader_running = 0;
static volatile int reader_stop = 0;
static volatile uint32_t frames_written = 0;  // by the thread
static volatile uint32_t frames_shown = 0;    // from the core, extended to 32 bits
static volatile uint32_t read_errors = 0;
static uint32_t start_frame = 0;

static int need_setup = 0;
static uint16_t sent_wr = 0xFFFF;
static int core_ok = -1;

// ------------------------------------------------------------- ZIP64 lookup

static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint64_t rd64(const uint8_t *p) { return rd32(p) | ((uint64_t)rd32(p + 4) << 32); }

static int read_at(int fd, void *buf, size_t len, uint64_t off)
{
	uint8_t *b = (uint8_t *)buf;
	while (len)
	{
		ssize_t r = pread(fd, b, len, (off_t)off);
		if (r < 0 && errno == EINTR) continue;
		if (r <= 0) return -1;
		b += r; len -= r; off += r;
	}
	return 0;
}

// Data offset of the first stored entry whose name ends in ".dscv", or 0.
static uint64_t find_dscv(int fd)
{
	uint64_t fsize = (uint64_t)lseek(fd, 0, SEEK_END);
	size_t tail = fsize < 65557 ? (size_t)fsize : 65557;
	uint8_t *buf = (uint8_t *)malloc(tail);
	if (!buf || read_at(fd, buf, tail, fsize - tail)) { free(buf); return 0; }
	long eocd = -1;
	for (long i = (long)tail - 22; i >= 0; i--) if (rd32(buf + i) == 0x06054b50) { eocd = i; break; }
	if (eocd < 0) { free(buf); return 0; }
	uint64_t entries = rd16(buf + eocd + 10), cd_size = rd32(buf + eocd + 12), cd_off = rd32(buf + eocd + 16);
	uint64_t eocd_abs = fsize - tail + eocd;
	free(buf);
	uint8_t loc[20], rec[56];
	if (eocd_abs >= 20 && !read_at(fd, loc, 20, eocd_abs - 20) && rd32(loc) == 0x07064b50 &&
		!read_at(fd, rec, 56, rd64(loc + 8)) && rd32(rec) == 0x06064b50)
	{
		entries = rd64(rec + 32);
		cd_size = rd64(rec + 40);
		cd_off = rd64(rec + 48);
	}
	if (cd_off + cd_size > fsize || cd_size > 16 * 1024 * 1024) return 0;
	uint8_t *cd = (uint8_t *)malloc(cd_size);
	if (!cd || read_at(fd, cd, cd_size, cd_off)) { free(cd); return 0; }

	uint64_t found = 0;
	size_t p = 0;
	for (uint64_t e = 0; e < entries && p + 46 <= cd_size && rd32(cd + p) == 0x02014b50; e++)
	{
		uint16_t method = rd16(cd + p + 10), nlen = rd16(cd + p + 28), xlen = rd16(cd + p + 30), clen = rd16(cd + p + 32);
		if (p + 46 + nlen + xlen + clen > cd_size) break;
		uint64_t usize = rd32(cd + p + 24), csize = rd32(cd + p + 20), lofs = rd32(cd + p + 42);
		const uint8_t *x = cd + p + 46 + nlen, *xe = x + xlen;
		while (x + 4 <= xe)
		{
			uint16_t id = rd16(x), sz = rd16(x + 2);
			const uint8_t *d = x + 4, *q = d;
			if (d + sz > xe) break;
			if (id == 0x0001)
			{
				if (usize == 0xffffffffu && q + 8 <= d + sz) { usize = rd64(q); q += 8; }
				if (csize == 0xffffffffu && q + 8 <= d + sz) { csize = rd64(q); q += 8; }
				if (lofs == 0xffffffffu && q + 8 <= d + sz) { lofs = rd64(q); q += 8; }
			}
			x = d + sz;
		}
		const char *name = (const char *)cd + p + 46;
		if (!method && nlen > 5 && !strncasecmp(name + nlen - 5, ".dscv", 5))
		{
			uint8_t lh[30];
			if (!read_at(fd, lh, 30, lofs) && rd32(lh) == 0x04034b50)
			{
				found = lofs + 30 + rd16(lh + 26) + rd16(lh + 28);
				break;
			}
		}
		p += 46 + nlen + xlen + clen;
	}
	free(cd);
	return found;
}

// ------------------------------------------------------------ reader thread

static void *reader_thread(void *)
{
	uint32_t n = (frame_bytes + 7) & ~7u;
	for (uint32_t k = frames_written; !reader_stop; k++)
	{
		// the slot being decoded (frames_shown % 8) and one spare stay untouched
		while (!reader_stop && k >= frames_shown + RING_SLOTS - 1) usleep(1000);
		if (reader_stop) break;

		uint32_t f = (start_frame + k) % frame_count;
		if (read_at(ldv_fd, stage, frame_bytes, frames_pos + (uint64_t)f * frame_stride))
		{
			read_errors++;
			memset(stage, 0, n);
		}
		memcpy(ring_map + (size_t)(k % RING_SLOTS) * SLOT_SIZE, stage, n);
		__sync_synchronize();
		frames_written = k + 1;
	}
	return NULL;
}

static void reader_halt()
{
	if (reader_running)
	{
		reader_stop = 1;
		pthread_join(reader, NULL);
		reader_running = 0;
	}
	reader_stop = 0;
}

// ------------------------------------------------------------- HPS <-> core

static void send_control(int enable, int restart)
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_CONTROL);
	spi_w((enable ? 1 : 0) | (restart ? 2 : 0));
	DisableIO();
}

static void send_setup()
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_PPS);
	for (int i = 0; i < 128; i++) spi_w((uint16_t)((i << 8) | ldv_pps[i]));
	DisableIO();

	uint32_t words = (frame_bytes + 7) / 8;
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_PARAMS);
	spi_w((uint16_t)words);
	spi_w((uint16_t)(words >> 16));
	spi_w((uint16_t)field1_row);
	DisableIO();
}

static void send_wrcount(uint16_t w)
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_WRCOUNT);
	spi_w(w);
	DisableIO();
}

// returns 0 if the core doesn't answer
static int read_status(uint16_t st[5])
{
	uint16_t magic = spi_uio_cmd_cont(UIO_LD_VGET);
	if (magic != LDV_MAGIC)
	{
		DisableIO();
		return 0;
	}
	for (int i = 0; i < 5; i++) st[i] = spi_w(0);
	DisableIO();
	return 1;
}

// ------------------------------------------------------------------ public

void ld_video_open(const char *path)
{
	reader_halt();
	if (ldv_fd >= 0)
	{
		send_control(0, 1);
		close(ldv_fd);
		ldv_fd = -1;
		printf("LDV: closed %s\n", ldv_name);
	}
	if (!path || !*path) return;

	snprintf(ldv_name, sizeof(ldv_name), "%s", path);
	char full[2048];
	if (path[0] == '/') snprintf(full, sizeof(full), "%s", path);
	else snprintf(full, sizeof(full), "%s/%s", getRootDir(), path);

	int fd = open(full, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
	{
		printf("LDV: cannot open %s\n", full);
		Info("LD video: cannot open file", 3000);
		return;
	}
	uint64_t base = find_dscv(fd);
	uint8_t h[512];
	if (!base || read_at(fd, h, sizeof(h), base) || memcmp(h, "DSCV", 4))
	{
		printf("LDV: %s has no DSCV video stream\n", full);
		Info("LD video: not a .dscmmi file", 3000);
		close(fd);
		return;
	}
	frame_count = rd32(h + 28);
	frame_bytes = rd32(h + 40);
	frame_stride = rd32(h + 44);
	frames_pos = base + rd64(h + 48);
	field1_row = rd16(h + 24);
	memcpy(ldv_pps, h + 128, 128);
	if (!frame_count || !frame_bytes || frame_bytes > SLOT_SIZE)
	{
		printf("LDV: bad DSCV header\n");
		Info("LD video: bad DSCV header", 3000);
		close(fd);
		return;
	}

	if (!ring_map) ring_map = (uint8_t *)shmem_map(RING_ADDR, RING_SLOTS * SLOT_SIZE);
	if (!stage) stage = (uint8_t *)malloc(SLOT_SIZE);
	if (!ring_map || !stage)
	{
		printf("LDV: cannot map the DDR3 ring\n");
		close(fd);
		return;
	}

	ldv_fd = fd;
	start_frame = 0;
	frames_written = 0;
	frames_shown = 0;
	read_errors = 0;
	sent_wr = 0xFFFF;
	core_ok = -1;
	need_setup = 1;

	unsigned bpp16 = ((ldv_pps[4] & 3) << 8) | ldv_pps[5];
	printf("LDV: playing %s: %u frames, %u bytes/frame, %.4g bpp, field 1 at row %u\n", full, frame_count, frame_bytes,
		bpp16 / 16.0, field1_row);
	// ld_video_poll() sends the PPS to the core and starts the reader thread
}

void ld_video_poll()
{
	static uint32_t timer = 0;
	static uint32_t report = 0;
	if (ldv_fd < 0) return;
	if (timer && !CheckTimer(timer)) return;
	timer = GetTimer(2);

	if (need_setup)
	{
		// New file: reset the core's player, send it the stream's PPS and
		// sizes, then start filling the ring from frame 0.
		need_setup = 0;
		reader_halt();
		start_frame = (start_frame + frames_shown) % frame_count;
		frames_written = 0;
		frames_shown = 0;
		sent_wr = 0xFFFF;
		send_control(0, 1);
		send_setup();
		send_control(1, 0);
		if (pthread_create(&reader, NULL, reader_thread, NULL) == 0) reader_running = 1;
	}

	uint16_t st[5];
	int ok = read_status(st);
	if (ok != core_ok)
	{
		core_ok = ok;
		if (!ok) printf("LDV: this core has no LD video player (rebuild with the LaserActive branch)\n");
	}
	if (!ok) return;

	// extend the core's 16-bit counter
	uint32_t shown = frames_shown;
	shown += (uint16_t)(st[0] - (uint16_t)shown);
	frames_shown = shown;

	uint16_t w = (uint16_t)frames_written;
	if (w != sent_wr)
	{
		send_wrcount(w);
		sent_wr = w;
	}

	if (!report || CheckTimer(report))
	{
		report = GetTimer(5000);
		printf("LDV: shown %u (disc frame %u), ring %u, late ticks %u, decode errors %u, decode %.1f ms, read errors %u\n",
			frames_shown, (start_frame + frames_shown) % frame_count, frames_written - frames_shown, st[1], st[2],
			st[3] * 256 / 53693.175, read_errors);
	}
}
