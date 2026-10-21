// LaserActive video: frame cache and commits. See megald_video.h.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>

#include "../../user_io.h"
#include "../../file_io.h"
#include "../../menu.h"
#include "../../spi.h"
#include "../../hardware.h"
#include "../../shmem.h"
#include "megald.h"
#include "megald_mmi.h"
#include "megald_video.h"

// Core-specific HPS commands, answered by rtl/hps_ext.v in the MegaCD core.
#define UIO_LD_VSET 0x52
#define UIO_LD_VGET 0x53
#define LDV_MAGIC    0x4C44  // reply to UIO_LD_VGET from a core with the LD video player
#define LDV_VERSION  4       // commit protocol, picture position

#define LDV_OP_CONTROL 0     // word: bit0 enable, bit1 restart, bit2 visible
#define LDV_OP_PPS     1     // words: {addr[6:0], data[7:0]} ...
#define LDV_OP_PARAMS  2     // words: frame_words[15:0], frame_words[17:16], field1_row
#define LDV_OP_COMMIT  3     // word: slot to show next
#define LDV_OP_POS     4     // words: horizontal offset (1/256 LD column), vertical offset (LD lines)

#define SLOT_BASE  0x30100000u
#define SLOTS      16
#define SLOT_SIZE  0x80000u
#define PREDICT    10        // frames to keep loaded ahead

// ------------------------------------------------------------------ state

static const ld_disc *vd = NULL;     // disc being shown

static uint8_t *slot_map = NULL;
static uint8_t *stage = NULL;

struct slot_t
{
	int frame;          // stream frame held, -1 none
	int loading;        // the reader thread is filling it
	int committed;      // a commit used it ...
	uint32_t seq;       // ... with this sequence number
	uint32_t used;      // LRU stamp
};

static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_t reader;
static int reader_running = 0;
static volatile int reader_stop = 0;
static slot_t slots[SLOTS];
static uint32_t lru_clock = 0;
static int want[PREDICT + 1];
static int want_n = 0;
static int pending_frame = -1;       // requested, waiting for its slot
static uint32_t read_errors = 0, loads = 0;

static uint32_t pushed = 0;          // commits sent to the core
static uint32_t done = 0;            // commits the core has finished (extended from 16 bits)

// request bookkeeping
#define REQS 64
static struct { int id; int state; uint32_t seq; } reqs[REQS];   // state 0 waiting, 1 committed, 2 finished
static int next_req = 0;
static int pending_req = -1;

static int core_ok = 0;
static int enabled = 0;
static int visible = 0, sent_visible = -1;
static int pos_x = 0, pos_y = 0, sent_pos = 0;
static uint16_t st[7];

// ------------------------------------------------------------- HPS <-> core

static void send_control(int enable, int restart, int vis)
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_CONTROL);
	spi_w((enable ? 1 : 0) | (restart ? 2 : 0) | (vis ? 4 : 0));
	DisableIO();
}

static void send_setup(const ld_disc *d)
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_PPS);
	for (int i = 0; i < 128; i++) spi_w((uint16_t)((i << 8) | d->pps[i]));
	DisableIO();

	uint32_t words = (d->frame_bytes + 7) / 8;
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_PARAMS);
	spi_w((uint16_t)words);
	spi_w((uint16_t)(words >> 16));
	spi_w((uint16_t)d->field1_row);
	DisableIO();
}

static void send_position()
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_POS);
	spi_w((uint16_t)(int16_t)pos_x);
	spi_w((uint16_t)(int16_t)pos_y);
	DisableIO();
	sent_pos = 1;
}

static void send_commit(int slot)
{
	spi_uio_cmd_cont(UIO_LD_VSET);
	spi_w(LDV_OP_COMMIT);
	spi_w((uint16_t)slot);
	DisableIO();
}

// 0 if the core doesn't have the commit-based player
static int read_status()
{
	uint16_t magic = spi_uio_cmd_cont(UIO_LD_VGET);
	if (magic != LDV_MAGIC)
	{
		DisableIO();
		return 0;
	}
	for (int i = 0; i < 7; i++) st[i] = spi_w(0);
	DisableIO();
	return st[6] == LDV_VERSION;
}

// ------------------------------------------------------------ reader thread

static int slot_of(int frame)
{
	for (int i = 0; i < SLOTS; i++) if (slots[i].frame == frame) return i;
	return -1;
}

static bool pinned(int i)
{
	return slots[i].loading || (slots[i].committed && (int32_t)(slots[i].seq - done) >= 0);
}

// next frame to load and the slot for it; called with mtx held
static bool pick_job(int &frame, int &slot)
{
	int order[PREDICT + 2];
	int n = 0;
	if (pending_frame >= 0) order[n++] = pending_frame;
	for (int i = 0; i < want_n; i++) order[n++] = want[i];

	for (int k = 0; k < n; k++)
	{
		int f = order[k];
		if (slot_of(f) >= 0) continue;
		// a victim: not in use, not wanted ahead of f; least recently used
		int best = -1;
		for (int i = 0; i < SLOTS; i++)
		{
			if (pinned(i)) continue;
			bool keep = false;
			for (int j = 0; j < k; j++) if (slots[i].frame == order[j]) keep = true;
			if (keep) continue;
			if (best < 0 || slots[i].frame < 0 || (slots[best].frame >= 0 && slots[i].used < slots[best].used)) best = i;
			if (slots[best].frame < 0) break;
		}
		if (best < 0) return false;
		frame = f;
		slot = best;
		return true;
	}
	return false;
}

// Main runs on core 1 (core 0 takes Linux's interrupts), and threads inherit
// that. This reader's copies into uncached DDR are heavy enough to hold the
// main loop up for a time slice now and then, so it runs on core 0, as Main's
// offload worker does.
static int start_on_core0(pthread_t *t, void *(*fn)(void *))
{
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0, &set);
	pthread_attr_setaffinity_np(&attr, sizeof(set), &set);
	int r = pthread_create(t, &attr, fn, NULL);
	pthread_attr_destroy(&attr);
	if (r) r = pthread_create(t, NULL, fn, NULL);   // affinity refused: run anywhere
	return r;
}

static void *reader_thread(void *)
{
	pthread_setname_np(pthread_self(), "ld-frames");
	while (!reader_stop)
	{
		int f, s;
		pthread_mutex_lock(&mtx);
		bool job = vd && pick_job(f, s);
		if (job)
		{
			slots[s].frame = f;
			slots[s].loading = 1;
			slots[s].committed = 0;
			slots[s].used = ++lru_clock;
		}
		const ld_disc *d = vd;
		pthread_mutex_unlock(&mtx);
		if (!job)
		{
			usleep(1000);
			continue;
		}

		uint32_t n = (d->frame_bytes + 7) & ~7u;
		uint64_t off = d->frames_off + (uint64_t)f * d->frame_stride;
		if (!mmi_read(d, stage, d->frame_bytes, off))
		{
			read_errors++;
			memset(stage, 0, n);
		}
		// the slots are the cache: drop the frame from Linux's page cache, or
		// at 12-16 MB/s it fills memory within a minute and every read after
		// that waits on evicting old pages (the audio's reads with it)
		posix_fadvise(d->fd, (off_t)off, (off_t)d->frame_bytes, POSIX_FADV_DONTNEED);
		memcpy(slot_map + (size_t)s * SLOT_SIZE, stage, n);
		__sync_synchronize();

		pthread_mutex_lock(&mtx);
		slots[s].loading = 0;
		loads++;
		pthread_mutex_unlock(&mtx);
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

// ------------------------------------------------------------------ public

static void reset_cache()
{
	for (int i = 0; i < SLOTS; i++) slots[i] = { -1, 0, 0, 0, 0 };
	want_n = 0;
	pending_frame = -1;
	pending_req = -1;
	pushed = done = 0;
	for (int i = 0; i < REQS; i++) reqs[i].id = -1;
}

static int start(const ld_disc *d)
{
	if (d->frame_bytes > SLOT_SIZE)
	{
		ld_log("video: frames of %u bytes don't fit the %u-byte slots", d->frame_bytes, SLOT_SIZE);
		return 0;
	}
	if (!read_status())
	{
		ld_log("video: this core has no LaserDisc video player (version %u, need %u)", st[6], LDV_VERSION);
		printf("LDV: this core has no LaserDisc video player matching this Main\n");
		return 0;
	}
	if (!slot_map) slot_map = (uint8_t *)shmem_map(SLOT_BASE, SLOTS * SLOT_SIZE);
	if (!stage) stage = (uint8_t *)malloc(SLOT_SIZE);
	if (!slot_map || !stage)
	{
		ld_log("video: cannot map DDR3");
		return 0;
	}

	reader_halt();
	pthread_mutex_lock(&mtx);
	reset_cache();
	vd = d;
	pthread_mutex_unlock(&mtx);
	read_errors = loads = 0;

	send_control(0, 1, 0);
	send_setup(d);
	send_position();
	send_control(1, 0, 0);
	enabled = 1;
	core_ok = 1;
	visible = 0;
	sent_visible = 0;

	if (start_on_core0(&reader, reader_thread) == 0) reader_running = 1;
	unsigned bpp16 = ((d->pps[4] & 3) << 8) | d->pps[5];
	ld_log("video: %u frames, %u bytes per frame (%.4g bpp)", d->frame_count, d->frame_bytes, bpp16 / 16.0);
	return 1;
}

static void stop()
{
	reader_halt();
	pthread_mutex_lock(&mtx);
	vd = NULL;
	reset_cache();
	pthread_mutex_unlock(&mtx);
	if (enabled) send_control(0, 1, 0);
	enabled = 0;
	visible = 0;
	sent_visible = 0;
}

int ld_video_attach(const ld_disc *d)
{
	return start(d);
}

void ld_video_detach()
{
	stop();
}

int ld_video_attached()
{
	return vd != NULL;
}

int ld_video_request(int frame)
{
	if (!vd || frame < 0 || (uint32_t)frame >= vd->frame_count) return -1;
	pthread_mutex_lock(&mtx);
	if (pending_req >= 0) reqs[pending_req % REQS].state = 2;   // replaced
	int id = next_req++;
	reqs[id % REQS].id = id;
	reqs[id % REQS].state = 0;
	pending_req = id;
	pending_frame = frame;
	pthread_mutex_unlock(&mtx);
	return id;
}

int ld_video_request_done(int req)
{
	if (req < 0) return 1;
	if (req < next_req - REQS) return 1;
	int r = 1;
	pthread_mutex_lock(&mtx);
	if (reqs[req % REQS].id == req)
	{
		if (reqs[req % REQS].state == 0) r = 0;
		else if (reqs[req % REQS].state == 1) r = (int32_t)(reqs[req % REQS].seq - done) < 0;
	}
	pthread_mutex_unlock(&mtx);
	return r;
}

void ld_video_predict(int frame, int step)
{
	if (!vd) return;
	pthread_mutex_lock(&mtx);
	want_n = 0;
	if (step == 0) step = 1;   // stills: keep the neighbours handy
	for (int k = 1; k <= PREDICT; k++)
	{
		int f = frame + step * k;
		if (f < 0 || (uint32_t)f >= vd->frame_count) break;
		want[want_n++] = f;
	}
	pthread_mutex_unlock(&mtx);
}

void ld_video_set_visible(int v)
{
	visible = v ? 1 : 0;
}

void ld_video_set_position(int x, int y)
{
	if (x < -32768) x = -32768;
	if (x > 32767) x = 32767;
	if (y < -128) y = -128;
	if (y > 127) y = 127;
	if (x != pos_x || y != pos_y) sent_pos = 0;
	pos_x = x;
	pos_y = y;
}

void ld_video_poll()
{
	static uint32_t timer = 0, report = 0;
	if (!vd) return;
	if (timer && !CheckTimer(timer)) return;
	timer = GetTimer(2);

	if (!read_status())
	{
		if (core_ok) ld_log("video: the core stopped answering");
		core_ok = 0;
		return;
	}
	core_ok = 1;


	pthread_mutex_lock(&mtx);
	done += (uint16_t)(st[0] - (uint16_t)done);
	for (int i = 0; i < REQS; i++)
		if (reqs[i].id >= 0 && reqs[i].state == 1 && (int32_t)(reqs[i].seq - done) < 0) reqs[i].state = 2;

	// commit the pending frame once its slot is loaded and the core has room
	int level = (st[4] >> 1) & 7;
	if (pending_frame >= 0 && level < 3)
	{
		int s = slot_of(pending_frame);
		if (s >= 0 && !slots[s].loading)
		{
			slots[s].committed = 1;
			slots[s].seq = pushed;
			slots[s].used = ++lru_clock;
			if (pending_req >= 0)
			{
				reqs[pending_req % REQS].state = 1;
				reqs[pending_req % REQS].seq = pushed;
			}
			pushed++;
			pending_frame = -1;
			pending_req = -1;
			pthread_mutex_unlock(&mtx);
			send_commit(s);
			pthread_mutex_lock(&mtx);
		}
	}
	pthread_mutex_unlock(&mtx);

	if (!sent_pos) send_position();
	if (visible != sent_visible)
	{
		send_control(1, 0, visible);
		sent_visible = visible;
	}

	if (!report || CheckTimer(report))
	{
		report = GetTimer(10000);
		ld_log("video: %u commits, %u decoded, %u late, %u skipped, %u decode errors, decode %.1f ms, %u loads, %u read errors%s",
			pushed, done, st[1], st[5], st[2], st[3] * 256 / 53693.175, loads, read_errors, visible ? "" : ", hidden");
	}
}
