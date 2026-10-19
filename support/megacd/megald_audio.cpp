// LaserActive: read-ahead for the disc's audio. See megald_audio.h.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include "../../hardware.h"
#include "megald.h"
#include "megald_audio.h"

#define SECTOR   2352
#define CH       32          // sectors per chunk (0.43 s)
#define NCH      16          // chunks kept (6.8 s, 2.4 MB)
#define AHEAD    150         // sectors to keep read ahead at normal speed (2 s)

struct chunk
{
	int id;                  // (lba + 150) / CH, or -1
	int state;               // 0 empty, 1 loading, 2 ready
	uint32_t used;
	uint8_t dig_ok[CH], ana_ok[CH];
	uint8_t dig[CH][SECTOR], ana[CH][SECTOR];
};

static chunk *chunks;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static pthread_t reader;
static int running, stop_req;
static int src_fd = -1;
static std::vector<ld_audio_seg> dig_segs;
static ld_audio_seg ana_seg;
static int want_id = -1, want_dir = 1;
static uint32_t lru_clock;
static ld_audio_stats_t stats;

// per-event lines in laseractive.log, at most EVENTS per 10 s
#define EVENTS 20
static int event_ok()
{
	static unsigned long window;
	static int n, dropped;
	unsigned long now = GetTimer(0);
	if (now - window >= 10000)
	{
		if (dropped) ld_log("audio: (%d more events not logged)", dropped);
		window = now;
		n = dropped = 0;
	}
	if (n < EVENTS) { n++; return 1; }
	dropped++;
	return 0;
}
static int sent_idle = 1;     // next ld_audio_sent's gap doesn't count

static int chunk_of(int lba) { return lba < -150 ? -1 : (lba + 150) / CH; }

static bool read_at(int fd, uint8_t *b, size_t len, uint64_t off)
{
	uint64_t start = off;
	size_t total = len;
	while (len)
	{
		ssize_t r = pread(fd, b, len, (off_t)off);
		if (r < 0 && errno == EINTR) continue;
		if (r <= 0) return false;
		b += r; len -= r; off += r;
	}
	// our own copy is the cache: don't let these pages crowd the video out
	posix_fadvise(fd, (off_t)start, (off_t)total, POSIX_FADV_DONTNEED);
	return true;
}

// sectors [l0, l0 + n) of one source into buf (n * SECTOR bytes), ok[] per sector
static void read_source(int fd, const ld_audio_seg *segs, size_t nsegs, int l0, int n, uint8_t *buf, uint8_t *ok)
{
	memset(ok, 0, n);
	for (size_t i = 0; i < nsegs; i++)
	{
		int a = l0 > segs[i].start ? l0 : segs[i].start;
		int b = l0 + n < segs[i].end ? l0 + n : segs[i].end;
		if (a >= b) continue;
		if (read_at(fd, buf + (size_t)(a - l0) * SECTOR, (size_t)(b - a) * SECTOR, segs[i].off + (uint64_t)(a - segs[i].start) * SECTOR))
			memset(ok + (a - l0), 1, b - a);
		else
			stats.read_errors++;
	}
}

static chunk *find(int id)
{
	for (int i = 0; i < NCH; i++) if (chunks[i].id == id && chunks[i].state) return &chunks[i];
	return NULL;
}

// next chunk to load, in order of need; with mtx held
static chunk *pick(int &id)
{
	if (want_id < 0) return NULL;
	int s = want_dir < 0 ? -1 : 1;
	int speed = want_dir < 0 ? -want_dir : want_dir;
	if (speed < 1) speed = 1;
	int ahead = (speed * AHEAD + CH - 1) / CH;
	if (ahead > NCH - 3) ahead = NCH - 3;

	int order[NCH];
	int n = 0;
	for (int k = 0; k <= ahead; k++) order[n++] = want_id + s * k;
	order[n++] = want_id - s;

	for (int k = 0; k < n; k++)
	{
		if (order[k] < 0 || find(order[k])) continue;
		chunk *victim = NULL;
		for (int i = 0; i < NCH; i++)
		{
			chunk *c = &chunks[i];
			if (c->state == 1) continue;
			bool wanted = false;
			for (int j = 0; j < n; j++) if (c->state && c->id == order[j]) wanted = true;
			if (wanted) continue;
			if (!victim || !c->state || (victim->state && c->used < victim->used)) victim = c;
			if (!victim->state) break;
		}
		if (!victim) return NULL;
		id = order[k];
		return victim;
	}
	return NULL;
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
	pthread_mutex_lock(&mtx);
	while (!stop_req)
	{
		int id;
		chunk *c = pick(id);
		if (!c)
		{
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 20 * 1000000;
			if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
			pthread_cond_timedwait(&cond, &mtx, &ts);
			continue;
		}
		c->id = id;
		c->state = 1;
		c->used = ++lru_clock;
		pthread_mutex_unlock(&mtx);

		int l0 = id * CH - 150;
		read_source(src_fd, dig_segs.data(), dig_segs.size(), l0, CH, &c->dig[0][0], c->dig_ok);
		read_source(src_fd, &ana_seg, 1, l0, CH, &c->ana[0][0], c->ana_ok);

		pthread_mutex_lock(&mtx);
		c->state = 2;
		stats.chunk_loads++;
	}
	pthread_mutex_unlock(&mtx);
	return NULL;
}

void ld_audio_stop()
{
	if (running)
	{
		pthread_mutex_lock(&mtx);
		stop_req = 1;
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&mtx);
		pthread_join(reader, NULL);
		running = 0;
	}
	stop_req = 0;
	src_fd = -1;
	want_id = -1;
	if (chunks) for (int i = 0; i < NCH; i++) chunks[i].id = -1, chunks[i].state = 0;
}

void ld_audio_start(int fd, const std::vector<ld_audio_seg> &digital, const ld_audio_seg &analog)
{
	ld_audio_stop();
	if (!chunks) chunks = (chunk *)calloc(NCH, sizeof(chunk));
	if (!chunks) return;
	for (int i = 0; i < NCH; i++) chunks[i].id = -1, chunks[i].state = 0;
	src_fd = fd;
	dig_segs = digital;
	ana_seg = analog;
	want_id = -1;
	want_dir = 1;
	memset(&stats, 0, sizeof(stats));
	sent_idle = 1;
	if (start_on_core0(&reader, reader_thread) == 0) running = 1;
}

void ld_audio_get(int lba, int dir, int16_t *digital, int *have_digital, int16_t *analog, int *have_analog)
{
	*have_digital = *have_analog = 0;
	if (src_fd < 0) return;
	int id = chunk_of(lba);
	if (id < 0) return;

	pthread_mutex_lock(&mtx);
	stats.sectors++;
	if (id != want_id || dir != want_dir)
	{
		want_id = id;
		want_dir = dir;
		pthread_cond_signal(&cond);
	}
	chunk *c = running ? find(id) : NULL;
	if (c && c->state == 2)
	{
		int k = lba + 150 - id * CH;
		c->used = ++lru_clock;
		if ((*have_digital = c->dig_ok[k])) memcpy(digital, c->dig[k], SECTOR);
		if ((*have_analog = c->ana_ok[k])) memcpy(analog, c->ana[k], SECTOR);
		pthread_mutex_unlock(&mtx);
		return;
	}
	stats.misses++;
	int state = c ? c->state : 0;
	pthread_mutex_unlock(&mtx);

	// not read yet (just started or seeked): read this sector directly
	unsigned long t0 = GetTimer(0);
	uint8_t ok;
	read_source(src_fd, dig_segs.data(), dig_segs.size(), lba, 1, (uint8_t *)digital, &ok);
	*have_digital = ok;
	read_source(src_fd, &ana_seg, 1, lba, 1, (uint8_t *)analog, &ok);
	*have_analog = ok;
	unsigned long took = GetTimer(0) - t0;
	if (event_ok())
		ld_log("audio: read-ahead miss at lba %d (chunk %s, %ld ms after the last seek), read directly in %lu ms",
			lba, state == 1 ? "still loading" : "not started", ld_ms_since_seek(), took);
}

void ld_audio_idle() { sent_idle = 1; }

void ld_audio_sent(unsigned long ms, int lba)
{
	static unsigned long last;
	unsigned long gap = ms - last;
	last = ms;
	if (sent_idle) { sent_idle = 0; return; }   // was stopped, paused or seeking
	pthread_mutex_lock(&mtx);
	if (gap > 29) stats.late++;
	if (gap > stats.worst_gap) stats.worst_gap = gap;
	pthread_mutex_unlock(&mtx);
	if (gap > 29 && event_ok())
		ld_log("audio: FIFO ran dry at lba %d: %lu ms since the last sector (%ld ms after the last seek)", lba, gap, ld_ms_since_seek());
}

void ld_audio_stats(ld_audio_stats_t *s)
{
	pthread_mutex_lock(&mtx);
	*s = stats;
	pthread_mutex_unlock(&mtx);
}
