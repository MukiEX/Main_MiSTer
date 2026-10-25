// LaserActive (Mega-LD / Sega PAC) support for the MegaCD core: the
// LaserDisc player. See megald.h.
//
// The player logic is a port of Nemesis's MCD::LD from ares
// (ares/md/mcd/megald.cpp, commit 9408cb4), which is based on extensive
// hardware tests. Function and variable names follow ares so the two can be
// compared; the comments here only cover what differs:
//
//  * ares computes output registers when the CPU reads them. Here they are
//    computed after every change and sent to the core, which serves the
//    reads. The core handles what has to be exact per read itself: the
//    echo registers, the output freeze snapshot, the hold after a CPU write,
//    and a busy bit on output 0x06 from the moment a command is written
//    until this module has processed it (rtl/MCD/PD6103A.vhd).
//  * ares delays drive-state changes on output 0x06 by 10 reads; here the
//    delay is 20 ms.
//  * The disc position is the CD drive's (megacdd.cpp, ld_mode), as in ares.
//    Video frames are committed to the core's decoder; a seek counts as
//    finished when its first frame is on screen.
//  * Picture stop codes come from the .dscv VBI table instead of decoding
//    the picture.
//  * Video field selection and the VDP/LD mixing faders are not done yet:
//    the LaserDisc picture is shown full screen whenever the player would
//    show it, otherwise the Mega Drive picture is.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <math.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include "../../file_io.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../hardware.h"
#include "megacd.h"
#include "megald.h"
#include "megald_mmi.h"
#include "megald_video.h"

// Core-specific HPS commands, answered by rtl/hps_ext.v in the MegaCD core.
#define UIO_LD_GET 0x50
#define UIO_LD_SET 0x51

#define LD_SET_OP_CONTROL 0
#define LD_SET_OP_OUTREGS 1
#define LD_SET_OP_ACK     2
#define LD_SET_OP_TOC     3

#define LD_LOG_GA_RESET   0x7F5A   // gate-array reset marker in the access log

#define LD_LOG_FILE  "laseractive.log"
#define LD_REGS_FILE "laseractive_pd6103a.log"
#define LD_CFG_FILE  "laseractive.cfg"

static const double videoFramesPerSecond = 30.0 / 1.001;

// ------------------------------------------------------------------ logging

static FILE *log_fp = NULL;
static FILE *regs_fp = NULL;
static int cfg_log_regs = 0;
static int cfg_log_reads = 0;
static double cfg_video_x = 0;   // LD picture position, LD columns
static int cfg_video_y = 0;      // ... and lines
static time_t cfg_mtime = 0;

static const char *root_path(const char *name)
{
	static char path[1024];
	snprintf(path, sizeof(path), "%s/%s", getRootDir(), name);
	return path;
}

// The logs are written by their own thread ("ld-log", on core 0). Writing
// from the main loop meant every line waited on the SD card, and with
// log_regs/log_reads on that held the drive tick up long enough to starve the
// audio FIFO and change the very timing being logged. Now a line is only
// copied into a queue here; if the queue ever fills, lines are dropped and
// counted, never waited for.

#define LQ_SLOTS 4096
#define LQ_LEN   320

struct lq_slot { uint8_t file; char text[LQ_LEN]; };
static lq_slot lq[LQ_SLOTS];
static unsigned lq_head, lq_tail;          // head: next to write; tail: next free
static unsigned lq_dropped;
static pthread_mutex_t lq_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t lq_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t lq_write_mtx = PTHREAD_MUTEX_INITIALIZER;   // one writer at a time
static pthread_t lq_thread;
static int lq_started;

static int cfg_log_sectors = 0;

static void lq_write_batch()
{
	static lq_slot batch[256];
	for (;;)
	{
		pthread_mutex_lock(&lq_mtx);
		unsigned n = 0, dropped = lq_dropped;
		lq_dropped = 0;
		while (lq_head != lq_tail && n < 256) batch[n++] = lq[lq_head++ % LQ_SLOTS];
		pthread_mutex_unlock(&lq_mtx);
		if (!n && !dropped) return;
		for (unsigned i = 0; i < n; i++)
		{
			FILE *fp = batch[i].file ? regs_fp : log_fp;
			if (fp) { fputs(batch[i].text, fp); fputc('\n', fp); }
		}
		if (dropped && log_fp) fprintf(log_fp, "%8.3f  log: %u lines were dropped (the card fell behind)\n", GetTimer(0) / 1000.0, dropped);
		if (n < 256) break;
	}
	if (log_fp) fflush(log_fp);
	if (regs_fp) fflush(regs_fp);
}

static void *lq_thread_fn(void *)
{
	pthread_setname_np(pthread_self(), "ld-log");
	for (;;)
	{
		pthread_mutex_lock(&lq_mtx);
		if (lq_head == lq_tail && !lq_dropped)
		{
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 250 * 1000000L;
			if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
			pthread_cond_timedwait(&lq_cond, &lq_mtx, &ts);
		}
		pthread_mutex_unlock(&lq_mtx);
		pthread_mutex_lock(&lq_write_mtx);
		lq_write_batch();
		pthread_mutex_unlock(&lq_write_mtx);
	}
	return NULL;
}

static void lq_start()
{
	if (lq_started) return;
	lq_started = 1;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0, &set);
	pthread_attr_setaffinity_np(&attr, sizeof(set), &set);
	if (pthread_create(&lq_thread, &attr, lq_thread_fn, NULL) && pthread_create(&lq_thread, NULL, lq_thread_fn, NULL))
		lq_started = 0;   // no thread: lines are written where they're logged
	pthread_attr_destroy(&attr);
}

// file 0: laseractive.log, 1: the PD6103A log. text is already timestamped.
static void lq_push(int file, const char *text)
{
	if (!lq_started)
	{
		FILE *fp = file ? regs_fp : log_fp;
		if (fp) { fputs(text, fp); fputc('\n', fp); fflush(fp); }
		return;
	}
	pthread_mutex_lock(&lq_mtx);
	if (lq_tail - lq_head >= LQ_SLOTS) lq_dropped++;
	else
	{
		lq_slot &s = lq[lq_tail++ % LQ_SLOTS];
		s.file = (uint8_t)file;
		snprintf(s.text, LQ_LEN, "%s", text);
		// wake the writer for the main log, or when the register log piles up
		if (!file || lq_tail - lq_head > LQ_SLOTS / 4) pthread_cond_signal(&lq_cond);
	}
	pthread_mutex_unlock(&lq_mtx);
}

// Everything queued, on the disk now (exit, restart).
static void lq_flush_now()
{
	pthread_mutex_lock(&lq_write_mtx);
	lq_write_batch();
	pthread_mutex_unlock(&lq_write_mtx);
}

static void log_line(const char *text)
{
	char line[1100];
	snprintf(line, sizeof(line), "%8.3f  %s", GetTimer(0) / 1000.0, text);
	lq_push(0, line);
	ld_crash_note(line);
}

void ld_log(const char *fmt, ...)
{
	if (!log_fp)
	{
		// keep the previous run's log (it may show why that run ended)
		char old_path[1100];
		snprintf(old_path, sizeof(old_path), "%s.1", root_path(LD_LOG_FILE));
		rename(root_path(LD_LOG_FILE), old_path);
		log_fp = fopen(root_path(LD_LOG_FILE), "w");
		if (!log_fp) return;
		lq_start();
	}
	char text[1024];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(text, sizeof(text), fmt, ap);
	va_end(ap);
	log_line(text);
}

// The last words of a run (exit, restart): written out before returning.
void ld_log_if_open(const char *text)
{
	if (!log_fp) return;
	log_line(text);
	lq_flush_now();
}

int ld_log_sectors() { return cfg_log_sectors; }

void ld_crash_setup()
{
	ld_crash_install(root_path("laseractive_crash.log"));
}

// ------------------------------------------------------------------ helpers

static uint8_t bcd_dec(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 15)); }
static uint8_t bcd_enc(unsigned v) { v %= 100; return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static int bit(uint8_t v, int n) { return (v >> n) & 1; }

static void msf_from(int frames, uint8_t &m, uint8_t &s, uint8_t &f)
{
	if (frames < 0) frames = -frames;
	m = (uint8_t)(frames / (75 * 60));
	s = (uint8_t)((frames / 75) % 60);
	f = (uint8_t)(frames % 75);
}

// --------------------------------------------------------------- the disc

static ld_disc disc;
static int disc_ok = 0;
static int disc_has_video = 0;

static unsigned long last_seek_ms;
long ld_ms_since_seek() { return last_seek_ms ? (long)(GetTimer(0) - last_seek_ms) : -1; }

const ld_disc *ld_get_disc() { return disc_ok ? &disc : NULL; }

// ----------------------------------------------------------- player state

enum { SP_Chapter, SP_HoursOrFrameH, SP_MinutesOrFrameM, SP_SecondsOrFrameL, SP_Frames };
enum { SeekToRedbookTime, SeekToRedbookRelativeTime, SeekToVideoFrame, SeekToVideoTime };

static struct
{
	uint8_t inputRegs[32];
	uint8_t inputFrozenRegs[32];
	uint8_t outputRegs[32];
	bool areInputRegsFrozen;

	bool operationErrorFlag1, operationErrorFlag2, operationErrorFlag3;
	bool seekEnabled;
	uint8_t currentSeekMode, currentSeekModeTimeFormat;
	uint8_t stopPointRegs[5];
	uint8_t seekPointRegs[5];
	bool reachedStopPoint, reachedStopPointPreviously;
	uint8_t activeSeekMode;
	uint8_t currentPlaybackMode, currentPlaybackSpeed;
	bool currentPlaybackDirection;
	uint8_t targetDriveState, currentDriveState;
	bool targetPauseState, currentPauseState;
	bool seekPerformedSinceLastFrameUpdate;
	uint8_t selectedTrackInfo;

	uint8_t analogAudioAttenuationLeft, analogAudioAttenuationRight;
	bool analogAudioFadeToMutedLeft, analogAudioFadeToMutedRight;

	bool stopPointEnabled;   // mcd.cdd.stopPointEnabled
	int targetStopPoint;     // mcd.cdd.targetStopPoint (ABA)
	int sectorRepeatCount;   // mcd.cdd.io.sectorRepeatCount

	// video
	int frameSkipBaseFrame, frameSkipCounter;
	int currentVideoFrameIndex;
	bool currentVideoFrameLeadIn, currentVideoFrameLeadOut;
	bool digitalMemoryFrameLatched;
	bool frameEverRequested;

	// output 0x06 drive-state delay (ares: 10 reads)
	bool driveDelayActive;
	uint32_t driveDelayUntil;

	// a seek is reported busy until its first frame is on screen:
	// -2 waiting for the first frame update, >= 0 the video request
	int seekWaitRequest;
} P;

// driven output registers, as last sent to the core
static uint8_t sent_out[32];
static uint8_t sent_valid[32];
static int need_apply = 1;
static int toc_sig_reset = 0;
static uint32_t pending_events = 0;   // bit n: output reg n set by an event, send it

static void cdd_seek_lba(int lba, bool startPaused)
{
	cdd.LDSeek(lba, startPaused ? 0 : 1);
}

static int current_aba() { return cdd.GetLBA() + 150; }
static bool isDiscLoaded() { return cdd.loaded != 0; }
static bool isDiscLaserdisc() { return isDiscLoaded() && cdd.ld_mode && disc_ok; }
static bool isLaserdiscClv() { return isDiscLaserdisc() && disc.clv; }
static bool isLaserdiscDigitalAudioPresent() { return isDiscLaserdisc(); }   // every .mmi side has a Redbook stream
static int firstTrack() { return 1; }
static int lastTrack() { return cdd.TrackCount(); }
static int trackCount() { return cdd.TrackCount(); }
static int currentTrack()
{
	int i = cdd.TrackOfLBA(cdd.GetLBA());
	if (i >= cdd.TrackCount()) i = cdd.TrackCount() - 1;
	return i + 1;
}
static bool isTrackAudio(int track) { return track >= 1 && track <= trackCount() && !cdd.TrackIsData(track - 1); }

static void getTrackTocData(int track, uint8_t &flags, uint8_t &m, uint8_t &s, uint8_t &f)
{
	if (track < 1 || track > trackCount()) { flags = 0; m = s = f = 0; return; }
	flags = cdd.TrackIsData(track - 1) ? 0x4 : 0x0;
	msf_from(cdd.TrackStart(track - 1) + 150, m, s, f);
}

static void seekToTime(int h, int m, int s, int f, bool startPaused)
{
	int aba = (((h * 60) + m) * 60 + s) * 75 + f;
	cdd_seek_lba(aba - 150, startPaused);
}

static void seekToRelativeTime(int track, int m, int s, int f, bool startPaused)
{
	if (track < firstTrack() || track > lastTrack()) return;
	cdd_seek_lba(cdd.TrackStart(track - 1) + m * 60 * 75 + s * 75 + f, startPaused);
}

static void seekToTrack(int track, bool startPaused)
{
	if (track < 1 || track > trackCount()) track = 1;
	cdd_seek_lba(cdd.TrackStart(track - 1), startPaused);
}

static int zeroBasedFrameIndexFromABA(int aba, bool processLeadIn = false)
{
	if (!processLeadIn && aba < 0) return 0;
	int frameIndex = (int)lround(((double)aba / 75.0) * videoFramesPerSecond);
	if (processLeadIn && aba < 0) frameIndex = (-frameIndex) - 1;
	return frameIndex;
}

static int abaFromZeroBasedFrameIndex(int frameIndex)
{
	return (int)lround(((double)frameIndex / videoFramesPerSecond) * 75.0);
}

static void VideoTimeToRedbookTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint8_t &frames)
{
	int videoTimeTotalFrames = ((((hours * 60) + minutes) * 60) + seconds) * 30 + frames;
	int t = (int)lround(((double)videoTimeTotalFrames / videoFramesPerSecond) * 75.0);
	frames = t % 75; t /= 75;
	seconds = t % 60; t /= 60;
	minutes = t % 60; t /= 60;
	hours = t;
}

// .dscv stream index of a player frame, or -1
static int stream_index(int idx, bool leadIn, bool leadOut)
{
	if (!disc_has_video || idx < 0) return -1;
	int n;
	if (leadIn) n = idx < disc.lead_in ? idx : -1;
	else if (leadOut) n = idx < disc.lead_out ? disc.lead_in + disc.active + idx : -1;
	else n = idx < disc.active ? disc.lead_in + idx : -1;
	return (n >= 0 && (uint32_t)n < disc.frame_count) ? n : -1;
}

// ----------------------------------------------------- forward declarations

static void processInputRegisterWrite(int regNum, uint8_t data, uint8_t previousData, bool wasDeferredRegisterWrite);
static void updateStopPointWithCurrentState();
static void resetSeekTargetToDefault();
static bool liveSeekRegistersContainsLatchableTarget();
static bool latchSeekTargetFromCurrentState();
static void performSeekWithLatchedState();
static void handleStopPointReached(int aba);

static void set_event_output(int r, uint8_t v)
{
	P.outputRegs[r] = v;
	pending_events |= 1u << r;
}

// ------------------------------------------------------------------- power

static void power(bool gateArrayReset)
{
	for (int i = 0; i < 0x0C; i++) P.inputRegs[i] = 0;
	if (!gateArrayReset) for (int i = 0x0C; i < 32; i++) P.inputRegs[i] = 0;
	memset(P.inputFrozenRegs, 0, sizeof(P.inputFrozenRegs));
	memset(P.outputRegs, 0, sizeof(P.outputRegs));
	P.areInputRegsFrozen = false;
	P.operationErrorFlag1 = P.operationErrorFlag2 = P.operationErrorFlag3 = false;
	P.seekEnabled = false;
	P.currentSeekMode = 0;
	P.currentSeekModeTimeFormat = 0;
	memset(P.stopPointRegs, 0, sizeof(P.stopPointRegs));
	P.reachedStopPoint = false;
	P.reachedStopPointPreviously = false;
	P.activeSeekMode = 0;
	P.currentPlaybackMode = 0;
	P.currentPlaybackSpeed = 0;
	P.currentPlaybackDirection = false;
	memset(P.seekPointRegs, 0, sizeof(P.seekPointRegs));
	P.targetDriveState = 0;
	P.currentPauseState = false;
	P.targetPauseState = false;
	P.seekPerformedSinceLastFrameUpdate = false;
	P.seekWaitRequest = -1;
	P.driveDelayActive = false;
	P.selectedTrackInfo = 0;
	P.stopPointEnabled = false;
	P.targetStopPoint = 0;
	P.sectorRepeatCount = 0;

	P.currentDriveState = 0x02;
	P.targetDriveState = P.currentDriveState;
	if (!gateArrayReset)
	{
		P.inputRegs[0x1A] = 0xFF;
		P.inputRegs[0x1B] = 0xFF;
		P.inputRegs[0x1C] = 0xFF;
		P.inputRegs[0x1D] = 0xFF;
		P.analogAudioAttenuationLeft = P.analogAudioAttenuationRight = 0;
		P.analogAudioFadeToMutedLeft = P.analogAudioFadeToMutedRight = false;

		P.currentVideoFrameIndex = -99999999;
		P.currentVideoFrameLeadIn = P.currentVideoFrameLeadOut = false;
		P.digitalMemoryFrameLatched = false;
		P.frameEverRequested = false;
		P.frameSkipBaseFrame = 0;
		P.frameSkipCounter = 0;
	}

	// the core resets its copy of the output block too
	memset(sent_valid, 0, sizeof(sent_valid));
	pending_events = 0;
	need_apply = 1;
}

// ---------------------------------------------------------- input writes

// MCD::LD::write for the input block. The output block and the freeze of
// the output block are the core's business.
static void write_input(int regNum, uint8_t data)
{
	if (regNum == 0x00)
	{
		if (P.areInputRegsFrozen) P.inputFrozenRegs[regNum] = data;

		bool previousInputRegsFrozenState = P.areInputRegsFrozen;
		P.areInputRegsFrozen = !bit(data, 7);
		if (!P.areInputRegsFrozen && previousInputRegsFrozenState)
		{
			uint8_t seekRegsPreviousState[6];
			for (int i = 0; i < 6; i++)
			{
				seekRegsPreviousState[i] = P.inputRegs[0x06 + i];
				P.inputRegs[0x06 + i] = P.inputFrozenRegs[0x06 + i];
			}
			if (liveSeekRegistersContainsLatchableTarget()) latchSeekTargetFromCurrentState();
			processInputRegisterWrite(0x02, P.inputFrozenRegs[0x02], P.inputRegs[0x02], true);
			processInputRegisterWrite(0x03, P.inputFrozenRegs[0x03], P.inputRegs[0x03], true);
			for (int i = 0; i < 32; i++)
			{
				uint8_t previousData = (i >= 0x06 && i <= 0x0B) ? seekRegsPreviousState[i - 0x06] : P.inputRegs[i];
				processInputRegisterWrite(i, P.inputFrozenRegs[i], previousData, true);
			}
		}
		else if (P.areInputRegsFrozen && !previousInputRegsFrozenState)
		{
			memcpy(P.inputFrozenRegs, P.inputRegs, sizeof(P.inputRegs));
		}
	}

	if (P.areInputRegsFrozen)
	{
		P.inputFrozenRegs[regNum] = data;
		if (regNum >= 0x19) processInputRegisterWrite(regNum, data, P.inputRegs[regNum], false);
	}
	else
	{
		processInputRegisterWrite(regNum, data, P.inputRegs[regNum], false);
	}
}

static void drive_play_or_pause(bool pause)
{
	if (pause) cdd.LDPause(); else cdd.LDPlay();
}

static void processInputRegisterWrite(int regNum, uint8_t data, uint8_t previousData, bool wasDeferredRegisterWrite)
{
	switch (regNum)
	{
	case 0x00:
		if (bit(previousData, 0) != bit(data, 0))
		{
			P.operationErrorFlag1 = P.operationErrorFlag2 = P.operationErrorFlag3 = false;
			P.reachedStopPoint = false;
			if (P.seekEnabled)
			{
				if (liveSeekRegistersContainsLatchableTarget())
				{
					if (latchSeekTargetFromCurrentState()) performSeekWithLatchedState();
				}
				else
				{
					performSeekWithLatchedState();
				}
			}

			int livePlaybackMode = P.inputRegs[0x03] >> 4;
			int livePlaybackSpeed = P.inputRegs[0x03] & 7;
			if (livePlaybackMode == 0x01 && P.currentPlaybackMode == 0x01 && livePlaybackSpeed == 0x01 && P.currentPlaybackSpeed == 0x01 && P.frameSkipCounter == -1)
			{
				P.frameSkipBaseFrame = 0;
				P.frameSkipCounter = 0;
			}
			else if (livePlaybackMode == 0x02 && P.currentPlaybackMode == 0x02 && livePlaybackSpeed == 0x01 && P.currentPlaybackSpeed == 0x00)
			{
				P.currentPlaybackSpeed = 0x01;
			}

			if (P.currentDriveState == 0x05) drive_play_or_pause(P.currentPauseState);
		}
		break;

	case 0x02:
	{
		if ((data & 0x3F) == (previousData & 0x3F)) break;

		P.seekEnabled = bit(data, 5);
		P.operationErrorFlag1 = P.operationErrorFlag2 = P.operationErrorFlag3 = false;

		int newDriveState = data & 0x0F;
		bool pauseFlag = bit(data, 4);
		bool previousPauseFlag = bit(previousData, 4);
		if (pauseFlag != previousPauseFlag)
		{
			if (newDriveState == 0x0 || newDriveState == 0x5 || newDriveState == 0x7)
			{
				P.targetPauseState = pauseFlag;
				P.currentPauseState = P.targetPauseState;
			}
		}

		P.targetDriveState = newDriveState;
		int before = P.currentDriveState;

		switch (P.targetDriveState)
		{
		case 0x01:
			cdd.LDOpenTray();
			P.currentDriveState = 0x01;
			resetSeekTargetToDefault();
			break;
		case 0x02:
			cdd.LDStop();
			P.currentDriveState = 0x02;
			resetSeekTargetToDefault();
			break;
		case 0x03:
			cdd.LDStop();
			P.currentDriveState = 0x03;
			resetSeekTargetToDefault();
			break;
		case 0x04:
			if (!isDiscLoaded())
			{
				P.operationErrorFlag1 = P.operationErrorFlag2 = true;
				break;
			}
			if (P.currentDriveState <= 3)
			{
				resetSeekTargetToDefault();
				seekToTrack(1, true);
				P.seekPerformedSinceLastFrameUpdate = true;
				P.currentPauseState = true;
			}
			if (P.currentDriveState < 0x04) P.currentDriveState = 0x04;
			else P.operationErrorFlag1 = true;
			break;
		case 0x05:
		{
			if (!isDiscLoaded())
			{
				P.operationErrorFlag1 = P.operationErrorFlag2 = true;
				break;
			}
			if (P.seekEnabled) P.reachedStopPoint = false;
			bool performedLoadOfDisc = false;
			if (P.currentDriveState <= 3)
			{
				resetSeekTargetToDefault();
				seekToTrack(1, true);
				P.seekPerformedSinceLastFrameUpdate = true;
				P.currentPauseState = true;
				performedLoadOfDisc = true;
			}
			P.currentPauseState = P.targetPauseState;
			P.currentDriveState = 0x05;
			if (P.seekEnabled && (performedLoadOfDisc || !wasDeferredRegisterWrite))
			{
				if (liveSeekRegistersContainsLatchableTarget())
				{
					if (latchSeekTargetFromCurrentState()) performSeekWithLatchedState();
				}
				else
				{
					performSeekWithLatchedState();
				}
			}
			drive_play_or_pause(P.currentPauseState || P.reachedStopPoint);
			break;
		}
		case 0x00:
		case 0x07:
			if (!isDiscLoaded())
			{
				if (P.targetDriveState == 0x07) P.operationErrorFlag1 = P.operationErrorFlag2 = true;
				break;
			}
			if (P.currentDriveState == 0x05) drive_play_or_pause(P.targetPauseState || P.reachedStopPoint);
			break;
		default:
			P.operationErrorFlag1 = true;
			if (!isDiscLoaded()) P.operationErrorFlag2 = true;
			break;
		}
		ld_log("drive request %02X: state %X -> %X%s%s, seek %s", data, before, P.currentDriveState,
			P.currentPauseState ? ", paused" : "", P.operationErrorFlag1 ? ", error" : "", P.seekEnabled ? "on" : "off");
		break;
	}

	case 0x03:
	{
		if (data == previousData) break;
		P.operationErrorFlag1 = P.operationErrorFlag2 = P.operationErrorFlag3 = false;
		int newPlaybackMode = data >> 4;
		if (newPlaybackMode >= 0x04)
		{
			P.operationErrorFlag1 = true;
		}
		else
		{
			int newPlaybackSpeed = data & 7;
			bool newPlaybackDirection = bit(data, 3);
			switch (newPlaybackMode)
			{
			case 0: newPlaybackSpeed = 0; break;
			case 1: newPlaybackDirection = false; break;
			case 2: newPlaybackDirection = (newPlaybackSpeed == 0) ? false : newPlaybackDirection; break;
			case 3: if (newPlaybackSpeed == 0x07) newPlaybackSpeed = 0x06; break;
			}
			P.currentPlaybackMode = newPlaybackMode;
			P.currentPlaybackSpeed = newPlaybackSpeed;
			P.currentPlaybackDirection = newPlaybackDirection;
			P.reachedStopPoint = false;
			if (P.currentDriveState == 0x05) drive_play_or_pause(P.currentPauseState);
			ld_log("playback mode %02X", data);
		}
		break;
	}

	case 0x05:
		P.selectedTrackInfo = (data < 0x99) ? bcd_dec(data) : data;
		break;

	case 0x06:
	{
		bool seekModeUpdated = false;
		int targetSeekMode = data & 3;
		if ((targetSeekMode == 1 || targetSeekMode == 2) && P.currentSeekMode != targetSeekMode)
		{
			P.currentSeekMode = targetSeekMode;
			seekModeUpdated = true;
		}
		int targetSeekModeTimeFormat = bit(data, 2);
		if ((targetSeekMode == 1 || targetSeekMode == 2) && P.currentSeekModeTimeFormat != targetSeekModeTimeFormat)
		{
			P.currentSeekModeTimeFormat = targetSeekModeTimeFormat;
			seekModeUpdated = true;
		}
		// (as in ares, the helpers below still see the previous value of this
		// register: it is updated at the end)
		if (targetSeekMode == 3)
		{
			if ((previousData & 7) != (data & 7)) updateStopPointWithCurrentState();
		}
		else if (seekModeUpdated && P.seekEnabled && P.currentDriveState == 0x5 &&
			((P.currentSeekMode == 1 && P.currentSeekModeTimeFormat == 1) || P.currentSeekMode == 2))
		{
			if (latchSeekTargetFromCurrentState()) performSeekWithLatchedState();
		}
		break;
	}

	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0B:
	{
		P.inputRegs[regNum] = data;
		int targetSeekMode = P.inputRegs[0x06] & 3;
		if (targetSeekMode == 3 && previousData != data) updateStopPointWithCurrentState();
		if (!wasDeferredRegisterWrite && P.seekEnabled && P.currentDriveState == 0x5 &&
			((P.currentSeekMode == 1 && P.currentSeekModeTimeFormat == 1) || P.currentSeekMode == 2))
		{
			if (latchSeekTargetFromCurrentState()) performSeekWithLatchedState();
		}
		break;
	}

	case 0x1E:
		P.inputRegs[regNum] = data;
		if (bit(P.inputRegs[0x1E], 1) && bit(P.inputRegs[0x1E], 3))
		{
			if (!bit(P.inputRegs[0x1E], 2))
			{
				P.analogAudioFadeToMutedLeft = true;
				P.analogAudioFadeToMutedRight = true;
			}
			else if (!bit(P.inputRegs[0x1E], 0)) P.analogAudioFadeToMutedLeft = true;
			else P.analogAudioFadeToMutedRight = true;
		}
		// falls through, as in ares
		__attribute__((fallthrough));
	case 0x1F:
		if (bit(P.inputRegs[0x1E], 1) && !bit(P.inputRegs[0x1E], 3))
		{
			if (!bit(P.inputRegs[0x1E], 2))
			{
				P.analogAudioAttenuationLeft = data;
				P.analogAudioAttenuationRight = data;
				P.analogAudioFadeToMutedLeft = false;
				P.analogAudioFadeToMutedRight = false;
			}
			else if (!bit(P.inputRegs[0x1E], 0))
			{
				P.analogAudioAttenuationLeft = data;
				P.analogAudioFadeToMutedLeft = false;
			}
			else
			{
				P.analogAudioAttenuationRight = data;
				P.analogAudioFadeToMutedRight = false;
			}
		}
		break;

	default:
		break;
	}

	P.inputRegs[regNum] = data;
}

// ------------------------------------------------------- seek and stop points

static void updateStopPointWithCurrentState()
{
	if (P.inputRegs[0x07] == 0xFF)
	{
		for (int r = 0x1A; r <= 0x1E; r++) set_event_output(r, 0xFF);
		set_event_output(0x1F, 0x01);
		P.stopPointEnabled = false;
		ld_log("stop point cleared");
		return;
	}

	P.stopPointRegs[SP_Chapter] = (P.inputRegs[0x07] == 0x00) ? 0xFF : P.inputRegs[0x07];
	P.stopPointRegs[SP_HoursOrFrameH] = !isDiscLaserdisc() ? 0x00 : (P.inputRegs[0x08] & 0x0F);
	P.stopPointRegs[SP_MinutesOrFrameM] = P.inputRegs[0x09];
	P.stopPointRegs[SP_SecondsOrFrameL] = P.inputRegs[0x0A];
	P.stopPointRegs[SP_Frames] = (isDiscLaserdisc() && !isLaserdiscClv()) ? 0x00 : P.inputRegs[0x0B];
	set_event_output(0x1A, P.stopPointRegs[SP_Chapter]);
	set_event_output(0x1B, P.stopPointRegs[SP_Frames]);
	set_event_output(0x1C, P.stopPointRegs[SP_SecondsOrFrameL]);
	set_event_output(0x1D, P.stopPointRegs[SP_MinutesOrFrameM]);
	set_event_output(0x1E, P.stopPointRegs[SP_HoursOrFrameH]);
	set_event_output(0x1F, 0x01);

	int stopAba;
	if (!isDiscLaserdisc())
	{
		stopAba = (bcd_dec(P.stopPointRegs[SP_MinutesOrFrameM]) * 60 + bcd_dec(P.stopPointRegs[SP_SecondsOrFrameL])) * 75 + bcd_dec(P.stopPointRegs[SP_Frames]);
	}
	else if (!isLaserdiscClv())
	{
		int frameNumber = bcd_dec(P.stopPointRegs[SP_HoursOrFrameH]) * 10000 + bcd_dec(P.stopPointRegs[SP_MinutesOrFrameM]) * 100 + bcd_dec(P.stopPointRegs[SP_SecondsOrFrameL]);
		stopAba = abaFromZeroBasedFrameIndex(frameNumber - 1);
	}
	else
	{
		uint8_t h = bcd_dec(P.stopPointRegs[SP_HoursOrFrameH]), m = bcd_dec(P.stopPointRegs[SP_MinutesOrFrameM]);
		uint8_t s = bcd_dec(P.stopPointRegs[SP_SecondsOrFrameL]), f = bcd_dec(P.stopPointRegs[SP_Frames]);
		VideoTimeToRedbookTime(h, m, s, f);
		stopAba = (((h * 60) + m) * 60 + s) * 75 + f;
	}

	P.targetStopPoint = stopAba;
	P.stopPointEnabled = true;
	P.reachedStopPointPreviously = false;
	ld_log("stop point set: aba %d, frame %d", stopAba, zeroBasedFrameIndexFromABA(stopAba, true) + 1);

	if (P.reachedStopPoint) handleStopPointReached(current_aba());
}

static void resetSeekTargetToDefault()
{
	P.currentSeekMode = 0x1;
	P.currentSeekModeTimeFormat = 0x1;
	memset(P.seekPointRegs, 0, sizeof(P.seekPointRegs));
	P.activeSeekMode = SeekToRedbookRelativeTime;
}

static bool liveSeekRegistersContainsLatchableTarget()
{
	int targetSeekMode = P.inputRegs[0x06] & 3;
	return !(targetSeekMode == 3 || targetSeekMode == 0);
}

static bool latchSeekTargetFromCurrentState()
{
	if (!liveSeekRegistersContainsLatchableTarget()) return false;

	P.currentSeekMode = P.inputRegs[0x06] & 3;
	P.currentSeekModeTimeFormat = bit(P.inputRegs[0x06], 2);

	if (!P.reachedStopPoint) P.operationErrorFlag1 = P.operationErrorFlag2 = P.operationErrorFlag3 = false;

	if (!isDiscLoaded() || P.currentDriveState <= 0x04 || P.targetDriveState != 0x05)
	{
		P.operationErrorFlag1 = true;
		P.operationErrorFlag2 = false;
		P.operationErrorFlag3 = true;
		return false;
	}

	if (P.currentSeekMode == 1)
	{
		uint8_t targetTrack = P.inputRegs[0x07];
		int targetTrackAsInt = (isDiscLaserdisc() && targetTrack == 0x00) ? firstTrack() : bcd_dec(targetTrack);
		if (targetTrack > 0x99 || targetTrackAsInt < firstTrack() || targetTrackAsInt > lastTrack())
		{
			P.operationErrorFlag1 = true;
			P.operationErrorFlag2 = false;
			P.operationErrorFlag3 = true;
			P.reachedStopPoint = false;
			return false;
		}
		bool allowSeekToTime = !isDiscLaserdisc() || (!P.currentSeekModeTimeFormat && isLaserdiscDigitalAudioPresent());
		P.seekPointRegs[SP_Chapter] = targetTrack;
		P.seekPointRegs[SP_HoursOrFrameH] = 0x00;
		P.seekPointRegs[SP_MinutesOrFrameM] = allowSeekToTime ? P.inputRegs[0x09] : 0x00;
		P.seekPointRegs[SP_SecondsOrFrameL] = allowSeekToTime ? P.inputRegs[0x0A] : 0x00;
		P.seekPointRegs[SP_Frames] = allowSeekToTime ? P.inputRegs[0x0B] : 0x00;
		P.activeSeekMode = SeekToRedbookRelativeTime;
	}

	if (P.currentSeekMode == 2)
	{
		bool ld = isDiscLaserdisc(), clv = isLaserdiscClv(), da = isLaserdiscDigitalAudioPresent();
		bool seekToRedbookTimecode = !ld || (!P.currentSeekModeTimeFormat && ld);
		bool seekToVideoFrame = ld && P.currentSeekModeTimeFormat && !clv;
		bool seekToVideoTime = ld && ((!clv && !da) || (clv && da));
		if (seekToRedbookTimecode)
		{
			P.seekPointRegs[SP_Chapter] = 0x00;
			P.seekPointRegs[SP_HoursOrFrameH] = 0x00;
			P.seekPointRegs[SP_MinutesOrFrameM] = P.inputRegs[0x09];
			P.seekPointRegs[SP_SecondsOrFrameL] = P.inputRegs[0x0A];
			P.seekPointRegs[SP_Frames] = P.inputRegs[0x0B];
			P.activeSeekMode = SeekToRedbookTime;
		}
		else if (seekToVideoFrame)
		{
			P.seekPointRegs[SP_Chapter] = 0x00;
			P.seekPointRegs[SP_HoursOrFrameH] = P.inputRegs[0x08];
			P.seekPointRegs[SP_MinutesOrFrameM] = P.inputRegs[0x09];
			P.seekPointRegs[SP_SecondsOrFrameL] = P.inputRegs[0x0A];
			P.seekPointRegs[SP_Frames] = 0x00;
			P.activeSeekMode = SeekToVideoFrame;
		}
		else if (seekToVideoTime)
		{
			P.seekPointRegs[SP_Chapter] = 0x00;
			P.seekPointRegs[SP_HoursOrFrameH] = P.inputRegs[0x08] & 0x0F;
			P.seekPointRegs[SP_MinutesOrFrameM] = P.inputRegs[0x09];
			P.seekPointRegs[SP_SecondsOrFrameL] = P.inputRegs[0x0A];
			P.seekPointRegs[SP_Frames] = P.inputRegs[0x0B];
			P.activeSeekMode = SeekToVideoTime;
		}
	}
	return true;
}

static int predicted_step();

static void performSeekWithLatchedState()
{
	P.frameSkipBaseFrame = 0;
	P.frameSkipCounter = 0;

	bool paused = P.targetPauseState && !P.reachedStopPoint;
	int before = cdd.GetLBA();
	switch (P.activeSeekMode)
	{
	case SeekToRedbookTime:
		seekToTime(0, bcd_dec(P.seekPointRegs[SP_MinutesOrFrameM]), bcd_dec(P.seekPointRegs[SP_SecondsOrFrameL]), bcd_dec(P.seekPointRegs[SP_Frames]), paused);
		break;
	case SeekToRedbookRelativeTime:
		seekToRelativeTime(bcd_dec(P.seekPointRegs[SP_Chapter]), bcd_dec(P.seekPointRegs[SP_MinutesOrFrameM]),
			bcd_dec(P.seekPointRegs[SP_SecondsOrFrameL]), bcd_dec(P.seekPointRegs[SP_Frames]), paused);
		break;
	case SeekToVideoFrame:
	{
		int frameNumber = bcd_dec(P.seekPointRegs[SP_HoursOrFrameH]) * 10000 + bcd_dec(P.seekPointRegs[SP_MinutesOrFrameM]) * 100 + bcd_dec(P.seekPointRegs[SP_SecondsOrFrameL]);
		int aba = abaFromZeroBasedFrameIndex((frameNumber > 1 ? frameNumber : 1) - 1);
		cdd_seek_lba(aba - 150, paused);
		break;
	}
	case SeekToVideoTime:
	{
		uint8_t h = bcd_dec(P.seekPointRegs[SP_HoursOrFrameH]), m = bcd_dec(P.seekPointRegs[SP_MinutesOrFrameM]);
		uint8_t s = bcd_dec(P.seekPointRegs[SP_SecondsOrFrameL]), f = bcd_dec(P.seekPointRegs[SP_Frames]);
		VideoTimeToRedbookTime(h, m, s, f);
		seekToTime(h, m, s, f, paused);
		break;
	}
	}
	P.seekPerformedSinceLastFrameUpdate = true;
	P.seekWaitRequest = -2;

	last_seek_ms = GetTimer(0);
	ld_audio_prefetch(cdd.GetLBA());
	int aba = current_aba();
	ld_log("seek (mode %d): lba %d -> %d, frame %d%s", P.activeSeekMode, before, cdd.GetLBA(),
		zeroBasedFrameIndexFromABA(aba, true) + 1, paused ? ", paused" : "");

	// start loading the target frame while the drive "seeks"
	int fi = zeroBasedFrameIndexFromABA(aba, true);
	bool li = aba < 0, lo = !li && fi >= disc.active;
	int si = stream_index(lo ? fi - disc.active : fi, li, lo);
	if (si >= 0) ld_video_predict(si - predicted_step(), predicted_step());
}

static void handleStopPointReached(int aba)
{
	if (P.currentPlaybackMode == 2 && P.currentPlaybackSpeed <= 1) return;

	if (bit(P.inputRegs[0x06], 7))
	{
		ld_log("stop point reached at frame %d: repeat", zeroBasedFrameIndexFromABA(aba, true) + 1);
		P.reachedStopPoint = false;
		cdd.LDPlay();
		performSeekWithLatchedState();
	}
	else
	{
		ld_log("stop point reached at frame %d: stop", zeroBasedFrameIndexFromABA(aba, true) + 1);
		for (int r = 0x1A; r <= 0x1E; r++) set_event_output(r, 0xFF);
		set_event_output(0x1F, 0x00);
		P.reachedStopPointPreviously = true;
		P.operationErrorFlag1 = false;
		P.operationErrorFlag2 = true;
		P.operationErrorFlag3 = true;
		P.reachedStopPoint = true;
		cdd.LDPause();
		P.stopPointEnabled = false;
	}
}

// ------------------------------------------------------------ video frames

// Frames the picture is expected to move per new frame, for prefetching
static int predicted_step()
{
	int dir = P.currentPlaybackDirection ? -1 : 1;
	switch (P.currentPlaybackMode)
	{
	case 1:
		return P.frameSkipCounter > 1 ? P.frameSkipCounter : 1;
	case 2:
		return P.currentPlaybackSpeed >= 2 ? dir : (P.currentPlaybackSpeed == 1 ? dir : 0);
	case 3:
	{
		static const int sectors[8] = { 1, 2, 3, 8, 14, 20, 1, 1 };
		int f = (int)lround(sectors[P.currentPlaybackSpeed & 7] * videoFramesPerSecond / 75.0 * 2.5);
		return dir * (f > 0 ? f : 1);
	}
	default:
		return 1;
	}
}

static void updateCurrentVideoFrameNumber(int aba)
{
	bool justCompletedSeekFirstFrame = P.seekPerformedSinceLastFrameUpdate;
	P.seekPerformedSinceLastFrameUpdate = false;
	bool seekFirstFrame = P.seekWaitRequest == -2;
	if (seekFirstFrame) P.seekWaitRequest = -1;

	int newVideoFrameIndex = zeroBasedFrameIndexFromABA(aba, true);
	bool newVideoFrameLeadIn = aba < 0;
	bool newVideoFrameLeadOut = !newVideoFrameLeadIn && newVideoFrameIndex >= disc.active;
	if (newVideoFrameLeadOut) newVideoFrameIndex -= disc.active;

	if (newVideoFrameLeadIn)
	{
		if (newVideoFrameIndex >= disc.lead_in) newVideoFrameIndex = disc.lead_in > 0 ? disc.lead_in - 1 : 0;
	}
	else if (newVideoFrameLeadOut)
	{
		if (newVideoFrameIndex >= disc.lead_out) newVideoFrameIndex = disc.lead_out > 0 ? disc.lead_out - 1 : 0;
	}

	bool playerIsLoopingOnSingleFrame = P.currentDriveState == 0x05 && !P.currentPauseState && P.currentPlaybackMode == 0x2 && P.currentPlaybackSpeed == 0x0;
	bool newFrameIsSameAsLastFrame = newVideoFrameIndex == P.currentVideoFrameIndex && newVideoFrameLeadIn == P.currentVideoFrameLeadIn &&
		newVideoFrameLeadOut == P.currentVideoFrameLeadOut;
	if (!playerIsLoopingOnSingleFrame && newFrameIsSameAsLastFrame) return;

	if (!justCompletedSeekFirstFrame && P.currentPlaybackMode == 0x01)
	{
		int newFrameSkipCounter = 0;
		switch (P.currentPlaybackSpeed)
		{
		case 0x00: newFrameSkipCounter = 1; break;
		case 0x01: newFrameSkipCounter = -1; P.currentPlaybackDirection = !P.currentPlaybackDirection; break;
		case 0x02: newFrameSkipCounter = 2; break;
		case 0x03: newFrameSkipCounter = 4; break;
		case 0x04: newFrameSkipCounter = 8; break;
		case 0x05: newFrameSkipCounter = 16; break;
		case 0x06: newFrameSkipCounter = 30; break;
		case 0x07: newFrameSkipCounter = 90; break;
		}
		if (P.frameSkipBaseFrame == 0 || P.frameSkipCounter != newFrameSkipCounter)
		{
			P.frameSkipBaseFrame = newVideoFrameIndex;
			P.frameSkipCounter = newFrameSkipCounter;
		}
		else
		{
			int frameDeltaFromBase = abs(newVideoFrameIndex - P.frameSkipBaseFrame);
			if (P.frameSkipCounter == 0 || (frameDeltaFromBase % P.frameSkipCounter) != 0) return;
		}
	}
	else if (P.currentPlaybackMode == 0x02 && P.currentPlaybackSpeed == 0x01)
	{
		P.currentPlaybackSpeed = 0;
	}
	else
	{
		P.frameSkipBaseFrame = 0;
		P.frameSkipCounter = 0;
	}

	if ((bit(P.inputRegs[0x0C], 5) && P.digitalMemoryFrameLatched) || bit(P.inputRegs[0x0C], 2))
	{
		if (bit(P.inputRegs[0x0C], 2) && !bit(P.inputRegs[0x0C], 5)) P.digitalMemoryFrameLatched = false;
		return;
	}

	P.digitalMemoryFrameLatched = true;
	if (newFrameIsSameAsLastFrame) return;   // (no field selection yet)

	P.currentVideoFrameIndex = newVideoFrameIndex;
	P.currentVideoFrameLeadIn = newVideoFrameLeadIn;
	P.currentVideoFrameLeadOut = newVideoFrameLeadOut;

	int si = stream_index(newVideoFrameIndex, newVideoFrameLeadIn, newVideoFrameLeadOut);
	if (si >= 0)
	{
		int req = ld_video_request(si);
		P.frameEverRequested = true;
		if (seekFirstFrame && req >= 0) P.seekWaitRequest = req;
		ld_video_predict(si, predicted_step());

		// picture stop codes, from the .dscv VBI table (lines 16, 17, 279, 280)
		if (!disc.clv && !bit(P.inputRegs[0x0C], 7) && (P.currentPlaybackMode != 2 || P.currentPlaybackSpeed > 1) && !disc.vbi.empty())
		{
			const uint32_t *c = &disc.vbi[(size_t)si * 4];
			if (c[0] == 0x82CFFF || c[1] == 0x82CFFF || c[2] == 0x82CFFF || c[3] == 0x82CFFF)
			{
				ld_log("picture stop code at frame %d", newVideoFrameIndex + 1);
				P.reachedStopPointPreviously = true;
				P.operationErrorFlag1 = false;
				P.operationErrorFlag2 = false;
				P.operationErrorFlag3 = true;
				P.reachedStopPoint = true;
				cdd.LDPause();
				P.stopPointEnabled = false;
			}
		}
	}
}

// ---------------------------------------------------------- CD drive hooks

int ld_sector_advance()
{
	int sectorAdvanceOffset = 1;
	switch (P.currentPlaybackMode)
	{
	case 0x00:
	case 0x01:
		sectorAdvanceOffset = 1;
		P.sectorRepeatCount = 0;
		break;
	case 0x02:
	{
		static const int every[8] = { 0, 1, 2, 4, 8, 16, 30, 90 };
		int sp = P.currentPlaybackSpeed & 7;
		if (sp == 0) { sectorAdvanceOffset = 0; P.sectorRepeatCount = 0; }
		else if (sp == 1) { sectorAdvanceOffset = 1; P.sectorRepeatCount = 0; }
		else sectorAdvanceOffset = (++P.sectorRepeatCount >= every[sp]) ? 1 : 0;
		if (P.currentPlaybackDirection) sectorAdvanceOffset = -sectorAdvanceOffset;
		if (sp >= 2 && sectorAdvanceOffset != 0) P.sectorRepeatCount = 0;
		break;
	}
	case 0x03:
	{
		static const int sectors[8] = { 1, 2, 3, 8, 14, 20, 1, 1 };   // 6/7 (search) not implemented, as in ares
		sectorAdvanceOffset = sectors[P.currentPlaybackSpeed & 7];
		if (P.currentPlaybackDirection) sectorAdvanceOffset = -sectorAdvanceOffset;
		P.sectorRepeatCount = 0;
		break;
	}
	}
	return sectorAdvanceOffset;
}

void ld_sector_reached(int lba, int end_of_disc)
{
	if (end_of_disc || !isDiscLaserdisc()) return;
	int aba = lba + 150;
	updateCurrentVideoFrameNumber(aba);
	if (P.stopPointEnabled && aba == P.targetStopPoint) handleStopPointReached(aba);
}

static int16_t clamp16(int v) { return (int16_t)(v < -32768 ? -32768 : v > 32767 ? 32767 : v); }

void ld_mix_audio(int16_t *s, const int16_t *analog, int have_digital, int have_analog)
{
	const uint8_t *in = P.inputRegs;
	int analogMixingMode = in[0x01] >> 6;
	int audioMixingInputSelection = in[0x0D] >> 6;
	bool digitalAudioMixingDisabled = analogMixingMode > 0 && (audioMixingInputSelection == 2 || (analogMixingMode == 1 && bit(in[0x0D], 4)));
	bool digitalAudioAttenuationDisabled = analogMixingMode == 0 || audioMixingInputSelection == 3;
	bool analogAudioMixingDisabled = analogMixingMode == 0 || (analogMixingMode == 1 && !bit(in[0x0D], 4));
	if (analogAudioMixingDisabled) have_analog = 0;

	float digitalFader = (float)in[0x0F] / 255.0f;
	int volL = P.analogAudioFadeToMutedLeft ? 0 : 0x100 - P.analogAudioAttenuationLeft;
	int volR = P.analogAudioFadeToMutedRight ? 0 : 0x100 - P.analogAudioAttenuationRight;
	bool stepping = P.currentPlaybackMode == 0x02;

	for (int i = 0; i < 588; i++)
	{
		int dl = have_digital ? s[i * 2] : 0, dr = have_digital ? s[i * 2 + 1] : 0;
		if (digitalAudioMixingDisabled) dl = dr = 0;
		if (!digitalAudioAttenuationDisabled)
		{
			dl = (int16_t)((float)dl * digitalFader);
			dr = (int16_t)((float)dr * digitalFader);
		}
		if (analogMixingMode != 0 && bit(in[0x0D], 0) && bit(in[0x0D], 1)) { dl /= 2; dr /= 2; }
		else if (bit(in[0x0D], 0)) dr = dl;
		else if (bit(in[0x0D], 1)) dl = dr;

		int al = have_analog ? analog[i * 2] : 0, ar = have_analog ? analog[i * 2 + 1] : 0;
		if (bit(in[0x0E], 7) || (bit(in[0x0E], 0) && bit(in[0x0E], 1))) al = ar = 0;
		else if (bit(in[0x0E], 0)) ar = al;
		else if (bit(in[0x0E], 1)) al = ar;
		al = (al * volL) >> 8;
		ar = (ar * volR) >> 8;

		if (stepping) dl = dr = al = ar = 0;
		s[i * 2] = clamp16(dl + al);
		s[i * 2 + 1] = clamp16(dr + ar);
	}
}

// ------------------------------------------------------- output registers

// Outputs 0x11-0x14 for a track-info selection (input 0x05 after ares'
// decoding). The core answers from a table of these (see upload_toc), except
// for selections 0x00 and 0xFF (current position), which it takes live.
static uint8_t toc_reg(int regNum, uint8_t sel)
{
	uint8_t flags, minute, second, frame;
	uint8_t data = 0;
	if (regNum == 0x11)
	{
		if (sel > trackCount()) data = 0;
		else
		{
			int t = sel == 0 ? currentTrack() : sel;
			getTrackTocData(t, flags, minute, second, frame);
			data = (flags << 4) | 0x01;
		}
		return data;
	}
	int k = regNum - 0x12;
	uint8_t v[3];
	if (sel == 0xA0 || sel == 0xB0)
	{
		v[0] = bcd_enc(firstTrack()); v[1] = bcd_enc(trackCount()); v[2] = 0;
		return v[k];
	}
	if (sel == 0xA1 || sel == 0xB1) msf_from(cdd.DiscEnd() + 150, minute, second, frame);
	else if ((sel == 0 || sel == 0xFF) && trackCount() > 0) msf_from(cdd.GetLBA() + 150, minute, second, frame);
	else if (sel > trackCount()) return 0xFF;
	else getTrackTocData(sel, flags, minute, second, frame);
	v[0] = minute; v[1] = second; v[2] = frame;
	return bcd_enc(v[k]);
}

// MCD::LD::getOutputRegisterValue for the registers this side drives
static uint8_t getOutputRegisterValue(int regNum)
{
	uint8_t previousData = P.outputRegs[regNum];
	uint8_t data = previousData;
	uint8_t minute, second, frame;
	bool loaded = isDiscLoaded(), ld = isDiscLaserdisc(), clv = isLaserdiscClv();
	int ds = P.currentDriveState;

	switch (regNum)
	{
	case 0x02:
		data = 0;
		if (ds < 0x02) data = 0x00;
		else if (!loaded) data = 0x10;
		else if (ld) data = (ds >= 0x04) ? (clv ? 0xC6 : 0xC5) : 0x80;
		else data = 0x20;
		break;
	case 0x03:
		if (ds < 0x02) data = 0;
		else if (ds < 0x04) {}
		else
		{
			data = (data & ~0x08) | (loaded ? 0x08 : 0);
			data = (data & 0x3F) | ((loaded && ld) ? ((disc.side_no > 1 ? 0x2 : 0x1) << 6) : 0);
		}
		break;
	case 0x04:
		data = 0;
		if (!loaded || !ld || ds < 0x04) data |= 0x20;
		if (!loaded || ld || ds < 0x02) data |= bit(P.inputRegs[0x0D], 4) << 7;
		if (loaded && ld && ds >= 0x04) data |= isTrackAudio(currentTrack()) ? 0x0C : 0x0E;
		else if (loaded && !ld && ds >= 0x04) data |= 0x2;
		break;
	case 0x06:
	{
		uint32_t now = GetTimer(0);
		if ((previousData & 0x0F) != ds || P.driveDelayActive)
		{
			if (!P.driveDelayActive)
			{
				P.driveDelayActive = true;
				P.driveDelayUntil = now + 20;
			}
			else if ((int32_t)(now - P.driveDelayUntil) >= 0)
			{
				data = (data & 0xF0) | ds;
				P.driveDelayActive = false;
			}
		}
		bool playing = cdd.status == CD_STAT_PLAY;
		bool seekBusy = (data & 0x0F) == 5 && (cdd.LDSeeking() || (playing && (P.seekPerformedSinceLastFrameUpdate ||
			P.seekWaitRequest == -2 || (P.seekWaitRequest >= 0 && !ld_video_request_done(P.seekWaitRequest)))));
		if (P.seekWaitRequest >= 0 && ld_video_request_done(P.seekWaitRequest)) P.seekWaitRequest = -1;
		if (seekBusy) data |= 0xA0;
		else data = (data & ~0xA0) | (P.driveDelayActive ? 0x80 : 0);
		data = (data & ~0x10) | ((P.currentPauseState || P.targetPauseState) ? 0x10 : 0);
		break;
	}
	case 0x07:
		data = (P.currentPlaybackMode << 4) | (P.currentPlaybackDirection ? 0x08 : 0) | (P.currentPlaybackSpeed & 7);
		if (P.reachedStopPoint) data = 0x20;
		else if (P.currentPlaybackMode == 0x02 && P.currentPlaybackSpeed == 0x00) data &= ~0x08;
		break;
	case 0x08:
		data = (data & 0x3E) | ((loaded && ld && ds >= 5) ? 0x80 : 0) |
			((loaded && ((!ld && ds >= 2) || (ld && ds >= 4))) ? 0x40 : 0) | ((loaded && ds >= 4) ? 0x01 : 0);
		break;
	case 0x09:
		data = (data & ~0x13) | (P.operationErrorFlag1 ? 0x10 : 0) | (P.operationErrorFlag2 ? 0x02 : 0) | (P.operationErrorFlag3 ? 0x01 : 0);
		break;
	case 0x11:
	case 0x12:
	case 0x13:
	case 0x14:
		data = toc_reg(regNum, P.selectedTrackInfo);
		break;
	case 0x15:
		if (loaded && ds >= 5) data = bcd_enc(currentTrack());
		else if (loaded && ds >= 2) data = 0x01;
		else data = 0x00;
		break;
	case 0x16:
	case 0x17:
	case 0x18:
	case 0x19:
		data = 0;
		if (loaded && ds >= 5)
		{
			if (ld)
			{
				int n = zeroBasedFrameIndexFromABA(current_aba()) + 1;
				if (clv)
				{
					switch (regNum)
					{
					case 0x16: data = bcd_enc((n / (60 * 60 * 30)) % 60); break;
					case 0x17: data = bcd_enc((n / (60 * 30)) % 60); break;
					case 0x18: data = bcd_enc((n / 30) % 60); break;
					case 0x19: data = bcd_enc(n % 30); break;
					}
				}
				else
				{
					switch (regNum)
					{
					case 0x16: data = bcd_enc((n / 10000) % 100); break;
					case 0x17: data = bcd_enc((n / 100) % 100); break;
					case 0x18: data = bcd_enc(n % 100); break;
					case 0x19: data = 0x00; break;
					}
				}
			}
			else
			{
				int t = currentTrack();
				msf_from(cdd.GetLBA() - (t >= 1 ? cdd.TrackStart(t - 1) : 0), minute, second, frame);
				switch (regNum)
				{
				case 0x16: data = 0x01; break;
				case 0x17: data = bcd_enc(minute); break;
				case 0x18: data = bcd_enc(second); break;
				case 0x19: data = bcd_enc(frame); break;
				}
			}
		}
		break;
	default:
		break;
	}
	P.outputRegs[regNum] = data;
	return data;
}

static const uint8_t driven_regs[] = { 0x02, 0x03, 0x04, 0x06, 0x07, 0x08, 0x09, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19 };

// -------------------------------------------------------------- HPS <-> core

static void send_control(int clear_ovf)
{
	spi_uio_cmd_cont(UIO_LD_SET);
	spi_w(LD_SET_OP_CONTROL);
	spi_w((cfg_log_reads ? 1 : 0) | (clear_ovf ? 2 : 0));
	DisableIO();
}

static void send_outputs(const uint8_t *regs, const uint8_t *vals, int n)
{
	if (!n) return;
	spi_uio_cmd_cont(UIO_LD_SET);
	spi_w(LD_SET_OP_OUTREGS);
	for (int i = 0; i < n; i++) spi_w(((regs[i] & 0x1F) << 8) | vals[i]);
	DisableIO();
}

static void send_ack()
{
	spi_uio_cmd_cont(UIO_LD_SET);
	spi_w(LD_SET_OP_ACK);
	spi_w(0);
	DisableIO();
}

// The whole TOC table (outputs 0x11-0x14 for every input 0x05 value), so
// the core can answer the BIOS's TOC reads at once.
static void upload_toc()
{
	spi_uio_cmd_cont(UIO_LD_SET);
	spi_w(LD_SET_OP_TOC);
	for (int raw = 0; raw < 256; raw++)
	{
		uint8_t sel = (raw < 0x99) ? bcd_dec(raw) : raw;
		spi_w((uint16_t)((raw << 8) | toc_reg(0x11, sel)));
		spi_w((uint16_t)((toc_reg(0x12, sel) << 8) | toc_reg(0x13, sel)));
		spi_w((uint16_t)(toc_reg(0x14, sel) << 8));
	}
	DisableIO();
}

static void update_outputs()
{
	uint8_t regs[48], vals[48];
	int n = 0;
	for (uint8_t r : driven_regs)
	{
		uint8_t v = getOutputRegisterValue(r);
		if (!sent_valid[r] || sent_out[r] != v)
		{
			regs[n] = r; vals[n] = v; n++;
			sent_out[r] = v;
			sent_valid[r] = 1;
		}
	}
	// player status, whenever the drive state, play mode or error flags change
	static uint8_t last6 = 0xFF, last7 = 0xFF, last9 = 0xFF;
	if (P.outputRegs[6] != last6 || P.outputRegs[7] != last7 || P.outputRegs[9] != last9)
	{
		last6 = P.outputRegs[6]; last7 = P.outputRegs[7]; last9 = P.outputRegs[9];
		ld_log("status: out 06=%02X 07=%02X 09=%02X; drive lba %d (%s%s), frame no. %d; 0C=%02X",
			last6, last7, last9, cdd.GetLBA(),
			cdd.status == CD_STAT_PLAY ? "play" : cdd.status == CD_STAT_PAUSE ? "pause" : cdd.status == CD_STAT_SEEK ? "seek" :
			cdd.status == CD_STAT_STOP ? "stop" : cdd.status == CD_STAT_OPEN ? "open" : cdd.status == CD_STAT_END ? "end" : "other",
			cdd.latency ? ", seeking" : "", zeroBasedFrameIndexFromABA(current_aba()) + 1, P.inputRegs[0x0C]);
	}
	for (int r = 0x1A; r <= 0x1F; r++)
	{
		if (pending_events & (1u << r))
		{
			regs[n] = r; vals[n] = P.outputRegs[r]; n++;
		}
	}
	pending_events = 0;
	send_outputs(regs, vals, n);
}

static void load_config()
{
	struct stat st;
	const char *path = root_path(LD_CFG_FILE);
	if (stat(path, &st) != 0)
	{
		cfg_mtime = 0;
		cfg_log_regs = cfg_log_reads = cfg_log_sectors = 0;
		ld_video_set_position(0, 0);
		return;
	}
	if (st.st_mtime == cfg_mtime) return;
	cfg_mtime = st.st_mtime;

	FILE *fp = fopen(path, "r");
	if (!fp) return;
	cfg_log_regs = cfg_log_reads = cfg_log_sectors = 0;
	cfg_video_x = 0;
	cfg_video_y = 0;
	char line[256];
	int v;
	double d;
	while (fgets(line, sizeof(line), fp))
	{
		if (sscanf(line, " log_regs %d", &v) == 1) cfg_log_regs = v ? 1 : 0;
		else if (sscanf(line, " log_reads %d", &v) == 1) cfg_log_reads = v ? 1 : 0;
		else if (sscanf(line, " log_sectors %d", &v) == 1) cfg_log_sectors = v ? 1 : 0;
		else if (sscanf(line, " video_x %lf", &d) == 1) cfg_video_x = d;
		else if (sscanf(line, " video_y %d", &v) == 1) cfg_video_y = v;
	}
	fclose(fp);
	ld_video_set_position((int)(cfg_video_x * 256 + (cfg_video_x < 0 ? -0.5 : 0.5)), cfg_video_y);
	need_apply = 1;
}

static void log_entry(uint16_t e)
{
	if (!cfg_log_regs) return;
	if (!regs_fp)
	{
		regs_fp = fopen(root_path(LD_REGS_FILE), "w");
		if (!regs_fp) return;
		lq_start();
	}
	char line[96];
	if (e == LD_LOG_GA_RESET)
		snprintf(line, sizeof(line), "%8.3f  -- gate-array reset --", GetTimer(0) / 1000.0);
	else
	{
		int is_read = (e >> 15) & 1, is_out = (e >> 14) & 1, reg = (e >> 9) & 0x1F, alt = (e >> 8) & 1;
		snprintf(line, sizeof(line), "%8.3f  %c %s %02X = %02X%s", GetTimer(0) / 1000.0, is_read ? 'R' : 'W', is_out ? "out" : "in ", reg, e & 0xFF,
			(is_read && alt) ? "  (snapshot/held)" : "");
	}
	lq_push(1, line);
}

// ------------------------------------------------------------------ public

const ld_disc *ld_disc_mount(const char *path)
{
	ld_disc_unmount();
	char full[1024];
	if (path[0] == '/') snprintf(full, sizeof(full), "%s", path);
	else snprintf(full, sizeof(full), "%s/%s", getRootDir(), path);

	char err[128];
	if (!mmi_open(&disc, full, 0, err, sizeof(err)))
	{
		ld_log("cannot use %s: %s", full, err);
		printf("LD: cannot use %s: %s\n", full, err);
		return NULL;
	}
	disc_ok = 1;
	disc_has_video = disc.has_video && ld_video_attach(&disc);
	ld_log("mounted \"%s\" (%s), side %d of %d (side no. %d, %s), video %s: %d lead-in, %d active, %d lead-out frames%s, analog audio %s",
		disc.title.c_str(), full, disc.media_index + 1, disc.media_count, disc.side_no, disc.clv ? "CLV" : "CAV",
		disc.has_video ? (disc_has_video ? "yes" : "not shown (core has no LD player)") : "no (not a .dscmmi)",
		disc.lead_in, disc.active, disc.lead_out, disc.vbi.empty() ? "" : ", VBI table", disc.audio_size ? "yes" : "no");

	// ares MCD::LD::load
	P.currentDriveState = 0x02;
	P.targetDriveState = P.currentDriveState;
	P.currentVideoFrameIndex = -99999999;
	return &disc;
}

// The drive's digital audio, as the CD drive found it in the cue sheet, plus
// the analog soundtrack (aligned with the video: lead-in frames come first).
void ld_audio_attach(const std::vector<ld_audio_seg> &digital, const ld_audio_seg &data)
{
	if (!disc_ok) return;
	ld_audio_seg a = { -150, -150, 0, 2352, 0 };
	if (disc.audio_size)
	{
		uint32_t leading = (uint32_t)((double)disc.lead_in * (44100.0 / videoFramesPerSecond));
		uint64_t lead = (uint64_t)leading * 4;
		a.off = disc.audio_off + lead;
		if (disc.audio_size > lead) a.end = -150 + (int)((disc.audio_size - lead) / 2352);
	}
	ld_audio_start(disc.fd, digital, a, data);
}

void ld_disc_unmount()
{
	if (!disc_ok) return;
	ld_audio_stop();
	ld_video_detach();
	mmi_close(&disc);
	disc_ok = 0;
	disc_has_video = 0;
	ld_log("disc removed");
}

void ld_reset()
{
	power(false);
	ld_log("reset");
}

void ld_poll()
{
	static uint32_t poll_timer = 0;
	static uint32_t cfg_timer = 0;
	static int powered = 0;

	if (!powered)
	{
		powered = 1;
		power(false);
	}

	if (!cfg_timer || CheckTimer(cfg_timer))
	{
		cfg_timer = GetTimer(1000);
		load_config();
	}

	if (poll_timer && !CheckTimer(poll_timer)) return;
	poll_timer = GetTimer(1);

	if (need_apply)
	{
		need_apply = 0;
		send_control(1);
		memset(sent_valid, 0, sizeof(sent_valid));
		toc_sig_reset = 1;
		for (int r = 0x1A; r <= 0x1F; r++) pending_events |= 1u << r;
	}

	uint16_t st = spi_uio_cmd_cont(UIO_LD_GET);
	int count = st & 0x3FF;
	int ovf = (st >> 15) & 1;
	int pend = (st >> 14) & 1;
	if (count > 512)
	{
		DisableIO();   // not this core
		return;
	}
	static uint16_t entries[512];
	for (int i = 0; i < count; i++) entries[i] = spi_w(0);
	DisableIO();

	for (int i = 0; i < count; i++)
	{
		uint16_t e = entries[i];
		log_entry(e);
		if (e == LD_LOG_GA_RESET)
		{
			power(true);
			ld_log("gate-array reset");
		}
		else if (!(e & 0x8000) && !(e & 0x4000))
		{
			write_input((e >> 9) & 0x1F, e & 0xFF);
		}
	}
	if (ovf)
	{
		ld_log("PD6103A log overflowed: commands were lost");
		send_control(1);
	}
	// audio read-ahead: a line every 10 s while the disc plays
	static uint32_t audio_report = 0;
	static uint32_t audio_last = 0;
	if (disc_ok && (!audio_report || CheckTimer(audio_report)))
	{
		audio_report = GetTimer(10000);
		ld_audio_stats_t a;
		ld_audio_stats(&a);
		if (a.sectors != audio_last)
			ld_log("audio: %u sectors, %u read directly (seek/start), %u chunk loads, %u read errors, %u late (FIFO ran dry), longest gap %u ms",
				a.sectors, a.misses, a.chunk_loads, a.read_errors, a.late, a.worst_gap);
		audio_last = a.sectors;
	}

	// TOC table: whenever the disc (or its TOC) changes, and after resets
	static uint32_t toc_sig = 0;
	if (toc_sig_reset) { toc_sig = 0; toc_sig_reset = 0; }
	uint32_t sig = 1 + (cdd.loaded ? 1 : 0) + (uint32_t)trackCount() * 2 + (uint32_t)cdd.DiscEnd() * 256;
	if (sig != toc_sig)
	{
		toc_sig = sig;
		upload_toc();
	}

	update_outputs();
	if (pend) send_ack();

	// LaserDisc picture: shown when the player would mix it in and isn't blanking it
	if (disc_has_video)
	{
		bool blanked = bit(P.inputRegs[0x0C], 2) || (P.currentPauseState && !bit(P.inputRegs[0x0C], 5));
		bool mixing = (P.inputRegs[0x01] >> 6) != 0;
		ld_video_set_visible(mixing && !blanked && P.frameEverRequested && P.currentDriveState >= 4);
	}
}
