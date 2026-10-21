// LaserActive: a record of why MiSTer_LD stopped.
//
// On a crash (segfault, bus error, illegal instruction, abort, divide by
// zero) the handler appends a report to /media/fat/laseractive_crash.log:
// the signal and faulting address, the registers at the fault (ARM), the
// thread, a backtrace, the memory map (to turn addresses into functions with
// `addr2line -f -e MiSTer.elf ADDR` on the unstripped build) and the last 100
// lines of laseractive.log. Then it lets the crash happen as before.
//
// Leaving any other way (a restart into another Main, exit) is noted in
// laseractive.log.
//
// Everything in the handler is async-signal-safe: open/write/read and
// preformatted text, no stdio or malloc.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <execinfo.h>
#include <ucontext.h>
#include <sys/prctl.h>
#include <sys/syscall.h>

#include "megald.h"

#define RING_LINES 100
#define RING_LEN   160

static char ring[RING_LINES][RING_LEN];
static volatile unsigned ring_next;
static char crash_path[1024];
static volatile sig_atomic_t in_crash;

// laseractive.log lines are copied here as they're written
void ld_crash_note(const char *line)
{
	unsigned i = ring_next++ % RING_LINES;
	size_t n = strlen(line);
	if (n >= RING_LEN) n = RING_LEN - 1;
	memcpy(ring[i], line, n);
	ring[i][n] = 0;
}

// --- async-signal-safe output

static void put(int fd, const char *s) { if (s) { ssize_t r = write(fd, s, strlen(s)); (void)r; } }

static void put_hex(int fd, unsigned long v)
{
	char b[2 + 2 * sizeof(v) + 1];
	int n = 2 * (int)sizeof(v);
	b[0] = '0'; b[1] = 'x';
	for (int i = 0; i < n; i++) b[2 + i] = "0123456789abcdef"[(v >> (4 * (n - 1 - i))) & 15];
	b[2 + n] = 0;
	put(fd, b);
}

static void put_dec(int fd, long v)
{
	char b[24];
	int i = 23;
	b[i] = 0;
	int neg = v < 0;
	unsigned long u = neg ? -(unsigned long)v : (unsigned long)v;
	do { b[--i] = (char)('0' + u % 10); u /= 10; } while (u && i > 1);
	if (neg) b[--i] = '-';
	put(fd, b + i);
}

static const char *sig_name(int s)
{
	switch (s)
	{
	case SIGSEGV: return "SIGSEGV (bad memory access)";
	case SIGBUS:  return "SIGBUS (bus error)";
	case SIGILL:  return "SIGILL (illegal instruction)";
	case SIGFPE:  return "SIGFPE (arithmetic error)";
	case SIGABRT: return "SIGABRT (abort)";
	default:      return "signal";
	}
}

static void handler(int sig, siginfo_t *si, void *uc_)
{
	if (in_crash) _exit(128 + sig);   // crashed while reporting
	in_crash = 1;

	int fd = open(crash_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
	if (fd >= 0)
	{
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		put(fd, "\n==== MiSTer_LD crashed at unix time "); put_dec(fd, (long)ts.tv_sec); put(fd, " ====\n");
		put(fd, "signal: "); put(fd, sig_name(sig)); put(fd, ", code "); put_dec(fd, si ? si->si_code : 0);
		put(fd, ", address "); put_hex(fd, si ? (unsigned long)si->si_addr : 0); put(fd, "\n");

		char tname[17] = {};
		prctl(PR_GET_NAME, tname, 0, 0, 0);
		put(fd, "thread: "); put(fd, tname); put(fd, " (tid "); put_dec(fd, (long)syscall(SYS_gettid)); put(fd, ")\n");

#if defined(__arm__)
		ucontext_t *uc = (ucontext_t *)uc_;
		if (uc)
		{
			put(fd, "pc "); put_hex(fd, uc->uc_mcontext.arm_pc);
			put(fd, "  lr "); put_hex(fd, uc->uc_mcontext.arm_lr);
			put(fd, "  sp "); put_hex(fd, uc->uc_mcontext.arm_sp);
			put(fd, "  fault "); put_hex(fd, uc->uc_mcontext.fault_address);
			put(fd, "\nr0 "); put_hex(fd, uc->uc_mcontext.arm_r0);
			put(fd, "  r1 "); put_hex(fd, uc->uc_mcontext.arm_r1);
			put(fd, "  r2 "); put_hex(fd, uc->uc_mcontext.arm_r2);
			put(fd, "  r3 "); put_hex(fd, uc->uc_mcontext.arm_r3);
			put(fd, "\n");
		}
#else
		(void)uc_;
#endif
		put(fd, "\nbacktrace (addr2line -f -e MiSTer.elf ADDR):\n");
		void *bt[48];
		int n = backtrace(bt, 48);
		backtrace_symbols_fd(bt, n, fd);

		put(fd, "\nmemory map:\n");
		int mfd = open("/proc/self/maps", O_RDONLY);
		if (mfd >= 0)
		{
			char b[1024];
			ssize_t r;
			while ((r = read(mfd, b, sizeof(b))) > 0) { ssize_t w = write(fd, b, (size_t)r); (void)w; }
			close(mfd);
		}

		put(fd, "\nlast lines of laseractive.log:\n");
		unsigned end = ring_next, start = end > RING_LINES ? end - RING_LINES : 0;
		for (unsigned i = start; i < end; i++) { put(fd, ring[i % RING_LINES]); put(fd, "\n"); }
		put(fd, "==== end ====\n");
		fsync(fd);
		close(fd);
	}

	// let it crash as it would have
	signal(sig, SIG_DFL);
	raise(sig);
}

static void at_exit()
{
	ld_log_if_open("MiSTer_LD is exiting");
}

void ld_crash_install(const char *crash_log_path)
{
	snprintf(crash_path, sizeof(crash_path), "%s", crash_log_path);

	// a separate stack, so a stack overflow can still be reported
	static char alt[64 * 1024];
	stack_t ss = {};
	ss.ss_sp = alt;
	ss.ss_size = sizeof(alt);
	sigaltstack(&ss, NULL);

	// backtrace() loads libgcc on first use: do that now, not in the handler
	void *warm[2];
	backtrace(warm, 2);

	struct sigaction sa = {};
	sa.sa_sigaction = handler;
	sa.sa_flags = SA_SIGINFO | SA_ONSTACK;
	sigemptyset(&sa.sa_mask);
	for (int s : { SIGSEGV, SIGBUS, SIGILL, SIGFPE, SIGABRT }) sigaction(s, &sa, NULL);

	atexit(at_exit);
}
