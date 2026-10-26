
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

#include "../../hardware.h"
#include "megacd.h"
#include "megald.h"
#include "megald_mmi.h"
#include "megald_video.h"
#include "../chd/mister_chd.h"

cdd_t cdd;

cdd_t::cdd_t() {
	latency = 10;
	loaded = 0;
	index = 0;
	lba = 0;
	scanOffset = 0;
	isData = 1;
	status = CD_STAT_NO_DISC;
	audioLength = 0;
	audioOffset = 0;
	chd_hunkbuf = NULL;
	chd_hunknum = -1;
	SendData = NULL;
	CanSendData = NULL;
	ld_mode = 0;
	ld_dir = 1;

	stat[0] = 0xB;
	stat[1] = 0x0;
	stat[2] = 0x0;
	stat[3] = 0x0;
	stat[4] = 0x0;
	stat[5] = 0x0;
	stat[6] = 0x0;
	stat[7] = 0x0;
	stat[8] = 0x0;
	stat[9] = 0x4;
}

static int sgets(char *out, int sz, char **in)
{
	*out = 0;
	do
	{
		char *instr = *in;
		int cnt = 0;

		while (*instr && *instr != 10)
		{
			if (*instr == 13)
			{
				instr++;
				continue;
			}

			if (cnt < sz - 1)
			{
				out[cnt++] = *instr;
				out[cnt] = 0;
			}

			instr++;
		}

		if(*instr == 10) instr++;
		*in = instr;
	}
	while (!*out && **in);

	return *out;
}


// A track file named by a cue sheet: a file next to the .cue, or for a
// LaserDisc image, an entry in the same archive folder as the cue entry.
int cdd_t::OpenTrackFile(fileTYPE *f, const char *name, int in_archive)
{
	if (!in_archive) return FileOpen(f, name);
	const ld_disc *d = ld_get_disc();
	const mmi_entry *e = d ? mmi_find(d, name) : NULL;
	if (!e)
	{
		printf("\x1b[32mMCD: %s is not in the archive\n\x1b[0m", name);
		return 0;
	}
	return FileOpenSub(f, d->path, e->off, e->size);
}

int cdd_t::LoadCUE(const char* filename) {
	static char toc[100 * 1024];

	memset(toc, 0, sizeof(toc));
	if (!FileLoad(filename, toc, sizeof(toc) - 1)) return 1;

	printf("\x1b[32mMCD: Open CUE: %s\n\x1b[0m", filename);
	return ParseCUE(toc, filename, 0);
}

int cdd_t::ParseCUE(char *toc, const char *cue_name, int in_archive) {
	static char fname[1024 + 10];
	static char line[128];
	char *ptr, *lptr;
	static char header[1024];

	snprintf(fname, sizeof(fname), "%s", cue_name);

	int mm, ss, bb, pregap = 0;

	char *buf = toc;
	while (sgets(line, sizeof(line), &buf))
	{
		lptr = line;
		while (*lptr == 0x20) lptr++;

		/* decode FILE commands */
		if (!(memcmp(lptr, "FILE", 4)))
		{
			ptr = fname + strlen(fname) - 1;
			while ((ptr - fname) && (*ptr != '/') && (*ptr != '\\')) ptr--;
			if (ptr - fname) ptr++;

			lptr += 4;
			while (*lptr == 0x20) lptr++;

			if (*lptr == '\"')
			{
				lptr++;
				while ((*lptr != '\"') && (lptr <= (line + 128)) && (ptr < (fname + 1023)))
					*ptr++ = *lptr++;
			}
			else
			{
				while ((*lptr != 0x20) && (lptr <= (line + 128)) && (ptr < (fname + 1023)))
					*ptr++ = *lptr++;
			}
			*ptr = 0;

			if(!OpenTrackFile(&this->toc.tracks[this->toc.last].f, fname, in_archive)) return -1;

			printf("\x1b[32mMCD: Open track file: %s\n\x1b[0m", fname);

			pregap = 0;

			this->toc.tracks[this->toc.last].offset = 0;

			if (!strstr(lptr, "BINARY") && !strstr(lptr, "MOTOROLA") && !strstr(lptr, "WAVE"))
			{
				FileClose(&this->toc.tracks[this->toc.last].f);
				printf("\x1b[32mMCD: unsupported file: %s\n\x1b[0m", fname);

				return -1;
			}
		}

		/* decode TRACK commands */
		else if ((sscanf(lptr, "TRACK %02d %*s", &bb)) || (sscanf(lptr, "TRACK %d %*s", &bb)))
		{
			if (bb != (this->toc.last + 1))
			{
				FileClose(&this->toc.tracks[this->toc.last].f);
				printf("\x1b[32mMCD: missing tracks: %s\n\x1b[0m", fname);
				break;
			}

			if (!this->toc.last)
			{
				if (strstr(lptr, "MODE1/2048"))
				{
					this->toc.tracks[0].sector_size = 2048;
				}
				else if (strstr(lptr, "MODE1/2352"))
				{
					this->toc.tracks[0].sector_size = 2352;

					FileSeek(&this->toc.tracks[0].f, 0x10, SEEK_SET);
				}

				if (this->toc.tracks[0].sector_size)
				{
					this->toc.tracks[0].type = TT_MODE1;

					FileReadAdv(&this->toc.tracks[0].f, header, 0x210);
					FileSeek(&this->toc.tracks[0].f, 0, SEEK_SET);
				}
			}
			else
			{
				if (!this->toc.tracks[this->toc.last].f.opened())
				{
					this->toc.tracks[this->toc.last - 1].end = 0;
				}
			}
		}

		/* decode PREGAP commands */
		else if (sscanf(lptr, "PREGAP %02d:%02d:%02d", &mm, &ss, &bb) == 3)
		{
			pregap += bb + ss * 75 + mm * 60 * 75;
		}

		/* decode INDEX commands */
		else if ((sscanf(lptr, "INDEX 00 %02d:%02d:%02d", &mm, &ss, &bb) == 3) ||
			(sscanf(lptr, "INDEX 0 %02d:%02d:%02d", &mm, &ss, &bb) == 3))
		{
			if (this->toc.last && !this->toc.tracks[this->toc.last - 1].end)
			{
				this->toc.tracks[this->toc.last - 1].end = bb + ss * 75 + mm * 60 * 75 + pregap;
			}
		}
		else if ((sscanf(lptr, "INDEX 01 %02d:%02d:%02d", &mm, &ss, &bb) == 3) ||
			(sscanf(lptr, "INDEX 1 %02d:%02d:%02d", &mm, &ss, &bb) == 3))
		{
			this->toc.tracks[this->toc.last].offset += pregap * 2352;

			if (!this->toc.tracks[this->toc.last].f.opened())
			{
				OpenTrackFile(&this->toc.tracks[this->toc.last].f, fname, in_archive);
				this->toc.tracks[this->toc.last].start = bb + ss * 75 + mm * 60 * 75 + pregap;
				if (this->toc.last && !this->toc.tracks[this->toc.last - 1].end)
				{
					this->toc.tracks[this->toc.last - 1].end = this->toc.tracks[this->toc.last].start;
				}
			}
			else
			{
				FileSeek(&this->toc.tracks[this->toc.last].f, 0, SEEK_SET);

				this->toc.tracks[this->toc.last].start = this->toc.end + pregap;
				this->toc.tracks[this->toc.last].offset += this->toc.end * 2352;

				int sectorSize = 2352;
				if (this->toc.tracks[this->toc.last].type) sectorSize = this->toc.tracks[0].sector_size;
				this->toc.tracks[this->toc.last].end = this->toc.tracks[this->toc.last].start + ((this->toc.tracks[this->toc.last].f.size + sectorSize - 1) / sectorSize);

				this->toc.tracks[this->toc.last].start += (bb + ss * 75 + mm * 60 * 75);
				this->toc.end = this->toc.tracks[this->toc.last].end;
			}

			printf("\x1b[32mMCD: Track = %u, start = %u, end = %u, offset = %u, type = %u\n\x1b[0m", cdd.toc.last, cdd.toc.tracks[cdd.toc.last].start, cdd.toc.tracks[cdd.toc.last].end, cdd.toc.tracks[cdd.toc.last].offset, cdd.toc.tracks[cdd.toc.last].type);

			this->toc.last++;
			if (this->toc.last == 99) break;
		}
	}

	if (this->toc.last && !this->toc.tracks[this->toc.last - 1].end)
	{
		this->toc.end += pregap;
		this->toc.tracks[this->toc.last - 1].end = this->toc.end;
	}

	if (!in_archive && strlen(fname) > 4)
	{
		memcpy(&fname[strlen(fname) - 4], ".sub", 4);
		FileOpen(&this->toc.sub, getFullPath(fname));
	}

	FileClose(&this->toc.tracks[this->toc.last].f);
	return 0;
}

int cdd_t::Load(const char *filename)
{
	//char fname[1024 + 10];
	static char header[32];
	fileTYPE *fd_img;

	Unload();
	ld_mode = 0;

	const char *ext = filename+strlen(filename)-4;
	if (mmi_is_archive_name(filename))
	{
		// LaserActive disc: the digital track is a cue sheet inside the archive
		static char cue[100 * 1024];
		const ld_disc *d = ld_disc_mount(filename);
		const mmi_entry *e = d ? mmi_find(d, d->redbook.c_str()) : NULL;
		memset(cue, 0, sizeof(cue));
		if (!e || e->size >= sizeof(cue) || !mmi_read(d, cue, e->size, e->off))
		{
			ld_disc_unmount();
			return (-1);
		}
		printf("\x1b[32mMCD: LaserDisc %s, digital track %s\n\x1b[0m", filename, d->redbook.c_str());
		if (ParseCUE(cue, d->redbook.c_str(), 1))
		{
			ld_disc_unmount();
			return (-1);
		}
		ld_mode = 1;
		ld_dir = 1;

	} else if (!strncasecmp(".cue", ext, 4))
	{
		ld_disc_unmount();
		if (LoadCUE(filename)) {
			return (-1);
		}
	} else if (!strncasecmp(".chd", ext, 4))  {
		ld_disc_unmount();
		chd_error err = mister_load_chd(filename, &this->toc);
		if (err != CHDERR_NONE)
		{
			printf("ERROR %s\n", chd_error_string(err));
			return -1;
		}

		if (this->chd_hunkbuf)
		{
			free(this->chd_hunkbuf);
		}

		this->chd_hunkbuf = (uint8_t *)malloc(this->toc.chd_hunksize);
		this->chd_hunknum = -1;
 	} else {
		return (-1);

	}

	if (this->toc.chd_f)
	{
		mister_chd_read_sector(this->toc.chd_f, 0, 0, 0, 0x10, (uint8_t *)header, this->chd_hunkbuf, &this->chd_hunknum);
	} else {
		fd_img = &this->toc.tracks[0].f;

		FileSeek(fd_img, 0, SEEK_SET);
		FileReadAdv(fd_img, header, 0x10);
	}

	if (this->toc.tracks[0].sector_size)
	{
		this->sectorSize = this->toc.tracks[0].sector_size;
	}
	else if (!memcmp("SEGADISCSYSTEM", header, 14))
	{
		this->sectorSize = 2048;
	}
	else
	{
		this->sectorSize = 2352;
	}

	printf("\x1b[32mMCD: Sector size = %u, Track 0 end = %u\n\x1b[0m", this->sectorSize, this->toc.tracks[0].end);

	if (this->ld_mode) LDStartReadAhead();

	if (this->toc.last)
	{
		this->toc.tracks[this->toc.last].start = this->toc.end;
		this->loaded = 1;

		printf("\x1b[32mMCD: CD mounted , last track = %u\n\x1b[0m", this->toc.last);

		return 1;
	}

	return 0;
}

void cdd_t::Unload()
{
	if (this->loaded)
	{
		if (this->toc.chd_f)
		{
			chd_close(this->toc.chd_f);
		}

		if (this->chd_hunkbuf)
		{
			free(this->chd_hunkbuf);
			this->chd_hunkbuf = NULL;
		}

		for (int i = 0; i < this->toc.last; i++)
		{
			if (this->toc.tracks[i].f.opened())
			{
				FileClose(&this->toc.tracks[i].f);
			}
		}

		if (this->toc.sub.opened()) FileClose(&this->toc.sub);

		this->loaded = 0;
	}

	memset(&this->toc, 0x00, sizeof(this->toc));
	this->sectorSize = 0;
}

void cdd_t::Reset() {
	latency = 10;
	index = 0;
	lba = 0;
	scanOffset = 0;
	isData = 1;
	status = CD_STAT_STOP;
	audioLength = 0;
	audioOffset = 0;
	chd_audio_read_lba = 0;

	stat[0] = 0x0;
	stat[1] = 0x0;
	stat[2] = 0x0;
	stat[3] = 0x0;
	stat[4] = 0x0;
	stat[5] = 0x0;
	stat[6] = 0x0;
	stat[7] = 0x0;
	stat[8] = 0x0;
	stat[9] = 0xF;
}

void cdd_t::Update() {
	if (this->status == CD_STAT_STOP || this->status == CD_STAT_TRAY || this->status == CD_STAT_OPEN)
	{
		if (this->latency > 0)
		{
			this->latency--;
			return;
		}
		// Neo Geo CDZ does not like the status changing to TOC here.
		//this->status = this->loaded ? CD_STAT_TOC : CD_STAT_NO_DISC;
		if (!this->loaded)
		{
			this->status = CD_STAT_NO_DISC;
		}
	}
	else if (this->status == CD_STAT_SEEK)
	{
		if (this->latency > 0)
		{
			this->latency--;
			return;
		}
		this->status = CD_STAT_PAUSE;
	}
	else if (this->status == CD_STAT_PLAY)
	{
		if (this->latency > 0)
		{
			this->latency--;
			return;
		}

		if (this->ld_mode)
		{
			LDUpdate();
			return;
		}

		if (this->index >= this->toc.last)
		{
			this->status = CD_STAT_END;
			return;
		}

		if (CanSendData && !CanSendData(this->toc.tracks[this->index].type))
		{
			// Not ready yet to receive sector
			return;
		}

		if (this->toc.tracks[this->index].type)
		{
			// CD-ROM (Mode 1)
			uint8_t header[4];
			msf_t msf;
			LBAToMSF(this->lba + 150, &msf);
			header[0] = BCD(msf.m);
			header[1] = BCD(msf.s);
			header[2] = BCD(msf.f);
			header[3] = 0x01;

			SectorSend(header);
		}
		else
		{
			if (this->lba >= this->toc.tracks[this->index].start)
			{
				this->isData = 0x00;
			}
			SectorSend(0);
		}

		this->lba++;
		this->chd_audio_read_lba++;

		if (this->lba >= this->toc.tracks[this->index].end)
		{
			this->index++;

			this->isData = 0x01;

			if (this->toc.tracks[this->index].f.opened())
			{
				FileSeek(&this->toc.tracks[this->index].f, (this->toc.tracks[this->index].start * 2352) - this->toc.tracks[this->index].offset, SEEK_SET);
			}
		}
	}
	else if (cdd.status == CD_STAT_SCAN)
	{
		this->lba += this->scanOffset;

		if (this->lba >= this->toc.tracks[this->index].end)
		{
			this->index++;
			if (this->index < this->toc.last)
			{
				this->lba = this->toc.tracks[this->index].start;
			}
			else
			{
				this->lba = this->toc.end;
				this->chd_audio_read_lba = this->lba;
				this->status = CD_STAT_END;
				this->isData = 0x01;
				return;
			}
		}
		else if (this->lba < this->toc.tracks[this->index].start)
		{
			if (this->index > 0)
			{
				this->index--;
				this->lba = this->toc.tracks[this->index].end;
			}
			else
			{
				this->lba = 0;
			}
		}

		this->chd_audio_read_lba = this->lba;

		this->isData = this->toc.tracks[this->index].type;

		if (this->toc.sub.opened()) FileSeek(&this->toc.sub, this->lba * 96, SEEK_SET);

		if (this->toc.tracks[this->index].type)
		{
			// DATA track
			FileSeek(&this->toc.tracks[0].f, this->lba * this->sectorSize, SEEK_SET);
		}
		else if (this->toc.tracks[this->index].f.opened())
		{
			// AUDIO track
			FileSeek(&this->toc.tracks[this->index].f, (this->lba * 2352) - this->toc.tracks[this->index].offset, SEEK_SET);
		}
	}
}

// ------------------------------------------------------------- LaserActive

void cdd_t::LDSeek(int lba, int play)
{
	if (lba < -150) lba = -150;
	if (lba >= this->toc.end) lba = this->toc.end - 1;
	SeekToLBA(lba, play);
	this->status = play ? CD_STAT_PLAY : CD_STAT_SEEK;
	if (!play && !this->latency) this->status = CD_STAT_PAUSE;
	this->isData = this->toc.tracks[this->index].type ? 1 : 0;
}

void cdd_t::LDPlay()
{
	// a seek in progress ends in play instead of pause
	if (this->status == CD_STAT_PLAY || this->status == CD_STAT_SEEK) this->status = CD_STAT_PLAY;
	else if (this->loaded) this->status = CD_STAT_PLAY;
}

void cdd_t::LDPause()
{
	if (this->status == CD_STAT_PLAY && this->latency > 0) this->status = CD_STAT_SEEK;
	else if (this->status == CD_STAT_PLAY || this->status == CD_STAT_SCAN || this->status == CD_STAT_END) this->status = CD_STAT_PAUSE;
}

void cdd_t::LDStop()
{
	this->status = this->loaded ? CD_STAT_STOP : CD_STAT_NO_DISC;
	this->latency = 0;
	this->isData = 1;
}

// Between drive ticks: top the CD audio FIFO up as soon as it has room, so a
// late tick doesn't let it run dry (it holds only ~29 ms).
// Where the disc's audio tracks and data track are in the archive, for the
// read-ahead (megald_audio.cpp).
// Where the CD audio of a tick that also carried a data sector goes. Into the
// CDC as well (index 4), the CDC decodes it as a second sector: its header is
// soundtrack samples, it raises a decoder interrupt and it's written into the
// CDC buffer, so the buffer wraps twice as fast and the BIOS's read engine
// loses sectors (Road Blaster's loads failed and then hung). Cores with
// protocol 5 take it on index 7, for the CD audio FIFO only.
static int ld_audio_index = MCD_CDDA_IO_INDEX;

void cdd_t::LDStartReadAhead()
{
	int v = ld_video_core_version();
	ld_audio_index = v >= 5 ? 7 : MCD_CDDA_IO_INDEX;
	if (v < 5) ld_log("this core (LaserDisc protocol %d) sends the soundtrack under data sectors into the CDC too; data loads may fail", v);
	std::vector<ld_audio_seg> segs;
	ld_audio_seg data = { 0, 0, 0, 2352, 0 };
	for (int i = 0; i < this->toc.last; i++)
	{
		auto &t = this->toc.tracks[i];
		if (!t.f.opened()) continue;
		if (t.type)
		{
			// as ReadData reads it: track 0's file, from LBA 0, at sectorSize
			if (i != 0 || !this->sectorSize) continue;
			int ss = this->sectorSize;
			int b = t.end;
			int hi = (int)(t.f.size / ss);
			if (b > hi) b = hi;
			if (b > 0) data = { 0, b, (uint64_t)t.f.base, ss, ss == 2048 ? 0 : 16 };
			continue;
		}
		int a = t.start, b = t.end;
		int lo = (t.offset + 2351) / 2352;                       // first LBA inside the file
		int hi = (int)((t.f.size + t.offset) / 2352);           // end of the file's whole sectors
		if (a < lo) a = lo;
		if (b > hi) b = hi;
		if (a >= b) continue;
		segs.push_back({ a, b, (uint64_t)(t.f.base + (__off64_t)a * 2352 - t.offset), 2352, 0 });
	}
	ld_audio_attach(segs, data);
}

void cdd_t::LDTopUp()
{
	if (!this->ld_mode) return;
	if (this->status == CD_STAT_PLAY && this->latency == 0) LDUpdate();
	else ld_audio_idle();
}

// The regions in the disc's header (the first data sector, 0x1F0): bit 0 J,
// bit 1 U, bit 2 E; 0 if there's no data track or no region letters.
int cdd_t::DiscRegion()
{
	if (!this->loaded || !this->toc.last || !this->toc.tracks[0].type) return 0;
	uint8_t b[0x200];
	memset(b, 0, sizeof(b));
	int skip = this->sectorSize == 2048 ? 0 : 16;
	if (this->toc.chd_f)
	{
		if (mister_chd_read_sector(this->toc.chd_f, this->toc.tracks[0].offset, 0, skip, sizeof(b), b, this->chd_hunkbuf, &this->chd_hunknum) != CHDERR_NONE) return 0;
	}
	else
	{
		if (!this->toc.tracks[0].f.opened() || !FileSeek(&this->toc.tracks[0].f, skip, SEEK_SET)) return 0;
		if (FileReadAdv(&this->toc.tracks[0].f, b, sizeof(b)) != (int)sizeof(b)) return 0;
	}
	int r = 0;
	for (int i = 0x1F0; i < 0x1F3; i++)
	{
		if (b[i] == 'J') r |= 1;
		else if (b[i] == 'U') r |= 2;
		else if (b[i] == 'E') r |= 4;
	}
	return r;
}

void cdd_t::LDOpenTray()
{
	this->status = CD_STAT_OPEN;
	this->latency = 0;
	this->isData = 1;
}

// One sector tick while a LaserDisc plays. The tick is paced by the core's
// CD audio FIFO (one 2352-byte sector of audio is sent per tick, digital,
// analog or silence), so the disc runs at exactly 75 sectors a second on the
// core's clock. As in ares (MCD::CDD::clock/sample/advance), the current
// sector's data and audio are sent, then the player decides how far to move.
void cdd_t::LDUpdate()
{
	if (CanSendData && !CanSendData(0)) return;

	if (this->lba >= this->toc.end || this->toc.last == 0)
	{
		this->status = CD_STAT_END;
		this->isData = 1;
		ld_sector_reached(this->lba, 1);
		return;
	}

	this->index = this->toc.GetTrackByLBA(this->lba);
	int data = this->toc.tracks[this->index].type ? 1 : 0;
	this->isData = data;

	// Data sectors go out for the pregap too (no data, but a header and a
	// decoder interrupt each): the BIOS seeks a few sectors before its
	// target and gives up if the decoder stays silent, as Main's normal
	// CD path and ares also do.
	if (data)
	{
		uint8_t header[4];
		msf_t msf;
		LBAToMSF(this->lba + 150, &msf);
		header[0] = BCD(msf.m);
		header[1] = BCD(msf.s);
		header[2] = BCD(msf.f);
		header[3] = 0x01;
		SectorSend(header);
	}

	// both audio sources come from the read-ahead (megald_audio.cpp); a data
	// sector has no digital audio, but the analog soundtrack plays over it
	static int16_t audio[588 * 2], analog[588 * 2];
	int have_digital = 0, have_analog = 0;
	ld_audio_get(this->lba, this->ld_dir, audio, &have_digital, analog, &have_analog);
	if (data || this->lba < this->toc.tracks[this->index].start) have_digital = 0;
	if (!have_digital) memset(audio, 0, sizeof(audio));
	ld_mix_audio(audio, analog, have_digital, have_analog);
	if (SendData) SendData((uint8_t *)audio, 2352, data ? ld_audio_index : MCD_CDDA_IO_INDEX);
	ld_audio_sent(GetTimer(0), this->lba);

	int adv = ld_sector_advance();
	this->ld_dir = adv;
	int next = this->lba + adv;
	if (next < -150) next = -150;
	if (next >= this->toc.end)
	{
		this->lba = this->toc.end;
		this->status = CD_STAT_END;
		this->isData = 1;
		ld_sector_reached(this->lba, 1);
		return;
	}
	this->lba = next;
	this->chd_audio_read_lba = next;
	this->index = this->toc.GetTrackByLBA(next);
	ld_sector_reached(next, 0);
}

void cdd_t::CommandExec() {
	msf_t msf;

	switch (comm[0]) {
	case CD_COMM_IDLE:
		if (this->latency <= 3)
		{
			stat[0] = this->status;
			if (stat[1] == 0x0f)
			{
				int lba = this->lba + 150;
				LBAToMSF(lba, &msf);
				stat[1] = 0x0;
	                        stat[2] = BCD(msf.m) >> 4;
				stat[3] = BCD(msf.m) & 0xF;
				stat[4] = BCD(msf.s) >> 4;
				stat[5] = BCD(msf.s) & 0xF;
				stat[6] = BCD(msf.f) >> 4;
				stat[7] = BCD(msf.f) & 0xF;
				stat[8] = this->toc.tracks[this->index].type ? 0x04 : 0x00;
			} else if (stat[1] == 0x00) {
				int lba = this->lba + 150;
				LBAToMSF(lba, &msf);
                                stat[2] = BCD(msf.m) >> 4;
                                stat[3] = BCD(msf.m) & 0xF;
                                stat[4] = BCD(msf.s) >> 4;
                                stat[5] = BCD(msf.s) & 0xF;
                                stat[6] = BCD(msf.f) >> 4;
                                stat[7] = BCD(msf.f) & 0xF;
                                stat[8] = this->toc.tracks[this->index].type ? 0x04 : 0x00;
			} else if (stat[1] == 0x01) {
				int lba = abs(this->lba - this->toc.tracks[this->index].start);
				LBAToMSF(lba,&msf);
                                stat[2] = BCD(msf.m) >> 4;
                                stat[3] = BCD(msf.m) & 0xF;
                                stat[4] = BCD(msf.s) >> 4;
                                stat[5] = BCD(msf.s) & 0xF;
                                stat[6] = BCD(msf.f) >> 4;
                                stat[7] = BCD(msf.f) & 0xF;
                                stat[8] = this->toc.tracks[this->index].type ? 0x04 : 0x00;
			} else if (stat[1] == 0x02) {
                               stat[2] = (cdd.index < this->toc.last) ? BCD(this->index + 1) >> 4 : 0xA;
                               stat[3] = (cdd.index < this->toc.last) ? BCD(this->index + 1) & 0xF : 0xA;
			}
		}

		//printf("MCD: Command IDLE, status = %u\n\x1b[0m", this->status);
		break;

	case CD_COMM_STOP:
		this->status = CD_STAT_STOP;
		this->isData = 1;

		stat[0] = this->status;
		stat[1] = 0;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command STOP, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;

	case CD_COMM_TOC:
		if (this->status == CD_STAT_STOP) {
			this->status = CD_STAT_TOC;
		}
		switch (comm[3]) {
		case 0: {
			int lba_ = this->lba + 150;
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x0;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = BCD(msf.f) >> 4;
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = this->toc.tracks[this->index].type << 2;

			//printf("\x1b[32mMCD: Command TOC 0, lba = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, status = %02X%08X, frame = %u\n\x1b[0m", lba, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], (uint32_t)(GetStatus() >> 32), (uint32_t)GetStatus(), frame);
		}
			break;

		case 1: {
			int lba_ = abs(this->lba - this->toc.tracks[this->index].start);
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x1;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = BCD(msf.f) >> 4;
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = this->toc.tracks[this->index].type << 2;

			//printf("\x1b[32mMCD: Command TOC 1, lba = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, status = %02X%08X, frame = %u\n\x1b[0m", lba, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], (uint32_t)(GetStatus() >> 32), (uint32_t)GetStatus(), frame);
		}
			break;

		case 2: {
			stat[0] = this->status;
			stat[1] = 0x2;
			stat[2] = ((this->index < this->toc.last) ?  BCD(this->index + 1) >> 4 : 0xA);
			stat[3] = ((this->index < this->toc.last) ? BCD(this->index + 1) & 0xF : 0xA);
			stat[4] = 0;
			stat[5] = 0;
			stat[6] = 0;
			stat[7] = 0;
			stat[8] = 0;

			//printf("\x1b[32mMCD: Command TOC 2, index = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, status = %02X%08X, frame = %u\n\x1b[0m", this->index, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], (uint32_t)(GetStatus() >> 32), (uint32_t)GetStatus(), frame);

		}
			break;

		case 3: {
			int lba_ = this->toc.end + 150;
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x3;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = BCD(msf.f) >> 4;
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = 0;

			//printf("\x1b[32mMCD: Command TOC 3, lba = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
		}
			break;

		case 4: {
			stat[0] = this->status;
			stat[1] = 0x4;
			stat[2] = 0;
			stat[3] = 1;
			stat[4] = BCD(this->toc.last) >> 4;
			stat[5] = BCD(this->toc.last) & 0xF;
			stat[6] = 0;
			stat[7] = 0;
			stat[8] = 0;

			//printf("\x1b[32mMCD: Command TOC 4, last = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", this->toc.last, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
		}
			break;

		case 5: {
			int track = comm[4] * 10 + comm[5];
			int lba_ = this->toc.tracks[track - 1].start + 150;
			LBAToMSF(lba_, &msf);

			stat[0] = this->status;
			stat[1] = 0x5;
			stat[2] = BCD(msf.m) >> 4;
			stat[3] = BCD(msf.m) & 0xF;
			stat[4] = BCD(msf.s) >> 4;
			stat[5] = BCD(msf.s) & 0xF;
			stat[6] = (BCD(msf.f) >> 4) | (this->toc.tracks[track - 1].type << 3);
			stat[7] = BCD(msf.f) & 0xF;
			stat[8] = BCD(track) & 0xF;

			//printf("\x1b[32mMCD: Command TOC 5, lba = %i, track = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba_, track, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
		}
			break;

		case 6:
			stat[0] = this->status;
			stat[1] = 0x6;
			stat[2] = 0;
			stat[3] = 0;
			stat[4] = 0;
			stat[5] = 0;
			stat[6] = 0;
			stat[7] = 0;
			stat[8] = 0;
			break;

		default:

			break;
		}
		break;

	case CD_COMM_PLAY: {
		int lba_;
		MSFToLBA(&lba_, comm[2] * 10 + comm[3], comm[4] * 10 + comm[5],	comm[6] * 10 + comm[7]);
		lba_ -= 150;

		SeekToLBA(lba_, 1);

		this->isData = 1;

		this->status = CD_STAT_PLAY;

		stat[0] = CD_STAT_SEEK;
		stat[1] = 0xf;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command PLAY, lba = %i, index = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba_, this->index, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
	}
		break;

	case CD_COMM_SEEK: {
		int lba_;
		MSFToLBA(&lba_, comm[2] * 10 + comm[3], comm[4] * 10 + comm[5], comm[6] * 10 + comm[7]);
		lba_ -= 150;

		SeekToLBA(lba_, 0);

		this->isData = 1;

		this->status = CD_STAT_SEEK;

		stat[0] = this->status;
		stat[1] = 0xf;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command PLAY, lba = %i, index = %i, command = %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X, frame = %u\n\x1b[0m", lba_, this->index, comm[9], comm[8], comm[7], comm[6], comm[5], comm[4], comm[3], comm[2], comm[1], comm[0], frame);
	}
		break;

	case CD_COMM_PAUSE:
		this->isData = 0x01;

		this->status = CD_STAT_PAUSE;

		stat[0] = this->status;
		//printf("\x1b[32mMCD: Command PAUSE, status = %X, frame = %u\n\x1b[0m", this->status, frame);
		break;

	case CD_COMM_RESUME:
		this->status = CD_STAT_PLAY;
		stat[0] = this->status;
		this->audioOffset = 0;
		//printf("\x1b[32mMCD: Command RESUME, status = %X\n\x1b[0m", this->status);
		break;

	case CD_COMM_FW_SCAN:
		this->scanOffset = CD_SCAN_SPEED;
		this->status = CD_STAT_SCAN;
		stat[0] = this->status;
		break;

	case CD_COMM_RW_SCAN:
		this->scanOffset = -CD_SCAN_SPEED;
		this->status = CD_STAT_SCAN;
		stat[0] = this->status;
		break;

	case CD_COMM_TRACK_MOVE:
		this->isData = 1;
		this->status = CD_STAT_PAUSE;
		stat[0] = this->status;
		break;

	case CD_COMM_TRACK_PLAY: {
		int index = comm[2] * 10 + comm[3];
		if (index > 0)
		{
			index -= 1;
		}
		int lba = this->toc.tracks[index].start;

		SeekToLBA(lba, 1);

		this->isData = 1;

		this->status = CD_STAT_PLAY;

		stat[0] = CD_STAT_SEEK;
		stat[1] = 0xf;
		stat[2] = 0;
		stat[3] = 0;
		stat[4] = 0;
		stat[5] = 0;
		stat[6] = 0;
		stat[7] = 0;
		stat[8] = 0;

		//printf("\x1b[32mMCD: Command CD_COMM_TRACK_PLAY, index: %u, status = %u \n\x1b[0m", index, this->status);
	}
		break;

	case CD_COMM_TRAY_CLOSE:
		this->isData = 1;
		this->status = this->loaded ? CD_STAT_TOC : CD_STAT_NO_DISC;
		stat[0] = CD_STAT_STOP;

		//printf("\x1b[32mMCD: Command TRAY_CLOSE, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;

	case CD_COMM_TRAY_OPEN:
		this->isData = 1;
		this->status = CD_STAT_OPEN;
		stat[0] = CD_STAT_OPEN;

		//printf("\x1b[32mMCD: Command TRAY_OPEN, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;

	default:
		stat[0] = this->status;

		//printf("\x1b[32mMCD: Command undefined, status = %u, frame = %u\n\x1b[0m", this->status, frame);
		break;
	}
}

uint64_t cdd_t::GetStatus(uint8_t crc_start) {
	uint8_t n9 = ~(crc_start + stat[0] + stat[1] + stat[2] + stat[3] + stat[4] + stat[5] + stat[6] + stat[7] + stat[8]);
	return ((uint64_t)(n9 & 0xF) << 36) |
		((uint64_t)(stat[8] & 0xF) << 32) |
		((uint64_t)(stat[7] & 0xF) << 28) |
		((uint64_t)(stat[6] & 0xF) << 24) |
		((uint64_t)(stat[5] & 0xF) << 20) |
		((uint64_t)(stat[4] & 0xF) << 16) |
		((uint64_t)(stat[3] & 0xF) << 12) |
		((uint64_t)(stat[2] & 0xF) << 8) |
		((uint64_t)(stat[1] & 0xF) << 4) |
		((uint64_t)(stat[0] & 0xF) << 0);
}

int cdd_t::SetCommand(uint64_t c, uint8_t crc_start) {
	comm[0] = (c >> 0) & 0xF;
	comm[1] = (c >> 4) & 0xF;
	comm[2] = (c >> 8) & 0xF;
	comm[3] = (c >> 12) & 0xF;
	comm[4] = (c >> 16) & 0xF;
	comm[5] = (c >> 20) & 0xF;
	comm[6] = (c >> 24) & 0xF;
	comm[7] = (c >> 28) & 0xF;
	comm[8] = (c >> 32) & 0xF;
	comm[9] = (c >> 36) & 0xF;

	uint8_t crc = (~(crc_start + comm[0] + comm[1] + comm[2] + comm[3] + comm[4] + comm[5] + comm[6] + comm[7] + comm[8])) & 0xF;
	if (comm[9] != crc)
		return -1;

	return 0;
}

void cdd_t::LBAToMSF(int lba, msf_t* msf) {
	msf->m = (lba / 75) / 60;
	msf->s = (lba / 75) % 60;
	msf->f = (lba % 75);
}

void cdd_t::MSFToLBA(int* lba, uint8_t m, uint8_t s, uint8_t f) {
	*lba = f + s * 75 + m * 60 * 75;
}

void cdd_t::MSFToLBA(int* lba, msf_t* msf) {
	*lba = msf->f + msf->s * 75 + msf->m * 60 * 75;
}

void cdd_t::SeekToLBA(int lba, int play) {
	int index = 0;

	this->latency = 0;
	if (play)
	{
		this->latency = 11;
	}

	this->latency += (abs(lba - this->lba) * 120) / 270000;

	this->lba = lba;

	while ((this->toc.tracks[index].end <= lba) && (index < this->toc.last)) index++;
	this->index = index;

	if (lba < this->toc.tracks[index].start)
	{
		lba = this->toc.tracks[index].start;
	}

	if (this->toc.tracks[index].type)
	{
		/* DATA track */
		FileSeek(&this->toc.tracks[0].f, lba * this->sectorSize, SEEK_SET);
	}
	else if (this->toc.tracks[index].f.opened())
	{
		/* PCM AUDIO track */
		FileSeek(&this->toc.tracks[index].f, (lba * 2352) - this->toc.tracks[index].offset, SEEK_SET);
	}

	if (play)
	{
		this->chd_audio_read_lba = this->lba;
		this->audioOffset = 0;
	}

	if (this->toc.sub.opened()) FileSeek(&this->toc.sub, lba * 96, SEEK_SET);

}

// LaserActive log_sectors: where the last data sector came from
// (1 read-ahead, 2 read-ahead miss, 3 plain file read; 0 none)
static int ld_data_src;

static uint32_t sector_crc(const uint8_t *p, int n)
{
	static uint32_t t[256];
	if (!t[1]) for (uint32_t i = 0; i < 256; i++) { uint32_t c = i; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1))); t[i] = c; }
	uint32_t c = 0xFFFFFFFFu;
	while (n--) c = t[(c ^ *p++) & 0xFF] ^ (c >> 8);
	return ~c;
}

void cdd_t::ReadData(uint8_t *buf)
{
	// the pregap before LBA 0 (seeks land a few sectors early): blank, as on a
	// pressed disc, rather than whatever was on the stack
	if (this->lba < 0) memset(buf, 0, 2048);
	if (this->toc.tracks[this->index].type && (this->lba >= 0))
	{

		if (this->toc.chd_f)
		{
			int read_offset = 0;
			if (this->sectorSize != 2048)
			{
				read_offset += 16;
			}

			mister_chd_read_sector(this->toc.chd_f, this->lba + this->toc.tracks[0].offset, 0, read_offset, 2048, buf, this->chd_hunkbuf, &this->chd_hunknum);
		} else {
			// LaserDisc: from the read-ahead, so the drive tick never waits on the card
			ld_data_src = 0;
			if (this->ld_mode && (ld_data_src = ld_audio_get_data(this->lba, this->ld_dir, buf))) return;
			ld_data_src = 3;
			if (this->sectorSize == 2048)
			{
				FileSeek(&this->toc.tracks[0].f, this->lba * 2048, SEEK_SET);
			} else {
			FileSeek(&this->toc.tracks[0].f, this->lba * 2352 + 16, SEEK_SET);
			}
			FileReadAdv(&this->toc.tracks[0].f, buf, 2048);
		}
	}
}

int cdd_t::ReadCDDA(uint8_t *buf)
{
	this->audioLength = 2352 + 2352 - this->audioOffset;
	this->audioOffset = 2352;

	//printf("\x1b[32mMCD: AUDIO LENGTH %d LBA: %d INDEX: %d START: %d END %d\n\x1b[0m", this->audioLength, this->lba, this->index, this->toc.tracks[this->index].start, this->toc.tracks[this->index].end);
	//

	if (this->isData)
	{
		return this->audioLength;
	}

	if (this->toc.chd_f)
	{
		for(int i = 0; i < this->audioLength / 2352; i++)
		{
			mister_chd_read_sector(this->toc.chd_f, this->chd_audio_read_lba + this->toc.tracks[this->index].offset, 2352*i, 0, 2352, buf, this->chd_hunkbuf, &this->chd_hunknum);
		}

		//CHD audio requires byteswap. There's probably a better way to do this...

		for (int swapidx = 0; swapidx < this->audioLength; swapidx += 2)
		{
			uint8_t temp = buf[swapidx];
			buf[swapidx] = buf[swapidx+1];
			buf[swapidx+1] = temp;
		}

		if ((this->audioLength / 2352) > 1)
		{
			this->chd_audio_read_lba++;
		}

	} else if (this->toc.tracks[this->index].f.opened()) {
		FileReadAdv(&this->toc.tracks[this->index].f, buf, this->audioLength);
	}

	return this->audioLength;
}

void InterleaveSubcode(uint8_t *subc_data, uint16_t *buf)
{
	for(int i = 0, n=0; i < 96; i+=2,n++)
	{
		int code = 0;
		for (int j = 0; j < 8; j++)
		{
			int bits = (subc_data[(j * 12) + (i / 8)] >> (6 - (i&6))) & 3;
			code |= ((bits & 1) << (15 - j));
			code |= ((bits >> 1) << (7 - j));
		}
		buf[n] = code;
	}
}

int cdd_t::ReadSubcode(uint16_t* buf)
{
	int err = 0;
	uint8_t subc[96];
	if (this->toc.chd_f)
	{
		//Just use the read sector call with an offset, since we previously read that sector, it is already in the hunk cache
		if (this->toc.tracks[this->index].sbc_type == SUBCODE_RW_RAW) {
			mister_chd_read_sector(this->toc.chd_f, this->chd_audio_read_lba + this->toc.tracks[this->index].offset, 0, CD_MAX_SECTOR_DATA, 96, (uint8_t *)buf, this->chd_hunkbuf, &this->chd_hunknum);
		} else if (this->toc.tracks[this->index].sbc_type == SUBCODE_RW) {
			mister_chd_read_sector(this->toc.chd_f, this->chd_audio_read_lba + this->toc.tracks[this->index].offset, 0, CD_MAX_SECTOR_DATA, 96, subc, this->chd_hunkbuf, &this->chd_hunknum);
			InterleaveSubcode(subc, buf);
		} else {
			err = -1;
		}
	} else if (this->toc.sub.opened()) {
		FileReadAdv(&this->toc.sub, subc, 96);
		InterleaveSubcode(subc, buf);
	} else {
		err = -1;
	}

	return err;
}


int cdd_t::SectorSend(uint8_t* header)
{
	uint8_t buf[2352 + 2352];
	int len = 2352;
	uint8_t index = MCD_DATA_IO_INDEX;

	if (header) {
		memcpy(buf + 12, header, 4);
		ld_data_src = 0;
		ReadData(buf + 16);
		if (this->ld_mode && ld_log_sectors())
		{
			// one line per data sector the core is given: which, where from,
			// a CRC of its 2048 bytes (to check against the image), and the
			// time since the previous one
			static unsigned long last_ms;
			unsigned long now = GetTimer(0);
			static const char *src[] = { "nothing (no data track here)", "read-ahead", "read-ahead MISS", "file" };
			const char *from = this->lba < 0 ? "the pregap (blank)" : src[ld_data_src & 3];
			ld_log("sector: lba %d, header %02X:%02X:%02X mode %02X, from %s, crc %08X, %lu ms after the previous",
				this->lba, header[0], header[1], header[2], header[3], from,
				ld_data_src ? sector_crc(buf + 16, 2048) : 0, last_ms ? now - last_ms : 0);
			last_ms = now;
		}
	}
	else {
		index = MCD_CDDA_IO_INDEX;
		len = ReadCDDA(buf);
	}

	SubcodeSend();
	if (SendData)
		return SendData(buf, len, index);

	return 0;
}


int cdd_t::SubcodeSend()
{
	uint16_t buf[98 / 2];

	int err = ReadSubcode(buf);

	if (!err && SendData)
		return SendData((uint8_t*)buf, 98, MCD_SUB_IO_INDEX);

	return 0;
}





