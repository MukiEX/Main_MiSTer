// LaserActive disc images (.dscmmi / .mmi). See megald_mmi.h.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "megald_mmi.h"

static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint64_t rd64(const uint8_t *p) { return rd32(p) | ((uint64_t)rd32(p + 4) << 32); }

static bool read_at(int fd, void *buf, size_t len, uint64_t off)
{
	uint8_t *b = (uint8_t *)buf;
	while (len)
	{
		ssize_t r = pread(fd, b, len, (off_t)off);
		if (r < 0 && errno == EINTR) continue;
		if (r <= 0) return false;
		b += r; len -= r; off += r;
	}
	return true;
}

bool mmi_read(const ld_disc *d, void *buf, size_t len, uint64_t off)
{
	return d->fd >= 0 && read_at(d->fd, buf, len, off);
}

bool mmi_is_archive_name(const char *path)
{
	const char *e = strrchr(path, '.');
	return e && (!strcasecmp(e, ".dscmmi") || !strcasecmp(e, ".mmi"));
}

// ------------------------------------------------------------ ZIP directory

static bool read_directory(ld_disc *d)
{
	int fd = d->fd;
	uint64_t fsize = (uint64_t)lseek(fd, 0, SEEK_END);
	size_t tail = fsize < 65557 ? (size_t)fsize : 65557;
	std::vector<uint8_t> buf(tail);
	if (!tail || !read_at(fd, buf.data(), tail, fsize - tail)) return false;
	long eocd = -1;
	for (long i = (long)tail - 22; i >= 0; i--) if (rd32(&buf[i]) == 0x06054b50) { eocd = i; break; }
	if (eocd < 0) return false;
	uint64_t entries = rd16(&buf[eocd + 10]), cd_size = rd32(&buf[eocd + 12]), cd_off = rd32(&buf[eocd + 16]);
	uint64_t eocd_abs = fsize - tail + eocd;
	uint8_t loc[20], rec[56];
	if (eocd_abs >= 20 && read_at(fd, loc, 20, eocd_abs - 20) && rd32(loc) == 0x07064b50 &&
		read_at(fd, rec, 56, rd64(loc + 8)) && rd32(rec) == 0x06064b50)
	{
		entries = rd64(rec + 32);
		cd_size = rd64(rec + 40);
		cd_off = rd64(rec + 48);
	}
	if (cd_off + cd_size > fsize || cd_size > 64 * 1024 * 1024) return false;
	std::vector<uint8_t> cd(cd_size);
	if (cd_size && !read_at(fd, cd.data(), cd_size, cd_off)) return false;

	size_t p = 0;
	for (uint64_t e = 0; e < entries && p + 46 <= cd_size && rd32(&cd[p]) == 0x02014b50; e++)
	{
		uint16_t method = rd16(&cd[p + 10]), nlen = rd16(&cd[p + 28]), xlen = rd16(&cd[p + 30]), clen = rd16(&cd[p + 32]);
		if (p + 46 + nlen + xlen + clen > cd_size) break;
		uint64_t usize = rd32(&cd[p + 24]), csize = rd32(&cd[p + 20]), lofs = rd32(&cd[p + 42]);
		const uint8_t *x = &cd[p + 46 + nlen], *xe = x + xlen;
		while (x + 4 <= xe)
		{
			uint16_t id = rd16(x), sz = rd16(x + 2);
			const uint8_t *q = x + 4;
			if (q + sz > xe) break;
			if (id == 0x0001)
			{
				const uint8_t *qe = q + sz;
				if (usize == 0xffffffffu && q + 8 <= qe) { usize = rd64(q); q += 8; }
				if (csize == 0xffffffffu && q + 8 <= qe) { csize = rd64(q); q += 8; }
				if (lofs == 0xffffffffu && q + 8 <= qe) { lofs = rd64(q); q += 8; }
			}
			x += 4 + sz;
		}
		if (!method && csize == usize)
		{
			uint8_t lh[30];
			if (read_at(fd, lh, 30, lofs) && rd32(lh) == 0x04034b50)
			{
				mmi_entry me;
				me.name.assign((const char *)&cd[p + 46], nlen);
				me.off = lofs + 30 + rd16(lh + 26) + rd16(lh + 28);
				me.size = usize;
				if (me.off + me.size <= fsize) d->entries.push_back(me);
			}
		}
		p += 46 + nlen + xlen + clen;
	}
	return !d->entries.empty();
}

const mmi_entry *mmi_find(const ld_disc *d, const char *name)
{
	for (auto &e : d->entries) if (!strcasecmp(e.name.c_str(), name)) return &e;
	return nullptr;
}

static bool read_entry_text(const ld_disc *d, const char *name, std::string &out)
{
	const mmi_entry *e = mmi_find(d, name);
	if (!e || e->size > 4 * 1024 * 1024) return false;
	out.resize(e->size);
	return !e->size || read_at(d->fd, &out[0], e->size, e->off);
}

// ------------------------------------------------------------ minimal JSON

struct jval
{
	enum { NUL, BOOL, NUM, STR, ARR, OBJ } t = NUL;
	double n = 0;
	std::string s;
	std::vector<jval> a;
	std::vector<std::pair<std::string, jval>> o;

	const jval &operator[](const char *k) const
	{
		static const jval none;
		if (t == OBJ) for (auto &kv : o) if (kv.first == k) return kv.second;
		return none;
	}
	std::string str() const { return t == STR ? s : std::string(); }
	long num() const { return t == NUM ? (long)n : (t == STR ? atol(s.c_str()) : 0); }
};

struct jparser
{
	const char *p, *e;
	int depth = 0;
	void ws() { while (p < e && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++; }
	bool lit(const char *w) { size_t l = strlen(w); if ((size_t)(e - p) >= l && !memcmp(p, w, l)) { p += l; return true; } return false; }

	bool string(std::string &out)
	{
		if (p >= e || *p != '"') return false;
		p++;
		while (p < e && *p != '"')
		{
			char c = *p++;
			if (c == '\\' && p < e)
			{
				c = *p++;
				switch (c)
				{
				case 'n': out += '\n'; break;
				case 't': out += '\t'; break;
				case 'r': out += '\r'; break;
				case 'b': out += '\b'; break;
				case 'f': out += '\f'; break;
				case 'u':
				{
					unsigned v = 0;
					for (int i = 0; i < 4 && p < e; i++, p++)
					{
						char h = *p;
						v = v * 16 + (h >= '0' && h <= '9' ? h - '0' : (h | 32) >= 'a' && (h | 32) <= 'f' ? (h | 32) - 'a' + 10 : 0);
					}
					if (v < 0x80) out += (char)v;
					else if (v < 0x800) { out += (char)(0xC0 | (v >> 6)); out += (char)(0x80 | (v & 63)); }
					else { out += (char)(0xE0 | (v >> 12)); out += (char)(0x80 | ((v >> 6) & 63)); out += (char)(0x80 | (v & 63)); }
					break;
				}
				default: out += c; break;
				}
			}
			else out += c;
		}
		if (p >= e) return false;
		p++;
		return true;
	}

	bool value(jval &v)
	{
		if (++depth > 64) return false;
		ws();
		bool ok = false;
		if (p >= e) ok = false;
		else if (*p == '{')
		{
			v.t = jval::OBJ;
			p++; ws();
			if (p < e && *p == '}') { p++; ok = true; }
			else while (true)
			{
				std::pair<std::string, jval> kv;
				ws();
				if (!string(kv.first)) break;
				ws();
				if (p >= e || *p != ':') break;
				p++;
				if (!value(kv.second)) break;
				v.o.push_back(std::move(kv));
				ws();
				if (p < e && *p == ',') { p++; continue; }
				if (p < e && *p == '}') { p++; ok = true; }
				break;
			}
		}
		else if (*p == '[')
		{
			v.t = jval::ARR;
			p++; ws();
			if (p < e && *p == ']') { p++; ok = true; }
			else while (true)
			{
				jval x;
				if (!value(x)) break;
				v.a.push_back(std::move(x));
				ws();
				if (p < e && *p == ',') { p++; continue; }
				if (p < e && *p == ']') { p++; ok = true; }
				break;
			}
		}
		else if (*p == '"') { v.t = jval::STR; ok = string(v.s); }
		else if (lit("true")) { v.t = jval::BOOL; v.n = 1; ok = true; }
		else if (lit("false")) { v.t = jval::BOOL; v.n = 0; ok = true; }
		else if (lit("null")) { v.t = jval::NUL; ok = true; }
		else
		{
			char *end;
			std::string tmp(p, (size_t)(e - p) < 64 ? (size_t)(e - p) : 64);
			v.n = strtod(tmp.c_str(), &end);
			if (end != tmp.c_str()) { v.t = jval::NUM; p += end - tmp.c_str(); ok = true; }
		}
		depth--;
		return ok;
	}
};

static bool json_parse(const std::string &text, jval &out)
{
	jparser jp;
	jp.p = text.data();
	jp.e = text.data() + text.size();
	// skip a UTF-8 byte order mark
	if (jp.e - jp.p >= 3 && (uint8_t)jp.p[0] == 0xEF && (uint8_t)jp.p[1] == 0xBB && (uint8_t)jp.p[2] == 0xBF) jp.p += 3;
	return jp.value(out);
}

static bool ends_with(const std::string &s, const char *suffix)
{
	size_t l = strlen(suffix);
	return s.size() >= l && !strcasecmp(s.c_str() + s.size() - l, suffix);
}

// ------------------------------------------------------------------ open

void mmi_close(ld_disc *d)
{
	if (d->fd >= 0) close(d->fd);
	d->fd = -1;
	d->entries.clear();
	d->vbi.clear();
	d->has_video = false;
}

bool mmi_open(ld_disc *d, const char *path, int media_index, char *err, int errlen)
{
	mmi_close(d);
	snprintf(d->path, sizeof(d->path), "%s", path);
	d->title.clear(); d->media_name.clear();
	d->redbook.clear(); d->raw_audio.clear(); d->raw_video.clear();
	d->media_count = 0; d->media_index = 0; d->side_no = 1; d->clv = false;
	d->lead_in = d->active = d->lead_out = 0;
	d->has_video = false;
	d->audio_off = d->audio_size = 0;

	d->fd = open(path, O_RDONLY | O_CLOEXEC);
	if (d->fd < 0) { snprintf(err, errlen, "cannot open file"); return false; }
	if (!read_directory(d)) { snprintf(err, errlen, "not an uncompressed ZIP archive"); mmi_close(d); return false; }

	std::string text;
	jval info;
	if (!read_entry_text(d, "MediaInfo.json", text) || !json_parse(text, info) || info["media"].t != jval::ARR || info["media"].a.empty())
	{
		snprintf(err, errlen, "no usable MediaInfo.json");
		mmi_close(d);
		return false;
	}
	d->title = info["name"].str();
	const std::vector<jval> &media = info["media"].a;
	d->media_count = (int)media.size();
	d->media_index = (media_index >= 0 && media_index < d->media_count) ? media_index : 0;
	const jval &m = media[d->media_index];
	d->media_name = m["name"].str();
	d->side_no = m["sideNo"].num() ? (int)m["sideNo"].num() : 1;
	d->clv = ends_with(m["format"].str(), "CLV");
	for (auto &st : m["streams"].a)
	{
		std::string type = st["type"].str();
		if (type == "Redbook") d->redbook = st["file"].str();
		else if (type == "RawAudio") d->raw_audio = st["file"].str();
		else if (type == "RawVideo")
		{
			d->raw_video = st["file"].str();
			d->lead_in = (int)st["framesInLeadInRegion"].num();
			d->active = (int)st["framesInActiveRegion"].num();
			d->lead_out = (int)st["framesInLeadOutRegion"].num();
		}
	}
	if (d->redbook.empty() || !mmi_find(d, d->redbook.c_str()))
	{
		snprintf(err, errlen, "side %d has no digital track", d->media_index + 1);
		mmi_close(d);
		return false;
	}

	if (!d->raw_audio.empty())
	{
		const mmi_entry *e = mmi_find(d, d->raw_audio.c_str());
		if (e) { d->audio_off = e->off; d->audio_size = e->size; }
	}

	// The .dscv that replaces the side's video: DSCMMI.json first, then by name.
	std::string dscv;
	jval dm;
	if (!d->raw_video.empty() && read_entry_text(d, "DSCMMI.json", text) && json_parse(text, dm))
	{
		for (auto &v : dm["video"].a)
			if (!strcasecmp(v["source_entry"].str().c_str(), d->raw_video.c_str())) dscv = v["entry"].str();
	}
	if (dscv.empty() && !d->raw_video.empty())
	{
		dscv = d->raw_video;
		size_t dot = dscv.rfind('.');
		if (dot != std::string::npos) dscv.resize(dot);
		dscv += ".dscv";
	}
	const mmi_entry *ve = dscv.empty() ? nullptr : mmi_find(d, dscv.c_str());
	uint8_t h[512];
	if (ve && ve->size >= 512 && read_at(d->fd, h, 512, ve->off) && !memcmp(h, "DSCV", 4))
	{
		d->dscv_off = ve->off;
		d->field1_row = rd16(h + 24);
		d->frame_count = rd32(h + 28);
		d->frame_bytes = rd32(h + 40);
		d->frame_stride = rd32(h + 44);
		d->frames_off = ve->off + rd64(h + 48);
		uint64_t vbi_off = rd64(h + 56);
		uint32_t vbi_size = rd32(h + 64);
		memcpy(d->pps, h + 128, 128);
		if (d->frame_count && d->frame_bytes && d->frames_off + (uint64_t)d->frame_count * d->frame_stride <= ve->off + ve->size)
		{
			d->has_video = true;
			if (vbi_off && vbi_size >= 16 && ve->off + vbi_off + (uint64_t)d->frame_count * vbi_size <= ve->off + ve->size)
			{
				std::vector<uint8_t> raw((size_t)d->frame_count * vbi_size);
				if (read_at(d->fd, raw.data(), raw.size(), ve->off + vbi_off))
				{
					d->vbi.resize((size_t)d->frame_count * 4);
					for (uint32_t f = 0; f < d->frame_count; f++)
						for (int i = 0; i < 4; i++) d->vbi[f * 4 + i] = rd32(&raw[(size_t)f * vbi_size + i * 4]);
				}
			}
		}
		// MediaInfo without region counts: treat the whole stream as the active region
		if (d->has_video && !d->active && !d->lead_in && !d->lead_out) d->active = d->frame_count;
	}

	err[0] = 0;
	return true;
}
