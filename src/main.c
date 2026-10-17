/*
 * valetudo-helper-felroot
 *
 * The FEL USB protocol implementation and progress bar are derived from
 * xfel/fel.c and xfel/progress.c:
 *
 *     XFEL - Tiny FEL tools for allwinner SOC.
 *     Copyright (c) 2007-2022 Jianjun Jiang <8192542@qq.com>
 *     Licensed under the MIT License (see LICENSE for full text).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * The sparse image download/flash pipeline is a C port of the resparse logic
 * from Android platform_system_core:
 *
 *     fastboot.cpp  Copyright (C) 2008 The Android Open Source Project (BSD)
 *     libsparse     Copyright (C) 2010, 2012 The Android Open Source Project
 *                   (Apache-2.0)
 *
 * See THIRD_PARTY_NOTICES and LICENSE for the full license texts.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <libusb.h>

#define VERSION "2026.10.0"
#define PROG_NAME "valetudo-helper-felroot"

#define ALLWINNER_VID 0x1f3a
#define FEL_PID 0xefe8
#define FB_PID 0x1010

struct usb_request_t {
	char     magic[8];
	uint32_t length;
	uint32_t unknown1;
	uint16_t request;
	uint32_t length2;
	char     pad[10];
} __attribute__((packed));

struct fel_request_t {
	uint32_t request;
	uint32_t address;
	uint32_t length;
	uint32_t pad;
} __attribute__((packed));

struct fel_version_t {
	char     magic[8];
	uint32_t id;
	uint32_t firmware;
	uint16_t protocol;
	uint8_t  dflag;
	uint8_t  dlength;
	uint32_t scratchpad;
	uint8_t  pad[8];
};

struct chip_profile_t {
	uint32_t    id;
	const char *name;
	uint32_t    fsbl_addr;
	uint32_t    payload_addr;
	unsigned    dram_wait_sec;
};

static const struct chip_profile_t PROFILES[] = {
	{ 0x00191200, "MR536", 0x0004C000, 0x4A000000, 22 },
	{ 0x00185500, "MR813", 0x00028000, 0x4A000000,  3 },
};

static const struct chip_profile_t *find_profile(uint32_t id)
{
	for (size_t i = 0; i < sizeof(PROFILES) / sizeof(PROFILES[0]); i++)
		if (PROFILES[i].id == id)
			return &PROFILES[i];
	return NULL;
}

struct xfel_ctx_t {
	libusb_device_handle *hdl;
	uint8_t epout;
	uint8_t epin;
	struct fel_version_t version;
};

static volatile sig_atomic_t g_interrupt = 0;

static void die(const char *msg)
{
	fprintf(stderr, "[!] %s\n", msg);
	exit(1);
}

static inline void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)(v >> 8);
}
static inline void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)((v >> 8) & 0xff);
	p[2] = (uint8_t)((v >> 16) & 0xff);
	p[3] = (uint8_t)((v >> 24) & 0xff);
}

static void usb_bulk_send(libusb_device_handle *hdl, int ep, const uint8_t *buf, size_t len)
{
	const size_t max_chunk = 128 * 1024;
	while (len > 0) {
		size_t chunk = len < max_chunk ? len : max_chunk;
		int bytes = 0;
		int r = libusb_bulk_transfer(hdl, ep, (void *)buf, (int)chunk, &bytes, 10000);
		if (r != 0 || bytes == 0)
			die("usb bulk send error");
		len -= (size_t)bytes;
		buf += bytes;
	}
}

static void usb_bulk_recv(libusb_device_handle *hdl, int ep, uint8_t *buf, size_t len)
{
	while (len > 0) {
		int bytes = 0;
		int r = libusb_bulk_transfer(hdl, ep, (void *)buf, (int)len, &bytes, 10000);
		if (r != 0 || bytes == 0)
			die("usb bulk recv error");
		len -= (size_t)bytes;
		buf += bytes;
	}
}

static void send_usb_request(struct xfel_ctx_t *ctx, uint16_t type, uint32_t length)
{
	uint8_t req[sizeof(struct usb_request_t)];
	memset(req, 0, sizeof(req));
	memcpy(req, "AWUC", 4);
	put_le32(req + 8,  length);
	put_le32(req + 12, 0x0c000000u);
	put_le16(req + 16, type);
	put_le32(req + 18, length);
	usb_bulk_send(ctx->hdl, ctx->epout, req, sizeof(req));
}

static void read_usb_response(struct xfel_ctx_t *ctx)
{
	uint8_t buf[13];
	usb_bulk_recv(ctx->hdl, ctx->epin, buf, sizeof(buf));
	if (memcmp(buf, "AWUS", 4) != 0)
		die("bad usb response magic");
}

static void usb_write(struct xfel_ctx_t *ctx, const uint8_t *buf, size_t len)
{
	send_usb_request(ctx, 0x12, (uint32_t)len);
	usb_bulk_send(ctx->hdl, ctx->epout, buf, len);
	read_usb_response(ctx);
}

static void usb_read(struct xfel_ctx_t *ctx, uint8_t *buf, size_t len)
{
	send_usb_request(ctx, 0x11, (uint32_t)len);
	usb_bulk_recv(ctx->hdl, ctx->epin, buf, len);
	read_usb_response(ctx);
}

static void send_fel_request(struct xfel_ctx_t *ctx, uint32_t type, uint32_t addr, uint32_t length)
{
	uint8_t req[sizeof(struct fel_request_t)];
	put_le32(req + 0,  type);
	put_le32(req + 4,  addr);
	put_le32(req + 8,  length);
	put_le32(req + 12, 0);
	usb_write(ctx, req, sizeof(req));
}

static void read_fel_status(struct xfel_ctx_t *ctx)
{
	uint8_t buf[8];
	usb_read(ctx, buf, sizeof(buf));
}

static void fel_version(struct xfel_ctx_t *ctx)
{
	send_fel_request(ctx, 0x001, 0, 0);
	usb_read(ctx, (uint8_t *)&ctx->version, sizeof(ctx->version));
	read_fel_status(ctx);
}

struct progress_t {
	uint64_t total;
	uint64_t done;
	double   start;
};

static double now_sec(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec + (double)tv.tv_usec / 1.0e6;
}

static void progress_start(struct progress_t *p, uint64_t total)
{
	p->total = total;
	p->done  = 0;
	p->start = now_sec();
}

static void progress_update(struct progress_t *p, uint64_t bytes)
{
	p->done += bytes;
	double ratio = p->total > 0 ? (double)p->done / (double)p->total : 0.0;
	double elapsed = now_sec() - p->start;
	double speed = elapsed > 0 ? (double)p->done / elapsed : 0.0;
	double eta = speed > 0 ? (double)(p->total - p->done) / speed : 0.0;
	int bar_width = 32;
	int pos = (int)(bar_width * ratio);
	if (pos > bar_width) pos = bar_width;

	fprintf(stderr, "\r%3.0f%% [", ratio * 100.0);
	for (int i = 0; i < pos; i++)        fputc('=', stderr);
	for (int i = pos; i < bar_width; i++) fputc(' ', stderr);
	if (p->done < p->total) {
		int s = (int)(eta + 0.5);
		fprintf(stderr, "] %.2f MB/s, ETA %02d:%02d    ",
			speed / (1024.0 * 1024.0), s / 60, s % 60);
	} else {
		fprintf(stderr, "] %.2f MB/s                ",
			speed / (1024.0 * 1024.0));
	}
	fflush(stderr);
}

static void progress_stop(struct progress_t *p)
{
	(void)p;
	fputc('\n', stderr);
}

static void fel_write_progress(struct xfel_ctx_t *ctx, uint32_t addr, const uint8_t *buf, size_t len)
{
	struct progress_t p;
	progress_start(&p, len);
	while (len > 0) {
		size_t n = len > 65536 ? 65536 : len;
		send_fel_request(ctx, 0x101, addr, (uint32_t)n);
		usb_write(ctx, buf, n);
		read_fel_status(ctx);
		addr += (uint32_t)n;
		buf += n;
		len -= n;
		progress_update(&p, n);
	}
	progress_stop(&p);
}

static void fel_exec(struct xfel_ctx_t *ctx, uint32_t addr)
{
	send_fel_request(ctx, 0x102, addr, 0);
	read_fel_status(ctx);
}

static int usb_claim_and_find_eps(libusb_device_handle *hdl, uint8_t *epin, uint8_t *epout)
{
	struct libusb_config_descriptor *config;

	if (libusb_kernel_driver_active(hdl, 0))
		libusb_detach_kernel_driver(hdl, 0);
	if (libusb_claim_interface(hdl, 0) != 0)
		return -1;

	*epin = 0;
	*epout = 0;
	if (libusb_get_active_config_descriptor(libusb_get_device(hdl), &config) != 0)
		return -1;
	for (int i = 0; i < config->bNumInterfaces; i++) {
		const struct libusb_interface *iface = config->interface + i;
		for (int j = 0; j < iface->num_altsetting; j++) {
			const struct libusb_interface_descriptor *s = iface->altsetting + j;
			for (int k = 0; k < s->bNumEndpoints; k++) {
				const struct libusb_endpoint_descriptor *ep = s->endpoint + k;
				if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK)
					continue;
				if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN)
					*epin = ep->bEndpointAddress;
				else
					*epout = ep->bEndpointAddress;
			}
		}
	}
	libusb_free_config_descriptor(config);

	if (!*epin || !*epout)
		return -1;
	return 0;
}

static int open_fel_device(struct xfel_ctx_t *ctx)
{
	libusb_device **list = NULL;
	ssize_t count = libusb_get_device_list(NULL, &list);
	int found = 0;

	for (ssize_t i = 0; i < count; i++) {
		struct libusb_device_descriptor desc;
		if (libusb_get_device_descriptor(list[i], &desc) != 0)
			continue;
		if (desc.idVendor != ALLWINNER_VID || desc.idProduct != FEL_PID)
			continue;
		if (libusb_open(list[i], &ctx->hdl) != 0) {
			fprintf(stderr, "[!] found robot in FEL mode but could not open it (are you root?)\n");
			break;
		}
		found = 1;
		break;
	}
	libusb_free_device_list(list, 1);
	if (!found)
		return 0;

	if (usb_claim_and_find_eps(ctx->hdl, &ctx->epin, &ctx->epout))
		die("could not set up the fel usb interface");

	fel_version(ctx);
	return 1;
}

static void close_fel_device(struct xfel_ctx_t *ctx);

static void on_signal(int sig)
{
	(void)sig;
	g_interrupt = 1;
}

static const int CRITICAL_SIGNALS[] = {
	SIGINT, SIGTERM, SIGQUIT, SIGHUP, SIGTSTP, SIGTTIN, SIGTTOU, SIGPIPE,
};
static sigset_t g_critical_mask;
static sigset_t g_restore_mask;

static void enter_critical_section(void)
{
	sigemptyset(&g_critical_mask);
	for (size_t i = 0; i < sizeof(CRITICAL_SIGNALS) / sizeof(CRITICAL_SIGNALS[0]); i++)
		sigaddset(&g_critical_mask, CRITICAL_SIGNALS[i]);
	sigprocmask(SIG_BLOCK, &g_critical_mask, &g_restore_mask);
}

static void leave_critical_section_and_maybe_exit(void)
{
	sigset_t pending;
	sigpending(&pending);
	for (size_t i = 0; i < sizeof(CRITICAL_SIGNALS) / sizeof(CRITICAL_SIGNALS[0]); i++) {
		if (sigismember(&pending, CRITICAL_SIGNALS[i]))
			exit(130);
	}
	sigprocmask(SIG_SETMASK, &g_restore_mask, NULL);
}

static void close_fel_device(struct xfel_ctx_t *ctx)
{
	if (ctx->hdl) {
		libusb_release_interface(ctx->hdl, 0);
		libusb_attach_kernel_driver(ctx->hdl, 0);
		libusb_close(ctx->hdl);
		ctx->hdl = NULL;
	}
}

#define FB_COMMAND_SZ 4096
#define FB_RESPONSE_SZ 256
#define FB_RESP_TIMEOUT_MS 30000

struct fb_ctx_t {
	libusb_device_handle *hdl;
	uint8_t epin;
	uint8_t epout;
};

static uint8_t *read_file(const char *path, size_t *out_len);

static void fb_raw_send(const struct fb_ctx_t *ctx, const void *buf, size_t len)
{
	usb_bulk_send(ctx->hdl, ctx->epout, buf, len);
}

static int fb_raw_recv(const struct fb_ctx_t *ctx, void *buf, size_t len)
{
	int bytes = 0;
	int r = libusb_bulk_transfer(ctx->hdl, ctx->epin, buf, (int)len, &bytes, FB_RESP_TIMEOUT_MS);
	if (r != 0)
		return -1;
	return bytes;
}

static int fb_read_response(struct fb_ctx_t *ctx, char *body, size_t body_sz, uint32_t *data_size)
{
	char resp[FB_RESPONSE_SZ + 1];

	for (;;) {
		int n = fb_raw_recv(ctx, resp, FB_RESPONSE_SZ);
		if (n < 0) {
			fprintf(stderr, "\n[!] usb error while waiting for response\n");
			return 'X';
		}
		resp[n] = 0;
		if (n >= 4 && !memcmp(resp, "INFO", 4)) {
			continue;
		}
		if (n >= 4 && !memcmp(resp, "TEXT", 4)) {
			continue;
		}
		if (n >= 4 && !memcmp(resp, "OKAY", 4)) {
			if (body) {
				size_t k = (size_t)(n - 4) < (body_sz - 1) ? (size_t)(n - 4) : (body_sz - 1);
				memcpy(body, resp + 4, k);
				body[k] = 0;
			}
			return 'O';
		}
		if (n >= 4 && !memcmp(resp, "FAIL", 4)) {
			if (body) {
				size_t k = (size_t)(n - 4) < (body_sz - 1) ? (size_t)(n - 4) : (body_sz - 1);
				memcpy(body, resp + 4, k);
				body[k] = 0;
			}
			return 'F';
		}
		if (n >= 4 && !memcmp(resp, "DATA", 4)) {
			if (data_size)
				*data_size = (uint32_t)strtoul(resp + 4, NULL, 16);
			return 'D';
		}
		fprintf(stderr, "\n[!] unrecognized device response (%d bytes, starting '%c%c%c%c')\n",
			n, resp[0], resp[1], resp[2], resp[3]);
		return 'X';
	}
}

static int fb_command(struct fb_ctx_t *ctx, const char *cmd, char *body, size_t body_sz, uint32_t *data_size)
{
	size_t len = strlen(cmd);
	if (len > FB_COMMAND_SZ) {
		fprintf(stderr, "[!] command too long\n");
		return 'X';
	}
	fb_raw_send(ctx, cmd, len);
	return fb_read_response(ctx, body, body_sz, data_size);
}

static void fb_close(struct fb_ctx_t *ctx)
{
	if (ctx->hdl) {
		libusb_release_interface(ctx->hdl, 0);
		libusb_close(ctx->hdl);
		ctx->hdl = NULL;
	}
}

static int fb_open_gadget(struct fb_ctx_t *ctx)
{
	libusb_device **list = NULL;
	ssize_t count = libusb_get_device_list(NULL, &list);
	int found = 0;

	for (ssize_t i = 0; i < count && !found; i++) {
		struct libusb_device_descriptor desc;
		if (libusb_get_device_descriptor(list[i], &desc) != 0)
			continue;
		if (desc.idVendor != ALLWINNER_VID || desc.idProduct != FB_PID)
			continue;
		if (libusb_open(list[i], &ctx->hdl) != 0) {
			fprintf(stderr, "[!] found robot gadget but could not open it (are you root?)\n");
			continue;
		}
		if (usb_claim_and_find_eps(ctx->hdl, &ctx->epin, &ctx->epout)) {
			libusb_close(ctx->hdl);
			ctx->hdl = NULL;
			continue;
		}
		found = 1;
	}
	libusb_free_device_list(list, 1);
	return found;
}

static int fb_wait_gadget(struct fb_ctx_t *ctx, unsigned timeout_s)
{
	time_t deadline = time(NULL) + timeout_s;
	for (;;) {
		if (g_interrupt) {
			fprintf(stderr, "[!] interrupted\n");
			return 0;
		}
		if (fb_open_gadget(ctx))
			return 1;
		if (time(NULL) >= deadline)
			return 0;
		sleep(1);
	}
}

static int fb_check_dustversion(struct fb_ctx_t *ctx, char *version_out)
{
	char body[FB_RESPONSE_SZ];
	int rc = 'X';

	for (int attempt = 0; attempt < 3; attempt++) {
		rc = fb_command(ctx, "getvar:dustversion", body, sizeof(body), NULL);
		if (rc != 'X')
			break;
		sleep(2);
	}
	if (rc == 'F') {
		fprintf(stderr, "[!] device does not know getvar:dustversion\n");
		return 0;
	}
	if (rc != 'O') {
		fprintf(stderr, "[!] device did not answer getvar:dustversion\n");
		return 0;
	}
	if (version_out)
		strcpy(version_out, body);
	return 1;
}

#define SPARSE_HEADER_MAGIC    0xed26ff3a
#define FILE_HEADER_SZ         28
#define CHUNK_HEADER_SZ        12
#define DEFAULT_BLOCK_SZ       4096
#define RESPARSE_LIMIT         (1ULL * 1024 * 1024 * 1024)

#define CHUNK_TYPE_RAW         0xcac1
#define CHUNK_TYPE_FILL        0xcac2
#define CHUNK_TYPE_DONT_CARE   0xcac3
#define CHUNK_TYPE_CRC32       0xcac4

struct sp_block {
	struct sp_block *next;
	uint64_t        offset;	/* byte offset in the output image */
	uint64_t        len;
	int             type;
	uint32_t        fill_val;	/* FILL only */
	uint64_t        file_off;	/* RAW: offset of the data in the input file */
};

struct sp_file {
	uint32_t         blk_sz;
	uint64_t         len;
	struct sp_block *blocks;
	struct sp_block *last;
};

static struct sp_block *sp_block_new(uint64_t off, uint64_t len, int type)
{
	struct sp_block *b = calloc(1, sizeof(*b));
	if (!b) die("oom");
	b->offset = off;
	b->len = len;
	b->type = type;
	return b;
}

static void sp_free(struct sp_file *sf)
{
	while (sf->blocks) {
		struct sp_block *n = sf->blocks->next;
		free(sf->blocks);
		sf->blocks = n;
	}
	sf->last = NULL;
}

static void sp_add(struct sp_file *sf, struct sp_block *b)
{
	struct sp_block *t = sf->last;

	if (t && t->type == b->type && t->offset + t->len == b->offset &&
	    (b->type == CHUNK_TYPE_FILL ? t->fill_val == b->fill_val
					: t->file_off + t->len == b->file_off)) {
		t->len += b->len;
		free(b);
		return;
	}
	if (!sf->blocks)
		sf->blocks = b;
	else
		sf->last->next = b;
	sf->last = b;
}

static uint64_t sp_align_up(uint64_t v, uint32_t blk_sz)
{
	return (v + blk_sz - 1) / blk_sz * blk_sz;
}

static uint32_t sp_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int fb_get_max_download(struct fb_ctx_t *ctx, uint64_t *out)
{
	char body[FB_RESPONSE_SZ];
	char cmd[FB_COMMAND_SZ];

	snprintf(cmd, sizeof(cmd), "getvar:max-download-size");
	int rc = fb_command(ctx, cmd, body, sizeof(body), NULL);
	if (rc != 'O') {
		rc = fb_command(ctx, "getvar:downloadsize", body, sizeof(body), NULL);
		if (rc != 'O')
			return -1;
	}

	char *end = NULL;
	unsigned long long v = strtoull(body, &end, 0);
	if (end == body || v == 0)
		return -1;

	*out = v;
	return 0;
}

static int fb_send_raw(struct fb_ctx_t *ctx, const char *part, const char *path,
		       const uint8_t *buf, size_t len)
{
	char cmd[FB_COMMAND_SZ];
	char body[FB_RESPONSE_SZ];
	uint32_t size;

	fprintf(stderr, "[*] sending '%s' (%zu bytes)\n", path, len);

	snprintf(cmd, sizeof(cmd), "download:%08x", (unsigned)len);
	int rc = fb_command(ctx, cmd, body, sizeof(body), &size);
	if (rc != 'D') {
		fprintf(stderr, "[!] download rejected (%s%s)\n",
			rc == 'F' ? "FAIL" : "no DATA", rc == 'F' ? body : "");
		return -1;
	}

	usb_bulk_send(ctx->hdl, ctx->epout, buf, len);

	rc = fb_read_response(ctx, body, sizeof(body), NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] download failed\n");
		return -1;
	}

	snprintf(cmd, sizeof(cmd), "flash:%s", part);
	rc = fb_command(ctx, cmd, body, sizeof(body), NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] flash failed\n");
		return -1;
	}
	return 0;
}

static int sp_import(struct sp_file *sf, const uint8_t *buf, size_t len)
{
	memset(sf, 0, sizeof(*sf));

	if (len >= FILE_HEADER_SZ) {
		uint16_t major    = (uint16_t)(buf[4] | (buf[5] << 8));
		uint16_t file_hs  = (uint16_t)(buf[8] | (buf[9] << 8));
		uint16_t chunk_hs = (uint16_t)(buf[10] | (buf[11] << 8));
		uint32_t blk_sz   = sp_le32(buf + 12);

		if (sp_le32(buf) == SPARSE_HEADER_MAGIC && major == 1 &&
		    file_hs >= FILE_HEADER_SZ && chunk_hs >= CHUNK_HEADER_SZ &&
		    blk_sz > 0 && (blk_sz % 4) == 0) {
			uint32_t total_blks   = sp_le32(buf + 16);
			uint32_t total_chunks = sp_le32(buf + 20);
			uint64_t pos = file_hs;
			uint64_t img_off = 0;
			int ok = 1;

			sf->blk_sz = blk_sz;
			sf->len = (uint64_t)total_blks * blk_sz;

			for (uint32_t i = 0; i < total_chunks && ok; i++) {
				uint16_t ctype;
				uint32_t csz, tsz, chunk_data;
				struct sp_block *nb;

				if (pos + chunk_hs > len) { ok = 0; break; }
				ctype = (uint16_t)(buf[pos] | (buf[pos + 1] << 8));
				csz   = sp_le32(buf + pos + 4);
				tsz   = sp_le32(buf + pos + 8);
				pos += chunk_hs;

				if (tsz < chunk_hs) { ok = 0; break; }
				chunk_data = tsz - chunk_hs;

				switch (ctype) {
				case CHUNK_TYPE_RAW:
					if (chunk_data != csz * blk_sz ||
					    pos + chunk_data > len) {
						ok = 0;
						break;
					}
					nb = sp_block_new(img_off, (uint64_t)csz * blk_sz,
							  CHUNK_TYPE_RAW);
					nb->file_off = pos;
					sp_add(sf, nb);
					pos += chunk_data;
					break;
				case CHUNK_TYPE_FILL:
					if (chunk_data != 4 || pos + 4 > len) {
						ok = 0;
						break;
					}
					nb = sp_block_new(img_off, (uint64_t)csz * blk_sz,
							  CHUNK_TYPE_FILL);
					nb->fill_val = sp_le32(buf + pos);
					sp_add(sf, nb);
					pos += 4;
					break;
				case CHUNK_TYPE_DONT_CARE:
					if (chunk_data != 0)
						ok = 0;
					break;
				case CHUNK_TYPE_CRC32:
					if (chunk_data != 4 || pos + 4 > len) {
						ok = 0;
						break;
					}
					pos += 4;
					break;
				default:
					ok = 0;
					break;
				}
				img_off += (uint64_t)csz * blk_sz;
			}
			if (ok && img_off == sf->len)
				return 1;
			sp_free(sf);
			memset(sf, 0, sizeof(*sf));
		}
	}

	sf->blk_sz = DEFAULT_BLOCK_SZ;
	sf->len = len;
	for (uint64_t off = 0; off < len; off += DEFAULT_BLOCK_SZ) {
		uint64_t n = len - off < DEFAULT_BLOCK_SZ ? len - off
							 : DEFAULT_BLOCK_SZ;
		int is_fill = 0;

		if (n == DEFAULT_BLOCK_SZ) {
			is_fill = 1;
			for (uint32_t w = 1; w < DEFAULT_BLOCK_SZ / 4; w++) {
				if (memcmp(&buf[off], &buf[off + w * 4], 4) != 0) {
					is_fill = 0;
					break;
				}
			}
		}
		if (is_fill) {
			struct sp_block *b = sp_block_new(off, n, CHUNK_TYPE_FILL);
			memcpy(&b->fill_val, &buf[off], 4);
			sp_add(sf, b);
		} else {
			struct sp_block *b = sp_block_new(off, n, CHUNK_TYPE_RAW);
			b->file_off = off;
			sp_add(sf, b);
		}
	}
	return 0;
}

static uint32_t sp_count_chunks(const struct sp_file *sf, uint64_t from_off,
				uint64_t to_off)
{
	uint32_t chunks = 0;
	uint64_t last_block = 0;
	uint64_t total_blks = sp_align_up(sf->len, sf->blk_sz) / sf->blk_sz;

	for (const struct sp_block *b = sf->blocks; b; b = b->next) {
		if (b->offset >= to_off)
			break;
		if (b->offset + b->len <= from_off)
			continue;
		uint64_t boff = b->offset > from_off ? b->offset : from_off;
		uint64_t bend = b->offset + b->len < to_off ? b->offset + b->len
							    : to_off;
		if (boff / sf->blk_sz > last_block)
			chunks++;
		chunks++;
		last_block = (bend + sf->blk_sz - 1) / sf->blk_sz;
	}
	if (last_block < total_blks)
		chunks++;
	return chunks;
}

static size_t sp_emit_piece(const struct sp_file *sf, uint64_t from_off,
			    uint64_t to_off, const uint8_t *file, uint8_t *out)
{
	uint64_t total_blks = sp_align_up(sf->len, sf->blk_sz) / sf->blk_sz;
	uint32_t chunks = sp_count_chunks(sf, from_off, to_off);
	uint64_t pos = FILE_HEADER_SZ;
	uint64_t last_block = 0;

	put_le32(out + 0,  SPARSE_HEADER_MAGIC);
	put_le16(out + 4,  1);
	put_le16(out + 6,  0);
	put_le16(out + 8,  FILE_HEADER_SZ);
	put_le16(out + 10, CHUNK_HEADER_SZ);
	put_le32(out + 12, sf->blk_sz);
	put_le32(out + 16, (uint32_t)total_blks);
	put_le32(out + 20, chunks);
	put_le32(out + 24, 0);

	for (const struct sp_block *b = sf->blocks; b; b = b->next) {
		if (b->offset >= to_off)
			break;
		if (b->offset + b->len <= from_off)
			continue;
		uint64_t boff = b->offset > from_off ? b->offset : from_off;
		uint64_t bend = b->offset + b->len < to_off ? b->offset + b->len
							    : to_off;
		uint64_t blen = bend - boff;
		uint64_t block = boff / sf->blk_sz;

		if (block > last_block) {
			put_le16(out + pos, CHUNK_TYPE_DONT_CARE);
			put_le16(out + pos + 2, 0);
			put_le32(out + pos + 4, (uint32_t)(block - last_block));
			put_le32(out + pos + 8, CHUNK_HEADER_SZ);
			pos += CHUNK_HEADER_SZ;
		}

		if (b->type == CHUNK_TYPE_FILL) {
			put_le16(out + pos, CHUNK_TYPE_FILL);
			put_le16(out + pos + 2, 0);
			put_le32(out + pos + 4, (uint32_t)(blen / sf->blk_sz));
			put_le32(out + pos + 8, CHUNK_HEADER_SZ + 4);
			pos += CHUNK_HEADER_SZ;
			put_le32(out + pos, b->fill_val);
			pos += 4;
		} else {
			uint64_t rnd = sp_align_up(blen, sf->blk_sz);
			put_le16(out + pos, CHUNK_TYPE_RAW);
			put_le16(out + pos + 2, 0);
			put_le32(out + pos + 4, (uint32_t)(rnd / sf->blk_sz));
			put_le32(out + pos + 8, (uint32_t)(CHUNK_HEADER_SZ + rnd));
			pos += CHUNK_HEADER_SZ;
			memcpy(out + pos, file + b->file_off + (boff - b->offset),
			       (size_t)blen);
			pos += blen;
			memset(out + pos, 0, (size_t)(rnd - blen));
			pos += rnd - blen;
		}
		last_block = (bend + sf->blk_sz - 1) / sf->blk_sz;
	}

	if (last_block < total_blks) {
		put_le16(out + pos, CHUNK_TYPE_DONT_CARE);
		put_le16(out + pos + 2, 0);
		put_le32(out + pos + 4, (uint32_t)(total_blks - last_block));
		put_le32(out + pos + 8, CHUNK_HEADER_SZ);
		pos += CHUNK_HEADER_SZ;
	}
	return (size_t)pos;
}

struct sp_range { uint64_t from, to; };

static int sp_resperse(const struct sp_file *sf, uint64_t limit,
		       struct sp_range **ranges, size_t *count)
{
	const uint32_t blksz = sf->blk_sz;
	const int64_t overhead = FILE_HEADER_SZ + 2 * CHUNK_HEADER_SZ + 4;
	const int64_t budget = (int64_t)limit - overhead;
	struct sp_range *rs = NULL;
	size_t n = 0, cap = 0;
	uint64_t start = 0;
	uint64_t last_block = 0;
	int64_t file_len = 0;
	int have_prev = 0;

	if (budget < (int64_t)(2 * blksz))
		return -1;

	const struct sp_block *b = sf->blocks;
	while (b) {
		/* portion of this block not yet covered by the current piece */
		uint64_t boff = b->offset > start ? b->offset : start;
		uint64_t rem = b->offset + b->len - boff;
		uint64_t block = boff / blksz;
		int64_t cnt = 0;

		if (block > last_block)
			cnt += CHUNK_HEADER_SZ;
		if (b->type == CHUNK_TYPE_FILL)
			cnt += CHUNK_HEADER_SZ + 4;
		else
			cnt += CHUNK_HEADER_SZ + (int64_t)sp_align_up(rem, blksz);

		if (file_len + cnt > budget) {
			uint64_t close_at;

			if (!have_prev ||
			    budget - file_len - CHUNK_HEADER_SZ > budget / 8) {
				int64_t room = budget - file_len - CHUNK_HEADER_SZ;

				room -= room % (int64_t)blksz;
				if (room >= (int64_t)rem)
					close_at = boff + rem;
				else if (room > 0)
					close_at = boff + (uint64_t)room;
				else
					close_at = boff;
			} else {
				close_at = boff;
			}
			if (close_at <= start)
				close_at = boff + rem;

			if (n == cap) {
				cap = cap ? cap * 2 : 16;
				rs = realloc(rs, cap * sizeof(*rs));
				if (!rs) die("oom");
			}
			rs[n].from = start;
			rs[n].to = close_at;
			n++;

			start = close_at;
			file_len = 0;
			last_block = 0;
			have_prev = 0;
			continue;
		}

		file_len += cnt;
		last_block = (b->offset + b->len + blksz - 1) / blksz;
		have_prev = 1;
		b = b->next;
	}

	if (start < sf->len) {
		if (n == cap) {
			cap = cap ? cap * 2 : 16;
			rs = realloc(rs, cap * sizeof(*rs));
			if (!rs) die("oom");
		}
		rs[n].from = start;
		rs[n].to = sf->len;
		n++;
	}

	*ranges = rs;
	*count = n;
	return 0;
}

static int fb_send_sparse_piece(struct fb_ctx_t *ctx, const char *part,
				const uint8_t *piece, size_t piece_len,
				unsigned current, unsigned total)
{
	char cmd[FB_COMMAND_SZ];
	char body[FB_RESPONSE_SZ];
	uint32_t size;

	fprintf(stderr, "[*] Sending sparse '%s' %u/%u (%zu KB)\n",
		part, current, total, piece_len / 1024);

	snprintf(cmd, sizeof(cmd), "download:%08x", (unsigned)piece_len);
	int rc = fb_command(ctx, cmd, body, sizeof(body), &size);
	if (rc != 'D') {
		fprintf(stderr, "[!] download rejected (%s%s)\n",
			rc == 'F' ? "FAIL" : "no DATA", rc == 'F' ? body : "");
		return -1;
	}

	usb_bulk_send(ctx->hdl, ctx->epout, piece, piece_len);

	rc = fb_read_response(ctx, body, sizeof(body), NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] download failed\n");
		return -1;
	}

	fprintf(stderr, "[*] Writing '%s'\n", part);
	snprintf(cmd, sizeof(cmd), "flash:%s", part);
	rc = fb_command(ctx, cmd, body, sizeof(body), NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] flash failed\n");
		return -1;
	}
	return 0;
}

static int fb_flash_file(struct fb_ctx_t *ctx, const char *part, const char *path)
{
	size_t len = 0;
	uint8_t *buf = read_file(path, &len);
	uint64_t limit = 0;
	struct sp_file sf;
	struct sp_range *ranges = NULL;
	size_t count = 0;
	int r = -1;

	sp_import(&sf, buf, len);

	if (fb_get_max_download(ctx, &limit) || (uint64_t)len <= limit) {
		r = fb_send_raw(ctx, part, path, buf, len);
		goto out;
	}

	if (limit > RESPARSE_LIMIT)
		limit = RESPARSE_LIMIT;

	if (sp_resperse(&sf, limit, &ranges, &count)) {
		fprintf(stderr, "[!] max-download-size too small for sparse split\n");
		goto out;
	}

	uint8_t *piece = malloc((size_t)limit);
	if (!piece) die("oom");

	for (size_t i = 0; i < count; i++) {
		size_t piece_len = sp_emit_piece(&sf, ranges[i].from, ranges[i].to,
						 buf, piece);
		if (piece_len > limit) {
			fprintf(stderr, "[!] internal error: piece %zu exceeds limit\n", i);
			free(piece);
			goto out;
		}
		if (fb_send_sparse_piece(ctx, part, piece, piece_len,
					 (unsigned)(i + 1), (unsigned)count)) {
			free(piece);
			goto out;
		}
	}
	r = 0;
	free(piece);

out:
	free(ranges);
	sp_free(&sf);
	free(buf);
	return r;
}

static int fb_get_samples(struct fb_ctx_t *ctx, const char *path)
{
	char body[FB_RESPONSE_SZ];
	uint32_t total = 0;
	FILE *f = fopen(path, "wbx");
	struct progress_t p;

	if (!f) {
		if (errno == EEXIST)
			fprintf(stderr, "[!] '%s' already exists\n", path);
		else
			fprintf(stderr, "[!] cannot open '%s' for writing\n", path);
		return -1;
	}

	int rc = fb_command(ctx, "upload", body, sizeof(body), &total);
	if (rc != 'D') {
		fprintf(stderr, "[!] upload rejected (%s)\n", rc == 'F' ? "FAIL" : "no DATA");
		fclose(f);
		return -1;
	}

	fprintf(stderr, "[*] receiving samples\n");
	progress_start(&p, total);
	uint64_t remaining = total;
	while (remaining > 0) {
		uint8_t chunk[128 * 1024];
		size_t want = remaining < sizeof(chunk) ? (size_t)remaining : sizeof(chunk);
		int n = fb_raw_recv(ctx, chunk, want);
		if (n <= 0) {
			fprintf(stderr, "\n[!] short read while receiving samples\n");
			goto fail;
		}
		if (fwrite(chunk, 1, (size_t)n, f) != (size_t)n) {
			fprintf(stderr, "\n[!] write to '%s' failed\n", path);
			goto fail;
		}
		remaining -= (uint64_t)n;
		progress_update(&p, (uint64_t)n);
	}
	progress_stop(&p);
	rc = fb_read_response(ctx, body, sizeof(body), NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] upload did not finish cleanly\n");
		fclose(f);
		remove(path);
		return -1;
	}
	fclose(f);
	return 0;
fail:
	fclose(f);
	remove(path);
	return -1;
}

static uint8_t *read_file(const char *path, size_t *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "[!] cannot open '%s'\n", path);
		exit(1);
	}
	fseek(f, 0, SEEK_END);
	long sz = ftell(f);
	fseek(f, 0, SEEK_SET);
	if (sz <= 0) die("file is empty or ftell failed");
	uint8_t *buf = malloc((size_t)sz);
	if (!buf) die("oom");
	if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) die("read failed");
	fclose(f);
	*out_len = (size_t)sz;
	return buf;
}

static void require_egon_fsbl(const uint8_t *buf, size_t len, const char *path)
{
	const char magic[8] = { 'e', 'G', 'O', 'N', '.', 'B', 'T', '0' };

	if (len < 12 || memcmp(buf + 4, magic, 8) != 0) {
		fprintf(stderr,
			"[!] '%s' does not look like an Allwinner fsbl (no eGON.BT0 header)\n"
			"[!] expected <fsbl.bin> <payload.bin> - did you swap the arguments?\n",
			path);
		exit(1);
	}
}

static void usage(void)
{
	fprintf(stderr,
		"%s %s\n"
		"\n"
		"Usage:\n"
		"  %s boot <fsbl.bin> <payload.bin>\n"
		"  %s getvar <name>\n"
		"  %s oem <op> [arg]\n"
		"  %s flash <partition> <image>\n"
		"  %s get_samples <file>\n"
		"  %s reboot\n",
		PROG_NAME, VERSION, PROG_NAME, PROG_NAME, PROG_NAME, PROG_NAME, PROG_NAME, PROG_NAME);
}

static int cmd_boot(const char *fsbl_path, const char *payload_path);
static int cmd_getvar(const char *name);
static int cmd_oem(int argc, char **argv);
static int cmd_flash(const char *part, const char *path);
static int cmd_get_samples(const char *path);
static int cmd_reboot(void);

int main(int argc, char **argv)
{
	if (argc < 2) {
		usage();
		return 1;
	}
	if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
		usage();
		return 0;
	}

	const char *verb = argv[1];
	int rc;

	if (!strcmp(verb, "boot")) {
		if (argc != 4) {
			usage();
			return 1;
		}
		rc = cmd_boot(argv[2], argv[3]);
	} else if (!strcmp(verb, "getvar")) {
		if (argc != 3) {
			usage();
			return 1;
		}
		rc = cmd_getvar(argv[2]);
	} else if (!strcmp(verb, "oem")) {
		if (argc < 3) {
			usage();
			return 1;
		}
		rc = cmd_oem(argc - 1, argv + 1);
	} else if (!strcmp(verb, "flash")) {
		if (argc != 4) {
			usage();
			return 1;
		}
		rc = cmd_flash(argv[2], argv[3]);
	} else if (!strcmp(verb, "get_samples")) {
		if (argc != 3) {
			usage();
			return 1;
		}
		rc = cmd_get_samples(argv[2]);
	} else if (!strcmp(verb, "reboot")) {
		if (argc != 2) {
			usage();
			return 1;
		}
		rc = cmd_reboot();
	} else {
		fprintf(stderr, "[!] unknown verb '%s'\n", verb);
		usage();
		return 1;
	}
	return rc;
}

static int cmd_boot(const char *fsbl_path, const char *payload_path)
{
	unsetenv("LIBUSB_DEBUG");
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);
	if (libusb_init(NULL) != 0) die("libusb_init failed");

	size_t fsbl_len = 0;
	uint8_t *fsbl = read_file(fsbl_path, &fsbl_len);
	require_egon_fsbl(fsbl, fsbl_len, fsbl_path);
	size_t payload_len = 0;
	uint8_t *payload = read_file(payload_path, &payload_len);

	struct xfel_ctx_t ctx;
	memset(&ctx, 0, sizeof(ctx));
	for (size_t i = 0; i < sizeof(CRITICAL_SIGNALS) / sizeof(CRITICAL_SIGNALS[0]); i++)
		signal(CRITICAL_SIGNALS[i], on_signal);

	fprintf(stderr, "[*] looking for robot in FEL mode\n");
		if (!open_fel_device(&ctx)) {
			fprintf(stderr, "[!] no robot in FEL mode found\n");
			return 1;
		}

		if (g_interrupt) {
			fprintf(stderr, "[!] interrupted\n");
			close_fel_device(&ctx);
			libusb_exit(NULL);
			return 130;
		}

		const struct chip_profile_t *prof = find_profile(ctx.version.id);
	if (!prof) {
		fprintf(stderr, "[!] unsupported chip id 0x%08x\n", ctx.version.id);
		close_fel_device(&ctx);
		libusb_exit(NULL);
		return 1;
	}

	const uint32_t fsbl_addr    = prof->fsbl_addr;
	const uint32_t payload_addr = prof->payload_addr;
	const unsigned sleep_sec    = prof->dram_wait_sec;

	fprintf(stderr, "[*] found %s-based robot\n", prof->name);

	enter_critical_section();

	fprintf(stderr, "[*] writing fsbl...\n");
	fel_write_progress(&ctx, fsbl_addr, fsbl, fsbl_len);
	fprintf(stderr, "[*] jumping to fsbl...\n");
	fel_exec(&ctx, fsbl_addr);

	fprintf(stderr, "[*] waiting %us for DRAM training...\n", sleep_sec);
	sleep(sleep_sec); // Actual human comment: DON'T remove this on the MR536 unless you want to disassemble the bot to pull the battery

	fprintf(stderr, "[*] writing payload...\n");
	fel_write_progress(&ctx, payload_addr, payload, payload_len);
	fprintf(stderr, "[*] jumping to payload...\n");
	fel_exec(&ctx, payload_addr);

	leave_critical_section_and_maybe_exit();
	close_fel_device(&ctx);
	free(fsbl);
	free(payload);

	fprintf(stderr, "[*] waiting for the robot to come up as a fastboot device...\n");
	struct fb_ctx_t fb;
	memset(&fb, 0, sizeof(fb));
	if (!fb_wait_gadget(&fb, 30)) {
		fprintf(stderr, "[!] the robot did not enumerate as a fastboot device\n");
		libusb_exit(NULL);
		return 1;
	}
	char version[FB_RESPONSE_SZ];
	if (!fb_check_dustversion(&fb, version)) {
		libusb_exit(NULL);
		return 1;
	}
	fprintf(stderr, "[*] reported dustversion: %s\n", version);
	fb_close(&fb);
	libusb_exit(NULL);
	return 0;
}

static int fb_open_checked(struct fb_ctx_t *out)
{
	memset(out, 0, sizeof(*out));
	unsetenv("LIBUSB_DEBUG");
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);
	if (libusb_init(NULL) != 0) {
		fprintf(stderr, "[!] libusb_init failed\n");
		return -1;
	}

	if (!fb_open_gadget(out)) {
		fprintf(stderr, "[!] no robot in fastboot mode found\n");
		libusb_exit(NULL);
		return -1;
	}
	return 0;
}

static int cmd_getvar(const char *name)
{
	struct fb_ctx_t fb;
	char body[FB_RESPONSE_SZ];

	if (strlen(name) > 250) {
		fprintf(stderr, "[!] variable name too long\n");
		return 1;
	}
	if (fb_open_checked(&fb))
		return 1;

	char cmd[FB_COMMAND_SZ];
	snprintf(cmd, sizeof(cmd), "getvar:%s", name);
	int rc = fb_command(&fb, cmd, body, sizeof(body), NULL);
	fb_close(&fb);
	libusb_exit(NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] getvar:%s failed\n", name);
		return 1;
	}
	printf("%s\n", body);
	return 0;
}

static int cmd_oem(int argc, char **argv)
{
	struct fb_ctx_t fb;
	char body[FB_RESPONSE_SZ];
	char cmd[FB_COMMAND_SZ];

	size_t used = 3;
	memcpy(cmd, "oem", 3);
	for (int i = 1; i < argc; i++) {
		int written = snprintf(cmd + used, sizeof(cmd) - used, " %s", argv[i]);
		if (written < 0 || (size_t)written >= sizeof(cmd) - used) {
			fprintf(stderr, "[!] oem command too long\n");
			return 1;
		}
		used += (size_t)written;
	}
	cmd[used] = 0;

	if (fb_open_checked(&fb))
		return 1;

	int rc = fb_command(&fb, cmd, body, sizeof(body), NULL);
	fb_close(&fb);
	libusb_exit(NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] '%s' failed\n", cmd);
		return 1;
	}
	fprintf(stderr, "[*] OKAY\n");
	return 0;
}

static int cmd_flash(const char *part, const char *path)
{
	struct fb_ctx_t fb;

	if (strlen(part) > 200) {
		fprintf(stderr, "[!] partition name too long\n");
		return 1;
	}

	enter_critical_section();
	if (fb_open_checked(&fb)) {
		leave_critical_section_and_maybe_exit();
		return 1;
	}

	int r = fb_flash_file(&fb, part, path);
	leave_critical_section_and_maybe_exit();
	fb_close(&fb);
	libusb_exit(NULL);
	return r == 0 ? 0 : 1;
}

static int cmd_get_samples(const char *path)
{
	struct fb_ctx_t fb;

	enter_critical_section();
	if (fb_open_checked(&fb)) {
		leave_critical_section_and_maybe_exit();
		return 1;
	}

	int r = fb_get_samples(&fb, path);
	leave_critical_section_and_maybe_exit();
	fb_close(&fb);
	libusb_exit(NULL);
	return r == 0 ? 0 : 1;
}

static int cmd_reboot(void)
{
	struct fb_ctx_t fb;
	char body[FB_RESPONSE_SZ];

	if (fb_open_checked(&fb))
		return 1;

	int rc = fb_command(&fb, "reboot", body, sizeof(body), NULL);
	fb_close(&fb);
	libusb_exit(NULL);
	if (rc != 'O') {
		fprintf(stderr, "[!] reboot failed\n");
		return 1;
	}
	fprintf(stderr, "[*] rebooting\n");
	return 0;
}
