/*
 * valetudo-helper-felboot
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
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <libusb.h>

#define VERSION "2026.07.0"
#define PROG_NAME "valetudo-helper-felboot"

#define FEL_VID 0x1f3a
#define FEL_PID 0xefe8

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
};

static const struct chip_profile_t PROFILES[] = {
	{ 0x00191200, "MR536", 0x0004C000, 0x4A000000 },
	{ 0x00189000, "MR813", 0x00028000, 0x4A000000 },
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
		if (r != 0)
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
		if (r != 0)
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

static void fel_write(struct xfel_ctx_t *ctx, uint32_t addr, const uint8_t *buf, size_t len)
{
	while (len > 0) {
		size_t n = len > 65536 ? 65536 : len;
		send_fel_request(ctx, 0x101, addr, (uint32_t)n);
		usb_write(ctx, buf, n);
		read_fel_status(ctx);
		addr += (uint32_t)n;
		buf += n;
		len -= n;
	}
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
		fprintf(stderr, "] %.2f MB/s    ", speed / (1024.0 * 1024.0));
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

static int open_fel_device(struct xfel_ctx_t *ctx)
{
	libusb_device **list = NULL;
	ssize_t count = libusb_get_device_list(NULL, &list);
	int found = 0;

	for (ssize_t i = 0; i < count; i++) {
		struct libusb_device_descriptor desc;
		if (libusb_get_device_descriptor(list[i], &desc) != 0)
			continue;
		if (desc.idVendor != FEL_VID || desc.idProduct != FEL_PID)
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

	if (libusb_kernel_driver_active(ctx->hdl, 0))
		libusb_detach_kernel_driver(ctx->hdl, 0);
	if (libusb_claim_interface(ctx->hdl, 0) != 0)
		die("could not claim usb interface");

	struct libusb_config_descriptor *config;
	ctx->epin = 0;
	ctx->epout = 0;
	if (libusb_get_active_config_descriptor(libusb_get_device(ctx->hdl), &config) != 0)
		die("could not get config descriptor");
	for (int i = 0; i < config->bNumInterfaces; i++) {
		const struct libusb_interface *iface = config->interface + i;
		for (int j = 0; j < iface->num_altsetting; j++) {
			const struct libusb_interface_descriptor *s = iface->altsetting + j;
			for (int k = 0; k < s->bNumEndpoints; k++) {
				const struct libusb_endpoint_descriptor *ep = s->endpoint + k;
				if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK)
					continue;
				if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN)
					ctx->epin = ep->bEndpointAddress;
				else
					ctx->epout = ep->bEndpointAddress;
			}
		}
	}
	libusb_free_config_descriptor(config);

	if (!ctx->epin || !ctx->epout)
		die("could not locate bulk endpoints");

	fel_version(ctx);
	return 1;
}

static void close_fel_device(struct xfel_ctx_t *ctx);

static struct xfel_ctx_t *g_ctx = NULL;

static void on_signal(int sig)
{
	(void)sig;
	if (g_ctx)
		close_fel_device(g_ctx);
	libusb_exit(NULL);
	fprintf(stderr, "\n[!] interrupted\n");
	exit(130);
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
	if (sz < 0) die("ftell failed");
	uint8_t *buf = malloc((size_t)sz);
	if (!buf) die("oom");
	if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) die("read failed");
	fclose(f);
	*out_len = (size_t)sz;
	return buf;
}

static void usage(const char *argv0)
{
	fprintf(stderr,
		"%s %s\n"
		"\n"
		"Usage:\n"
		"  %s <fsbl.bin> <payload.bin>\n",
		PROG_NAME, VERSION, argv0);
}

int main(int argc, char **argv)
{
	const char *fsbl_path    = NULL;
	const char *payload_path = NULL;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			usage(argv[0]);
			return 0;
		} else if (argv[i][0] == '-') {
			fprintf(stderr, "[!] unknown option '%s'\n", argv[i]);
			usage(argv[0]);
			return 1;
		} else if (!fsbl_path) {
			fsbl_path = argv[i];
		} else if (!payload_path) {
			payload_path = argv[i];
		} else {
			fprintf(stderr, "[!] unexpected extra argument '%s'\n", argv[i]);
			usage(argv[0]);
			return 1;
		}
	}

	if (!fsbl_path || !payload_path) {
		usage(argv[0]);
		return 1;
	}

	unsetenv("LIBUSB_DEBUG");
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);
	if (libusb_init(NULL) != 0) die("libusb_init failed");

	size_t fsbl_len = 0;
	uint8_t *fsbl = read_file(fsbl_path, &fsbl_len);
	size_t payload_len = 0;
	uint8_t *payload = read_file(payload_path, &payload_len);

	struct xfel_ctx_t ctx;
	memset(&ctx, 0, sizeof(ctx));
	g_ctx = &ctx;
	signal(SIGINT,  on_signal);
	signal(SIGTERM, on_signal);

	fprintf(stderr, "[*] looking for robot in FEL mode\n");
	if (!open_fel_device(&ctx)) {
			fprintf(stderr, "[!] no robot in FEL mode found\n");
		return 1;
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
	const unsigned sleep_sec    = 22; // Why not 20? Why not 25? Because fuck you that's why. (It probably could be 20, but this is more fun)

	fprintf(stderr, "[*] found %s-based robot\n", prof->name);
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

	fprintf(stderr, "[*] done, now continue with the rooting instructions\n");

	close_fel_device(&ctx);
	libusb_exit(NULL);
	free(fsbl);
	free(payload);
	return 0;
}
