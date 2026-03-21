/*
 * Copyright (C) 2014 by Kyle Keen <keenerd@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* a collection of user friendly tools
 * todo: use strtol for more flexible int parsing
 * */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#include "convenience.h"

#ifndef _WIN32
#ifndef V4L2_SDR_FMT_CU8
#define V4L2_SDR_FMT_CU8 v4l2_fourcc('C', 'U', '0', '8')
#endif

struct swradio_dev {
	int fd;
	char path[PATH_MAX];
	uint32_t active_caps;
	uint32_t buffer_size;
	uint32_t pixelformat;
	uint32_t sdr_tuner_index;
	uint32_t rf_tuner_index;
	uint32_t sdr_tuner_caps;
	uint32_t rf_tuner_caps;
	int supports_read;
};

static int xioctl(int fd, unsigned long request, void *arg)
{
	int result;

	do {
		result = ioctl(fd, request, arg);
	} while (result == -1 && errno == EINTR);

	return result;
}

static uint32_t active_caps(const struct v4l2_capability *cap)
{
	if (cap->capabilities & V4L2_CAP_DEVICE_CAPS) {
		return cap->device_caps;
	}

	return cap->capabilities;
}

static int query_tuner_caps(swradio_dev_t *dev)
{
	struct v4l2_tuner tuner;
	uint32_t index;
	int found_sdr;
	int found_rf;

	found_sdr = 0;
	found_rf = 0;
	dev->sdr_tuner_index = 0;
	dev->rf_tuner_index = 0;
	dev->sdr_tuner_caps = 0;
	dev->rf_tuner_caps = 0;

	for (index = 0; index < 8; index++) {
		memset(&tuner, 0, sizeof(tuner));
		tuner.index = index;
		if (xioctl(dev->fd, VIDIOC_G_TUNER, &tuner) == -1) {
			if (errno == EINVAL) {
				break;
			}
			return -1;
		}

		if (!found_sdr && tuner.type == V4L2_TUNER_SDR) {
			dev->sdr_tuner_index = index;
			dev->sdr_tuner_caps = tuner.capability;
			found_sdr = 1;
		}

		if (!found_rf && tuner.type == V4L2_TUNER_RF) {
			dev->rf_tuner_index = index;
			dev->rf_tuner_caps = tuner.capability;
			found_rf = 1;
		}
	}

	if (!found_sdr) {
		errno = ENODEV;
		return -1;
	}

	return 0;
}

static int hz_to_v4l2_units(uint32_t tuner_caps, uint32_t hz, uint32_t *result)
{
	uint64_t value;

	if (tuner_caps & V4L2_TUNER_CAP_1HZ) {
		*result = hz;
		return 0;
	}

	if (tuner_caps & V4L2_TUNER_CAP_LOW) {
		value = ((uint64_t)hz * 2u + 62u) / 125u;
		*result = (uint32_t)value;
		return 0;
	}

	value = ((uint64_t)hz + 31250u) / 62500u;
	*result = (uint32_t)value;
	return 0;
}

static int set_frequency_for_tuner(swradio_dev_t *dev, uint32_t tuner_index, uint32_t tuner_type,
	uint32_t tuner_caps, uint32_t hz)
{
	struct v4l2_frequency frequency;

	memset(&frequency, 0, sizeof(frequency));
	frequency.tuner = tuner_index;
	frequency.type = tuner_type;
	hz_to_v4l2_units(tuner_caps, hz, &frequency.frequency);

	return xioctl(dev->fd, VIDIOC_S_FREQUENCY, &frequency);
}

static int query_control(const swradio_dev_t *dev, uint32_t id, struct v4l2_queryctrl *query)
{
	memset(query, 0, sizeof(*query));
	query->id = id;
	if (xioctl(dev->fd, VIDIOC_QUERYCTRL, query) == -1) {
		return -1;
	}
	if (query->flags & V4L2_CTRL_FLAG_DISABLED) {
		errno = ENOTSUP;
		return -1;
	}
	return 0;
}

static int set_control(const swradio_dev_t *dev, uint32_t id, int value)
{
	struct v4l2_control ctrl;

	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = id;
	ctrl.value = value;
	return xioctl(dev->fd, VIDIOC_S_CTRL, &ctrl);
}

static int pick_manual_gain_control(swradio_dev_t *dev, uint32_t *gain_id, uint32_t *auto_id,
	struct v4l2_queryctrl *query)
{
	static const struct {
		uint32_t gain_id;
		uint32_t auto_id;
	} candidates[] = {
		{ V4L2_CID_RF_TUNER_LNA_GAIN, V4L2_CID_RF_TUNER_LNA_GAIN_AUTO },
		{ V4L2_CID_RF_TUNER_RF_GAIN, 0 },
		{ V4L2_CID_RF_TUNER_IF_GAIN, V4L2_CID_RF_TUNER_IF_GAIN_AUTO },
		{ V4L2_CID_RF_TUNER_MIXER_GAIN, V4L2_CID_RF_TUNER_MIXER_GAIN_AUTO },
	};
	size_t index;

	for (index = 0; index < sizeof(candidates) / sizeof(candidates[0]); index++) {
		if (query_control(dev, candidates[index].gain_id, query) == 0) {
			*gain_id = candidates[index].gain_id;
			*auto_id = candidates[index].auto_id;
			return 0;
		}
	}

	errno = ENOTSUP;
	return -1;
}
#endif

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff;

	suff = 1.0;
	len = strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
	case 'g':
	case 'G':
		suff *= 1e3;
		/* fall through */
	case 'm':
	case 'M':
		suff *= 1e3;
		/* fall through */
	case 'k':
	case 'K':
		suff *= 1e3;
		suff *= atof(s);
		s[len-1] = last;
		return suff;
	}
	s[len-1] = last;
	return atof(s);
}

double atoft(char *s)
/* time suffixes, returns seconds */
{
	char last;
	int len;
	double suff;

	suff = 1.0;
	len = strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
	case 'h':
	case 'H':
		suff *= 60;
		/* fall through */
	case 'm':
	case 'M':
		suff *= 60;
		/* fall through */
	case 's':
	case 'S':
		suff *= atof(s);
		s[len-1] = last;
		return suff;
	}
	s[len-1] = last;
	return atof(s);
}

double atofp(char *s)
/* percent suffixes */
{
	char last;
	int len;
	double suff;

	suff = 1.0;
	len = strlen(s);
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
	case '%':
		suff *= 0.01;
		suff *= atof(s);
		s[len-1] = last;
		return suff;
	}
	s[len-1] = last;
	return atof(s);
}

int nearest_gain(swradio_dev_t *dev, int target_gain)
{
#ifndef _WIN32
	struct v4l2_queryctrl query;
	uint32_t gain_id;
	uint32_t auto_id;
	int nearest;

	(void)auto_id;
	if (pick_manual_gain_control(dev, &gain_id, &auto_id, &query) == -1) {
		return target_gain;
	}

	nearest = target_gain;
	if (nearest < query.minimum) {
		nearest = query.minimum;
	}
	if (nearest > query.maximum) {
		nearest = query.maximum;
	}
	if (query.step > 1) {
		nearest = query.minimum + ((nearest - query.minimum) / query.step) * query.step;
	}

	return nearest;
#else
	(void)dev;
	(void)target_gain;
	return -1;
#endif
}

int verbose_device_open(swradio_dev_t **dev, const char *path)
{
#ifndef _WIN32
	struct v4l2_capability capability;
	struct v4l2_fmtdesc fmtdesc;
	struct v4l2_format format;
	swradio_dev_t *device;
	int has_cu8;

	device = calloc(1, sizeof(*device));
	if (!device) {
		return -1;
	}

	device->fd = open(path, O_RDWR | O_NONBLOCK);
	if (device->fd == -1) {
		free(device);
		return -1;
	}

	memset(&capability, 0, sizeof(capability));
	if (xioctl(device->fd, VIDIOC_QUERYCAP, &capability) == -1) {
		close(device->fd);
		free(device);
		return -1;
	}

	device->active_caps = active_caps(&capability);
	device->supports_read = (device->active_caps & V4L2_CAP_READWRITE) != 0;
	if ((device->active_caps & V4L2_CAP_SDR_CAPTURE) == 0 ||
	    (device->active_caps & V4L2_CAP_TUNER) == 0) {
		fprintf(stderr, "ERROR: %s is not an SDR capture device.\n", path);
		close(device->fd);
		free(device);
		errno = ENODEV;
		return -1;
	}

	if (!device->supports_read) {
		fprintf(stderr, "ERROR: %s does not support V4L2 read() capture yet.\n", path);
		close(device->fd);
		free(device);
		errno = ENOTSUP;
		return -1;
	}

	if (query_tuner_caps(device) == -1) {
		close(device->fd);
		free(device);
		return -1;
	}

	has_cu8 = 0;
	memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.type = V4L2_BUF_TYPE_SDR_CAPTURE;
	while (xioctl(device->fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
		if (fmtdesc.pixelformat == V4L2_SDR_FMT_CU8) {
			has_cu8 = 1;
			break;
		}
		fmtdesc.index++;
	}

	if (!has_cu8) {
		fprintf(stderr, "ERROR: %s does not expose CU8 IQ samples.\n", path);
		close(device->fd);
		free(device);
		errno = ENOTSUP;
		return -1;
	}

	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_SDR_CAPTURE;
	format.fmt.sdr.pixelformat = V4L2_SDR_FMT_CU8;
	if (xioctl(device->fd, VIDIOC_S_FMT, &format) == -1) {
		close(device->fd);
		free(device);
		return -1;
	}

	device->buffer_size = format.fmt.sdr.buffersize;
	if (device->buffer_size == 0) {
		device->buffer_size = 16384;
	}
	device->pixelformat = format.fmt.sdr.pixelformat;
	strncpy(device->path, path, sizeof(device->path) - 1);
	device->path[sizeof(device->path) - 1] = '\0';

	fprintf(stderr, "Using swradio device %s\n", device->path);
	fprintf(stderr, "  Driver: %s\n", capability.driver);
	fprintf(stderr, "  Card: %s\n", capability.card);
	fprintf(stderr, "  Bus: %s\n", capability.bus_info);
	fprintf(stderr, "  SDR tuner index: %u\n", device->sdr_tuner_index);
	fprintf(stderr, "  RF tuner index: %u\n", device->rf_tuner_caps ? device->rf_tuner_index : device->sdr_tuner_index);
	fprintf(stderr, "  IQ format: CU8\n");
	fprintf(stderr, "  Capture mode: read()\n");

	*dev = device;
	return 0;
#else
	(void)dev;
	(void)path;
	return -1;
#endif
}

void swradio_close(swradio_dev_t *dev)
{
#ifndef _WIN32
	if (!dev) {
		return;
	}
	if (dev->fd >= 0) {
		close(dev->fd);
	}
	free(dev);
#else
	(void)dev;
#endif
}

ssize_t swradio_read_sync(swradio_dev_t *dev, void *buf, size_t len)
{
#ifndef _WIN32
	struct pollfd pollfd;
	ssize_t result;

	pollfd.fd = dev->fd;
	pollfd.events = POLLIN;
	pollfd.revents = 0;

	result = poll(&pollfd, 1, 200);
	if (result == 0) {
		return 0;
	}
	if (result == -1) {
		if (errno == EINTR) {
			return 0;
		}
		return -1;
	}

	result = read(dev->fd, buf, len);
	if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		return 0;
	}
	return result;
#else
	(void)dev;
	(void)buf;
	(void)len;
	return -1;
#endif
}

size_t swradio_get_buffer_size(const swradio_dev_t *dev)
{
	if (!dev) {
		return 0;
	}
	return dev->buffer_size;
}

int verbose_set_frequency(swradio_dev_t *dev, uint32_t frequency)
{
#ifndef _WIN32
	int r;

	r = set_frequency_for_tuner(dev,
		dev->rf_tuner_caps ? dev->rf_tuner_index : dev->sdr_tuner_index,
		dev->rf_tuner_caps ? V4L2_TUNER_RF : V4L2_TUNER_SDR,
		dev->rf_tuner_caps ? dev->rf_tuner_caps : dev->sdr_tuner_caps,
		frequency);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq on %s.\n", dev->path);
	} else {
		fprintf(stderr, "Tuned to %u Hz.\n", frequency);
	}
	return r;
#else
	(void)dev;
	(void)frequency;
	return -1;
#endif
}

int verbose_set_sample_rate(swradio_dev_t *dev, uint32_t samp_rate)
{
#ifndef _WIN32
	int r;

	r = set_frequency_for_tuner(dev, dev->sdr_tuner_index, V4L2_TUNER_SDR,
		dev->sdr_tuner_caps, samp_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
	} else {
		fprintf(stderr, "Sampling at %u S/s.\n", samp_rate);
	}
	return r;
#else
	(void)dev;
	(void)samp_rate;
	return -1;
#endif
}

int verbose_direct_sampling(swradio_dev_t *dev, int on)
{
	(void)dev;
	if (on != 0) {
		fprintf(stderr, "WARNING: Direct sampling is not supported through the generic swradio backend.\n");
	}
	return on == 0 ? 0 : -1;
}

int verbose_offset_tuning(swradio_dev_t *dev)
{
	(void)dev;
	fprintf(stderr, "WARNING: Offset tuning is not supported through the generic swradio backend.\n");
	return -1;
}

int verbose_auto_gain(swradio_dev_t *dev)
{
#ifndef _WIN32
	struct v4l2_queryctrl query;
	int applied;

	applied = 0;
	if (query_control(dev, V4L2_CID_RF_TUNER_LNA_GAIN_AUTO, &query) == 0 &&
	    set_control(dev, V4L2_CID_RF_TUNER_LNA_GAIN_AUTO, 1) == 0) {
		applied = 1;
	}
	if (query_control(dev, V4L2_CID_RF_TUNER_IF_GAIN_AUTO, &query) == 0 &&
	    set_control(dev, V4L2_CID_RF_TUNER_IF_GAIN_AUTO, 1) == 0) {
		applied = 1;
	}
	if (query_control(dev, V4L2_CID_RF_TUNER_MIXER_GAIN_AUTO, &query) == 0 &&
	    set_control(dev, V4L2_CID_RF_TUNER_MIXER_GAIN_AUTO, 1) == 0) {
		applied = 1;
	}

	if (!applied) {
		fprintf(stderr, "No V4L2 gain control is exposed by %s; leaving device gain unchanged.\n", dev->path);
		return 0;
	}

	fprintf(stderr, "Tuner gain set to automatic.\n");
	return 0;
#else
	(void)dev;
	return -1;
#endif
}

int verbose_gain_set(swradio_dev_t *dev, int gain)
{
#ifndef _WIN32
	struct v4l2_queryctrl query;
	uint32_t gain_id;
	uint32_t auto_id;
	int nearest;

	if (pick_manual_gain_control(dev, &gain_id, &auto_id, &query) == -1) {
		fprintf(stderr, "WARNING: No manual V4L2 gain control is exposed by %s.\n", dev->path);
		return -1;
	}

	if (auto_id != 0) {
		set_control(dev, auto_id, 0);
	}

	nearest = nearest_gain(dev, gain);
	if (set_control(dev, gain_id, nearest) == -1) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
		return -1;
	}

	fprintf(stderr, "Tuner gain control set to %d (driver-specific units).\n", nearest);
	return 0;
#else
	(void)dev;
	(void)gain;
	return -1;
#endif
}

int verbose_ppm_set(swradio_dev_t *dev, int ppm_error)
{
	(void)dev;
	if (ppm_error != 0) {
		fprintf(stderr, "WARNING: PPM correction is not supported through the generic swradio backend.\n");
		return -1;
	}
	return 0;
}

int verbose_reset_buffer(swradio_dev_t *dev)
{
	(void)dev;
	return 0;
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
