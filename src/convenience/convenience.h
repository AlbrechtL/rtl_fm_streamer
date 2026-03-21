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

/* a collection of user friendly tools */

#ifndef CONVENIENCE_H
#define CONVENIENCE_H

#include <stdint.h>
#include <sys/types.h>

typedef struct swradio_dev swradio_dev_t;

double atofs(char *s);
double atoft(char *s);
double atofp(char *s);

int nearest_gain(swradio_dev_t *dev, int target_gain);
int verbose_device_open(swradio_dev_t **dev, const char *path);
void swradio_close(swradio_dev_t *dev);
ssize_t swradio_read_sync(swradio_dev_t *dev, void *buf, size_t len);
size_t swradio_get_buffer_size(const swradio_dev_t *dev);
int verbose_set_frequency(swradio_dev_t *dev, uint32_t frequency);
int verbose_set_sample_rate(swradio_dev_t *dev, uint32_t samp_rate);
int verbose_direct_sampling(swradio_dev_t *dev, int on);
int verbose_offset_tuning(swradio_dev_t *dev);
int verbose_auto_gain(swradio_dev_t *dev);
int verbose_gain_set(swradio_dev_t *dev, int gain);
int verbose_ppm_set(swradio_dev_t *dev, int ppm_error);
int verbose_reset_buffer(swradio_dev_t *dev);

#endif

