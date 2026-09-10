// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Device;

/**
 * Reports whether an output interface is currently open for this USB audio codec.
 *
 * The device itself implements AUDIO_CODEC_TYPE. This helper exposes only its
 * hot-plug state; PCM, volume, and mute operations use the standard audio codec API.
 */
bool usb_host_uac_is_connected(struct Device* device);

#ifdef __cplusplus
}
#endif
