#pragma once

#include <stdint.h>

#include "osd.h"

void osd_video_init();
void osd_video_update();
void osd_video_fire(const uint32_t line);