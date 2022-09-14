#pragma once

#include "osd.h"

typedef enum {
  FIELD_START_BROAD,
  FIELD_START_SHORT,
  FIELD_LINES,
  FIELD_END_SHORT
} osd_timer_state_t;

void osd_timer_init();
