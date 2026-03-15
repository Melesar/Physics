#ifndef PROFILER_H
#define PROFILER_H

typedef struct {

} profiler_config;

#ifndef BND_PROFILING

#define PROFILE_BLOCK(name)
#define PROFILE_FUNCTION

void profiler_init(profiler_config config) {}
void profiler_teardown() {}
#else

#include <stdint.h>

typedef struct {
  char *label;
  uint64_t start_time;
} profiler_marker;

#define CONCAT(a, b) a##b
#define MARKER_NAME(a,b) CONCAT(a,b)

#define PROFILE_BLOCK(name)\
profiler_marker MARKER_NAME(marker_, __LINE__) __attribute__((__cleanup__(profiler_clean_up)))\
  = profiler_start_block(name);

#define PROFILE_FUNCTION PROFILE_BLOCK(__func__)

void profiler_init(profiler_config config);
void profiler_teardown();

profiler_marker profiler_start_block(const char *name);
void profiler_end_block(profiler_marker marker);
void profiler_clean_up(profiler_marker *marker);

#endif

#endif
