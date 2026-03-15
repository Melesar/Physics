#ifndef PROFILER_H
#define PROFILER_H

#include <stdint.h>

typedef struct {
  uint32_t labels_storage_capacity;
  uint32_t labels_slots_capacity;
  uint32_t stack_capacity;
} profiler_config;

#ifndef BND_PROFILING

#define PROFILE_BLOCK(name)
#define PROFILE_FUNCTION

void profiler_init(profiler_config config) {}
void profiler_teardown() {}
#else

typedef struct {
  char *label;
  uint64_t start_time;
} profiler_marker;

typedef struct {
  char *s;
  uint8_t len;
} label;

typedef struct {
  uint64_t value;
} labels_slot;

typedef struct {
  char *storage;
  labels_slot *slots;
  uint32_t capacity;
  uint32_t mask;
  uint32_t storage_ptr;
} labels;

#define LABELS_STORAGE_FULL 0xFFFFFFFF
#define INVALID_LABEL (label) { NULL, 0 }

#define CONCAT(a, b) a##b
#define MARKER_NAME(a,b) CONCAT(a,b)

#define PROFILE_BLOCK(name)\
profiler_marker MARKER_NAME(marker_, __LINE__) __attribute__((__cleanup__(profiler_clean_up)))\
  = profiler_start_block(name);

#define PROFILE_FUNCTION PROFILE_BLOCK(__func__)

void profiler_init(profiler_config config);
void profiler_teardown();

profiler_marker profiler_start_block(const char *name);
void profiler_end_block();
void profiler_clean_up(profiler_marker *marker);

labels labels_init(uint32_t storage_capacity, uint32_t slots_capacity);
uint32_t labels_store(labels *self, label l);
label labels_get(labels *self, uint32_t id);
void labels_teardown(labels self);

#endif

#endif
