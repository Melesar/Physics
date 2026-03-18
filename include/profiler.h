#ifndef PROFILER_H
#define PROFILER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint32_t labels_storage_capacity;
  uint32_t labels_slots_capacity;
  uint32_t stack_capacity;
  uint32_t samples_memory_size;
  uint32_t frame_headers_capacity;
} profiler_config;

#ifndef BND_PROFILING

#define PROFILE_BLOCK(name)
#define PROFILE_FUNCTION

static void profiler_init_default() {}
static void profiler_init(profiler_config config) {}
static void profiler_teardown() {}

static void profiler_start_frame() {}
static void profiler_end_frame() {}

#else

typedef struct {
  char *label;
  uint64_t start_time;
  uint32_t sample_index;
} profiler_marker;

typedef struct {
  uint32_t label_id;
  uint32_t parent_index;
  uint64_t time;
} profiler_sample;

typedef struct {
  uint32_t offset;
  uint16_t count;
  uint8_t mask;
} profiler_frame_header;

typedef struct {
  profiler_sample *framebuffer;
  uint32_t frame_index;
  uint32_t samples_available;
  uint8_t id;
} profiler_monitor;

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
profiler_marker MARKER_NAME(marker_, __LINE__) __attribute__((__cleanup__(profiler_end_block)))\
  = profiler_start_block(name);

#define PROFILE_FUNCTION PROFILE_BLOCK(__func__)

void profiler_init_default();
void profiler_init(profiler_config config);
void profiler_teardown();

void profiler_start_frame();
void profiler_end_frame();

profiler_marker profiler_start_block(const char *name);
void profiler_end_block(profiler_marker *marker);

bool profiler_get_label(uint32_t label_id, label *label);

bool profiler_monitor_start(profiler_monitor *monitor);
bool profiler_monitor_should_run(profiler_monitor *monitor);
bool profiler_monitor_read_next_frame(profiler_monitor *monitor);
void profiler_monitor_wait_for_frame(const profiler_monitor *monitor);

labels labels_init(uint32_t storage_capacity, uint32_t slots_capacity);
void labels_teardown(labels self);

bool label_is_valid(label l);
uint32_t labels_store(labels *self, label l);
label labels_get(labels *self, uint32_t id);

#endif

#endif
