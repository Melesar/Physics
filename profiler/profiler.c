#include "profiler.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define  __USE_POSIX199309 // What??
#include <time.h>

labels labels_storage;

profiler_marker *markers_stack;
uint32_t markers_count;
uint32_t markers_capacity;

static label marker_label(profiler_marker marker) {
  return (label) { marker.label, strlen(marker.label) };
}

static uint64_t get_time() {
  struct timespec time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
  return time.tv_nsec;
}

static void end_block() {
  profiler_marker last_marker = markers_stack[--markers_count];

  uint64_t elapsed_ns = get_time() - last_marker.start_time;
  uint32_t marker_id = labels_store(&labels_storage, marker_label(last_marker));
}

void profiler_init(profiler_config config) {
  labels_storage = labels_init(config.labels_storage_capacity, config.labels_slots_capacity);
  markers_stack = calloc(config.stack_capacity, sizeof(profiler_marker));
  markers_count = 0;
  markers_capacity = config.stack_capacity;
}

void profiler_teardown() {
  labels_teardown(labels_storage);
}

void profiler_start_frame() {


}

void profiler_end_frame() {

}

profiler_marker profiler_start_block(const char *name) {
  assert(markers_count < markers_capacity);

  profiler_marker marker = { (char*)name, get_time() };
  markers_stack[markers_count++] = marker;

  return marker;
}

void profiler_end_block(profiler_marker *marker) {
  end_block();
}
