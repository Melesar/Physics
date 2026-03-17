#include "profiler.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define  __USE_POSIX199309 // What??
#include <time.h>

labels labels_storage;

profiler_marker *markers_stack;
uint32_t markers_count;
uint32_t markers_capacity;

profiler_sample *samples;
uint32_t samples_capacity;
uint32_t frame_start;
uint32_t frame_offset;
uint32_t max_frame_size;

profiler_frame_header *frame_headers;
uint32_t frame_header_index;
uint32_t frame_headers_capacity;
uint32_t frame_headers_mask;

const uint8_t max_monitors_count = 8;

pthread_t monitor_threads[3];

union monitors_t {
  struct {
    uint8_t count;
    uint8_t mask;
  };
  uint16_t data;
} monitors;

void* text_file_monitor_run();

static label marker_label(profiler_marker marker) {
  return (label) { marker.label, strlen(marker.label) };
}

static uint64_t get_time() {
  struct timespec time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
  return time.tv_nsec;
}

static void end_block() {
  if (markers_count == 0)
    return;

  profiler_marker last_marker = markers_stack[--markers_count];

  uint64_t elapsed_ns = get_time() - last_marker.start_time;
  samples[last_marker.sample_index].time = elapsed_ns;
}

void profiler_init_default() {
  profiler_init((profiler_config) {
    .samples_memory_size = 1 << 20, // 1 Mb
    // .samples_memory_size = 1 << 10, // 1024 B
    .labels_slots_capacity = 128,
    .labels_storage_capacity = 1 << 16, // 32 Kb
    .stack_capacity = 512,
    .frame_headers_capacity = 64,
  });
}

void profiler_init(profiler_config config) {
  labels_storage = labels_init(config.labels_storage_capacity, config.labels_slots_capacity);
  markers_stack = calloc(config.stack_capacity, sizeof(profiler_marker));
  markers_count = 0;
  markers_capacity = config.stack_capacity;

  uint32_t desired_samples_capacity = config.samples_memory_size / sizeof(profiler_sample);
  samples_capacity = 1;
  while(samples_capacity < desired_samples_capacity)
    samples_capacity <<= 1;

  samples = calloc(samples_capacity, sizeof(profiler_sample));
  frame_start = frame_offset = 0;
  max_frame_size = 0;

  frame_headers_capacity = 1;
  while(frame_headers_capacity < config.frame_headers_capacity)
    frame_headers_capacity <<= 1;
  frame_headers_mask = frame_headers_capacity - 1;
  frame_headers = calloc(frame_headers_capacity, sizeof(profiler_frame_header));

  monitors.data = 0;
  pthread_create(&monitor_threads[0], NULL, &text_file_monitor_run, NULL);
}

void profiler_teardown() {
  labels_teardown(labels_storage);
  free(markers_stack);
  free(samples);
  free(frame_headers);
}

void profiler_start_frame() {
  frame_offset = 0;
  markers_count = 0;
}

void profiler_end_frame() {
  max_frame_size = frame_offset > max_frame_size ? frame_offset : max_frame_size;
  frame_start += frame_offset;

  if (samples_capacity < frame_start || samples_capacity - frame_start < max_frame_size) {
    frame_start = 0;
  }
}

profiler_marker profiler_start_block(const char *name) {
  assert(markers_count < markers_capacity);

  uint32_t sample_index = frame_start + frame_offset;
  profiler_marker marker = { (char*)name, get_time(), sample_index };
  profiler_marker parent_marker = markers_count > 0
    ? markers_stack[markers_count - 1]
    : (profiler_marker) { .sample_index = 0xFFFFFFFF };

  samples[sample_index] = (profiler_sample) {
    .label_id = labels_store(&labels_storage, marker_label(marker)),
    .parent_index = parent_marker.sample_index,
    .time = 0
  };

  markers_stack[markers_count++] = marker;
  frame_offset += 1;

  return marker;
}

void profiler_end_block(profiler_marker *marker) {
  end_block();
}

bool profiler_monitor_start(profiler_monitor *monitor) {
  if (monitors.count >= max_monitors_count)
    return false;

  // TODO think this through
  uint8_t monitor_id = __atomic_fetch_add(&monitors.count, (uint8_t)1, __ATOMIC_RELAXED);
  uint8_t new_monitor_mask = 1 << monitor_id;

  __atomic_fetch_or(&monitors.mask, new_monitor_mask, __ATOMIC_RELAXED);

  return true;
}
