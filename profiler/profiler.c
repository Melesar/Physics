#include "profiler.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    // .samples_memory_size = 1 << 20, // 1 Mb
    .samples_memory_size = 1 << 10, // 1024 B
    .labels_slots_capacity = 128,
    .labels_storage_capacity = 1 << 16, // 32 Kb
    .stack_capacity = 512,
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
}

static void dump() {
  FILE* f = fopen("profiler_dump.txt", "w");
  if (!f) {
    printf("Failed to open dump file\n");
    return;
  }

  char buffer[512];
  const uint32_t step = 4;
  for (uint32_t i = 0; i < frame_start; i += step) {
    fprintf(f, "Frame %d:\n", i / step);

    for (uint32_t j = 0; j < step; ++j) {
      profiler_sample sample = samples[i + j];

      label title = labels_get(&labels_storage, sample.label_id);
      memcpy(buffer, title.s, title.len);
      buffer[title.len] = 0;

      uint64_t time_ns = sample.time;
      double time_ms = time_ns / 1000000.0;

      fprintf(f, "%s: %.5f\n", buffer, time_ms);
    }

    fprintf(f, "\n");
  }

  fclose(f);
}

void profiler_teardown() {
  dump();

  labels_teardown(labels_storage);
  free(markers_stack);
  free(samples);
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
    dump();
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
