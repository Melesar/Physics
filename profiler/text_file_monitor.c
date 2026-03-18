#include "profiler.h"
#include <stdio.h>
#include <string.h>

#define MAX_BUFFER_SIZE 100

char buffer[MAX_BUFFER_SIZE + 1];

static char *read_label(profiler_sample sample) {
  label label;
  if (profiler_get_label(sample.label_id, &label)) {
    uint32_t size = label.len < MAX_BUFFER_SIZE ? label.len : MAX_BUFFER_SIZE;
    memcpy(buffer, label.s, size);
    buffer[size] = 0;

    return buffer;
  }

  return "unknown";
}

void* text_file_monitor_run(void *data) {
  FILE *f = fopen("bandura.prof.txt", "w");
  if (!f)
    return NULL;

  profiler_monitor monitor;
  if (!profiler_monitor_start(&monitor)) {
    fclose(f);
    return NULL;
  }

  uint32_t running_count = 0;
  while(profiler_monitor_should_run(&monitor)) {
    if (!profiler_monitor_read_next_frame(&monitor)) {
      // Sleep?
      continue;
    }

    fprintf(f, "Frame %d:\n", running_count++);
    for(uint32_t sample_index = 0; sample_index < monitor.samples_available; ++sample_index) {
      profiler_sample sample = monitor.framebuffer[sample_index];
      char *label = read_label(sample);

      uint64_t time_ns = sample.time;
      double time_ms = time_ns / 1000000.0;

      fprintf(f, "%s: %.5f\n", label, time_ms);
    }

    fprintf(f, "\n");
  }

  fclose(f);
  return NULL;
}
