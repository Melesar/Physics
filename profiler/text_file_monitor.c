#include "profiler.h"
#include <stdio.h>
#include <string.h>

void* text_file_monitor_run(void *data) {
  FILE *f = fopen("bandura.prof.txt", "w");
  if (!f)
    return NULL;

  profiler_monitor monitor;
  if (!profiler_monitor_start(&monitor)) {
    fclose(f);
    return NULL;
  }

  while(profiler_monitor_should_run(&monitor)) {
    if (!profiler_monitor_read_next_frame(&monitor)) {
      // Sleep?
      continue;
    }


    for(uint32_t frame_index = 0; frame_index < monitor.samples_available; ++frame_index) {
      profiler_sample sample = monitor.framebuffer[frame_index];


      // ...
    }

  }

  fclose(f);
  return NULL;
}
