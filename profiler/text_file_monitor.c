#ifdef __linux__
  #define _POSIX_C_SOURCE 199309L // This is to have clock_gettime, which is otherwise not available under -std=c99
  #define _XOPEN_SOURCE 500
#endif

#include "profiler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_BUFFER_SIZE 100
#define CALL_TREE_CAPACITY 1024

typedef struct {
  uint32_t label_id;
  uint32_t parent_index;
  uint64_t total_time;
  uint32_t call_count;
} final_sample;

char buffer[MAX_BUFFER_SIZE + 1];

final_sample *call_tree;
uint32_t tree_index;

static uint32_t process_samples(profiler_sample *samples, uint32_t samples_count, final_sample *output, uint32_t available_capacity) {
  return 0;
}

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

  call_tree = calloc(CALL_TREE_CAPACITY, sizeof(final_sample));
  tree_index = 0;

  profiler_monitor monitor;
  if (!profiler_monitor_start(&monitor)) {
    fclose(f);
    free(call_tree);
    return NULL;
  }

  uint32_t running_count = 0;
  while(profiler_monitor_should_run(&monitor)) {
    profiler_monitor_wait_for_frame(&monitor);

    while (profiler_monitor_read_next_frame(&monitor)) {
      uint32_t processed_count = process_samples(monitor.framebuffer, monitor.samples_available, call_tree, CALL_TREE_CAPACITY);
      (void) processed_count;

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
  }

  fclose(f);
  free(call_tree);
  return NULL;
}

// ========== TESTS =================

#ifdef BND_TESTS

#include "testing.h"

static void work() {
  usleep(1000);
}

static void simulate_frame() {
  PROFILE_FUNCTION

  {
    for(int i = 0; i < 10; ++i)
    {
      PROFILE_BLOCK("water_the_plant")
      work();

      if (i == 5 || i == 7) {
        PROFILE_BLOCK("change_soil")
        work();
      }
    }

    {
      PROFILE_BLOCK("clean_dishes")
      work();
    }

    for(int i = 0; i < 3; ++i) {
      PROFILE_BLOCK("clean_the_room")

      if (i == 0) {
        PROFILE_BLOCK("collect_toys")
        work();
      }

      {
        PROFILE_BLOCK("wipe_dust")

        for (int j = 0; j < 4; ++j) {
          PROFILE_BLOCK("sort_shelf")
          work();
        }

        work();
      }

      {
        PROFILE_BLOCK("vaccuum")
        work();
      }
    }
  }

  work();
}

void total_count_and_time_is_calculated_correctly() {
  call_tree = calloc(CALL_TREE_CAPACITY, sizeof(final_sample));

  profiler_config config = profiler_default_config();
  config.auto_enable_monitors = false;

  profiler_init(config);

  profiler_monitor monitor;
  profiler_monitor_start(&monitor);

  profiler_start_frame();

  simulate_frame();

  profiler_end_frame();

  assert(profiler_monitor_read_next_frame(&monitor));
  assert(monitor.samples_available == 36);

  uint32_t processed_count = process_samples(monitor.framebuffer, monitor.samples_available, call_tree, CALL_TREE_CAPACITY);
  assert(processed_count == 9);

  assert(!profiler_monitor_read_next_frame(&monitor));

  profiler_teardown();
  free(call_tree);
}

void text_file_monitor_tests() {
  TESTS_BEGIN("Text file monitor")
    TEST(total_count_and_time_is_calculated_correctly)
  TESTS_END
}

#endif
