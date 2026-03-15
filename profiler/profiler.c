#include "profiler.h"

profiler_marker profiler_start_block(const char *name) {
  return (profiler_marker) { 0 };
}

void profiler_end_block(profiler_marker marker) {

}

void profiler_clean_up(profiler_marker *marker) {
  profiler_end_block(*marker);
}

void profiler_init(profiler_config config) {

}

void profiler_teardown() {

}
