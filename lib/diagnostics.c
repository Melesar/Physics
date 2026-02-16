#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#include "pmath.h"

#define RPRAND_IMPLEMENTATION
#include "rprand.h"

percentiles percentiles_init(uint32_t size) {
  percentiles p;
  p.size = size;
  p.count = 0;
  p.values = malloc(sizeof(float) * size);

  rprand_set_seed(time(NULL));
  memcpy(p.seed, rprand_state, 4 * sizeof(uint32_t));

  return p;
}

void percentiles_track(percentiles *p, float value) {
  if (p->count < p->size) {
    p->values[p->count] = value;
  } else {
    memcpy(rprand_state, p->seed, 4 * sizeof(uint32_t));
    uint32_t index = rprand_get_value(0, p->count - 1);
    if (index < p->size) {
      p->values[index] = value;
    }
    memcpy(p->seed, rprand_state, 4 * sizeof(uint32_t));
  }

  p->count += 1;
}

int cmp(const void *a, const void *b) {
  float x = *(float*)a;
  float y = *(float*)b;
  return (x > y) - (x < y);
}

void percentiles_query(percentiles *p, float *p50, float *p75, float *p99) {
  uint32_t count = p->count < p->size ? p->count : p->size;
  qsort(p->values, count, sizeof(float), cmp);

  *p50 = p->values[count / 2];
  *p75 = p->values[count * 3 / 4];
  *p99 = p->values[count * 99 / 100];
}

void percentiles_free(percentiles p) {
  free(p.values);
}
