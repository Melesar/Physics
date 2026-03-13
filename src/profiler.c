#include <stdint.h>
#include <stdlib.h>

typedef enum {
  LOCK_READ = 1,
  LOCK_WRITE = 2,
} lock;

typedef struct {
  uint32_t storage_ptr;
  uint32_t storage_len;
} labels_slot;

typedef struct {
  char *storage;
  labels_slot *slots;
  uint32_t capacity;
  uint32_t storage_ptr;
  uint8_t lock;
} labels;

static labels labels_init(uint32_t storage_capacity, uint32_t slots_capacity) {
  labels self = { 0 };
  self.storage = malloc(storage_capacity);

  uint32_t slots_count = 1;
  while(slots_count < slots_capacity) {
    slots_count <<= 1;
  }

  self.slots = calloc(slots_count, sizeof(labels_slot));
  self.capacity = slots_count;

  return self;
}

static uint32_t labels_store(labels self, char *label) {
  return 0;
}

static char *labels_get(labels self, uint32_t id) {
  return "Hello";
}

static void lables_teardown(labels self) {
  free(self.storage);
  free(self.slots);
}

// ========= TESTS ============

// #define BND_TESTS
#ifdef BND_TESTS
#include "testing.h"
#include <string.h>

static void storing_label_allows_to_retreive_it() {
  labels l = labels_init(32, 32);
  uint32_t id = labels_store(l, "Hello");
  char *label = labels_get(l, id);

  assert(strcmp(label, "Hello") == 0);
}

extern void labels_tests() {
TESTS_BEGIN("Profiling labels")
  TEST(storing_label_allows_to_retreive_it)
TESTS_END
}

#endif
