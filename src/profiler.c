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

static void labels_teardown(labels self) {
  free(self.storage);
  free(self.slots);
}

// ========= TESTS ============

#ifdef BND_TESTS
#include "testing.h"
#include <string.h>

static void storing_label_allows_to_retreive_it() {
  labels l = labels_init(32, 32);
  uint32_t id = labels_store(l, "Hello");
  char *label = labels_get(l, id);

  assert(strcmp(label, "Hello") == 0);

  labels_teardown(l);
}

static void requesting_invalid_id_gives_null_pointer() {
  labels l = labels_init(32, 32);
  char *label = labels_get(l, 200);

  assert(label == NULL);

  labels_teardown(l);
}

static void storing_same_label_multiple_times_gives_the_same_id() {
  labels l = labels_init(32, 32);
  uint32_t id = labels_store(l, "Hello");

  for(int i = 0; i < 10; ++i) {
    uint32_t another_id = labels_store(l, "Hello");
    assert(id == another_id);
  }

  labels_teardown(l);
}

static void different_labels_produce_different_ids() {
  labels l = labels_init(1000, 32);
  uint32_t ids[] = {
    labels_store(l, "Hello"),
    labels_store(l, "foo"),
    labels_store(l, "bas"),
    labels_store(l, "42"),
    labels_store(l, "Starship Millenium Falcon"),
  }

  for(int i = 0; i < 5; ++i) {
    for(int j = 0; j < 5; ++j) {
      if (i != j)
        assert(ids[i] != ids[j]);
    }
  }

  labels_teardown(l);
}

extern void labels_tests() {
TESTS_BEGIN("Profiling labels")
  TEST(storing_label_allows_to_retreive_it)
  TEST(requesting_invalid_id_gives_null_pointer)
  TEST(storing_same_label_multiple_times_gives_the_same_id)
  TEST(different_labels_produce_different_ids)
TESTS_END
}

#endif
