#if defined(MSC_VER)
  #pragma message ("Profiling is not supported with MSVC")
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define LOCK_WRITE 0x80
#define LABELS_STORAGE_FULL 0xFFFFFFFF
#define INVALID_LABEL (label) { NULL, 0 }

typedef struct {
  char *s;
  uint8_t len;
} label;

typedef struct {
  uint32_t storage_ptr;
  uint32_t storage_len;
} labels_slot;

typedef struct {
  char *storage;
  labels_slot *slots;
  uint32_t capacity;
  uint32_t mask;
  uint32_t storage_ptr;
  uint8_t lock;
} labels;

static inline uint32_t hash(label l) {
  uint32_t h = 2166136261u;
  for (uint8_t i = 0; i < l.len; i++) {
      h ^= (uint8_t)l.s[i];
      h *= 16777619u;
  }
  return h;
}

static bool label_is_valid(label l) { return l.s != NULL && l.len != 0; }

static inline bool slot_free(labels_slot slot) {
  return slot.storage_ptr == (uint32_t)~0;
}

static void slot_write(labels* labels, labels_slot *slot, label l) {
  uint8_t no_lock = 0;
  while (!__atomic_compare_exchange_n(&labels->lock, &no_lock, LOCK_WRITE, true, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
    no_lock = 0;
  }

  slot->storage_ptr = labels->storage_ptr;
  slot->storage_len = l.len;

  memcpy(labels->storage + labels->storage_ptr, l.s, l.len);
  labels->storage_ptr += l.len;

  __atomic_exchange_n(&labels->lock, 0, __ATOMIC_RELEASE);

}

static label slot_read(const labels *labels, labels_slot slot) {
  return !slot_free(slot) ? (label) { labels->storage + slot.storage_ptr, slot.storage_len } : INVALID_LABEL;
}


static labels labels_init(uint32_t storage_capacity, uint32_t slots_capacity) {
  labels self = { 0 };
  self.storage = malloc(storage_capacity);

  uint32_t slots_count = 1;
  while(slots_count < slots_capacity) {
    slots_count <<= 1;
  }

  uint32_t slots_memory_size = slots_count * sizeof(labels_slot);
  self.slots = malloc(slots_memory_size);
  memset(self.slots, 0xFF, slots_memory_size);

  self.capacity = slots_count;
  self.mask = slots_count - 1;

  return self;
}

static uint32_t labels_store(labels *self, label l) {
  uint32_t h = hash(l);
  uint32_t index = h & self->mask;
  uint32_t initial_index = index;

  if (slot_free(self->slots[index])) {
    slot_write(self, &self->slots[index], l);
    return index;
  }

  do {
    label stored_label = slot_read(self, self->slots[index]);

    if (stored_label.len == l.len && strncmp(stored_label.s, l.s, l.len) == 0)
      return index;

    index = (index + 1) & self->mask;

    if (index == initial_index) {
      return LABELS_STORAGE_FULL;
    }
  } while(!slot_free(self->slots[index]));

  slot_write(self, &self->slots[index], l);

  return index;
}

static label labels_get(labels *self, uint32_t id, uint8_t reader_id) {
  if (id >= self->capacity || reader_id >= 7) {
    return INVALID_LABEL;
  }

  uint8_t mask = 1 << reader_id;
  uint8_t current_lock = __atomic_load_n(&self->lock, __ATOMIC_ACQUIRE);
  do {
    while(current_lock & LOCK_WRITE) {
      current_lock = __atomic_load_n(&self->lock, __ATOMIC_RELAXED);
    }

  } while(!__atomic_compare_exchange_n(&self->lock, &current_lock, current_lock | mask, true, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED));

  label l = slot_read(self, self->slots[id]);

  __atomic_fetch_and(&self->lock, ~mask, __ATOMIC_RELAXED);

  return l;
}

static void labels_teardown(labels self) {
  free(self.storage);
  free(self.slots);
}

// ========= TESTS ============

#ifdef BND_TESTS
#include "testing.h"

#define LABEL(s) (label) { s, strlen(s) }

static bool label_equal(const label l, char *s) {
  return strncmp(l.s, s, l.len) == 0;
}

static void storing_label_allows_to_retreive_it() {
  labels ls = labels_init(32, 32);
  uint32_t id = labels_store(&ls, LABEL("Hello"));
  label l = labels_get(&ls, id, 0);

  assert(label_equal(l, "Hello"));

  labels_teardown(ls);
}

static void requesting_invalid_id_gives_null_pointer() {
  labels ls = labels_init(32, 32);
  label l = labels_get(&ls, 200, 0);

  assert(!label_is_valid(l));

  labels_teardown(ls);
}

static void storing_same_label_multiple_times_gives_the_same_id() {
  labels ls = labels_init(32, 32);
  uint32_t id = labels_store(&ls, LABEL("Hello"));

  for(int i = 0; i < 10; ++i) {
    uint32_t another_id = labels_store(&ls, LABEL("Hello"));
    assert(id == another_id);
  }

  labels_teardown(ls);
}

static void different_labels_produce_different_ids() {
  labels ls = labels_init(1000, 32);
  uint32_t ids[] = {
    labels_store(&ls, LABEL("Hello")),
    labels_store(&ls, LABEL("foo")),
    labels_store(&ls, LABEL("bas")),
    labels_store(&ls, LABEL("42")),
    labels_store(&ls, LABEL("Starship Millenium Falcon"))
  };

  for(int i = 0; i < 5; ++i) {
    for(int j = 0; j < 5; ++j) {
      if (i != j)
        assert(ids[i] != ids[j]);
    }
  }

  labels_teardown(ls);
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
