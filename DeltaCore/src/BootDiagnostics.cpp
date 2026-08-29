#include "BootDiagnostics.h"

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>

static uint8_t g_reset_cause __attribute__((section(".noinit")));
static uint32_t g_boot_session = 0;

namespace {

constexpr uint16_t BOOT_SLOT_MAGIC = 0xD34A;
constexpr uint8_t BOOT_SLOT_COUNT = 32;

struct BootSlot {
  uint16_t magic;
  uint16_t sequence;
  uint16_t sequence_inv;
};

BootSlot EEMEM ee_boot_slots[BOOT_SLOT_COUNT];

bool validSlot(const BootSlot &slot) {
  return slot.magic == BOOT_SLOT_MAGIC &&
         uint16_t(slot.sequence ^ slot.sequence_inv) == 0xFFFFU &&
         slot.sequence != 0U;
}

bool sequenceNewer(const uint16_t candidate, const uint16_t current) {
  return int16_t(candidate - current) > 0;
}

void readSlot(const uint8_t index, BootSlot &slot) {
  eeprom_read_block(&slot, &ee_boot_slots[index], sizeof(BootSlot));
}

void writeSlot(const uint8_t index, const uint16_t sequence) {
  BootSlot *dst = &ee_boot_slots[index];

  // Invalidate first and commit the magic last. A reset/power loss during an
  // EEPROM write can therefore leave an old valid slot, but never a partially
  // written new slot that looks valid.
  eeprom_update_word(&dst->magic, 0U);
  eeprom_update_word(&dst->sequence, sequence);
  eeprom_update_word(&dst->sequence_inv, uint16_t(~sequence));
  eeprom_update_word(&dst->magic, BOOT_SLOT_MAGIC);
}

} // namespace

void captureResetCauseEarly() __attribute__((naked, section(".init3"), used));
void captureResetCauseEarly() {
  g_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

namespace deltacore {

uint8_t bootResetCause() { return g_reset_cause; }

uint32_t beginBootSession() {
  bool found = false;
  uint8_t newest_index = 0;
  uint16_t newest_sequence = 0;

  for (uint8_t i = 0; i < BOOT_SLOT_COUNT; ++i) {
    BootSlot slot;
    readSlot(i, slot);
    if (!validSlot(slot)) continue;
    if (!found || sequenceNewer(slot.sequence, newest_sequence)) {
      found = true;
      newest_index = i;
      newest_sequence = slot.sequence;
    }
  }

  uint16_t next_sequence = found ? uint16_t(newest_sequence + 1U) : 1U;
  if (next_sequence == 0U) next_sequence = 1U;
  const uint8_t next_index = found ? uint8_t((newest_index + 1U) % BOOT_SLOT_COUNT) : 0U;

  writeSlot(next_index, next_sequence);
  g_boot_session = next_sequence;
  return g_boot_session;
}

uint32_t bootSessionId() { return g_boot_session; }

} // namespace deltacore
