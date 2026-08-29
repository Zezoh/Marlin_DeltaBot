#include "BootDiagnostics.h"

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>

static uint8_t g_reset_cause __attribute__((section(".noinit")));
static uint32_t g_boot_session = 0;
static uint32_t EEMEM ee_boot_counter;

void captureResetCauseEarly() __attribute__((naked, section(".init3"), used));
void captureResetCauseEarly() {
  g_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

namespace deltacore {

uint8_t bootResetCause() { return g_reset_cause; }

uint32_t beginBootSession() {
  uint32_t value = eeprom_read_dword(&ee_boot_counter);
  if (value == 0xFFFFFFFFUL || value == 0UL) value = 1UL;
  else ++value;
  eeprom_update_dword(&ee_boot_counter, value);
  g_boot_session = value;
  return g_boot_session;
}

uint32_t bootSessionId() { return g_boot_session; }

} // namespace deltacore
