#include "BootDiagnostics.h"

#include <avr/io.h>
#include <avr/wdt.h>

static uint8_t g_reset_cause __attribute__((section(".noinit")));

void captureResetCauseEarly() __attribute__((naked, section(".init3"), used));
void captureResetCauseEarly() {
  g_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

namespace deltacore {
uint8_t bootResetCause() { return g_reset_cause; }
}
