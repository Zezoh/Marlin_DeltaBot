#pragma once

#include <stdint.h>

namespace deltacore {
uint8_t bootResetCause();
uint32_t beginBootSession();
uint32_t bootSessionId();
}
