#pragma once

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace deltacore {
namespace command_parser {

struct LinearMoveArgs {
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_f = false;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float f = 0.0f;
};

inline bool commandStarts(const char *line, const char *cmd) {
  const size_t n = strlen(cmd);
  return strncmp(line, cmd, n) == 0 &&
         (line[n] == '\0' || line[n] == ' ' || line[n] == '\t');
}

inline bool commandExact(const char *line, const char *cmd) {
  if (!commandStarts(line, cmd)) return false;
  const char *p = line + strlen(cmd);
  while (*p == ' ' || *p == '\t') ++p;
  return *p == '\0';
}

inline void skipSpace(const char *&p) {
  while (*p == ' ' || *p == '\t') ++p;
}

inline bool parseFiniteNumber(const char *&p, float &value) {
  char *end = nullptr;
  const double parsed = strtod(p, &end);
  if (end == p || !isfinite(parsed)) return false;
  value = float(parsed);
  if (!isfinite(value)) return false;
  p = end;
  return true;
}

inline bool parseOptionalSingleFloatParam(const char *line, const char *cmd,
                                          const char key, bool &has_value,
                                          float &value) {
  if (!commandStarts(line, cmd)) return false;
  const char *p = line + strlen(cmd);
  skipSpace(p);
  if (*p == '\0') {
    has_value = false;
    return true;
  }
  if (*p != key) return false;
  ++p;
  if (!parseFiniteNumber(p, value)) return false;
  skipSpace(p);
  if (*p != '\0') return false;
  has_value = true;
  return true;
}

inline bool parseLinearMove(const char *line, LinearMoveArgs &out) {
  if (!(commandStarts(line, "G0") || commandStarts(line, "G1"))) return false;
  const char *p = line + 2;
  skipSpace(p);

  bool seen_x = false, seen_y = false, seen_z = false, seen_f = false;
  while (*p) {
    const char key = *p++;
    bool *seen = nullptr;
    float *dest = nullptr;
    switch (key) {
      case 'X': seen = &seen_x; dest = &out.x; break;
      case 'Y': seen = &seen_y; dest = &out.y; break;
      case 'Z': seen = &seen_z; dest = &out.z; break;
      case 'F': seen = &seen_f; dest = &out.f; break;
      default: return false;
    }
    if (*seen) return false;
    *seen = true;
    if (!parseFiniteNumber(p, *dest)) return false;

    // Compact G-code such as X1Y2 is valid, but any other fused token (M105,
    // punctuation, garbage, etc.) is rejected instead of being silently eaten.
    if (*p && *p != ' ' && *p != '\t' &&
        *p != 'X' && *p != 'Y' && *p != 'Z' && *p != 'F') return false;
    skipSpace(p);
  }

  out.has_x = seen_x;
  out.has_y = seen_y;
  out.has_z = seen_z;
  out.has_f = seen_f;
  return true;
}

}  // namespace command_parser
}  // namespace deltacore
