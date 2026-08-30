#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "CommandParser.h"

using namespace deltacore::command_parser;

static void expect_move_ok(const char *line) {
  LinearMoveArgs a;
  assert(parseLinearMove(line, a));
}

static void expect_move_bad(const char *line) {
  LinearMoveArgs a;
  assert(!parseLinearMove(line, a));
}

int main() {
  expect_move_ok("G1 X40 Y0 Z120 F4800");
  expect_move_ok("G0 X1Y2Z3F6000");
  expect_move_ok("G1");
  expect_move_ok("G1 F7200");
  expect_move_ok("G1 X1.25 Y-2.5 Z1E2 F7200");

  // Real hardware corruption signatures: these must never be accepted as G0/G1.
  expect_move_bad("G1 X-6 Y-2M105");
  expect_move_bad("G-1 X-6 Y-2M105");
  expect_move_bad("G1 X0 Y0M400");
  expect_move_bad("G1 --17");
  expect_move_bad("G1 X1 X2");
  expect_move_bad("G1 XNAN");
  expect_move_bad("G1 XINF");
  expect_move_bad("G1 X1#");

  bool has = false;
  float value = 0.0f;
  assert(parseOptionalSingleFloatParam("M111", "M111", 'S', has, value) && !has);
  assert(parseOptionalSingleFloatParam("M111 S2", "M111", 'S', has, value) && has && value == 2.0f);
  assert(!parseOptionalSingleFloatParam("M111 S2M105", "M111", 'S', has, value));
  assert(!parseOptionalSingleFloatParam("M204 S1600 M105", "M204", 'S', has, value));
  assert(!parseOptionalSingleFloatParam("M970 SNAN", "M970", 'S', has, value));

  assert(commandExact("M105", "M105"));
  assert(commandExact("M105   ", "M105"));
  assert(!commandExact("M105 G1 X0", "M105"));
  assert(!commandExact("M115 M105", "M115"));

  puts("PASS strict command parser: fused/interleaved commands rejected fail-closed");
  return 0;
}
