# DeltaCore v0.2 - first hardware test

Do this in order. The first command after flashing is **M119**, not G28.

## 1. Flash

With PlatformIO:

```powershell
cd DeltaCore
pio run -e megaatmega2560 -t upload --upload-port COMx
```

Or on Windows:

```powershell
.\flash_windows.ps1 -Port COMx
```

Open a serial terminal at **250000 baud**.

Expected boot text includes:

```text
DeltaCore 0.2 - Mega2560 / MKS MINI v2.0
SAFE BOOT: motors disabled, heaters/extruder untouched, G28 required before G1
ok READY
```

## 2. Verify endstops BEFORE homing

With all three carriages away from their top MAX switches:

```text
M119
```

Expected:

```text
ENDSTOPS A:open B:open C:open
```

Manually press only A's top switch and send `M119` again. A must show `TRIGGERED`, B/C `open`.

Repeat for B, then C.

**Do not send G28 if any switch reports the wrong state or wrong tower.**

## 3. First G28

Keep one hand ready to cut power. Send:

```text
G28
```

Expected behavior:

1. A/B/C move upward toward their MAX switches.
2. Each tower stops independently when its own switch triggers.
3. All three back off approximately 5 mm.
4. All three approach slowly.
5. Each stops independently on its MAX switch.
6. Serial reports:

```text
ok HOME_DONE X0.000 Y0.000 Z225.000
```

If any tower moves down instead of up, immediately cut power / send `M112` if communication is still responsive. Do not continue until direction mapping is corrected.

## 4. First controlled Z motion

After successful homing:

```text
G1 Z210 F600
```

This is a slow 15 mm downward Cartesian move at 10 mm/s.

Then:

```text
G1 Z190 F1200
G1 Z210 F1200
```

Expected: all three towers move smoothly and synchronously, with no abrupt stop between Delta sub-segments.

## 5. Center travel

```text
G1 Z160 F1800
G1 Z120 F2400
G1 Z160 F2400
```

## 6. Small XY test at safe Z

At Z160:

```text
G1 X20 Y0 Z160 F1800
G1 X20 Y20 Z160 F1800
G1 X0 Y20 Z160 F1800
G1 X0 Y0 Z160 F1800
```

Then repeat in the opposite direction if motion is clean.

## 7. Report state

```text
STATUS
M114
M119
```

Record the full serial output plus any observed vibration, clicking, missed steps, wrong directions, or pauses.

## Emergency commands

```text
M112
```

Stops motion, clears the queue, disables all three drivers, and invalidates the known position.

After `M112`:

```text
M999
G28
```

A new home is mandatory before any Cartesian move.
