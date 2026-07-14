# ST3215 Calibration Dump

Read-only debugging tool: prints each motor's calibrated arc (`range_min`,
`range_max`, the values station-viewer uses to render the URDF) alongside
the physical servo's EEPROM position offset (register `0x1F`), plus the
raw present position and a computed midpoint.

Never writes to a motor — safe to run at any time, including while the
station is already running (it only subscribes to the existing
`st3215/inference` stream, it doesn't open the serial port itself).

## Why

Calibration only persists `range_min`/`range_max` on the host (in
`normfs`); the per-motor EEPROM `offset` used to zero the encoder is
written once during calibration and otherwise lives only inside the
servo itself. If the rendered arm doesn't visually match the real arm's
pose, this script gives you both numbers side by side to compare against
the URDF's own joint limits.

## Running

```bash
# Single bus (auto-detects), one snapshot
python main.py --server localhost

# Keep printing on every frame
python main.py --server localhost --watch

# Multiple buses on one station
python main.py --server localhost --bus <bus-serial-number>
```

## Output

```
Bus 0123456789  (8 motors)
 id  position   offset  range_min  range_max  midpoint  frozen
  1      2048        0          0       4095      2047   True
  2      1500      -50        200       3800      2000   True
  ...
```

- **position** — raw present position (0-4095), sign bit already resolved.
- **offset** — signed EEPROM position offset (register `0x1F`), applied by
  the servo itself before it reports `position`.
- **range_min / range_max** — the calibrated arc persisted on the host;
  this is what feeds the station-viewer's URDF rendering.
- **midpoint** — `range_min`/`range_max` centered (same formula the
  calibrator uses to zero an arc), *not* read from the servo — for
  comparison against `offset` only.
- **frozen** — whether calibration has been frozen for this motor.
