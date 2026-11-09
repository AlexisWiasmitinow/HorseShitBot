# TFT battery display integration

## Root cause

The running branch still used the original monolithic status-screen renderer.
The redesigned renderer existed only as uncommitted files in the Issue #20
worktree. It expected `sensor_msgs/BatteryState` on `/battery/status`, while:

1. normal startup did not pass `enable_battery:=true`; and
2. the committed battery node published `percentage = NaN`.

Consequently the redesigned screen could not display a truthful percentage
even if copied into the running branch.

## Integrated TFT files

- `drivers/ili9341_display.py`: corrected no-hardware geometry to the ILI9341
  native 320×240 landscape size. This is the only TFT hardware adapter and
  depends on `luma.lcd`, Pillow, SPI and platform GPIO.
- `drivers/status_screen_render.py`: pure Pillow renderer; no ROS, SPI, GPIO or
  Modbus access.
- `drivers/battery_readout.py`: validates and formats recent
  `BatteryState` values; does not estimate state of charge.
- `nodes/status_screen_node.py`: subscribes to battery and existing robot
  telemetry, rejects stale values, and sends rendered images to the display.
- `scripts/render_status_screen_previews.py`: uses the production renderer
  without opening hardware.

The node continues to display wheel, lift, brush, bin-door, gamepad, network,
temperature and recorder status. It subscribes to both normal recorder status
topics and the legacy standalone recorder topic. No web-dashboard files from
the Issue #20 worktree were copied.

Runtime dependencies were already declared by the package: ROS 2
`sensor_msgs`, Pillow and `luma.lcd`. DejaVu Sans improves visual fidelity but
the renderer has a Pillow fallback font.

The final renderer uses a white background, light-grey cards, dark text and
green/amber/red semantic states. No display-driver inversion or hardware
initialization change was required; the theme is entirely a Pillow rendering
choice.

## Battery data flow

1. `mks_bus_node` remains the sole owner of the shared RS485/Modbus serial
   device.
2. Once per second, `battery_modbus_node` calls
   `/modbus/read_holding_register` for N43IC04 device 33, channel 1.
3. `AdcRegisterMap` converts the raw count using the measured
   `voltage = (raw + 15.2) / 49.25` affine calibration.
4. Instantaneous voltage is compared with low and critical safety thresholds.
5. A rolling median is applied only to the display state-of-charge estimate.
6. The empirical 2026-09-25 discharge curve maps filtered voltage to 0–100%;
   the node publishes this as the required 0–1 `BatteryState.percentage`.
7. `status_screen_node` displays the published voltage and percentage
   verbatim. Missing, invalid, not-present, or more-than-five-second-old data
   is rendered as unavailable.

The percentage is voltage-derived and indicative. It is not coulomb counting
and varies with load, temperature, ageing and battery variation.

## Modbus safety

The integration creates no second Modbus client or serial owner.
`battery_modbus_node` uses only the service exposed by `mks_bus_node`; the bus
owner serializes battery requests with motor traffic and can report busy.
Launch rejects `enable_battery:=true` when the shared MKS bus node is disabled.
Normal startup enables the monitor, while `--no-battery` remains available.

Safety thresholds continue to use instantaneous voltage, so the 30-sample
display smoothing window cannot delay a low-voltage indication. No shutdown,
motor, calibration, or actuator behavior was enabled or changed.

## Calibration provenance and limitation

The raw-to-voltage fit is supported by five recorded points from 20.0 V to
28.0 V. The voltage-to-SoC curve comes from one 21.4-hour discharge recording
to BMS cutoff. This is enough to publish a bounded, documented estimate, but
not enough to claim a generally accurate battery fuel gauge. A current sensor
or BMS/coulomb counter would be required for stronger accuracy.

## Physical validation

The integrated stack was exercised on `horseshitbot01`. The physical TFT
displayed live battery information and the observed `/battery/status` sample
was approximately 27.98 V, 1.0 (100%), health GOOD, and present. These are
hardware-test observations, not fallback or hardcoded values. The screen
continues to show an unavailable state if telemetry is missing, invalid or
stale.

## Validation

Offline tests cover:

- 0–1 fraction to 0–100 percentage formatting;
- invalid, missing and stale telemetry;
- voltage formatting and health escalation;
- empirical voltage calibration and SoC interpolation;
- median filtering without delaying instantaneous safety evaluation;
- wheel, actuator, gamepad and recorder snapshot retention;
- 320×240 rendering for normal, low, critical and unavailable states;
- controls and E-stop rendering without hardware access.

Generate previews with:

```bash
python3 scripts/render_status_screen_previews.py \
  --out previews/status_screen
```

## Proposed deployment

Deployment must wait until the current robot session can be intentionally
stopped.

1. Review the branch, preview and calibration provenance.
2. Stop the existing robot stack using the normal service/operator procedure.
3. Build this branch in the robot workspace.
4. Run `./scripts/start.sh --dry-run` if available in the deployment branch,
   and confirm exactly one `status_screen_node`, one `mks_bus_node`, and one
   `battery_modbus_node` will start.
5. Start normally. Do not start a standalone TFT or battery process.
6. Verify `/battery/status` has finite voltage and percentage in `[0, 1]`.
7. Compare displayed voltage against a trusted meter before relying on it.
8. Confirm motor communication remains healthy while the 1 Hz ADC polling is
   active.
9. Use `--no-battery` and stop deployment if bus errors increase or voltage
   does not match the independent meter.

## Remaining risks

- SoC is based on one voltage-only discharge experiment.
- Battery voltage under motor load can move percentage substantially despite
  median filtering.
- The configured low/critical voltage thresholds still require a separate
  safety review; this integration does not change them.
- The physical panel orientation must be confirmed during the approved
  deployment window.
