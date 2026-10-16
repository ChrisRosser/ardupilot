# Payload tilt release

Releases the payload automatically when the vehicle has lost attitude control:
tilted past `PREL_ANGLE` with at least one motor commanded at or above
`PREL_PWM`, continuously for `PREL_TIME` seconds, while armed and flying.

On this airframe `ANGLE_MAX` is 30°, so 45° of actual tilt only happens when the
attitude controller cannot hold the vehicle, and a motor near full output means
it has no authority left to recover. Dropping the payload removes the load that
is overwhelming the motors.

## Behaviour

- **Trigger:** tilt from level, combining roll and pitch, at or above
  `PREL_ANGLE`, and the highest motor output at or above `PREL_PWM`, both
  holding continuously for `PREL_TIME`. Either condition clearing resets the
  timer.
- **Only in flight:** requires the vehicle to be armed and flying
  (`vehicle:get_likely_flying()`).
- **Latched:** once released, the payload stays released until the vehicle is
  disarmed. Disarming resets the script for the next flight.
- **Warn-only by default:** with `PREL_ACTION` 0 the script sends a GCS warning
  and logs the trigger but releases nothing. Fly this way first and check the
  `PREL` log before selecting a release action.
- **Messages:** a critical GCS message when both conditions first hold, and an
  emergency message when the payload is released.
- **Log:** `PREL` at 10 Hz while armed, and every loop while the conditions hold:
  tilt (deg), highest motor PWM, which motor, flying, conditions met, released.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `PREL_ENABLE` | 0 | 1 enables the script |
| `PREL_ANGLE` | 45 | Tilt from level, degrees |
| `PREL_PWM` | 1950 | Motor output PWM; any one motor reaching it counts |
| `PREL_TIME` | 0.5 | Seconds both conditions must hold |
| `PREL_ACTION` | 0 | 0 warn and log only, 1 relay, 2 servo PWM |
| `PREL_RELAY` | 1 | Relay number for action 1 (1 is `RELAY1`) |
| `PREL_RLY_ON` | 1 | 1 if relay on releases, 0 if relay off releases |
| `PREL_SRV_CH` | 0 | Servo output channel for action 2 (1 is `SERVO1`) |
| `PREL_SRV_PWM` | 1900 | PWM that releases the payload for action 2 |

The parameter table uses key 111. If another script on the vehicle already uses
111, change `PARAM_TABLE_KEY` in the script.

## Setup

1. Copy `payload_tilt_release.lua` to `APM/scripts` on the SD card. `SCR_ENABLE`
   must be 1. Reboot, then reboot again after the parameters first appear.
2. Set `PREL_ENABLE` 1 and leave `PREL_ACTION` 0.
3. Configure the release output: either `RELAYn_FUNCTION` 1 and `RELAYn_PIN`
   for a relay-driven hook, or a servo output for a servo-driven hook.
4. On the ground with the vehicle disarmed and the hook loaded safely, test the
   release output itself with the GCS relay or servo test, not through this
   script.
5. Fly with `PREL_ACTION` 0 and confirm the `PREL` log shows sensible tilt and
   PWM and that the conditions never hold in normal flight.
6. Then set `PREL_ACTION` and its output parameters.

## Notes

- Runs at 50 Hz, so the release fires within about `PREL_TIME` + 20 ms of both
  conditions holding.
- `FS_CRASH_CHECK` disarms after 2 s of large attitude error. Keep `PREL_TIME`
  well below that so the payload is released while the motors are still
  running.
- A servo release is re-asserted every loop while armed after release; once
  disarmed the servo output returns to normal control.
