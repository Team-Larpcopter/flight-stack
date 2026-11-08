# PX4 forwarding audit — 2026-11-08

Scope: mission runtime for SUPER, EGO, route follower/Hari, fixed course,
triangle and takeoff/hold. These launch the executive with `skybeta_compat=true`,
vision age checking disabled, VIO-health gating disabled and executive geofence
disabled. This is a forwarding-policy audit, not a claim that our entire stack
is identical to SkyBeta or that PX4 will reject every unsuitable input.

## Removed from the active forwarding path

- Body odometry: position-jump threshold, permanent exception latch, covariance
  health publication gate, sample timestamp high-water gate and numeric-value rejection.
- Shared map registration: post-initialization numeric and backward-time filtering.
- Vision bridge: numeric quaternion/vector/covariance rejection; clock rollback
  cannot strand the mission stream behind an old timestamp.
- Planner bridge: sticky epoch/alignment faults, sequence restart rejection,
  clock rollback clearing and producer-age cutoff. Configured mission goal is
  republished after a localization epoch change.
- Polynomial forwarding: continuity, terminal velocity/acceleration, coefficient
  finiteness and arbitrary piece count/degree/duration limits.
- Executive: numeric setpoint filtering, ID-based command rejection and moving-final
  sample rejection. A moving final sample is forwarded but is not evidence of arrival.
- Mission command stream: receipt deadline and terminal OFFBOARD/LAND rejection or
  confirmation-timeout faults while the authorized request remains active.
- Other-vehicle status packets are ignored instead of permanently failing this mission.

NaN/Inf values reach the PX4 publishers; values beyond float32 range encode as Inf.
Measurement timestamps are preserved. No reset is invented from a position correction.

## Remaining behavior (explicitly not hidden)

- Correct declared frames and configured rigid transforms are needed to perform
  coordinate conversion. Initial map/ground registration needs a usable initial
  pose; malformed startup input must not poison the reference for later input.
- Wire decoding still needs coefficients matching their declared dimensions,
  positive finite durations, and matching yaw/position timelines.
- Explicit goal cancellation, mission phase transitions, pilot authorization,
  pilot mode exit, disarming and completed landing retain their mission semantics.
  No automatic arming or Offboard reacquisition after pilot takeover is added.
- Missing odometry produces no invented samples. Consecutive duplicate stamps are
  coalesced. The vision timer consumes the latest received sample without a backlog.
- Planner/map collision logic and map-update readiness are not removed: they
  generate the route and protect map initialization, rather than adjudicating PX4 inputs.
- Optional non-mission strict modes (`skybeta_compat=false`, positive vision age limit,
  explicitly enabled health gate/geofence) still exist; the listed mission launches
  do not enable them.

Regression coverage exercises large pose corrections, high covariance, NaN samples,
clock rollback and recovery, active alignment/epoch changes, restarted sequences,
and actual PX4 setpoint serialization of NaN/Inf/overflow.
