# UMRR-96 signal quality and filtering (2026-09-24)

Sensor 230739 (`0x38553`), firmware 5.2.2, Ethernet, CAN target output off
(read back; 18.18 Hz, 55 ms cycle). Radar **stationary** on a bench; indoor
room with about 2.5–3 m of clear floor in front of it. No sensor setting was
written during this work.

Goals, in the order agreed: Nav2 obstacle evidence, moving-object detection,
Doppler ego-velocity aiding for LIO, and a cleaner operator view.

## Captures

Recorded in the workspace's ignored `results/umrr96-filtering-20260924/`
(driver cloud, target header, raw quality, timing, diagnostics):

| Bag | Duration | Scans | Content |
|---|---:|---:|---|
| `static` | 61.5 s | 1,117 | Empty scene, nothing moving |
| `walk` | 101.6 s | 1,845 | One person: walking toward/away (≈0.7–2.8 m), crossing, standing, swaying, leaving |
| `walk_proc` | 102.0 s | 1,846 | `walk` replayed through the processing node after the changes below |

Analysis scripts (top-level workspace `scripts/`): `umrr96_static_analysis.py`,
`umrr96_walk_analysis.py`, `umrr96_ghost_eval.py`.

## Findings

### Static scene

| Measure | Result |
|---|---|
| Detections per scan | 26.4 mean (19–33) |
| \|radial speed\| | p99 5.8 mm/s; none above 0.1 m/s |
| Ego-velocity fit (truth 0) | valid 1,117/1,117; std 0.4 / 0.6 / 2.7 mm/s (x/y/z) |
| Polar cells (0.5 m × 3°) | 20 persistent (≥50% of scans) hold 72% of detections; 34 intermittent 25%; 88 flicker (<5%) only 3% |
| Jitter of persistent returns | range std p50 3 mm (p90 13 cm); azimuth p50 0.3° (p90 0.8°) |
| SNR persistent vs flicker | median 38 vs 30 dB — overlapping; an SNR cut cannot separate them |
| Nearest return | 0.98 m, −49.5°, SNR 54 dB, always present (likely a nearby fixture) |

### Walk-through

- Motion is detected in every scan while walking (5–10 moving detections at
  0.35–0.6 m/s). Swaying produced 0.08–0.12 m/s, below the old 0.2 m/s gate.
- **Multipath ghosts dominate moving detections**: ~2/3 lie beyond 3.2 m (up
  to 14 m), where the person could not be. The small room makes this a labelled
  set: movers < 3.0 m are real, > 3.4 m are ghosts.
- A standing person produced only ~0.7 direct returns per scan (2.6 m) but
  ~10 new zero-Doppler detections per scan elsewhere (≈7.7 m).
- While the person was out of view (by their account), a stationary return
  appeared at ≈5 m, 38–42° right, absent from the static capture: most likely
  the person seen by reflection — a *persistent* ghost that a persistence
  filter alone would accept.
- **RCS is unusable for filtering**: linear values with p50 ≈ 0 for movers and
  ghosts alike (static p50 0.68, p99 2,064). Worth checking against the vendor
  interface definition.
- SNR appeared to separate ghosts (raw SNR < 25 dB rejects 73%, keeps 100%),
  but after range normalisation (SNR + 40·log10 r) the distributions coincide:
  the effect was distance, not a ghost signature.

### Ego-velocity fit with a walking person (truth 0)

| Settings | Movers absorbed as static | \|v\| p99 | \|v\| max |
|---|---:|---:|---:|
| Old: residual 0.20, noise floor 0.05, std floor 0.10 | 823 / 4,965 | 0.148 m/s | 1.79 m/s |
| New: residual 0.05, noise floor 0.02, std floor 0.05 | 11 / 4,911 | 0.027 m/s | 0.20 m/s |

### Ghost-rejection rules (single scan)

Scored on `walk` (real kept = movers < 3 m retained; ghosts removed = movers
> 3.4 m rejected):

| Rule | Ghosts removed | Real kept |
|---|---:|---:|
| Nearer mover, any speed (gap 0.5 m) | 96% | 83% |
| Nearer mover with same \|speed\| (gap 1.0, ±0.25 m/s) | 86% | 92% |
| … plus behind a same-scan static return (±4°) | 89% | 91% |
| **Gap 1.5 m, ±0.25 m/s, static ±4° (chosen)** | **85%** | **97%** |
| Behind persistent static map only | 33–38% | 97% |

## Implemented (`smartmicro_processing`, commit `a0358c9`)

- `ghosts.py`: `ghost_mask()` with `GhostConfig(range_gap=1.5,
  speed_tolerance=0.25, wall_azimuth_deg=4.0)`, applied to Doppler outliers
  using ego-compensated speeds (so it remains valid on a moving platform).
- New outputs `~/moving_targets` and `~/moving_ghosts`; `doppler_outliers`
  unchanged. Diagnostics add `moving` and `ghosts` counts.
- `config/umrr96_processing.yaml`: residual 0.05, noise floor 0.02, velocity
  std floor 0.05 (static noise was ≈3 mm/s; kept conservative until data with
  the sensor moving), ghost parameters.
- Replay of `walk`: 96.9% real kept, 84.7% ghosts removed; 22.8% of
  `moving_targets` are still ghosts. 123 processing tests pass.

## Moving-object tracker (`smartmicro_processing/tracker.py`)

Clusters `moving_targets` (0.6 m), runs a constant-velocity EKF per track with
position and radial-speed updates, confirms tracks on 8 hits in 10 scans and
coasts 1 s. A confirmed track may hold for 5 s on *novel* zero-Doppler
detections (standing still, tangential motion); a 30 s background occupancy
model keeps walls from holding tracks. A track farther than another confirmed
track with the same or double radial speed (first/second-order bounce) is a
ghost and is not published. The ghost rules gained the same doubled-speed test
per detection: 96.3% real kept, 90.4% ghosts removed.

Outputs: `~/tracked_objects` (PointCloud2: x, y, z, vx, vy, speed, track_id,
age) and `~/track_markers` (MarkerArray, only with subscribers).

Scored on `walk` (`scripts/umrr96_tracker_eval.py`; enter/leave windows are
excluded from ghost scoring because the person may really pass beyond 3.4 m):

| Phase | Person tracked | Track ids | Ghost track present |
|---|---:|---:|---:|
| Walking (6–40 s) | 95.6% | 4 | 1.8% |
| Standing still | 60.7% | 2 | 0% |
| Swaying | 58.3% | 1 | 0% |
| Absent | — | — | 0% |
| All labelled scans | | | 0.7% (1 id) |

Progression: without the background model and continuous ghost test, ghost
tracks appeared in 17–52% of scans and the walk split into 20 ids.

## Short-path ghosts and `tracked_targets`

A 91.6 s capture (`scoot`: one person scooting back and forth on a chair
within ~3 m) showed ghosting that the single-scan rules miss: 30% of scans with
movers contained more than one moving cluster. The extra clusters were
typically 0.4–1.4 m (median 0.9 m) beyond the nearest one — inside the 1.5 m
`range_gap` — at a bearing ~28° away, with 0.87× the speed, the same Doppler
sign (89%), 12 dB weaker SNR and a single point: short bounces off nearby
surfaces.

A single-scan "weaker farther copy" rule halved multi-cluster scans (→ 12%)
but removed 11–14% of the person's real returns on `walk` (limbs share the
signature) and only about a third of the remaining labelled ghosts, so it was
not adopted. Temporal consistency works better: `~/tracked_targets` publishes
only `moving_targets` within `track_radius` (0.8 m) of a confirmed track.

| | `moving_targets` | `tracked_targets` |
|---|---:|---:|
| `scoot`: scans with >1 moving cluster (node replay) | 30.4% | 6.6% |
| `walk`: labelled ghosts left after the single-scan rules | 100% | 18% |
| `walk`: real returns kept (relative) | 100% | 85% |
| `walk`: person shown while walking | — | 85% of scans |

Cost: a new object appears only after confirmation (~0.4 s), and some limb
returns away from the track centre are dropped. `moving_targets` remains the
low-latency, unconfirmed stream; RViz shows `tracked_targets` in red.

## Nav2 obstacle evidence (`smartmicro_processing/obstacles.py`)

`~/obstacles` (sensor frame, scan stamp, z = `obstacle_height`) contains:
static Doppler inliers whose polar cell (0.5 m × 3°, no neighbourhood) was hit
in ≥ 3 of the last 5 scans; non-ghost movers within 0.8 m of a confirmed track;
each confirmed track position; and any non-ghost return within 1 m. While a
track exists, *novel* static returns (outside the 30 s background) more than
1.5 m beyond the nearest track are dropped as its multipath. If the Doppler
fit fails, all quality targets pass through persistence alone (conservative).

Scored with `scripts/umrr96_obstacle_eval.py` ("far" = novel points beyond
3.4 m, i.e. ghosts in this room):

| | Raw | Filtered |
|---|---:|---:|
| Static: flicker detections kept | 100% | 22% |
| Static: persistent structure kept | 100% | 97.4% |
| Walk: person marked while walking | 83% | 85% |
| Walk: person marked while standing | 57% | 44% |
| Far points/scan, walking | 10.8 | 0.06 |
| Far points/scan, standing | 9.5 | 1.5 |
| Far points/scan, person out of view | 1.3 | 1.0 |

A 3×3 neighbourhood let flicker borrow persistence from adjacent structure
(58% kept); 2-of-4 persistence doubled the flicker kept for +10% standing
coverage. The out-of-view reflection (≈5 m, −40°) has no track to attribute
it to and remains.

Nav2: an ObstacleLayer keeps marks until a clearing source raytraces them, so
pair the radar source with a clearing lidar source (radar ghosts in space the
lidar sees as free are then cleared), or use STVL with voxel decay for a
radar-only costmap. See `config/nav2_obstacle_layer.example.yaml`.

## Background warm-up and moving-sensor guard (2026-09-25)

The obstacle metrics above never measured structure while a track exists (the
static capture has no tracks). Scored on `walk` (`scripts/umrr96_obstacle_eval.py`,
new "structure beyond a track" column: returns beyond 3.4 m in cells present in
≥ 50% of `static` scans, while a confirmed track exists):

| | Structure beyond a track kept | Far ghost points/scan, walking |
|---|---:|---:|
| Before | 38.8% | 0.07 |
| Background warm-up 3 s | 72.4% | 0.07 |
| … plus `background_threshold` 0.5 → 0.3 (adopted) | 80.6% | 0.14 |
| Shadow rule off (`shadow_gap` ≤ 0), for reference | 88.1% | 1.97 |

Cause: the background was an exponential average from zero, so a wall hit in
every scan became background only after 0.7 time constants (21 s at 30 s); until
then the shadow rule dropped *all* static returns beyond the person. Any single
rejected scan (stale or non-monotonic stamp, wrong frame) also rebuilt the
tracker, restarting that 21 s. Now:

- The occupancy is bias-corrected (normalised by the accumulated weight): a cell
  hit in every scan is background after `background_warmup` (3 s). Until then
  the shadow rule is off.
- Rejected scans clear the outputs but keep tracker and background; only a
  backwards clock (bag loop) resets them. The tracker drops tracks after a data
  gap longer than `max_coast`, and persistence restarts after a gap longer than
  `stale_timeout`.
- The background assumes a scene-fixed sensor. When the fitted sensor speed
  exceeds `sensor_moving_speed` (0.05 m/s) for 3 consecutive scans, the
  background is reset (shadow rule off) until the sensor has been still for
  the warm-up again. The static captures never exceeded 0.011 m/s; the walk
  exceeded 0.05 m/s in 5 isolated scans. Pure rotation about the sensor's own
  axis produces no Doppler and is not detected.
- A failed Doppler fit now coasts the tracks (a recorded miss) and still
  publishes `tracked_objects`; track positions stay in `obstacles`.

Ghost-removal, tracking and far-ghost numbers are otherwise unchanged.

Requiring the same Doppler sign for the same-speed ghost rule (so that two
people walking in opposite directions are not merged) was evaluated and not
adopted: ghosts removed fell from 93.7% to 83.5% and ghost-track scans rose
from 0.7% to 6.3%: many `walk` ghosts carry the opposite sign.

## Limits

- One room, one person, stationary radar; parameters are tuned on the same
  recording they are scored on (no held-out data yet). The same-speed ghost
  rule ignores bearing and Doppler sign: of two real movers with similar
  |speed| (or one about twice the other) more than 1.5 m apart in range, at
  any bearing, the farther one is rejected, and a slow near mover (e.g.
  swaying at 0.1 m/s) suppresses farther movers up to about 0.7 m/s. The
  tracker's ghost rule behaves the same way. A second real person is only
  marked in `obstacles` within `safety_range` or through static persistence.
  A two-person capture is needed before relying on moving-object output.
- Ego-velocity settings are validated only with the sensor at rest; motion
  may raise Doppler noise and needs a moving-platform check before fusion.
- Doppler sign convention remains unverified against controlled motion.

## Next steps

1. **Live check** of `moving_targets` vs `moving_ghosts` in RViz while walking.
2. ~~Moving-object tracker~~ — done (above). Remaining: standing coverage
   (61%) is limited by ~0.7 direct returns per scan; the background model
   assumes a fixed sensor — on a moving platform it must run in a fixed frame
   (odom) with TF, like the accumulation node.
3. ~~Nav2 obstacle persistence~~ — done (above). Remaining: untracked
   persistent ghosts (the ≈5 m / −40° return) need lidar clearing or a map;
   standing coverage (44%); a live Nav2 costmap test; on a moving platform the
   polar persistence window (5 scans ≈ 0.28 s) must stay below one cell of
   motion, or the filter must move to a fixed frame.
4. **Moving-sensor validation** of the ego-velocity covariance (NEES against
   lidar/odometry) and the Doppler sign, then feed `experimental_velocity` to
   RESPLE/deliriom trials.
5. **RCS field**: confirm units/validity with the vendor interface definition.
6. **Operator view**: `smartmicro_processing/rviz/umrr96_moving.rviz` shows
   static inliers (white), moving targets (red, 0.5 s trail) and rejected
   ghosts (blue). Start it with the driver running:

   ```bash
   ros2 launch smartmicro_processing umrr96_processing.launch.py
   rviz2 -d $(ros2 pkg prefix smartmicro_processing)/share/smartmicro_processing/rviz/umrr96_moving.rviz
   ```
