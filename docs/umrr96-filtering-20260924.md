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

## Limits

- One room, one person, stationary radar. Two movers at the same speed and
  bearing, ≥ 1.5 m apart in range, lose the farther one.
- Ego-velocity settings are validated only with the sensor at rest; motion
  may raise Doppler noise and needs a moving-platform check before fusion.
- Doppler sign convention remains unverified against controlled motion.

## Next steps

1. **Live check** of `moving_targets` vs `moving_ghosts` in RViz while walking.
2. **Moving-object tracker**: cluster `moving_targets`, constant-velocity
   tracks with confirmation (M-of-N) — expected to remove most remaining
   ghosts (they rarely form consistent tracks) and to hold a person through
   tangential motion and standing still (zero Doppler) using nearby static
   detections.
3. **Nav2 obstacle persistence**: k-of-n polar/voxel persistence on the
   sensor-frame cloud to drop the 3% flicker; decide how to treat persistent
   ghosts (e.g. the ≈5 m / −40° return) — candidates: occlusion reasoning
   against the static map, or lidar cross-check when available.
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
