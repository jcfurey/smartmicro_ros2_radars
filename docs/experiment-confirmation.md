# Experiment: confirmation from consistent motion

Branch `experiment/umrr96-confirmation`, based on shared checkpoint `3aee7a7`.
The worktree YAML enables `evidence_confirmation`; the Python default is false.
The original point rejection, clustering, association and standing support are
held fixed.

Each moving hit earns a bounded score from position and signed radial-speed
prediction errors, using existing spatial/noise scales. A miss subtracts one.
Six hits with at least 4.5 accumulated evidence can confirm early; the existing
8-of-10 rule remains a fallback. A scan earns at most one vote regardless of
point count. These scores are heuristic consistency measures, not calibrated
probabilities. History resets with a new track or a long data gap.

## Results

On the previously examined UMRR walk, person-proxy track presence changes from
591/618 to **593/618 walking scans** and from 63/108 to **64/108 swaying scans**.
Standing remains 88/145. Far-track presence stays 10/618 while walking and zero
during standing/swaying. No tracks appear in the 290-scanned interior absence
interval, or in the two quiet captures totaling 1,591 scans.

The gain is small: three more retained direct-proxy points are associated with
confirmed tracks across walking/swaying. It does not solve standing support.
The 20 seeded synthetic crossing runs show no difference from the baseline:
7,237/7,240 object opportunities matched, no ID changes or unmatched tracks.
This is a reasonable candidate for another capture, not a demonstrated general
accuracy improvement.

## Validation and use

124 processing pytest cases and package lint pass, including coherent motion,
inconsistent motion, misses, duplicates, data-gap reset and ROS parameter wiring.
The node integration check confirms that an early track never inserts old XYZ
into the current-detection cloud. Disabling the experiment exactly matches the
baseline tracker state digest across all 3,436 recorded scans.

Build/install lives at workspace `.colcon/umrr96-exp-confirmation/`. Its two
isolated ROS test domains are 181/182. No radar was powered up or commanded.
Use one processing overlay/publisher at a time in a future live comparison.
Recorded point gates and display decay remain unchanged.

The [evidence](experiment-confirmation.json) records counts, configurations,
source hashes and the full report paths. Shared comparison scripts are in the
workspace's `src/smartmicro_ros2_radars/tools/assessment/`; the comparison report
there documents the command line and the limitations of the room-range proxies.
