# Experiment: joint association

Branch `experiment/umrr96-association`, based on shared checkpoint `3aee7a7`.
The worktree enables `joint_association` and leaves `association_uncertainty`
false. Python defaults leave joint assignment disabled. SciPy is declared as a
runtime dependency for the assignment solver.

The active mode jointly matches all tracks in each existing priority tier:
confirmed first, then tentative. Within the existing 1.2 m distance gate it
maximizes matched tracks and then minimizes total distance. Explicit dummy
assignments allow misses, and an observation cannot update two tracks. Point
rejection, clustering, confirmation and static support remain fixed.

## Results

In the controlled failure case, tracks predicted at `(2,0)` and `(3,0)` receive
observations `(2.4,0)` and `(1.4,0)`. Greedy matching updates one track and creates
a redundant tentative track; joint matching updates both existing tracks.

On the saved UMRR walk, the active distance mode preserves all scored coverage
and far-track counts: walking 591/618 with far tracks in 10/618, standing 88/145,
swaying 63/108 with no far tracks in either later phase. It makes a different
full rollout, rather than merely scoring alternative assignments on baseline
states. The two quiet recordings produce no confirmed tracks, and the scored
absence interval also stays empty.

Twenty seeded synthetic crossings with measurement noise and missing detections
yield the same aggregate as baseline: 7,237/7,240 object opportunities matched,
zero ID changes and zero unmatched tracks. The known assignment defect is fixed,
but these samples do not establish a broader accuracy improvement.

## Rejected uncertainty-gate variants

The branch retains an explicitly optional EKF innovation-cost mode for research.
It uses the existing covariance model, a nominal 99% chi-square gate and
per-track miss choices. These are not calibrated probability or coverage claims.
`association_doppler` controls the signed-speed term only in that mode.

With both uncertainty and Doppler enabled, the replay's swaying person-proxy
coverage falls from 63/108 to **0/108**, while far-track presence rises to 32/108.
Position-only innovation gating preserves swaying but loses three walking scans.
These settings are not the active worktree configuration. Copying an uncertainty
formula does not compensate for an uncalibrated measurement/cluster noise model.

## Validation and use

124 processing pytest cases and package lint pass, covering one-to-one matches,
misses, confirmed priority, observation permutation, the counterexample, signed
velocity cost and ROS configuration. With joint assignment disabled, tracker
states match baseline exactly across all 3,436 recorded scans.

Build/install: workspace `.colcon/umrr96-exp-association/`; test domains 185/186.
No hardware or RViz persistence changes were made. The separate point and track
magnitude-only ghost rules can still suppress independent people before or after
association; this experiment does not solve those rules. A physical multi-person
capture is needed to assess practical benefit.

See [the evidence](experiment-association.json) and the shared comparison scripts
in workspace `src/smartmicro_ros2_radars/tools/assessment/`.
