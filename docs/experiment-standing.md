# Experiment: standing support from a saved background

Branch `experiment/umrr96-standing`, based on shared checkpoint `3aee7a7`.
The worktree YAML enables `standing_support` with a 30-second absolute hold cap;
the Python feature default is false. Moving-point rejection and association are
unchanged.

When a moving track confirms against a warmed background, it saves the known
background cells. After motion stops, current static returns can support it if
they are outside that saved background and remain within the existing 0.5 m
gate and a 0.5 m stop anchor. Each static point can support at most one track.
Three supported scans in five establish standing support. After establishment,
intermittent current returns may renew it; requiring a new dense burst on every
renewal caused the first prototype to lose the person in the recorded walk.

The track still expires after the existing one second without support and has
an absolute 30-second static-only cap. New moving hits reset its standing anchor
and verification. Sensor movement invalidates the saved background. Unknown
static input cannot renew support. The current-scan display does not acquire
any historical points; internal track coasting is distinct from RViz decay.

## Results

Recorded standing coverage improves from **88/145 to 145/145 scans**; swaying
coverage improves from **63/108 to 108/108**. Walking remains 591/618. There are
no added far tracks: walking remains 10/618, standing/swaying zero. The scored
absence interval has zero tracks across 290 scans, as do the two quiet captures
totaling 1,591 scans.

Of the 145 standing scans, 98 have a current supporting return; the remaining
47 rely on bounded track coasting. Baseline values were 53 current-supported
scans out of 88 covered. This is track continuity, not increased native point
density or an independently annotated person-detection rate.

Changing only the legacy `static_hold` to 30 seconds also reaches 145/145 and
108/108 in this particular recording. The saved-background mechanism matters
in the controlled long-stop test: both the baseline and timeout-only control
eventually absorb the person into background and cover only 69/240 standing
scans, while this branch covers **240/240 with current support in every scan**.
After departure, it coasts for 18 scans (about 0.99 seconds), then disappears;
there are zero lingering tracks beyond the existing coast limit, including when
a nearby previously occluded wall reappears.

## Validation and limits

128 processing pytest cases and package lint pass. New tests cover long stops,
intermittent returns, current support expiry, absolute age, anchor drift, nearby
wall reappearance, exclusive support, sensor-motion reset and ROS parameters.
Disabled tracker states match baseline exactly across 3,436 recorded scans.
The synthetic crossing suite shows no change from baseline.

This is the strongest result among the three worktrees, but it uses one already
examined room recording and controlled synthetic cases. Frozen background cells
can reject a person whose return overlaps preexisting structure, and a persistent
novel reflector at the person's position can still be ambiguous. No obstacle
layer or independent multi-person accuracy evaluation was performed. It remains
an experimental branch pending a physical standing/exit capture near walls.

Build/install: workspace `.colcon/umrr96-exp-standing/`; test domains 183/184.
The radar remained off. See the [evidence](experiment-standing.json) and the
shared comparison report in workspace `src/smartmicro_ros2_radars/docs/`.
