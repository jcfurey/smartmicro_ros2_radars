# Additional radar paper review, 2026-09-28

This follows the [ingest/filtering review](umrr96-ingest-filtering-research-20260928.md)
and its implemented per-point audit. Scope remains the current UMRR-96, a fixed
installation with possibly moving people, and a current-scan display with zero
point decay. This research did not change the live session or sensor settings.

The most useful next experiment is a small, measurable change to grouping and
association, evaluated against labeled multipath examples. Learned ghost
classification and explicit reflector estimation are promising follow-ups.
Published accuracy figures below are not predictions for this sensor.

## What the current pipeline makes possible

We have target-level position, signed radial velocity, power/noise/SNR, RCS and
four reported variances. Variance units/calibration and some quality semantics
remain unresolved. We do not have ADC samples, complex per-antenna data, a
range-Doppler cube or independent departure-angle measurements through the
audited SDK. These are the compatibility boundaries for the assessments below.

The [latest live check](umrr96-classification-live-20260928.json) averaged 23.25
points per scan at 18.18 Hz. This was an entirely static-classified sample, not
a measurement of returns per person or a multi-person tracking benchmark.
Existing synthetic cases already show two actionable failures: spatial grouping
can merge nearby points with opposing radial speeds, and the farther-return
ghost rule can reject an independent mover at another bearing.

## Papers and transferable ideas

### 1. Sparse tracking with uncertainty and global assignment

**Pegoraro and Rossi, 2021 — Real-time People Tracking and Identification from
Sparse mm-Wave Radar Point-clouds.** Reviewed full-text sections IV-C–E and
tracking evaluation; [author manuscript](https://arxiv.org/html/2105.11368).

The tracker transforms polar measurement uncertainty into a position-dependent
Cartesian covariance. It scores cluster-to-track matches using a probabilistic
association approximation and selects an overall assignment with the Hungarian
algorithm. The paper also estimates object extent and adds a separate identity
classifier. A limitation relevant here: its track-management logic deletes one
of two sufficiently close tracks.

**Our assessment:** borrow covariance-aware association and compare global
assignment with our greedy matching. Identity recognition is outside our task.
Do not copy close-track deletion: it would conflict with our crossing-person
test. Use calibrated noise floors initially, rather than assuming the SDK
variances already represent trustworthy covariance. Extent estimates should
remain optional when there are too few returns.

### 2. Doppler-aware grouping for indoor crossings

**Gao et al., June 2026 — High-Accuracy Indoor Multiple-Extended-Target Tracking
Algorithm Based on 60 GHz Millimeter-Wave Radar.** Screened the publisher's
indexed sections and author abstract; direct full-page/PDF retrieval failed.
[Publisher](https://www.mdpi.com/1424-8220/26/12/3758),
[abstract and publication record](https://pubmed.ncbi.nlm.nih.gov/42356731/).

The method combines DBSCAN spatial-extent and velocity-consistency constraints,
SNR-based candidate validation, modified nearest-neighbor association and an
EKF. Experiments use a TI IWR6843ISK platform, including crossing people. The
paper also recognizes stationary-person track deletion as a problem.

**Our assessment:** a directly relevant source for the clustering experiment,
but its numeric gates are not established for our radar. Compare signed-Doppler
compatibility against our position-only grouping. Keep isolated returns
observable; copying a minimum cluster-size rejection could worsen sparsity.
Radial velocity depends on bearing and limb motion, so a tight universal
same-speed gate can split one person. Zero radial velocity alone must not erase
a person standing still or moving tangentially.

### 3. Joint reasoning about real objects, ghosts and reflectors

**Ding et al., 2026 — Multipath Extended Target Tracking with Labeled Random
Finite Sets**, arXiv v2, March 7; reviewed as a preprint, not a verified journal
publication. [Full text](https://arxiv.org/html/2602.03464v2).

MPET-GLMB jointly models object existence, measurement grouping and propagation
path assignment, while estimating reflector states. Its initialization uses
lines in stationary returns. Table V reports **3.51 FPS** for the proposed
method in that ablation benchmark. Its empirical section uses Radar Ghost
Dataset measurements; future work includes embedded real-time deployment.

**Our assessment:** a strong architectural reference for retaining competing
direct/reflected-path explanations. A full port is not the first implementation:
that published throughput does not establish suitability for our 18.18 Hz
stream. Reflector initialization also needs adequate geometric support, which
our sparse wall patches may not provide. First test a bounded pairwise geometry
score offline, leaving ambiguous measurements visible. This would be an
adaptation inspired by the paper, not a reproduction of its filter.

### 4. A recent learned ghost classifier to investigate further

**PGA-TCN — Physics-Guided Attention and Temporal Convolutional Network-Based
Millimeter-Wave Radar Ghost Removal**, IEEE TAES early access, September 7,
2026. [Publisher abstract](https://ieeexplore.ieee.org/abstract/document/11683363),
DOI `10.1109/TAES.2026.3731603`. Abstract-level screening only.

The model combines adjacent-frame velocity changes, short-window spatial
consistency and temporal convolutions, with a physical-consistency training
loss. The abstract reports tests beyond the initial pedestrian benchmark in
additional indoor scenes.

**Our assessment:** relevant to current-scan ghost labels using internal history.
Before considering implementation, verify the exact input fields, window
length, causal inference, startup delay, training labels, code/weights and
transfer to sparse UMRR data. Small parameter storage alone would not establish
low latency. No usable author implementation was confirmed in this search.
Internal history can inform today's labels without publishing yesterday's XYZ;
that is our intended output contract, not a verified property of this paper.

### 5. Why wall reconstruction is a different data problem

**Zhou et al. — RISE: Single Static Radar-based Indoor Scene Understanding**,
2025 preprint, v5 updated August 3, 2026.
[Full text](https://arxiv.org/html/2511.14019v5),
[author project](https://rise-cvpr.github.io/).

RISE uses moving people and their multipath returns to infer reflectors from a
fixed radar. Its enhancement operates on separate transmit/receive complex
signals to construct a range–arrival-angle–departure-angle cube. A learned
completion stage then estimates layout and objects. The experiments use a TI
cascade radar. Although the project advertises a release, its linked
[repository](https://github.com/kaichen-z/RISE) was empty when checked.

**Our assessment:** highly relevant to the oblique-wall question, but the full
front end cannot run on our exposed target list. A narrower geometric experiment
could use already-detected direct/ghost pairs, with explicit uncertainty about
pairing. An inferred wall model should be a separate output with its support
and uncertainty, rather than fabricated raw detections.

### 6. Dense mapping and strong filtering: useful boundaries

**Lu et al., MobiSys 2020 — See Through Smoke: Robust Indoor Mapping with
Low-cost mmWave Radar (milliMap).**
[Paper](https://arxiv.org/abs/1911.00398),
[author code](https://github.com/ChristopherLu/milliMap).
The method learns dense map reconstruction with lidar supervision and indoor
geometry priors; its semantic component uses spectral responses.
**Our assessment:** a future mapping reference, not a way to obtain more measured
returns from this SDK. A learned completed map must remain distinguishable from
observations. The spectral component needs data outside our current interface.

**Shaker, January 2026 — Real-Time 4D Radar Perception for Robust Human Detection
in Harsh Enclosed Environments.**
[Full-text preprint](https://arxiv.org/html/2601.13364).
The study uses a 12-TX/16-RX cascaded imaging radar and filters on RCS, angles and
velocity in an enclosed test environment. **Our assessment:** its much denser
sensor and human-detection objective limit transfer. Excluding strong metal
returns or near-zero Doppler would work against our wall-observation objective;
do not adopt those thresholds as a general denoising recipe.

## A public benchmark worth using

**Kraus et al., IROS 2021 — The Radar Ghost Dataset**, with an open manuscript
posted in 2024. [Paper](https://arxiv.org/html/2404.01437),
[author repository](https://github.com/flkraus/ghosts),
[version 1.1 data](https://zenodo.org/records/6676246).

The release documents 111 original hand-labeled sequences and 460 additional
sequences formed by overlaying recordings. Radar rows include range, bearing,
radial velocity, amplitude, sensor and object/multipath annotations. There are
two radars and no radar elevation. Version 1.1 fixes a lidar synchronization
issue. These are different sensors and environments from our bench.

**Proposed use:** replay a single radar at a time for point-level ghost tests,
using the documented coordinates and measured time steps. Do not pretend the
missing elevation supports our 3D ego-velocity fit, or substitute amplitude for
our SNR gate. Test the compatible ghost/tracking stages separately. Keep original
and overlaid sequences separate, and split by scene/source recording so an
overlay cannot leak a training recording into evaluation.

Follow the [label definitions](https://github.com/flkraus/ghosts/blob/main/label_convention.md):
background is unlabeled, ignore/uncertain regions are special cases, and bounce
order is encoded rather than a simple class number. Report direct-return
retention and ghost rejection on eligible labels. Do not treat every unlabeled
point as a known true wall return. Data was inspected through documentation;
archives were not downloaded or evaluated during this review.

## Recommended experiments for this repository

These are proposed adaptations, not claims that a paper has validated our setup.

1. Establish a labeled baseline for the existing point and track ghost rules.
   Report real-return suppression separately from ghost retention, by range,
   bearing and bounce class. Retain the synthetic equal/opposite/double-speed
   independent-mover cases and add crossing, standing, tangential motion and
   partial occlusion recordings on this sensor.
2. Compare grouping variants with everything else fixed: current spatial
   linkage, signed-Doppler-aware linkage, then uncertainty-scaled gates.
   Preserve singleton measurements. Measure both accidental merging of people
   and fragmentation of a single person.
3. Compare greedy and global assignment independently of clustering changes.
   Start with a position/radial-velocity residual score and empirically justified
   noise. Report ID switches, missed people, spurious tracks, confirmation delay,
   and callback p95/p99. With 18.18 Hz input, sustained throughput must comfortably
   fit the roughly 55 ms frame interval without creating an output backlog.
4. Add multipath geometry as supporting evidence only after association is
   reliable. Require evidence for a particular reflector/path, and retain an
   ambiguous state when the room geometry or pairing is underdetermined.
   Evaluate this against a no-ghost-suppression baseline as well as today's rules.
5. Investigate the learned temporal classifier only with suitable labels and a
   causal timing contract. Measure warm-up and per-frame delivery separately.
   All variants must retain current source indices/stamps and zero display decay.

The next implementation I would choose is **Doppler-aware grouping plus a
separately evaluated global-assignment option**, after establishing the labeled
baseline. It uses available data and directly addresses observed failure cases.
The two changes should be evaluated individually before combining them.
