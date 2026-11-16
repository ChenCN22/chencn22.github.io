---
layout: page
title: "Robust LiDAR Odometry with Vehicle-Kinematic Inequality Constraints"
permalink: /research/lidar-odometry
---

**Course project, CMU 24-785 Engineering Optimization · Fall 2025 · team project**

<figure style="margin:1rem 0;"><img src="/assets/img/lidar-odom/open-area-comparison.jpg" alt="Estimated trajectory with and without the kinematic constraint on the AutoMine route, zoomed on an open area" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">AutoMine mining route (6.3 km). Left: with the vehicle-kinematic constraints the estimate stays on a car-like path through the open area. Right: plain point-to-plane ICP wanders where the scan has no structure to lock onto.</figcaption></figure>

<div style="background:#f6f6f6;border-left:4px solid #c41230;padding:.8rem 1rem;margin:1rem 0;font-size:.95em;">
<strong>In brief.</strong> Autonomous mining trucks and farm vehicles work where GPS is unreliable and the LiDAR often sees nothing but flat ground. We added the one thing the scan cannot tell the odometry but the vehicle already knows, that a car cannot slide sideways or spin in place, as inequality constraints inside the scan-matching solver. Drift fell by about a quarter on two long routes, and the solver's own multipliers show the constraints switch on exactly where the scan fails.
</div>

## Introduction

Vehicles that drive themselves in mines, quarries and fields need to know where they are without
GPS, which is blocked by pit walls and unreliable near buildings. LiDAR odometry provides that: it
estimates the pose change between consecutive scans by aligning point clouds (point-to-plane ICP)
and integrates the increments over time. The weak spot is structure-less terrain. In an open pit,
a flat field or a long corridor, many poses fit the scan equally well, the alignment becomes
ill-posed and the estimate drifts, sometimes into motions no car could physically make.

A car's own kinematics rule those motions out. With a fixed wheelbase and a maximum steering angle,
it cannot move sideways, cannot yaw faster than its forward speed allows, and cannot reverse without
intending to. **The objective of this project** was to put those physical bounds into the ICP
problem as inequality constraints, leave the rest of a standard odometry pipeline unchanged, and
measure how much drift they remove on real driving data.

## Methods

**Bicycle-model constraints.** For each scan pair the yaw increment is bounded, in both directions,
by the forward translation times tan(maximum steering angle) divided by the wheelbase, plus a
forward-only constraint on the translation. The steering limit comes from the test vehicle's
wheelbase and turning circle through Ackermann geometry, so the constraints need no tuning
parameter beyond the vehicle's spec sheet.

**Solver.** The damped Gauss–Newton step of ICP becomes a sequence of small quadratic programs,
solved with an active-set method (qpmad, Goldfarb–Idnani) under Levenberg–Marquardt damping, a Huber
robust kernel and Jacobian column scaling. The pipeline is C++ and ROS 2; the IMU is used only for
the initial guess and scan de-skewing.

**Evaluation.** Two public routes with ground truth: AutoMine, a 6.3 km open-pit mining route, and
HeLiPR Bridge03, a 19 km urban route. The metric is drift, the trajectory RMSE divided by the
distance travelled, with and without the constraints. The Lagrange multipliers of the QP are
logged over time to see *where* the constraints become active.

## Results

Drift fell from **1.7% to 1.4%** on AutoMine and from **1.9% to 1.4%** on HeLiPR Bridge03.

<div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1rem 0;">
  <figure style="margin:0;"><img src="/assets/img/lidar-odom/automine-trajectory.jpg" alt="AutoMine trajectory with constraint vs. ground truth" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">AutoMine, with constraints. Dashed: ground truth; colour: deviation from it.</figcaption></figure>
  <figure style="margin:0;"><img src="/assets/img/lidar-odom/helipr-trajectory.jpg" alt="HeLiPR Bridge03 trajectory with constraint vs. ground truth" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">HeLiPR Bridge03 (urban, 19 km), with constraints.</figcaption></figure>
</div>

The multipliers show when the constraints matter. They are zero on most of the route, activate
briefly on sharp turns, and are largest in the vast open area near the end of the AutoMine run,
which is exactly where unconstrained ICP wanders off.

<figure style="margin:1rem 0;"><img src="/assets/img/lidar-odom/duals-over-time.jpg" alt="Lagrange multipliers of the inequality constraints over time, annotated with sharp turns, open-area turns, a big turn and a vast open area" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Lagrange multipliers of the active-set QP over the AutoMine run. The constraints activate on sharp turns and, most strongly, in the open area at the end.</figcaption></figure>

## Discussion

The result confirms the premise: the constraints help where the scan carries no information and
stay silent where it does. That makes them a bound rather than a prior, and a sanity check showed
the difference. Shrinking the maximum steering angle to 90% of the vehicle's true value
over-constrains the turns and drift goes back up to 1.9%, so the bounds must come from the real
vehicle geometry, not from tuning.

The improvement is real but modest, about a quarter of the drift, and it is limited in three ways.
The constraints only bound planar motion, so vertical drift is untouched. They assume a car-like
vehicle; a skid-steer or a legged robot violates them. And because they act only when the
alignment is ill-posed, they cannot repair a scan match that converges confidently to the wrong
answer. A natural extension is to tie the bounds to measured speed from wheel odometry, which would
tighten them in slow turns.

## My role

Team project; my teammates implemented the constrained QP step inside the ICP loop. I owned the
data side and the constraint parameters:

- **Dataset survey** across GOOSE, HeLiPR, KITTI and AutoMine for routes with LiDAR, IMU and
  ground truth, and a check of which could actually be replayed in ROS 2.
- **Data pipeline:** converters from HeLiPR (Ouster point clouds, Xsens IMU, ground truth) and
  KITTI into ROS 2 bags the pipeline could replay, including an Ouster bag converter.
- **Vehicle-parameter derivation:** wheelbase and turning circle to the steering-angle bounds
  through Ackermann geometry, which is what the constraints are built on.

*Stack: C++, ROS 2, qpmad, Python (data conversion), AutoMine and HeLiPR datasets.*
