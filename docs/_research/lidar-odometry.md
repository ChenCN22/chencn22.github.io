---
layout: page
title: "Robust LiDAR Odometry with Vehicle-Kinematic Inequality Constraints"
permalink: /research/lidar-odometry
---

**Course project, CMU 24-785 Engineering Optimization · Fall 2025**

<figure style="margin:1rem 0;"><img src="/assets/img/lidar-odom/open-area-comparison.jpg" alt="Estimated trajectory with and without the kinematic constraint on the AutoMine route, zoomed on an open area" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">AutoMine mining route (6.3 km). Left: with the vehicle-kinematic constraints the estimate stays on a car-like path through the open area. Right: plain point-to-plane ICP wanders where the scan has no structure to lock onto.</figcaption></figure>

LiDAR odometry estimates the pose change between consecutive scans by aligning point clouds
(point-to-plane ICP). In structure-less scenes such as open pits, flat fields and long corridors the
alignment becomes ill-posed: many poses fit the scan equally well, and the solver drifts off.

The idea of this project: a car cannot move sideways or spin in place, so the pose increment is
bounded by the vehicle's own kinematics. We add those bounds as **inequality constraints** to the ICP
problem and keep the rest of the pipeline unchanged.

- **Bicycle-model constraints.** The yaw increment is bounded by the forward translation times
  tan(max steering angle) / wheelbase, both ways, plus a forward-only constraint. Steering limits come
  from the vehicle's wheelbase and turning circle (Ackermann geometry).
- **Solver.** The damped Gauss–Newton step of ICP becomes a sequence of small QPs, solved with an
  active-set method (qpmad, Goldfarb–Idnani) under Levenberg–Marquardt damping, a Huber robust
  kernel and Jacobian column scaling. C++ / ROS 2; the IMU is used only for the initial guess and
  scan deskewing.
- **Result.** Drift (RMSE / trajectory length) fell from **1.7 % to 1.4 %** on a 6.3 km AutoMine mining
  route and from **1.9 % to 1.4 %** on the 19 km HeLiPR Bridge03 urban route.

## When do the constraints matter?

<figure style="margin:1rem 0;"><img src="/assets/img/lidar-odom/duals-over-time.jpg" alt="Lagrange multipliers of the inequality constraints over time, annotated with sharp turns, open-area turns, a big turn and a vast open area" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Lagrange multipliers of the active-set QP over the AutoMine run. The constraints activate on sharp turns and, most strongly, in the vast open area near the end, exactly where unconstrained ICP fails.</figcaption></figure>

The multipliers also show the constraint is a bound, not a prior: shrinking the maximum steering angle
to 90 % of the vehicle's value over-constrains the turns and drift goes back up to 1.9 %.

<div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1rem 0;">
  <figure style="margin:0;"><img src="/assets/img/lidar-odom/automine-trajectory.jpg" alt="AutoMine trajectory with constraint vs. ground truth" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">AutoMine, with constraints. Dashed: ground truth; colour: deviation from it.</figcaption></figure>
  <figure style="margin:0;"><img src="/assets/img/lidar-odom/helipr-trajectory.jpg" alt="HeLiPR Bridge03 trajectory with constraint vs. ground truth" style="width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.85em;margin-top:.3rem;">HeLiPR Bridge03 (urban, 19 km), with constraints.</figcaption></figure>
</div>

## My part

Dataset survey and data pipeline: a converter from HeLiPR (Ouster point clouds, Xsens IMU, ground
truth) and KITTI into ROS 2 bags the pipeline could replay, and the vehicle-parameter derivation
(wheelbase, turning circle → steering-angle bounds) behind the constraints.

*Stack: C++, ROS 2, qpmad, Python (data conversion), AutoMine and HeLiPR datasets.*
