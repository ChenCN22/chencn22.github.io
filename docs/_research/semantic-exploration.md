---
layout: page
title: "5-DoF Semantic Exploration with VLM-Guided Active Inspection"
permalink: /research/semantic-exploration
---

# 5-DoF Semantic Exploration with VLM-Guided Active Inspection

**CERLAB, CMU · 2025 – 2026 · my main M.S. research project**

📄 **Paper:** X. Zhan\*, **S. Chen\***, K. Shimada, "Pose-aware Legged Robot Semantic
Exploration with Omnidirectional Perception in Confined Unknown Environments,"
[arXiv:2609.19460](https://arxiv.org/abs/2609.19460), Sep 2026 — *submitted to ICRA 2027,
under review.* \*Equal contribution.

## The problem

Industrial inspection robots usually treat exploration as a pure geometry problem: cover
the unknown volume, then stop. But real inspection targets — a robot arm, a valve, a
machine tool — have *semantics*, and the most informative view of them is often not the
one a flat-bodied robot gets for free. A ground robot's limited vertical field of view and
the short standoff distances of a confined space leave the **upper surfaces** of tall
equipment unobserved from planar (3-DoF: x, y, yaw) viewpoints.

A quadruped like Boston Dynamics Spot can fix that by tilting its body — adding **pitch and
roll** for 5-DoF viewpoints — but every extra posture and observation costs mission time.
This project is about getting that coverage-vs-efficiency trade-off right.

## What the system does

Spot, carrying an omnidirectional camera–LiDAR suite, explores an unknown industrial scene
end-to-end:

1. **Frontier-based exploration** with an incremental roadmap planner (information-gain
   scoring, TSP-ordered global tours) drives coverage of the unknown map.
2. **Semantic mapping** — 2D instance-segmentation detections are back-projected using the
   **per-instance visible-pixel masks** (not just bounding boxes), DBSCAN-filtered, and
   fused into 3D semantic object boxes in the occupancy map.
3. **5-DoF viewpoint sampling** — candidate viewpoints around each mapped object add the
   robot's **body pitch and roll** to its planar pose; postures are selected from the
   partial object map by **expected coverage gain**.
4. **Aim-aligned execution** — approach and body reorientation are aligned with the
   viewing aim, so the robot spends fewer postures (and less time) per object.
5. **VLM-assisted viewpoint pruning** — an object-centric strategy in which a
   vision-language model, given the persistent observation history and a bird's-eye-view
   (BEV) map, prunes redundant inspection visits. A geometric answer remains the automatic
   fallback on timeout or API error, so the autonomy stack never blocks on a network call.
6. The resulting semantic viewpoints are **merged with geometric exploration viewpoints**
   in one global exploration planner.

The VLM's reasoning is overlaid on the queried frame and streamed to rviz, which makes its
decisions auditable in real time.

## Results

**Simulation** — three Isaac Sim industrial environments with ground-truth surfaces,
repeated runs per scene, compared against a planar (3-DoF) planning baseline and several
other exploration baselines:

- **+8–10 percentage points** of final target-surface coverage over the planar baseline;
- **17–32% less exploration time**;
- **53–73% fewer postures** than competing 5-DoF methods;
- the **highest mean object-coverage AUC** among the evaluated baselines.

**Real world (qualitative demonstration)** — the full system ran on a Boston Dynamics Spot
with the omnidirectional camera–LiDAR suite in a university machine shop, detecting and
reconstructing five target machines. The sensor side of that deployment is on the
[LiDAR–camera registration](/research/lidar-camera-registration) page.

## Engineering notes I'm proud of

- The VLM bridge is a standalone ROS node speaking plain `PoseStamped`/`Vector3Stamped`
  topics with stamp-matched request/response — the C++ planner gained zero new
  dependencies (no JSON, no HTTP) and stays fully functional without the bridge.
- A reproducible evaluation harness scores every autonomous run against a ground-truth
  scan of the scene — target-surface coverage over time, time-to-coverage, semantic
  completeness, and VLM query cost — under a real-time profile (simulation real-time
  factor pinned to 1.0) so latency-sensitive comparisons stay honest. Isaac Sim and the
  planner stack run in separate Docker containers bridged over ROS, with scenes, robots,
  and sensors configured from version-controlled YAML.
- Plenty of hard-won autonomy robustness: e.g. diagnosing a cold-start deadlock where a
  forward-facing pinhole camera leaves the robot an "unknown-voxel island" with no valid
  roadmap transitions — fixed with an in-place scan rotation
  ([write-up here]({% post_url 2026-07-21-exploration-cold-start-debugging %})).

## Next steps

Richer VLM context (object-map crops, multi-frame queries), and porting the validated
planner onto the Isaac Sim 5.0 + ROS2 branch.

*Stack: ROS Noetic + Isaac Sim (this branch), Isaac Sim 5.0 + ROS2 (parallel branch),
C++ (planner), Python (VLM bridge, simulator tooling), Docker.*
