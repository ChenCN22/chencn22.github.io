---
layout: page
title: "6-DoF Semantic Exploration with VLM-Guided Active Inspection"
permalink: /research/semantic-exploration
---

# 6-DoF Semantic Exploration with VLM-Guided Active Inspection

**CERLAB Roboteam, CMU · 2025 – present (ongoing, my current main project)**

Industrial inspection robots usually treat exploration as a pure geometry problem: cover
the unknown volume, then stop. But real inspection targets — a robot arm, a valve, a
conveyor — have *semantics*, and the most informative view of them is often not the one a
flat-bodied robot gets for free. Tall equipment needs a camera tilted **up**; low bases
need a camera tilted **down**. A quadruped like Boston Dynamics Spot can do both by tilting
its body, which turns viewpoint planning into a genuine 6-DoF problem.

## What the system does

A simulated Spot (NVIDIA Isaac Sim, kinematic control for fast iteration) autonomously
explores an industrial scene end-to-end:

1. **Frontier-based exploration** with an incremental roadmap planner (DEP-style
   information gain, TSP-ordered global tours) drives coverage of the unknown map.
2. **Semantic mapping** — 2D instance-segmentation detections are back-projected through
   the depth camera using the **per-instance visible-pixel masks** (not just bounding
   boxes), DBSCAN-filtered, and fused into 3D semantic object boxes in the occupancy map.
3. **Object viewpoints** are sampled around each mapped object and injected into the
   global plan, so the robot deliberately swings by the things it recognized.
4. **6-DoF posture decision at each object viewpoint** — this is the interesting part.
   Two interchangeable decision modes share the same trigger point, discrete posture set,
   and execution path:
   - **Geometric baseline:** sweep discrete body roll/pitch candidates and pick the one
     that maximizes unseen-voxel information gain around the object.
   - **VLM mode:** send the live camera frame to a vision-language model (Gemini; GPT-4o
     supported through the same interface) which reasons about *what part of the object
     is missed by the flat view* and picks a posture from the same discrete set. The
     planner keeps the geometric answer as an automatic fallback on timeout or API error,
     so the autonomy stack never blocks on a network call.

The VLM's perception and one-sentence justification are overlaid on the queried frame and
streamed to rviz, which makes its decisions auditable in real time.

## Engineering notes I'm proud of

- The VLM bridge is a standalone ROS node speaking plain `PoseStamped`/`Vector3Stamped`
  topics with stamp-matched request/response — the C++ planner gained zero new
  dependencies (no JSON, no HTTP) and stays fully functional without the bridge.
- Reproducible multi-container setup: Isaac Sim and the planner stack run in separate
  Docker containers bridged over ROS, with the scene, robots, and sensors configured
  from version-controlled YAML.
- Plenty of hard-won autonomy robustness: e.g. diagnosing a cold-start deadlock where a
  forward-facing pinhole camera leaves the robot an "unknown-voxel island" with no valid
  roadmap transitions — fixed with an in-place scan rotation
  ([write-up here]({% post_url 2026-07-21-exploration-cold-start-debugging %})).

## Benchmarking

Both decision modes now run inside a reproducible evaluation harness that scores a full
autonomous run against a ground-truth scan of the scene. It compares a ladder of planner
variants under identical conditions:

- **Geometric (no VLM):** posture chosen by unseen-voxel information gain — the zero-cost baseline.
- **VLM-per-viewpoint:** the model is queried on arrival at each object to pick the inspection posture.
- **Session + predicted posture:** the posture is decided once per object at a "consultation"
  step and executed on arrival with zero wait, cutting redundant VLM calls.
- **Receding-horizon:** postures are pre-decided as the tour approaches each object, so the
  robot inspects targets *without stopping* — the closest to how a real deployment would run.

Runs are scored on **ground-truth surface coverage**, **time-to-90%-coverage**, semantic
completeness, and **VLM query cost** (calls + tokens), under a real-time profile (simulation
real-time factor pinned to 1.0) so the latency-sensitive receding-horizon comparison is honest.

## Next steps

Richer VLM context (object-map crops, multi-frame queries), and porting the validated
planner ladder onto the Isaac Sim 5.0 + ROS2 branch.

*Stack: ROS Noetic + Isaac Sim (this branch), Isaac Sim 5.0 + ROS2 (parallel branch),
C++ (planner), Python (VLM bridge, simulator tooling), Docker.*
