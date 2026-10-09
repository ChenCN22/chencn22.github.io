---
layout: page
title: "3D Reconstruction of Shoe Interiors for Footwear Fit"
permalink: /research/in-shoe-reconstruction
---


**CERLAB, CMU · Sep 2026 – present · research assistant with Prof. Kenji Shimada**


<figure style="margin:1rem 0;"><img src="/assets/img/shoe/concept.jpg" alt="Concept: a slender probe is inserted into the shoe cavity and scanned along a rail; the cavity wall carries printed fiducials so each view can be registered." style="max-width:100%;border-radius:6px;background:#fff;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Concept: a slender probe is inserted into the shoe cavity and scanned along a rail; the cavity wall carries printed fiducials so each view can be registered.</figcaption></figure>
## The problem

Whether a shoe actually fits is decided by the geometry of its *inside* — and for people
with special foot shapes (diabetic feet are the motivating case) a poor fit is a medical
problem, not a comfort one. Yet the interior of a closed shoe is one of the hardest things
to measure: a narrow, dark, low-texture cavity that no ordinary 3D scanner can see into.

The goal is a **geometry-only 3D reconstruction of the shoe cavity** (no appearance needed)
accurate enough to compare against a foot model. The scope was recently widened to other
closed interiors such as garment sleeves and trouser legs.

## What I'm doing
<figure style="margin:1rem 0;"><img src="/assets/img/shoe/stereo-error.jpg" alt="Error budget: depth error vs. wall distance for candidate stereo baselines — the basis for choosing baseline, lens and standoff." style="max-width:720px;border-radius:6px;background:#fff;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Error budget: depth error vs. wall distance for candidate stereo baselines — the basis for choosing baseline, lens and standoff.</figcaption></figure>


I'm the researcher on this project, from requirements to hardware:

- **Requirements and error budget** — derived the depth-of-field and reconstruction-error
  requirements for a probe that has to work at very short standoff inside a cavity.
- **Sensing concept and component selection** — compared candidate sensing approaches and
  chose optics and sensors that satisfy the budget; two sensing routes are being validated
  in parallel.
- **Probe and fixture design** — designed the insertion probe and a manual rail fixture for
  repeatable, static, point-by-point measurements.
- **Test plan with ground truth** — first on enlarged 3D-printed shoe-cavity halves whose
  CAD model is the ground truth, then on real shoes.


<figure style="margin:1rem 0;"><img src="/assets/img/shoe/probe-head.jpg" alt="Preliminary probe-head concept and the scanning motion (push/pull + roll) used to cover the side walls and toe end." style="max-width:100%;border-radius:6px;background:#fff;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Preliminary probe-head concept and the scanning motion (push/pull + roll) used to cover the side walls and toe end.</figcaption></figure>
**Target:** reconstruction error ≤ 1 mm. Hardware is on order and the fixtures are being
printed; first measurements are next.

*Stack: optical design & error analysis, SolidWorks (probe/fixture), 3D printing, Python
(calibration & reconstruction).*
