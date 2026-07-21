---
layout: page
title: "DC Motor PID Control in Noisy Environments"
permalink: /projects/dc-motor
---

# DC Motor Control in Noisy Environments

**Solo project → published at ICCSM 2024 (Shanghai)**

A comparative study of two ways to make PID control robust to measurement noise:

- **Wavelet MRPID** — multi-resolution decomposition of the error signal, with separate
  gains per frequency band, so high-frequency noise can be de-emphasized without
  sacrificing tracking response.
- **CNN-attention integrated PID** — a learned front-end that filters/weights the error
  signal before the PID law.

Both were implemented and benchmarked on a DC-motor speed-control testbed under injected
noise, comparing tracking error, overshoot, and control-effort smoothness.

**Publication:** S. Chen, "DC Motor Control in Noisy Environments: A Comparative Study of
Wavelet MRPID vs. CNN-Attention Integrated PID," *Int. Conf. on Computer Science and
Mechatronics (ICCSM)*, Shanghai, 2024.
