---
layout: post
title: "Diffusion Models — IDL Lecture 23 (11/17)"
date: 2025-11-17
categories: [Deep Learning]
tags: [Diffusion, VAE, DDPM, DDIM, SDE, Score Matching, CFG, Stable Diffusion]
---

# Diffusion Models — IDL Lecture 23 (11/17)

---

## 1. Motivation

- **VAE** takes **one giant step** from noise to image → often results in *blurry outputs*  
- **Diffusion models** break this step into **many tiny steps**, improving detail and stability  

---

## 2. Definition

### Forward Process
- Gradual **noise addition** over multiple timesteps  
- Forms a **Markov chain**  
- Purpose: gradually destroy structure in the data until pure Gaussian noise remains  

### Reverse Process
- **Denoising** process  
- Learns to invert the forward chain: predict and remove noise step-by-step  

---

## 3. How Noise is Added

- Gaussian noise is added progressively  
- Each timestep adds a small, controlled amount of noise  

**Model input:** noisy image and timestep  
**Model output:** predicted noise  

---

## 4. Training Objective

- Loss: **MSE** between actual added noise and model-predicted noise  
- Often scaled by \( \alpha_t \) (noise schedule parameter)  
- Implicitly learns **image reconstruction**  

---

## 5. Training and Sampling

Key components:
- **Timestep** \( t \)
- **Noise schedule** \( \alpha_t \)
- **Random noise** \( \epsilon \)

**Sampling:**  
Reverse diffusion — starting from noise, iteratively denoise to generate an image.

---

## 6. DDIM (Denoising Diffusion Implicit Models)

### DDPM Challenges
- Requires ~1000 timesteps  
- Slow sampling  

### DDIM Improvements
- **Skips timesteps intelligently**  
- **Non-Markovian** process  
- Enables **deterministic sampling**

**Key Components:**
- Predicted \( x_0 \)  
- Estimated noise \( \epsilon \)  
- Denoise function using schedule (\( \mu, \sigma \))  

---

## 7. SDE & Score Matching

- **DDPM:** fixed discrete timesteps and noise schedule  
- **DDIM:** continuous-time formulation via SDEs  

### Modeling Diffusion
- Represented as **Stochastic Differential Equations (SDEs)**  
- **Probability Flow ODEs**: deterministic equivalent of SDEs → exact reconstruction possible  
- Provides a **unified framework** for diffusion-based generative models  

---

## 8. Forward Diffusion Dynamics

- **Drift term:** pulls data toward the mode  
- **Diffusion term:** injects noise  

---

## 9. Score Function

- Models \( \nabla_x \log p(x) \) — points toward high-probability regions  
- No need to compute the normalizing constant \( Z \)  
- **Denoising Score Matching (DSM):** learn backward score for denoising  

---

## 10. CFG — Classifier-Free Guidance

- Traditional approach: classifier steers gradient → adds computation  
- **Classifier-free:**  
  - Train both *conditional* and *unconditional* models jointly  
  - During sampling: interpolate between them using **guidance scale (γ)**  

---

## 11. Performance Metrics

- **Fréchet Inception Distance (FID)** — realism and diversity  
- **Inception Score (IS)** — quality and discriminability  
- **Precision & Recall** — fidelity vs coverage  

---

## 12. Latent Diffusion Models (LDM)

### Why Latent Space?
- Pixel-level diffusion is computationally expensive  
- Latent space: lower dimensional, more efficient  

### Architecture
- **Encoder–Decoder** structure  
- **Stable Diffusion**: operates in latent space  
- **DiT (Diffusion Transformer):** replaces U-Net backbone with Transformer  

---

## 13. Summary

| Model | Approach | Key Trait |
|:------|:----------|:-----------|
| **VAE** | One big step | Blurry results |
| **Diffusion** | Many small steps | Stable and detailed |

- Forward diffusion uses **fixed equations** (no training needed)  
- Reverse process (denoising) is **learned**  

---

## 14. Additional Notes

1. Forward schedule: fixed Gaussian addition per step  
2. Reverse process: shared denoising parameters  
3. SDE → continuous generalization of DDPM  
4. Classifier guidance:  
   - **With classifier:** corrects generated output gradients  
   - **Without classifier:** blend conditional & unconditional outputs (CFG)

---
