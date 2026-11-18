---
layout: page
title: "LLM-Guided RL Manipulator with an Imperfect Pick-and-Place Skill"
permalink: /research/llm-manipulator
---

**Course project, CMU 24-880 AI Agents for Engineers · Fall 2026**

<figure style="margin:1rem 0;"><video autoplay muted loop playsinline controls poster="/assets/img/llm-manipulator/thumb.jpg" style="width:100%;max-width:560px;display:block;margin:0 auto;border-radius:6px;"><source src="/assets/img/llm-manipulator/agent-episode.mp4" type="video/mp4"></video><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">One held-out episode with an unreliable camera, replayed at real speed. The front camera wrongly reports the yellow block as placed; the agent checks with the top camera, sees the claim contradicted, and keeps working. When a pick fails silently at the rim, it pushes the block inward, picks again, and finishes the task.</figcaption></figure>

<div style="background:#f6f6f6;border-left:4px solid #c41230;padding:.8rem 1rem;margin:1rem 0;font-size:.95em;">
<strong>In brief.</strong> Language-model agents are starting to direct robots by calling learned skills such as "pick" and "place". Those skills fail without saying so, and the cameras that report the result can be wrong. I built a simulated robot arm where both happen and compared agent designs across seven language models. The design that keeps what each camera said in program memory, instead of asking the model to remember it, finished 79–85% of tasks with a faulty camera on six of the seven models, while a standard agent fell below a rule that never looks at all.
</div>

## Introduction

Factories and homes are starting to use large language models (LLMs) as the planner on top of a robot's learned skills: the model reads the task, calls "pick" or "place", and decides what to do next. This makes robots easier to instruct, but it only helps if the agent notices when something goes wrong. Learned skills fail on a sizable share of attempts and rarely report it, and a camera that checks the result can itself be mistaken. An agent that trusts every report either stops before the job is done or keeps repeating a failed action, and each extra model call costs time and money.

**The objective of this project** was to find which agent designs still finish the task when both the skill and the perception are unreliable, and why, with results that hold across different language models rather than only the one the prompts were written for.

## Methods

**Testbed.** A simulated Franka Panda arm in PyBullet has a learned pick-and-place skill, trained with reinforcement learning (TQC with hindsight experience replay) on a single block and then frozen. The agent uses it through six tools. Tasks use four blocks: building a stack, moving blocks into zones in a given order, and placing under a constraint. An action returns only "executed"; the agent has to look to find out whether it worked. The skill fails on about a third of its attempts, mostly near the table rim.

<figure style="margin:1rem 0;"><img src="/assets/img/llm-manipulator/scene-front.jpg" alt="Front camera view of the simulated table: four colored blocks, three gray target zones and a rim around the workspace" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">The scene from the front camera: four blocks, three target zones and a rim. The agent never learns from the skill whether an action worked; it has to look, and a look can be wrong.</figcaption></figure>

**Perception.** Three cameras (front, top, side) report the scene in two settings. In the first, reports are text and errors are injected at known rates, either at random or from one badly placed camera. In the second, perception is real: a vision-language model (VLM) detects the blocks in rendered images, and the calibrated camera geometry maps each detection to a zone or a stack.

**A clean simulation.** Identical runs first differed in 10 to 14 of 60 episodes. Making PyBullet's contact ordering deterministic and caching each model answer made every run repeatable, so differences between agents come from the agents.

**Agents.** All agents are LangGraph state graphs built from one library of nodes; they share the model call, the prompt and the tools and differ only in what the model sees when it chooses the next call. Nine designs were built (plan-and-execute, ReAct, reflection, memory, a forced second camera, self-assessment, a belief table, an independent reviewer, and evidence kept in code). Three went into the large runs: plain ReAct, ReAct with a forced second camera, and the evidence design, which keeps per-goal camera reports in program memory, fuses them, and treats all of them as stale after the arm moves.

**References and evaluation.** Four rule-based policies without an LLM serve as references: one that never looks, one that trusts a single look, one that takes the majority of three cameras, and one that acts on the ground-truth scene. Agents were compared on 120 held-out episodes per condition with seven LLMs, five run locally and two through an API.

## Results

<figure style="margin:1rem 0;"><img src="/assets/img/llm-manipulator/results-models.png" alt="Dot plot of tasks completed per language model for three agents and two rule-based references under three camera conditions" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Held-out episodes, same code and seeds, one row per language model. Green: evidence kept in code; blue: plain ReAct; orange: forced second camera. Gray marks: the rule that never looks and the ground-truth rule.</figcaption></figure>

On the model the prompts were tuned on, the three agents finished almost the same number of held-out episodes (388, 382 and 379 of 480). Across the other models the picture changed. With one faulty camera, the evidence design finished **79–85%** of the episodes on six of the seven models, while plain ReAct finished only **36–59%** on five of them, below the rule that never looks (65%). Its failures were almost all one event: it believed a single false "done" report and stopped. Against the rules, the evidence design matched the majority-of-three rule within about 3 points, and with a correct camera the stronger models reached the ground-truth rule. It also used about a quarter of the tokens of the other agents.

<figure style="margin:1rem 0;"><img src="/assets/img/llm-manipulator/results-skill.png" alt="Left: pick success by distance to the rim. Right: recovery with and without pushing first" style="max-width:100%;border-radius:6px;"><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Left: the pick skill is reliable in the middle of the table and weak at the rim. Right: pushing the block toward the middle before picking again recovers more failed picks, most of all at the rim.</figcaption></figure>

<figure style="margin:1rem 0;"><video autoplay muted loop playsinline controls poster="/assets/img/llm-manipulator/push-recovery-poster.jpg" style="width:100%;max-width:480px;display:block;margin:0 auto;border-radius:6px;"><source src="/assets/img/llm-manipulator/push-recovery.mp4" type="video/mp4"></video><figcaption style="color:#666;font-size:.9em;margin-top:.4rem;">Push as recovery, shown with the rule that reads the true scene: the pick fails at the rim, a push moves the block inward, and the next pick works.</figcaption></figure>

Most of the remaining failures come from the skill, not the agent: picks fail mostly when a block lies close to the table rim. A second skill, push, trained the same way, moves the block toward the middle first. After a failed pick, pushing and then picking recovered 83% of blocks within 2 cm of the rim, against 57% for simply picking again (**+26 points**), and 95% against 81% farther in (+14 points, 95% interval +7 to +21). Given push as a recovery tool, the evidence agent finished **93–96%** of held-out episodes with a correct camera, against 85% without it, and 87–88% against 79% with one faulty camera; all 95% intervals exclude zero.

## Discussion

The main lesson is about where behavior should live. A check written into the prompt depends on how each model reads that prompt, and it did not carry over to models it was not tuned on. The same check written into the program, as state the agent keeps outside the model, worked on every capable model. This matches a principle from the course: state belongs in the application layer, not in the context window.

Two results set limits. First, differences found on the episodes used for design did not hold on new episodes with the same model, so only held-out results are reported here. Second, on short tabletop tasks the best agents only matched a good hand-written rule. Agents and learned components should pay off in long-horizon tasks with many goals and branching decisions, where no rule can list every case. The study is also limited to simulation and to one family of skills.

The next step is already running: a memory across episodes of where each skill fails, tested on tasks where the agent chooses which blocks to move, so it can, for example, push a block away from the rim before it ever fails.

## My role

I designed the study and did all of the work: the simulation testbed, skill training, the agents and rule-based references, the VLM perception pipeline, all experiments on the lab GPU server, and the analysis.
