# March 14 Worklog Update

## Big Win Today: Force Feedback is Working

Thanks to @jlamperez's amazing writeup, I finally got force feedback working in my cheatcode teleop flow.

![image|690x359, 100%](upload://4nXAhcIqRcNswpQEaqIIWl2tbwC.jpeg)

As you can see in the terminal output on the right side:

```yaml
TFs found! Starting APPROACH phase.
[CheatCode] Force: 0.2N | Phase: APPROACH | z_off: 0.2000
[CheatCode] Force: 3.1N | Phase: APPROACH | z_off: 0.2000
[CheatCode] Force: 1.5N | Phase: APPROACH | z_off: 0.2000
[CheatCode] Force: 1.6N | Phase: APPROACH | z_off: 0.2000
[CheatCode] Force: 1.3N | Phase: APPROACH | z_off: 0.2000
Hover reached (err=0.0063m). Entering ALIGN phase.
[CheatCode] Force: 1.2N | Phase: ALIGN | z_off: 0.0500
[CheatCode] Force: 1.1N | Phase: ALIGN | z_off: 0.0500
Aligned! (xy=0.0024m, ang=0.000rad, dwell=1.7s). Starting INSERT.
[CheatCode] Force: 0.6N | Phase: INSERT | z_off: 0.0500
[CheatCode] Force: 0.6N | Phase: INSERT | z_off: 0.0100
[CheatCode] Force: 0.9N | Phase: INSERT | z_off: -0.0100
[CheatCode] Force: 1.1N | Phase: INSERT | z_off: -0.0100
[CheatCode] Force: 5.8N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 8.6N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 10.5N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.9N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.6N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.6N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.4N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 10.9N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.7N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.7N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.9N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 13.4N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 12.9N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 13.4N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 13.0N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 13.4N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 13.7N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 4.9N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 6.6N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 7.9N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 9.6N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 11.3N | Phase: INSERT | z_off: -0.0150
[CheatCode] Force: 13.1N | Phase: INSERT | z_off: -0.0150
(insertion completed)
Teleop loop time: 16.86ms (59 Hz))

```

The current state machine now incorporates force feedback, and this run gave me much better visibility into what is actually happening during insertion.

## What I Changed Today

### A) Added a full PROJECT_CONTEXT.md for future agents

I created a comprehensive context file at:

`mystuff/for_agents/PROJECT_CONTEXT.md`

It is 216 lines and includes:
- competition overview
- pipeline strategy
- repository structure
- `aic_teleop.py` architecture
- scoring system
- trial configurations
- technical details
- practical notes for AI agents

The goal is simple: if any future AI agent jumps into this repo, it should have enough context to contribute effectively without me re-explaining everything from scratch.

### B) Rewrote AICCheatCodeTeleop (v2) in `aic_teleop.py`

I made major upgrades to the cheatcode teleop state machine:

- Added a dedicated **ALIGN phase**
  - Dwell at 5 cm hover height for at least 1 second
  - Require XY error < 3 mm and angular error < 0.05 rad before INSERT
- Slowed insertion descent from **0.07 m/s → 0.02 m/s**
- Added **force-proportional insertion speed control**
  - starts slowing at 5N
  - fully pauses at 15N
  - replaces old binary go/stop behavior
- Improved recovery behavior
  - retreat only to 5 cm (not 20 cm)
  - return to ALIGN (not APPROACH)
  - maximum 3 retries
- Added live XY correction during INSERT by continuously re-reading plug TF
- Tightened phase transitions
  - `dist_to_target < 0.01` (instead of the loose 0.2 value used before in recovery logic)
- Added angular convergence gate before entering INSERT
- Clamped max linear velocity during INSERT to 60% for gentler motion
- Tuned gains:
  - `kp_linear`: 1.0 → 1.2
  - `ki_linear`: 0.15 → 0.2
  - `kp_angular`: 1.5 → 2.0
  - `max_linear_vel`: 0.1 → 0.08

### C) Lowered retreat threshold from 18N to 10N

I originally wrote a recovery mode that retreats at 18N, but looking at the terminal evidence above, the robot can get badly stuck around **10-13N** without ever reaching 18N.

So with the old threshold, it could keep pushing while stuck and just hope for the best.

New behavior chain is now:
- **0-5N**: full speed
- **5-15N**: linear slowdown
- **≥15N**: full stop
- **≥10N for 0.5s**: retreat and retry

This should make the behavior much safer and reduce the chance of force-penalty disasters.

## Side Project Note

I have also been working on a parallel project I started a couple of months ago, but not consistently:

https://discourse.openrobotics.org/t/energy-efficient-autonomous-navigation-benchmarking/53208?u=rocky_shao

## Tooling Note (Slate)

I also found a new coding tool called Slate. A lot of today’s changes came from working through one giant, unorganized prompt with it. I’m definitely not an expert in coding, engineering, or AI-agent workflows yet, but this tool exceeded my expectations in the first hour, so I wanted to share that.

![image|690x359](upload://5fJlptwL6vyZyxbjGxTm0brZSyy.jpeg)

Slate also pointed out that my current cheatcode teleop is using **velocity control** instead of **position control**. I haven’t fully verified that myself yet, so I can’t confirm 100% today, but I’ll dig into it tomorrow.

## Build Thread

Main thread:
https://discourse.openrobotics.org/t/rockys-open-source-build-thread-ai-for-industry-challenge/53155

Overall, today was a solid step forward: better force-aware behavior, cleaner state transitions, and better project context for future iterations.