# Apex Overfit — Autonomous Racing Driver for TORCS

**Competition:** IBM AI Racing League 2026 · **Language:** Python 3 · **Simulator:** TORCS

This repository holds the code for the self-driving agent we built for the
**IBM AI Racing League 2026**. The agent pilots a car inside TORCS (The Open
Racing Car Simulator), reading live sensor data and converting it into steering,
throttle, braking and gear commands in real time — chasing the fastest possible
lap while keeping the car clean on track.

---

## Overview

The challenge was to write a program that drives a race car on its own, with no
human input, by responding tick-by-tick to what the car "feels": its speed, its
heading relative to the road, and how far the track edges are in every sensor
direction.

### Design philosophy

Instead of a heavy learned model, we deliberately went for a **transparent,
sensor-reactive controller** — every command is recomputed from scratch each
frame, with no stored map and no memory between laps. The logic rests on a few
simple ideas:

- **Look-ahead = speed.** The forward range sensors tell us how far the road is
  open ahead. Lots of clear road → it's a straight, push hard. A short reading →
  a corner is near, ease the target speed down.
- **Smooth throttle.** Acceleration is handled by a PD controller on the speed
  error, so power comes on progressively instead of stabbing the pedal.
- **Stable steering.** A PD law blends heading correction, track-centering and
  lateral-drift damping, softened at high speed to kill nervous twitching.
- **Braking that cooperates with cornering.** Brake force scales with how far
  over the safe speed we are, and bleeds off as the wheel turns in, so the tyres
  aren't asked to brake and corner at their limit simultaneously.

We started from a cautious version with a conservative speed ceiling, then
gradually tightened the steering, throttle and braking response until the car
ran smoothly and posted a competitive qualifying time.

### Result

On the **Corkscrew** circuit our best standing-start lap was **1:26.84**.

---

## Requirements

* **Python 3**
* **TORCS** (The Open Racing Car Simulator)
* The **SCR** (Simulated Car Racing) server patch installed in the TORCS
  directory

---

## Setup

1. **Get the code:**
   ```
   git clone https://github.com/<your-account>/apex-overfit
   ```

2. **Configure TORCS:**
   * Open TORCS.
   * Go to `Race` → `Practice` → `Configure Race`.
   * Add a single driver — `scr_server 1` — and pick your track.
   * Start with `New Race`. TORCS will hold on a waiting screen, listening for a
     UDP connection on port `3001`.

---

## Running the driver

Once TORCS is sitting at the starting line, start the controller:

```
python torcs_jm_par.py
```

To capture per-tick telemetry (useful for tuning), set the log flag:

```
TORCS_LOG=1 python torcs_jm_par.py
```

Run `python torcs_jm_par.py -h` for the full list of options (port, host, track
tag, etc.).

---

## Repository contents

| File | Role |
|------|------|
| `torcs_jm_par.py` | Our driver — all the steering, speed and gear logic lives here. |
| `snakeoil3_gym.py` | UDP client handling the TORCS connection, adapted for the Gym setup. |
| `snakeoil3_jm2.py` | A second UDP client variant, used for a direct script-to-simulator link. |
| `gym_torcs.py` | Gym-style environment wrapper sitting on top of TORCS. |
| `autostart.sh` | Helper script that launches and drives the TORCS GUI. |

All tunable constants for the driver are gathered in a single configuration
block at the top of `torcs_jm_par.py`, so behaviour can be adjusted without
touching the control logic.

---

## Use of IBM Granite

Following the competition rules, we used the **IBM Granite** models as an
assistant during development. Concretely, Granite helped us with:

* **Scaffolding** — outlining an initial class layout and the shape of the main
  driving loop.
* **Debugging & refactoring** — tracking down TORCS UDP communication issues and
  cleaning up the code for clarity and performance.
* **Documentation** — phrasing code comments and shaping this README.

---

## Hotlap

A video of our fastest lap will be added here soon.

---

## Team Apex Overfit

* Artsiom Karzhaneuski
* Nazar Tkachuk
* Dmytrii Mryts
* Roman Dovhal
* Dmytro Golubtsov
