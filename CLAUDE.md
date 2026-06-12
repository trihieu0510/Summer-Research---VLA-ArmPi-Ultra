# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## The link for the manual of this project
https://wiki.hiwonder.com/projects/ArmPi-Ultra/en/latest/index.html

## What this project is

A summer research project building **voice-controlled robotic manipulation** on a **Hiwonder ArmPi Ultra** arm (**Raspberry Pi 5 with 16GB RAM** + STM32, 6 servos, Aurora RGB-D depth camera, WonderEcho Pro mic). The Pi has **no GPU** — the binding compute limit is CPU/thermal (4 cores), not RAM. The goal: *speak a command → robot perceives the scene → robot executes the task.*

The chosen architecture is a **modular pipeline** (not an end-to-end VLA), staged as:
`Speech-to-Text → Scene perception (YOLOv8 + depth + hand-eye calibration) → LLM reasoning (→ JSON action) → Inverse kinematics → Servo motion → Vision-based verification`

`Documents/ArmPi_Ultra_VLA_Roadmap.pdf` is the authoritative 6-phase plan (Phase 0 prerequisites → Phase 5 evaluation, Phase 6 stretch = SmolVLA fine-tuning). `Documents/Research Proposal.pdf` is the formal proposal. Read the roadmap before making architectural decisions — phases have explicit exit criteria and are meant to be done in order.

## Two-machine workflow (critical)

This repo is the **laptop side** (Windows). Code is edited here and run on the **Pi**, which runs ROS 2 Humble inside a **Docker container**. The two are connected by a LAN cable using **Windows ICS** — the Pi gets an address in `192.168.137.x` while the laptop keeps WiFi for internet. SSH is the primary control channel.

- `voice_arm_control.py` and any `rclpy`/ROS-message code **only runs on the Pi**, never on Windows. The `# pyrefly: ignore [missing-import]` comments exist because the ROS packages aren't installed on the laptop.
- `grayscale.py` is the only file meant to run locally on Windows (a Phase 0 OpenCV warm-up).

## Running things on the Pi

The Hiwonder stack auto-starts on boot and grabs the STM32 serial port. **Always free it before launching anything:**

```bash
~/.stop_ros.sh            # releases the STM32 serial port from the autostart
```

Then launch:

```bash
# Bare hardware (servos move, no AI) — use this to test motion:
ros2 launch sdk armpi_ultra.launch.py

# Full voice pipeline:
ros2 launch large_models llm_control_servo.launch.py
```

ROS 2 environment is sourced permanently in the Pi's `~/.bashrc` (Humble + all Hiwonder vars). **`need_compile=False` is required — setting it to `True` crashes every launch** with a `'need_compile'` KeyError. API keys for the bundled pipeline live in `~/ros2_ws/src/large_models/large_models/config.py`.

Common ROS 2 introspection: `ros2 node list`, `ros2 topic list`, `ros2 topic echo <topic>`, `ros2 topic hz <topic>`, `ros2 service list`.

## Servo command reference

Servos use raw units **0–1000 where 500 is center**; each has hard mechanical limits — test new motions slowly with small ranges. Two command paths appear in this repo:

- `main.py` — a `ros2 topic pub` one-liner against `/servo_controller` (`servo_controller_msgs/ServosPosition`).
- `voice_arm_control.py` — publishes `ros_robot_controller_msgs/ServosPosition` to `/ros_robot_controller/bus_servo/set_position`.

Confirm the live topic/message type with `ros2 topic list` / `ros2 interface show` before assuming which applies — the correct one depends on which launch file is running.

## Key file: voice_arm_control.py

A ROS 2 node (`VoiceArmController`) that subscribes to `/voice_words`, sends the transcript to a **local Qwen LLM via Ollama** (`http://localhost:11434/v1`, model `qwen:0.5b`, `temperature=0.0`), and maps the returned JSON `{"action": ...}` to fixed servo poses (`home`/`left`/`right`/`open`/`close`/`nod`). Note this is a local-LLM substitution for the roadmap's suggested cloud reasoning stage (GPT-4o-mini / Claude). Comments and log strings are in Vietnamese.

## Conventions

- One virtualenv per machine/phase; `venv/` and `armpi/` (both Windows Python 3.13) are laptop-side warm-up envs and are **not** the Pi's runtime — never reason about Pi dependencies from them. Never `sudo pip install` globally on the Pi.
- `Daily Update.txt` is the running work journal (the roadmap's `JOURNAL.md` practice). Add a dated entry when you make meaningful progress.
- Git: remote is `origin` on GitHub (`trihieu0510/Summer-Research`), default branch `master`. Commit per working increment.
