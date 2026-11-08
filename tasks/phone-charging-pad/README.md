# Put a phone on a cluttered charging pad

Instruction given to the policy: `Put the phone on the charging pad so it charges.`

Files in this folder: `task.json`, `phone_charging_pad.py` (scene, success, partial credit, the scripted demo and the scoring self-test), `README.md`.

> **Status: draft, not yet run on the simulator.** The scoring self-test passes offline. The scene, the demo and
> every number marked TODO below still need a run on the program's launchable before this is submitted.

## Environment

Built for the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots: TODO (`inspect-robots --version`)
- yam-arena commit: TODO (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream: TODO, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: TODO (`sha256sum phone_charging_pad.py | cut -c1-12`; should equal `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable:

```bash
python tasks/phone-charging-pad/phone_charging_pad.py
```

It asserts the scoring rules on 19 hand-built states, checks the layout and replays the left arm's plan through the
kinematics at 20 Hz, checking that every joint stays inside the URDF limits with an IK error under 0.5 mm.

The scoring states cover:

- **Charging (1.0):** the phone centred, turned 90 degrees, or 9 mm off centre.
- **Partial credit:**
  - 15 mm off centre (0.5);
  - 30 mm off centre, or tilted 6 degrees on the pad (0.25).
- **Scores 0:**
  - **The keys still on the pad:** the phone laid on top of them, the keys pushed to the pad's edge, or the keys on
    top of the centred phone.
  - **Something off the table:** the keys swept off, or the phone.
  - **The phone not on the pad:** screen down, flat on the table beside it, or held 20 mm above it.
  - **Only the keys moved.**
- **Still full score:** the keys just past the pad's rim on the table.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/phone-charging-pad/phone_charging_pad.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the phone on the charging pad so it charges." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/phone-charging-pad-dream
```

Scoring is automatic. `task_score` reports 1.0 when `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`. Otherwise it reports `score()` at the end of the episode, as defined in `task.json`:
0 if the keys are over the pad or anything is off the table, otherwise 1.0, 0.5, 0.25 or 0 by where the phone lies.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/phone-charging-pad/phone_charging_pad.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the phone on the charging pad so it charges." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/phone-charging-pad-demo
```

The demo is `demo()` in the task file. It is open loop and deterministic, and uses the left arm only:

1. The arm picks the keys off the pad across their 25 mm width and sets them on the table 25 mm beyond the pad's far
   edge.
2. It picks the phone across its 72 mm width, with the gripper fully open (about 94 mm), and sets it on the pad's
   centre from 3 mm above.

The plan is 28.4 s (about 570 steps) plus the 1 s hold. Result: TODO (`task_score`, steps, termination reason).

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s of simulated time), `--max-steps 1200`.
- Runtime per trial on an L40S: TODO.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout, and the open-loop demo needs it.
  Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible.

## Submitted videos and scores

TODO: log names, trials, scores and termination reasons for the Dream-1 run and the demo. The setup screenshot should
be frame 0 of the top camera at 640 x 360.

## Reset between trials

Nothing is consumed. Lay the phone back screen up at its mark and the keys back on the pad's centre; the pad does not move.
