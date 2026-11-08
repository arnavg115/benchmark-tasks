# Turn a dial to match a reference dial

Instruction given to the policy: `Turn the blue knob so it points the same way as the black pointer on the other dial.`

Files in this folder:

| File | What it is |
|---|---|
| `task.json` | The task's details |
| `dial_match.py` | Scene, success, partial credit, the scripted demo and the scoring self-test |
| `geometry.py` | Dimensions shared by the task file and the USD generator |
| `make_dial_usda.py` | Writes `assets/dial.usda` |
| `assets/dial.usda` | The turnable dial: an articulation with one revolute joint (generated; do not edit by hand) |
| `scenekit.py` | Loads USD articulations into yam_arena and reports their joint positions to the scorer. Copied unchanged from `tasks/lock-before-loading/scenekit.py` |

> **Status: draft, not yet run on the simulator.** The scoring self-test passes offline, and `assets/dial.usda`
> parses with OpenUSD (usd-core 26.8) with the intended prims, joint and limits. Whether Isaac Sim loads the hand-built
> articulation, the demo and every number marked TODO below still need a run on the program's launchable before this
> is submitted.

## Environment

Built for the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots: TODO (`inspect-robots --version`)
- yam-arena commit: TODO (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream: TODO, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: TODO (`sha256sum dial_match.py | cut -c1-12`; should equal `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`.

## Regenerate the dial

```bash
python tasks/dial-match/make_dial_usda.py
```

This rewrites `assets/dial.usda` from `geometry.py`. The file follows the structure of the hinged platform in
`tasks/lock-before-loading/assets`:

- a base link carrying the articulation root, pinned to the world by a fixed joint;
- a knob link on a revolute joint about z, with limits of 0 to 270 degrees;
- `Cylinder` and `Cube` primitives with `displayColor`, as in that task's `pin.usda`;
- ticks and the pointer tip as visual geometry only.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable:

```bash
python tasks/dial-match/dial_match.py
```

It asserts the scoring rules on 12 knob angles and checks the geometry, then replays the left arm's plan through the
kinematics at 20 Hz, checking that every joint stays inside the URDF limits with an IK error under 0.5 mm.

The angles cover:

- the start;
- exactly on target;
- each band's edges: 4.9 and 5 degrees off (1.0), 6 and 15 off (0.5), 30 off (0.25), 31 off (0);
- snapping to either neighbouring tick, 90 or 135 degrees (0.25);
- the 270 degree stop (0);
- 230 degrees (0).

The geometry checks confirm that the target is inside the knob's range while its end-for-end reading is not, that the
plates don't overlap, and that the bar clears the ticks.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/dial-match/dial_match.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the blue knob so it points the same way as the black pointer on the other dial." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dial-match-dream
```

Scoring is automatic. `task_score` reports 1.0 when `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`. Otherwise it reports `score()` at the end of the episode, as defined in `task.json`:
1.0, 0.5 or 0.25 within 5, 15 or 30 degrees of the reference, otherwise 0.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/dial-match/dial_match.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the blue knob so it points the same way as the black pointer on the other dial." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dial-match-demo
```

The demo is `demo()` in the task file. It is open loop and deterministic, and uses the left arm only:

1. The arm grips the knob across its 12 mm bar, with the TCP over the hinge and the pad tips 4 mm up the bar.
2. It turns the wrist from −57.5 to +57.5 degrees over 5 s.
3. It lets go and lifts.

The plan is 13.6 s (about 270 steps) plus the 1 s hold. `AIM_DEG` adds extra wrist turn if the knob lands short;
tune it on the stack. Result: TODO (`task_score`, steps, termination reason).

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s of simulated time), `--max-steps 1200`.
- Runtime per trial on an L40S: TODO.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout, and the open-loop demo needs it.
  Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible.

## Submitted videos and scores

TODO: log names, trials, scores and termination reasons for the Dream-1 run and the demo. The setup screenshot should
be frame 0 of the top camera at 640 x 360, so that the ticks and red tips are legible.

## Reset between trials

Nothing is consumed. Turn the knob back against its 0 degree stop.
