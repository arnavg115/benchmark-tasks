# Give exact change from a coin till

Instruction given to the policy: `The customer is owed 35 cents. Take exactly 35 cents from the till and put it in the change tray.`

Files in this folder: `task.json`, `make_change.py` (scene, success, partial credit, the scripted demo and the scoring self-test), `README.md`.

> **Status: draft, not yet run on the simulator.** The scoring self-test passes offline. The scene, the demo and
> every number marked TODO below still need a run on the program's launchable before this is submitted.

## Environment

Built for the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots: TODO (`inspect-robots --version`)
- yam-arena commit: TODO (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream: TODO, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: TODO (`sha256sum make_change.py | cut -c1-12`; should equal `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable:

```bash
python tasks/make-change/make_change.py
```

It asserts the scoring rules on 21 hand-built states and replays the left arm's plan through the kinematics at 20 Hz,
checking that every joint stays inside the URDF limits with an IK error under 0.5 mm. The states cover:

- **Full score:** the start (0); a quarter and a dime (1.0); a quarter and two nickels (1.0).
- **Overpaying (0):** one coin of each kind (40 cents); all six coins (80 cents); two quarters (50 cents).
- **Under the target:** 25 cents as one quarter or as two dimes and a nickel (0.36); 30 cents (0.43).
- **Loose coins:** 35 cents with a nickel loose on the counter or off the table (0.75).
- **Coins that still count:** a coin upside down, stacked, or on edge leaning on the tray wall; a nickel moved to
  another till compartment.
- **Not counted:** a coin still sliding (scores, but is not success); a coin held 30 mm above the tray, lying on the
  tray rim, just outside the tray floor, or on a till divider.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/make-change/make_change.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The customer is owed 35 cents. Take exactly 35 cents from the till and put it in the change tray." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/make-change-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`. Otherwise it reports the task file's `score()` at the end of the episode, as
defined in `task.json`. The rules are:

- over 35 cents in the tray: 0;
- exactly 35 cents with no coin loose: 1.0;
- exactly 35 cents with a coin loose: 0.75;
- under 35 cents: 0.5 × cents / 35.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/make-change/make_change.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The customer is owed 35 cents. Take exactly 35 cents from the till and put it in the change tray." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/make-change-demo
```

The demo is `demo()` in the task file. It is open loop and deterministic: the left arm takes the inboard quarter and
then the inboard dime, and drops each into the tray from 3 mm above the rim. Each pick works like this:

1. The fingers open to the coin's diameter plus 10 mm.
2. The arm descends until the pad tips are 0.4 mm above the table.
3. The fingers close on the coin's rim.

The plan is 25.8 s (about 516 steps) plus the 1 s hold. Result: TODO (`task_score`, steps, termination reason).

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400`.
- Runtime per trial on an L40S: TODO.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout, and the open-loop demo needs it.
  Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible.

## Submitted videos and scores

TODO: log names, trials, scores and termination reasons for the Dream-1 run and the demo. The setup screenshot should
be frame 0 of the top camera at 640 x 360, so that the coins are legible.

## Reset between trials

Nothing is consumed. Lay the six coins back flat at their marks in the till; the till and tray do not move.
