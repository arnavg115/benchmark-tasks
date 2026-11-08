# Dispose of lab waste into the right containers

Instruction given to the policy: `Dispose of everything on the bench. Sharps go in the red sharps container, anything else that touched the culture goes in the orange biohazard bin, and the clean cardboard goes in the grey trash.`

Files in this folder: `task.json`, `lab_waste_sort.py` (scene, success, partial credit, the scripted demo and the scoring self-test), `README.md`.

> **Status: draft, not yet run on the simulator.** The scoring self-test passes offline. The scene, the demo and
> every number marked TODO below still need a run on the program's launchable before this is submitted.

## Environment

Built for the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots: TODO (`inspect-robots --version`)
- yam-arena commit: TODO (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream: TODO, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: TODO (`sha256sum lab_waste_sort.py | cut -c1-12`; should equal `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable:

```bash
python tasks/lab-waste-sort/lab_waste_sort.py
```

It asserts the scoring rules on 22 hand-built states, checks the geometry and replays both arms' plans through the
kinematics at 20 Hz, checking that every joint stays inside the URDF limits with an IK error under 0.5 mm.

The scoring states cover:

- **Full and partial credit:** all six correct (1.0); only the sharps done (2/6); the left row done (3/6).
- **Strategies that score 0:**
  - taking each item to its arm's nearest container;
  - everything into any one container;
  - a sharp in the biohazard bin or the trash;
  - the conical tube in the trash;
  - a sharp or culture item off the table.
- **Penalties:** an item in the wrong but harmless container costs a point, for example cardboard in the biohazard
  bin or the dish in the sharps container (4/6).
- **Not counted:** the scalpel lying on the sharps lid, the syringe left between containers, a tube still held above
  a rim, a tube just outside a bin floor.

The geometry checks confirm that both sharps fit through the slot, that the containers don't overlap and that every
drop point lands inside its container.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/lab-waste-sort/lab_waste_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Dispose of everything on the bench. Sharps go in the red sharps container, anything else that touched the culture goes in the orange biohazard bin, and the clean cardboard goes in the grey trash." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/lab-waste-sort-dream
```

Scoring is automatic. `task_score` reports 1.0 when `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`. Otherwise it reports `score()` at the end of the episode, as defined in `task.json`:
0 on a hard failure, otherwise max(0, correct − misplaced) / 6.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/lab-waste-sort/lab_waste_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Dispose of everything on the bench. Sharps go in the red sharps container, anything else that touched the culture goes in the orange biohazard bin, and the clean cardboard goes in the grey trash." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/lab-waste-sort-demo
```

The demo is `demo()` in the task file. It is open loop and deterministic, and the arms take turns so they never
share the midline.

1. **Left arm:** drops the syringe and the scalpel through the sharps slot and the culture dish into the biohazard
   bin, then parks over its own side.
2. **Right arm:** waits parked until then, drops the cardboard tube and box into the trash and the conical tube into
   the biohazard bin.

Each item is picked top-down, carried at 230 mm above the table and released with its bottom 10 mm above the
opening it must clear. The plan is 81.9 s (about 1,640 steps) plus the 1 s hold. Result: TODO (`task_score`, steps,
termination reason).

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400`.
- Runtime per trial on an L40S: TODO.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout, and the open-loop demo needs it.
  Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible.

## Submitted videos and scores

TODO: log names, trials, scores and termination reasons for the Dream-1 run and the demo. The setup screenshot should
be frame 0 of the top camera at 640 x 360.

## Reset between trials

Nothing is consumed. Take the six items out of the containers and set them back on their marks; the containers do not move.
