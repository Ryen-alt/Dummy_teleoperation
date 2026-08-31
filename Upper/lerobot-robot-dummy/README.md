# LeRobot adapter for Dummy

This package is the only layer allowed to depend on LeRobot. The control and
recording package writes lossless Raw Session schema v4 data; this package verifies
and converts accepted episodes offline. `lerobot==0.4.0` is intentionally pinned so
an upstream dataset API change cannot affect robot control or collection.

Install both local packages in a dedicated export environment:

```bash
python -m venv .venv-export
source .venv-export/bin/activate
pip install -e ../dummy-host -e .
```

Export locally (this command does not upload anything):

```bash
dummy-export-lerobot-v3 \
  --session ../dummy-host/sessions/<session-id> \
  --recipe configs/export_recipe.example.yaml \
  --repo-id my-user/dummy-pick-v1 \
  --output datasets/dummy-pick-v1
```

The adapter infers camera feature shapes from the first validated frame, uses
`applied_action` as the training label only after ACK, exact seven-node CAN
queueing and post-command coherent feedback are all present. It preserves raw
sample/tick/control-time provenance and writes `dummy_export_metadata.json` with
source config hashes and episode IDs.

`constraints-lerobot.txt` pins the API boundary. Generate the full transitive
lock in the target Linux/CUDA environment, because Torch/video wheels are
platform-specific; never reuse the robot control environment for training.
