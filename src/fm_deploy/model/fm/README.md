# `model/fm/` — FM (flow-matching) checkpoints

Drop the FM checkpoint you want to fly here, then pass it to inference:

```bash
cp ~/saved_net/fm/run_<ts>/fm_planner_<ts>.pth ./
ros2 launch fm_planner fm_planning_unknown.launch.py \
    model_path:=$PWD/src/fm_planner/model/fm/fm_planner_<ts>.pth goal_x:=20.0
```

Checkpoint format: `{state_dict, norm_mean, norm_std}` (see `fm_planner/fm_model.py`).
`.onnx` needs its sibling `.onnx.data`. These files are not committed by default
(large); keep them here for a tidy layout or `.gitignore` them per your workflow.
