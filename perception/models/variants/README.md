# Detector weights

The demo uses `amanita_river_v6_yolo11n.pt`, which has two classes: `amanita_muscaria_red` and
`native_mushroom`. The other files record how the model developed. All are YOLO11n with the same input size, so they
swap without code changes, but a one-class model detects no natives.

`../amanita_muscaria_yolo11n.pt` is what `yolo_node` loads with no `model_path`. It is a one-class copy of
`amanita_sim_yolo11n.pt`. The demo launches pass `amanita_river_v6_yolo11n.pt` instead.

Metrics are stored in each checkpoint and come from that model's own validation set, so they do not compare across rows.

| Model | Trained on | Classes | mAP50 | mAP50-95 | Why it was replaced |
|---|---|---|---|---|---|
| `amanita_realphoto` | iNaturalist photographs, machine-labelled | 1 | 0.858 | 0.723 | Mean confidence about 0.16 on rendered frames |
| `amanita_sim` | Gazebo frames with projected labels | 1 | 0.971 | 0.768 | Short range; poor at quad viewing angles |
| `amanita_sim_v2` | Ground and aerial capture, merged | 1 | 0.960 | 0.851 | Superseded by a larger capture |
| `amanita_sim_v3` | Enlarged capture, ground and aerial | 1 | 0.973 | 0.895 | Overfitted to a green background |
| `amanita_sim_v3_closeup` | Close-up frames at grasping range | 1 | 0.970 | 0.903 | One class; the river world needed natives |
| `amanita_river_v4` | River dataset with brown native caps | 2 | 0.717 | 0.520 | Brown caps matched the leaf litter |
| `amanita_river_v6` | Blue native caps at 1 to 5 m, river banks included | 2 | 0.985 | 0.799 | Final model |

## Switching without rebuilding

The node takes the weights path as a ROS parameter:

    ros2 run perception yolo_node --ros-args \
        -p model_path:=$(ros2 pkg prefix perception)/share/perception/models/variants/amanita_realphoto_yolo11n.pt

Or make one the default by copying it over `../amanita_muscaria_yolo11n.pt` and
running `colcon build --packages-select perception`.

## Why the real-photo model is kept

It scores 0.858 on photographs but collapses to about 0.16 confidence on Gazebo renders, firing on grass and tree
trunks. That gap is the reason the simulated training data exists. Keep these weights if the detector is ever pointed
at a real camera.
