# Interchangeable detector weights

Two YOLO11n models for the same single class, `amanita_muscaria_red`. They are
drop-in replacements for each other: identical architecture, identical class
list, identical input size. Only the training data differs.

`../amanita_muscaria_yolo11n.pt` is the one the node loads by default, and is a
copy of `amanita_sim_yolo11n.pt`.

| | `amanita_realphoto_yolo11n.pt` | `amanita_sim_yolo11n.pt` |
|---|---|---|
| Trained on | 876 iNaturalist photographs | 400 Gazebo frames |
| Labels | machine-generated (YOLO-World + colour filter) | exact, projected from known 3D poses |
| mAP50 (own val set) | 0.876 | 0.972 |
| mAP50-95 | 0.720 | 0.736 |
| Precision / recall | 0.765 / 0.839 | 0.990 / 0.950 |
| Mean confidence on sim frames | ~0.16 | ~0.83 |
| Best for | real mushrooms, photographs | this simulation |

The sim model is a fine-tune *of* the real-photo model, not a separate training
run - it starts from those weights and adapts them to rendered imagery.

## Switching without rebuilding

The node takes the weights path as a ROS parameter:

    ros2 run skidsteer_perception yolo_detector --ros-args \
        -p model_path:=$(ros2 pkg prefix skidsteer_perception)/share/skidsteer_perception/models/variants/amanita_realphoto_yolo11n.pt

Or make one the default by copying it over `../amanita_muscaria_yolo11n.pt` and
running `colcon build --packages-select skidsteer_perception`.

## Why both are kept

The real-photo model scores 0.876 on photographs but collapses to ~0.16 on
Gazebo renders - it fires on grass and tree trunks at any threshold low enough
to detect a mushroom at all. That gap is the reason the sim model exists, and
the pair is the evidence for it: same architecture, same class, same code path,
an order of magnitude apart on sim imagery purely because of training data.

Keep the real-photo weights if the detector is ever pointed at a real camera.
