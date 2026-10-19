# tests

Headless checks for the GUI, the Debug UI and the status bridge. No ROS, no
simulation, no display, no built workspace.

```bash
python3 src/RS1_Team5_/ammo_gui_node/tests/run_tests.py
python3 src/RS1_Team5_/ammo_gui_node/tests/run_tests.py --render
```

`stubs/` holds a small stand-in for `rclpy` and the message types: nodes
record what they publish instead of talking to a middleware, and a test
delivers messages by calling `node.feed(topic, msg)` and steps time with
`node.fire_timers()`. That is what makes it possible to assert on a whole
mission in a couple of seconds.

| file | what it covers |
|---|---|
| `test_log_lines.py` | the lines the bridge reads are still written by the mission nodes, and the state publishers are still there |
| `test_bridge.py` | every log line and topic the bridge consumes, and the status it produces |
| `test_windows.py` | the timeline, the event feed, the map and its key, the camera panels, both teleop panels, the setup page, the operator page, the Debug UI, and the launch command the setup page builds |
| `render_map.py` | draws the map against both world-meta shapes |
| `render_windows.py` | draws the Debug UI mid-mission |

`test_log_lines.py` is the one that will fail first when somebody else
changes their node. It does not run anything - it looks in
`ammo_mission_control/` and `quad_zcams/` for the fixed part of each line the
bridge matches, and tells you which one has gone.

Pictures from `--render` land in `tests/out/`, which is not in git.
