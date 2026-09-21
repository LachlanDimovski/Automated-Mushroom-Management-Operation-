# ammo_gui_node

Danil's operator GUI and Debug UI. Two PySide6 windows plus a small bridge
node, all in this package. Separate from `ammo_dashboard` (Lachlan's OpenCV
HUD) - nothing here touches it, and both can run at the same time.

## One-time setup, per machine

```bash
pip3 install PySide6
sudo apt install libxcb-cursor0        # Qt 6.5+ needs this for the xcb plugin
```

That is the whole dependency list. No OpenCV: ROS images are converted to Qt
images with numpy (`image_utils.py`).

## Running - three terminals

**1. The simulation.** Bring this up once and leave it running. Nothing
below ever restarts it, so setting up another run does not cost a Gazebo
reload.

```bash
ros2 launch 41068_ignition_bringup forest_world.launch.py \
    world:=simple_trees parrot:=true nav2:=true husky_yolo:=true \
    ground_truth_localization:=true
```

**2. The operator GUI.** Starts the status bridge with it.

```bash
ros2 launch ammo_gui_node gui.launch.py
```

**3. The Debug UI.** Optional, read-only, open and close it whenever.

```bash
ros2 launch ammo_gui_node debug.launch.py
```

The mission nodes are *not* in any of those. The GUI starts them itself when
you press Launch, so it can pause and resume them individually.

## What the GUI does

Setup page: pick the world (must match terminal 1), pick or edit the
mushroom layout, choose Autonomous or Manual drive, press Launch. Launch
spawns the mushrooms, starts `quad_search_node`, `cage_capture_node`,
`truth_localizer` and `collision_guard_node`, and starts the clock.

Mission page: elapsed clock, mushrooms found and collected counted
separately, both robots' states, the map, and the mushroom log with a
camera frame from each robot per mushroom. Controls are Pause/Resume, End
Mission, and clicking the map to send the rover somewhere.

Clicking the map suspends the rover's mission node, cancels whatever Nav2
was doing and drives the rover to your point with its own Nav2 goal. The
quadcopter keeps flying and keeps reporting mushrooms while that happens.
When the rover arrives, its node is resumed and it goes back to the queue,
planning a fresh route from wherever it ended up. A second click replaces
the first.

## What the Debug UI does

Both live camera feeds, the same map widget the GUI uses, each robot's state
and velocity, and the mission timeline - one grey bar per minute, filled
left to right with a colour per state, wrapping to a new row every minute,
with markers where mushrooms were found, collected, or where you sent the
rover. It publishes nothing, so opening it mid-run is safe.

The timeline needs the bridge, which `gui.launch.py` starts. To watch a
mission somebody started without the GUI, use `debug.launch.py bridge:=true`
- but never with the GUI running, or two bridges publish the same topic.

## How it gets its data

`mission_bridge` reconstructs the mission from things already on the wire -
`/rosout`, `husky1/cage/waypoints`, `husky1/cage/done`, odometry,
`parrot1/detections` - and republishes it as `ammo/mission/status` and
`ammo/mission/events`. Both windows read only those two topics.

That was a deliberate choice: **no file outside this package was changed.**
`cage_capture_node` keeps a state but only logs it, and `quad_search_node`
has no state variable at all, so rather than edit either of them the bridge
reads their logs. See `integration/NOTES.md` for the small additions that
would replace that with something cleaner, and for the log lines the bridge
currently depends on.

## What is real, and what is a first pass

- Map, poses, trails, cameras, clock, found/collected counts, the mushroom
  log, the timeline, manual driving and map goals are all wired to live data
  and work as described.
- **Pause** cancels the Nav2 goal, publishes zero velocity to both robots at
  20 Hz, and suspends the mission nodes. That order matters: both robots use
  Gazebo's `VelocityControl` plugin, which holds its last command forever, so
  stopping the nodes alone would leave them coasting. On resume the rover's
  Nav2 goal was cancelled, so it re-plans - and because its own timers were
  frozen, a long pause can make it give up on the waypoint it was on and
  re-queue it. Harmless, but you will see it in the log.
- **Mushroom count** is what the quadcopter reported, de-duplicated within
  1 m to match `cage_capture_node`'s own merge distance. "Collected" comes
  from its `Collected N mushroom(s)` log line, so a grab that closes on
  nothing is not counted.
- **Snapshots** are held in memory only. Closing the window loses them.
- **Leftover mushrooms** are removed when you End Mission, via the same
  Ignition service `cage_capture_node` uses. If that fails they stay in the
  world and the next run will find them again.
- No pan or zoom on the map - it auto-fits and grows to keep both robots on
  screen.

## Testing

`tests/test_all.py` in this repo's sibling build folder runs the bridge
logic, both windows and the timeline headless against stub ROS messages.
It is not part of the colcon build.
