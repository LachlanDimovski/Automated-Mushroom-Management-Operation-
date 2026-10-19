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

## Running

The GUI starts the simulation itself. Close any Gazebo that is already
running first; Launch refuses to start a second one.

**1. The operator GUI.** Starts the status bridge with it.

```bash
ros2 launch ammo_gui_node gui.launch.py
```

**2. The Debug UI.** Optional, read-only, open and close it whenever.

```bash
ros2 launch ammo_gui_node debug.launch.py
```

River worlds need `bash forest_worlds/setup.sh` once; they only appear in the
world list after that.

> On WSL, if Gazebo dies a few seconds after Launch with an Ogre or
> `GL3PlusTextureGpu` error, untick **Show the Gazebo window** on the setup
> page, or export `LIBGL_ALWAYS_SOFTWARE=1` before launching. The GUI watches
> for that specific crash and says so rather than leaving you with an empty
> mission page.

## What the GUI does

**Setup page.** Pick the world, pick or edit the mushroom layout, choose the
search pattern, choose Autonomous or Manual drive, press Launch. (The capture
tool is always the gripper now, so that choice is gone. The robot counts are
visible but inert - see the note under them.) Launch runs
`cage_mission.launch.py` (Gazebo, Nav2, the z-camera quad,
`rover_capture_node`, `truth_localizer` and `collision_guard_node`) with the
listed mushrooms and starts the clock. Worlds that already contain mushrooms
offer them as the first preset rather than spawning a second set on top.

The preview beside the setup is drawn from the world's own `.meta.json` using
the same widget the live map uses, so a mushroom that would spawn inside a
pond is visible before you press Launch rather than after.

**Search pattern** exposes the quad's own launch arguments: square loops out
from the spawn or lawnmower rows, the spacing between them, cruise speed and
height, whether to keep moving through corners, and whether to plan around
the world file at all. The defaults are the ones the team's launch file uses.

**Mission page.** The clock, mushrooms found and collected counted
separately, natives identified and left alone, how much of the area the
quadcopter has mapped, both robots' states, the map, both camera feeds, and
the mushroom log with a camera still per mushroom.

Controls are Pause/Resume, End Mission, clicking the map to send the rover
somewhere, and hand-driving either robot. Double-click a camera to fill the
window with it; Esc or a second double-click goes back.

Clicking the map suspends the rover's mission node, cancels whatever Nav2 was
doing and drives the rover to your point with its own Nav2 goal. The
quadcopter keeps flying and keeps reporting mushrooms while that happens.
When the rover arrives, its node is resumed and it goes back to the queue. A
second click replaces the first.

**Hand-driving.** Ticking *Armed* on a teleop panel suspends that robot's
mission node first, because two publishers on one velocity topic means the
robot obeys whichever message arrived last. The rover is W/S to drive and A/D
to turn, and goes through `collision_guard` so the guard can still veto
driving into a trunk. The quadcopter adds Q/E to slide sideways and R/F for
up and down, and goes straight to `parrot1/cmd_vel` - there is no guard in
front of it.

## What the Debug UI does

Both camera feeds, large, showing the raw camera with the detector's boxes
drawn over it - red for an invasive amanita, teal for a native - so you can
see the classification live rather than at the detector's 2 Hz. The map, the
robots' states and velocities, the area mapped, the mission timeline at two
minutes a row, and the event feed.

The event feed is where every piece of trouble the rover reports ends up in
plain English: fords crossed, Nav2 legs stalled, places it got boxed in,
spots where the gripper closed on nothing, mushrooms it gave up on. Colour
tells you how much to care, and the filter hides everything that went fine.

It publishes nothing, so opening it mid-run is safe. The events topic is
transient-local, so opening it ten minutes in still draws the timeline from
the start.

## How it gets its data

Both mission nodes publish their own state now - `husky1/rover/state` and
`parrot1/quad_state` - so the GUI knows what the rover is doing and which of
the quadcopter's eleven phases it is in, rather than guessing from whether
the quad happens to be moving. See `integration/NOTES.md` for exactly what
was added to those two packages and why; it is three small additive changes,
and the bridge falls back to reading `/rosout` if they are ever removed.

`mission_bridge` consolidates those, plus `husky1/cage/waypoints`,
`husky1/cage/done`, `husky1/cage/missed`, `parrot1/ground_obstacles`,
odometry and `parrot1/detections`, into `ammo/mission/status` and
`ammo/mission/events`. Both windows read only those two topics.

## The map

Everything static is drawn from the world's `.meta.json` - the same file the
mission nodes read. Trees, fallen trees, logs, stumps, rocks, debris, ponds
and cleared lanes on a forest world; the river at its real width and the ford
lanes across it on a river world. The key lists only what the loaded world
actually has, so a forest run does not advertise fords.

On top of that go the live things: mushrooms found, collected, given up on
and grabbed-but-empty; natives left in place; where Nav2 gave up; where the
rover got boxed in; which fords it crossed; and the leg it is currently
driving.

## What is real, and what is a first pass

- Map, poses, trails, cameras, clock, counts, the mushroom log, the event
  feed, the timeline, coverage, both teleop panels and map goals are all
  wired to live data and work as described.
- **Pause** cancels the Nav2 goal, publishes zero velocity to both robots at
  20 Hz, and suspends the mission nodes. That order matters: both robots use
  Gazebo's `VelocityControl` plugin, which holds its last command forever, so
  stopping the nodes alone would leave them coasting. On resume the rover
  re-plans, and because its own timers were frozen, a long pause can make it
  give up on the waypoint it was on and re-queue it.
- **Ending a mission closes the simulation.** Keeping Gazebo up between runs
  needs a mission-only launch path, which this package does not have any
  more; the clean way to add it back is to load `cage_mission.launch.py` as a
  module and reuse its own node factory, the way `zcam_mission.launch.py`
  does, so the node set cannot drift.
- **Snapshots** in the mushroom log are held in memory only. Closing the
  window loses them.
- **Native mushrooms** are wired end to end, but whether any appear depends
  on the weights the detector loads. The event feed says which classes it
  came up with, so if `native_mushroom` is not in that line the counter will
  sit at zero and that is the reason.
- No pan or zoom on the map - it auto-fits to the robots and the mushrooms
  and grows to keep them on screen. It deliberately does not fit the hedge
  boundary, which would shrink everything that matters to a few pixels.

## Testing

```bash
python3 src/RS1_Team5_/ammo_gui_node/tests/run_tests.py
python3 src/RS1_Team5_/ammo_gui_node/tests/run_tests.py --render
```

218 checks, a few seconds, no ROS, no simulation, no display and no built
workspace - `tests/stubs/` stands in for rclpy and the message types. The
`--render` flag also writes pictures of both windows and the map into
`tests/out/`, which is the quickest way to see what a change did to a layout.

One of those suites, `test_log_lines.py`, checks the lines the bridge reads
are still present in the mission nodes' source. If a teammate rewords one,
that is where you find out.
