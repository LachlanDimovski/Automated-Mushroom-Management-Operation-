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
running first; Prepare refuses to start a second one.

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

> On WSL, if Gazebo dies a few seconds after Prepare with an Ogre or
> `GL3PlusTextureGpu` error, untick **Show the Gazebo window** on the setup
> page, or export `LIBGL_ALWAYS_SOFTWARE=1` before launching. The GUI watches
> for that specific crash and says so rather than leaving you with an empty
> mission page.

## Starting a run: Prepare, then Launch

A run starts in two stages, and almost everything awkward about the old
one-button Launch goes away as a result.

**Prepare** runs `cage_mission.launch.py` - Gazebo, Nav2, the z-camera quad,
`rover_capture_node`, `truth_localizer`, `collision_guard_node` and both YOLO
instances - or `fleet_mission.launch.py` if you asked for more than one of
either robot. **Every robot is held still from the instant Prepare is
pressed**, before a single node exists, so there is no window in which the
quadcopter can take off. The checklist fills in behind that, and when the
last piece arrives the mushrooms are spawned and the world is photographed
from directly above.

Holding uses the robots' own **`teleop_active` stand-down**, the same
heartbeat the PS4 controller sends: while it keeps arriving,
`rover_capture_node` and `zcam_quad_node` stop commanding their robot
entirely and, when it stops, re-plan from where they were left. That is much
better than stopping their processes - a standing-down node keeps its timers,
its clock and its Nav2 connection, and the quad goes on reporting sightings
the whole time. The GUI publishes zero velocity alongside the heartbeat,
because Gazebo's VelocityControl keeps the last command it was given forever.

**Launch** thaws those two processes and starts the clock.

Three things follow from the split, and they are the reason for it:

- The mission clock starts when the mission does. It used to start when the
  launch command returned and spend its first minute on Gazebo's start-up.
- The quadcopter has never seen the mushrooms when the run begins, because
  they are spawned after it is frozen. So you can look at where they landed,
  shuffle them, and look again, without giving anything away.
- A run that will not work says so before you have watched it for a minute.
  The checklist names what has not arrived, and Nav2 or a detector that never
  comes up is visible rather than looking like a robot that does nothing.

**Ending a mission stops everything**, including Gazebo, and then kills by
name anything that outlived its process group - Ignition re-parents its own
server and GUI, which is how a closed Gazebo window went on feeding camera
panels and why it sometimes took a reboot to clear. Keeping the simulation
between runs was faster, and it caused most of the trouble worth having: a
latched occupancy grid reading as a live coverage number that never moved,
phantom robots on the feeds, and a Prepare that refused to start because
something was already running.

The Debug UI keeps the run on screen instead - see the mission report below.

**Setup page.** Pick the world, pick or edit the mushroom layout, choose the
search pattern, choose Autonomous or Manual drive, set how many of each robot
you want, press Prepare. The capture tool is always the gripper now, so that
choice is gone.

A fleet is forced headless: two quadcopters means two YOLO instances and two
depth cameras, and Gazebo's own renderer on top of that is what left a world
that never finished loading. Every camera is in the GUI regardless.

**The robot counts are real.** One of each runs `cage_mission.launch.py` as
before. More than one of either runs `quad_zcams/fleet_mission.launch.py`
instead - up to four quadcopters and five skidsteers, with their
`fleet_coordinator` handing mushrooms out between the rovers - and the
mission page grows a camera panel, a map marker and a state card per robot.
The ceilings are their launch file's own.

**Two worlds are offered**, because the team settled on two for the demo:
`simple_trees` and `river_moderate_aus`.

`simple_trees` lives in `41068_ignition_bringup/worlds/simple_trees.sdf` and
is in git, so it is on every machine. No ponds, no river, nothing to bog down
on, and it loads in seconds - the world to fall back to while something else
is being debugged.

`river_moderate_aus` is the mushroom demo world: a real river with three
rocky crossings, no ponds, Australian flora, and the only world the two-class
`amanita_river_v6` weights were trained against, so the only world where
native mushrooms can be detected at all. It is the world
`forest_worlds/demo_river.sh` runs. **It is not in git and never can be** -
`forest_worlds/.gitignore` excludes `worlds/`, `models/`, `world_assets/`,
`textures/`, `assets_src/` and `dem/`, so what is committed is the generator
scripts only. Each machine builds its own copy once:

```bash
cd src/RS1_Team5_/forest_worlds && bash setup.sh      # internet, Blender >= 4.2
./view.sh moderate                                    # check it opens
```

The last line of `setup.sh`, `build_world.py --preset moderate --flora
australian`, is what writes `forest_worlds/worlds/river_moderate_aus.sdf`
alongside its `.meta.json`. Until that file exists the world is simply absent
from the dropdown, because `world_available` looks for it - which is why
whoever on the team has this world built it themselves.

`forest_flat`, `forest_random` and `large_demo` are off the menu. They still
resolve by name for their metas and map drawing if anything asks.

Layout presets are the world's own mushrooms (on a world that ships some),
whatever is in `layouts/<world>.json`, and **Random placement**, which puts
them clear of every solid thing in the world file - trees, fallen trees,
logs, stumps, rocks, debris and decoys, each by its own radius plus a margin
- outside the pond shores and the river, apart from each other and away from
the spawn clearing, and can be rolled again as often as you like. A crowded
world may not have room for as many as you asked for, and says so rather
than quietly giving you fewer. Every preset is editable: the coordinate
rows are the real list, and Prepare spawns exactly what is in them.

The preview beside the setup is drawn from the world's own `.meta.json` using
the same widget the live map uses, so a mushroom that would spawn inside a
pond is visible before you press Prepare rather than after.

**Ready page.** The checklist on the left fills in as each piece arrives:
Gazebo's clock, both mission node processes, both robots' odometry, both
cameras, both detectors, Nav2's action server, and the rover's `husky1_map`
frame on `/husky1/tf` (**not** `/tf` - every robot gets its own TF topic). On the right is the photo from above, the same placement drawn from
the world file, and the coordinates. Shuffle moves the mushrooms and
re-photographs; Retake just re-photographs; Back to setup leaves everything
running.

If something never arrives, the button offers to launch anyway and says what
will be missing. Patience is measured from the last thing that arrived rather
than from the start - a river world takes far longer to come up, but in both
cases pieces keep arriving until the last one does, so forty-five seconds
with nothing new is a stall whatever the world. Launching anyway still
freezes the nodes and spawns the mushrooms; it just does not wait for the
photo. Gazebo and the mission nodes are the two checks it will not waive -
without those there is no mission to run.

The photo needs a working camera in Gazebo. On software rendering it can
take a long time or never arrive, which is why there is a timeout and the
drawing underneath - the drawing has the same information in it.

**Search pattern** exposes the quad's own launch arguments: square loops out
from the spawn or lawnmower rows, the spacing between them, cruise speed and
height, whether to keep moving through corners, and whether to plan around
the world file at all. The defaults are the ones the team's launch file uses.

**Abort** is the emergency stop, beside Pause. The skidsteer halts, the
quadcopter lands where it is, and the timeline grows a dark red band with a
stop sign on it running from the moment you pressed it to the moment
everything actually came to rest - after which the clock stops. Resume hands
the robots back and starts the clock again; End Mission is beside it.

Landing is the GUI flying the quad down on `parrot1/agl` while holding the
stand-down, which is the state in which the quad's own node expects somebody
else to be flying it. Their `ammo/mission_complete` also lands a quad, with
full obstacle avoidance, but it flies home first - which an emergency stop
should not do.

**Mission page.** Everything here starts when you press Launch: the clock,
mushrooms found and collected counted separately, natives identified and left alone, how much of the area the
quadcopter has mapped, both robots' states, the map, both camera feeds, and
the mushroom log with a camera still per mushroom.

The cameras are a grid: every quadcopter, every skidsteer, and an
**overhead** feed from a camera the GUI spawns high above the world looking
straight down. It is the same camera that takes the Prepare photo, left
running. On software rendering it is a third rendered camera and costs
frames, so it can be turned off.

Controls are Pause/Resume, Abort, End Mission, clicking the map, and
hand-driving either robot. **MARK** on the quad panel asks the quadcopter to
send the mushroom in its view to the rover's queue - the quad answers with
its own gated sighting, so the projection is its own and not a guess made
here. The map click has two modes: **GO** sends the skidsteer there with a
Nav2 goal, **COLLECT** queues that point on `husky1/cage/waypoints` as
though the quadcopter had found it. Double-click a camera to fill the
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

## The mission report

When a run ends, the Debug UI switches to a report rather than going quiet:
the counts, the whole timeline, the last overhead frame with **every
mushroom revealed and tagged** with what became of it - collected, gave up,
nothing there, never found - and a short read on what the numbers say and
which knob would change it.

It stays up until the next Prepare. The window dropping back to its waiting
banner with the cameras coming up one at a time is the visible proof that it
has picked up the new run, which was otherwise hard to tell.

## What the Debug UI does

Until a mission is launched, a banner across the top says which stage the
run is in - nothing running, preparing, or held and ready - and the
timeline, the event feed, the minimap's markers and the coverage figure stay
empty. The camera feeds and both robots' states and speeds are live
throughout, because during Prepare they are how you tell the simulation
really came up.

Once a mission starts: both camera feeds, large, showing the raw camera with the detector's boxes
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

Press **M** over either window to ring every mushroom and every robot on the
overhead view. The robots are not decoration: a robot is visible in the
picture, so if its circle sits on it, the mushroom circles are in the right
place too. The projection is in `overhead.py` - a straight-down, centred,
flat camera is a scale and two sign flips, and those two signs are the only
thing that could be wrong.

The **world edge** is drawn as a near-white dashed box with its size on it,
read from `world.playable_size_m`, and the map opens fitted to show all of
it. That matters because a Nav2 goal outside the playable area is rejected,
which from the operator's side looks exactly like the rover ignoring the
click; a click outside the box is now refused with a reason instead of being
sent. A world with no `.meta.json` has no box and allows everything.

Every robot gets its own marker and trail, labelled Q1, Q2, S1 and so on
once there is more than one of either.

**FIT** switches between the whole world and in close on the robots. Whole
world is the default, because it is the only way to see where you may click,
but a 60 m world makes the trees small - so the symbols are drawn brighter
and at a floor size when zoomed out, and one button gets the detail back.

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
- **Ending a mission keeps the simulation.** It freezes the two mission node
  processes and removes the mushrooms the GUI spawned; Gazebo, Nav2 and the
  perception stack stay up. That is done by stopping processes rather than by
  a second, mission-only launch path, which is deliberate: a launch path of
  our own would drift from the team's the first time they edited theirs.
  Changing world is what tears it down.
- **Mushrooms are spawned by the GUI**, not by `mushrooms:=`, and they are
  spawned after the mission nodes are frozen. Spawn height is estimated from
  the world file's own objects, because `forest_random` rises over three
  metres and a mushroom spawned at z = 0 there is underground.
- **A world's own mushrooms can be taken out** - the generators name them
  `mushroom_00`, `mushroom_01`, ... and write the names into the meta - which
  is what lets Shuffle work on the river world and on the forest worlds. They
  do not come back until Gazebo restarts.
- **Snapshots** in the mushroom log are held in memory only. Closing the
  window loses them.
- **Box colours come from the weights' own class names.** The v6 model emits
  `amanita_muscaria_red` and `native_mushroom`; `DET_COLOURS` in
  `contract.py` lists both. Retrain with different names and every box goes
  back to the unknown-class blue until that dictionary is updated.
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

555 checks, a few seconds, no ROS, no simulation, no display and no built
workspace - `tests/stubs/` stands in for rclpy and the message types. The
`--render` flag also writes pictures of the setup page, the ready page
part-way through and finished, both windows, the Debug UI's held state and
the map into `tests/out/`, which is the quickest way to see what a change did to a layout.

One of those suites, `test_log_lines.py`, checks the lines the bridge reads
are still present in the mission nodes' source. If a teammate rewords one,
that is where you find out.
