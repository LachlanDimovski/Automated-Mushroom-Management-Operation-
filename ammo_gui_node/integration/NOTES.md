# Integration notes - for the team

This package used to change nothing outside itself. That is no longer true:
there are now **three small additions in two other packages**, all of them
additive, all of them listed below with the reason. Everything else the GUI
needs it still takes off the wire.

## 1. What was added outside `ammo_gui_node/`

### `ammo_mission_control/ammo_mission_control/cage_capture_node.py`

The rover has always had a state (`DRIVE`, `NAV`, `ALIGN`, `LOWER`,
`COLLECT`, `RETRACT`, `RAISE`, and `FORD`/`UNWEDGE` in the subclass). It just
never left the node - it was written to the log and nowhere else, so the GUI
had to read `/rosout` and scrape `-> NAV` out of it.

Three lines fix that:

- `String` added to the existing `std_msgs.msg` import.
- A latched publisher on **`husky1/rover/state`** next to the existing
  `done`/`missed` publishers, which publishes the initial state once.
- One `self.state_pub.publish(String(data=s))` inside `_set_state`.

`_set_state` is the single funnel for every state change in both the base
class and `RoverCapture`, so one line there covers all nine states for both
executables. Behaviour is unchanged; the publisher is latched
(`TRANSIENT_LOCAL`) so a GUI opened mid-run gets the current state
immediately rather than waiting for the next transition.

### `quad_zcams/quad_zcams/rover_capture_node.py`

One extra publish, and `String` imported. The `FORD` → `NAV` transition at
the end of `_ford_step` assigns `self.state` directly instead of calling
`_set_state`, so it is the one transition the line above does not catch.
Without this the GUI shows `FORD` until the next real state change.

### `quad_zcams/quad_zcams/zcam_quad_node.py`

The quadcopter had no state variable at all, which is why the GUI could only
ever say "searching" or "idle" - it was guessing from whether the quad
happened to be moving. Added:

- A `Phase` class of plain strings at module level (deliberately **not**
  imported from the GUI package - the mission node must not depend on it;
  `ammo_gui_node/contract.py` holds the matching copy).
- `_phase()`, which resolves the phase from variables the flight code
  already maintains: `have_odom`, `taken_off`, `mode`, `looking`,
  `escaping`, `holding`, `wp_seen_at`, `hold_view`, `stationary_since`,
  `recovery_until`.
- `_publish_state()`, publishing a small JSON block on
  **`parrot1/quad_state`** on change and once a second otherwise.
- `_control_step()`, overriding the parent only to call it afterwards.
- One line recording `recentred_at` where a hold re-aims.

`_publish_state` is wrapped in a bare `except` on purpose: a GUI feed must
never be able to interrupt the flight loop. It publishes after `_shape`, so
it sees the ZCam-only transitions that happen inside it.

None of this changes how either robot behaves. If you want it gone, deleting
the publishers puts the GUI back on the log-reading path automatically - see
section 3.

### `quad_zcams/quad_zcams/zcam_quad_node.py` - a merge left two of one method

`ZCamQuad` ended up defining `_control_step` **twice**: once near the top
(the two-line version added for the GUI, which calls `_publish_state`) and
once further down (the teleop and return-home version). Python keeps the
later definition, so the first was dead and **`parrot1/quad_state` stopped
updating entirely** - the GUI fell back to scraping `/rosout` and nobody
noticed, because the fallback works.

Fixed by renaming the flight code to `_flight_step` and making
`_control_step` a three-line wrapper that calls it and then publishes the
state in a `finally`. A wrapper rather than a line at the bottom because
`_flight_step` returns early on three separate paths - returning home, the
operator in control, normal flight - and the state has to go out on all of
them.

Behaviour is unchanged. Worth a look if you merge this area again.

## 2. Log lines the bridge still reads

State now comes from the topics above. Events still come from the log, and
that is deliberate: "the rover gave up at (3.1, -4.2)" is a sentence the node
already writes exactly once with the numbers in it, and inventing a topic per
event would be a lot of ceremony for no new information.

| Log line | What the UI does with it |
|---|---|
| `Waypoint queued (x,y); N in queue.` / `N waypoint(s) left.` | how many mushrooms are waiting |
| `Collected N mushroom(s) with the …` | the collected count |
| `Given up after repeated deferrals.` | the gave-up count, kept apart from collected |
| `Deferring to the back of the queue (attempt i/n).` | the "retrying later" note |
| `At ford k (x,y) - driving the lane across myself.` | ford marker on the map, band on the timeline |
| `Ford k crossing timed out at (x,y) …` | amber line in the event feed |
| `Boxed in at (x,y) (c m to the nearest …` | stuck marker on the map and the timeline |
| `Nav2 leg stalled: moved < N m in N s …` | amber line in the event feed |
| `Nav2 gave up at (x,y) - later routes avoid it (N spot(s)).` | red cross on the map - explains later routing |
| `Nothing at (x,y) - later handoffs within N m are dropped.` | hollow circle on the map, and the mushroom reads "nothing there" rather than "gave up" |
| `No route on the quad's map - routing on …` | event feed |
| `No route to (x,y) on the quad's map around N failed spot(s) …` | event feed, and the rover's detail line |
| `Queue empty - retrying the N mushroom(s) given up earlier.` | the "second pass" chip |
| `River blocked on the route map: N m2, N ford(s) open.` | the ford count under the coverage bar |
| `Nav2 -> (x,y) facing (gx,gy) through N via point(s) …` | the planned leg drawn on the map |
| `native mushroom N logged at (x, y) - left in place` | fallback for natives (normally read off `parrot1/quad_state`) |

**If you reword one of these, the matching thing in the UI quietly stops
moving.** Nothing crashes. `tests/test_log_lines.py` fails, which is the
point of it - run `python3 ammo_gui_node/tests/run_tests.py` and it will tell
you which line went.

## 3. What is still a workaround, and what would replace it

Items 1 and 5 of the old list are done - both nodes publish their state now.
What is left:

1. **Say whether a mushroom was collected or given up on.**
   `husky1/cage/done` still fires for both, which is why the bridge has to
   read `Collected N mushroom(s)` / `Given up after repeated deferrals` to
   tell them apart. `husky1/cage/missed` already does exactly the right
   thing for an empty grab; a second flag alongside `done` would finish the
   job.
2. **Obey a pause flag.** Subscribe to `ammo/mission/pause`; when set,
   publish a zero `Twist` and cancel the current Nav2 goal instead of
   stepping the state machine. This is the one that would most improve
   things: right now pause has to suspend the whole process, and a suspended
   node comes back with stale timers and sometimes defers the waypoint it was
   on.
3. **A `manual_drive` parameter** that skips `DRIVE`/`NAV` but still runs
   `ALIGN` → `COLLECT` when the operator has driven within range. At the
   moment manual mode works because the operator's commands and the node's
   both go to `cmd_vel_raw`, which is not a clean arrangement. The same
   applies to the quadcopter, which the GUI can now hand-fly by suspending
   its node.
4. **Say why a detection was not dispatched.** `_gated_projection` rejects a
   sighting for seven different good reasons - confidence, aspect ratio, the
   projection not reaching the ground, a native, too near the rover, beyond
   `det_range`, already dispatched - and says nothing about which. One log
   line naming the gate would appear in the GUI's event feed for free, and
   would answer "YOLO boxed it, why was it not collected?" without anyone
   having to read the source.
5. **Publish the detection confidence alongside the waypoint.** The bridge
   pairs a waypoint with the most recent score off `parrot1/detections`,
   which is usually the right one but is a guess.

If the state publishers are ever removed, the bridge notices: it only trusts
`husky1/rover/state` and `parrot1/quad_state` while they are fresh, and falls
back to the `/rosout` path it used before. Nothing in either window changes.

## 4. Your mechanisms the GUI now uses instead of its own

These were all already in the repo. The GUI used to work around every one of
them, and does not any more.

- **`<robot>/teleop_active`** is how every robot is held still: Prepare,
  Pause, Abort, hand-driving and an operator Nav2 goal all publish the
  heartbeat at 5 Hz, inside your 0.5 s `TELEOP_HOLD_S`. It replaces
  `SIGSTOP` on the mission node processes, which is strictly worse: a frozen
  node comes back with stale timers, a stale `/clock` and sometimes defers
  the waypoint it was on, which is the whole reason
  `config/fastdds_udp_only.xml` had to exist. A standing-down node keeps all
  of that, and the quad keeps reporting sightings while the operator flies
  it. `hold_nodes()` is still in `mission_launcher.py` as a last resort.
  The GUI publishes the zeros too, because VelocityControl latches.
- **`parrot1/teleop_mark`** is the GUI's MARK button, on the quad teleop
  panel. The quad answers with its own latest gated sighting and its own
  freshness check, so nothing in the GUI projects anything - which matters,
  because `_gated_projection` is the part of this system with the most
  hard-won detail in it. Same as PS4 CROSS, different button.
- **`parrot1/agl`** is what an abort's descent watches, not odometry z. On
  `forest_random` the ground rises over three metres, so they are not the
  same number.
- **`quad_zcams/fleet_mission.launch.py`** is what the setup page's robot
  counts run once either is above one, with `quads:=` and `rovers:=` and the
  `forest_worlds` resource path exported first. The GUI does not spawn a
  second robot itself.

What the GUI could not use, and why:

- **`quad_mission`'s `/parrot1/mission/abort`.** It is a real abort, but the
  node is not in `cage_mission.launch.py` and it publishes `parrot1/cmd_vel`,
  so running it alongside the search node breaks the topic-ownership rule in
  the README. An abort therefore holds the quad down with `teleop_active` -
  the state in which you already expect somebody else to be flying it - and
  descends it on `parrot1/cmd_vel` until `parrot1/agl` says it is down.
- **`ammo/mission_complete`.** It lands a quad properly, with avoidance, but
  it flies home first, which an emergency stop must not do. It is on the
  list below as the obvious thing to wire to a Return Home button.

## 5. The detector's class names

`DET_COLOURS` in `contract.py` must match the weights' own class list
exactly. The v6 model's `metadata.yaml` says `0: amanita_muscaria_red`,
`1: native_mushroom`; the GUI had `mushroom_amanita`, the same two words the
other way round, so **every invasive box was drawn in the unknown-class blue
while the README said invasive boxes are red**. Fixed, with the shorter
spellings kept as fallbacks. If you retrain with different class names, that
dictionary is the one place to change.

## 6. Settings the GUI overrides

- **`husky_yolo:=true`.** Both `cage_mission.launch.py` and
  `zcam_mission.launch.py` default it to false, which leaves the rover with
  no detector - so `husky1/detections` and `husky1/detections/image` never
  publish and the rover's camera panel has nothing to draw. The GUI turns it
  on. It costs a second YOLO instance; if that hurts on a slow machine, that
  is the knob.
- **Operator goals are sent in `husky1_map` with no stamp**, which is what
  `cage_capture_node._send_nav` and `rover_capture_node._nav_to` both do. A
  goal in `map` is silently unusable - there is no bare `map` in the rover's
  TF tree with truth_localizer running - and a stamped goal can out-run the
  TF buffer. Getting either wrong looks like the rover turning on the spot
  and then reporting that it arrived.
- **A zero is published to `cmd_vel_raw` before an operator goal**, and the
  goal waits `GUARD_SETTLE_S` after that. `collision_guard` replays the last
  command it saw on that topic for `RAW_CMD_TIMEOUT_S` (0.5 s), so a goal
  sent the instant the rover's node is frozen has the guard's replay and
  Nav2 taking turns driving.

## 7. Things worth knowing either way

- **The GUI no longer passes `mushrooms:=`.** It spawns them itself, with
  the same `ros2 run ros_ign_gazebo create` call `cage_mission.launch.py`
  makes and the same `demo_mushroom_N` names, but *after* the mission nodes
  are frozen - which is the whole point of the Prepare stage: the quadcopter
  cannot have seen a mushroom the operator is still deciding about. It also
  spawns at an estimated ground height rather than z = 0, because on
  `forest_random` the terrain rises over three metres and a mushroom at z = 0
  is underground. The height is interpolated from the trees in the world's
  `.meta.json` and calibrated against the mushrooms in it, which sit exactly
  on the surface.
- **The GUI can remove a world's own mushrooms.** Both forest generators name
  them `mushroom_00`, `mushroom_01`, ... and write the names into the meta,
  so `ign service .../remove` can take them out - which is what lets a
  different layout or a shuffle work on `forest_flat` and `forest_random`
  instead of adding a second set on top. Nothing is removed unless the
  operator picked a layout other than the world's own.
- The search-pattern settings on the setup page (`center_out`,
  `row_spacing`, `cruise_speed`, `cruise_alt`, `strafe`, `planner`,
  `rover_speed`) are declared in `zcam_mission.launch.py`, not in
  `cage_mission.launch.py`. They reach it because an argument given on the
  `ros2 launch` command line becomes a launch configuration that an inner
  `DeclareLaunchArgument` does not overwrite - the same route `model_path`
  and `imgsz` already take. If that ever stops being true, the GUI would
  have to launch `zcam_mission.launch.py` directly instead.
- The quadcopter's "N% of the area seen" is computed in the bridge from
  `parrot1/ground_obstacles` - the fraction of cells that are not -1 - which
  is the same quantity `rover_capture_node` prints once a minute, but
  continuous. It is **not** `ZCamQuad.cover`, which is a different grid
  measuring the camera footprint rather than the shared map.
- Both robots use Gazebo's `VelocityControl` plugin, which holds the last
  velocity it was given indefinitely. Anything that stops commanding a robot
  has to send an explicit zero, or it keeps going. That is why pause
  publishes zeros at 20 Hz rather than just suspending the nodes.
- `yolo_node` is capped at 2 Hz. Both windows show the annotated feed by
  default now, because that is where you can see the detector telling an
  invasive amanita from a native - but it does look like a slideshow. The
  word in the top right of each camera swaps it back to the raw feed.
- The GUI runs `cage_mission.launch.py` as one process group and pauses by
  stopping the mission node processes inside it (`mission_launcher.py`).
  Arming either teleop panel suspends that robot's node for the same reason:
  two publishers on one velocity topic means the robot obeys whichever
  message arrived last.
- **Prepare and Launch are the same mechanism.** Prepare starts your launch
  file unchanged and then `SIGSTOP`s `rover_capture_node` and
  `zcam_quad_node` **as soon as those processes exist** - seconds after the
  launch, not at the end of the checklist - while publishing zero velocity to
  both robots at 20 Hz, because VelocityControl would otherwise keep flying a
  quadcopter whose node was frozen one second into a take-off. Launch
  `SIGCONT`s them. `collision_guard_node` is
  deliberately left awake throughout - it is the thing that publishes the
  stop. Ending a mission re-freezes them and leaves Gazebo up, so a second
  run on the same world costs nothing. None of this needs a second launch
  path, which was the alternative and would have drifted from yours the
  first time you edited it.
- **The rover's TF is on `/husky1/tf`, not `/tf`.**
  `41068_ignition.launch.py` gives every robot its own ("TF topics are under
  `/<namespace>/tf`"), and `cage_mission.launch.py` remaps `truth_localizer`
  and `collision_guard` onto it, so `husky1_map` never appears on the global
  `/tf`. Worth knowing for anything else that goes looking for it - a
  readiness check watching `/tf` waits forever, which is exactly what
  happened the first time this ran.
- **Nothing in the GUI watches a timer to decide the simulation is up.** It
  waits for `/clock` to have a publisher, for both odometry topics and both
  camera topics to deliver a message, for both `*/detections` topics to have
  a publisher, for Nav2's `husky1/navigate_to_pose` server to answer, and for
  `husky1_map` to appear on `/husky1/tf`. If you rename any of those, the checklist
  stops at that line and says what it is waiting for - which is a better
  failure than the mission starting into a robot that cannot plan.
- With the default `quad:=zcam` the rover node is `rover_capture_node`, a
  subclass of `cage_capture_node` with the same node name (`cage_capture`)
  and the same log lines.
