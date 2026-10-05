# Integration notes - for the team

Nothing outside `ammo_gui_node/` was changed to make the GUI and the Debug
UI work. This file is here for two reasons: so nobody breaks them by
accident, and so we can agree on the small additions that would let the
workaround be deleted.

## 1. Log lines the bridge currently depends on

`mission_bridge.py` reads `/rosout` to work out what the rover is doing,
because `cage_capture_node` never publishes its state anywhere else. These
are the exact strings it matches, all of them already in the file - none
were added for us:

| Log line (in `cage_capture_node.py`) | What the UI uses it for |
|---|---|
| `-> NAV`, `-> ALIGN`, `-> COLLECT`, … (`_set_state`) | rover state, and the colour on the timeline |
| `Waypoint queued (x,y); N in queue.` | how many mushrooms are waiting |
| `N waypoint(s) left.` | same, after one is finished |
| `Collected N mushroom(s) with the gripper.` | the collected count |
| `Given up after repeated deferrals.` | the gave-up count, kept apart from collected |
| `Deferring to the back of the queue (attempt i/n).` | the "retrying later" note |
| `Nav2 failed toward …` / `Nav2 could not reach …` | the "route blocked" note |

**If you reword one of these, the matching number in the UI quietly stops
moving.** Nothing crashes, and nothing else is affected - but please say so
in the PR, or better, do item 2 below and we can drop this table entirely.

## 2. What would replace all of that

Four small, additive changes. None of them alter behaviour; they only make
information that already exists reachable by other nodes.

**`cage_capture_node.py`**

1. Publish the state. One publisher, and one line inside `_set_state` -
   the state string already exists, it just never leaves the node.
2. Say whether a mushroom was collected or given up on. `husky1/cage/done`
   currently fires for both, which is why the bridge has to read the log to
   tell them apart. `_collect` already knows the count (`len(under)`), so a
   second topic, or a flag alongside the point, is enough.
3. Obey a pause flag. Subscribe to `ammo/mission/pause`; when set, publish
   a zero `Twist` and cancel the current Nav2 goal instead of stepping the
   state machine. This is the one that would most improve things: right now
   pause has to suspend the whole process, and a suspended node comes back
   with stale timers and sometimes defers the waypoint it was on.
4. A `manual_drive` parameter that skips `DRIVE`/`NAV` but still runs
   `ALIGN` → `COLLECT` when the operator has driven within range. At the
   moment manual mode works because the operator's commands and the node's
   both go to `cmd_vel_raw`, which is not a clean arrangement.

**`quad_search_node.py`**

5. An explicit state (searching / detouring / holding / waiting for the
   rover), published the same way. The bridge currently infers this from
   whether the quad is moving, which is right most of the time and wrong
   when it is hovering for a reason we cannot see.
6. Publish the detection confidence alongside the waypoint it reports. The
   bridge pairs the waypoint with the most recent score off
   `parrot1/detections`, which is usually the right one but is a guess.

**`perception/yolo_node.py`** - nothing needed. Confidence is already in
`Detection2DArray`, and the annotated image is already published.

## 3. Things worth knowing either way

- `simple_trees.sdf` contains no mushrooms; the GUI's layout is passed to
  `cage_mission.launch.py` as `mushrooms:=`. forest_flat, forest_random and
  the river worlds already contain theirs.
- Both robots use Gazebo's `VelocityControl` plugin, which holds the last
  velocity it was given indefinitely. Anything that stops commanding a robot
  has to send an explicit zero, or it keeps going.
- `yolo_node` is capped at 2 Hz, so the annotated image is not usable as a
  live view. The Debug UI shows the raw cameras and only grabs an annotated
  frame at the moment of a sighting.
- The GUI runs `cage_mission.launch.py` as one process group and pauses by
  stopping the mission node processes inside it (`mission_launcher.py`).
  With the default `quad:=zcam` the rover node is `rover_capture_node`, a
  subclass of `cage_capture_node` with the same node name and log lines.
