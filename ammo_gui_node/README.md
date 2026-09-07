# ammo_gui_node

Danil's operator GUI - separate from `ammo_dashboard` (Lachlan's read-only
debug HUD). This one is interactive: mission setup, live mission progress,
mushroom count, elapsed time, pause/resume, manual keyboard teleop of the
Parrot, and an occupancy-grid map view with robot pose/heading and mushroom
markers.

## One-time setup (per machine)

```bash
pip3 install PySide6
```

That's the only new dependency this package needs beyond what's already
installed for the rest of the workspace. No other pip/apt packages, no
change to any other package's dependencies.

## Running

With the core sim already up in one terminal (see the main repo cheat
sheet - husky+parrot, SLAM, `world:=large_demo` or `simple_trees`):

```bash
ros2 launch ammo_gui_node gui.launch.py
```

The home page lets you set a working area (hectares) and pick Autonomous or
Manual mode, then "Begin Mission" launches the matching
`ammo_mission_control` launch file for you (`autonomous.launch.py` or
`mushroom_mission.launch.py`) - no separate terminal command needed for
that part. It does **not** start the sim itself; that stays a separate
step, same as today.

In Manual mode, the teleop panel on the right flies the Parrot from the
keyboard (W/A/S/D/R/F/Q/E - click the panel and check "Teleop armed"
first). This is the keyboard alternative to `ps4_drone_test.py`, not
something to run alongside it - both publish to `/parrot1/cmd_vel` with no
arbitration between them, so only one should be active at a time.

## What's real vs. what's a first pass

- Map, robot poses, elapsed time, pause/resume-as-display-freeze, and
  keyboard teleop are wired to live topics/state and work as described.
- Mushroom count and mushroom markers need one additive change to
  `ammo_mission_control/ammo_mission_control/mushroom_mission_node.py`
  (publishing each newly-confirmed mushroom position on a new
  `mushroom_found` topic) - see that file's diff separately. **That change
  needs Lachlan's sign-off before it's pushed**, since it's his file.
  Until it's in place, mushroom count/markers just stay at zero rather
  than erroring - everything else works fine without it.
- "Area explored" progress is a real number (fraction of the occupancy
  grid that's no longer unknown, scaled against the working-area figure)
  but it's a proxy, not true coverage-within-boundary - there's no actual
  geofencing yet, so the working-area figure doesn't constrain the robots,
  it's just what the progress bar is measured against.
- Pause/resume freezes the display only - it doesn't stop the robots or
  the mission-control nodes. A real pause needs coordination with the
  mission-control side; this is deliberately the "looks right, not wired
  up for real yet" version for now (see `operator_page.py`'s docstring).
- No pan/zoom on the map view yet - it auto-fits to the widget. Easy to
  add later if it's needed.
