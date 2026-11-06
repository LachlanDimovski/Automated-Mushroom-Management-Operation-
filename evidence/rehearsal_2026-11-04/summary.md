# Demo rehearsal, 4 Nov 2026 (the day before C4)

Both demo parts were run on the laptop from the hand-in repo, using the exact commands in `DEMO_DAY.md`.

- Code: `handin-main` at `366d30f`, which includes Danil's PR #26 and Darcy's GUI README. It is pushed to `handin/main`.
- Power profile: performance.
- Logs: `~/.ros/ammo_demo/rehearsal_1104/`. The scripts that drove the runs are listed under "How it was tested".

## Part 1: `demo/part1_fleet.sh` + `demo/part1_timer.sh` (PASS)

| Check | Result |
|---|---|
| Sim clock start | about 20 s after the command |
| 12/12 collected | 800 s (13.3 min) from sim start, 702 s of sim time |
| Natives | 4 logged, each 0.45 to 0.73 m from its true position. 0 false, 0 sent to the rover |
| Bridge `start` command | sent by the script by itself (`river_moderate_aus_dt2`, 12, 2 quads, 1 rover) |
| AMMO Fleet Debug window | renders with Danil's changes: banner, clock, timeline, minimap and live feeds |
| Tracebacks | 0 |
| Empty grabs, water reports, UNWEDGE | 0, 0, 0 |
| Nav2 | did not come up at launch; the rover's fallback started its nodes directly within 1 s. 2 goals gave up and were retried from another side. 54 "Controller patience exceeded" lines are normal Nav2 noise |
| CPU | 93 to 99 °C throughout (started at 48 °C) |

What to expect on the day:
- About 1 to 2.5 minutes after 12/12, the coordinator ends the simulation by itself. The status bridge stops with it,
  so the Fleet Debug window stays open but stops updating. Talk through the result first, then run
  `bash demo/stop_sim.sh`.
- The bridge `start` command and the timer both waited for the sim clock, as intended.

## Part 2: `demo/part2_gui.sh` (operator GUI, cage_mission, PS4)

The launch command was smoke-tested on its own: the bridge and the GUI both start. The full sequence was then driven
through the real operator GUI code. A script pressed the same buttons an operator would, while the status bridge, a
scripted PS4 controller on `/joy` and Darcy's showcase (`ros2 launch quad_showcase/quad_showcase.launch.py`) ran
alongside, as they will on the day.

### Run 3, default setup ("Show the Gazebo window" ticked): PASS

| Step | Result |
|---|---|
| Prepare to READY | 19 s |
| First collection | 596 s after Launch (see "Timing" below) |
| Pause, 25 s | both robots held within 0.1 s. The rover drifted 0.05 m at most, the quad 0.00 m |
| Resume | both released within 0.1 s; the rover's next Nav2 goal came 0.9 s later |
| R1, rover | 3.6 m forward in 6 s, then a turn on the spot (0.06 m of drift). On release it was handed back at once and re-planned 0.3 s later |
| L1, quad | 2.4 m forward in 4 s, then a climb. On release it resumed its search 0.1 s later |
| OPTIONS, TRIANGLE | teleop mode drove the rover 1.8 m with no button held; TRIANGLE handed the rover back (it re-planned 0.4 s later) and drove the quad 1.8 m; OPTIONS again handed the quad back |
| Scripted PS4 checks | 9/9 passed |
| End Mission | after the third collection. The window went back to the setup page and no simulation process was left |
| Tracebacks | GUI 0, bridge 0, showcase 0, PS4 script 0. The mission log had 6, all `KeyboardInterrupt` while nodes shut down after End Mission |

Minor observation: just after Pause the hold dropped for about 0.5 s and came straight back. The rover sent a Nav2
goal in that gap and cancelled it again; neither robot moved. See "Pause hold lapses" below.

### Run 4, fallback setup ("Show the Gazebo window" unticked, 3D view opened separately): PASS

Same sequence, except that Pause was pressed 60 s after Launch with nothing found yet, which is the plan for the day.
The 3D view was opened with `source demo/env.sh && ign gazebo -g` once the mission was running.

| Step | Result |
|---|---|
| Prepare to READY | 19 s |
| 3D view (`ign gazebo -g`) | joined the running simulation and drew the whole world, 0 errors. It is the same default layout the ticked window uses |
| Pause, 25 s | both robots held. The rover drifted 0.00 m, the quad 0.20 m |
| PS4 | 8/9. The one miss was a scripting assumption: R1 was released before the quad had found anything, so the rover had no goal and waited where it was left, which is correct. When the quad reported its first mushroom, the rover planned a route 0.6 s later |
| First collection | 446 s (7.4 min) after Launch |
| End Mission | the window went back to the setup page and the separate 3D view closed with the simulation; no process was left |
| Tracebacks | GUI 0, bridge 0, showcase 0, PS4 script 0. The mission log had 4, all `KeyboardInterrupt` during shutdown |

### Run 5, the exact demo-day setup: PASS

Run 4 again, with the GUI node on simulation time. `demo/part2_gui.sh` starts the GUI through `gui.launch.py` with
`use_sim_time:=true`, and the driven runs before this one had used wall-clock time.

| Step | Result |
|---|---|
| Prepare to READY | 19 s, window unticked; the 3D view opened with `ign gazebo -g`, 0 errors |
| Pause at 60 s, 25 s | both robots held. The rover drifted 0.00 m, the quad 0.18 m |
| PS4 | every behaviour correct. R1: 3.6 m forward, a turn with 0.01 m of drift, handed back at once. L1: 2.4 m, handed back at once. OPTIONS/TRIANGLE: 1.8 m each robot. The script marked one check as failed for the same reason as run 4: teleop mode was switched off before the quad had reported anything, so the rover had no goal to return to |
| First collection | 437 s (7.3 min) after Launch |
| End Mission | back to the setup page; the simulation and the separate 3D view closed; no process left |
| Tracebacks | GUI 0, bridge 0, showcase 0, PS4 script 0. The mission log had 6, all `KeyboardInterrupt` during shutdown |

## Pause hold lapses (minor, left alone for the demo)

While paused, the GUI holds each robot with a 5 Hz `teleop_active` heartbeat (`TELEOP_HZ` in
`ammo_gui_node/contract.py`), and each robot node lets go 0.5 s after the last beat. Under full CPU load the beats
sometimes arrive late and the hold lapses for a moment:

- Run 3 (wall-clock GUI): 1 lapse of about 0.5 s.
- Run 4 (wall-clock GUI): none.
- Run 5 (sim-time GUI, as on the day): 4 lapses of 0.08 to 0.18 s in 25 s. With the sim at about half real time, a
  5 Hz sim-time beat leaves half the margin of a wall-clock one.

Neither robot moved in any of them (0.00 m rover, 0.18 m quad at most), so nobody watching would notice. After the
demo, Danil could raise `TELEOP_HZ` to 10 or 20, or drive the heartbeat from a steady clock. It is a one-line change,
but not one to make the night before.

## The Gazebo window start-up hang

With "Show the Gazebo window" ticked, Gazebo Fortress starts its server and its window as a pair. The server then
waits for one start-up message from the window (`/gazebo/starting_world`, the `wait_gui` option in
`/usr/lib/ruby/ignition/cmdgazebo6.rb`) before it loads the world. If that one message is missed, the server waits
forever: the spawners keep printing "Requesting list of world names", there is no `/clock`, and Gazebo logs no
error. Unticked, the server runs with `-s`, which turns the wait off.

- On 4 Nov, 2 of 9 starts with the window hung this way: the first two of the morning. The other 7 came up
  normally, and the 6 of those started from the GUI were READY in 18 to 20 s.
- The GUI shows it clearly: after about 60 s, "Simulation" and "Mission nodes" are ticked while both telemetry rows
  and both camera rows stay empty, and after about 90 s the page says the simulation has not come up.
- Recovery: press BACK TO SETUP (it stops everything), untick the box and press Prepare again.
- Avoiding it: untick the box before Prepare and open the 3D view with `ign gazebo -g` (run 4). The view is the
  same, and End Mission closes it too.

## Timing for the Part 2 slot

On `river_moderate_aus` with the world's own layout, every invasive mushroom is 15 m or more from the start, so the
quad needs 7 to 10 minutes to find the first one (446 s and 596 s after Launch in runs 4 and 3). Do Pause and the PS4
takeover in the first couple of minutes; a collection is unlikely inside an 8-minute slot, and Part 1 already shows
twelve. Before the first find the rover has no goal, so after an R1 release it waits where it was left. Once the quad
reports a mushroom, it plans a route within a second.

Darcy's showcase and the separate 3D view each use about one CPU core. The CPU sat at 97 to 100 °C in every run.

## How it was tested

The scripts are in `rehearsal_scripts/` next to this file, and every run's logs are in
`~/.ros/ammo_demo/rehearsal_1104/` (`part1/`, `part2_run3/` to `part2_run5/`, plus the two hung starts in `part2/`
and `part2_run2/`).

- `part1_rehearsal.sh DIR` runs `demo/part1_fleet.sh` and `demo/part1_timer.sh` unchanged, with their logs sent to
  DIR. It logs progress every 30 s and takes off-screen Fleet Debug screenshots.
- `part2_rehearsal.sh DIR` starts the status bridge, `teleop_test.py` and `gui_rehearsal.py`. Once the mission runs it
  starts Darcy's showcase with the demo-day command. With `VIEWER=1` it also opens `ign gazebo -g`.
- `gui_rehearsal.py` builds the real `OperatorApp` and presses its buttons: Prepare, Launch, Pause, Resume and
  End Mission. Settings: `SHOW_SIM=0` unticks the Gazebo window, `PAUSE_AT_S` pauses at a fixed time after Launch,
  `SIM_TIME=1` puts the GUI node on simulation time as `gui.launch.py` does, and `PREPARE_ONLY=1` stops at READY.
- `teleop_test.py` stands in for the PS4 controller on `/joy`. It records `/rosout` and both robots' odometry, and
  checks each takeover step against the odometry and the robots' own log lines.
- `trials.sh DIR N` runs N Prepare-only starts and records how long each took to reach READY.

Not covered: the real PS4 controller (none was plugged in). Do the stick check in `DEMO_DAY.md` before the slot.

## Final check, 4 Nov 13:46 to 14:12 (both parts again, exact demo setup): PASS

| Part | Result |
|---|---|
| Part 1 (`part1_fleet.sh` + `part1_timer.sh`, each in its own session as two terminals) | 12/12 in 571 s (9.5 min) from sim start, CPU started at 42 °C. All 6 natives logged (5 within 0.5 to 0.9 m, native_03 1.8 m off at the world edge) plus the known false native at the water's edge near (17.7, -12.2); none sent to the rover. 0 Nav2 give-ups. Fleet Debug live throughout. The sim ended itself after 12/12; the bridge traceback and a Nav2 lifecycle-manager crash happen during that automatic shutdown |
| `bash demo/part2_gui.sh` | starts the status bridge and the GUI (2 tracebacks only on Ctrl+C) |
| Part 2 driven run (sim-time GUI, window unticked, `ign gazebo -g`, showcase) | READY 19 s; Pause 61 s after Launch held 25 s (rover 0.00 m, quad 0.06 m); scripted PS4 9/9; first collection 412 s after Launch; End Mission back to setup, simulation and viewer closed, nothing left running; GUI, bridge, showcase and PS4 script 0 tracebacks |
| Start reliability, window unticked | 4 Prepare-only starts READY in 18 to 19 s; 7 of 7 unticked starts today |

## Second check, 4 Nov 15:24 to 15:52 (both parts again, exact demo setup): PASS

Logs: `~/.ros/ammo_demo/rehearsal_1104/check2_part1/`, `check2_part2/` and `check2_trials/`.

| Check | Result |
|---|---|
| Setup | The hand-in repo on GitHub is still at `366d30f`, and the team repo's main is unchanged at `211e644`. No file in `src/` or `install/` changed after the 14:12 check. The river_v6 model, both river worlds, the Python libraries and Gazebo 6.18 are in place. No PS4 controller was plugged in |
| Part 1 | 12/12 in 637 s (10.6 min) from sim start, 586 s of sim time. CPU 47 °C at the start, 95 to 98 °C during. 5 of the 6 natives logged, 0.2 to 0.9 m from their true positions (native_03 1.6 m; native_01 was logged a second time 1.4 m off; native_02 was not seen). 0 false, 0 sent to the rover. One Nav2 goal gave up and was retried from another side. 0 tracebacks, shutdown included. Fleet Debug showed all 3 cameras live |
| `bash demo/part2_gui.sh` | starts the status bridge and the GUI. The 2 tracebacks are `KeyboardInterrupt` from the Ctrl+C that ends the check |
| Part 2 driven run | READY in 18 s. Pause 60 s after Launch held 25 s (rover 0.000 m, quad 0.14 m; 2 heartbeat lapses per robot, 0.05 to 0.08 s). Scripted PS4 9/9. First collection 443 s after Launch. End Mission went back to the setup page and closed the simulation and the 3D view (0 viewer errors), with nothing left running. GUI, bridge and showcase 0 tracebacks; the mission log's 2 are YOLO nodes stopping on Ctrl+C |
| Start reliability, window unticked | 3 more Prepare-only starts with the CPU still hot from Part 2: READY in 18 to 19 s. 11 of 11 unticked starts today |

## Repeated test, 4 Nov 16:01 to 19:47: both parts over and over, no code changes

`rehearsal_scripts/soak.sh` ran Part 1, Part 2 and two Prepare-only starts per cycle, and `soak_verdict.py` marked
each part. Part 2 alternated: odd cycles paused 60 s after Launch (the demo plan); even cycles paused at the first
collection, so the PS4 takeover happened while the rover had a goal. Cycle 8 was one extra Part 2 to fill the
remaining time. Logs: `~/.ros/ammo_demo/rehearsal_1104/soak/` (`soak_summary.log` has one line per part).

| Cycle | Part 1 | Part 2 | Starts |
|---|---|---|---|
| 1 | PASS, 9.2 min, natives 6/6 | PASS, pause at 60 s: READY 19 s, quad 0.13 m, PS4 9/9, first collection 10.0 min | 19 s, 19 s |
| 2 | PASS, 10.9 min, 5/6, 2 Nav2 retries | WARN, pause at first collection (7.1 min): quad crept 0.53 m during 21 heartbeat lapses; every takeover worked | 19 s, 19 s |
| 3 | PASS, 10.3 min, 5/6 | PASS, 60 s: READY 19 s, quad 0.18 m, PS4 9/9, first collection 7.2 min | 20 s, 19 s |
| 4 | PASS, 11.0 min, 5/6 | PASS, first collection (8.0 min): quad 0.25 m, every takeover worked | 19 s, 19 s |
| 5 | PASS, 13.1 min, 5/6, 1 Nav2 retry | PASS, 60 s: READY 64 s (Nav2 fallback), quad 0.00 m, PS4 9/9, first collection 6.8 min | 19 s, 19 s |
| 6 | PASS, 11.1 min, 4/6 plus the known false native at the water's edge | WARN, first collection (6.7 min): quad crept 0.50 m during 19 lapses; every takeover worked | 19 s, 18 s |
| 7 | PASS, 12.3 min, 6/6, 2 Nav2 retries | PASS, 60 s: READY 20 s, quad 0.41 m (braking from 0.7 m/s), PS4 9/9, first collection 12.4 min (crossing retry) | 19 s, 19 s |
| 8 | (not run) | WARN, 60 s: READY 19 s, quad glided 0.9 m and stopped within 1.5 s, PS4 9/9, first collection 10.8 min | 18 s, 19 s |

No native was sent to the rover in any run. No traceback appeared during any mission, or in the GUI, bridge or
showcase; the ones in the logs are all nodes stopping at shutdown. Every End Mission returned to the setup page and
left nothing running.

Findings:
- Part 1 collected 12/12 in all 10 runs today, 9.2 to 13.3 min from sim start. The slow runs came from the mission
  itself (a slower first search, or Nav2 giving up on a spot and retrying from another side), with the sim at its
  usual speed (0.85 to 0.93x real time).
- Pause: the quad brakes to a stop within 1 to 2 s, gliding up to about 1 m if it was at full speed, then holds.
  While paused, the GUI's 5 Hz heartbeat runs on sim time, and the robots hold for 0.5 s of wall time after each beat
  (`time.monotonic()` in `zcam_quad_node.py` and `rover_capture_node.py`). With the sim at 0.45x real time, beats
  arrive about 0.43 s apart, so the hold lapses for 0.05 to 0.5 s many times, and the quad's search resumes for each
  lapse. Paused 6 to 8 min into a run, the quad crept 0.25 to 0.53 m in small steps over 25 s. Run 4 this morning
  had the GUI on wall-clock time and showed no lapses. Fix after the demo: run the GUI's stand-down timer on the wall
  clock, or raise `TELEOP_HZ` in `ammo_gui_node/contract.py`.
- PS4: every takeover and hand-back worked. Two scripted checks failed for test reasons. Right after the first
  collection the rover faces an obstacle 0.25 to 0.42 m away, and the collision guard blocks R1 forward (turning on
  the spot works). When TRIANGLE hands the rover back, it re-plans at once, before the check starts looking.
- Prepare: 24 of 24 unticked starts in this test reached READY, 23 in 18 to 20 s. One took 64 s: Nav2's lifecycle
  manager stalled configuring `controller_server`, and the rover's own fallback (`_start_nav2` in
  `rover_capture_node.py`) started Nav2 after 60 s. `DEMO_DAY.md` now says to wait in that case. Across the whole
  day, 35 of 35 unticked starts reached READY.
- River crossing: 3 of the day's 55 crossings timed out (1 in Part 1, 2 in Part 2). Each time the rover went back
  to Nav2 and crossed on the next try, at a cost of 1 to 2.5 min.
- Part 2 timing over 13 runs today: the quad's first find came 4.9 to 9.2 min after Launch, and the first
  collection 6.7 to 12.4 min after Launch.
