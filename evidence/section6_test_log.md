# Section 6 and README: every line tested (6 Nov 2026)

Blank system: a fresh Ubuntu 22.04.5 base in a sandbox on the SSD. Each terminal was a separate interactive shell with nothing sourced, and lines were typed one at a time. The final install block was read straight from the README. Repo `main` at `e50405a`.

Sandbox limits: it ran as root, `sudo` was a stand-in, and it ran on the team laptop.

## Install (Terminal 1, blank system): all pass
| Line | Result |
|---|---|
| ROS apt source, 5 lines | Pass |
| `sudo apt install ...` | Pass, 7 min |
| `pip3 install ...` | Pass, 2 min |
| `gh auth login` | Pass. Prompts answered with the existing token, so the browser route was not used |
| `gh auth setup-git`, `source`, `mkdir`, `git lfs install`, `git clone` | Pass |
| `colcon build` | Pass, 10 packages, 20 s |
| `bash demo/get_world.sh`, and with a file | Pass |
| `check_env.py` | Pass. Fails in a new terminal until `source demo/env.sh` |
| `run_tests.py` | Pass, 663 checks, no ROS needed |
| `git lfs pull` | Pass |

## Scenarios
| Line | Terminal | Result |
|---|---|---|
| `bash demo/part1_fleet.sh` | 1 | Pass, 12 of 12: 665 s blank, 748 s laptop |
| `bash demo/part1_timer.sh` | 2 | Pass |
| `bash demo/stop_sim.sh` | 1 after the run; 2 or 3 otherwise | Pass, CLEAN |
| `QUADS=3 ... row_spacing:=4.0`, `WORLD=...m1`, `HEADLESS=false`, `ROVERS=2` | 1 | Pass, started and ran 70-80 s |
| `bash demo/part2_gui.sh`, steps 1-5 and 7 | 1 | Pass |
| `source demo/env.sh && ign gazebo -g` | 2 | Pass |
| Step 6 PS4 | | Scripted `/joy` only |
| `powerprofilesctl set performance` | 1 | Pass on the laptop only; the sandbox has no such service |

## By hand: all started and ran 15-75 s, none run to mission end
`cage_mission` (3 printed lines, plus `quad:=stock`, `tool:=cage`, `model_path:=`), `ammo_main`, `fleet_mission quads:=1 rovers:=1`, `zcam_mission ps4:=false`, fleet by hand with `fleet_debug.launch.py`, `fleet_debug` alone, `gui.launch.py`, dashboard (both forms), `quad_mission_node`, `ps4_teleop inspect`, world generator.

iPad: apt line, rosbridge, web_video_server and the page server pass; page and video ports returned 200. The page was not opened on an iPad.

`quad_mission` with a sim, three terminals: `liftoff` raised the quadcopter from 0.8 m to 1.7 m. The other services were not called.

`bash forest_worlds/setup.sh`: finished on the laptop with Blender 5.2, in a copy, in under 10 minutes. The built worlds were not flown.

`41068_ignition_bringup/README.md`: all install lines, all launch lines, teleop, dynamic world and autonomy demos pass.

`run_capture.sh --hover 1.0`: ran for 100 s, recorded 10 frames and kept 0 boxes on that short default run. The README gives this line as how the data was captured.

## Package READMEs
Every command was run on the blank system, except where noted.

| Doc | Result |
|---|---|
| `worlds/README_forest_random.md` | Generator and all five launches pass |
| `ammo_gui_node/README.md`, `TEAM_SETUP.md` | `debug.launch.py`, `gui.launch.py`, `--render`, `get_world.sh` pass. Unpinned pip lines and the `src/RS1_Team5_/` path are now fixed |
| `quad_showcase/README.md` | Pass beside a live GUI mission |
| `perception/models/variants/README.md` | Model swap and rebuild pass. Table updated to all 7 models |
| `forest_worlds/README.md` | `make_gums.py`, `build_world.py` (3 forms) pass on the laptop. `git checkout mushroom-cage` failed: removed. `snap install blender` was not run; Blender was already on the laptop |
| `view.sh` | Failed on every mushroom world: fixed, pass with `moderate` and `moderate_aus` |
| `demo_river.sh` | Mission never started: fixed, pass |
| `README_MUSHROOM.md` | `nm`, `ign model --list`, `pip install`, rebuild pass. Launch line and path fixed, and the new line passes. The Sketchfab source and licence are still blank |
| `TRAINING_PC.md` | venv, `pip install ultralytics` and the torch check pass. `train_mushroom.py` stops without the dataset bundle and weights, which are not in the repo. Needs a GPU PC |

## Fixed because of these tests
- `git clone` before login; `colcon build` with ROS not sourced.
- Unpinned `pip3` pulled torch 2.14 and setuptools 84, which broke `colcon build`. Now pinned.
- Three `ros-humble-ros-ign-*` packages, `gh`, colcon, pip, `ros-humble-desktop` and `ignition-fortress` were missing.
- ROS apt source was described but not given. iPad lines could not run as printed.
- Step 7's `stop_sim.sh` needs Terminal 3. `check_env.py` needs `source demo/env.sh`. Add-on demos need a running sim.

## Extra tests, 6 Nov evening
Run on the laptop at the same commit, for Section 5.1 of the report. Data: `final_tests_2026-11-06/`.

| Test | Result |
|---|---|
| R1 detection range, 240 placements | Clear-view recall at 0.25 confidence: 100%, 100%, 97%, 68% at 1, 2, 3, 4 m. At 0.6: 100%, 98%, 91%, 60% |
| R12 guard stop, 24 runs | No contact. Closest 0.91 m (low box, Nav2 input, 1.0 m/s); the other 23 stayed 1.07 m or more away |
| Look-alike decoys, one 2 + 1 run, 1250 s cap | All 6 decoys reported, 3 repeat reports, 12 of 12 real mushrooms reported, 6 of 12 collected, stalled |

Two first attempts were discarded and rerun: R1 fed RGB frames to YOLO, which needs BGR, and some R12 lanes ran off the 15 m grass plane.

## Still untested
- A real PS4 controller (R11).
- The Table 32 failures, which were not provoked.
- A non-root user on a real Ubuntu desktop.
