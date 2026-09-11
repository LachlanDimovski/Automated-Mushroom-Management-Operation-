# RS1_Team5_
Robotics Studio 1. Master Repository for Team 5. 


AMMO -> Automated Mushroom Management Operations 


<img width="1254" height="1254" alt="Adobe Express - file" src="https://github.com/user-attachments/assets/64d25d77-6fab-4a71-aa8a-e338860c7090" />

## Node topology: what runs with what

Several nodes are alternatives to each other, not additions. Running two from
the same group together makes them fight over the same topic. Pick exactly
one per group:

**Husky drive command (publish `husky1/cmd_vel_raw`):**
`skidsteer_controller_node` | `quad_follow_node` | `mushroom_mission_node` | `teleop_twist_keyboard` (remapped)

**Husky safety gate (subscribe `husky1/cmd_vel_raw`, publish `husky1/cmd_vel` - always run exactly one alongside whichever drive node above is active):**
`collision_guard_node` (skidsteer_controller, C++) | `cbf_guard_node` (skid_cbf_guard, Python)

**Quad flight control (publish `parrot1/cmd_vel`):**
`ps4_drone_test.py` (manual) | `quad_search_node` (autonomous)

**Dashboard (view only, no topic conflict but redundant to run both):**
`dashboard.py` | `objective_trail_dashboard.py`

The `ammo_mission_control/launch/*.launch.py` files bundle the correct
combination for a given scenario (e.g. `autonomous.launch.py` runs
`quad_search_node` + `mushroom_mission_node` + a safety gate) - prefer
launching one of those over assembling nodes by hand.
