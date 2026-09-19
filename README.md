# RS1_Team5_
Robotics Studio 1. Master Repository for Team 5. 


AMMO -> Automated Mushroom Management Operations 


<img width="1254" height="1254" alt="Adobe Express - file" src="https://github.com/user-attachments/assets/64d25d77-6fab-4a71-aa8a-e338860c7090" />

## Running the mission

One launch runs everything - headless Gazebo, Nav2, ground-truth localiser,
the rover's cage/gripper node, and (with `parrot:=true`) the quad's search
flight and detection bridge:

    ros2 launch ammo_mission_control cage_mission.launch.py world:=simple_trees headless:=true \
        mushrooms:=6,-5,-5,-5.5,-2,6.5          # demo mushrooms; the quad has to find them
    ros2 launch ammo_mission_control cage_mission.launch.py world:=forest_flat headless:=true
    ros2 launch ammo_mission_control cage_mission.launch.py parrot:=false count:=4   # rover-only test

Flow: quad flies its legs (lawnmower over the world, or `quad_waypoints:=x,y,...`),
detouring round known trunks and lidar returns -> YOLO hit -> quad holds at a
canopy-clear point facing it -> `quad_waypoint_bridge` ray-casts the pixel to a
ground point once the quad is still -> `husky1/cage/waypoints` -> rover Nav2 to a
trunk-clear standoff, odometry creep, front gripper -> `husky1/cage/done` -> quad
resumes.

Topic ownership (never run two publishers of the same one):

- `husky1/cmd_vel_raw` -> `collision_guard_node` -> `husky1/cmd_vel`: `cage_capture_node`
  (transit), teleop (`ipad_teleop.html`, keyboard). Nav2 and the final approach write
  `husky1/cmd_vel` directly.
- `parrot1/cmd_vel`: `quad_search_node` (autonomous) or `ps4_drone_test.py` (manual).

Viewing (cheap, no Gazebo GUI): `ros2 run ammo_dashboard dashboard` - one OpenCV
HUD with both YOLO feeds, speed/altitude and a minimap; it also publishes itself
so the iPad page can show it. iPad: `rosbridge_websocket`, `web_video_server`,
`python3 ipad_teleop_server.py`, then `http://<pc>:8000/` (see `ipad_teleop.html`).
