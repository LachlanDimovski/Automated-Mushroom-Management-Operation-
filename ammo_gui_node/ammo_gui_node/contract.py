"""The one place topic names, mission states and their colours are defined.

Both windows and the bridge import from here, so renaming a topic is a
one-line change rather than a hunt through five files.

Why a bridge node exists at all
-------------------------------
The two mission nodes (`quad_search_node`, `cage_capture_node`) do not
publish their state anywhere - `cage_capture_node` keeps a state string but
only ever writes it to its own log, and `quad_search_node` has no state
variable at all. Rather than edit either file (they belong to the rest of
the team), `mission_bridge` reconstructs the mission state from things that
are already on the wire - `/rosout`, `husky1/cage/waypoints`,
`husky1/cage/done`, odometry - and republishes it as the small JSON
contract below.

That means the GUI and the Debug UI only ever read AMMO_* topics. If the
team later adds real status publishing to their nodes, the bridge's
ingest side gets deleted and nothing in either window has to change.
"""

# -- our own contract -------------------------------------------------------
# std_msgs/String carrying JSON. String keeps this dependency-free: a custom
# .msg would need a second, CMake-based package just for two message types.
STATUS_TOPIC = 'ammo/mission/status'   # full snapshot, ~5 Hz
EVENTS_TOPIC = 'ammo/mission/events'   # one message per thing that happened
PAUSE_TOPIC = 'ammo/mission/pause'     # std_msgs/Bool, published by the GUI
COMMAND_TOPIC = 'ammo/mission/command'  # GUI -> bridge: start, end, operator goal

# -- topics that already exist in the repo ----------------------------------
HUSKY_ODOM = 'husky1/odometry'
PARROT_ODOM = 'parrot1/odometry'
HUSKY_SCAN = 'husky1/scan'

HUSKY_CAM = 'husky1/camera/image'
HUSKY_CAM_YOLO = 'husky1/detections/image'
PARROT_CAM = 'parrot1/camera/image'
PARROT_CAM_YOLO = 'parrot1/detections/image'
HUSKY_DETECTIONS = 'husky1/detections'
PARROT_DETECTIONS = 'parrot1/detections'

# cage_capture_node's queue input and its "finished with this one" output.
CAGE_WAYPOINTS = 'husky1/cage/waypoints'
CAGE_DONE = 'husky1/cage/done'

HUSKY_CMD = 'husky1/cmd_vel'          # what the Gazebo bridge actually relays
HUSKY_CMD_RAW = 'husky1/cmd_vel_raw'  # goes through collision_guard first
PARROT_CMD = 'parrot1/cmd_vel'
HUSKY_TELEOP = 'husky1/teleop_active'  # std_msgs/Bool heartbeat: the rover's mission stands down while it arrives

# Cancelling every Nav2 goal on pause. Sending a CancelGoal with a zero
# goal_id and zero stamp means "cancel everything you are running", which is
# the only way to stop a goal another node started.
NAV_CANCEL_SERVICE = 'husky1/navigate_to_pose/_action/cancel_goal'

ROSOUT = '/rosout'

# Node names as they appear in /rosout (set by each node's super().__init__).
NODE_CAGE = 'cage_capture'
NODE_QUAD = 'quad_search_node'

# -- mission states ---------------------------------------------------------
# What the timeline is made of. Deliberately coarse: these are the phases an
# operator cares about, not the rover's internal seven-state machine.
IDLE = 'IDLE'
SEARCH = 'SEARCH'
FOUND = 'FOUND'
TRANSIT = 'TRANSIT'
ALIGN = 'ALIGN'
CAPTURE = 'CAPTURE'
OPERATOR = 'OPERATOR'
PAUSED = 'PAUSED'

STATE_ORDER = [IDLE, SEARCH, FOUND, TRANSIT, ALIGN, CAPTURE, OPERATOR, PAUSED]

STATE_COLOURS = {
    IDLE: '#2E3A44',
    SEARCH: '#4F9CD6',
    FOUND: '#E8B33A',
    TRANSIT: '#6FBF73',
    ALIGN: '#46A08A',
    CAPTURE: '#B07BD0',
    OPERATOR: '#7A86D6',
    PAUSED: '#6E6A63',
}

# Short enough to fit the legend, plain enough to read at a glance.
STATE_LABELS = {
    IDLE: 'idle',
    SEARCH: 'searching',
    FOUND: 'mushroom found',
    TRANSIT: 'rover transit',
    ALIGN: 'aligning',
    CAPTURE: 'capturing',
    OPERATOR: 'operator goal',
    PAUSED: 'paused',
}

# The longer line shown under the timeline while a state is running.
STATE_DETAIL = {
    IDLE: 'waiting for a mission',
    SEARCH: 'quadcopter flying its search pattern',
    FOUND: 'quadcopter holding over a sighting',
    TRANSIT: 'rover driving to the mushroom',
    ALIGN: 'rover lining up for the grab',
    CAPTURE: 'rover collecting the mushroom',
    OPERATOR: 'rover heading to the goal you set',
    PAUSED: 'robots held - display frozen',
}

# cage_capture_node's own states -> ours. Its RETRACT/RAISE/LOWER/COLLECT are
# all "the arm is doing something", which is one phase to an operator.
ROVER_STATE_MAP = {
    'DRIVE': TRANSIT,
    'NAV': TRANSIT,
    'CROSSING': TRANSIT,       # rover_capture_node: crossing the river
    'UNWEDGE': TRANSIT,    # rover_capture_node: backing out of a stall
    'TELEOP': OPERATOR,    # rover_capture_node: driven by hand (PS4 R1 held)
    'ALIGN': ALIGN,
    'LOWER': CAPTURE,
    'COLLECT': CAPTURE,
    'RETRACT': CAPTURE,
    'RAISE': CAPTURE,
}

# -- per-robot states (shown as a plain readout, not on the timeline) --------
ROVER_IDLE = 'IDLE'
QUAD_SEARCHING = 'SEARCHING'
QUAD_HOLDING = 'HOLDING'
QUAD_WAITING = 'WAITING'
QUAD_IDLE = 'IDLE'

# -- event kinds ------------------------------------------------------------
EV_MISSION_START = 'mission_start'
EV_MISSION_END = 'mission_end'
EV_FOUND = 'found'
EV_COLLECTED = 'collected'
EV_ABANDONED = 'abandoned'
EV_OPERATOR_GOAL = 'operator_goal'
EV_STATE = 'state'
EV_PAUSE = 'pause'
EV_RESUME = 'resume'
EV_NOTE = 'note'

# Two sightings closer than this are the same mushroom. Matches
# cage_capture_node's own MERGE_M so our count agrees with its queue.
MERGE_M = 1.0
