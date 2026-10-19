"""The one place topic names, mission states and their colours are defined.

Both windows and the bridge import from here, so renaming a topic is a
one-line change rather than a hunt through five files.

How the GUI learns what the robots are doing
--------------------------------------------
Both mission nodes now publish their own state, which they did not use to:

  * `husky1/rover/state`  - one word per transition, latched, added as a
                            single line inside `CageCapture._set_state`
  * `parrot1/quad_state`  - a small JSON block at 4 Hz from `ZCamQuad`

`mission_bridge` prefers those. When neither is publishing - an older build
of someone else's package, or a mission started before those changes were
installed - it falls back to reading `/rosout` exactly as it used to, so the
GUI degrades to the old behaviour instead of showing nothing.

Events are still read from the log, and that is the right place for them:
"the rover gave up at (3.1, -4.2)" is a sentence the node already writes
once, and nothing is gained by inventing a topic for each one.
"""

# -- our own contract -------------------------------------------------------
# std_msgs/String carrying JSON. String keeps this dependency-free: a custom
# .msg would need a second, CMake-based package just for two message types.
STATUS_TOPIC = 'ammo/mission/status'   # full snapshot, ~5 Hz
EVENTS_TOPIC = 'ammo/mission/events'   # one message per thing that happened
PAUSE_TOPIC = 'ammo/mission/pause'     # std_msgs/Bool, published by the GUI
COMMAND_TOPIC = 'ammo/mission/command'  # GUI -> bridge: start, end, operator goal

# -- state topics the mission nodes publish ---------------------------------
ROVER_STATE_TOPIC = 'husky1/rover/state'   # std_msgs/String, latched
QUAD_STATE_TOPIC = 'parrot1/quad_state'    # std_msgs/String carrying JSON

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

# cage_capture_node's queue input and its "finished with this one" outputs.
CAGE_WAYPOINTS = 'husky1/cage/waypoints'
CAGE_DONE = 'husky1/cage/done'
CAGE_MISSED = 'husky1/cage/missed'     # gripped, but nothing was there

# The quad's shared ground map. The rover routes on it, and its "known"
# fraction is the honest answer to "how much has the quadcopter seen?".
QUAD_GROUND_MAP = 'parrot1/ground_obstacles'

HUSKY_CMD = 'husky1/cmd_vel'          # what the Gazebo bridge actually relays
HUSKY_CMD_RAW = 'husky1/cmd_vel_raw'  # goes through collision_guard first
PARROT_CMD = 'parrot1/cmd_vel'

# Cancelling every Nav2 goal on pause. Sending a CancelGoal with a zero
# goal_id and zero stamp means "cancel everything you are running", which is
# the only way to stop a goal another node started.
NAV_CANCEL_SERVICE = 'husky1/navigate_to_pose/_action/cancel_goal'

# Where the rover's TF tree actually is. NOT /tf: `41068_ignition.launch.py`
# gives every robot its own - "TF topics are under /<namespace>/tf and
# /<namespace>/tf_static" - and `cage_mission.launch.py` remaps
# truth_localizer, collision_guard and Nav2 onto /husky1/tf. `husky1_map` is
# published there and nowhere else, so a readiness check watching the global
# /tf waits forever. Both are watched anyway, in case a future launch file
# stops namespacing them.
ROVER_TF_TOPIC = '/husky1/tf'
ROVER_TF_STATIC = '/husky1/tf_static'
GLOBAL_TF_TOPIC = '/tf'
GLOBAL_TF_STATIC = '/tf_static'

# The frame a Nav2 goal for the rover must be given in. NOT 'map': with
# truth_localizer running there is no bare `map` frame in the rover's TF
# tree, and a goal in one is silently unusable. This is the frame
# cage_capture_node and rover_capture_node both send their own goals in.
ROVER_MAP_FRAME = 'husky1_map'
# How long to let the collision guard forget the mission node's last command
# before sending our own goal. The guard replays whatever it last saw on
# cmd_vel_raw for RAW_CMD_TIMEOUT_S (0.5 s in collision_guard.hpp), so a goal
# sent immediately after freezing that node fights the replay.
GUARD_SETTLE_S = 0.7

ROSOUT = '/rosout'

# The temporary camera the GUI spawns above the world to take one top-down
# photo of the spawn, then removes again. Not part of anybody's launch file -
# the model and the bridge are both started and stopped by mission_launcher.
TOPDOWN_TOPIC = '/ammo/topdown/image'

# Gazebo's own clock, bridged by the team's launch. Its presence is the
# cheapest honest answer to "is the simulation actually up?".
CLOCK_TOPIC = '/clock'

# Nav2's action for the rover. The readiness check waits for this server
# rather than for a topic, because a goal sent before it is up is simply
# rejected - which is what used to look like the rover ignoring a click.
NAV_ACTION = 'husky1/navigate_to_pose'

# Node names as they appear in /rosout (set by each node's super().__init__).
# rover_capture_node subclasses cage_capture_node without renaming it, so the
# rover logs under 'cage_capture' either way.
NODE_CAGE = 'cage_capture'
NODE_QUAD = 'quad_search_controller'

# -- mission states ---------------------------------------------------------
# What the timeline is made of. Deliberately coarse: these are the phases an
# operator cares about, not the rover's internal nine-state machine.
IDLE = 'IDLE'
SEARCH = 'SEARCH'
FOUND = 'FOUND'
TRANSIT = 'TRANSIT'
FORD = 'FORD'
ALIGN = 'ALIGN'
CAPTURE = 'CAPTURE'
STUCK = 'STUCK'
OPERATOR = 'OPERATOR'
PAUSED = 'PAUSED'

STATE_ORDER = [IDLE, SEARCH, FOUND, TRANSIT, FORD, ALIGN, CAPTURE, STUCK,
               OPERATOR, PAUSED]

STATE_COLOURS = {
    IDLE: '#2E3A44',
    SEARCH: '#4F9CD6',
    FOUND: '#E8B33A',
    TRANSIT: '#6FBF73',
    FORD: '#3F8FBF',
    ALIGN: '#46A08A',
    CAPTURE: '#B07BD0',
    STUCK: '#C56A3A',
    OPERATOR: '#7A86D6',
    PAUSED: '#6E6A63',
}

# Short enough to fit the legend, plain enough to read at a glance.
STATE_LABELS = {
    IDLE: 'idle',
    SEARCH: 'searching',
    FOUND: 'mushroom found',
    TRANSIT: 'rover transit',
    FORD: 'crossing the river',
    ALIGN: 'aligning',
    CAPTURE: 'capturing',
    STUCK: 'stuck / backing out',
    OPERATOR: 'operator goal',
    PAUSED: 'paused',
}

# The longer line shown under the timeline while a state is running.
STATE_DETAIL = {
    IDLE: 'waiting for a mission',
    SEARCH: 'quadcopter flying its search pattern',
    FOUND: 'quadcopter holding over a sighting',
    TRANSIT: 'rover driving to the mushroom',
    FORD: 'rover driving a ford lane across the river',
    ALIGN: 'rover lining up for the grab',
    CAPTURE: 'rover collecting the mushroom',
    STUCK: 'rover boxed in - reversing out',
    OPERATOR: 'rover heading to the goal you set',
    PAUSED: 'robots held - display frozen',
}

# The rover's own states -> ours. LOWER/COLLECT/RETRACT/RAISE are all "the
# arm is doing something", which is one phase to an operator. FORD and
# UNWEDGE are worth their own colour: they are the two places a river run
# visibly differs from a forest run.
ROVER_STATE_MAP = {
    'DRIVE': TRANSIT,
    'NAV': TRANSIT,
    'FORD': FORD,          # rover_capture_node: crossing the river
    'UNWEDGE': STUCK,      # rover_capture_node: backing out of a stall
    'ALIGN': ALIGN,
    'LOWER': CAPTURE,
    'COLLECT': CAPTURE,
    'RETRACT': CAPTURE,
    'RAISE': CAPTURE,
}

ROVER_STATE_DETAIL = {
    'DRIVE': 'picking a route',
    'NAV': 'driving to the standoff point',
    'FORD': 'crossing the river at a ford',
    'UNWEDGE': 'backing out of a stall',
    'ALIGN': 'lining up on the mushroom',
    'LOWER': 'gripper down',
    'COLLECT': 'closing on the mushroom',
    'RAISE': 'lifting clear',
    'RETRACT': 'backing off to try again',
}

# -- the quadcopter's phases ------------------------------------------------
# These are the phases ZCamQuad actually implements, rather than the
# "moving or not moving" guess the bridge used to make. Resolved in
# priority order by the node itself and published as `phase`.
QUAD_NO_ODOM = 'NO TELEMETRY'
QUAD_TAKEOFF = 'CLIMBING'
QUAD_SEARCHING = 'SEARCHING'
QUAD_APPROACH = 'APPROACHING'
QUAD_HOLDING = 'HOLDING'
QUAD_DISPATCH = 'HANDING OVER'
QUAD_RETRY_VIEW = 'NEW VIEWPOINT'
QUAD_SLIDE = 'SLIDING PAST'
QUAD_LOOK = 'PEERING IN'
QUAD_ESCAPE = 'BACKTRACKING'
QUAD_CLIMB = 'CLIMBING CLEAR'
QUAD_IDLE = 'IDLE'
QUAD_WAITING = 'WAITING'      # kept: older bridges published it

QUAD_PHASE_DETAIL = {
    QUAD_NO_ODOM: 'no odometry from the quadcopter yet',
    QUAD_TAKEOFF: 'climbing to cruise height',
    QUAD_SEARCHING: 'flying the search pattern',
    QUAD_APPROACH: 'closing on a sighting to look properly',
    QUAD_HOLDING: 'holding still over the mushroom',
    QUAD_DISPATCH: 'passing the position to the rover',
    QUAD_RETRY_VIEW: 'no clear view - trying another angle',
    QUAD_SLIDE: 'sliding sideways past an obstacle',
    QUAD_LOOK: 'looking into a gap from open air',
    QUAD_ESCAPE: 'retracing its path out of a dead end',
    QUAD_CLIMB: 'climbing to clear an obstacle',
    QUAD_IDLE: 'no mission running',
    QUAD_WAITING: 'holding position',
}

# Which mission state a quad phase implies, when the rover has nothing to say.
QUAD_PHASE_MAP = {
    QUAD_APPROACH: FOUND,
    QUAD_HOLDING: FOUND,
    QUAD_DISPATCH: FOUND,
    QUAD_RETRY_VIEW: FOUND,
}

# -- per-robot readouts -----------------------------------------------------
ROVER_IDLE = 'IDLE'

# -- event kinds ------------------------------------------------------------
EV_MISSION_START = 'mission_start'
EV_MISSION_END = 'mission_end'
EV_FOUND = 'found'
EV_COLLECTED = 'collected'
EV_ABANDONED = 'abandoned'
EV_MISSED = 'missed'
EV_NATIVE = 'native'
EV_OPERATOR_GOAL = 'operator_goal'
EV_STATE = 'state'
EV_PAUSE = 'pause'
EV_RESUME = 'resume'
EV_NOTE = 'note'
# river and recovery, all read from lines rover_capture_node already writes
EV_FORD = 'ford'
EV_FORD_FAIL = 'ford_fail'
EV_WEDGED = 'wedged'
EV_STALL = 'stall'
EV_EMPTY = 'empty'
EV_GAVE_UP_HERE = 'gave_up_here'
EV_REROUTE = 'reroute'
EV_RETRY_PHASE = 'retry_phase'
EV_COVERAGE = 'coverage'
EV_GOAL_FAILED = 'goal_failed'
EV_DETECTOR = 'detector'        # which classes the detector was loaded with

# How the Debug UI colours a line in the event feed.
SEV_GOOD = 'good'
SEV_INFO = 'info'
SEV_WARN = 'warn'
SEV_BAD = 'bad'

EVENT_SEVERITY = {
    EV_MISSION_START: SEV_INFO,
    EV_MISSION_END: SEV_INFO,
    EV_FOUND: SEV_INFO,
    EV_COLLECTED: SEV_GOOD,
    EV_ABANDONED: SEV_BAD,
    EV_MISSED: SEV_WARN,
    EV_NATIVE: SEV_GOOD,
    EV_OPERATOR_GOAL: SEV_INFO,
    EV_STATE: SEV_INFO,
    EV_PAUSE: SEV_WARN,
    EV_RESUME: SEV_INFO,
    EV_NOTE: SEV_INFO,
    EV_FORD: SEV_INFO,
    EV_FORD_FAIL: SEV_WARN,
    EV_WEDGED: SEV_WARN,
    EV_STALL: SEV_WARN,
    EV_EMPTY: SEV_WARN,
    EV_GAVE_UP_HERE: SEV_BAD,
    EV_REROUTE: SEV_INFO,
    EV_RETRY_PHASE: SEV_INFO,
    EV_COVERAGE: SEV_INFO,
    EV_GOAL_FAILED: SEV_BAD,
    EV_DETECTOR: SEV_INFO,
}

# -- detections drawn over the camera feeds ---------------------------------
# The GUI draws the boxes itself from `husky1/detections` and
# `parrot1/detections` rather than showing yolo_node's annotated image,
# because that image is published at 2 Hz and looks like a slideshow next to
# the raw camera. Every Detection2D carries the box in pixels plus the class
# name and the score, so nothing is lost by drawing it ourselves.
#
# A class name not listed here is drawn in the default colour, so a model
# with new classes still shows its boxes.
DET_COLOURS = {
    'mushroom_amanita': '#D63434',   # the invasive one, the thing to collect
    'amanita': '#D63434',
    'mushroom': '#D63434',
    'native_mushroom': '#46A08A',    # identified and deliberately left alone
    'native': '#46A08A',
    'person': '#E8B33A',
}
DET_DEFAULT_COLOUR = '#4F9CD6'
# Boxes arrive at the detector's rate (2 Hz), the video at the camera's.
# Hold the last set briefly so they do not flicker, and fade them when they
# are clearly older than the frame underneath.
DET_HOLD_S = 1.6
DET_FADE_S = 0.8

# Events that are only noise in a feed a human reads.
FEED_HIDDEN = {EV_STATE, EV_COVERAGE}

# -- map markers ------------------------------------------------------------
# Runtime points worth remembering on the map. The key is the marker kind as
# the bridge publishes it; the map draws a symbol per kind and the legend
# lists only the kinds actually present.
MK_FORD = 'ford_cross'
MK_WEDGE = 'wedged'
MK_EMPTY = 'empty'
MK_GAVE_UP = 'gave_up'
MK_NATIVE = 'native'

MARKER_LABELS = {
    MK_FORD: 'ford crossing',
    MK_WEDGE: 'boxed in',
    MK_EMPTY: 'nothing there',
    MK_GAVE_UP: 'Nav2 gave up',
    MK_NATIVE: 'native - left alone',
}

# -- world-file symbols -----------------------------------------------------
# The `type` values that appear in a world's .meta.json. The two generators
# in the repo disagree: the river builder writes trees only (always radius
# 0.3), while the forest builder writes nine types with real radii, lengths
# and yaws. Everything here is optional, so a thinner meta simply draws
# fewer symbols.
OBJ_LABELS = {
    'tree': 'tree',
    'fallen_tree': 'fallen tree',
    'log': 'log',
    'stump': 'stump',
    'rock': 'rock',
    'clutter': 'debris',
    'pond': 'pond',
    'mushroom': 'mushroom in the world file',
    'decoy': 'decoy',
}

# Drawn in this order, so small things end up on top of big ones.
OBJ_ORDER = ['pond', 'tree', 'fallen_tree', 'log', 'stump', 'rock', 'clutter',
             'decoy', 'mushroom']

# -- timeline ---------------------------------------------------------------
# Two minutes a row rather than one: a full river run is fifteen minutes or
# more, and thirty rows will not fit on a screen.
TIMELINE_ROW_S = 120.0

# Two sightings closer than this are the same mushroom. Matches
# cage_capture_node's own MERGE_M so our count agrees with its queue.
MERGE_M = 1.0

# A native mushroom reported twice within this is the same one.
NATIVE_MERGE_M = 1.5
