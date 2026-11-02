# Sourced by the demo scripts: ROS, the built workspace and the river world models on Gazebo's path.
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=${AMMO_WS:-$(cd "$REPO/../.." && pwd)}
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "No built workspace at $WS. Clone the repo into <ws>/src and build, or set AMMO_WS." >&2
    return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
F=$REPO/forest_worlds
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 ROS_LOG_DIR=${ROS_LOG_DIR:-/dev/shm/ammo_ros_log}
export IGN_GAZEBO_RESOURCE_PATH="$(ls -d $F/models/* $F/world_assets 2>/dev/null | tr '\n' ':')${IGN_GAZEBO_RESOURCE_PATH:-}"
LOGS=${AMMO_LOGS:-$HOME/.ros/ammo_demo}
mkdir -p "$LOGS"
