from _bag import bag
from geometry_msgs.msg import Pose, Twist

_PoseWithCov = bag('PoseWithCovariance', pose=Pose)
_TwistWithCov = bag('TwistWithCovariance', twist=Twist)
Odometry = bag('Odometry', header=None, pose=_PoseWithCov, twist=_TwistWithCov)

MapMetaData = bag('MapMetaData', resolution=0.25, width=0, height=0, origin=Pose)
OccupancyGrid = bag('OccupancyGrid', header=None, info=MapMetaData, data=list)
