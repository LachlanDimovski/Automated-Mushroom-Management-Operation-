from _bag import bag
from geometry_msgs.msg import PoseStamped

NavigateToPose = bag('NavigateToPose')
NavigateToPose.Goal = bag('NavigateToPoseGoal', pose=PoseStamped)
