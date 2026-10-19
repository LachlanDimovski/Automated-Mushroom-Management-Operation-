from _bag import Header, Point, Quaternion, Vector3, bag

Twist = bag('Twist', linear=Vector3, angular=Vector3)
Pose = bag('Pose', position=Point, orientation=Quaternion)
PoseStamped = bag('PoseStamped', header=Header, pose=Pose)
PointStamped = bag('PointStamped', header=Header, point=Point)
