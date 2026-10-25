from _bag import Header, Quaternion, Vector3, bag

Transform = bag('Transform', translation=Vector3, rotation=Quaternion)
TransformStamped = bag('TransformStamped', header=Header, child_frame_id='',
                       transform=Transform)
TFMessage = bag('TFMessage', transforms=list)
