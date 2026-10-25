from _bag import bag

Image = bag('Image', height=0, width=0, encoding='rgb8', step=0, data=bytes,
            is_bigendian=0, header=None)
LaserScan = bag('LaserScan', ranges=list, angle_min=0.0, angle_increment=0.01,
                range_min=0.1, range_max=30.0)
JointState = bag('JointState', name=list, position=list)
