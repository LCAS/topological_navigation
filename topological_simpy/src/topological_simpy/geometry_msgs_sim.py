#! /usr/bin/env python


# fake ROS PoseStamped
class Pose():
    def __init__(self):
        self.point = {k: float() for k in ['x', 'y', 'z']}
        self.quaternion = {k: float() for k in ['x', 'y', 'z', 'w']}
        self.pose = {'point': self.point, 'quaternion': self.quaternion}
