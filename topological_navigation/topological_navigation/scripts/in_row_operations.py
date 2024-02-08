import math
import numpy as np  
from geometry_msgs.msg import PoseStamped

class RowOperations:
    def __init__(self, initial_edges, step_size=2.0):
        self.initial_edges = initial_edges
        self.initial_path = False
        self.step_size = step_size
        if len(self.initial_edges) > 1:
            self.terminal_pose = self.getPoseSE2(self.initial_edges[-1])
            self.getApproximatedLine()
        else:
            print("Can not creat a projeted path...")
            self.initial_path = False
        
    def isPlanCalculated(self, ):
        return self.initial_path

    def shortestDistance(x1, y1, a, b, c): 
        d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b))
        return d 
    
    def getPoseSE2(self, point):
        return np.array([point.pose.position.x, point.pose.position.y])

    def getSE2Pose(self, point, orientation):
        target_pose = PoseStamped()
        target_pose.pose.position.x = point[0]
        target_pose.pose.position.y = point[1]
        target_pose.pose.position.z = 0.3
        target_pose.pose.orientation = orientation
        return target_pose

    def getApproximatedLine(self):
        """
        (a,b,c) to represent the linear equation ax+by+c=0.
        """
        p1 = self.getPoseSE2(self.initial_edges[0])
        p2 = self.terminal_pose
        self.a = p2[1] - p1[1]
        self.b = p1[0] - p2[0]
        self.c = -1.0*(self.a * (p1[0]) + self.b * (p1[1]))
        self.initial_path = True 
    
    def vecScale(self, v, a):
        return (a * v[0], a * v[1])

    def vecMulAdd(self, v1, v2, a=1):
        """calc v1+a*v2"""
        return (v1[0] + a * v2[0], v1[1] + a * v2[1])

    def innerProd(self, v1, v2):
        """v1^T * v2"""
        return v1[0] * v2[0] + v1[1] * v2[1]

    def getClosestPoint(self, p):
        """
        Params:
            coef: 3-tuple (a,b,c) to represent the linear equation ax+by+c=0.
            p: 2-tuple (px,py) to represent the point.
            https://math.stackexchange.com/questions/62633/orthogonal-projection-of-a-point-onto-a-line
        """
        a = self.a 
        b = self.b 
        c = self.c
        # find p0 at the line
        if b == 0:
            p0 = (-c / a, 0)
        else:
            p0 = (0, -c / b)
        if a == 0:
            v = (1, 0)
        elif b == 0:
            v = (0, 1)
        else:
            v = (b, -a)
        target_pose = self.vecMulAdd(
            p0, self.vecScale(v, self.innerProd(self.vecMulAdd(p, p0, -1), v) / self.innerProd(v, v))
        )
        return np.array([target_pose[0], target_pose[1]])

    def getNextGoal(self, current_pose_original):
        current_pose = self.getPoseSE2(current_pose_original)
        closest_pose = self.getClosestPoint(current_pose)
        norm_dis = np.linalg.norm(self.terminal_pose-closest_pose)
        updated_pose = None
        if(norm_dis < self.step_size):
            updated_pose = self.getSE2Pose(self.terminal_pose, current_pose_original.pose.orientation)
            return updated_pose, True 
        else:
            updated_pose = current_pose + self.step_size*((self.terminal_pose-closest_pose)/norm_dis)
            updated_pose = self.getSE2Pose(updated_pose, current_pose_original.pose.orientation) 
            return updated_pose, False
