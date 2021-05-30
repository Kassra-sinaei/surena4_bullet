import numpy as np
import math


class Ankle():
    def ankle(self, step_time, ds_time, height, alpha = 0.5, num_step = 6, dt = 1/240):
        self.tStep_ = step_time
        self.tDS_ = ds_time
        self.dt_ = dt
        self.alpha = alpha
        self.stepCount = num_step
        self.leftFirst = True
        self.height_ = height

        pass

    def updateFoot(self,foot_pose):

        pass
    
    def generateTrajectory(self):

        pass

    def getTrajectoryLR(self):
        
        return 

    def getTrajectoryL(self):
        
        return

if __name__ == "__main__":
    anklePlanner = Ankle(1.0, 0.3, 0.05)
    rF =np.array([[0.0,0.115,0.0],
                 [0.0,-0.115,0.0],
                 [0.4,0.115,0.0],
                 [0.8,-0.115,0.0],
                 [1.2,0.115,0.0],
                 [1.6,-0.115,0.0],
                 [2.0,0.115,0.0],
                 [2.0,-0.115,0.0]])
    anklePlanner.updateFoot()
    pass