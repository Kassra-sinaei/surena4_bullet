import numpy as np
import math
from matplotlib import pyplot as plt

class Ankle:
    def __init__(self, step_time, ds_time, height, dt, alpha = 0.5, num_step = 6):
        self.tStep_ = step_time
        self.tDS_ = ds_time
        self.dt_ = dt
        self.alpha_ = alpha
        self.stepCount_ = num_step
        self.leftFirst_ = True
        self.height_ = height

        pass

    def updateFoot(self,foot_pose):
        self.footPose_ = foot_pose
        if self.footPose_[0,1] < 0:
            self.leftFirst_ = False # left foot swings first
        pass
    
    def generateTrajectory(self):
        self.lFoot_ = list("")
        self.rFoot_ = list("")

        if (self.leftFirst_):       # left foot swings first
            for step in range(1, self.stepCount_+1):
                if step % 2 == 0:   #  left is support, right swings
                    for time in np.arange(0.0, (1-self.alpha_) * self.tDS_, self.dt_):
                        self.lFoot_.append(self.footPose_[step])
                        self.rFoot_.append(self.footPose_[step - 1])
                        
                    coefs = self.poly5(self.footPose_[step-1],self.footPose_[step+1], self.height_,self.tStep_-self.tDS_)
                    for time in np.arange(0.0, self.tStep_ - self.tDS_, self.dt_):
                        self.lFoot_.append(self.footPose_[step])
                        self.rFoot_.append(coefs[0] + coefs[1] * time + coefs[2] * time**2 + coefs[3] * time**3 + coefs[4] * time**4 + coefs[5] * time**5)
                        
                    for time in np.arange(0.0,(self.alpha_) * self.tDS_, self.dt_):
                        self.lFoot_.append(self.footPose_[step])
                        self.rFoot_.append(self.footPose_[step + 1])
                        
                else:
                    for time in np.arange(0.0, (1-self.alpha_) * self.tDS_, self.dt_):
                        self.lFoot_.append(self.footPose_[step - 1])
                        self.rFoot_.append(self.footPose_[step])
                        
                    coefs = self.poly5(self.footPose_[step-1],self.footPose_[step+1], self.height_,self.tStep_-self.tDS_)
                    for time in np.arange(0.0, self.tStep_ - self.tDS_, self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(coefs[0] + coefs[1] * time + coefs[2] * time**2 + coefs[3] * time**3 + coefs[4] * time**4 + coefs[5] * time**5)
                        
                    for time in np.arange(0.0,self.alpha_ * self.tDS_,self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(self.footPose_[step + 1])
                        
        else:         # right foot swings first
            for step in range(1, self.stepCount+1):
                if step % 2 == 0:   
                    for time in np.arange(0.0, (1-self.alpha_ * self.tDS_), self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(self.footPose_[step - 1])
                        
                    coefs = self.poly5(self.footPose_[step-1],self.footPose_[step+1], self.height_,self.tStep_-self.tDS_)
                    for time in np.arange(0.0, self.tStep_ - self.tDS_, self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(coefs[0] + coefs[1] * time + coefs[2] * time**2 + coefs[3] * time**3 + coefs[4] * time**4 + coefs[5] * time**5)
                        
                    for time in np.arange(0.0,(self.alpha) * self.tDS_,self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(self.footPose_[step + 1])
                        
                else:
                    for time in np.arange(0.0, (1-self.alpha_) * self.tDS_, self.dt_):
                        self.lFoot_.append(self.footPose_[step - 1])
                        self.rFoot_.append(self.footPose_[step])
                        
                    coefs = self.poly5(self.footPose_[step-1],self.footPose_[step+1], self.height_,self.tStep_-self.tDS_)
                    for time in np.arange(0.0, self.tStep_ - self.tDS_, self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(coefs[0] + coefs[1] * time + coefs[2] * time**2 + coefs[3] * time**3 + coefs[4] * time**4 + coefs[5] * time**5)
                        
                    for time in np.arange(0.0,(self.alpha) * self.tDS_,self.dt_):
                        self.rFoot_.append(self.footPose_[step])
                        self.lFoot_.append(self.footPose_[step + 1])
                        
        pass

    def getTrajectoryR(self):
        temp=np.array(self.rFoot_)
        plt.plot(temp[:,0])
        plt.plot(temp[:,1])
        plt.plot(temp[:,2])
        plt.title("left foot")
        plt.show()
        return self.rFoot_

    def getTrajectoryL(self):
        temp=np.array(self.lFoot_)
        plt.plot(temp[:,0])
        plt.plot(temp[:,1])
        plt.plot(temp[:,2])
        plt.title("left foot")
        plt.show()
        return self.lFoot_

    def poly5(self,x0, xf, z_max, tf):
        ans = list("")
        ans.append(x0)
        ans.append(np.zeros(3))
        ans.append(np.zeros(3))
        ans.append(10/tf**3 * (xf - x0))
        ans.append(-15/tf**4 * (xf - x0))
        ans.append(6/tf**5 * (xf - x0))

        ans[0][2] = 0.0
        ans[1][2] = 0.0
        ans[2][2] = 16 * z_max / tf**2
        ans[3][2] = -32 * z_max / tf**3
        ans[4][2] = 16 * z_max / tf**4
        ans[5][2] = 0.0

        return ans 

if __name__ == "__main__":
    dt = 1/240
    anklePlanner = Ankle(1.0, 0.3, 0.05,dt)
    rF =np.array([[0.0,0.115,0.0],
                 [0.0,-0.115,0.0],
                 [0.4,0.115,0.0],
                 [0.8,-0.115,0.0],
                 [1.2,0.115,0.0],
                 [1.6,-0.115,0.0],
                 [2.0,0.115,0.0],
                 [2.0,-0.115,0.0]])
    anklePlanner.updateFoot(rF)
    anklePlanner.generateTrajectory()
    left = np.array(anklePlanner.getTrajectoryL())
    right = np.array(anklePlanner.getTrajectoryR())
    print(len(left))
    print(len(right))
    plt.plot(left[:,0])
    plt.plot(left[:,1])
    plt.plot(left[:,2])
    plt.title("left foot")
    plt.show()
    plt.plot(right[:,0])
    plt.plot(right[:,1])
    plt.plot(right[:,2])
    plt.title("right foot")
    plt.show()
    pass