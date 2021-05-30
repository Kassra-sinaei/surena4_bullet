import numpy as np
import math

class Robot:
    def __init__(self, shank, hip, pelvis_lengt):
        self.config_ = np.zeros(28)
        self.shank_ = shank
        self.hip_ = hip
        self.pelvis_ = pelvis_lengt # half the pelvis length!
        pass

    def spinOffline(self):

        pass

    def doIK(self, pelvisP, pelvisR, leftP, leftR, rightP, rightR):
        self.config_[0:6] = self.geometricIK(pelvisP, pelvisR,rightP, rightR, self.hip_, -self.pelvis_, self.shank_)
        self.config_[6:12] = self.geometricIK(pelvisP, pelvisR,leftP, leftR, self.hip_, self.pelvis_, self.shank_)
        return self.config_[0:12]

    def Rroll(self, phi):
        R = np.eye(3)
        R[1,1] = np.cos(phi)
        R[1,2] = -np.sin(phi)
        R[2,1] = np.sin(phi)
        R[2,2] = np.cos(phi)
        return R

    def Rpitch(self, theta):
        R = np.eye(3)
        R[0,0] = np.cos(theta)
        R[0,2] = np.sin(theta)
        R[2,0] = -np.sin(theta)
        R[2,2] = np.cos(theta)
        return R

    def geometricIK(self, p1,R1,p7,R7, A , d, B):
        """
            inputs:
                a -----------> Length of Hip
                b -----------> length of Shank
                (p1, R1) ----> position and attitude of body
                (p7, R7) ----> position and attitude of right foot
            output
                joint angles --> q = [q2,q3,q4,q5,q6,q7]
        """
        D = np.array([0,d,0])
        #E = np.array([0,0,e])
        r = np.matmul(R7.T , (p1 + np.matmul(R1 , D) - p7))
        C = np.sqrt(r[0]**2 + r[1]**2 +r[2]**2)
        c5 = (C**2 - A**2 - B**2) / (2*A*B)
        if c5 >= 1:
            q5 = 0.0
        elif c5 <= -1:
            q5 = np.pi
        else:
            q5 = np.arccos(c5) # Knee Pitch    
        q6a = np.arcsin((A/C)*np.sin(np.pi-q5)) # Ankle Pitch Sub
        q7 = np.arctan2(r[1],r[2]) # Ankle Roll
        if q7 > np.pi/2:
            q7 -= np.pi
        elif q7 < -np.pi/2:
            q7 += np.pi
        q6 = -np.arctan2(r[0], np.sign(r[2]) * np.sqrt(r[1]**2 + r[2]**2)) - q6a # Ankle Pitch
        
        R = np.matmul(R1.T , np.matmul(R7 , np.matmul(self.Rroll(-q7) , self.Rpitch(-q6-q5))))
        q2 = np.arctan2(-R[0,1], R[1,1]) # Hip Yaw
        q3 = np.arctan2(R[2,1], -R[0,1] * np.sin(q2) + R[1,1] * np.cos(q2))  # Hip Roll
        q4 = np.arctan2(-R[2,0], R[2,2]) # Hip Pitch
                        
        return([q2,q3,q4,q5,q6,q7])
