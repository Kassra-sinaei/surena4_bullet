import numpy as np
import math

K_G = 9.81

class DCMPlanner:
    def __init__(self,deltaZ,  stepTime,  doubleSupportTime,  dt, stepCount = 6, alpha = 0.5):
        self.deltaZ_ = deltaZ
        self.tStep_ = stepTime
        self.tDS_ = doubleSupportTime
        self.dt_ = dt
        self.stepCount_ = stepCount
        self.alpha_ = alpha
        self.xi_ = list("")
        pass

    def getXiTrajectory(self):

        pass

    def getXiDot(self):

        pass

    def getCoMTrajectory(self):
        self.COM_ = np.zeros_like(self.xi_)

        self.COM_[0] = [0,0,0.8] 
        for index in range(1,self.COM_.shape[0]):
            inte = np.zeros((3))
            for t in range(index):
                inte += (self.dt_) * self.xi_[t] * math.exp((t*self.dt_)/(math.sqrt(self.deltaZ_/K_G)))

            self.COM_[index] = (inte / math.sqrt(self.deltaZ_/K_G) + self.COM_[0]) * math.exp((-index/100)/math.sqrt(self.deltaZ_/K_G))
        pass

    def setFoot(self, rF):
        self.rF_ = rF
        pass

    def updateVRP(self):
        self.rVRP_ = np.copy(self.rF_)
        self.rVRP_[:,2] += self.deltaZ_
        pass

    def updateXiEOS(self):
        self.xiEOS_ = np.copy(self.rVRP)
        self.xiEOS_[-1] = self.rVRP_[-1]

        for index in range(np.size(self.rVRP_,0)-2,-1,-1):
            self.xiEOS_[index] = self.rVRP_[index+1] + math.exp(-math.sqrt(K_G/self.deltaZ_) * self.tStep_) * (self.xiEOS_[index+1] - self.rVRP_[index+1])
        pass

    def updateXiSS(self):
        for iter in range(int((1/self.dt_) * self.tStep_ * self.rF_.shape[0])):
            time = iter * self.dt_
            i = int(time / self.tStep_)
            t = time % (self.tStep_)
            self.xi_.append(self.rVRP_[i] + math.exp(math.sqrt(K_G/self.deltaZ_) * (t - self.tStep_)) * (self.xiEOS_[i] - self.rVRP_[i]))
        pass

    def updateDSPose(self):
        self.xiDS_i = np.zeros((np.size(self.rF_,0),3))
        self.xiDS_e = np.zeros((np.size(self.rF_,0),3))
        for index in range(np.size(self.rVRP_,0)):
            if index == 0:
                self.xiDS_i[index] = self.xi_[0]
                self.xiDS_e[index] = self.rVRP_[index] + math.exp(math.sqrt(K_G/self.deltaZ_) * self.tDS_ * (1-self.alpha_)) * (self.xi_[0] - self.rVRP_[index])
            else:
                self.xiDS_i[index] = self.rVRP_[index-1] + math.exp(-math.sqrt(K_G/self.deltaZ_) * self.tDS_ * self.alpha_) * (self.xiEOS_[index-1] - self.rVRP_[index-1])
                self.xiDS_e[index] = self.rVRP_[index] +   math.exp( math.sqrt(K_G/self.deltaZ_) * self.tDS_ * (1-self.alpha_)) * (self.xiEOS_[index-1] - self.rVRP_[index])
        pass
    
    def interpolate1(self, x1, x2, v1, v2):
        a = v1+v2+2*x1-2*x2
        b = -2*v1-v2-3*x1+3*x2
        c = v1
        d = x1
        return a, b, c, d
    
    def updateDS(self):
        CDS_coefs = list('')
        for i in range(np.size(self.rVRP_,0)):
            if i == 0:
                xi_dot_i = (self.xiDS_i[i] - self.xi_[0]) * math.sqrt(K_G/self.deltaZ_)
                xi_dot_e = (self.xiDS_e[i] - self.rVRP_[0]) * math.sqrt(K_G/self.deltaZ_)
            else:
                xi_dot_i = (self.xiDS_i[i] - self.rVRP_[i-1]) * math.sqrt(K_G/self.deltaZ_)
                xi_dot_e = (self.xiDS_e[i] - self.rVRP_[i]) * math.sqrt(K_G/self.deltaZ_)
            CDS_coefs.append(self.interpolate1(self.xiDS_i[i],self.xiDS_e[i], xi_dot_i, xi_dot_e))

        xi_cds = list('')
        for i in range(len(CDS_coefs)):
            a, b, c, d = CDS_coefs[i]
            CDS_trajectory = np.zeros((int(self.tDS_*(1/self.dt_)),3))
            for t in range(int(self.tDS_*(1/self.dt_))):
                CDS_trajectory[t] = a * (t* self.dt_)**3 + b * (t*self.dt_)**2 + c * (t*self.dt_) + d
            xi_cds.append(CDS_trajectory)

        xi_final = np.array(self.xi_)
        for step in range(self.rVRP_.shape[0]):
            if step == 0:
                xi_final[:int(self.tDS_ * (1-self.alpha_) *(1/self.dt_))] = xi_cds[0][:int((1-self.alpha_)*(1/self.dt_)*self.tDS_)]
            else: 
                xi_final[int((1/self.dt_) * self.tStep_ * step - (self.tDS_ * self.alpha_ *(1/self.dt_))):int((1/self.dt_) * self.tStep_ * step +(self.tDS_ * (1-self.alpha_) *(1/self.dt_)))] = xi_cds[step][:]
        
        self.xi_ = xi_final

        pass




if __name__ == "__main__":
    pass