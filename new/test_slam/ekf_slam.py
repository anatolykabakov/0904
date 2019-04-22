"""
Extended Kalman Filter SLAM example
author: Atsushi Sakai (@Atsushi_twi)
"""

import math
import numpy as np

class ekf_slam(object):
    def __init__(self, Landmarks):
        self.landmarks = Landmarks
        # EKF state covariance
        self.Cx = np.diag([0.3, 0.3, np.deg2rad(30.0)])**2

        self.M_DIST_TH = 1  # Threshold of Mahalanobis distance for data association.
        self.STATE_SIZE = 3  # State size [x,y,yaw]
        self.LM_SIZE = 2  # LM state size [x,y]
        # State Vector [x y yaw v]'
        self.xEst = np.zeros((self.STATE_SIZE, 1))
        self.xTrue = np.zeros((self.STATE_SIZE, 1))
        self.PEst = np.eye(self.STATE_SIZE)
        # history
        self.hxEst = self.xEst
        self.hxTrue = self.xTrue
        self.hxDR = self.xTrue
        self.initP = np.eye(2)

        self.xDR = np.zeros((self.STATE_SIZE, 1))  # Dead reckoning
        
    def ekf_slam(self, u, scan, deltaTime):
        z = self.landmarks.extract_landmarks(scan)
        #print(z)

        # Predict
        S = self.STATE_SIZE
        self.xEst[0:S] = self.motion_model(self.xEst[0:S], u, deltaTime)
        G, Fx = self.jacob_motion(self.xEst[0:S], u, deltaTime)
        self.PEst[0:S, 0:S] = G.T * self.PEst[0:S, 0:S] * G + Fx.T * self.Cx * Fx
        

        # Update
        for iz in range(len(z[:, 0])):  # for each observation
            minid = self.search_correspond_LM_ID(self.xEst, self.PEst, z[iz, 0:2])
            #print(minid)

            nLM = self.calc_n_LM(self.xEst)
            if minid == nLM:
                print("New LM")
                # Extend state and covariance matrix
                xAug = np.vstack((self.xEst, self.calc_LM_Pos(self.xEst, z[iz, :])))
                PAug = np.vstack((np.hstack((self.PEst, np.zeros((len(self.xEst), self.LM_SIZE)))),
                                  np.hstack((np.zeros((self.LM_SIZE, len(self.xEst))), self.initP))))
                self.xEst = xAug
                self.PEst = PAug
            lm = self.get_LM_Pos_from_state(self.xEst, minid)
            y, S, H = self.calc_innovation(lm, self.xEst, self.PEst, z[iz, 0:2], minid)
            #print(y)

            K = (self.PEst @ H.T) @ np.linalg.inv(S)
            self.xEst = self.xEst + (K @ y)
            self.PEst = (np.eye(len(self.xEst)) - (K @ H)) @ self.PEst
##
        self.xEst[2] = self.pi_2_pi(self.xEst[2])

        return self.xEst, self.PEst


    def motion_model(self, x, u, DT):

        F = np.array([[1.0, 0, 0],
                      [0, 1.0, 0],
                      [0, 0, 1.0]])

        B = np.array([[DT * math.cos(x[2, 0]), 0],
                      [DT * math.sin(x[2, 0]), 0],
                      [0.0, DT]])

        x = (F @ x) + (B @ u)
        return x


    def calc_n_LM(self, x):
        n = int((len(x) - self.STATE_SIZE) / self.LM_SIZE)
        return n


    def jacob_motion(self, x, u, DT):

        Fx = np.hstack((np.eye(self.STATE_SIZE), np.zeros((self.STATE_SIZE, self.LM_SIZE * self.calc_n_LM(x)))))

        jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                       [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                       [0.0, 0.0, 0.0]])

        G = np.eye(self.STATE_SIZE) + Fx.T * jF * Fx

        return G, Fx,


    def calc_LM_Pos(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
        #zp[0, 0] = x[0, 0] + z[0, 0] * math.cos(x[2, 0] + z[0, 1])
        #zp[1, 0] = x[1, 0] + z[0, 0] * math.sin(x[2, 0] + z[0, 1])

        return zp


    def get_LM_Pos_from_state(self, x, ind):

        lm = x[self.STATE_SIZE + self.LM_SIZE * ind: self.STATE_SIZE + self.LM_SIZE * (ind + 1), :]

        return lm


    def search_correspond_LM_ID(self, xAug, PAug, zi):
        """
        Landmark association with Mahalanobis distance
        """

        nLM = self.calc_n_LM(xAug)

        mdist = []

        for i in range(nLM):
            lm = self.get_LM_Pos_from_state(xAug, i)
            y, S, H = self.calc_innovation(lm, xAug, PAug, zi, i)
            mdist.append(y.T @ np.linalg.inv(S) @ y)
        #print(mdist)

        mdist.append(self.M_DIST_TH)  # new landmark

        minid = mdist.index(min(mdist))

        return minid


    def calc_innovation(self, lm, xEst, PEst, z, LMid):
        delta = lm - xEst[0:2]
        q = (delta.T @ delta)[0, 0]
        zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
        zp = np.array([[math.sqrt(q), self.pi_2_pi(zangle)]])
        y = (z - zp).T
        y[1] = self.pi_2_pi(y[1])
        H = self.jacobH(q, delta, xEst, LMid + 1)
        S = H @ PEst @ H.T + self.Cx[0:2, 0:2]

        return y, S, H


    def jacobH(self, q, delta, x, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                      [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

        G = G / q
        nLM = self.calc_n_LM(x)
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

        F = np.vstack((F1, F2))

        H = G @ F

        return H


    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
