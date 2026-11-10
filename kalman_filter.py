

import numpy as np



# Extended Kalman Filter for planar motion.
# State order: [world x, world y, heading, angular velocity, forward velocity,
#               forward acceleration].
# Measurement order: [odom forward velocity, odom angular velocity, IMU ax, IMU ay].
class kalman_filter:
    
    # Copy inputs so filter updates do not modify the caller's arrays.
    def __init__(self, P,Q,R, x, dt):
        
        self.P = np.array(P, dtype=float, copy=True)
        self.Q = np.array(Q, dtype=float, copy=True)
        self.R = np.array(R, dtype=float, copy=True)
        self.x = np.array(x, dtype=float, copy=True)
        self.dt = float(dt)
        # P (6x6): uncertainty in our current state estimate.
        # Q (6x6): process noise added each prediction, allowing imperfect motion.
        # R (4x4): measurement noise; larger entries mean less trust in that sensor.
        # dt: time between prediction steps, in seconds.
        
    # Predict both the state and its uncertainty one time step forward.
    def predict(self):

        # Linearize the motion model at the state BEFORE propagation.
        self.A = self.jacobian_A()
        
        self.motion_model()
        
        # A propagates existing uncertainty; Q adds motion-model uncertainty.
        self.P = self.A @ self.P @ self.A.T + self.Q

    # Correct the prediction using the latest four sensor measurements.
    def update(self, z):
        # C is the measurement Jacobian H, evaluated at the predicted state.
        self.C = self.jacobian_H()
        
        # Innovation covariance: uncertainty in the predicted measurement plus sensor noise.
        S = self.C @ self.P @ self.C.T + self.R
        # K = P C^T S^-1; solve the system to avoid explicitly inverting S.
        kalman_gain = np.linalg.solve(S.T, (self.P @ self.C.T).T).T
        
        # Correct the predicted state using actual minus predicted sensor readings.
        self.x = self.x + kalman_gain @ (z - self.measurement_model())
        
        # Update state uncertainty after incorporating the measurements (I is 6x6).
        self.P = (np.eye(self.P.shape[0]) - kalman_gain @ self.C) @ self.P
        
    
    # h(x): predicted sensor readings in robot axes (forward x, left y).
    # Planar, aligned-IMU model: ax = vdot and turning acceleration ay = v*w.
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v*w, # ay
        ])
        
    def motion_model(self):
        # Forward Euler integration in world coordinates.
        # Angular velocity and forward acceleration stay constant during prediction;
        # their uncertainty can grow through Q and measurements can correct them.
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        displacement = v * dt
        
        self.x = np.array([
            x + displacement * np.cos(th),
            y + displacement * np.sin(th),
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self):
        # A[i,j] = derivative of predicted state i with respect to current state j.
        # This describes how small state errors affect the next prediction.
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              -v*np.sin(th)*dt, 0,          np.cos(th)*dt,  0],
            [0, 1,              v*np.cos(th)*dt, 0,          np.sin(th)*dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # H[i,j] = derivative of predicted measurement i with respect to state j.
    # The final row differentiates v*w: derivative wrt w is v, wrt v is w.
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # Return an independent snapshot so callers cannot accidentally change the filter.
    def get_states(self):
        return self.x.copy()
