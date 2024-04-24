import numpy as np

def quaternProd(a, b):
    ab = np.zeros(4)
    ab[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    ab[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    ab[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    ab[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return ab

def quaternConj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

class MahonyAHRS:
    def __init__(self, sample_period=1/256, quaternion=None, kp=1, ki=0):
        self.sample_period = sample_period
        self.quaternion = np.array([1, 0, 0, 0]) if quaternion is None else quaternion
        self.Kp = kp
        self.Ki = ki
        self.eInt = np.array([0, 0, 0])
    
    def update(self, gyroscope, accelerometer, magnetometer):
        q = self.quaternion
        
        if np.linalg.norm(accelerometer) == 0:
            return 
        accelerometer = accelerometer / np.linalg.norm(accelerometer)
        
        if np.linalg.norm(magnetometer) == 0:
            return
        magnetometer = magnetometer / np.linalg.norm(magnetometer)
        
        h = quaternProd(q, quaternProd(np.array([0, *magnetometer]), quaternConj(q)))
        b = np.array([0, np.linalg.norm(h[1:3]), 0, h[3]])
        
        v = np.array([2*(q[1]*q[3] - q[0]*q[2]),
                      2*(q[0]*q[1] + q[2]*q[3]),
                      q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2])
        w = np.array([2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]),
                      2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]),
                      2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2)])
        
        e = np.cross(accelerometer, v) + np.cross(magnetometer, w)
        if self.Ki > 0:
            self.eInt += e * self.sample_period
        else:
            self.eInt = np.array([0, 0, 0])
        
        gyroscope += self.Kp * e + self.Ki * self.eInt
        qDot = 0.5 * quaternProd(q, np.array([0, *gyroscope]))
        q += qDot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)
    
    def update_imu(self, gyroscope, accelerometer):
        q = self.quaternion
        
        # Normalise accelerometer measurement
        if np.linalg.norm(accelerometer) == 0:
            return  # avoid division by zero
        accelerometer = accelerometer / np.linalg.norm(accelerometer)
        
        # Estimated direction of gravity
        v = np.array([2*(q[1]*q[3] - q[0]*q[2]),
                    2*(q[0]*q[1] + q[2]*q[3]),
                    q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2])
        
        # Error is cross product between estimated direction and measured direction of gravity
        e = np.cross(accelerometer, v)
        if self.Ki > 0:
            self.eInt += e * self.sample_period
        else:
            self.eInt = np.array([0, 0, 0])
        
        # Apply feedback terms
        gyroscope += self.Kp * e + self.Ki * self.eInt
        
        # Compute rate of change of quaternion
        qDot = 0.5 * quaternProd(q, np.array([0, *gyroscope]))
        
        # Integrate to yield quaternion
        q += qDot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)  # normalise quaternion
