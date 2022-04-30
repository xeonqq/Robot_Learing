import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, atan2

odo_raw = np.genfromtxt('../data/odom_motions',delimiter=' ',skip_header=5)
scan_match = np.genfromtxt('../data/scanmatched_motions',delimiter=' ',skip_header=5)
def pose2mat(pose):
    theta = pose[2]
    x = pose[0]
    y = pose[1]
    cs = cos(theta)
    sn = sin(theta)
    return np.array([[cs, -sn, x],[sn, cs, y],[0,0,1]])

def mat2pose(mat):
    return np.array([mat[0,2],mat[1,2],atan2(mat[1,0], mat[0,0])])

def compute_traj(odo):
    poses = [np.array([0,0,0])]
    T = pose2mat(poses[-1])
    for delta in odo:
        T = T.dot(pose2mat(delta))
        poses.append(mat2pose(T))
    return np.asarray(poses)

def sqr_error(odo, scan_match):
    error = np.sum((odo- scan_match)**2)/len(odo)
    return error

A = np.transpose(odo_raw).dot(odo_raw)
calibration_mat = np.empty((3,3))

#for i in range(3):
#    b = np.transpose(odo_raw).dot(scan_match[:,i])
#    calibration_mat[i,:] = np.linalg.inv(A).dot(b)
B = np.transpose(odo_raw).dot(scan_match)
calibration_mat = np.linalg.inv(A).dot(B)
calibration_mat = calibration_mat.T

print("calibration mat: ", calibration_mat)
print("error before calibration: ", sqr_error(odo_raw, scan_match))

odo_cali = calibration_mat.dot(odo_raw.T).T
print("error after calibration: ", sqr_error(odo_cali, scan_match))

odo_raw_traj = compute_traj(odo_raw)
odo_cali_traj= compute_traj(odo_cali)
scan_match_traj = compute_traj(scan_match)
plt.plot(odo_raw_traj[:,0], odo_raw_traj[:,1], label="odo_raw")
plt.plot(odo_cali_traj[:,0], odo_cali_traj[:,1], label="odo_cali")
plt.plot(scan_match_traj[:,0], scan_match_traj[:,1], label="scan_match")
plt.legend()
plt.show()
