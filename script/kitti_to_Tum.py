from scipy.spatial.transform import Rotation as R
import numpy as np

def read_times_file(filename):
    with open(filename, 'r') as f:
        times = [float(line.strip()) for line in f.readlines()]
    print("len times",len(times))
    return times

def read_trajectory_file(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    trajectory = []
    for line in lines:
        values = line.strip().split()
        trajectory.append([float(value) for value in values])
    print("len trajectory",len(trajectory))
    return trajectory

# Tr_velo_to_cam means lidar frame with respect to camera frame  (lidar2camera)
Tr_velo_to_cam = np.array([[7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03],
                          [1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02],
                          [9.998621e-01, 7.523790e-03, -1.480755e-02, -2.717806e-01],
                          [ 0, 0, 0, 1]])
# Tr_velo_to_cam = np.array([1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1])
Tr_cam_to_velo = np.linalg.inv(Tr_velo_to_cam)

def convert_to_tum_format(times, trajectory, output_filename):
    with open(output_filename, 'w') as f:
        for i, time in enumerate(times):
            if i < len(trajectory):
                r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz = trajectory[i]
                rotation_matrix = np.array([[r11, r12, r13],
                                            [r21, r22, r23],
                                            [r31, r32, r33]])

                t = np.array([tx, ty, tz])
                t = Tr_cam_to_velo[0:3,0:3].dot(rotation_matrix).dot(Tr_velo_to_cam[0:3, 3]) + Tr_cam_to_velo[0:3,0:3].dot(t) + Tr_cam_to_velo[0:3,3] 
                rotation_matrix = Tr_cam_to_velo[0:3,0:3].dot(rotation_matrix.dot(Tr_velo_to_cam[0:3, 0:3]))
                quaternion = R.from_matrix(rotation_matrix).as_quat()
                line = f"{time} {t[0]} {t[1]} {t[2]} {quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]}\n"
                f.write(line)

times = read_times_file('/home/sti/Downloads/data_odometry_calib/dataset/sequences/00/times.txt')
trajectory = read_trajectory_file('/home/sti/Downloads/data_odometry_poses/dataset/poses/00.txt')
convert_to_tum_format(times, trajectory, 'tum_format_pose.txt')
