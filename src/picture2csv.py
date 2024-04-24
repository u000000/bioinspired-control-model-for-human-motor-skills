import cv2
import pandas as pd
import numpy as np
import os
import sympy as sp
import matplotlib.pyplot as plt

def picture2csv(file_name:str):
    img = cv2.imread(file_name, cv2.IMREAD_GRAYSCALE)
    _, binary_image = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY)

    # Look for black pixels
    black_line_points = np.argwhere(binary_image == 0)

    df = pd.DataFrame(black_line_points, columns=['x', 'y'])
    df.to_csv('line.csv', index=False, header=False)

    # Use imshow to show the line exported to csv
    cv2.imshow('path',binary_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return df

# Define a function to compute the transformation matrix based on DH parameters
def dh_matrix(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.sin(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Helper function to compute wrist center from the end effector's position and orientation
def get_wrist_center(gripper_point, R0g, dg=0.303):
    xu, yu, zu = gripper_point
    nx, ny, nz = R0g[:, 2]  # Approach vector
    xw = xu - dg * nx
    yw = yu - dg * ny
    zw = zu - dg * nz
    return xw, yw, zw

def get_first_three_angles(wrist_center):
    """
    Calculate the first three joint angles (q1, q2, q3) given the wrist center.
    """
    x, y, z = wrist_center
    # Constants based on the robot's physical dimensions:
    d1 =  0.1625  # Distance from base frame to first joint
    a2 = -0.425  # Length of the second arm segment
    a3 = -0.3922   # Length of the third arm segment

    # Joint angle calculations
    q1 = sp.atan2(y, x)

    # Intermediate values for q2, q3
    r = sp.sqrt(x**2 + y**2)  # Planar distance from base frame's z-axis
    s = z - d1
    D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)

    q3 = sp.atan2(sp.sqrt(1 - D**2), D)
    q2 = sp.atan2(s, r) - sp.atan2(a3 * sp.sin(q3), a2 + a3 * sp.cos(q3))

    return q1.evalf(), q2.evalf(), q3.evalf()


def get_last_three_angles(R):
    """
    Calculate the last three joint angles (q4, q5, q6) from the rotation matrix R (R3_6 in DH terms).
    """
    q4 = sp.atan2(R[2, 1], R[2, 0]).evalf()
    q5 = sp.atan2(sp.sqrt(R[2, 0]**2 + R[2, 1]**2), R[2, 2]).evalf()
    q6 = sp.atan2(-R[1, 0], R[0, 0]).evalf()

    return q4, q5, q6

def controller(data) -> None:
    # # Generate random joint angles
    # joints_angles = np.random.randint(10, 30, (df_points.shape[0], 2))

    # Define joint variables
    theta = sp.symbols('theta_1:7')  # Generates theta_1 to theta_6
    alpha, beta, gamma = sp.symbols('alpha beta gamma', real=True)

    # DH parameters for each joint of the robot
    dh_params = [
        {'theta': theta[0], 'd': 0.1625, 'a': 0, 'alpha': sp.pi/2},
        {'theta': theta[1], 'd': 0, 'a': -0.425, 'alpha': 0},
        {'theta': theta[2], 'd': 0, 'a': -0.3922, 'alpha': 0},
        {'theta': theta[3], 'd': 0.1333, 'a': 0, 'alpha': sp.pi/2},
        {'theta': theta[4], 'd': 0.0997, 'a': 0, 'alpha': -sp.pi/2},
        {'theta': theta[5], 'd': 0.0996, 'a': 0, 'alpha': 0}
    ]

    # Admittance Control Parameters for position
    Mp = np.diag([2, 2]) * 0.2
    Dp = np.diag([10, 10]) * 0.2
    Kp = np.diag([13, 13])

    # Initial State for position
    pc = np.zeros(2)
    vc = np.zeros(2)

    df = pd.DataFrame(columns=['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])


    #OVERWRITE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    data['x'] = 0.3 * np.sin(time / 3)
    data['y'] = 0.3 * np.sin(time / 3) * np.cos(time / 3)
    data.to_csv('line.csv', index=False, header=False)

    # pc_hist = np.zeros((2, N))
    # Main simulation loop
    for index, row in data.iterrows():
        # Position control
        p_error = [pc[0] - row.x, pc[1] - row.y]
        acc = np.linalg.solve(Mp, -Dp @ vc - Kp @ p_error)
        print('error: ' + str(p_error))
        vc += acc * dt
        pc += vc * dt
        # pc_hist[:, i] = pc
        # Compute joint angles based on the current end effector's position
        gripper_point = (pc[0], pc[1], 0)  # Assuming flat movement in xy-plane
        R0g = sp.eye(3)  # Assuming no rotation for simplicity
        wrist_center = get_wrist_center(gripper_point, R0g)
        j1, j2, j3 = get_first_three_angles(wrist_center)
        print('j1: ' + str(j1))
        print('j2: ' + str(j2))
        print('j3: ' + str(j3))
        # Placeholder rotation matrix for last three angles
        R = sp.eye(3)  # Placeholder
        j4, j5, j6 = get_last_three_angles(R)
        joint_angles = [j1, j2, j3, j4, j5, j6]
        df.loc[len(df)] = joint_angles

    df.to_csv('jointAngles.csv', index=False, header=False)

# def visualize_joints(pc_hist, joint_angles):
#     # Visualization of results
#     plt.figure(figsize=(12, 24))

#     # X Position Plot
#     plt.subplot(8, 1, 1)
#     plt.plot(time, pc_hist[0, :], label='Actual X')  # Access the first row for X positions
#     plt.plot(time, pd[0, :], 'r--', label='Desired X')
#     plt.title('X Position Over Time')
#     plt.xlabel('Time (s)')
#     plt.ylabel('X (m)')
#     plt.legend()

#     # Y Position Plot
#     plt.subplot(8, 1, 2)
#     plt.plot(time, pc_hist[1, :], label='Actual Y')  # Access the second row for Y positions
#     plt.plot(time, pd[1, :], 'r--', label='Desired Y')
#     plt.title('Y Position Over Time')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Y (m)')
#     plt.legend()

#     # Joint angles plots
#     for j in range(6):
#         plt.subplot(8, 1, j+3)
#         plt.plot(time, joint_angles[j, :], label=f'Joint {j+1} Angle')
#         plt.title(f'Joint {j+1} Angle Over Time')
#         plt.xlabel('Time (s)')
#         plt.ylabel('Angle (rad)')
#         plt.legend()

#     plt.tight_layout()
#     plt.show()


if __name__ == "__main__":


    path = os.path.dirname(__file__)
    file_name = os.path.join(path, 'signature.png')
    data = picture2csv(file_name)

    # Simulation Parameters
    dt = 0.01
    N = len(data)
    time = np.arange(0, dt * N, dt)
    
    controller(data)