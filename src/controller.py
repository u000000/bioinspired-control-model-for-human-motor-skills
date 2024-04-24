import numpy as np
import matplotlib.pyplot as plt
import sympy as sp

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

# Simulation Parameters
dt = 0.01
N = 3000
time = np.arange(0, dt * N, dt)

# Admittance Control Parameters for position
Mp = np.diag([2, 2]) * 0.2
Dp = np.diag([10, 10]) * 0.2
Kp = np.diag([13, 13])

# Initial State for position
pc = np.zeros(2)
vc = np.zeros(2)

# Desired trajectories for position
pd = np.zeros((2, N))
pd[0, :] = 0.3 * np.sin(time / 3)
pd[1, :] = 0.3 * np.sin(time / 3) * np.cos(time / 3)

# Initialize arrays for storing joint angles
joint_angles = np.zeros((6, N))

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

# Main simulation loop
for i in range(N):
    # Position control
    p_error = pc - pd[:, i]
    acc = np.linalg.solve(Mp, -Dp @ vc - Kp @ p_error)
    vc += acc * dt
    pc += vc * dt

    # Compute joint angles based on the current end effector's position
    gripper_point = (pc[0], pc[1], 0)  # Assuming flat movement in xy-plane
    R0g = sp.eye(3)  # Assuming no rotation for simplicity
    wrist_center = get_wrist_center(gripper_point, R0g)
    j1, j2, j3 = get_first_three_angles(wrist_center)
    # Placeholder rotation matrix for last three angles
    R = sp.eye(3)  # Placeholder
    j4, j5, j6 = get_last_three_angles(R)
    joint_angles[:, i] = [j1, j2, j3, j4, j5, j6]

# Visualization of results
plt.figure(figsize=(12, 24))

# X Position Plot
plt.subplot(8, 1, 1)
plt.plot(time, pc_hist[0, :], label='Actual X')  # Access the first row for X positions
plt.plot(time, pd[0, :], 'r--', label='Desired X')
plt.title('X Position Over Time')
plt.xlabel('Time (s)')
plt.ylabel('X (m)')
plt.legend()

# Y Position Plot
plt.subplot(8, 1, 2)
plt.plot(time, pc_hist[1, :], label='Actual Y')  # Access the second row for Y positions
plt.plot(time, pd[1, :], 'r--', label='Desired Y')
plt.title('Y Position Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Y (m)')
plt.legend()

# Joint angles plots
for j in range(6):
    plt.subplot(8, 1, j+3)
    plt.plot(time, joint_angles[j, :], label=f'Joint {j+1} Angle')
    plt.title(f'Joint {j+1} Angle Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()

plt.tight_layout()
plt.show()
