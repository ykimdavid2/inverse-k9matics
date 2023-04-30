import numpy as np
from scipy.io import savemat
import time
from math import sqrt
from tqdm import tqdm
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
from quadsim.envs import env_builder
from quadsim.robots import robot_config


DT: float = .002  # [s]
SIM_DURATION: float = 10.  # [s]
DEFUALT_JOINT_POS: np.ndarray = np.array([
    -.1, .8, -1.5,
    .1, .8, -1.5,
    -.1, .8, -1.5,
    .1, .8, -1.5
])
TORQUE_LIMIT: np.ndarray = np.array([
    20., 55., 55.,
    20., 55., 55.,
    20., 55., 55.,
    20., 55., 55.
])

from quadsim.robots import a1

import control
control = control.control

#pybullet.disconnect()

desc = 'Controlling with ALTERED PY CODE'

# Create a robot instance according to the robot type
env = env_builder.build_regular_env(
    robot_class=a1.A1,
    motor_control_mode=robot_config.MotorControlMode.POSITION,
    enable_rendering=True,
    on_rack=False,
    wrap_trajectory_generator=False
)
env.set_time_step(1, sim_step=DT)
robot = env.robot
"""
zero_command = np.zeros(12)
for t in range(120):
    robot.Step(zero_command, robot_config.MotorControlMode.TORQUE)
"""


#----------------------------------------------------------------------------
# IGNORE EVERYTHING ELSE FOR NOW, BELOW CODE WILL MOVE ONE JOINT
# IMPORTANT INFO:
#  - Controlling requires a call to robot.Step(), which takes in a [Position]
#  - [Position] is a size 12 vector of Joints, labeled like:
#    ["FR_hip_joint",   "FR_upper_joint",   "FR_lower_joint",
#     "FL_hip_joint",   "FL_upper_joint",   "FL_lower_joint",
#     "RR_hip_joint",   "RR_upper_joint",   "RR_lower_joint",
#     "RL_hip_joint",   "RL_upper_joint",   "RL_lower_joint"]

#  - If we want, we can simplify it by ignoring all hip_joints so it becomes an 8 motor system. 
#       - i.e. only use joint values of [1,2,4,5,7,8,10,11]
#  - A diagram exists in this folder that shows positive directions for motors

# Start the robot standing
robot.Step(DEFUALT_JOINT_POS, robot_config.MotorControlMode.POSITION)

# Choose which joint to rotate
joint = 2

new_pos = DEFUALT_JOINT_POS
step = np.zeros(12, dtype=float)

#Choose Size of Step
step[joint] = .000

print(new_pos)
print(step)
# For length of simulation, move joint to new_pos
for t in tqdm(range(1000), desc='Lifting up the robot'):
    new_pos = new_pos + step
    robot.Step(new_pos, robot_config.MotorControlMode.POSITION)
    time.sleep(.002)

#-----------------------------------------------------------------------------

"""
# Move the motors slowly to the initial stand-up configuration
robot.ReceiveObservation()
current_motor_angle = robot.GetMotorAngles()

out_stored = DEFUALT_JOINT_POS
for t in tqdm(range(200), desc='Lifting up the robot'):
#for t in tqdm(range(int(SIM_DURATION / DT)), desc='Simulation Time'):    
    time.sleep(.002)
    robot.ReceiveObservation()
    blend_ratio = np.minimum(t / 100., 1.)
    out = (1 - blend_ratio) * current_motor_angle + \
        blend_ratio * DEFUALT_JOINT_POS
    robot.Step(out, robot_config.MotorControlMode.POSITION)
    #print(out)
    out_stored = np.vstack([out_stored, out])

print(out_stored)
"""
"""
# Data buffers
distance = 0.
score = 10. + 65. * min(distance, 1.)
t_rec = np.ones([0, 1], dtype=float)
v_rec = np.ones([0, 3], dtype=float)
omega_rec = np.ones([0, 3], dtype=float)
quat_rec = np.ones([0, 4], dtype=float)
q_rec = np.ones([0, 12], dtype=float)
dq_rec = np.ones([0, 12], dtype=float)
foot_contact_rec = np.ones([0, 4], dtype=float)
out_rec = np.ones([0, 12], dtype=float)
kp_rec = np.ones([0, 12], dtype=float)
kd_rec = np.ones([0, 12], dtype=float)
tau_rec = np.ones([0, 12], dtype=float)
pos_rec = np.ones([0, 3], dtype=float)
score_rec = np.ones([0, 1], dtype=float)

# Execute the controller
for t in tqdm(range(int(SIM_DURATION / DT)), desc=desc):
    #robot.Step(tau, robot_config.MotorControlMode.POSITION)

    # Observation
    robot.ReceiveObservation()
    t = float(DT * t)
    v = np.asarray(robot.GetBaseVelocity())
    omega = robot.GetBaseRollPitchYawRate()
    quat = np.asarray(robot.GetBaseOrientation())
    q = robot.GetMotorAngles()
    dq = robot.GetMotorVelocities()
    foot_contact = np.asarray(robot.GetFootContacts(), dtype=float)

    # Update distance score
    px, py, pz = robot.GetBasePosition()
    p = sqrt(px ** 2 + py ** 2)
    distance = max(p, distance)
    score = 10. + 65. * min(distance, 1.)
    pos = np.array([px, py, pz], dtype=float)

    # Log data
    v_rec = np.vstack([v_rec, v])
    omega_rec = np.vstack([omega_rec, omega])
    quat_rec = np.vstack([quat_rec, quat])
    q_rec = np.vstack([q_rec, q])
    dq_rec = np.vstack([dq_rec, dq])
    foot_contact_rec = np.vstack([foot_contact_rec, foot_contact])
    pos_rec = np.vstack([pos_rec, pos])
    score_rec = np.vstack([score_rec, np.array([score], dtype=float)])
    t_rec = np.vstack([t_rec, np.array([t])])

    # Calculate control
    
    out, use_torque, kp, kd = control(t, v, omega, quat, q, dq, foot_contact)

    # Apply control
    if not use_torque:
        # Calculate the torques from the position commands
        tau = (out - q) * kp - dq * kd
    else:
        tau = out.copy()
    tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
    robot.Step(tau, robot_config.MotorControlMode.TORQUE)

    # Log data
    out_rec = np.vstack([out_rec, out])
    kp_rec = np.vstack([kp_rec, kp])
    kd_rec = np.vstack([kd_rec, kd])
    tau_rec = np.vstack([tau_rec, tau])
    
    
    time.sleep(.001)
"""

robot.Terminate()
