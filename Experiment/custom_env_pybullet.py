import numpy as np
import pybullet as p
import pybullet_data
import time

# Building the env
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

goal_position = [0.5, 0.5, 0.2]
# This is for loading the models and were in the env to place them, also made some changes to the env
plane_id = p.loadURDF("plane.urdf", [0, 0, 0])  #[0, 0, 0, 1])
p.changeDynamics(plane_id, -1, lateralFriction=1.0)
p.changeVisualShape(plane_id, -1, rgbaColor=[0.8, 0.8, 0.8, 1])

p.loadURDF("r2d2.urdf", [0, 0, 0] , [0, 0, 0, 2])

# Loading the arm and changing size of a sphere that the arm will grab
sphere_radius = 0.10
sphere = p.loadURDF("sphere2.urdf", [0.5, 0.5, 0.2], [0, 0, 0, 1], globalScaling= sphere_radius)
targid = p.loadURDF("franka_panda/panda.urdf", goal_position , [0, 0, 0, 1] ,useFixedBase = True) # useFixedBase makes it so the arm does not fall when moving, if set to True
obj_of_focus = targid

# Added a line for the goal position
p.addUserDebugLine(goal_position, [goal_position[0], goal_position[1], goal_position[2] + 0.2], [0, 1, 0], 2)

# Camera on robot to get another pov, still under construction
camera_matrix = p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=[0, 0, 0.5],
    distance=1.0,
    yaw=0,
    pitch=-30,
    roll=0,
    upAxisIndex=2
)

projection_matrix = p.computeProjectionMatrixFOV(
    fov=60, aspect=1.0, nearVal=0.1, farVal=3.1
)

# Disabled rendering for a faster sim
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.addUserDebugParameter("Light", 0, 100, 50)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# Joint control
jointid = 4
jlower = p.getJointInfo(targid, jointid)[8]
jupper = p.getJointInfo(targid, jointid)[9]

simulation_duration = 20
time_per_step = 0.02
total_steps = int(simulation_duration / time_per_step)

# Defining the 2 positions the arm is going to move between
target_position_1 = [np.random.uniform(jlower, jupper), np.random.uniform(jlower, jupper)]
target_position_2 = [np.random.uniform(jlower, jupper), np.random.uniform(jlower, jupper)]

# First target pos for the arm
for step in range(total_steps // 2):
    p.setJointMotorControlArray(targid, [2, 4], p.POSITION_CONTROL, targetPositions = target_position_1)
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition = focus_position)
    p.stepSimulation()
    time.sleep(time_per_step)

# The second one
for step in range(total_steps // 2, total_steps):
    p.setJointMotorControlArray(targid, [2, 4], p.POSITION_CONTROL, targetPositions = target_position_2)
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition = focus_position)
    p.stepSimulation()
    time.sleep(time_per_step)

# Only works for one pos
# for step in range(total_steps):
#     joint_two_targ = np.random.uniform(jlower, jupper)
#     joint_four_targ = np.random.uniform(jlower, jupper)
#     p.setJointMotorControlArray(targid, [2, 4], p.POSITION_CONTROL, targetPositions = [joint_two_targ, joint_four_targ])
#     focus_position, _ = p.getBasePositionAndOrientation(targid)
#     p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition = focus_position)
#     p.stepSimulation()
#     time.sleep(time_per_step)