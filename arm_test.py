import pybullet as p
import time
import numpy as np

# Setup
p.connect(p.GUI)
p.setGravity(0, 0, 0)

urdf_path = "/home/joshuachoi16/school_workspace/LumaBot/my_arm.urdf"
robot = p.loadURDF(urdf_path, useFixedBase=True)

# Camera
p.resetDebugVisualizerCamera(
    cameraDistance=0.8,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.2]
)

# Get joint info
num_joints = p.getNumJoints(robot)
revolute_indices = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode()
    joint_type = info[2]
    print(f"Joint {i}: {joint_name} | type={joint_type}")
    if joint_type == p.JOINT_REVOLUTE:
        revolute_indices.append(i)

ee_index = num_joints - 1
print(f"Revolute joints: {revolute_indices}")
print(f"End effector index: {ee_index}")

# Joint limits
lower_limits = [-1.5707, -1.5707, -1.5707, -1.5707, -1.5707]
upper_limits = [ 1.5707,  1.5707,  1.5707,  1.5707,  1.5707]
joint_ranges = [ 3.1414,  3.1414,  3.1414,  3.1414,  3.1414]
rest_poses   = [ 0, 0, 0, 0, 0]

# Target position
target_pos = [0.2, -0.05, 0.2]
p.addUserDebugText("TARGET", target_pos, textColorRGB=[1, 0, 0], textSize=1.5)
p.addUserDebugLine([0, 0, 0], target_pos, [1, 0, 0])

# Main loop
step = 0
error_text_id = None

while True:
    # Solve IK with limits
    joint_angles = p.calculateInverseKinematics(
        robot,
        ee_index,
        target_pos,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses
    )

    # Instant version (comment out to use dynamics)
    for idx, joint_idx in enumerate(revolute_indices):
        p.resetJointState(robot, joint_idx, joint_angles[idx])

    # Dynamics version (uncomment to use)
    # for idx, joint_idx in enumerate(revolute_indices):
    #     p.setJointMotorControl2(
    #         robot, joint_idx,
    #         p.POSITION_CONTROL,
    #         targetPosition=joint_angles[idx],
    #         force=500
    #     )

    # Get end effector position
    ee_state = p.getLinkState(robot, ee_index)
    ee_pos = np.array(ee_state[4])
    error = np.linalg.norm(ee_pos - np.array(target_pos))

    # Get current joint states (actual positions, not IK targets)
    current_angles = [p.getJointState(robot, j)[0] for j in revolute_indices]

    # Print every 100 steps
    if step % 100 == 0:
        print(f"--- Step {step} ---")
        print(f"EE:              {np.round(ee_pos, 4)}")
        print(f"Target:          {target_pos}")
        print(f"Error:           {error:.4f}m")
        print(f"IK angles (rad): {np.round(joint_angles, 4)}")
        print(f"IK angles (deg): {np.round(np.degrees(joint_angles), 2)}")
        print(f"Actual    (rad): {np.round(current_angles, 4)}")
        print(f"Actual    (deg): {np.round(np.degrees(current_angles), 2)}")
        # Warn if near limits
        for i, angle in enumerate(joint_angles):
            if angle <= lower_limits[i] + 0.05 or angle >= upper_limits[i] - 0.05:
                print(f"  ⚠️  Joint {i+1} near limit: {np.degrees(angle):.1f}°")
        print()

    # Display error on screen
    msg = f"Error: {error:.4f}m"
    if error_text_id is None:
        error_text_id = p.addUserDebugText(msg, [0.2, 0.2, 0.3], textColorRGB=[0, 1, 0])
    else:
        error_text_id = p.addUserDebugText(msg, [0.2, 0.2, 0.3],
                                            textColorRGB=[0, 1, 0],
                                            replaceItemUniqueId=error_text_id)

    p.stepSimulation()
    time.sleep(1/240)
    step += 1