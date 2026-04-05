import pybullet as p
import time
import numpy as np

# =============================================================================
# MANUAL CCD IK — mirrors Core/Src/angle_compute.c
# Keep in sync with URDF and angle_compute.c geometry tables.
# =============================================================================

JOINT_XYZ = np.array([
    [ 0.000,  0.000,  0.000],   # joint1
    [ 0.000,  0.000,  0.044],   # joint2
    [-0.140,  0.000,  0.000],   # joint3
    [ 0.105,  0.005,  0.014],   # joint4
    [ 0.016, -0.006,  0.029],   # joint5
], dtype=float)

JOINT_RPY = np.array([
    [ 0.0,      0.0,      0.0     ],
    [ 0.0,      1.5707,   0.0     ],
    [ 3.1416,   0.0,      2.0943  ],   # updated to match URDF
    [-1.5707,  -1.5707,  -1.5707  ],
    [ 1.5707,  -1.5707,   1.5707  ],
], dtype=float)

EE_XYZ  = np.array([0.05, 0.0, 0.0])
Q_LOWER = np.full(5, -1.5707)
Q_UPPER = np.full(5,  1.5707)
Q_LOWER[3] = 0
Q_UPPER[3] = 0

CCD_MAX_ITER  = 100
CCD_TOL       = 0.005


def mat4_from_origin(x, y, z, roll, pitch, yaw):
    """T = Trans(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll) — URDF RPY convention."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    T = np.eye(4)
    T[0,0]=cy*cp;   T[0,1]=cy*sp*sr-sy*cr;  T[0,2]=cy*sp*cr+sy*sr;  T[0,3]=x
    T[1,0]=sy*cp;   T[1,1]=sy*sp*sr+cy*cr;  T[1,2]=sy*sp*cr-cy*sr;  T[1,3]=y
    T[2,0]=-sp;     T[2,1]=cp*sr;            T[2,2]=cp*cr;            T[2,3]=z
    return T


def mat4_rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)


def c_fk(q):
    """FK — mirrors IK_FK() in angle_compute.c.
    Returns: joint_pos (5,3), joint_axis (5,3), ee_pos (3,)"""
    T = np.eye(4)
    joint_pos  = np.zeros((5, 3))
    joint_axis = np.zeros((5, 3))
    for i in range(5):
        T = T @ mat4_from_origin(*JOINT_XYZ[i], *JOINT_RPY[i])
        joint_pos[i]  = T[:3, 3]
        joint_axis[i] = T[:3, 2]   # Z column = joint axis
        T = T @ mat4_rotz(q[i])
    ee = T[:3, 3] + T[:3, :3] @ EE_XYZ
    return joint_pos, joint_axis, ee


def c_ccd_solve(target, q_init=None):
    """CCD solver — mirrors IK_CCD_Solve() in angle_compute.c.
    Returns (q, ee_final, error)."""
    target = np.asarray(target, dtype=float)
    q = q_init.copy() if q_init is not None else np.zeros(5)

    for _ in range(CCD_MAX_ITER):
        _, _, ee = c_fk(q)
        if np.linalg.norm(ee - target) < CCD_TOL:
            break
        for i in range(4, -1, -1):
            joint_pos, joint_axis, ee = c_fk(q)
            ax = joint_axis[i]

            r_ee  = ee             - joint_pos[i]
            r_tgt = target         - joint_pos[i]

            perp_ee  = r_ee  - np.dot(r_ee,  ax) * ax
            perp_tgt = r_tgt - np.dot(r_tgt, ax) * ax

            len_ee  = np.linalg.norm(perp_ee)
            len_tgt = np.linalg.norm(perp_tgt)
            if len_ee < 1e-6 or len_tgt < 1e-6:
                continue

            cross = np.cross(perp_ee, perp_tgt)
            sin_a = np.dot(cross, ax) / (len_ee * len_tgt)
            cos_a = np.dot(perp_ee, perp_tgt) / (len_ee * len_tgt)
            angle = np.arctan2(sin_a, cos_a)

            q[i] = np.clip(q[i] + angle, Q_LOWER[i], Q_UPPER[i])

    _, _, ee_final = c_fk(q)
    return q, ee_final, np.linalg.norm(ee_final - target)


# =============================================================================
# PYBULLET SETUP
# =============================================================================
p.connect(p.GUI)
p.setGravity(0, 0, 0)

urdf_path = "/home/joshuachoi16/school_workspace/LumaBot/my_arm.urdf"
robot = p.loadURDF(urdf_path, useFixedBase=True)

p.resetDebugVisualizerCamera(
    cameraDistance=0.8,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.2]
)

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

lower_limits = [-1.5707, -1.5707, -1.5707, -1.5707, -1.5707]
upper_limits = [ 1.5707,  1.5707,  1.5707,  1.5707,  1.5707]
joint_ranges = [ 3.1414,  3.1414,  3.1414,  3.1414,  3.1414]
rest_poses   = [ 0, 0, 0, 0, 0]

target_pos = [0.2, 0.2, 0.2]
target_marker = p.addUserDebugText("TARGET", target_pos, textColorRGB=[1, 0, 0], textSize=1.5)
p.addUserDebugLine([0, 0, 0], target_pos, [1, 0, 0])

# Mode toggle slider: 0 = IK mode, 1 = Manual slider mode
mode_slider = p.addUserDebugParameter("Mode: 0=IK  1=Manual", 0, 1, 0)

# Joint sliders (always created, only active in manual mode)
joint_sliders = []
for idx in revolute_indices:
    name = p.getJointInfo(robot, idx)[1].decode()
    slider = p.addUserDebugParameter(name, -1.5707, 1.5707, 0.0)
    joint_sliders.append((idx, slider))

# =============================================================================
# MAIN LOOP
# =============================================================================
step = 0
error_text_id  = None
ccd_text_id    = None
ccd_ee_text_id = None
mode_text_id   = None

print("\n" + "="*70)
print("Mode slider: 0 = IK comparison   1 = Manual joint sliders")
print("="*70 + "\n")

while True:
    mode = round(p.readUserDebugParameter(mode_slider))

    if mode == 1:
        # ------------------------------------------------------------------
        # MANUAL MODE: sliders drive each joint directly
        # ------------------------------------------------------------------
        angles_rad = []
        for joint_idx, slider in joint_sliders:
            val = p.readUserDebugParameter(slider)
            p.resetJointState(robot, joint_idx, val)
            angles_rad.append(val)

        ee_state = p.getLinkState(robot, ee_index)
        ee_pos   = np.array(ee_state[4])

        msg = f"Manual  EE: {np.round(ee_pos, 3)}"
        if mode_text_id is None:
            mode_text_id = p.addUserDebugText("[ MANUAL MODE ]", [0.2, 0.3, 0.40], textColorRGB=[1, 1, 0])
            ccd_ee_text_id = p.addUserDebugText(msg, [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0])
        else:
            mode_text_id   = p.addUserDebugText("[ MANUAL MODE ]", [0.2, 0.3, 0.40], textColorRGB=[1, 1, 0],   replaceItemUniqueId=mode_text_id)
            ccd_ee_text_id = p.addUserDebugText(msg,               [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0], replaceItemUniqueId=ccd_ee_text_id)

        if step % 100 == 0:
            print(f"[Manual] EE: {np.round(ee_pos, 4)}")
            print(f"  Angles (deg):       {np.round(np.degrees(angles_rad), 2)}")
            print(f"  Servo cmd (deg+90): {np.round(np.degrees(angles_rad) + 90, 2)}")
            print()

    else:
        # ------------------------------------------------------------------
        # IK MODE: PyBullet IK + Manual CCD comparison
        # ------------------------------------------------------------------
        pb_ik = p.calculateInverseKinematics(
            robot, ee_index, target_pos,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
        )
        for idx, joint_idx in enumerate(revolute_indices):
            p.resetJointState(robot, joint_idx, pb_ik[idx])

        ee_state = p.getLinkState(robot, ee_index)
        pb_ee    = np.array(ee_state[4])
        pb_error = np.linalg.norm(pb_ee - np.array(target_pos))

        ccd_q, ccd_ee, ccd_error = c_ccd_solve(target_pos, q_init=np.zeros(5))

        if step % 100 == 0:
            pb_angles  = np.array(pb_ik[:5])
            _, _, fk_of_pb  = c_fk(pb_angles)
            _, _, fk_of_ccd = c_fk(ccd_q)
            fk_pb_err  = np.linalg.norm(fk_of_pb  - np.array(target_pos))
            fk_ccd_err = np.linalg.norm(fk_of_ccd - np.array(target_pos))

            print(f"--- Step {step} ---")
            print(f"Target:                    {np.round(target_pos, 4)}")
            print()
            print(f"[PyBullet IK]")
            print(f"  Angles     (deg):        {np.round(np.degrees(pb_angles), 2)}")
            print(f"  Servo cmd  (deg+90):     {np.round(np.degrees(pb_angles) + 90, 2)}")
            print(f"  PyBullet FK → EE:        {np.round(pb_ee, 4)}  err={pb_error:.4f}m")
            print(f"  Manual  FK → EE:         {np.round(fk_of_pb, 4)}  err={fk_pb_err:.4f}m")
            if fk_pb_err > 0.01:
                print(f"  *** GEOMETRY MISMATCH ***")
            print()
            print(f"[Manual CCD IK]")
            print(f"  Angles     (deg):        {np.round(np.degrees(ccd_q), 2)}")
            print(f"  Servo cmd  (deg+90):     {np.round(np.degrees(ccd_q) + 90, 2)}")
            print(f"  Manual  FK → EE:         {np.round(fk_of_ccd, 4)}  err={fk_ccd_err:.4f}m")
            if fk_ccd_err > CCD_TOL:
                print(f"  *** CCD did not converge ***")
            print(f"  Angle diff vs PyBullet:  {np.round(np.degrees(ccd_q - pb_angles), 2)} deg")
            print()

        pb_msg     = f"PyBullet IK  err={pb_error:.4f}m"
        ccd_msg    = f"C CCD IK     err={ccd_error:.4f}m"
        ccd_ee_msg = f"CCD EE: {np.round(ccd_ee, 3)}"

        if mode_text_id is None:
            mode_text_id   = p.addUserDebugText("[ IK MODE ]",  [0.2, 0.3, 0.40], textColorRGB=[0, 1, 1])
            error_text_id  = p.addUserDebugText(pb_msg,         [0.2, 0.3, 0.35], textColorRGB=[0, 1, 0])
            ccd_text_id    = p.addUserDebugText(ccd_msg,        [0.2, 0.3, 0.30], textColorRGB=[0, 0.5, 1])
            ccd_ee_text_id = p.addUserDebugText(ccd_ee_msg,     [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0])
        else:
            mode_text_id   = p.addUserDebugText("[ IK MODE ]",  [0.2, 0.3, 0.40], textColorRGB=[0, 1, 1],   replaceItemUniqueId=mode_text_id)
            error_text_id  = p.addUserDebugText(pb_msg,         [0.2, 0.3, 0.35], textColorRGB=[0, 1, 0],   replaceItemUniqueId=error_text_id) if error_text_id else p.addUserDebugText(pb_msg, [0.2, 0.3, 0.35], textColorRGB=[0, 1, 0])
            ccd_text_id    = p.addUserDebugText(ccd_msg,        [0.2, 0.3, 0.30], textColorRGB=[0, 0.5, 1], replaceItemUniqueId=ccd_text_id)   if ccd_text_id   else p.addUserDebugText(ccd_msg, [0.2, 0.3, 0.30], textColorRGB=[0, 0.5, 1])
            ccd_ee_text_id = p.addUserDebugText(ccd_ee_msg,     [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0], replaceItemUniqueId=ccd_ee_text_id)

    p.stepSimulation()
    time.sleep(1/240)
    step += 1
