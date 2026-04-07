import pybullet as p
import time
import numpy as np
import tkinter as tk

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
    [ 3.1416,   0.0,      2.0943  ],
    [-1.5707,  -1.5707,  -1.5707  ],
    [ 1.5707,  -1.5707,   1.5707  ],
], dtype=float)

EE_XYZ  = np.array([0.03, 0.0, 0.0])
Q_LOWER = np.full(5, -1.5707)
Q_UPPER = np.full(5,  1.5707)
Q_LOWER[3] = -0.1
Q_UPPER[3] =  0.1

CCD_MAX_ITER   = 100
CCD_TOL        = 0.005
AUTO_STEP_GAIN = 0.0005  # m/pixel per frame — tune up/down if too slow/fast


def mat4_from_origin(x, y, z, roll, pitch, yaw):
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
    """Returns joint_pos, joint_axis, ee_pos, ee_x, ee_y, ee_z.
    EE frame: X=camera forward, Y=camera down, Z=camera left."""
    T = np.eye(4)
    joint_pos  = np.zeros((5, 3))
    joint_axis = np.zeros((5, 3))
    for i in range(5):
        T = T @ mat4_from_origin(*JOINT_XYZ[i], *JOINT_RPY[i])
        joint_pos[i]  = T[:3, 3]
        joint_axis[i] = T[:3, 2]
        T = T @ mat4_rotz(q[i])
    ee = T[:3, 3] + T[:3, :3] @ EE_XYZ
    return joint_pos, joint_axis, ee, T[:3, 0].copy(), T[:3, 1].copy(), T[:3, 2].copy()


def c_ccd_solve(target, q_init=None, target_x=None, x_weight=0.4):
    """CCD IK solver.
    target_x   : if provided, steer EE X axis toward this world-space direction
    x_weight   : blend 0=position only, 1=orientation only (default 0.4)
    """
    target = np.asarray(target, dtype=float)
    q = q_init.copy() if q_init is not None else np.zeros(5)
    for _ in range(CCD_MAX_ITER):
        _, _, ee, _, _, _ = c_fk(q)
        if np.linalg.norm(ee - target) < CCD_TOL:
            break
        for i in range(4, -1, -1):
            joint_pos, joint_axis, ee, ee_x, _, _ = c_fk(q)
            ax = joint_axis[i]

            # --- Position correction ---
            r_ee  = ee     - joint_pos[i]
            r_tgt = target - joint_pos[i]
            perp_ee  = r_ee  - np.dot(r_ee,  ax) * ax
            perp_tgt = r_tgt - np.dot(r_tgt, ax) * ax
            len_ee  = np.linalg.norm(perp_ee)
            len_tgt = np.linalg.norm(perp_tgt)
            pos_angle = 0.0
            if len_ee > 1e-6 and len_tgt > 1e-6:
                cross = np.cross(perp_ee, perp_tgt)
                sin_a = np.dot(cross, ax) / (len_ee * len_tgt)
                cos_a = np.dot(perp_ee, perp_tgt) / (len_ee * len_tgt)
                pos_angle = np.arctan2(sin_a, cos_a)

            # --- X-axis orientation hold ---
            ori_angle = 0.0
            if target_x is not None:
                d_cur = np.dot(ee_x,    ax)
                d_des = np.dot(target_x, ax)
                perp_cur = ee_x     - d_cur * ax
                perp_des = target_x - d_des * ax
                lc = np.linalg.norm(perp_cur)
                ld = np.linalg.norm(perp_des)
                if lc > 1e-6 and ld > 1e-6:
                    cross = np.cross(perp_cur, perp_des)
                    sin_a = np.dot(cross, ax) / (lc * ld)
                    cos_a = np.dot(perp_cur, perp_des) / (lc * ld)
                    ori_angle = np.arctan2(sin_a, cos_a)

            angle = (1.0 - x_weight) * pos_angle + x_weight * ori_angle
            q[i] = np.clip(q[i] + angle, Q_LOWER[i], Q_UPPER[i])

    _, _, ee_final, _, _, _ = c_fk(q)
    return q, ee_final, np.linalg.norm(ee_final - target)


# =============================================================================
# PYBULLET SETUP
# =============================================================================
p.connect(p.GUI)
p.setGravity(0, 0, 0)

urdf_path = "/home/joshuachoi16/school_workspace/LumaBot/my_arm.urdf"
robot = p.loadURDF(urdf_path, useFixedBase=True)

p.resetDebugVisualizerCamera(
    cameraDistance=0.8, cameraYaw=45, cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.2]
)

num_joints = p.getNumJoints(robot)
revolute_indices = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    if info[2] == p.JOINT_REVOLUTE:
        revolute_indices.append(i)

ee_index = num_joints - 1

lower_limits = [-1.5707] * 5
upper_limits = [ 1.5707] * 5
joint_ranges = [ 3.1414] * 5
rest_poses   = [0] * 5

target_pos = [0.2, 0.2, 0.2]
p.addUserDebugText("TARGET", target_pos, textColorRGB=[1, 0, 0], textSize=1.5)
p.addUserDebugLine([0, 0, 0], target_pos, [1, 0, 0])

mode_slider = p.addUserDebugParameter("Mode: 0=IK  1=Manual  2=Camera", 0, 2, 0)

joint_sliders = []
for idx in revolute_indices:
    name = p.getJointInfo(robot, idx)[1].decode()
    joint_sliders.append((idx, p.addUserDebugParameter(name, -1.5707, 1.5707, 0.0)))

# =============================================================================
# MOCK CAMERA WINDOW (tkinter) — mode 2
# Draggable ball represents the hand detection centroid.
# Ball position feeds visual servoing: mirrors IK_Set_Angles/MODE_AUTO.
# =============================================================================
CAM_W, CAM_H = 480, 360     # display size (scaled up for easier dragging)
IMG_W, IMG_H = 320, 240     # logical image size (matches real camera)
CX, CY = CAM_W // 2, CAM_H // 2   # centre pixel

BALL_R = 14

root = tk.Tk()
root.title("Mock Camera — drag ball to simulate hand detection")
root.resizable(False, False)

canvas = tk.Canvas(root, width=CAM_W, height=CAM_H, bg="#151515",
                   cursor="crosshair", highlightthickness=0)
canvas.pack()

# Grid lines
for gx in range(0, CAM_W, 48):
    canvas.create_line(gx, 0, gx, CAM_H, fill="#2a2a2a")
for gy in range(0, CAM_H, 36):
    canvas.create_line(0, gy, CAM_W, gy, fill="#2a2a2a")

# Frame-centre crosshair
canvas.create_line(CX - 20, CY, CX + 20, CY, fill="#555", width=1)
canvas.create_line(CX, CY - 20, CX, CY + 20, fill="#555", width=1)
canvas.create_oval(CX-3, CY-3, CX+3, CY+3, fill="#555", outline="")

# Error arrow (updated each frame)
arrow_line = canvas.create_line(CX, CY, CX, CY, fill="#3a7fff", width=2,
                                 arrow=tk.LAST, arrowshape=(10, 12, 4))

# Draggable detection ball
ball_x, ball_y = float(CX), float(CY)
ball = canvas.create_oval(
    CX - BALL_R, CY - BALL_R, CX + BALL_R, CY + BALL_R,
    fill="#00cc44", outline="#ffffff", width=2
)
ball_label = canvas.create_text(CX, CY - BALL_R - 8, text="DETECTION",
                                 fill="#00cc44", font=("Courier", 8))

# HUD text items
hud_err  = canvas.create_text(6, 8,  anchor="nw", fill="#cccccc",
                               font=("Courier", 10), text="px err: --")
hud_ik   = canvas.create_text(6, 24, anchor="nw", fill="#cccccc",
                               font=("Courier", 10), text="IK err: --")
hud_ee   = canvas.create_text(6, 40, anchor="nw", fill="#888888",
                               font=("Courier", 9),  text="EE: --")
hud_hint = canvas.create_text(CAM_W - 4, CAM_H - 6, anchor="se",
                               fill="#444444", font=("Courier", 8),
                               text="click or drag to move detection")


def _move_ball(x, y):
    global ball_x, ball_y
    ball_x = float(np.clip(x, BALL_R, CAM_W - BALL_R))
    ball_y = float(np.clip(y, BALL_R, CAM_H - BALL_R))
    canvas.coords(ball,
                  ball_x - BALL_R, ball_y - BALL_R,
                  ball_x + BALL_R, ball_y + BALL_R)
    canvas.coords(ball_label, ball_x, ball_y - BALL_R - 8)


canvas.bind("<Button-1>",       lambda e: _move_ball(e.x, e.y))
canvas.bind("<B1-Motion>",      lambda e: _move_ball(e.x, e.y))

# Persistent CCD state — warm-started each frame
_cam_q = np.zeros(5)
_cam_x_dir = None   # EE X axis direction locked at camera-mode entry

# =============================================================================
# AXIS VISUALISATION — red=X  green=Y  blue=Z
# Drawn on every link (all balls) and updated every frame.
# EE gets slightly longer lines so it stands out.
# =============================================================================
AXIS_LEN = 0.04   # joint axes
EE_AXIS_LEN = 0.06  # EE axes (longer)
_o = [0, 0, 0]

_link_ax_x, _link_ax_y, _link_ax_z = [], [], []
for i in range(num_joints):
    _link_ax_x.append(p.addUserDebugLine(_o, _o, [1, 0, 0], lineWidth=2))
    _link_ax_y.append(p.addUserDebugLine(_o, _o, [0, 1, 0], lineWidth=2))
    _link_ax_z.append(p.addUserDebugLine(_o, _o, [0, 0, 1], lineWidth=2))

# =============================================================================
# MAIN LOOP
# =============================================================================
step = 0
prev_mode = -1
mode_text_id = ccd_ee_text_id = error_text_id = ccd_text_id = None

print("\n" + "="*60)
print("Mode 0 = IK comparison  |  1 = Manual sliders  |  2 = Camera")
print("="*60 + "\n")

while True:
    mode = round(p.readUserDebugParameter(mode_slider))

    # Reset camera-mode warm-start when entering mode 2
    if mode == 2 and prev_mode != 2:
        _cam_q = np.zeros(5)   # all joints at 90° servo (= 0 rad IK)
    prev_mode = mode

    # ------------------------------------------------------------------
    if mode == 1:
        angles_rad = []
        for joint_idx, slider in joint_sliders:
            val = p.readUserDebugParameter(slider)
            p.resetJointState(robot, joint_idx, val)
            angles_rad.append(val)
        ee_state = p.getLinkState(robot, ee_index)
        ee_pos   = np.array(ee_state[4])
        msg = f"Manual EE: {np.round(ee_pos, 3)}"
        if mode_text_id is None:
            mode_text_id   = p.addUserDebugText("[ MANUAL MODE ]", [0.2, 0.3, 0.40], textColorRGB=[1, 1, 0])
            ccd_ee_text_id = p.addUserDebugText(msg, [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0])
        else:
            mode_text_id   = p.addUserDebugText("[ MANUAL MODE ]", [0.2, 0.3, 0.40], textColorRGB=[1, 1, 0],   replaceItemUniqueId=mode_text_id)
            ccd_ee_text_id = p.addUserDebugText(msg,               [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0], replaceItemUniqueId=ccd_ee_text_id)
        if step % 100 == 0:
            print(f"[Manual] EE: {np.round(ee_pos, 4)}")
            print(f"  Angles (deg):       {np.round(np.degrees(angles_rad), 2)}")
            print(f"  Servo cmd (deg+90): {np.round(np.degrees(angles_rad) + 90, 2)}")
            print()

    # ------------------------------------------------------------------
    elif mode == 2:
        # Map display coords → logical image coords (IMG_W x IMG_H)
        img_x = ball_x / CAM_W * IMG_W
        img_y = ball_y / CAM_H * IMG_H

        # Pixel error from frame centre — mirrors angle_compute.c IK_Set_Angles
        err_x = img_x - 160.0   # + = detection right of centre
        err_y = img_y - 120.0   # + = detection below centre

        # Incremental EE nudge along EE image-plane axes (MODE_AUTO logic).
        # ee_y points downward in this URDF so + err_y moves EE down (correct).
        # Step is clamped to 10 mm/frame to prevent IK branch flips.
        # EE frame: X=fwd, Y=down, Z=left
        # camera right = -ee_z,  camera down = +ee_y
        _, _, ee_pos, ee_x, ee_y, ee_z = c_fk(_cam_q)

        nudge = AUTO_STEP_GAIN * (-err_x * ee_z + err_y * ee_y)
        nudge_len = np.linalg.norm(nudge)
        if nudge_len > 0.01:
            nudge = nudge * 0.01 / nudge_len
        target = ee_pos + nudge
        _cam_q, cam_ee, cam_err = c_ccd_solve(target, q_init=_cam_q)
        delta_ee = cam_ee - ee_pos

        for idx, joint_idx in enumerate(revolute_indices):
            p.resetJointState(robot, joint_idx, _cam_q[idx])

        # Update tkinter HUD
        err_px = (err_x**2 + err_y**2) ** 0.5
        canvas.itemconfig(hud_err, text=f"px err: {err_px:.1f}")
        canvas.itemconfig(hud_ik,  text=f"IK err: {cam_err*1000:.1f} mm")
        canvas.itemconfig(hud_ee,  text=f"EE: {np.round(cam_ee, 3)}")
        canvas.coords(arrow_line, CX, CY, int(ball_x), int(ball_y))

        cam_msg = f"Camera EE: {np.round(cam_ee, 3)}"
        if mode_text_id is None:
            mode_text_id   = p.addUserDebugText("[ CAMERA MODE ]", [0.2, 0.3, 0.40], textColorRGB=[0, 1, 0.5])
            ccd_ee_text_id = p.addUserDebugText(cam_msg, [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0])
        else:
            mode_text_id   = p.addUserDebugText("[ CAMERA MODE ]", [0.2, 0.3, 0.40], textColorRGB=[0, 1, 0.5], replaceItemUniqueId=mode_text_id)
            ccd_ee_text_id = p.addUserDebugText(cam_msg,            [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0], replaceItemUniqueId=ccd_ee_text_id)

        # Print every frame so we can see exactly what is happening
        print(f"[step {step:5d}] err=({err_x:+6.1f},{err_y:+6.1f})px")
        print(f"  ee_pos  : {np.round(ee_pos, 4)}")
        print(f"  ee_x    : {np.round(ee_x,   4)}   (camera fwd   in world)")
        print(f"  ee_y    : {np.round(ee_y,   4)}   (camera down  in world)")
        print(f"  ee_z    : {np.round(ee_z,   4)}   (camera left  in world)")
        print(f"  nudge   : {np.round(nudge,  6)}")
        print(f"  new EE  : {np.round(cam_ee, 4)}   delta={np.round(delta_ee, 4)}")
        print()

    # ------------------------------------------------------------------
    else:
        pb_ik = p.calculateInverseKinematics(
            robot, ee_index, target_pos,
            lowerLimits=lower_limits, upperLimits=upper_limits,
            jointRanges=joint_ranges, restPoses=rest_poses,
        )
        for idx, joint_idx in enumerate(revolute_indices):
            p.resetJointState(robot, joint_idx, pb_ik[idx])
        ee_state = p.getLinkState(robot, ee_index)
        pb_ee    = np.array(ee_state[4])
        pb_error = np.linalg.norm(pb_ee - np.array(target_pos))
        ccd_q, ccd_ee, ccd_error = c_ccd_solve(target_pos, q_init=np.zeros(5))

        if step % 100 == 0:
            pb_angles = np.array(pb_ik[:5])
            _, _, fk_of_pb,  _, _, _ = c_fk(pb_angles)
            _, _, fk_of_ccd, _, _, _ = c_fk(ccd_q)
            fk_pb_err  = np.linalg.norm(fk_of_pb  - np.array(target_pos))
            fk_ccd_err = np.linalg.norm(fk_of_ccd - np.array(target_pos))
            print(f"--- Step {step} ---")
            print(f"[PyBullet IK]")
            print(f"  Angles    (deg):    {np.round(np.degrees(pb_angles), 2)}")
            print(f"  Servo cmd (deg+90): {np.round(np.degrees(pb_angles) + 90, 2)}")
            print(f"  PyBullet FK → EE:  {np.round(pb_ee, 4)}  err={pb_error:.4f}m")
            print(f"  Manual   FK → EE:  {np.round(fk_of_pb, 4)}  err={fk_pb_err:.4f}m")
            if fk_pb_err > 0.01:
                print(f"  *** GEOMETRY MISMATCH ***")
            print(f"[Manual CCD IK]")
            print(f"  Angles    (deg):    {np.round(np.degrees(ccd_q), 2)}")
            print(f"  Servo cmd (deg+90): {np.round(np.degrees(ccd_q) + 90, 2)}")
            print(f"  Manual   FK → EE:  {np.round(fk_of_ccd, 4)}  err={fk_ccd_err:.4f}m")
            if fk_ccd_err > CCD_TOL:
                print(f"  *** CCD did not converge ***")
            print(f"  Angle diff vs PyBullet: {np.round(np.degrees(ccd_q - pb_angles), 2)} deg")
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
            error_text_id  = (p.addUserDebugText(pb_msg,  [0.2, 0.3, 0.35], textColorRGB=[0, 1, 0],   replaceItemUniqueId=error_text_id)  if error_text_id  else p.addUserDebugText(pb_msg,  [0.2, 0.3, 0.35], textColorRGB=[0, 1, 0]))
            ccd_text_id    = (p.addUserDebugText(ccd_msg, [0.2, 0.3, 0.30], textColorRGB=[0, 0.5, 1], replaceItemUniqueId=ccd_text_id)    if ccd_text_id    else p.addUserDebugText(ccd_msg, [0.2, 0.3, 0.30], textColorRGB=[0, 0.5, 1]))
            ccd_ee_text_id = p.addUserDebugText(ccd_ee_msg, [0.2, 0.3, 0.25], textColorRGB=[1, 0.5, 0], replaceItemUniqueId=ccd_ee_text_id)

    # ------------------------------------------------------------------
    # Update axis lines for every link (red=X  green=Y  blue=Z)
    for i in range(num_joints):
        ls  = p.getLinkState(robot, i, computeForwardKinematics=True)
        lp  = np.array(ls[4])
        lR  = np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)
        alen = EE_AXIS_LEN if i == ee_index else AXIS_LEN
        lw   = 3 if i == ee_index else 2
        _link_ax_x[i] = p.addUserDebugLine(lp, lp + alen * lR[:, 0], [1, 0, 0], lineWidth=lw, replaceItemUniqueId=_link_ax_x[i])
        _link_ax_y[i] = p.addUserDebugLine(lp, lp + alen * lR[:, 1], [0, 1, 0], lineWidth=lw, replaceItemUniqueId=_link_ax_y[i])
        _link_ax_z[i] = p.addUserDebugLine(lp, lp + alen * lR[:, 2], [0, 0, 1], lineWidth=lw, replaceItemUniqueId=_link_ax_z[i])

    # Pump tkinter events (non-blocking)
    try:
        root.update()
    except tk.TclError:
        break   # window was closed

    p.stepSimulation()
    time.sleep(1/60)
    step += 1
